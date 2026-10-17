# SPDX-License-Identifier: GPL-2.0-only

"""Prepare local products and reconcile optional player-path apply results.

Example (repository root):
  TMPDIR=/private/var/folders/7h/xq0frx8x03q49hg7ykfs8zxc0000gr/T/ \
    ~/.local/bin/python3.12 -m tools.map_import

Apply through the local headless wrapper with the already deployed extension:
  maps/processed/run_apply.sh
  # Optional: up to three full days at the maximum supported speed; both bounds apply.
  maps/processed/run_apply.sh --grow-days 3 --grow-max-seconds 180 --grow-max-frames 120000
  ~/.local/bin/python3.12 -m tools.map_import --outcome maps/processed/niirala/apply_results.json

Relative runner paths resolve from the Godot project directory. The preparation
CLI never starts Godot, invokes run.sh or performs a network request. Outputs are
restricted to ignored paths under maps/processed to protect local ODbL data.

The apply script uses public player validation, commit, undo, vegetation, outside
connection, speed and save APIs. No lower-level graph writes or Rust hooks are
needed. A disconnected car edit is undone and deferred; every retry is recorded.
The crop's continuing road gets one outside connection, using the existing
connection tool's normal extension. The final save is confirmed paused.
"""

import argparse
import hashlib
import json
import math
import re
import sqlite3
from pathlib import Path
import subprocess

from .mapping import POLICY, map_role, table
from .conflate import collapse_roundabouts, conflate, prepare_streets, prune_stubs
from .water import WaterMask
from .normalise import normalise
from .osm import OsmAdapter
from .projection import tm35fin
from .report import build_report, summary
from .terrain import load_transform
from .zoning import frontage_intents

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def safe_output(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT / 'maps/processed'):
        raise ValueError('Outputs must stay under maps/processed')
    for name in ('neutral.json', 'plan.json', 'report.json', 'summary.txt', 'apply_results.json', 'city.sqlite'):
        check = subprocess.run(['git', 'check-ignore', '-q', str(path/name)], cwd=ROOT, check=False)
        if check.returncode != 0:
            raise ValueError(f'Output is not git-ignored: {path/name}')
    return path


def validate_outcome(outcome, plan, plan_hash):
    if outcome.get('schema_version') != 1 or outcome.get('plan_sha256') != plan_hash:
        raise ValueError('Apply results do not match this exact plan')
    for field, products in (('roads', plan['strokes']), ('zoning', plan['zoning'])):
        known = {p['id'] for p in products}
        ids = [r['id'] for r in outcome[field]]
        if len(set(ids)) != len(ids) or not set(ids) <= known:
            raise ValueError(f'Duplicate/unknown apply IDs in {field}')
        if outcome.get('complete') and set(ids) != known:
            raise ValueError(f'Completed apply omits {field}')
        if any(r['status'] not in ('accepted', 'partial', 'refused', 'skipped') or not isinstance(r.get('reason'), str)
               for r in outcome[field]):
            raise ValueError(f'Invalid apply status in {field}')
    if any(not math.isfinite(p['area_m2']) or p['area_m2'] < 0 for p in outcome['parcels']):
        raise ValueError('Invalid committed parcel areas')


def prepare(args):
    output = safe_output(args.output)
    transform = load_transform(args.dem_dir, args.world)
    center = transform.world(*tm35fin(args.lat, args.lon))
    half = args.size_m/2
    box = (center[0]-half, center[1]-half, center[0]+half, center[1]+half)
    if any(not math.isfinite(v) for v in (*box, args.simplify_m)) or args.size_m <= 0 or args.simplify_m < 0:
        raise ValueError('Target size must be positive and simplification nonnegative')
    if box[0] < -(transform.max_e-transform.min_e)/2 or box[2] > (transform.max_e-transform.min_e)/2 or box[1] < -(transform.max_n-transform.min_n)/2 or box[3] > (transform.max_n-transform.min_n)/2:
        raise ValueError('Target box lies outside the authored world')
    features = normalise(OsmAdapter().read(args.source), transform)
    decisions = {(f['id'], role): map_role(f, role) for f in features for role in f['roles']}
    roads = [(f, decisions[(f['id'], 'road')]) for f in features if 'road' in f['roles']
             and decisions[(f['id'], 'road')]['action'] in ('road', 'footpath')]
    # Keep neutral source geometry immutable; conflation owns its working copy.
    from copy import deepcopy
    roads = deepcopy(roads)
    roundabouts = collapse_roundabouts(features, roads, decisions)
    roads, conflation = conflate(roads, decisions)
    conflation['roundabouts'] = roundabouts
    strokes, preparation, endpoint_snaps = prepare_streets(roads, box, args.simplify_m)
    strokes = prune_stubs(strokes, preparation['discarded'])
    water = WaterMask.load(args.world)
    strokes, water_audit = water.clip(strokes)
    strokes = prune_stubs(strokes, preparation['discarded'])
    preparation.update(conflation=conflation, endpoint_snaps=endpoint_snaps, water=water_audit, water_fills=water.fills)
    zones = frontage_intents(features, decisions, strokes, box, transform.zone_cell_m)
    plan = {'schema_version': 1, 'source_sha256': digest(args.source), 'world_sha256': digest(args.world),
            'source_license': 'OpenStreetMap contributors, ODbL; local derived data',
            'world_transform': transform.record(), 'target_box_xz': box,
            'center_wgs84': [args.lat, args.lon], 'center_world_xz': center,
            'parameters': {'size_m': args.size_m, 'simplify_m': args.simplify_m, **POLICY},
            'capabilities': table(), 'strokes': strokes, 'zoning': zones}
    # Validate an outcome before replacing any previous products.
    plan_bytes = (json.dumps(plan, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + '\n').encode()
    outcome = json.loads(args.outcome.read_text()) if args.outcome else None
    if outcome is not None:
        validate_outcome(outcome, plan, hashlib.sha256(plan_bytes).hexdigest())
        log = args.outcome.with_name('apply.log')
        counts = re.findall(r'CCH rebuilt:.*?→ (\d+) (?:DISCONNECTED components|component)', log.read_text()) if log.exists() else []
        outcome['car_components'] = int(counts[-1]) if counts else None
        save_path = output/'city.sqlite'
        if outcome.get('saved') and save_path.exists():
            with sqlite3.connect(save_path.resolve().as_uri()+'?mode=ro', uri=True) as conn:
                outcome['population'] = conn.execute('SELECT coalesce(sum(member_count),0) FROM households').fetchone()[0]
                outcome['buildings'] = conn.execute('SELECT count(*) FROM buildings').fetchone()[0]
                outcome['saved_speed'] = conn.execute('SELECT speed_multiplier FROM time_state').fetchone()[0]
    report = build_report(features, decisions, plan, preparation, outcome)
    output.mkdir(parents=True, exist_ok=True)
    write_json(output/'neutral.json', {'schema_version': 1, 'coordinate_system': 'world_xz_metres',
                                      'world_transform': transform.record(), 'features': features})
    (output/'plan.json').write_bytes(plan_bytes)
    write_json(output/'report.json', report)
    (output/'summary.txt').write_text(summary(report))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', type=Path, default=ROOT/'maps/raw/osm/niirala_osm_api.json')
    parser.add_argument('--world', type=Path, default=ROOT/'godot/bootstrap/worlds/kuopio_324km2_10m.sqlite')
    parser.add_argument('--dem-dir', type=Path, default=ROOT/'maps/raw/Kuopio/324km2')
    parser.add_argument('--output', type=Path, default=ROOT/'maps/processed/niirala')
    parser.add_argument('--lat', type=float, default=62.889552)
    parser.add_argument('--lon', type=float, default=27.652509)
    parser.add_argument('--size-m', type=float, default=1600)
    parser.add_argument('--simplify-m', type=float, default=1.0)
    parser.add_argument('--outcome', type=Path)
    args = parser.parse_args()
    try:
        report = prepare(args)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f'map import: {exc}\n')
    print(summary(report), end='')


if __name__ == '__main__':
    main()
