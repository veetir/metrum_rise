# SPDX-License-Identifier: GPL-2.0-only

"""Prepare offline roads for Niirala, Savilahti and Neulamäki.

Python 3.12 standard library only. Default output: maps/processed/kuopio_sw.
--area niirala selects the old 1600 m box; --bbox WEST SOUTH EAST NORTH selects
an explicit geographic box. --include-park-paths enables standalone paths;
sidewalks remain excluded. This iteration emits no zoning or growth products.

Apply using the deployed extension directly, with TMPDIR set as in CLAUDE.md:
  godot --headless --path godot --script res://scripts/tools/map_import_apply.gd -- \
    --plan ../maps/processed/kuopio_sw/plan.json --output ../maps/processed/kuopio_sw/run

All geometry is sent through player validation and acknowledged commit APIs.
Detached streets remain in the save for local repair rather than being undone.
World editor fill operations change the produced save only. A derived world
copy gives complete edited fills bathymetry before roads are built.
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
from .curves import prepare_curves
from .survey import apply_road_surveys, inspect, university_access
from .roundabouts import detect_roundabouts
from .water_beds import prepare_initial
from .normalise import normalise
from .osm import OsmAdapter
from .projection import tm35fin
from .report import build_report, summary
from .terrain import load_transform

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
    for name in ('neutral.json', 'plan.json', 'report.json', 'summary.txt', 'apply_results.json', 'city.sqlite', 'water-world.sqlite'):
        check = subprocess.run(['/bin/bash', '-c',
                                'exec git -c filter.lfs.clean= -c filter.lfs.smudge= -c filter.lfs.process= -c filter.lfs.required=false "$@"',
                                'map-import-git', 'check-ignore', '-q', str(path/name)], cwd=ROOT, check=False)
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
    if args.bbox:
        west, south, east, north = args.bbox
        if not west < east or not south < north:
            raise ValueError('Bounding box requires WEST < EAST and SOUTH < NORTH')
        corners = [transform.world(*tm35fin(lat, lon)) for lat in (south, north) for lon in (west, east)]
        box = (min(p[0] for p in corners), min(p[1] for p in corners), max(p[0] for p in corners), max(p[1] for p in corners))
    elif args.area == 'kuopio-sw':
        corners = [transform.world(*tm35fin(lat, lon)) for lat in (62.871, 62.901) for lon in (27.588, 27.674)]
        box = (min(p[0] for p in corners), min(p[1] for p in corners), max(p[0] for p in corners), max(p[1] for p in corners))
    if any(not math.isfinite(v) for v in (*box, args.simplify_m)) or args.size_m <= 0 or args.simplify_m < 0:
        raise ValueError('Target size must be positive and simplification nonnegative')
    if box[0] < -(transform.max_e-transform.min_e)/2 or box[2] > (transform.max_e-transform.min_e)/2 or box[1] < -(transform.max_n-transform.min_n)/2 or box[3] > (transform.max_n-transform.min_n)/2:
        raise ValueError('Target box lies outside the authored world')
    features = normalise(OsmAdapter().read(args.source), transform)
    # Keep neutral source geometry immutable; conflation owns its working copy.
    from copy import deepcopy
    working = deepcopy(features)
    ring_detection = detect_roundabouts(working)
    decisions = {(f['id'], role): map_role(f, role, args.include_park_paths) for f in working for role in f['roles']}
    roads = [(f, decisions[(f['id'], 'road')]) for f in working if 'road' in f['roles']
             and decisions[(f['id'], 'road')]['action'] in ('road', 'footpath')]
    road_surveys = apply_road_surveys(roads, args.include_park_paths)
    roundabouts = collapse_roundabouts(working, roads, decisions)
    roads, conflation = conflate(roads, decisions)
    from .medians import simplify_prisma
    prisma = tuple(round(v, 3) for v in transform.world(*tm35fin(62.887133, 27.629350)))
    roads, conflation['prisma'] = simplify_prisma(roads, prisma, decisions)
    roads, access_survey = university_access(roads, decisions)
    conflation['university_access'] = access_survey
    conflation['roundabouts'] = roundabouts
    conflation['roundabout_detection'] = ring_detection
    conflation['road_surveys'] = road_surveys
    strokes, preparation, endpoint_snaps = prepare_streets(roads, box, args.simplify_m)
    strokes = prune_stubs(strokes, preparation['discarded'])
    strokes = prepare_curves(strokes, features, [{'name': 'Savilahdentie', 'centre_xz': prisma, 'radius_m': 75}],
                             [r['centre_xz'] for r in roundabouts if r['status'] == 'planned'])
    preparation.update(conflation=conflation, endpoint_snaps=endpoint_snaps, water=[], water_fills=[])
    zones = []
    survey = inspect(features, args.world, box)
    output.mkdir(parents=True, exist_ok=True)
    bed_world = output/'water-world.sqlite'
    beds = prepare_initial(args.world, survey['water_edits'], bed_world)
    beds['world_sha256'] = digest(bed_world)
    plan = {'schema_version': 1, 'source_sha256': digest(args.source), 'world_sha256': digest(args.world),
            'source_license': 'OpenStreetMap contributors, ODbL; local derived data',
            'world_transform': transform.record(), 'target_box_xz': box,
            'center_wgs84': [args.lat, args.lon], 'center_world_xz': center,
            'parameters': {'size_m': args.size_m, 'simplify_m': args.simplify_m, 'include_park_paths': args.include_park_paths, **POLICY},
            'water_edits': survey['water_edits'], 'water_beds': beds, 'capabilities': table(), 'strokes': strokes, 'zoning': zones,
            'survey_crossings': [r for r in access_survey if r.get('marked')],
            'roundabouts': [r for r in roundabouts if r['status'] == 'planned' and
                            box[0] <= r['centre_xz'][0] <= box[2] and box[1] <= r['centre_xz'][1] <= box[3]]}
    # Validate an outcome before replacing any previous products.
    plan_bytes = (json.dumps(plan, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False) + '\n').encode()
    outcome = json.loads(args.outcome.read_text()) if args.outcome else None
    if outcome is not None:
        validate_outcome(outcome, plan, hashlib.sha256(plan_bytes).hexdigest())
        log = args.outcome.with_name('apply.log')
        counts = re.findall(r'CCH rebuilt:.*?→ (\d+) (?:DISCONNECTED components|component)', log.read_text()) if log.exists() else []
        outcome['car_components'] = int(counts[-1]) if counts else outcome.get('car_components')
        save_path = Path(outcome.get('save_path') or output/'city.sqlite').resolve()
        if not save_path.is_relative_to(ROOT/'maps/processed'):
            raise ValueError('Outcome save must stay under maps/processed')
        if outcome.get('saved') and save_path.exists():
            with sqlite3.connect(save_path.resolve().as_uri()+'?mode=ro', uri=True) as conn:
                outcome['saved_speed'] = conn.execute('SELECT speed_multiplier FROM time_state').fetchone()[0]
    report = build_report(features, decisions, plan, preparation, outcome)
    output.mkdir(parents=True, exist_ok=True)
    write_json(output/'neutral.json', {'schema_version': 1, 'coordinate_system': 'world_xz_metres',
                                      'world_transform': transform.record(), 'features': features})
    (output/'plan.json').write_bytes(plan_bytes)
    write_json(output/'report.json', report)
    write_json(output/'survey.json', survey)
    (output/'summary.txt').write_text(summary(report))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--area', choices=('kuopio-sw', 'niirala'), default='kuopio-sw')
    parser.add_argument('--bbox', nargs=4, type=float, metavar=('WEST', 'SOUTH', 'EAST', 'NORTH'))
    parser.add_argument('--include-park-paths', action='store_true')
    parser.add_argument('--source', type=Path, default=ROOT/'maps/raw/osm/kuopio_sw_osm_api.json')
    parser.add_argument('--world', type=Path, default=ROOT/'godot/bootstrap/worlds/kuopio_324km2_10m.sqlite')
    parser.add_argument('--dem-dir', type=Path, default=ROOT/'maps/raw/Kuopio/324km2')
    parser.add_argument('--output', type=Path, default=ROOT/'maps/processed/kuopio_sw')
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
