# SPDX-License-Identifier: GPL-2.0-only

"""Offline OSM approach rules mapped onto saved junction topology.

Reuses SegmentIndex for local attribution. Each tagged approach inspects local
junctions in a 32 m grid, then incident arms only. No runtime simulation logic.
"""

from collections import defaultdict
import math

from .connectivity import attribute, read_save, source_index, topology
from .geometry import distance, length
from .mapping import POLICY


def turn_lanes(properties):
    """Return directional, median-to-kerb lane tokens in OSM travel order."""
    result = []
    oneway = properties.get('oneway', 0)
    for direction, key, count_key in ((1, 'turn_lanes_forward', 'lanes_forward'), (-1, 'turn_lanes_backward', 'lanes_backward')):
        text = properties.get(key, '')
        if not text and direction == (oneway or 1):
            text = properties.get('turn_lanes', '')
        if not text:
            continue
        lanes = [tuple(token for token in lane.split(';') if token) for lane in text.split('|')]
        count = properties.get(count_key) or (properties.get('lanes') if oneway else None)
        if count is not None and count != len(lanes):
            result.append({'direction': direction, 'reason': 'tag_lane_count_mismatch', 'lanes': lanes})
            continue
        result.append({'direction': direction, 'lanes': lanes, 'reason': ''})
    return result


def fit_compact_left_pocket(action, edge):
    """Fit source storage between exact existing clips; a left pocket keeps roadbed width.

    Twelve metres of storage and taper are the native compact minima. Keep the
    compiler's 0.5 m retained span, and source dimensions at millimetre precision.
    Right pockets widen the footprint and still require the normal width preview.
    """
    if not action['left'] or action['right']:
        return
    near = 0 if action['at_start'] else 1
    tapers = edge.get('tapers', {})
    if near in tapers or any(t['pocket'] for t in tapers.values()):
        return
    clips = [edge.get('start_clip', 0), edge.get('end_clip', 0)]
    far = max(clips[1-near], tapers.get(1-near, {}).get('length_m', 0))
    room = edge['length_m'] - clips[near] - far - 0.5
    taper = min(action['taper_m'], room - 12.0)
    storage = min(action['pocket_m'], room - taper)
    if storage >= 12.0 and taper >= 12.0:
        action['pocket_m'] = math.floor(storage * 1000) / 1000
        action['taper_m'] = math.floor(taper * 1000) / 1000


def bus_section_pocket_removals(edges):
    """Undo an imported pocket only when its base exactly matches authored bus intent.

    The current tool cannot author bus ranks together with a pocket. Preserve the
    source/survey bus section instead of creating an extra lane that blocks it.
    """
    actions = []
    for e in edges:
        source = e.get('source', {})
        if not any(source.get('bus_masks', [])):
            continue
        counts = [source['fwd_lanes'], source['bkw_lanes']]
        if not e.get('source_forward', True):
            counts.reverse()
        for end, taper in e.get('tapers', {}).items():
            if taper['pocket'] and [taper['from_fwd'], taper['from_bkw']] == counts:
                actions.append({'kind': 'pocket', 'edge': e['id'], 'at_start': end == 1,
                                'left': False, 'right': False, 'pocket_m': 12, 'taper_m': 12,
                                'reason': 'restore_authored_bus_cross_section',
                                'source_ids': source.get('source_ids', [])})
    return actions


def prepare(path, plan, features):
    nodes, edges = read_save(path)
    _, incident = topology(nodes, edges)
    attribute(edges, source_index(plan))
    grid = defaultdict(list)
    for n, es in incident.items():
        if len(es) >= 3:
            p = nodes[n][::2]
            grid[math.floor(p[0]/32), math.floor(p[1]/32)].append(n)
    selected, audit = {}, []
    maximum_gap = POLICY['carriageway_distance_m']
    for f in features:
        p = f['properties']
        name = p.get('name')
        if len(f['points']) < 2:
            continue
        for spec in turn_lanes(p):
            record = {'source_id': f['id'], 'name': name, **spec}
            audit.append(record)
            if spec['reason']:
                continue
            points = f['points'][::spec['direction']]
            tip = points[-1]
            u = [tip[j]-points[-2][j] for j in range(2)]
            norm = math.hypot(*u)
            if not norm:
                record['reason'] = 'zero_source_heading'
                continue
            u = [v/norm for v in u]
            choices = []
            for x in range(math.floor((tip[0]-maximum_gap)/32), math.floor((tip[0]+maximum_gap)/32)+1):
                for z in range(math.floor((tip[1]-maximum_gap)/32), math.floor((tip[1]+maximum_gap)/32)+1):
                    for n in grid.get((x, z), []):
                        q = nodes[n][::2]
                        gap = distance(tip, q)
                        if gap > maximum_gap:
                            continue
                        for e in incident[n]:
                            if f['id'] not in e['source'].get('source_ids', []) and (not name or name not in e['source'].get('names', [])):
                                continue
                            at_start = e['a'] == n
                            if not (e['bkw'] if at_start else e['fwd']):
                                continue
                            shape = e['points'] if not at_start else e['points'][::-1]
                            # Travel tangent into the selected end, sampled beyond its tiny controls.
                            b = shape[-1]
                            a = next((a for a in reversed(shape[:-1]) if distance(a[::2], b[::2]) >= 15), shape[0])
                            v = [b[j]-a[j] for j in (0, 2)]
                            dot = sum(u[j]*v[j] for j in range(2))/max(.001, math.hypot(*v))
                            if dot >= .7:
                                choices.append((gap, e['id'], n, at_start, e))
            if not choices:
                record['reason'] = 'no_matching_saved_junction_within_carriageway_limit'
                continue
            gap, _, n, at_start, e = min(choices)

            if any(e['source'].get('bus_masks', [])):
                record.update(edge=e['id'], junction=n, at_start=at_start, source_gap_m=gap,
                              reason='bus_section_requires_combined_pocket_rank_authoring')
                continue
            lanes = spec['lanes']
            left = bool(lanes and lanes[0] and all(t in ('left', 'slight_left', 'sharp_left') for t in lanes[0]))
            right = bool(lanes and lanes[-1] and all(t in ('right', 'slight_right', 'sharp_right') for t in lanes[-1]))
            # Pockets reserve the exclusive movements; through rules are installed after splitting.
            action = {'kind': 'pocket', 'edge': e['id'], 'at_start': at_start, 'left': left, 'right': right,
                      'pocket_m': max(60., min(100., round(length(points)))), 'taper_m': 20.,
                      'source_id': f['id'], 'name': name, 'junction': n, 'source_lanes': lanes,
                      'source_gap_m': gap, 'base_lanes': e['bkw'] if at_start else e['fwd']}
            record.update(edge=e['id'], junction=n, at_start=at_start, source_gap_m=gap)
            if left and e['median_width'] < 3.8:
                record['reason'] = 'source_median_too_narrow_for_game_left_pocket'
                # Keep a tagged right pocket when the left cannot consume the existing median.
                action['left'] = False
            if not action['left'] and not right:
                record['reason'] = record['reason'] or 'no_exclusive_pocket_lane'
                continue
            incoming = e['bkw'] if at_start else e['fwd']
            if incoming == len(lanes):
                record['reason'] = 'saved_approach_already_has_source_lane_count'
                continue
            base = next((t['from_bkw' if at_start else 'from_fwd']
                         for t in e.get('tapers', {}).values() if t['pocket']), incoming)
            action['base_lanes'] = base
            if base + int(action['left']) + int(action['right']) != len(lanes):
                record['reason'] = 'source_through_count_mismatch'
                continue
            fit_compact_left_pocket(action, e)
            key = e['id'], at_start
            score = (gap, -int(left)-int(right), f['id'])
            if key not in selected or score < selected[key][0]:
                selected[key] = score, action
    return {'actions': bus_section_pocket_removals(edges) + [v[1] for _, v in sorted(selected.items())], 'source_audit': audit}


def movement(incoming, outgoing):
    """Use the game's inclusive 45-degree straight sector in world XZ coordinates."""
    dot = sum(a*b for a, b in zip(incoming, outgoing))
    cross = incoming[0]*outgoing[1]-incoming[1]*outgoing[0]
    if dot >= abs(cross):
        return 'through'
    return 'left' if cross < 0 else 'right'


def prepare_rules(path, plan, features):
    """Filter native lane connectors by OSM while retaining every untagged source lane.

    The game switches the whole node to whitelist mode, so installation includes
    all of its other existing car and pedestrian connectors in the same plan.
    """
    import sqlite3
    from pathlib import Path
    nodes, edges = read_save(path)
    lookup = {e['id']: e for e in edges}
    planned = prepare(path, plan, features)
    base = defaultdict(set)
    with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as c:
        for n, a, la, b, lb in c.execute('SELECT j.node_id,a.edge_id,a.lane_idx,b.edge_id,b.lane_idx FROM saved_lanes j JOIN saved_lanes a ON a.lane_id=j.from_lane_id JOIN saved_lanes b ON b.lane_id=j.to_lane_id WHERE j.edge_id=-1'):
            base[n].add((a, la, b, lb))
    aliases = {'slight_left': 'left', 'sharp_left': 'left', 'slight_right': 'right',
               'sharp_right': 'right', 'merge_to_left': 'through', 'merge_to_right': 'through'}
    selected, audit = {}, []
    for source in planned['source_audit']:
        if 'edge' not in source:
            continue
        item = dict(source)
        audit.append(item)
        e = lookup[source['edge']]
        count = e['bkw'] if source['at_start'] else e['fwd']
        if count != len(source['lanes']):
            item['rule_reason'] = 'saved_approach_count_differs_from_osm'
            continue
        allowed = [set(aliases.get(t, t) for t in tokens) for tokens in source['lanes']]
        if any(tokens-{'left', 'through', 'right', 'none'} for tokens in allowed):
            item['rule_reason'] = 'unsupported_turn_token'
            continue
        key = source['junction'], source['edge']
        score = source['source_gap_m'], source['source_id']
        if key not in selected or score < selected[key][0]:
            selected[key] = score, source, allowed
    by_node = defaultdict(list)
    for (node, _), value in selected.items():
        by_node[node].append(value)
    actions = []
    for node, specs in sorted(by_node.items()):
        rules = set(base[node])
        for _, source, allowed in specs:
            edge = lookup[source['edge']]
            at_start = source['at_start']
            shape = edge['points'] if at_start else edge['points'][::-1]
            incoming = [shape[0][j]-shape[1][j] for j in (0, 2)]
            for rule in sorted(base[node]):
                a, lane, b, _ = rule
                if a != edge['id'] or abs(lane) >= 100 or b not in lookup or b == a:
                    continue
                rank = lane if lane >= 0 else -lane-1
                tokens = allowed[rank]
                if not tokens or 'none' in tokens:
                    continue
                target = lookup[b]
                shape = target['points'] if target['a'] == node else target['points'][::-1]
                outgoing = [shape[1][j]-shape[0][j] for j in (0, 2)]
                if movement(incoming, outgoing) not in tokens:
                    rules.discard(rule)
        changed = len(base[node]-rules)
        if not any(abs(r[1]) < 100 for r in rules):
            continue
        actions.append({'kind': 'lane_rule', 'node': node, 'rules': sorted(rules),
                        'restricted_native_connectors': changed,
                        'source_ids': sorted(s['source_id'] for _, s, _ in specs)})
    return {'actions': actions, 'source_audit': audit}


if __name__ == '__main__':
    import json
    from pathlib import Path
    import sys
    from .normalise import normalise
    from .osm import OsmAdapter
    from .terrain import WorldTransform
    plan = json.loads(Path(sys.argv[2]).read_text())
    transform = WorldTransform(**{k: plan['world_transform'][k] for k in WorldTransform.__dataclass_fields__})
    features = normalise(OsmAdapter().read(Path(sys.argv[3])), transform)
    result = (prepare_rules if '--rules' in sys.argv else prepare)(sys.argv[1], plan, features)
    Path(sys.argv[4]).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(f"{len(result['actions'])} lane actions from {len(result['source_audit'])} tagged approaches")
