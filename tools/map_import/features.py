# SPDX-License-Identifier: GPL-2.0-only

"""Resolve source markings/permissions after the acknowledged topology commits.

Uses the existing source SegmentIndex, a 32 m node grid and incident arms.
O(source vertices + saved vertices + local candidates), plus stable sorting;
no simulation tick work, extra dependency or SQLite mutation.
"""

from collections import defaultdict
import json
import math
from pathlib import Path
import sys

from .connectivity import attribute, read_save, source_index, topology
from .geometry import distance, point_segment_distance
from .pockets import prepare_rules


def crossing_intents(features, membership):
    """Crossing ways author paint only at shared source nodes, never inferred intersections."""
    intents, audit = [], []
    for f in features:
        if len(f['points']) == 1:
            intents.append(f)
            continue
        marked = f['properties'].get('crossing_marked')
        if marked is None:
            continue
        linked = False
        for node, point in zip(f['node_ids'], f['points']):
            roads = membership[node] - {f['id']}
            if not roads:
                continue
            linked = True
            intents.append({'id': f['id']+'/'+node, 'points': [point],
                            'road_source_ids': roads, 'crossing_way': True,
                            'properties': {'crossing_marked': marked}})
        if not linked:
            audit.append({'source_id': f['id'], 'reason': 'crossing_way_has_no_shared_source_node'})
    return intents, audit


def prepare(path, plan, features):
    nodes, edges = read_save(path)
    _, incident = topology(nodes, edges)
    attribute(edges, source_index(plan))
    membership = defaultdict(set)
    for f in features:
        if len(f['points']) > 1 and 'road' in f['roles']:
            for n in f['node_ids']:
                membership[n].add(f['id'])
    grid = defaultdict(list)
    for node, arms in incident.items():
        if len(arms) >= 2:
            x, _, z = nodes[node]
            grid[math.floor(x/32), math.floor(z/32)].append(node)
    actions, audit = [], []
    # Snapshot IDs are read after all structural edits. Source permissions are
    # constant along each prepared section; a count mismatch is never truncated.
    for e in edges:
        masks = e['source'].get('bus_masks', [0, 0])
        if not any(masks):
            continue
        counts = [e['source']['fwd_lanes'], e['source']['bkw_lanes']]
        if not e.get('source_forward', True):
            masks, counts = masks[::-1], counts[::-1]
        if [e['fwd'], e['bkw']] == counts:
            actions.append({'kind': 'bus_lanes', 'edge': e['id'], 'masks': masks})
        else:
            audit.append({'edge': e['id'], 'reason': 'source_bus_section_count_mismatch'})
    selected = {}
    surveys = [{'id': 'survey/university/'+str(i), 'points': [r['point_xz']],
                'road_source_ids': r['road_source_ids'], 'properties': {'crossing_marked': r['marked']}}
               for i, r in enumerate(plan.get('survey_crossings', []))]
    intents, way_audit = crossing_intents(features, membership)
    audit.extend(way_audit)
    for f in intents + surveys:
        p = f['properties']
        if len(f['points']) != 1:
            continue
        if p.get('junction_control'):
            audit.append({'source_id': f['id'], 'control': p['junction_control'],
                          'point_xz': f['points'][0], 'direction': p.get('control_direction', ''),
                          'reason': 'TRAFFIC_06_authored_control_API_pending'})
        marked = p.get('crossing_marked')
        if marked is None:
            continue
        record = {'source_id': f['id'], 'marked': marked}
        audit.append(record)
        point = f['points'][0]
        sources = set(f.get('road_source_ids', membership[f['id']]))
        choices = []
        # This is an explicit source registration limit, not a search for a
        # favorable renderer result. Markings without an existing arm are deferred.
        radius = 24.0
        for x in range(math.floor((point[0]-radius)/32), math.floor((point[0]+radius)/32)+1):
            for z in range(math.floor((point[1]-radius)/32), math.floor((point[1]+radius)/32)+1):
                for node in grid.get((x, z), ()):
                    gap = distance(point, nodes[node][::2])
                    if gap > radius:
                        continue
                    for e in incident[node]:
                        if not sources.intersection(e['source'].get('source_ids', ())):
                            continue
                        points = [q[::2] for q in e['points']]
                        shape_gap = min(point_segment_distance(point, a, b) for a, b in zip(points, points[1:]))
                        choices.append((shape_gap, gap, e['id'], node))
        if not choices:
            record['reason'] = 'marked_source_has_no_existing_arm_within_24m'
            continue
        shape_gap, gap, edge, node = min(choices)
        record.update(edge=edge, node=node, source_gap_m=shape_gap, junction_gap_m=gap)
        key = node, edge
        priority = 0 if f['id'].startswith('survey/') else 2 if f.get('crossing_way') else 1
        score = (priority, shape_gap, gap, f['id'])
        if key not in selected or score < selected[key][0]:
            selected[key] = score, {'kind': 'crosswalk', 'node': node, 'edge': edge,
                                    'marked': marked, 'source_id': f['id']}
    # New v95 ordinary arms start unmarked, but the roundabout preset explicitly
    # authors entry paint. Remove only those existing marked entries that source
    # metadata does not retain; absent entries need no transaction or sparse row.
    for e in edges:
        for node in e.get('marked_nodes', ()):
            desired = selected.get((node, e['id']))
            if desired is None or not desired[1]['marked']:
                actions.append({'kind': 'crosswalk', 'node': node, 'edge': e['id'], 'marked': False})
    actions.extend(a for _, a in (selected[key] for key in sorted(selected)))
    turns = prepare_rules(path, plan, features)
    actions.extend(turns['actions'])
    return {'actions': actions, 'source_audit': audit, 'turn_lanes': turns['source_audit']}


if __name__ == '__main__':
    plan = json.loads(Path(sys.argv[2]).read_text())
    features = json.loads(Path(sys.argv[3]).read_text())['features']
    result = prepare(sys.argv[1], plan, features)
    Path(sys.argv[4]).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
