# SPDX-License-Identifier: GPL-2.0-only

"""Offline, indexed centreline conflation; all thresholds live in mapping.POLICY.

Work is O(segment bounding-box grid cells + sampled length + local candidates),
plus stable sorting. The 32 m grid serves parallel detection and endpoint projection;
there is no all-pairs road/path scan. No work runs in the simulation tick.
"""

from collections import defaultdict
from copy import deepcopy
import math

from .geometry import build_strokes, distance, junction_audit, radius_audit, simplify
from .mapping import POLICY


def collapse_roundabouts(features, roads, decisions):
    """Stage closed motor-traffic rings for ROAD-62 atomic junction replacement.

    Shared source nodes group split OSM ring ways. Approaches temporarily meet
    at the ring centre; apply replaces that junction through the player API.
    Foot/cycle rings are not vehicle roundabouts. Invalid/open rings are reported
    without moving their approaches. Work is linear in ring nodes plus sorting.
    """
    circles = sorted((f for f in features if f['properties'].get('roundabout') and not f['issues']
                      and f['properties'].get('road_class') not in ('cycleway', 'footway', 'path', 'pedestrian')), key=lambda f: f['id'])
    parents = list(range(len(circles)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    owner = {}
    for i, f in enumerate(circles):
        for n in f['node_ids']:
            if n in owner:
                parents[root(i)] = root(owner[n])
            owner[n] = i
    groups = defaultdict(list)
    for i, f in enumerate(circles):
        groups[root(i)].append(f)
    remap, audit = {}, []
    for fs in groups.values():
        points = [p for f in fs for p in f['points']]
        if not points:
            continue
        lo = tuple(min(p[j] for p in points) for j in range(2))
        hi = tuple(max(p[j] for p in points) for j in range(2))
        degree = defaultdict(int)
        for f in fs:
            for a, b in zip(f['node_ids'], f['node_ids'][1:]):
                degree[a] += 1
                degree[b] += 1
        diameter = max(hi[j]-lo[j] for j in range(2)) + 3.5
        if any(n != 2 for n in degree.values()) or diameter > 60.0:
            audit.append({'source_ids': [f['id'] for f in fs], 'status': 'unsupported',
                          'reason': 'open_or_oversized_ring', 'diameter_m': round(diameter, 3)})
            continue
        centre = tuple(round((lo[j]+hi[j])/2, 3) for j in range(2))
        node = 'roundabout/'+min(f['id'] for f in fs)
        for f in fs:
            for n in f['node_ids']:
                remap[n] = node, centre
            decisions[f['id'], 'road'] = {'action': 'skip', 'reason': 'roundabout_player_replacement', 'priority': 0}
        audit.append({'source_ids': [f['id'] for f in fs], 'centre_xz': centre,
                      'diameter_m': round(diameter, 3), 'status': 'planned',
                      'source_lanes': max(int(f['properties'].get('lanes') or 1) for f in fs),
                      'imported_lanes': 1,
                      'remaining': 'two_lane_entry_selection_and_spiral_exits' if any(
                          (f['properties'].get('lanes') or 1) > 1 for f in fs) else ''})
    for f, _ in roads:
        for i, n in enumerate(f['node_ids']):
            if n in remap:
                f['node_ids'][i], f['points'][i] = remap[n]
    return audit


def project(p, a, b):
    dx, dz = b[0]-a[0], b[1]-a[1]
    denom = dx*dx+dz*dz
    t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dz)/denom)) if denom else 0
    return a[0]+dx*t, a[1]+dz*t


class SegmentIndex:
    def __init__(self, records):
        self.grid = defaultdict(list)
        self.segments = []
        for key, points, payload in records:
            for a, b in zip(points, points[1:]):
                if distance(a, b) < .01:
                    continue
                i = len(self.segments)
                self.segments.append((key, a, b, payload))
                for x in range(math.floor(min(a[0], b[0])/32), math.floor(max(a[0], b[0])/32)+1):
                    for z in range(math.floor(min(a[1], b[1])/32), math.floor(max(a[1], b[1])/32)+1):
                        self.grid[x, z].append(i)

    def nearest(self, p, radius, predicate=lambda s: True):
        candidates = set()
        for x in range(math.floor((p[0]-radius)/32), math.floor((p[0]+radius)/32)+1):
            for z in range(math.floor((p[1]-radius)/32), math.floor((p[1]+radius)/32)+1):
                candidates.update(self.grid.get((x, z), ()))
        best = None
        for i in sorted(candidates):
            s = self.segments[i]
            if not predicate(s):
                continue
            q = project(p, s[1], s[2])
            d = distance(p, q)
            if d <= radius and (best is None or (d, i) < best[:2]):
                best = d, i, q, s
        return best


def samples(points, spacing):
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(distance(a, b)/spacing))
        for i in range(n):
            yield (a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n), (b[0]-a[0], b[1]-a[1]), distance(a, b)/n


def aligned(u, a, b, opposite=False):
    v = b[0]-a[0], b[1]-a[1]
    denom = math.hypot(*u)*math.hypot(*v)
    dot = (u[0]*v[0]+u[1]*v[1])/denom if denom else 0
    limit = math.cos(math.radians(POLICY['parallel_angle_deg']))
    return dot < -limit if opposite else abs(dot) >= limit


def conflate(roads, decisions):
    roads = deepcopy(sorted(roads, key=lambda r: r[0]['id']))
    from .medians import merge_carriageways
    changed, audit = merge_carriageways(roads, decisions)
    streets = SegmentIndex((f['id'], f['points'], f) for f, m in changed if m['action'] == 'road')
    result, dropped = [], []
    for f, m in changed:
        if m['action'] != 'footpath' or f['properties'].get('survey_underpass'):
            result.append((f, m))
            continue
        total, parallel = 0, 0
        for p, u, run in samples(f['points'], POLICY['sample_m']):
            total += run
            if streets.nearest(p, POLICY['parallel_distance_m'], lambda s: aligned(u, s[1], s[2])):
                parallel += run
        if total and parallel/total >= POLICY['parallel_fraction']:
            decisions[f['id'], 'road'] = {**m, 'action': 'skip', 'reason': 'parallel_to_road_sidewalk'}
            dropped.append({'source_ids': [f['id']], 'reason': 'parallel_to_road_sidewalk', 'length_m': round(total, 3)})
        else:
            result.append((f, m))
    return result, {'carriageways': audit, 'parallel_paths': dropped}


def snap_endpoints(roads, only_kind=None):
    """Project near-miss endpoints to local street segments and pin the target.

    Inserts target vertices before build_strokes so simplification cannot erase
    intended junctions. Only endpoints move; a 12 m bound prevents broad drift.
    """
    index = SegmentIndex((f['id'], f['points'], (f, m)) for f, m in roads if m['action'] == 'road')
    insertions = defaultdict(list)
    audit = []
    for f, m in roads:
        if only_kind is not None and m['action'] != only_kind:
            continue
        for i in (0, len(f['points'])-1):
            p = f['points'][i]
            hit = index.nearest(p, POLICY['endpoint_snap_m'], lambda s: s[0] != f['id'])
            if not hit:
                continue
            if hit[0] < .01:
                target = hit[3][3][0]
                # Median reattachment already projected this endpoint onto the segment.
                # It still needs a target vertex so curve fitting preserves the junction.
                if (any(distance(p, end) < .01 for end in (hit[3][1], hit[3][2]))
                        or f['properties'].get('layer', 0) != target['properties'].get('layer', 0)):
                    continue
            q = tuple(round(v, 3) for v in hit[2])
            node = f'snap/{q[0]:.3f}/{q[1]:.3f}'
            f['points'][i], f['node_ids'][i] = q, node
            insertions[hit[3][0]].append((hit[3][1], hit[3][2], q, node))
            audit.append({'source_id': f['id'], 'distance_m': round(hit[0], 3), 'point_xz': q})
    for f, _ in roads:
        additions = insertions[f['id']]
        if not additions:
            continue
        points, ids = [], []
        for a, b, na, nb in zip(f['points'], f['points'][1:], f['node_ids'], f['node_ids'][1:]):
            entries = [(a, na), (b, nb)] + [(q, node) for oa, ob, q, node in additions if oa == a and ob == b]
            entries.sort(key=lambda e: (distance(a, e[0]), e[1]))
            for q, node in entries:
                if points and points[-1] == q:
                    ids[-1] = node
                else:
                    points.append(q)
                    ids.append(node)
        f['points'], f['node_ids'] = points, ids
    return audit


def prepare_streets(roads, box, tolerance):
    """Prepare the car graph before paths can insert/move any of its vertices.

    Path policy must not change road geometry or placement order. Paths project
    onto the finished car centreline features; the game creates their actual
    road junctions during placement, after all car streets have settled.
    """
    cars = deepcopy([(f, m) for f, m in roads if m['action'] == 'road'])
    paths = deepcopy([(f, m) for f, m in roads if m['action'] == 'footpath'])
    snaps = snap_endpoints(cars)
    car_strokes, car_audit = build_strokes(cars, box, tolerance, POLICY['endpoint_merge_m'])
    snaps.extend(snap_endpoints(cars+paths, only_kind='footpath'))
    path_strokes, path_audit = build_strokes(paths, box, tolerance, POLICY['endpoint_merge_m'])
    strokes = car_strokes+path_strokes
    for i, s in enumerate(strokes):
        s['id'] = f'stroke/{i:05d}'
    audit = {'merged_nodes': car_audit['merged_nodes']+path_audit['merged_nodes'],
             'discarded': car_audit['discarded']+path_audit['discarded'],
             'junctions': junction_audit(strokes), 'radius_warnings': radius_audit(strokes)}
    return strokes, audit, snaps


def prune_stubs(strokes, discarded):
    arms = defaultdict(int)
    for s in strokes:
        for p in s['points_xz']:
            arms[tuple(p)] += 1
    result = []
    for s in strokes:
        dead_end = any(arms[tuple(p)] == 1 for p in (s['points_xz'][0], s['points_xz'][-1]))
        if s['kind'] == 'footpath' and (s['length_m'] < POLICY['standalone_path_min_m'] or s['length_m'] < POLICY['dead_end_path_m'] and dead_end):
            discarded.append({'source_ids': s['source_ids'], 'reason': 'short_dead_end_path' if dead_end else 'short_standalone_path', 'length_m': s['length_m']})
        else:
            result.append(s)
    return result
