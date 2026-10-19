# SPDX-License-Identifier: GPL-2.0-only

"""Deterministic topology-preserving preparation; never predicts game acceptance.

Junctions/way endpoints merge to the nearest *existing representative* within 2 m
in a grid (ties by coordinate/ID); no transitive drift and no averaging. Only
same-layer nodes merge. Shared nodes and their adjacent vertices stay pinned.
Original ways stay continuous through junctions. Matching endpoint pieces join
with identical class, name, direction, lanes and speed; at branches the unique
compatible pair must have <=25 degrees of deflection. Directed chains never
reverse. These are explicit lossy policies, not searches for a favorable game
validator result. Junctions and first/last segments are protected from smoothing.

RDP is bounded to windows of 128 original vertices: O(128*n) worst case,
O(n) memory. Joining/clip are O(n), sorting O(n log n); grid queries cost local
occupancy. No all-pairs network scans. The game's normal validator remains the
authority on physical radius, junction angles/degree and surface crowding.
"""

from collections import defaultdict, deque
import math


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def length(points):
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def point_segment_distance(p, a, b):
    dx, dz = b[0]-a[0], b[1]-a[1]
    denom = dx*dx+dz*dz
    t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dz)/denom)) if denom else 0
    return distance(p, (a[0]+t*dx, a[1]+t*dz))


def simplify(points, tolerance=1.0, pinned=frozenset()):
    if len(points) < 3:
        return list(points)
    anchors = {0, 1, len(points)-2, len(points)-1}
    for i, p in enumerate(points):
        if tuple(p) in pinned:
            anchors.update(range(max(0, i-1), min(len(points), i+2)))
    anchors.update(range(0, len(points), 128))
    keep = set(anchors)
    ordered = sorted(anchors)
    stack = list(zip(ordered, ordered[1:]))
    while stack:
        lo, hi = stack.pop()
        if hi-lo < 2:
            continue
        error, index = max((point_segment_distance(points[i], points[lo], points[hi]), i)
                           for i in range(lo+1, hi))
        if error > tolerance:
            keep.add(index)
            stack.extend(((lo, index), (index, hi)))
    return [points[i] for i in sorted(keep)]


def clip_segment(a, b, box):
    low, high = 0.0, 1.0
    dx, dz = b[0]-a[0], b[1]-a[1]
    for p, q in ((-dx, a[0]-box[0]), (dx, box[2]-a[0]),
                 (-dz, a[1]-box[1]), (dz, box[3]-a[1])):
        if p == 0:
            if q < 0:
                return None
            continue
        t = q/p
        if p < 0:
            low = max(low, t)
        else:
            high = min(high, t)
        if low > high:
            return None
    if high-low <= 1e-12:
        return None
    return tuple(round(a[j]+low*(b[j]-a[j]), 3) for j in range(2)), tuple(round(a[j]+high*(b[j]-a[j]), 3) for j in range(2))


def clip_line(points, owners, box):
    fragments, current, sources = [], [], set()
    for a, b, owner in zip(points, points[1:], owners):
        clipped = clip_segment(a, b, box)
        if clipped is None:
            if len(current) > 1:
                fragments.append((current, sorted(sources)))
            current, sources = [], set()
            continue
        start, end = clipped
        if current and current[-1] != start:
            fragments.append((current, sorted(sources)))
            current, sources = [], set()
        if not current:
            current.append(start)
        if end != current[-1]:
            current.append(end)
            sources.add(owner)
    if len(current) > 1:
        fragments.append((current, sorted(sources)))
    return fragments


def merge_nodes(roads, tolerance):
    neighbors = defaultdict(set)
    positions, layers = {}, {}
    candidates = set()
    for f, _ in roads:
        ids = f['node_ids']
        for node, p in zip(ids, f['points']):
            positions[node] = tuple(p)
            layers[node] = f['properties']['layer']
        if ids:
            candidates.update((ids[0], ids[-1]))
        for a, b in zip(ids, ids[1:]):
            neighbors[a].add(b)
            neighbors[b].add(a)
    candidates.update(n for n, arms in neighbors.items() if len(arms) != 2)
    grid, representatives = defaultdict(list), {}
    merged = []
    for node in sorted(candidates, key=lambda n: (positions[n], n)):
        p = positions[node]
        cell = math.floor(p[0]/tolerance), math.floor(p[1]/tolerance)
        nearby = [rep for x in range(cell[0]-1, cell[0]+2) for z in range(cell[1]-1, cell[1]+2)
                  for rep in grid[(x, z)] if layers[rep] == layers[node]
                  and distance(p, positions[rep]) <= tolerance]
        rep = min(nearby, key=lambda n: (distance(p, positions[n]), positions[n], n)) if nearby else node
        representatives[node] = rep
        if rep == node:
            grid[cell].append(node)
        else:
            merged.append({'node': node, 'representative': rep, 'distance_m': round(distance(p, positions[rep]), 3)})
    return representatives, positions, merged


def build_strokes(roads, box, tolerance=1.0, merge_m=2.0):
    roads = sorted(roads, key=lambda item: item[0]['id'])
    provenance = {f['id']: f.get('source_ids', [f['id']]) for f, _ in roads}
    reps, positions, merged = merge_nodes(roads, merge_m)
    cooked = []
    incidence = defaultdict(set)
    for f, mapping in roads:
        ids = [reps.get(n, n) for n in f['node_ids']]
        ids = [n for i, n in enumerate(ids) if not i or n != ids[i-1]]
        if mapping.get('oneway', f['properties']['oneway']) == -1:
            ids.reverse()
        for a, b in zip(ids, ids[1:]):
            incidence[a].add(b)
            incidence[b].add(a)
        cooked.append((f, mapping, ids))
    pinned = {positions[n] for n, arms in incidence.items() if len(arms) != 2}
    segments, discarded, source_shapes = [], [], set()
    for f, m, ids in cooked:
        if len(ids) < 2:
            discarded.append({'source_ids': [f['id']], 'reason': 'collapsed_by_node_merge'})
            continue
        shape = tuple(positions[n] for n in ids)
        key = (m['action'], m['fwd_lanes'], m['bkw_lanes'], shape)
        reverse = (m['action'], m['bkw_lanes'], m['fwd_lanes'], shape[::-1])
        if key in source_shapes or reverse in source_shapes:
            discarded.append({'source_ids': [f['id']], 'reason': 'duplicate_stroke'})
            continue
        source_shapes.add(key)
        p = f['properties']
        signature = (m['action'], m['fwd_lanes'], m['bkw_lanes'], p['road_class'], p['name'], (m.get('median_style', 0), m.get('median_width_m', 0)), (p.get('bridge',False),p.get('tunnel',False),p.get('layer',0)), bool(m.get('oneway', p['oneway'])))
        segments.append({'nodes': ids, 'source': f['id'], 'mapping': m,
                         'signature': signature, 'class': p['road_class']})
    ends = defaultdict(list)
    for i, s in enumerate(segments):
        ends[(s['nodes'][0], s['signature'])].append(i)
        ends[(s['nodes'][-1], s['signature'])].append(i)
    used, strokes = set(), []
    for seed, original in enumerate(segments):
        if seed in used:
            continue
        used.add(seed)
        ids = deque(original['nodes'])
        owners = deque([original['source']] * (len(ids)-1))
        reversible = not original['signature'][-1] and original['signature'][1] == original['signature'][2]
        for at_start in (False, True):
            while ids[0] != ids[-1]:
                node = ids[0] if at_start else ids[-1]
                candidates = ends[(node, original['signature'])]
                if len(candidates) != 2:
                    break
                unused = [i for i in candidates if i not in used]
                if len(unused) != 1:
                    break
                i = unused[0]
                nxt = segments[i]
                part = list(nxt['nodes'])
                correct = part[-1] == node if at_start else part[0] == node
                if not correct:
                    if not reversible:
                        break
                    part.reverse()
                if len(incidence[node]) > 2:
                    own_tip = ids[1] if at_start else ids[-2]
                    new_tip = part[-2] if at_start else part[1]
                    origin = positions[node]
                    u = (positions[own_tip][0]-origin[0], positions[own_tip][1]-origin[1])
                    v = (positions[new_tip][0]-origin[0], positions[new_tip][1]-origin[1])
                    denom = math.hypot(*u)*math.hypot(*v)
                    if denom == 0 or (u[0]*v[0]+u[1]*v[1])/denom > math.cos(math.radians(155)):
                        break
                used.add(i)
                own = [nxt['source']] * (len(part)-1)
                if at_start:
                    ids.extendleft(reversed(part[:-1]))
                    owners.extendleft(reversed(own))
                else:
                    ids.extend(part[1:])
                    owners.extend(own)
        ids, owners = list(ids), list(owners)
        points = [positions[n] for n in ids]
        # The tool refuses identical endpoint nodes. A closed loop needs two commits.
        cuts = [0, len(points)-1]
        if ids[0] == ids[-1] and len(points) > 3:
            total, station = length(points), 0.0
            for j in range(1, len(points)-1):
                station += distance(points[j-1], points[j])
                if station >= total/2:
                    cuts.insert(1, j)
                    break
        for lo, hi in zip(cuts, cuts[1:]):
            for clipped, sources in clip_line(points[lo:hi+1], owners[lo:hi], box):
                clipped = simplify(clipped, tolerance, pinned)
                size = length(clipped)
                if size < 2:
                    discarded.append({'source_ids': sources, 'reason': 'below_2m_stroke', 'length_m': round(size, 3)})
                    continue
                m = original['mapping']
                order_sources = sources
                sources = sorted({source for owner in sources for source in provenance[owner]})
                strokes.append({'source_ids': sources, '_order_sources': order_sources,
                                'points_xz': clipped, 'road_class': original['class'],
                                'kind': m['action'], 'fwd_lanes': m['fwd_lanes'], 'bkw_lanes': m['bkw_lanes'],
                                'median_style': m.get('median_style', 0), 'median_width_m': m.get('median_width_m', 0),
                                'snap': True, 'priority': m['priority'], 'length_m': round(size, 3),
                                'junctions_xz': [p for p in clipped[1:-1] if p in pinned]})
    strokes.sort(key=lambda s: (s['priority'], s['_order_sources'], s['points_xz']))
    unique, seen = [], set()
    for stroke in strokes:
        del stroke['_order_sources']
        points = tuple(map(tuple, stroke['points_xz']))
        key = (stroke['kind'], stroke['fwd_lanes'], stroke['bkw_lanes'], points)
        reverse = (stroke['kind'], stroke['bkw_lanes'], stroke['fwd_lanes'], points[::-1])
        if key in seen or reverse in seen:
            discarded.append({'source_ids': stroke['source_ids'], 'reason': 'duplicate_stroke'})
        else:
            seen.add(key)
            stroke['id'] = f"stroke/{len(unique):05d}"
            unique.append(stroke)
    return unique, {'merged_nodes': merged, 'discarded': discarded, 'junctions': junction_audit(unique), 'radius_warnings': radius_audit(unique)}


def junction_audit(strokes):
    """Input-only angle/degree warnings. Physical compiler validation is deferred."""
    arms = defaultdict(set)
    for s in strokes:
        pts = s['points_xz']
        for i, p in enumerate(pts):
            for j in (i-1, i+1):
                if 0 <= j < len(pts) and distance(p, pts[j]) >= 0.05:
                    arms[tuple(p)].add(round(math.atan2(pts[j][1]-p[1], pts[j][0]-p[0]), 9))
    result = []
    for p, directions in sorted(arms.items()):
        if len(directions) < 3:
            continue
        angles = sorted(directions)
        smallest = min((angles[(i+1) % len(angles)]-angle) % (2*math.pi) for i, angle in enumerate(angles))
        if len(angles) > 6 or math.degrees(smallest) < 25:
            result.append({'point_xz': p, 'degree': len(angles), 'min_angle_deg': round(math.degrees(smallest), 3),
                           'status': 'input_warning_not_game_refusal'})
    return result


def roadbed_half_width(stroke):
    # build_surface_edge + visual_profile_half_widths_for_edge: 3.5 m lanes,
    # 1.5 m sidewalks (curb included); zero-lane footpaths are 2 m wide.
    return 1.0 if stroke['kind'] == 'footpath' else (stroke['fwd_lanes']+stroke['bkw_lanes'])*1.75 + 1.5 + stroke.get('median_width_m', 0)/2


def radius_audit(strokes):
    """ROAD-54 window measurement on input, not the compiler's physical geometry.

    O(n * bounded window segments); runs below 5 cm terminate windows. Radius
    below 1.25*half-width is a risk, not a refusal: the physical compiler may
    round the geometry differently. No game validator result is fabricated.
    """
    warnings = []
    for stroke in strokes:
        points = stroke['points_xz']
        segments = [(math.atan2(b[1]-a[1], b[0]-a[0]), distance(a, b)) for a, b in zip(points, points[1:])]
        half_width = roadbed_half_width(stroke)
        minimum = math.inf
        for first, (heading, run) in enumerate(segments[:-1]):
            if run < 0.05:
                continue
            arc, turn = run/2, 0.0
            for next_index in range(first+1, len(segments)):
                next_heading, next_run = segments[next_index]
                if next_run < 0.05:
                    break
                turn += (next_heading-heading+math.pi) % math.tau - math.pi
                heading = next_heading
                if arc + next_run/2 >= half_width*3:
                    arc += next_run/2
                    break
                arc += next_run
            if abs(turn) > 0:
                minimum = min(minimum, arc/abs(turn))
        if minimum < 1.25*half_width:
            warnings.append({'stroke_id': stroke['id'], 'input_radius_m': round(minimum, 3),
                             'illegal_limit_m': 1.25*half_width, 'status': 'input_warning_not_game_refusal'})
    return warnings
