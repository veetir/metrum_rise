# SPDX-License-Identifier: GPL-2.0-only

"""Offline C1 Hermite centrelines sampled at the player's 4 m curve interval.

Chord-length derivatives preserve surveyed vertices and share tangents on both
sides of every vertex. Source junctions remain exact interpolation points.
O(vertices + sampled length); no simulation hot path or extra dependencies.
"""

import math
from collections import defaultdict
from .geometry import distance, length, roadbed_half_width


def smooth(points, step=4.0, fixed_tangents=None):
    points = [tuple(p) for i, p in enumerate(points) if not i or distance(p, points[i-1]) > .05]
    if len(points) < 2:
        return points
    fixed_tangents = fixed_tangents or {}
    tangents = []
    for i, p in enumerate(points):
        a, b = points[max(0, i-1)], points[min(len(points)-1, i+1)]
        run = distance(a, p)+distance(p, b)
        tangents.append(fixed_tangents.get(p, tuple((b[j]-a[j])/run if run else 0 for j in range(2))))
    result = [points[0]]
    for i, (a, b) in enumerate(zip(points, points[1:])):
        run = distance(a, b)
        n = max(1, math.ceil(run/step))
        for k in range(1, n+1):
            t = k/n
            h0, h1 = 2*t**3-3*t*t+1, -2*t**3+3*t*t
            h2, h3 = t**3-2*t*t+t, t**3-t*t
            result.append(tuple(round(h0*a[j]+h1*b[j]+run*(h2*tangents[i][j]+h3*tangents[i+1][j]), 3) for j in range(2)) if k < n else b)
    return result


def straight_throat(points, centre, radius):
    """Keep an explicitly authored junction throat straight through C1 fitting.

    The centre must already be a source vertex. Insert the two surveyed throat
    limits and constrain their shared tangent, preserving all outside vertices.
    """
    points, centre = [tuple(p) for p in points], tuple(centre)
    if centre not in points:
        return points, {}
    i = points.index(centre)
    if i == 0 or i == len(points)-1:
        return points, {}
    a, b = points[i-1], points[i+1]
    run = distance(a, b)
    axis = tuple((b[j]-a[j])/run for j in range(2))
    left, right = i, i
    while left and distance(points[left-1], centre) < radius:
        left -= 1
    while right+1 < len(points) and distance(points[right+1], centre) < radius:
        right += 1
    if not left or right == len(points)-1:
        return points, {}
    anchors = [tuple(centre[j]+sign*radius*axis[j] for j in range(2)) for sign in (-1, 1)]
    controls = points[:left]+[anchors[0], centre, anchors[1]]+points[right+1:]
    return controls, {p: axis for p in (anchors[0], centre, anchors[1])}


def name_boundary_controls(strokes, roundabout_centres=()):
    """Share one C1 direction where a unique divided corridor changes its name.

    Source controls inside the existing roadbed cannot author an independent
    bend before the through corridor resumes. Pinned junctions stay intact.
    Endpoint incidence and local prefix/suffix walks are O(strokes + vertices).
    """
    excluded = set(map(tuple, roundabout_centres))
    groups = defaultdict(list)
    for s in strokes:
        if s.get('kind') != 'road' or not s.get('median_style') or not s['fwd_lanes'] or not s['bkw_lanes']:
            continue
        for end in (0, -1):
            groups[tuple(s['points_xz'][end]), s['road_class'], s.get('layer', 0)].append((s, end))
    changes = {}
    for (joint, _, _), arms in groups.items():
        if joint in excluded or len(arms) != 2 or not all(s['names'] for s, _ in arms) or set(arms[0][0]['names']) & set(arms[1][0]['names']):
            continue
        prepared = []
        for s, end in arms:
            points, tangents = changes.get(s['id'], ([tuple(p) for p in s['points_xz']], {}))
            points = list(points)
            pins = set(map(tuple, s.get('junctions_xz', [])))
            neighbor = 1 if end == 0 else -2
            half = roadbed_half_width(s)
            while len(points) > 2 and distance(points[neighbor], joint) < half and points[neighbor] not in pins:
                points.pop(neighbor)
            gap = distance(points[neighbor], joint)
            if gap < half:
                break
            outward = tuple((points[neighbor][j]-joint[j])/gap for j in range(2))
            prepared.append((s, end, points, dict(tangents), outward))
        if len(prepared) != 2 or sum(a*b for a,b in zip(prepared[0][4], prepared[1][4])) > -.5:
            continue
        axis = tuple(prepared[0][4][j]-prepared[1][4][j] for j in range(2))
        norm = math.hypot(*axis)
        axis = tuple(v/norm for v in axis)
        for i, (s, end, points, tangents, _) in enumerate(prepared):
            sign = (1 if end == 0 else -1) * (1 if i == 0 else -1)
            tangents[joint] = tuple(sign*v for v in axis)
            changes[s['id']] = points, tangents
    return changes


def prepare_curves(strokes, features, straight_throats=(), roundabout_centres=()):
    lookup = {f['id']: f for f in features}
    for s in strokes:
        sources = [lookup[i] for i in s['source_ids'] if i in lookup]
        s['names'] = sorted({f['properties']['name'] for f in sources if f['properties']['name']})
    joined = name_boundary_controls(strokes, roundabout_centres)
    for s in strokes:
        # Authoring semantics come from the retained segment, not the union of
        # provenance IDs: opposing chains may include a distant short bridge.
        s.setdefault('layer', 0)
        s.setdefault('bridge', False)
        s.setdefault('tunnel', False)
        s['source_points_xz'] = s['points_xz']
        controls, tangents = joined.get(s['id'], (s['points_xz'], {}))
        # Joining reprojected carriageway sections can put their former mouth
        # controls inside the final wider roadbed. Keep actual source junctions,
        # but remove those obsolete controls before fitting the approach tangent.
        if s.get('median_style'):
            controls = list(controls)
            pins = set(map(tuple, s.get('junctions_xz', [])))
            half = roadbed_half_width(s)
            for end, neighbor in ((0, 1), (-1, -2)):
                while len(controls) > 2 and distance(controls[end], controls[neighbor]) < half and tuple(controls[neighbor]) not in pins:
                    controls.pop(neighbor)
        if tangents:
            s['name_boundary_joints_xz'] = sorted(tangents)
        for throat in straight_throats:
            if throat['name'] in s['names']:
                controls, fixed = straight_throat(controls, throat['centre_xz'], throat['radius_m'])
                tangents.update(fixed)
                if fixed:
                    s.setdefault('straight_throats', []).append(throat)
        s['points_xz'] = smooth(controls, fixed_tangents=tangents)
        s['length_m'] = round(length(s['points_xz']), 3)
    return strokes
