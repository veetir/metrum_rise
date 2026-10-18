# SPDX-License-Identifier: GPL-2.0-only

"""Offline C1 Hermite centrelines sampled at the player's 4 m curve interval.

Chord-length derivatives preserve surveyed vertices and share tangents on both
sides of every vertex. Source junctions remain exact interpolation points.
O(vertices + sampled length); no simulation hot path or extra dependencies.
"""

import math
from .geometry import distance, length


def smooth(points, step=4.0):
    points = [tuple(p) for i, p in enumerate(points) if not i or distance(p, points[i-1]) > .05]
    if len(points) <= 2:
        return points
    tangents = []
    for i, p in enumerate(points):
        a, b = points[max(0, i-1)], points[min(len(points)-1, i+1)]
        run = distance(a, p)+distance(p, b)
        tangents.append(tuple((b[j]-a[j])/run if run else 0 for j in range(2)))
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


def prepare_curves(strokes, features):
    lookup = {f['id']: f for f in features}
    for s in strokes:
        sources = [lookup[i] for i in s['source_ids'] if i in lookup]
        s['names'] = sorted({f['properties']['name'] for f in sources if f['properties']['name']})
        s['layer'] = max((f['properties'].get('layer', 0) for f in sources), default=0)
        s['bridge'] = any(f['properties'].get('bridge') for f in sources)
        s['tunnel'] = any(f['properties'].get('tunnel') for f in sources)
        s['source_points_xz'] = s['points_xz']
        s['points_xz'] = smooth(s['points_xz'])
        s['length_m'] = round(length(s['points_xz']), 3)
    return strokes
