# SPDX-License-Identifier: GPL-2.0-only

"""Offline structural source profiles, using bank heights rather than DEM below decks.

An explicit source bridge is one deck between its source endpoints. Interpolate
height by horizontal arc length: a riverbed or lower street must not pull its
interior down. No arbitrary elevation offset, inferred bridge, or terrain edit.
Native clearance validation still owns crossings and may refuse the product.
Cost O(structure vertices + strokes log strokes), offline only.
"""

from collections import defaultdict

from .geometry import distance


def prepare_structure_profiles(strokes, terrain):
    """Attach bank-to-bank profiles and exact source continuation levels to strokes."""
    joints = defaultdict(set)
    for s in strokes:
        level = int(s.get('layer', 0)) or (1 if s.get('bridge') else -1 if s.get('tunnel') else 0)
        for p in (s['points_xz'][0], s['points_xz'][-1]):
            joints[tuple(p)].add(level)
    for s in strokes:
        s['endpoint_levels'] = [sorted(joints[tuple(p)]) for p in (s['points_xz'][0], s['points_xz'][-1])]
        if not s.get('bridge'):
            continue
        points = s['points_xz']
        stations = [0.0]
        for a, b in zip(points, points[1:]):
            stations.append(stations[-1]+distance(a, b))
        start, end = terrain.at(points[0]), terrain.at(points[-1])
        if stations[-1] <= 0:
            continue
        s['points_xyz'] = [[p[0], round(start+(end-start)*station/stations[-1], 6), p[1]]
                           for p, station in zip(points, stations)]
        s['profile_authority'] = 'source_bridge_bank_chord'
    # Install decks before ordinary roads pin the source joints to the ground below.
    # Paths remain after motor topology, including their independent bridge decks.
    strokes.sort(key=lambda s: (s['kind'] == 'footpath', not s.get('bridge', False), s['priority'], s['id']))
    for i, s in enumerate(strokes):
        s['id'] = f'stroke/{i:05d}'
    return strokes
