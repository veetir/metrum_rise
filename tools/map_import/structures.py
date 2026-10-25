# SPDX-License-Identifier: GPL-2.0-only

"""Offline structural source profiles, using bank heights rather than DEM below decks.

An explicit source bridge is one deck between its source endpoints. Lower roads
use a bounded 100 m approach neighborhood on either side of exact source-layer
crossings; distant terrain and junction elevations remain source samples. Interpolate
height by horizontal arc length: a riverbed or lower street must not pull its
interior down. No arbitrary elevation offset, inferred bridge, or terrain edit.
Native clearance validation still owns crossings and may refuse the product.
Cost O(source vertices + local indexed candidates + strokes log strokes), offline only.
"""

from collections import defaultdict
import bisect
import math

from .geometry import distance
from .mapping import POLICY
from .conflate import SegmentIndex


SOURCE_CROSSING_APPROACH_M = 100.0


def source_crossings(strokes):
    # Reuse the existing segment grid, visiting only cells touched by each segment.
    levels = {s['id']: int(s.get('layer', 0)) or (1 if s.get('bridge') else -1 if s.get('tunnel') else 0)
              for s in strokes}
    index = SegmentIndex((s['id'], s['points_xz'], s) for s in strokes)
    lower = defaultdict(list)
    for s in strokes:
        station = 0.0
        for a, b in zip(s['points_xz'], s['points_xz'][1:]):
            candidates = set()
            for x in range(math.floor(min(a[0], b[0])/32), math.floor(max(a[0], b[0])/32)+1):
                for z in range(math.floor(min(a[1], b[1])/32), math.floor(max(a[1], b[1])/32)+1):
                    candidates.update(index.grid.get((x, z), ()))
            for i in sorted(candidates):
                key, c, d, other = index.segments[i]
                if levels[key] <= levels[s['id']]:
                    continue
                # A shared source terminal is a structural continuation, not a crossing.
                if set(map(tuple, (s['points_xz'][0], s['points_xz'][-1]))) & set(map(tuple, (other['points_xz'][0], other['points_xz'][-1]))):
                    continue
                u = b[0]-a[0], b[1]-a[1]
                v = d[0]-c[0], d[1]-c[1]
                delta = c[0]-a[0], c[1]-a[1]
                cross = u[0]*v[1]-u[1]*v[0]
                if abs(cross) < 1e-9:
                    continue
                t = (delta[0]*v[1]-delta[1]*v[0])/cross
                r = (delta[0]*u[1]-delta[1]*u[0])/cross
                if 0 <= t <= 1 and 0 <= r <= 1:
                    lower[s['id']].append(station + t*distance(a, b))
            station += distance(a, b)
    return lower


def prepare_structure_profiles(strokes, terrain):
    """Attach bank-to-bank profiles and exact source continuation levels to strokes."""
    lower = source_crossings(strokes)
    joints = defaultdict(set)
    for s in strokes:
        level = int(s.get('layer', 0)) or (1 if s.get('bridge') else -1 if s.get('tunnel') else 0)
        for p in (s['points_xz'][0], s['points_xz'][-1]):
            joints[tuple(p)].add(level)
    for s in strokes:
        s['endpoint_levels'] = [sorted(joints[tuple(p)]) for p in (s['points_xz'][0], s['points_xz'][-1])]
        if not s.get('bridge') and s['id'] not in lower:
            continue
        points = s['points_xz']
        stations = [0.0]
        for a, b in zip(points, points[1:]):
            stations.append(stations[-1]+distance(a, b))
        if stations[-1] <= 0:
            continue
        intervals = [(0, len(points)-1)] if s.get('bridge') else []
        for crossing in sorted(lower.get(s['id'], [])):
            a = max(0, bisect.bisect_right(stations, crossing-SOURCE_CROSSING_APPROACH_M)-1)
            b = min(len(points)-1, bisect.bisect_left(stations, crossing+SOURCE_CROSSING_APPROACH_M))
            if intervals and a <= intervals[-1][1]:
                intervals[-1] = intervals[-1][0], max(b, intervals[-1][1])
            else:
                intervals.append((a, b))
        heights = [terrain.at(p) for p in points]
        for a, b in intervals:
            if stations[b] <= stations[a]:
                continue
            start, end = heights[a], heights[b]
            for i in range(a, b+1):
                heights[i] = start+(end-start)*(stations[i]-stations[a])/(stations[b]-stations[a])
        if s.get('bridge'):
            # A DEM samples the lower road even at a short deck's OSM endpoints.
            # Recover the supports outside that data-resolution footprint, then
            # interpolate the whole deck without a place-specific height offset.
            halo = terrain.cell * POLICY['bridge_bank_halo_cells']
            a, b = points[0], points[-1]
            u = (a[0]-points[1][0], a[1]-points[1][1])
            v = (b[0]-points[-2][0], b[1]-points[-2][1])
            ur, vr = math.hypot(*u), math.hypot(*v)
            if ur and vr:
                supports = [(a[0]+u[0]*halo/ur, a[1]+u[1]*halo/ur),
                            (b[0]+v[0]*halo/vr, b[1]+v[1]*halo/vr)]
                start, end = map(terrain.at, supports)
                heights = [start+(end-start)*(halo+station)/(stations[-1]+2*halo) for station in stations]
                s['bridge_support_points_xyz'] = [[q[0], y, q[1]] for q, y in zip(supports, (start, end))]
        s['points_xyz'] = [[p[0], round(y, 6), p[1]] for p, y in zip(points, heights)]
        s['profile_authority'] = 'source_bridge_bank_chord' if s.get('bridge') else 'source_lower_crossing_chord'
    # Source-shared ordinary approaches use the same deck endpoint and resume
    # sampled ground outside the support halo. The Rust standard solver smooths
    # these supplied targets before the layered clearance envelope is applied.
    deck_ends = {tuple(s['points_xz'][i]): s['points_xyz'][i][1] for s in strokes
                 if s.get('bridge') and 'points_xyz' in s for i in (0, -1)}
    for s in strokes:
        if s.get('bridge'):
            continue
        points = s['points_xz']
        ends = [i for i in (0, -1) if tuple(points[i]) in deck_ends]
        if not ends:
            continue
        stations = [0.0]
        for a, b in zip(points, points[1:]):
            stations.append(stations[-1]+distance(a, b))
        heights = [p[1] for p in s['points_xyz']] if 'points_xyz' in s else [terrain.at(p) for p in points]
        for i in ends:
            heights[i] = deck_ends[tuple(points[i])]
        halo = terrain.cell * POLICY['bridge_bank_halo_cells']
        for i in ends:
            a, b = (0, min(len(points)-1, bisect.bisect_left(stations, halo))) if i == 0 else (
                max(0, bisect.bisect_right(stations, stations[-1]-halo)-1), len(points)-1)
            if stations[b] <= stations[a]:
                continue
            start, end = heights[a], heights[b]
            for j in range(a, b+1):
                heights[j] = start+(end-start)*(stations[j]-stations[a])/(stations[b]-stations[a])
        s['points_xyz'] = [[p[0], round(y, 6), p[1]] for p, y in zip(points, heights)]
        s['profile_authority'] = 'source_bridge_approach_chord'
    # Install decks before ordinary roads pin the source joints to the ground below.
    # Paths remain after motor topology, including their independent bridge decks.
    strokes.sort(key=lambda s: (s['kind'] == 'footpath', not s.get('bridge', False), s['priority'], s['id']))
    for i, s in enumerate(strokes):
        s['id'] = f'stroke/{i:05d}'
    return strokes
