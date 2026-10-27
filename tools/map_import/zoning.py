# SPDX-License-Identifier: GPL-2.0-only

"""Sample frontage intents, not free-standing buildings or painted simulations.

A 128 m polygon grid answers local source-land-use queries; a road sample every
24 m checks both frontages. Most specific containing building overrides landuse;
unsupported buildings and non-zoneable landuse are exclusion masks. Areas of
multipolygons subtract inner rings. No point outside the target box is emitted.
The game resolves road alignment and terrain/site feasibility at apply time.

Cost O(polygon covered cells + road length / spacing + local candidate vertices),
not O(roads*all buildings). Dense data's candidate occupancy remains explicit.
Nominal parcel area is only an intent; report actual area from committed parcels.
"""

from collections import defaultdict
import math
from .geometry import distance


def contains(point, ring):
    x, z = point
    inside = False
    for a, b in zip(ring, ring[1:] + ring[:1]):
        if (a[1] > z) != (b[1] > z) and x < (b[0]-a[0])*(z-a[1])/(b[1]-a[1])+a[0]:
            inside = not inside
    return inside


def area(ring):
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a, b in zip(ring, ring[1:]+ring[:1]))) / 2


class PolygonIndex:
    def __init__(self, features, decisions, box):
        self.grid = defaultdict(list)
        self.records = []
        for f in features:
            for role in ('building', 'landuse'):
                if role not in f['roles'] or f['issues']:
                    continue
                rings = f['rings']
                if rings:
                    if any(not r['closed'] for r in rings):
                        continue
                    outer = [r['points'] for r in rings if r['role'] == 'outer']
                    inner = [r['points'] for r in rings if r['role'] == 'inner']
                elif f['properties']['closed'] and not f['issues']:
                    outer, inner = [f['points']], []
                else:
                    continue
                if not outer:
                    continue
                points = [p for r in outer for p in r]
                left, top = max(box[0], min(p[0] for p in points)), max(box[1], min(p[1] for p in points))
                right, bottom = min(box[2], max(p[0] for p in points)), min(box[3], max(p[1] for p in points))
                if left > right or top > bottom:
                    continue
                index = len(self.records)
                self.records.append({'id': f['id'], 'role': role, 'outer': outer, 'inner': inner,
                                     'area': sum(area(r) for r in outer)-sum(area(r) for r in inner),
                                     **decisions[(f['id'], role)]})
                for x in range(math.floor(left/128), math.floor(right/128)+1):
                    for z in range(math.floor(top/128), math.floor(bottom/128)+1):
                        self.grid[(x, z)].append(index)

    def at(self, point):
        candidates = [self.records[i] for i in self.grid[(math.floor(point[0]/128), math.floor(point[1]/128))]]
        containing = [r for r in candidates if any(contains(point, p) for p in r['outer'])
                      and not any(contains(point, p) for p in r['inner'])]
        return min(containing, key=lambda r: (r['role'] != 'building', r['area'], r['id'])) if containing else None


def frontage_intents(features, decisions, strokes, box, zone_cell_m):
    index = PolygonIndex(features, decisions, box)
    intents, seen = [], set()
    for stroke in strokes:
        if stroke['kind'] != 'road':
            continue
        points = stroke['points_xz']
        # The current ordinary road has 3.5 m lanes; leave one parcel half-depth beyond its edge.
        offset = (stroke['fwd_lanes']+stroke['bkw_lanes'])*3.5/2 + 3 + zone_cell_m
        station, next_sample = 0.0, 12.0
        for a, b in zip(points, points[1:]):
            run = distance(a, b)
            if run == 0:
                continue
            while next_sample < station+run:
                t = (next_sample-station)/run
                for side in (-1, 1):
                    p = (a[0]+t*(b[0]-a[0])-side*(b[1]-a[1])/run*offset,
                         a[1]+t*(b[1]-a[1])+side*(b[0]-a[0])/run*offset)
                    if not (box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3]):
                        continue
                    source = index.at(p)
                    if source is None or source['action'] not in ('residential', 'commercial'):
                        continue
                    key = (round(p[0]/12), round(p[1]/12))
                    if key in seen:
                        continue
                    seen.add(key)
                    intents.append({'id': f'zone/{len(intents):05d}', 'stroke_id': stroke['id'],
                                    'source_id': source['id'], 'zone_type': source['action'], 'density': 'low',
                                    'point_xz': tuple(round(v, 3) for v in p), 'frontage_cells': 2,
                                    'depth_cells': 2, 'nominal_area_m2': 4*zone_cell_m**2})
                next_sample += 24
            station += run
    return intents
