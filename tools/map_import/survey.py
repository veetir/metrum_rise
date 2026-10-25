# SPDX-License-Identifier: GPL-2.0-only

"""Local DEM/OSM registration evidence and water-level recommendations, stdlib only."""

from array import array
from collections import Counter
import json
import math
from pathlib import Path
import sqlite3
import sys
from .conflate import SegmentIndex, samples
from .geometry import distance
from .zoning import contains as point_in_polygon


class TerrainGrid:
    def __init__(self, path):
        with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as c:
            w, h, self.cell, chunk = c.execute('SELECT width_m,height_m,terrain_cell_m,terrain_chunk_m FROM world_definition_meta').fetchone()
            self.nx, self.nz = round(w/self.cell)+1, round(h/self.cell)+1
            span = math.ceil(chunk/self.cell)
            self.values = array('f', [0])*(self.nx*self.nz)
            for x, z, cw, ch, blob in c.execute('SELECT * FROM world_terrain_chunks'):
                data = array('f'); data.frombytes(blob)
                if sys.byteorder != 'little':
                    data.byteswap()
                for row in range(ch):
                    start = (z*span+row)*self.nx+x*span
                    self.values[start:start+cw] = array('f', (v*20 for v in data[row*cw:(row+1)*cw]))
            self.fills = [{'kind': kind, 'seed_xz': [x,z], 'surface_m': y} for table, kind in (('world_lake_fills','lake'),('world_open_water_fills','open')) for x,z,y in c.execute('SELECT * FROM '+table)]

    def at(self, p):
        x, z = p[0]/self.cell+(self.nx-1)/2, p[1]/self.cell+(self.nz-1)/2
        ix, iz = math.floor(x), math.floor(z)
        tx, tz = x-ix, z-iz
        i = iz*self.nx+ix
        return (self.values[i]*(1-tx)+self.values[i+1]*tx)*(1-tz)+(self.values[i+self.nx]*(1-tx)+self.values[i+self.nx+1]*tx)*tz


def rings(features):
    result = []
    for f in features:
        if f['source_tags'].get('natural') != 'water':
            continue
        if f['properties']['closed']:
            result.append((f['properties']['name'] or f['id'], f['points']))
        for r in f['rings']:
            if r['closed'] and r['role']=='outer':
                result.append((f['properties']['name'] or f['id'], r['points']))
    return result


def quantiles(values):
    v = sorted(values)
    return {'n':len(v), 'median_m': v[len(v)//2], 'p90_m':v[min(len(v)-1,int(len(v)*.9))], 'max_m':v[-1]} if v else None


def inspect(features, world, box, terrain=None, contour_step=10):
    terrain = terrain or TerrainGrid(world)
    waters = []
    observations = []
    edits = []
    for name, ring in rings(features):
        lo = [min(p[j] for p in ring) for j in (0,1)]; hi = [max(p[j] for p in ring) for j in (0,1)]
        if hi[0]<box[0] or hi[1]<box[1] or lo[0]>box[2] or lo[1]>box[3]:
            continue
        inside = [(x,z) for x in range(math.ceil(lo[0]/10)*10,math.ceil(hi[0]/10)*10,10) for z in range(math.ceil(lo[1]/10)*10,math.ceil(hi[1]/10)*10,10) if point_in_polygon((x,z),ring)]
        counts = Counter(round(terrain.at(p),2) for p in inside)
        if not counts:
            continue
        level, count = counts.most_common(1)[0]
        # NLS has no bathymetry: flood only centimetres above the surveyed flat surface.
        surface = level+.03
        flat = [p for p in inside if abs(terrain.at(p)-level)<.02]
        seed = min(flat or inside, key=lambda p:(distance(p, ((lo[0]+hi[0])/2,(lo[1]+hi[1])/2)),p))
        existing = [f for f in terrain.fills if f['kind']=='lake' and point_in_polygon(f['seed_xz'],ring)]
        old = min(existing,key=lambda f:distance(f['seed_xz'],seed)) if existing else None
        contour = []
        step = contour_step
        for x in range(math.ceil((lo[0]-150)/step)*step,math.ceil((hi[0]+150)/step)*step,step):
            for z in range(math.ceil((lo[1]-150)/step)*step,math.ceil((hi[1]+150)/step)*step,step):
                a=(x,z); ha=terrain.at(a)-surface
                for b in ((x+step,z),(x,z+step)):
                    hb=terrain.at(b)-surface
                    if ha*hb<0:
                        t=ha/(ha-hb); contour.append((x+(b[0]-x)*t,z+(b[1]-z)*t))
        # point-index segments must exceed the index's 1 cm degeneracy threshold.
        index=SegmentIndex((str(i),[p,(p[0]+.011,p[1])],None) for i,p in enumerate(contour))
        residuals=[]; vectors=[]
        for p,u,_ in samples(ring,10):
            hit=index.nearest(p,150)
            if hit:
                residuals.append(hit[0]); vectors.append([hit[2][j]-p[j] for j in (0,1)])
                run = math.hypot(*u)
                if count >= 100 and run:
                    n = (-u[1]/run,u[0]/run)
                    observations.append((name,p,n,sum(n[j]*(hit[2][j]-p[j]) for j in (0,1))))
        waters.append({'name':name,'osm_area_m2':len(inside)*100,'flat_surface_m':level,'flat_cells':count,'interior_cells':len(inside),'old_fill':old,'recommended_surface_m':surface,'shoreline_residuals':quantiles(residuals),'mean_shore_vector_xz_m':[sum(p[j] for p in vectors)/len(vectors) for j in (0,1)] if vectors else None})
        if count >= 30:
            edits.append({**(old or {'kind':'lake','seed_xz':None,'surface_m':None}),'new_surface_m':surface,'new_seed_xz':seed,'name':name})
    edits.extend({**f, 'new_surface_m':82.41,'new_seed_xz':f['seed_xz'],'name':'Kallavesi'} for f in terrain.fills if f['kind']=='open' and 86 < f['surface_m'] < 87)
    return {'shoreline_similarity_fit': fit_registration(observations), 'water':waters,'water_edits':edits,'coordinate_contract':{'scale':1,'rotation_deg':0,'origin_e':533000,'origin_n':6975000,'note':'Contract values, not a fitted registration. Shoreline mismatch cannot alone identify a transform; source DEM is 10 m.'}}


def fit_registration(observations):
    # Each lake has its own normal displacement: a sampled flat plateau is
    # inset from the surveyed shore. Do not misinterpret that erosion as scale.
    names=sorted({a[0] for a in observations})
    size=4+len(names)
    rows=[]
    for name,p,n,target in observations:
        x,z=(p[0]+700)/1000,(p[1]-1800)/1000
        row=[n[0],n[1],-n[0]*z+n[1]*x,n[0]*x+n[1]*z]+[float(name==key) for key in names]
        rows.append((row,target))
    matrix=[[sum(row[i]*row[j] for row,_ in rows) for j in range(size)]+[sum(row[i]*target for row,target in rows)] for i in range(size)]
    for k in range(size):
        pivot=max(range(k,size),key=lambda i:abs(matrix[i][k]))
        matrix[k],matrix[pivot]=matrix[pivot],matrix[k]
        denom=matrix[k][k]
        if abs(denom)<1e-10:return {'identifiable':False}
        matrix[k]=[v/denom for v in matrix[k]]
        for i in range(size):
            if i==k:continue
            factor=matrix[i][k]
            matrix[i]=[a-factor*b for a,b in zip(matrix[i],matrix[k])]
    beta=[row[-1] for row in matrix]
    residuals=[target-sum(a*b for a,b in zip(row,beta)) for row,target in rows]
    return {'n':len(rows),'pivot_xz':[-700,1800],'translation_at_pivot_xz_m':beta[:2], 'rotation_deg':math.degrees(beta[2]/1000),'scale':1+beta[3]/1000,'normal_residual_rms_m':math.sqrt(sum(v*v for v in residuals)/len(residuals)), 'lake_normal_bias_m':dict(zip(names,beta[4:])), 'applied':False,'note':'Diagnostic nearest-contour linear similarity fit with per-lake normal bias. 10 m DEM sampling and shoreline shape dominate; this is not surveyed ground control.'}


def coast_residuals(features,terrain,box,surface,feature_id,step=2):
    """Compare local OSM coast fragments even when the extract cannot close the lake ring."""
    from .geometry import clip_line
    contour=[]
    for x in range(math.floor((box[0]-100)/step)*step,math.ceil((box[2]+100)/step)*step,step):
        for z in range(math.floor((box[1]-100)/step)*step,math.ceil((box[3]+100)/step)*step,step):
            a=(x,z);ha=terrain.at(a)-surface
            for b in ((x+step,z),(x,z+step)):
                hb=terrain.at(b)-surface
                if ha*hb<0:
                    t=ha/(ha-hb);contour.append((x+(b[0]-x)*t,z+(b[1]-z)*t))
    index=SegmentIndex((str(i),[p,(p[0]+.011,p[1])],None) for i,p in enumerate(contour))
    residuals=[]
    for f in features:
        if f['id']!=feature_id:continue
        for ring in f['rings']:
            if ring['role']!='outer':continue
            for points,_ in clip_line(ring['points'],['']*(len(ring['points'])-1),box):
                for p,_,_ in samples(points,10):
                    hit=index.nearest(p,150)
                    if hit:residuals.append(hit[0])
    return {'box_xz':box,'surface_m':surface,'shoreline_residuals':quantiles(residuals),'note':'Coast fragments, not an invented closed Kallavesi polygon.'}


def apply_road_surveys(roads, include_park_paths=True):
    """Apply the user's Oct 22 university section survey to the working copy.

    Source IDs bound this correction; neutral OSM records remain unchanged.
    The two short bridge ways incorrectly say two lanes per carriageway. The
    user's six-lane/median/bus-lane contract overrides those tags, not geometry.
    """
    university = {'osm/way/'+s for s in ('37568149', '613918836', '1119987973',
                                       '1119987974', '1119987975', '1119987976')}
    audit = []
    for f, m in roads:
        if f['id'] == 'osm/way/1119706187':
            # The unnamed return carriageway meets Sarastuskaari at the Prisma
            # fork and the surveyed Luola ring. Keep it in that paired corridor.
            f['properties']['name'] = 'Sarastuskaari'
            audit.append({'source_id': f['id'], 'reason': 'luola_return_carriageway_identity',
                          'authority': 'user_2026_10_24_and_source_node_continuity'})
        if f['id'] in ('osm/way/1119987950', 'osm/way/1119987951'):
            # Both the OSM unclassified ways and the photographed zebra carry cars.
            # Under solves vehicle clearance below the short level-one main-road span.
            f['properties'].update(survey_underpass=True, tunnel=False, layer=0)
            m.update(action='road', fwd_lanes=1, bkw_lanes=1, priority=4)
            audit.append({'source_id': f['id'], 'reason': 'surveyed_motor_road_underpass',
                          'reference': 'user-2026-10-21/university-underpass-1.png'})
        if include_park_paths and f['id'] == 'osm/way/987262495':
            # Piipolku uses the adjacent opening; it must survive parallel-path filtering.
            f['properties'].update(survey_underpass=True, tunnel=False, layer=0)
        if f['id'] not in university:
            continue
        f['properties'].update(lanes=3, through_lanes=3, survey_cross_section='university_2026_10_22')
        m.update(fwd_lanes=3, bkw_lanes=0)
        audit.append({'source_id': f['id'], 'lanes_per_direction': 3,
                      'median_style': 3, 'median_width_m': 2.0, 'median_width_is_surveyed': False,
                      'bus_lane_rank': 2, 'reference': 'user-2026-10-21/university-1.png',
                      'authority': 'user_cross_section_2026_10_22'})
    return audit


def university_access(roads, decisions):
    """Use the photographed ordinary two-way parking arm instead of two OSM slips.

    The reference point selects the surveyed main-road source only. Projection
    reuses SegmentIndex; normal source endpoint preparation creates the junction.
    """
    main = [(f, m) for f, m in roads if f['properties'].get('survey_cross_section') == 'university_2026_10_22']
    access = next((f for f, _ in roads if f['id'] == 'osm/way/1119987953'), None)
    if not main or access is None:
        return roads, []
    index = SegmentIndex((f['id'], f['points'], f) for f, _ in main)
    hit = index.nearest((-502.0, 1206.7), 12.0)
    if not hit:
        return roads, [{'reason': 'surveyed_university_main_not_found'}]
    q = tuple(round(v, 3) for v in hit[2])
    result = []
    for f, m in roads:
        if f['id'] == 'osm/way/1119987954':
            decisions[f['id'], 'road'] = {**m, 'action': 'skip', 'reason': 'surveyed_two_way_parking_access'}
            continue
        if f['id'] == access['id']:
            parking, node = f['points'][-1], f['node_ids'][-1]
            f['points'] = [parking, q]
            f['node_ids'] = [node, f'median/{q[0]:.3f}/{q[1]:.3f}']
            f['properties'].update(road_class='service', oneway=0, layer=0, bridge=False, tunnel=False)
            m.update(oneway=0, fwd_lanes=1, bkw_lanes=1, median_style=3, median_width_m=2.0, priority=4)
        result.append((f, m))
    return result, [{'source_ids': [access['id'], 'osm/way/1119987954'], 'point_xz': q,
                     'marked': True, 'road_source_ids': [access['id']],
                     'reference': 'user-2026-10-21/university_yliopistonranta_level_junction.png'}]
