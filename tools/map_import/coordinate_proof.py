# SPDX-License-Identifier: GPL-2.0-only

"""Independent projection check and reproducible DEM cross-section measurements."""

import math
from .projection import tm35fin
from .survey import TerrainGrid, quantiles
from .conflate import samples
from .source_dem import SourceDem


def transverse_mercator_series(lat, lon):
    # Independent Snyder longitude-power formulation (not the Krueger n-series).
    a=6378137.; f=1/298.257222101; e2=f*(2-f); ep=e2/(1-e2)
    p=math.radians(lat); l=math.radians(lon-27)
    n=a/math.sqrt(1-e2*math.sin(p)**2); t=math.tan(p)**2; c=ep*math.cos(p)**2; A=math.cos(p)*l
    m=a*((1-e2/4-3*e2**2/64-5*e2**3/256)*p-(3*e2/8+3*e2**2/32+45*e2**3/1024)*math.sin(2*p)+(15*e2**2/256+45*e2**3/1024)*math.sin(4*p)-35*e2**3/3072*math.sin(6*p))
    return (500000+.9996*n*(A+(1-t+c)*A**3/6+(5-18*t+t*t+72*c-58*ep)*A**5/120),
            .9996*(m+n*math.tan(p)*(A*A/2+(5-t+9*c+4*c*c)*A**4/24+(61-58*t+t*t+600*c-330*ep)*A**6/720)))


def proof(features,world,source_folder):
    terrain=TerrainGrid(world)
    source=SourceDem(source_folder)
    controls=[]
    for name,lat,lon in [('Niirala',62.889552,27.652509),('Neulamäki',62.883311,27.616350),('southwest',62.871,27.588),('northeast',62.901,27.674)]:
        e,n=tm35fin(lat,lon); e2,n2=transverse_mercator_series(lat,lon)
        world_point=(e-533000,6975000-n)
        controls.append({'source_dem_height_m':source.at(world_point),'world_dem_height_m':terrain.at(world_point),'resampling_height_residual_m':terrain.at(world_point)-source.at(world_point),'name':name,'lat':lat,'lon':lon,'tm35_en':[e,n],'world_xz':[e-533000,6975000-n],'independent_projection_residual_m':math.hypot(e-e2,n-n2)})
    sections=[]
    names={'Kehäkatu','Kaikukatu','Neulamäentie','Savilahdentie','Kolmisopentie','Vt 5'}
    for f in features:
        p=f['properties']
        if p['name'] not in names or p.get('bridge') or p.get('tunnel') or len(f['points'])<2 or 'road' not in f['roles']:continue
        for pos,u,_ in samples(f['points'],30):
            run=math.hypot(*u)
            if not run:continue
            normal=(-u[1]/run,u[0]/run)
            heights=[source.at((pos[0]+normal[0]*s,pos[1]+normal[1]*s)) for s in range(-30,31)]
            scores=[]
            for shift in range(-15,16):
                c=30+shift
                inside=sum((heights[c+j+1]-heights[c+j])**2 for j in range(-3,3))/6
                outside=[sum(abs(heights[c+sign*j+sign]-heights[c+sign*j]) for j in range(8,14))/6 for sign in (-1,1)]
                scores.append((inside,shift,outside))
            inside,shift,outside=min(scores,key=lambda a:(a[0],abs(a[1]),a[1]))
            # A measurable platform has low crossfall between steeper shoulders.
            # Discard boundary minima, which do not locate a platform in this window.
            if math.sqrt(inside)<.08 and min(outside)>.12 and abs(shift)<15:
                sections.append({'source_id':f['id'],'name':p['name'],'point_xz':pos,'normal_xz':normal,'best_cross_section_shift_m':shift,'platform_slope_rms':math.sqrt(inside),'shoulder_slope_means':outside})
    return {'known_coordinate_checks':controls,'dem_road_cross_sections':sections,
            'road_cross_section_abs_residuals':quantiles([abs(s['best_cross_section_shift_m']) for s in sections]),
            'limitations':['Known coordinate checks verify projection arithmetic against an independent formulation; they are not surveyed ground control.','Platform offsets are diagnostic measurements on identifiable sections; asymmetric road widths and nearby platforms can bias them.','Original 2 m DEM and OSM do not establish sub-metre absolute geodetic accuracy. No transform correction is justified by these measurements.']}
