# SPDX-License-Identifier: GPL-2.0-only

"""Prepare a save-only terrain-brush operation from the corrected still-water mask.

The NLS DEM contains water surfaces, not bathymetry. Deepen only already wet
sample centres in the import box; a sub-cell brush changes no adjacent sample.
The brush runs before road construction. The game recomputes still water at
its measured level after the brush finishes. Loaded-road terrain synchronization
is intentionally outside this preparation workflow.
No SQLite writes are performed here. Complexity is O(import-box terrain cells).
"""

import math
from .survey import TerrainGrid


def prepare_initial(world,box,edits):
    """Resolve lowered fills and bed samples before any road is constructed."""
    from .water import WaterMask
    terrain=TerrainGrid(world)
    replaced={tuple(e['seed_xz']) for e in edits if e.get('seed_xz')}
    fills=[f for f in terrain.fills if tuple(f['seed_xz']) not in replaced]
    for e in edits:
        seed=e['new_seed_xz'];surface=e['new_surface_m']
        if terrain.at(seed)>=surface:
            candidates=[]
            for dx in range(-250,251,10):
                for dz in range(-250,251,10):
                    q=[seed[0]+dx,seed[1]+dz]
                    if terrain.at(q)<surface-.005:candidates.append((dx*dx+dz*dz,dx,dz,q))
            if not candidates:continue
            seed=min(candidates)[-1]
        fills.append({'kind':'auto','seed_xz':seed,'surface_m':surface})
    mask=WaterMask.load(world,fills)
    samples=[]
    for iz in range(math.ceil(box[1]/mask.cell+(mask.height-1)/2),math.floor(box[3]/mask.cell+(mask.height-1)/2)+1):
        for ix in range(math.ceil(box[0]/mask.cell+(mask.width-1)/2),math.floor(box[2]/mask.cell+(mask.width-1)/2)+1):
            i=iz*mask.width+ix
            if .001<mask.depth[i]<.2:
                samples.append([round((ix-(mask.width-1)/2)*mask.cell,3),round((iz-(mask.height-1)/2)*mask.cell,3),round(mask.heights[i]-2,5)])
    return {'lowering_m':2,'radius_m':mask.cell*.4,'samples_xzy':samples}
