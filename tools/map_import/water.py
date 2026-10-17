# SPDX-License-Identifier: GPL-2.0-only

"""Read-only authored water reconstruction, matching world_definition.rs.

Floods are four-connected and edge-escape rules distinguish lakes/open water.
Source heights are f32 * 20; visible depth is bilinear. O(world samples + filled
cells per authored fill) setup, O(local stroke length / 1 m) clipping. This is
offline preparation, never a replacement for the game's placement validation.
"""

from array import array
from collections import deque
import math
from pathlib import Path
import sqlite3
import sys

from .geometry import distance, length, roadbed_half_width, simplify
from .mapping import POLICY


class WaterMask:
    def __init__(self, width, height, cell, depth):
        self.width, self.height, self.cell, self.depth = width, height, cell, depth

    @classmethod
    def load(cls, path):
        with sqlite3.connect(Path(path).resolve().as_uri()+'?mode=ro', uri=True) as conn:
            w, h, cell, chunk = conn.execute('SELECT width_m,height_m,terrain_cell_m,terrain_chunk_m FROM world_definition_meta').fetchone()
            nx, nz, span = round(w/cell)+1, round(h/cell)+1, math.ceil(chunk/cell)
            heights = array('f', [0])*(nx*nz)
            for x, z, cw, ch, blob in conn.execute('SELECT * FROM world_terrain_chunks ORDER BY chunk_z,chunk_x'):
                data = array('f')
                data.frombytes(blob)
                if sys.byteorder != 'little':
                    data.byteswap()
                for row in range(ch):
                    start = (z*span+row)*nx+x*span
                    heights[start:start+cw] = array('f', (v*20 for v in data[row*cw:(row+1)*cw]))
            depth = array('f', [0])*(nx*nz)
            fills = []
            for name, open_water in (('world_lake_fills', False), ('world_open_water_fills', True)):
                for x, z, surface in conn.execute('SELECT * FROM '+name+' ORDER BY rowid'):
                    sx, sz = max(0, min(nx-1, math.floor((x+w/2)/cell+.5))), max(0, min(nz-1, math.floor((z+h/2)/cell+.5)))
                    seed = sz*nx+sx
                    if heights[seed] >= surface:
                        continue
                    visited = bytearray(nx*nz)
                    visited[seed] = 1
                    queue, cells, edge = deque([seed]), array('I'), False
                    while queue:
                        i = queue.popleft()
                        cells.append(i)
                        px, pz = i % nx, i//nx
                        edge |= px == 0 or pz == 0 or px == nx-1 or pz == nz-1
                        for j in ((i-1 if px else -1), (i+1 if px+1 < nx else -1),
                                  (i-nx if pz else -1), (i+nx if pz+1 < nz else -1)):
                            if j >= 0 and not visited[j] and heights[j] <= surface:
                                visited[j] = 1
                                queue.append(j)
                    applied = edge == open_water
                    fills.append({'seed_xz': [x, z], 'surface_m': surface, 'filled_cells': len(cells), 'applied': applied})
                    if applied:
                        for i in cells:
                            depth[i] = max(depth[i], surface-heights[i])
        mask = cls(nx, nz, cell, depth)
        mask.fills = fills
        return mask

    def at(self, p):
        x = p[0]/self.cell+(self.width-1)/2
        z = p[1]/self.cell+(self.height-1)/2
        if not (0 <= x <= self.width-1 and 0 <= z <= self.height-1):
            return 0
        ix, iz = min(math.floor(x), self.width-2), min(math.floor(z), self.height-2)
        tx, tz = x-ix, z-iz
        i = iz*self.width+ix
        a = self.depth[i]*(1-tx)+self.depth[i+1]*tx
        b = self.depth[i+self.width]*(1-tx)+self.depth[i+self.width+1]*tx
        return a*(1-tz)+b*tz

    def safe(self, p, u, half):
        run = math.hypot(*u)
        if run == 0:
            return self.at(p) <= .001
        n = max(1, math.ceil(2*half))
        return all(self.at((p[0]-u[1]/run*(-half+2*half*i/n), p[1]+u[0]/run*(-half+2*half*i/n))) <= .001 for i in range(n+1))

    def clip(self, strokes):
        result, audit = [], []
        for s in strokes:
            chunks, current, touched = [], [], False
            half = roadbed_half_width(s)+POLICY['water_margin_m']
            for a, b in zip(s['points_xz'], s['points_xz'][1:]):
                u = b[0]-a[0], b[1]-a[1]
                n = max(1, math.ceil(distance(a, b)))
                for i in range(n+1):
                    p = tuple(round(a[j]+(b[j]-a[j])*i/n, 3) for j in range(2))
                    if self.safe(p, u, half):
                        if not current or current[-1] != p:
                            current.append(p)
                    else:
                        touched = True
                        if len(current) > 1:
                            chunks.append(current)
                        current = []
            if len(current) > 1:
                chunks.append(current)
            if not touched:
                result.append(s)
                continue
            safe_chunks = []
            for chunk in chunks:
                # Original junction pins survive clipping and simplification.
                pins = set(map(tuple, s.get('junctions_xz', [])))
                chunk = simplify(chunk, .1, pins)
                if length(chunk) >= 10:
                    safe_chunks.append(chunk)
            retained = sum(length(c) for c in safe_chunks)
            if s['length_m']-retained > .5:
                audit.append({'stroke_id': s['id'], 'source_ids': s['source_ids'], 'reason': 'authored_water_trim',
                              'removed_m': round(s['length_m']-retained, 3), 'safe_pieces': len(safe_chunks)})
            for i, chunk in enumerate(safe_chunks):
                result.append({**s, 'id': s['id']+f'/dry/{i}', 'points_xz': chunk, 'length_m': round(length(chunk), 3),
                               'junctions_xz': [p for p in s.get('junctions_xz', []) if p in chunk]})
        return result, audit
