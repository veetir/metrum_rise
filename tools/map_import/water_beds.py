# SPDX-License-Identifier: GPL-2.0-only

"""Offline bathymetry for complete edited still-water fills (WATER-05).

NLS DEM samples describe the lake surface, not its bed. Give shallow wet
samples the importer's existing 2 m bed lowering, across the entire edited
fill, independently of the road crop. Dry samples and unrelated fills stay
unchanged. Work is O(world samples + flooded cells per edit), with O(world
samples) scratch, at preparation only; no gameplay or render changes.
"""

from array import array
import argparse
import bisect
import json
import math
from pathlib import Path
import sqlite3
import sys
import tempfile

from .survey import TerrainGrid
from .water import WaterMask

LOWERING_M = 2.0
SHALLOW_DEPTH_M = .2


def _floats(blob):
    values = array('f')
    values.frombytes(blob)
    if sys.byteorder != 'little':
        values.byteswap()
    return values


def _blob(values):
    if sys.byteorder == 'little':
        return values.tobytes()
    values = array('f', values)
    values.byteswap()
    return values.tobytes()


def _copy_updated(source, output, update):
    # Publish only a complete DB; inputs (including city saves) are always read-only.
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('Water preparation requires a distinct output')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as directory:
        temporary = Path(directory)/'water.sqlite'
        with sqlite3.connect(source.as_uri()+'?mode=ro', uri=True) as src, sqlite3.connect(temporary) as dst:
            src.backup(dst)
            result = update(dst)
            dst.commit()
            if dst.execute('PRAGMA quick_check').fetchone() != ('ok',):
                raise ValueError('Prepared water database failed integrity check')
        temporary.replace(output)
    return result


def prepare_initial(world, edits, output):
    """Write a derived world with complete beds before player road construction."""
    terrain = TerrainGrid(world)
    fills = []
    for edit in edits:
        seed, surface = edit['new_seed_xz'], edit['new_surface_m']
        if terrain.at(seed) >= surface:
            candidates = []
            for dx in range(-250, 251, 10):
                for dz in range(-250, 251, 10):
                    q = [seed[0]+dx, seed[1]+dz]
                    if terrain.at(q) < surface-.005:
                        candidates.append((dx*dx+dz*dz, dx, dz, q))
            if not candidates:
                continue
            seed = min(candidates)[-1]
        fills.append({'kind': 'auto', 'seed_xz': seed, 'surface_m': surface})
    mask = WaterMask.load(world, fills)

    def update(conn):
        chunk, = conn.execute('SELECT terrain_chunk_m FROM world_definition_meta').fetchone()
        span = math.ceil(chunk/mask.cell)
        count = 0
        for x, z, width, height, blob in conn.execute('SELECT * FROM world_terrain_chunks ORDER BY chunk_z,chunk_x').fetchall():
            values = _floats(blob)
            changed = False
            for row in range(height):
                for col in range(width):
                    i = (z*span+row)*mask.width+x*span+col
                    if .001 < mask.depth[i] < SHALLOW_DEPTH_M:
                        j = row*width+col
                        values[j] = values[j]-LOWERING_M/20
                        count += 1
                        changed = True
            if changed:
                conn.execute('UPDATE world_terrain_chunks SET source_height_blob_f32_le=? WHERE chunk_x=? AND chunk_z=?',
                             (_blob(values), x, z))
        return count

    count = _copy_updated(world, output, update)
    return {'lowering_m': LOWERING_M, 'samples': count}


def repair_save(source, output, edits):
    """Repair a new city copy without terrain-brush road resynchronization.

    City saves persist source terrain and baseline depth, not authoring fills.
    Change both together so wet/dry membership and the saved surface elevation
    stay fixed. Only shallow samples at explicitly edited levels qualify. Roads,
    agents, camera, weather and every other table retain their exact rows.
    """
    if Path(output).exists():
        raise ValueError('Repair output already exists; choose a new save')
    levels = sorted({e['new_surface_m'] for e in edits})
    if any(not math.isfinite(v) for v in levels):
        raise ValueError('Water levels must be finite')

    def update(conn):
        width, height, blob = conn.execute('SELECT * FROM terrain_state').fetchone()
        heights = _floats(blob)
        w, h, blob = conn.execute('SELECT * FROM water_state').fetchone()
        depths = _floats(blob)
        if (width, height) != (w, h) or len(heights) != w*h or len(depths) != w*h:
            raise ValueError('Terrain/water dimensions differ')
        count = 0
        for i, depth in enumerate(depths):
            if not .001 < depth < SHALLOW_DEPTH_M:
                continue
            surface = heights[i]*20+depth
            # Only f32 reconstruction error is allowed, not a fitted lake level.
            index = bisect.bisect_left(levels, surface-.0001)
            if index == len(levels) or abs(levels[index]-surface) > .0001:
                continue
            old = heights[i]
            heights[i] = old-LOWERING_M/20
            depths[i] = depth+(old-heights[i])*20
            count += 1
        conn.execute('UPDATE terrain_state SET height_blob_f32_le=?', (_blob(heights),))
        conn.execute('UPDATE water_state SET baseline_depth_blob_f32_le=?', (_blob(depths),))
        return {'samples': count, 'lowering_m': LOWERING_M, 'levels_m': levels}

    return _copy_updated(source, output, update)


def main():
    parser = argparse.ArgumentParser(description='Repair cropped importer water beds into a new city save')
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--plan', required=True, type=Path)
    args = parser.parse_args()
    # Installation into the user's saves is a separate, explicit operation.
    from .cli import safe_output
    safe_output(args.output.parent)
    print(json.dumps(repair_save(args.source, args.output, json.loads(args.plan.read_text())['water_edits']), sort_keys=True))


if __name__ == '__main__':
    main()
