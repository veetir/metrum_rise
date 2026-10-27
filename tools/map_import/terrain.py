# SPDX-License-Identifier: GPL-2.0-only

"""Derive georeferencing from the *actual* DEM tiles and validate the world DB.

import_dem_world_definition.import_dem_array makes an inclusive W/cell+1 grid,
GDAL north-up rows run from max_n southward, world_definition.rs loads rows
without reversal, TerrainSystem::grid_to_world_coords subtracts half extents.
Thus x=E-(min_e+max_e)/2 and z=(min_n+max_n)/2-N, scale 1. Heights in the DB
are elevation/HEIGHT_SCALE (20); this has no effect on horizontal metres.

Only classic north-up GeoTIFF metadata is read; raster pixels are not decoded.
Unexpected CRS, rotation, dimensions or missing georeferencing fail closed.
"""

from dataclasses import asdict, dataclass
from pathlib import Path
import sqlite3
import struct


def tiff_tags(path):
    with Path(path).open('rb') as stream:
        marker = stream.read(2)
        endian = {'II': '<', 'MM': '>'}.get(marker.decode('ascii'))
        if endian is None:
            raise ValueError(f"Not a TIFF: {path}")
        magic, offset = struct.unpack(endian + 'HI', stream.read(6))
        if magic != 42:
            raise ValueError("Only classic TIFF metadata is supported")
        stream.seek(offset)
        count, = struct.unpack(endian + 'H', stream.read(2))
        entries = [stream.read(12) for _ in range(count)]
        tags = {}
        formats = {3: 'H', 4: 'I', 12: 'd'}
        for entry in entries:
            tag, kind, size = struct.unpack(endian + 'HHI', entry[:8])
            if tag not in {256, 257, 33550, 33922, 34735, 34264} or kind not in formats:
                continue
            fmt = endian + formats[kind] * size
            length = struct.calcsize(fmt)
            if length <= 4:
                payload = entry[8:8+length]
            else:
                stream.seek(struct.unpack(endian + 'I', entry[8:])[0])
                payload = stream.read(length)
            tags[tag] = struct.unpack(fmt, payload)
        return tags


def dem_bounds(directory):
    tiles = sorted(Path(directory).glob('*.tif'))
    if not tiles:
        raise ValueError(f"No GeoTIFF tiles in {directory}")
    bounds = []
    for path in tiles:
        tags = tiff_tags(path)
        if 34264 in tags or 33550 not in tags or 33922 not in tags:
            raise ValueError(f"Unsupported affine georeferencing in {path}")
        keys = tags.get(34735, ())
        crs = [keys[i+3] for i in range(4, len(keys), 4) if keys[i] == 3072 and keys[i+1] == 0]
        if crs != [3067]:
            raise ValueError(f"Expected EPSG:3067 in {path}, got {crs}")
        sx, sy, _ = tags[33550]
        gx, gy, _, east, north, _ = tags[33922]
        if sx <= 0 or sy <= 0:
            raise ValueError("Expected north-up positive pixel scales")
        west, top = east - gx*sx, north + gy*sy
        bounds.append((west, top - tags[257][0]*sy, west + tags[256][0]*sx, top))
    return min(b[0] for b in bounds), min(b[1] for b in bounds), max(b[2] for b in bounds), max(b[3] for b in bounds)


@dataclass(frozen=True)
class WorldTransform:
    min_e: float
    min_n: float
    max_e: float
    max_n: float
    cell_m: float
    zone_cell_m: float

    def world(self, east, north):
        return east - (self.min_e+self.max_e)/2, (self.min_n+self.max_n)/2 - north

    def record(self):
        return {**asdict(self), 'x_origin_e': (self.min_e+self.max_e)/2,
                'z_origin_n': (self.min_n+self.max_n)/2, 'x_axis': 'east', 'z_axis': 'south',
                'horizontal_scale': 1, 'stored_height_to_metres': 20}


def load_transform(dem_dir, world_path):
    bounds = dem_bounds(dem_dir)
    with sqlite3.connect(Path(world_path).resolve().as_uri() + '?mode=ro', uri=True) as conn:
        width, height, cell, zone = conn.execute(
            'SELECT width_m,height_m,terrain_cell_m,zone_cell_m FROM world_definition_meta'
        ).fetchone()
        if abs(width - (bounds[2]-bounds[0])) > 1e-6 or abs(height - (bounds[3]-bounds[1])) > 1e-6:
            raise ValueError("DEM coverage does not match world definition dimensions")
        # Inclusive sample bounds and chunk orientation are checked against the loader contract.
        chunk, = conn.execute('SELECT terrain_chunk_m FROM world_definition_meta').fetchone()
        import math
        span = math.ceil(chunk / cell)
        nx, nz = round(width/cell)+1, round(height/cell)+1
        for x, z, w, h, size in conn.execute(
            'SELECT chunk_x,chunk_z,width_samples,height_samples,length(source_height_blob_f32_le) FROM world_terrain_chunks'
        ):
            if x < 0 or z < 0 or w <= 0 or h <= 0 or w > span or h > span or x*span+w > nx or z*span+h > nz or size != w*h*4:
                raise ValueError('Invalid world terrain chunk layout')
    return WorldTransform(*bounds, cell, zone)
