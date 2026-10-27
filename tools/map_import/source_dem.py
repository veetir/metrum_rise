# SPDX-License-Identifier: GPL-2.0-only

"""Read local NLS float GeoTIFF tiles with TIFF LZW using only the standard library.

Used for independent registration checks against the original 2 m raster,
without relying on the game's resampled 10 m terrain. Only tiled, single-band,
32-bit floating point north-up EPSG:3067 TIFF with predictor 1 is accepted.
"""

from array import array
from collections import OrderedDict
import math
from pathlib import Path
import struct
import sys
from .terrain import dem_bounds, tiff_tags


def lzw(data):
    table=[bytes([i]) for i in range(256)]+[b'',b'']
    bits=9;offset=0;previous=b'';out=bytearray()
    while offset+bits<=len(data)*8:
        byte=offset//8;shift=offset%8
        window=int.from_bytes(data[byte:byte+3].ljust(3,b'\0'),'big')
        code=(window>>(24-shift-bits))&((1<<bits)-1);offset+=bits
        if code==256:
            table=table[:258];bits=9;previous=b'';continue
        if code==257:break
        if code<len(table):entry=table[code]
        elif code==len(table) and previous:entry=previous+previous[:1]
        else:raise ValueError('Invalid TIFF LZW code')
        out.extend(entry)
        if previous and len(table)<4096:
            table.append(previous+entry[:1])
            if len(table)==(1<<bits)-1 and bits<12:bits+=1
        previous=entry
    return out


def directory(path):
    with path.open('rb') as f:
        endian='<' if f.read(2)==b'II' else '>'
        magic,offset=struct.unpack(endian+'HI',f.read(6))
        if magic!=42:raise ValueError('Expected classic TIFF')
        f.seek(offset);n,=struct.unpack(endian+'H',f.read(2));entries=[f.read(12) for _ in range(n)]
        tags={}
        for entry in entries:
            tag,kind,count=struct.unpack(endian+'HHI',entry[:8])
            if kind not in (3,4):continue
            fmt=endian+{3:'H',4:'I'}[kind]*count;size=struct.calcsize(fmt)
            if size<=4:raw=entry[8:8+size]
            else:
                f.seek(struct.unpack(endian+'I',entry[8:])[0]);raw=f.read(size)
            tags[tag]=struct.unpack(fmt,raw)
    return endian,tags


class SourceDem:
    def __init__(self,folder):
        bounds=dem_bounds(folder)
        self.origin_e=(bounds[0]+bounds[2])/2
        self.origin_n=(bounds[1]+bounds[3])/2
        self.tiles=[];self.cache=OrderedDict()
        for path in sorted(Path(folder).glob('*.tif')):
            geo=tiff_tags(path);endian,tags=directory(path)
            if tags[258]!=(32,) or tags[259]!=(5,) or tags.get(317,(1,))!=(1,) or tags[277]!=(1,) or tags[339]!=(3,):raise ValueError('Unsupported source DEM encoding')
            sx,sy,_=geo[33550];gx,gy,_,east,north,_=geo[33922]
            self.tiles.append({'path':path,'endian':endian,'tags':tags,'west':east-gx*sx,'north':north+gy*sy,'cell':sx,'width':tags[256][0],'height':tags[257][0]})

    def _pixel(self,tile,x,y):
        t=tile['tags'];tw,th=t[322][0],t[323][0]
        x=max(0,min(tile['width']-1,x));y=max(0,min(tile['height']-1,y))
        index=(y//th)*math.ceil(tile['width']/tw)+x//tw
        key=(tile['path'],index)
        if key not in self.cache:
            with tile['path'].open('rb') as f:
                f.seek(t[324][index]);raw=lzw(f.read(t[325][index]))
            data=array('f');data.frombytes(raw)
            if (tile['endian']=='<') != (sys.byteorder=='little'):data.byteswap()
            if len(data)!=tw*th:raise ValueError('Truncated DEM tile')
            self.cache[key]=data
            if len(self.cache)>128:self.cache.popitem(last=False)
        else:self.cache.move_to_end(key)
        return self.cache[key][(y%th)*tw+x%tw]

    def at(self,p):
        e,n=p[0]+self.origin_e,self.origin_n-p[1]
        for tile in self.tiles:
            if tile['west']<=e<tile['west']+tile['width']*tile['cell'] and tile['north']-tile['height']*tile['cell']<n<=tile['north']:
                x=(e-tile['west'])/tile['cell']-.5;y=(tile['north']-n)/tile['cell']-.5
                ix,iy=math.floor(x),math.floor(y);tx,ty=x-ix,y-iy
                a=self._pixel(tile,ix,iy)*(1-tx)+self._pixel(tile,ix+1,iy)*tx
                b=self._pixel(tile,ix,iy+1)*(1-tx)+self._pixel(tile,ix+1,iy+1)*tx
                return a*(1-ty)+b*ty
        raise ValueError('Point outside source DEM')
