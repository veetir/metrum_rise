# SPDX-License-Identifier: GPL-2.0-only

"""North-up road reference PNGs at exactly the capture harness's map scale."""

import json
import math
from pathlib import Path
import struct
import zlib
from .geometry import clip_line
from .mapping import map_role


def png(path, width, height, pixels):
    def chunk(kind, data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data))
    rows = b''.join(b'\0'+pixels[y*width*3:(y+1)*width*3] for y in range(height))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b''))


def render(output, features, strokes=None, prefix='osm'):
    output=Path(output)
    views=json.loads((output/'views.json').read_text())
    for view in views:
        w,h=1200,900
        scale=h/view['height_m']
        box=(view['x']-w/scale/2,view['z']-h/scale/2,view['x']+w/scale/2,view['z']+h/scale/2)
        pixels=bytearray([235,239,230])*(w*h)
        def put(x,y,color):
            if 0<=x<w and 0<=y<h:
                i=(y*w+x)*3;pixels[i:i+3]=bytes(color)
        def line(points,color):
            for clipped,_ in clip_line(points,['']*(len(points)-1),box):
                for a,b in zip(clipped,clipped[1:]):
                    n=max(1,math.ceil(math.dist(a,b)*scale))
                    for i in range(n+1):
                        x=round((a[0]+(b[0]-a[0])*i/n-box[0])*scale)
                        y=round((a[1]+(b[1]-a[1])*i/n-box[1])*scale)
                        put(x,y,color);put(x+1,y,color)
        for f in features:
            if f['source_tags'].get('natural')=='water':
                if f['points']:line(f['points'],(45,118,195))
                for ring in f['rings']:
                    line(ring['points'],(45,118,195))
        for f in features:
            if 'road' not in f['roles'] or map_role(f,'road')['action']!='road':continue
            if len(f['points'])>1:
                line(f['points'],(110,110,110) if strokes is not None else (28,30,35))
        if strokes is not None:
            for points in strokes:line(points,(210,55,35))
        # 100 m scale bar at bottom left, ticks each 20 m.
        n=round(100*scale)
        for x in range(30,31+n):put(x,h-30,(0,0,0))
        for tick in range(6):
            for y in range(h-34,h-25):put(30+round(tick*20*scale),y,(0,0,0))
        png(output/f'{prefix}_{view["name"]}.png',w,h,pixels)


if __name__=='__main__':
    root=Path('maps/processed/kuopio_sw')
    render(root/'shots',json.loads((root/'features.json').read_text()))
