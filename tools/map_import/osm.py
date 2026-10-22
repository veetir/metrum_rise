# SPDX-License-Identifier: GPL-2.0-only

"""Offline OSM API/Overpass JSON adapter; all OSM-specific interpretation is here.

Unassembled relation fragments and missing member references are retained. Only
closed multipolygon rings become zoning masks; route/boundary relations are
provenance, never accidental duplicate roads. Geometry nodes remain first-class
neutral records, including untagged support nodes. Adapter cost is O(n log n).
"""

import json
import math
from .model import SourceFeature


def number(value):
    try:
        parsed = float(value)
        return parsed if math.isfinite(parsed) else None
    except (TypeError, ValueError):
        return None


def speed(value):
    if not value:
        return None
    pieces = str(value).split()
    v = number(pieces[0])
    return v * 1.609344 if v is not None and len(pieces) > 1 and pieces[1] == 'mph' else v


def marked_crossing(tags):
    """None means no paint authority; explicit absent markings override crossing type."""
    markings = tags.get('crossing:markings')
    if markings in ('no', 'none'):
        return False
    if markings:
        return True
    if (tags.get('highway') == 'crossing' or tags.get('footway') == 'crossing'
            or tags.get('cycleway') == 'crossing'):
        if tags.get('crossing') in ('marked', 'zebra', 'uncontrolled', 'traffic_signals'):
            return True
        if tags.get('crossing') in ('no', 'unmarked'):
            return False
    return None


def assemble_rings(members, ways, nodes):
    """Join endpoint-connected member ways by role, preserving open fragments."""
    rings = []
    for role in ('outer', 'inner'):
        remaining = {m['ref']: ways[m['ref']]['nodes'] for m in members
                     if m['type'] == 'way' and m.get('role', 'outer') in (role, '' if role == 'outer' else role)
                     and m['ref'] in ways and ways[m['ref']].get('nodes')}
        incidence = {}
        for key, ids in remaining.items():
            for endpoint in {ids[0], ids[-1]}:
                incidence.setdefault(endpoint, set()).add(key)
        for seed in sorted(remaining):
            if seed not in remaining:
                continue
            ids = list(remaining.pop(seed))
            while ids[-1] != ids[0]:
                choices = sorted(i for i in incidence.get(ids[-1], ()) if i in remaining)
                if not choices:
                    break
                part = remaining.pop(choices[0])
                ids.extend(part[1:] if part[0] == ids[-1] else list(reversed(part))[1:])
            missing = [i for i in ids if i not in nodes]
            rings.append({'role': role, 'closed': ids[0] == ids[-1] and not missing,
                          'node_ids': [f'osm/node/{i}' for i in ids],
                          'points': [(nodes[i]['lon'], nodes[i]['lat']) for i in ids if i in nodes],
                          'missing_nodes': missing})
    return rings


class OsmAdapter:
    def read(self, path):
        data = json.loads(path.read_text())
        elements = data.get('elements')
        if not isinstance(elements, list):
            raise ValueError('OSM JSON must contain an elements list')
        nodes = {e['id']: e for e in elements if e['type'] == 'node'}
        ways = {e['id']: e for e in elements if e['type'] == 'way'}
        available = {(e['type'], e['id']) for e in elements}
        features = []
        for e in sorted(elements, key=lambda e: (e['type'], e['id'])):
            tags = e.get('tags', {})
            roles = []
            if 'highway' in tags:
                roles.append('road')
            if 'railway' in tags:
                roles.append('rail')
            if 'building' in tags:
                roles.append('building')
            if 'landuse' in tags:
                roles.append('landuse')
            if not roles:
                roles.append('support' if e['type'] == 'node' and not tags else 'other')
            ids = e.get('nodes', [])
            points = [(nodes[i]['lon'], nodes[i]['lat']) for i in ids if i in nodes]
            if e['type'] == 'node':
                points = [(e['lon'], e['lat'])]
                ids = [e['id']]
            one_way = {'yes': 1, 'true': 1, '1': 1, '-1': -1}.get(tags.get('oneway'), 0)
            building_use = tags.get('building:use', tags.get('building', 'unknown'))
            if tags.get('shop'):
                building_use = 'retail'
            props = {'road_class': tags.get('highway'), 'name': tags.get('name', ''),
                     'lanes': number(tags.get('lanes')), 'lanes_forward': number(tags.get('lanes:forward')),
                     'lanes_backward': number(tags.get('lanes:backward')), 'oneway': one_way,
                     'ref': tags.get('ref', ''),
                     'median_surface': tags.get('median:surface', tags.get('divider:surface', '')),
                     'through_lanes': sum('through' in lane.split(';') for lane in tags.get('turn:lanes', '').split('|')) or None,
                     'turn_lanes': tags.get('turn:lanes', ''),
                     'turn_lanes_forward': tags.get('turn:lanes:forward', ''),
                     'turn_lanes_backward': tags.get('turn:lanes:backward', ''),
                     'speed_kmh': speed(tags.get('maxspeed')), 'service': tags.get('service', ''),
                     'track_grade': tags.get('tracktype', ''), 'surface': tags.get('surface', ''),
                     'access': tags.get('motorcar', tags.get('motor_vehicle', tags.get('vehicle', tags.get('access', '')))), 'footway': tags.get('footway', ''),
                     'oneway_confirmed': tags.get('oneway:source') == 'survey',
                     'roundabout': tags.get('junction') == 'roundabout',
                     'crossing_marked': marked_crossing(tags),
                     'junction_control': tags.get('highway') if tags.get('highway') in ('give_way', 'stop', 'traffic_signals') else None,
                     'control_direction': tags.get('direction', tags.get('traffic_signals:direction', '')),
                     'bus_lanes_forward': tags.get('bus:lanes:forward', tags.get('bus:lanes', '')),
                     'bus_lanes_backward': tags.get('bus:lanes:backward', ''),
                     'bridge': tags.get('bridge', 'no') != 'no', 'tunnel': tags.get('tunnel', 'no') != 'no',
                     'layer': number(tags.get('layer')) or 0, 'building_use': building_use,
                     'landuse': tags.get('landuse'), 'relation_type': tags.get('type', ''),
                     'closed': len(ids) > 3 and ids[0] == ids[-1],
                     'area': tags.get('area') == 'yes'}
            members = e.get('members', [])
            rings = assemble_rings(members, ways, nodes) if tags.get('type') == 'multipolygon' else []
            issues = ['missing_geometry_node'] if any(i not in nodes for i in ids) else []
            if any((m['type'], m['ref']) not in available for m in members):
                issues.append('missing_relation_member')
            if any(not r['closed'] for r in rings):
                issues.append('incomplete_multipolygon')
            features.append(SourceFeature(f"osm/{e['type']}/{e['id']}", roles, points,
                                          [f'osm/node/{i}' for i in ids], props, tags, members, rings, issues))
        return features
