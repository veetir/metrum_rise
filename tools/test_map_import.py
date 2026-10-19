# SPDX-License-Identifier: GPL-2.0-only

"""Small synthetic importer regressions; no OSM extracts, Godot or network needed."""

import copy
import json
from pathlib import Path
import random
import sqlite3
import struct
import tempfile
import unittest

from tools.map_import.cli import validate_outcome
from tools.map_import.geometry import build_strokes, clip_line, point_segment_distance, radius_audit, simplify
from tools.map_import.mapping import map_role, table
from tools.map_import.conflate import collapse_roundabouts, conflate, prepare_streets, snap_endpoints, prune_stubs
from tools.map_import.water import WaterMask
from tools.map_import.model import SourceFeature
from tools.map_import.normalise import normalise
from tools.map_import.osm import OsmAdapter, assemble_rings, number, speed
from tools.map_import.projection import tm35fin
from tools.map_import.report import build_report
from tools.map_import.terrain import WorldTransform, dem_bounds, load_transform
from tools.map_import.zoning import PolygonIndex, frontage_intents


def feature(identifier, points, ids=None, road_class='residential', **properties):
    return {'id': identifier, 'roles': ['road'], 'points': points,
            'node_ids': ids or [f'{identifier}/{i}' for i in range(len(points))],
            'properties': {'road_class': road_class, 'name': '', 'oneway': 0, 'speed_kmh': None,
                           'layer': 0, 'lanes': None, 'track_grade': '', **properties},
            'issues': [], 'rings': []}


def road(f):
    return f, map_role(f, 'road', include_park_paths=True)


def polygon(name, ring, role='landuse', use='residential', rings=None):
    return {'id': name, 'roles': [role], 'points': ring, 'issues': [], 'rings': rings or [],
            'properties': {'closed': True, 'landuse': use, 'building_use': use}}


def synthetic_tiff(path, east=100, north=200, crs=3067):
    entries = [(256, 4, [10]), (257, 4, [10]), (33550, 12, [2., 2., 0.]),
               (33922, 12, [0., 0., 0., east, north, 0.]),
               (34735, 3, [1, 1, 0, 1, 3072, 0, 1, crs])]
    offset = 8 + 2 + 12*len(entries) + 4
    directory, payload = [], b''
    for tag, kind, values in entries:
        raw = struct.pack('<' + {3: 'H', 4: 'I', 12: 'd'}[kind]*len(values), *values)
        storage = raw.ljust(4, b'\0') if len(raw) <= 4 else struct.pack('<I', offset+len(payload))
        directory.append(struct.pack('<HHI', tag, kind, len(values)) + storage)
        if len(raw) > 4:
            payload += raw
    path.write_bytes(b'II' + struct.pack('<HIH', 42, 8, len(entries)) + b''.join(directory) + b'\0'*4 + payload)


class ProjectionTest(unittest.TestCase):
    def test_known_tm35fin_points(self):
        # EPSG:3067 control values rounded to millimetres, independent of implementation.
        for lat, lon, east, north in ((60, 25, 388455.958, 6653097.435),
                                      (60, 24, 332705.179, 6655205.484),
                                      (65, 27, 500000., 7208454.582)):
            with self.subTest(lat=lat, lon=lon):
                e, n = tm35fin(lat, lon)
                self.assertAlmostEqual(e, east, delta=0.002)
                self.assertAlmostEqual(n, north, delta=0.002)
        self.assertEqual(tm35fin(0, 27), (500000, 0))

    def test_rejects_invalid_projection_domain(self):
        for point in ((90, 27), (60, float('nan')), (60, 150)):
            with self.assertRaises(ValueError):
                tm35fin(*point)

    def test_world_axes_and_normalisation(self):
        transform = WorldTransform(490000, 0, 510000, 20000, 10, 10)
        self.assertEqual(transform.world(490000, 20000), (-10000, -10000))
        self.assertEqual(transform.world(510000, 0), (10000, 10000))
        source = SourceFeature('custom', ['road'], [(27, 0)], ['n'], {}, {'opaque': 'kept'})
        result = normalise([source], transform)[0]
        self.assertEqual(result['points'], [(0, 10000)])
        self.assertEqual(result['source_tags'], {'opaque': 'kept'})

    def test_geotiff_coverage_and_world_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            synthetic_tiff(path/'tile.tif')
            self.assertEqual(dem_bounds(path), (100, 180, 120, 200))
            world = path/'world.sqlite'
            with sqlite3.connect(world) as conn:
                conn.executescript('CREATE TABLE world_definition_meta(width_m,height_m,terrain_cell_m,zone_cell_m,terrain_chunk_m);'
                                   'INSERT INTO world_definition_meta VALUES(20,20,10,10,20);'
                                   'CREATE TABLE world_terrain_chunks(chunk_x,chunk_z,width_samples,height_samples,source_height_blob_f32_le);')
            transform = load_transform(path, world)
            self.assertEqual(transform.world(100, 200), (-10, -10))
            with sqlite3.connect(world) as conn:
                conn.execute('UPDATE world_definition_meta SET width_m=10')
            with self.assertRaises(ValueError):
                load_transform(path, world)
            synthetic_tiff(path/'tile.tif', crs=4326)
            with self.assertRaises(ValueError):
                dem_bounds(path)


class AdapterTest(unittest.TestCase):
    def test_osm_preserves_semantics_and_missing_geometry(self):
        data = {'elements': [{'type': 'node', 'id': 1, 'lat': 60, 'lon': 27},
                             {'type': 'node', 'id': 2, 'lat': 60.001, 'lon': 27},
                             {'type': 'way', 'id': 3, 'nodes': [1, 2],
                              'tags': {'highway': 'residential', 'oneway': '-1', 'maxspeed': '30 mph', 'lanes': '2'}},
                             {'type': 'way', 'id': 4, 'nodes': [1, 99], 'tags': {'highway': 'path'}}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'synthetic.json'
            path.write_text(json.dumps(data))
            features = OsmAdapter().read(path)
        self.assertEqual(features[0].roles, ['support'])
        self.assertEqual(features[2].properties['oneway'], -1)
        self.assertAlmostEqual(features[2].properties['speed_kmh'], 48.28032)
        self.assertEqual(map_role(features[3].record(), 'road')['reason'], 'incomplete_geometry')
        self.assertIsNone(number('nan'))
        self.assertIsNone(speed('signals'))

    def test_car_access_overrides_general_access(self):
        data = {'elements': [{'type': 'node', 'id': 1, 'lat': 60, 'lon': 27},
                             {'type': 'node', 'id': 2, 'lat': 60.001, 'lon': 27},
                             {'type': 'way', 'id': 3, 'nodes': [1, 2],
                              'tags': {'highway': 'service', 'access': 'yes', 'motor_vehicle': 'no'}}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'synthetic.json'
            path.write_text(json.dumps(data))
            f = OsmAdapter().read(path)[-1].record()
        self.assertEqual(map_role(f, 'road')['reason'], 'restricted_access')

    def test_multipolygon_reversed_fragments_and_missing_members(self):
        nodes = {1: {'lon': 0, 'lat': 0}, 2: {'lon': 1, 'lat': 0}, 3: {'lon': 1, 'lat': 1}}
        ways = {1: {'nodes': [1, 2]}, 2: {'nodes': [3, 2]}, 3: {'nodes': [3, 1]}}
        members = [{'type': 'way', 'ref': i, 'role': 'outer'} for i in ways]
        result = assemble_rings(members, ways, nodes)
        self.assertEqual(len(result), 1)
        self.assertTrue(result[0]['closed'])
        del ways[3]
        self.assertFalse(assemble_rings(members, ways, nodes)[0]['closed'])


class MappingTest(unittest.TestCase):
    def test_class_mapping_and_footpath_lanes(self):
        for kind in ('primary', 'secondary', 'tertiary', 'residential', 'unclassified', 'living_street'):
            result = map_role(feature('r', [(0, 0), (10, 0)], road_class=kind), 'road')
            self.assertEqual((result['action'], result['fwd_lanes'], result['bkw_lanes']), ('road', 1, 1))
        for kind in ('footway', 'cycleway', 'path'):
            f = feature('r', [(0, 0), (10, 0)], road_class=kind)
            self.assertEqual(map_role(f, 'road')['reason'], 'paths_disabled')
            result = map_role(f, 'road', include_park_paths=True)
            self.assertEqual((result['action'], result['fwd_lanes'], result['bkw_lanes']), ('footpath', 0, 0))
        self.assertTrue(table())

    def test_unsupported_features_have_reasons(self):
        for kind in ('steps', 'corridor', 'construction', 'track'):
            result = map_role(feature('r', [(0, 0), (10, 0)], road_class=kind), 'road')
            self.assertEqual(result['action'], 'skip')
            self.assertTrue(result['reason'])
        self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], roundabout=True), 'road')['reason'], 'roundabout_ROAD_62')
        self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], bridge=True, layer=1), 'road')['action'], 'road')
        for kind in ('motorway', 'motorway_link'):
            self.assertEqual(map_role(feature('r', [(0, 0), (100, 0)], road_class=kind), 'road')['action'], 'road')

    def test_service_and_track_policy(self):
        self.assertEqual(map_role(feature('s', [(0, 0), (10, 0)], road_class='service'), 'road')['action'], 'road')
        for subtype in ('sidewalk', 'crossing'):
            self.assertEqual(map_role(feature('p', [(0, 0), (10, 0)], road_class='footway', footway=subtype), 'road')['reason'], 'road_includes_sidewalks')
        self.assertEqual(map_role(feature('a', [(0, 0), (10, 0)], road_class='service', service='alley'), 'road')['action'], 'road')
        self.assertEqual(map_role(feature('t', [(0, 0), (10, 0)], road_class='track', track_grade='grade1'), 'road', include_park_paths=True)['action'], 'footpath')
        self.assertEqual(map_role(feature('p', [(0, 0), (10, 0)], road_class='service', service='parking_aisle'), 'road')['action'], 'skip')

    def test_directional_and_invalid_lanes(self):
        for oneway in (1, -1):
            result = map_role(feature('r', [(0, 0), (10, 0)], road_class='primary', oneway=oneway, lanes=2), 'road')
            self.assertEqual((result['fwd_lanes'], result['bkw_lanes']), (2, 0))
        f = feature('r', [(0, 0), (10, 0)], road_class='primary', lanes=3, lanes_forward=2)
        result = map_role(f, 'road')
        self.assertEqual((result['fwd_lanes'], result['bkw_lanes']), (2, 1))
        for lanes in (-1, 0, 2.5, 9):
            self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], lanes=lanes), 'road')['action'], 'skip')

    def test_minor_roads_prefer_default_unless_oneway_confirmed(self):
        for kind in ('service', 'residential', 'unclassified', 'living_street', 'tertiary'):
            f = feature('r', [(0, 0), (50, 0)], road_class=kind, oneway=-1, lanes=3)
            self.assertEqual((map_role(f, 'road')['fwd_lanes'], map_role(f, 'road')['bkw_lanes']), (1, 1))
            f['properties']['oneway_confirmed'] = True
            self.assertEqual(map_role(f, 'road')['bkw_lanes'], 0)


class WaterBedTest(unittest.TestCase):
    def test_bed_plan_changes_only_wet_samples_inside_box(self):
        from tools.map_import.water_beds import prepare_initial
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sqlite'
            heights = [5.0]*9
            heights[4] = 4.95
            with sqlite3.connect(world) as c:
                c.executescript('CREATE TABLE world_definition_meta(width_m,height_m,terrain_cell_m,terrain_chunk_m);'
                                'INSERT INTO world_definition_meta VALUES(20,20,10,30);'
                                'CREATE TABLE world_terrain_chunks(chunk_x,chunk_z,width_samples,height_samples,source_height_blob_f32_le);'
                                'CREATE TABLE world_lake_fills(x,z,y);CREATE TABLE world_open_water_fills(x,z,y);')
                c.execute('INSERT INTO world_terrain_chunks VALUES(0,0,3,3,?)',(struct.pack('<9f',*heights),))
                c.execute('INSERT INTO world_lake_fills VALUES(0,0,101)')
            edits = [{'seed_xz':[0,0],'new_seed_xz':[0,0],'new_surface_m':99.03}]
            result = prepare_initial(world,(-5,-5,5,5),edits)
            self.assertEqual(result['samples_xzy'], [[0.0,0.0,97.0]])
            self.assertLess(result['radius_m'],10)

    def test_tiff_lzw_dictionary_and_clear_codes(self):
        from tools.map_import.source_dem import lzw
        bits = ''.join(format(code,'09b') for code in (256,65,66,258,260,257))
        bits += '0'*((-len(bits))%8)
        data = int(bits,2).to_bytes(len(bits)//8,'big')
        self.assertEqual(lzw(data),b'ABABABA')


class CurveTest(unittest.TestCase):
    def test_interpolates_survey_vertices_and_shares_tangents(self):
        from tools.map_import.curves import smooth
        from tools.map_import.geometry import distance
        points = [(0, 0), (40, 10), (70, 40), (80, 80)]
        cooked = smooth(points, .1)
        for p in points:
            self.assertIn(p, cooked)
        self.assertEqual(cooked, smooth(points, .1))
        i = cooked.index(points[1])
        a = (cooked[i][0]-cooked[i-1][0], cooked[i][1]-cooked[i-1][1])
        b = (cooked[i+1][0]-cooked[i][0], cooked[i+1][1]-cooked[i][1])
        self.assertGreater((a[0]*b[0]+a[1]*b[1])/(distance(a,(0,0))*distance(b,(0,0))), .999)
        self.assertEqual(smooth([(0, 0), (0, 0), (100, 0)]), [(0, 0), (100, 0)])

    def test_grade_change_splits_authoring_strokes(self):
        a = feature('a', [(0, 0), (40, 0)], ['a','joint'])
        b = feature('b', [(40, 0), (80, 0)], ['joint','b'], bridge=True, layer=1)
        self.assertEqual(len(build_strokes([road(a),road(b)],(-100,-100,100,100))[0]),2)


class StrokeTest(unittest.TestCase):
    box = (-1000, -1000, 1000, 1000)

    def test_join_reverse_and_preserve_provenance(self):
        a = feature('a', [(0, 0), (20, 0)], ['a', 'joint'])
        b = feature('b', [(40, 0), (20, 0)], ['b', 'joint'])
        result, _ = build_strokes([road(a), road(b)], self.box)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['source_ids'], ['a', 'b'])
        self.assertEqual(result[0]['length_m'], 40)

    def test_oneway_reverse_is_geometry_not_lane_swap(self):
        f = feature('r', [(0, 0), (20, 0)], oneway=-1, oneway_confirmed=True)
        strokes, _ = build_strokes([road(f)], self.box)
        self.assertEqual(strokes[0]['points_xz'], [(20, 0), (0, 0)])
        self.assertEqual((strokes[0]['fwd_lanes'], strokes[0]['bkw_lanes']), (1, 0))

    def test_opposing_oneway_pieces_do_not_join(self):
        a = feature('a', [(0, 0), (20, 0)], ['a', 'joint'], oneway=1, oneway_confirmed=True)
        b = feature('b', [(40, 0), (20, 0)], ['b', 'joint'], oneway=1, oneway_confirmed=True)
        self.assertEqual(len(build_strokes([road(a), road(b)], self.box)[0]), 2)

    def test_original_way_continuous_at_branch_and_junction_pinned(self):
        a = feature('a', [(-50, 0), (-20, 0), (0, 0), (20, 0), (50, 0)], ['a', 'b', 'j', 'c', 'd'])
        branch = feature('z', [(0, 0), (0, 40)], ['j', 'z'])
        strokes, _ = build_strokes([road(a), road(branch)], self.box, tolerance=100)
        trunk = strokes[0]
        self.assertIn((0, 0), trunk['points_xz'])
        self.assertEqual(len(strokes), 2)

    def test_join_compatible_street_through_branch(self):
        a = feature('a', [(-40, 0), (0, 0)], ['a', 'j'], name='Main')
        b = feature('b', [(0, 0), (40, 0)], ['j', 'b'], name='Main')
        c = feature('c', [(0, 0), (0, 40)], ['j', 'c'], name='Side')
        result, _ = build_strokes(list(map(road, (a, b, c))), self.box)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]['source_ids'], ['a', 'b'])

    def test_merge_uses_fixed_representative_without_drift(self):
        roads = [road(feature(str(i), [(x, 0), (x, 40)], [str(i), str(i)+'end']))
                 for i, x in enumerate((0, 1.5, 3))]
        _, audit = build_strokes(roads, self.box)
        self.assertTrue(audit['merged_nodes'])
        self.assertTrue(all(m['distance_m'] <= 2 for m in audit['merged_nodes']))
        self.assertFalse(any(m['node'] == '2' and m['representative'] == '0' for m in audit['merged_nodes']))

    def test_closed_loop_split(self):
        f = feature('loop', [(0, 0), (30, 0), (30, 30), (0, 30), (0, 0)], ['a', 'b', 'c', 'd', 'a'])
        strokes, _ = build_strokes([road(f)], self.box)
        self.assertEqual(len(strokes), 2)
        self.assertTrue(all(s['points_xz'][0] != s['points_xz'][-1] for s in strokes))
        self.assertEqual(sum(s['length_m'] for s in strokes), 120)

    def test_clip_reentry_and_source_ownership(self):
        result = clip_line([(-2, 0), (0, 0), (2, 0), (2, 2), (0, 0)], ['a', 'b', 'c', 'd'], (-1, -1, 1, 1))
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0], ([(-1, 0), (0, 0), (1, 0)], ['a', 'b']))
        self.assertEqual(result[1][1], ['d'])

    def test_simplification_error_bound_and_pins(self):
        points = [(i, (i % 7)*0.1) for i in range(1000)]
        reduced = simplify(points, 1, {points[500]})
        self.assertLess(len(reduced), 100)
        self.assertIn(points[500], reduced)
        self.assertIn(points[499], reduced)
        self.assertIn(points[501], reduced)
        for p in points:
            self.assertLessEqual(min(point_segment_distance(p, a, b) for a, b in zip(reduced, reduced[1:])), 1)

    def test_radius_risks_are_not_game_refusals(self):
        tight = {'id': 'tight', 'kind': 'road', 'fwd_lanes': 1, 'bkw_lanes': 1,
                 'points_xz': [(0, 0), (4, 0), (4, 1), (0, 1)]}
        warnings = radius_audit([tight])
        self.assertEqual(warnings[0]['status'], 'input_warning_not_game_refusal')
        self.assertLess(warnings[0]['input_radius_m'], warnings[0]['illegal_limit_m'])
        straight = {**tight, 'points_xz': [(0, 0), (20, 0), (40, 0)]}
        self.assertEqual(radius_audit([straight]), [])

    def test_deterministic_input_order_and_dedup(self):
        roads = [road(feature('a', [(0, 0), (20, 0)], ['a', 'j'])),
                 road(feature('b', [(20, 0), (40, 0)], ['j', 'b'])),
                 road(feature('dup', [(0, 50), (20, 50)])),
                 road(feature('dup2', [(0, 50), (20, 50)]))]
        expected = build_strokes(roads, self.box)
        random.Random(31).shuffle(roads)
        self.assertEqual(build_strokes(roads, self.box), expected)
        self.assertTrue(any(d['reason'] == 'duplicate_stroke' for d in expected[1]['discarded']))


class ConflationTest(unittest.TestCase):
    def test_path_policy_cannot_move_or_reorder_car_strokes(self):
        cars = [road(feature('a', [(0, 0), (100, 0)])), road(feature('b', [(100, 0), (200, 50)]))]
        path = road(feature('path', [(1.5, 0), (1.5, 100)], road_class='path'))
        box = (-500, -500, 500, 500)
        expected = prepare_streets(cars, box, 1)[0]
        actual = prepare_streets(cars+[path], box, 1)[0]
        self.assertEqual([s for s in actual if s['kind'] == 'road'], expected)

    def test_roundabout_approaches_remap_to_one_ordinary_junction(self):
        circle = feature('circle', [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)], ['a', 'b', 'c', 'd', 'a'], roundabout=True)
        fs = [feature('west', [(-100, 0), (0, 0)], ['w', 'a']),
              feature('east', [(20, 20), (100, 20)], ['c', 'e'])]
        roads = list(map(road, fs))
        decisions = {('circle', 'road'): map_role(circle, 'road')}
        audit = collapse_roundabouts([circle], roads, decisions)
        self.assertEqual(audit[0]['centre_xz'], (10, 10))
        self.assertEqual(roads[0][0]['node_ids'][-1], roads[1][0]['node_ids'][0])
        self.assertEqual(roads[0][0]['points'][-1], (10, 10))
        self.assertEqual(decisions['circle', 'road']['reason'], 'roundabout_as_ordinary_junction')

    def test_parallel_paths_drop_but_shortcut_survives(self):
        fs = [feature('r', [(0, 0), (200, 0)]),
              feature('side', [(0, 8), (200, 8)], road_class='cycleway'),
              feature('shortcut', [(100, 0), (100, 100)], road_class='path')]
        decisions = {(f['id'], 'road'): map_role(f, 'road', include_park_paths=True) for f in fs}
        result, audit = conflate([road(f) for f in fs], decisions)
        self.assertEqual([f['id'] for f, _ in result], ['r', 'shortcut'])
        self.assertEqual(audit['parallel_paths'][0]['source_ids'], ['side'])

    def test_split_carriageways_create_one_two_way_median(self):
        fs = [feature('a', [(0, 0), (100, 0)], road_class='primary', name='Main', oneway=1),
              feature('b', [(100, 12), (0, 12)], road_class='primary', name='Main', oneway=1)]
        result, audit = conflate([road(f) for f in fs], {})
        self.assertEqual(len(result), 1)
        f, m = result[0]
        self.assertTrue(all(p[1] == 6 for p in f['points']))
        self.assertEqual((m['fwd_lanes'], m['bkw_lanes']), (1, 1))
        self.assertEqual(len(audit['carriageways']), 2)
        self.assertEqual(conflate([road(f) for f in reversed(fs)], {}), (result, audit))
        strokes, _ = build_strokes(result, (-200, -200, 200, 200))
        self.assertEqual(strokes[0]['source_ids'], ['a', 'b'])

    def test_split_carriageways_preserve_asymmetric_lane_counts(self):
        fs = [feature('a', [(0, 0), (100, 0)], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('b', [(100, 12), (0, 12)], road_class='primary', name='Main', oneway=1, lanes=3)]
        result, _ = conflate([road(f) for f in fs], {})
        self.assertEqual(len(result), 1)
        self.assertEqual((result[0][1]['fwd_lanes'], result[0][1]['bkw_lanes']), (2, 3))

    def test_pair_directional_tags_and_reverse_oneway(self):
        fs = [feature('a', [(0, 0), (100, 0)], road_class='primary', name='Main', oneway=1, lanes_forward=2),
              feature('b', [(0, 12), (100, 12)], road_class='primary', name='Main', oneway=-1, lanes_backward=3)]
        result, _ = conflate(list(map(road, fs)), {})
        self.assertEqual(len(result), 1)
        self.assertEqual((result[0][1]['fwd_lanes'], result[0][1]['bkw_lanes']), (2, 3))
        self.assertEqual(result[0][1]['median_width_m'], 3.25)

    def test_median_width_styles_and_clamps(self):
        from tools.map_import.medians import median_section
        for separation, lanes, style, width in [(4, (1, 1), 1, .5),
                                                (5, (1, 1), 3, 1.5),
                                                (12, (1, 1), 2, 8.5),
                                                (40, (2, 2), 2, 12),
                                                (1, (1, 1), 1, .5)]:
            self.assertEqual(median_section(separation, *lanes),
                             {'median_style': style, 'median_width_m': width})
        self.assertEqual(median_section(10, 1, 1, paved=True)['median_width_m'], 3)

    def test_explicit_median_surface_context_selects_paving(self):
        fs = [feature('a', [(0, 0), (100, 0)], name='Main', oneway=1, median_surface='paving_stones'),
              feature('b', [(100, 10), (0, 10)], name='Main', oneway=1)]
        result, _ = conflate(list(map(road, fs)), {})
        self.assertEqual(result[0][1]['median_style'], 3)
        self.assertEqual(result[0][1]['median_width_m'], 3)

    def test_tertiary_pairs_merge_even_without_survey_override(self):
        fs = [feature('a', [(0, 0), (100, 0)], road_class='tertiary', name='Main', oneway=1, lanes=1),
              feature('b', [(100, 9), (0, 9)], road_class='tertiary', name='Main', oneway=1, lanes=1)]
        result, _ = conflate(list(map(road, fs)), {})
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][1]['median_width_m'], 5.5)
        strokes, _ = build_strokes(result, (-200, -200, 200, 200))
        self.assertEqual(strokes[0]['median_style'], 2)
        self.assertEqual(strokes[0]['median_width_m'], 5.5)

    def test_chain_pockets_and_layer_separation(self):
        fs = [feature('a', [(0, 0), (50, 0)], ['a0', 'a1'], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('a2', [(50, 0), (100, 0)], ['a1', 'a2'], road_class='primary', name='Main', oneway=1, lanes=3, through_lanes=2),
              feature('b', [(100, 12), (0, 12)], road_class='primary', name='Main', oneway=1, lanes=2)]
        result, _ = conflate(list(map(road, fs)), {})
        self.assertEqual(len(result), 2)
        self.assertTrue(all((m['fwd_lanes'], m['bkw_lanes']) == (2, 2) for _, m in result))
        fs[-1]['properties']['layer'] = 1
        result, audit = conflate(list(map(road, fs)), {})
        self.assertEqual(len(result), 3)
        self.assertTrue(all(a['status'] == 'fallback' for a in audit['carriageways']))

    def test_convergence_preserves_shared_tip_and_side_street(self):
        fs = [feature('a', [(0, 0), (25, -5), (100, -5), (125, 0)], ['tip0', 'a1', 'a2', 'tip1'], road_class='primary', name='Main', oneway=1),
              feature('b', [(125, 0), (100, 5), (25, 5), (0, 0)], ['tip1', 'b1', 'b2', 'tip0'], road_class='primary', name='Main', oneway=1),
              feature('side', [(25, 5), (25, 100)], ['b2', 's'])]
        result, _ = conflate(list(map(road, fs)), {})
        median = next(f for f, m in result if m.get('median_style'))
        self.assertEqual(median['points'][0], (0, 0))
        self.assertEqual(median['points'][-1], (125, 0))
        side = next(f for f, _ in result if f['id'] == 'side')
        self.assertAlmostEqual(side['points'][0][1], 0, delta=.6)

    def test_bridge_provenance_does_not_elevate_whole_corridor(self):
        fs = [feature('a', [(0, 0), (100, 0)], ['a0', 'a1'], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('a2', [(100, 0), (120, 0)], ['a1', 'a2'], road_class='primary', name='Main', oneway=1, lanes=2, bridge=True, layer=1),
              feature('b', [(100, 12), (0, 12)], ['b1', 'b0'], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('b2', [(120, 12), (100, 12)], ['b2', 'b1'], road_class='primary', name='Main', oneway=1, lanes=2, bridge=True, layer=1)]
        result, _ = conflate(list(map(road, fs)), {})
        ground = next(f for f, _ in result if f['id'] == 'a')
        self.assertEqual(ground['source_ids'], ['a', 'b'])
        from tools.map_import.curves import prepare_curves
        strokes, _ = build_strokes(result, (-200, -200, 200, 200))
        prepare_curves(strokes, fs)
        self.assertEqual(sum(s['bridge'] for s in strokes), 1)

    def test_prisma_divided_arms_share_a_straight_throat(self):
        from tools.map_import.medians import simplify_prisma
        fs = [feature('main', [(-30, -100), (-14, -50), (2, -10), (8, 20), (20, 60), (35, 100)], name='Savilahdentie'),
              feature('west', [(-150, 0), (-50, 0), (-10, 2)], name='Sarastuskaari')]
        result, audit = simplify_prisma(list(map(road, fs)), (0, 0), {})
        main, west = [f for f, _ in result]
        self.assertIn((0, 0), main['points'])
        self.assertEqual(west['points'][-1], (0, 0))
        self.assertIn('junction/prisma', main['node_ids'])
        axis = audit[-1]['axis_xz']
        for x, z in main['points'][1:-1]:
            self.assertAlmostEqual(x*axis[1]-z*axis[0], 0, delta=.001)

    def test_near_miss_inserts_shared_junction(self):
        roads = [road(feature('a', [(0, 0), (100, 0)])), road(feature('b', [(50, 5), (50, 80)]))]
        audit = snap_endpoints(roads)
        self.assertEqual(len(audit), 1)
        self.assertIn((50, 0), roads[0][0]['points'])
        self.assertEqual(roads[1][0]['points'][0], (50, 0))

    def test_short_dead_end_pruned_but_through_path_kept(self):
        fs = [feature('a', [(0, 0), (100, 0)]), feature('b', [(50, 0), (50, 30)], road_class='path')]
        strokes, audit = build_strokes(list(map(road, fs)), (-100, -100, 200, 200))
        result = prune_stubs(strokes, audit['discarded'])
        self.assertEqual([s['kind'] for s in result], ['road'])

    def test_water_bilinear_width_and_trim(self):
        depth = [0.0]*121
        depth[5*11+5] = 2.0
        mask = WaterMask(11, 11, 10, depth)
        self.assertEqual(mask.at((0, 0)), 2)
        self.assertEqual(mask.at((5, 0)), 1)
        s = {'id': 'wet', 'kind': 'road', 'fwd_lanes': 1, 'bkw_lanes': 1,
             'points_xz': [(-40, 0), (40, 0)], 'length_m': 80, 'source_ids': ['s']}
        clipped, audit = mask.clip([s])
        self.assertEqual(len(clipped), 2)
        self.assertGreaterEqual(audit[0]['removed_m'], 20)
        dry = {**s, 'points_xz': [(-40, 30), (40, 30)]}
        self.assertEqual(mask.clip([dry]), ([dry], []))

    def test_water_inclusive_world_edges_and_duplicate_vertices(self):
        mask = WaterMask(3, 3, 10, [1.0]*9)
        self.assertEqual(mask.at((10, 10)), 1)
        self.assertEqual(mask.at((10.1, 10)), 0)
        self.assertFalse(mask.safe((0, 0), (0, 0), 5))

    def test_authored_water_flood_and_edge_escape_rules(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'world.sqlite'
            samples = [0.1 if x in (0, 4) or z in (0, 4) else 0 for z in range(5) for x in range(5)]
            with sqlite3.connect(path) as conn:
                conn.executescript('CREATE TABLE world_definition_meta(width_m,height_m,terrain_cell_m,terrain_chunk_m);'
                                   'INSERT INTO world_definition_meta VALUES(40,40,10,50);'
                                   'CREATE TABLE world_terrain_chunks(chunk_x,chunk_z,width_samples,height_samples,source_height_blob_f32_le);'
                                   'CREATE TABLE world_lake_fills(world_x,world_z,surface_elevation_m);'
                                   'CREATE TABLE world_open_water_fills(world_x,world_z,surface_elevation_m);'
                                   'INSERT INTO world_lake_fills VALUES(0,0,1);')
                conn.execute('INSERT INTO world_terrain_chunks VALUES(0,0,5,5,?)', (struct.pack('<25f', *samples),))
            mask = WaterMask.load(path)
            self.assertEqual(mask.at((0, 0)), 1)
            self.assertEqual(sum(d > 0 for d in mask.depth), 9)
            with sqlite3.connect(path) as conn:
                conn.execute('UPDATE world_lake_fills SET surface_elevation_m=3')
            self.assertFalse(any(WaterMask.load(path).depth))
            with sqlite3.connect(path) as conn:
                conn.execute('INSERT INTO world_open_water_fills VALUES(0,0,3)')
            self.assertEqual(sum(d > 0 for d in WaterMask.load(path).depth), 25)

    def test_short_through_path_is_pruned_after_water_clipping(self):
        path = {'source_ids': ['p'], 'kind': 'footpath', 'length_m': 60, 'points_xz': [(0, 0), (60, 0)]}
        roads = [{'source_ids': ['r'], 'kind': 'road', 'length_m': 100, 'points_xz': [(0, 0), (0, 100)]},
                 {'source_ids': ['s'], 'kind': 'road', 'length_m': 100, 'points_xz': [(60, 0), (60, 100)]}]
        discarded = []
        self.assertEqual(prune_stubs([path, *roads], discarded), roads)
        self.assertEqual(discarded[0]['reason'], 'short_standalone_path')


class ZoningReportTest(unittest.TestCase):
    def test_polygon_holes_and_specific_building_masks(self):
        outer = [(-100, -100), (100, -100), (100, 100), (-100, 100)]
        hole = [(-10, -10), (10, -10), (10, 10), (-10, 10)]
        land = polygon('land', [], rings=[{'role': 'outer', 'closed': True, 'points': outer},
                                         {'role': 'inner', 'closed': True, 'points': hole}])
        apartment = polygon('apt', [(30, 30), (40, 30), (40, 40), (30, 40)], 'building', 'apartments')
        features = [land, apartment]
        decisions = {(f['id'], r): map_role(f, r) for f in features for r in f['roles']}
        index = PolygonIndex(features, decisions, (-200, -200, 200, 200))
        self.assertIsNone(index.at((0, 0)))
        self.assertEqual(index.at((35, 35))['action'], 'skip')
        self.assertEqual(index.at((70, 70))['action'], 'residential')

    def test_frontage_samples_and_no_false_acceptance(self):
        land = polygon('land', [(-100, -100), (100, -100), (100, 100), (-100, 100)])
        r = feature('road', [(-80, 0), (80, 0)])
        features = [land, r]
        decisions = {(f['id'], role): map_role(f, role) for f in features for role in f['roles']}
        strokes, preparation = build_strokes([road(r)], (-100, -100, 100, 100))
        zones = frontage_intents(features, decisions, strokes, (-100, -100, 100, 100), 10)
        self.assertTrue(zones)
        plan = {'strokes': strokes, 'zoning': zones, 'source_sha256': 'source', 'world_sha256': 'world',
                'world_transform': {}, 'target_box_xz': []}
        report = build_report(features, decisions, plan, preparation)
        self.assertEqual(report['totals']['road']['attempted'], 0)
        self.assertIsNone(report['totals']['road']['accepted'])
        self.assertIsNone(report['save_path'])
        self.assertEqual(len(report['features']), 2)
        outcome = {'roads': [{'id': strokes[0]['id'], 'status': 'refused', 'reason': 'junction_arm_angle'}],
                   'zoning': [], 'parcels': [], 'saved': False, 'complete': False}
        applied = build_report(features, decisions, plan, preparation, outcome)
        self.assertEqual(applied['totals']['road']['refused_by_reason'], {'junction_arm_angle': 1})
        self.assertEqual(next(f['status'] for f in applied['features'] if f['id'] == 'road'), 'refused')

    def test_street_report_keeps_missing_names_and_final_refusals(self):
        from tools.map_import.road_report import analyse
        missing = feature('lost', [(0, 10), (50, 10)], name='Lost Street')
        outside = feature('outside', [(1000, 0), (1100, 0)], name='Outside')
        stroke = {'id': 's', 'road_class': 'primary', 'kind': 'road', 'names': ['Main'],
                  'length_m': 100, 'source_ids': ['main']}
        outcome = {'roads': [{'id': 's', 'status': 'partial', 'accepted_length_m': 80,
                             'attempts': [{'reason': 'road_tight_curve'}, {'reason': ''}],
                             'failed_pieces': [{'reason': 'road_tight_curve'}]}], 'car_components': 1}
        result = analyse({'strokes': [stroke], 'target_box_xz': [-100, -100, 100, 100]}, outcome, [missing, outside])
        self.assertEqual([(r['street'], r['status']) for r in result['named_streets']],
                         [('Lost Street', 'missing'), ('Main', 'partly placed')])
        self.assertEqual(result['remaining_refusals'], {'road_tight_curve': 1})
        self.assertEqual(result['by_class']['primary']['placed_m'], 80)

    def test_saved_network_counts_canonical_ids_and_medians(self):
        from tools.map_import.road_report import save_stats
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'save.sqlite'
            with sqlite3.connect(path) as conn:
                conn.executescript('CREATE TABLE network_edges(start_node,end_node,physical_length,fwd_lanes,bkw_lanes,median_style);'
                                   'INSERT INTO network_edges VALUES(0,1,100,1,1,2),(1,2,200,1,1,0),(4,5,80,0,0,0);'
                                   'CREATE TABLE time_state(speed_multiplier);INSERT INTO time_state VALUES(0);')
            stats = save_stats(path)
            self.assertEqual((stats['car_components'], stats['car_edges'], stats['car_nodes']), (1, 2, 3))
            self.assertAlmostEqual(stats['car_km'], .3)
            self.assertEqual(stats['median_styles'][2]['edges'], 1)

    def test_outcome_hash_duplicate_and_completeness_checks(self):
        plan = {'strokes': [{'id': 's'}], 'zoning': []}
        outcome = {'schema_version': 1, 'plan_sha256': 'hash', 'complete': True, 'roads': [], 'zoning': [], 'parcels': []}
        with self.assertRaises(ValueError):
            validate_outcome(outcome, plan, 'hash')
        outcome['roads'] = [{'id': 's', 'status': 'accepted', 'reason': ''}]
        validate_outcome(outcome, plan, 'hash')
        with self.assertRaises(ValueError):
            validate_outcome(outcome, plan, 'other')
        outcome['roads'].append(copy.deepcopy(outcome['roads'][0]))
        with self.assertRaises(ValueError):
            validate_outcome(outcome, plan, 'hash')


if __name__ == '__main__':
    unittest.main()
