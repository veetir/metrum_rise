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
    return f, map_role(f, 'road')


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
            result = map_role(feature('r', [(0, 0), (10, 0)], road_class=kind), 'road')
            self.assertEqual((result['action'], result['fwd_lanes'], result['bkw_lanes']), ('footpath', 0, 0))
        self.assertTrue(table())

    def test_unsupported_features_have_reasons(self):
        for kind in ('motorway', 'motorway_link', 'steps', 'corridor', 'construction', 'service', 'track'):
            result = map_role(feature('r', [(0, 0), (10, 0)], road_class=kind), 'road')
            self.assertEqual(result['action'], 'skip')
            self.assertTrue(result['reason'])
        self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], roundabout=True), 'road')['reason'], 'roundabout_ROAD_62')
        self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], bridge=True), 'road')['reason'], 'grade_separation_deferred')

    def test_service_and_track_policy(self):
        self.assertEqual(map_role(feature('a', [(0, 0), (10, 0)], road_class='service', service='alley'), 'road')['action'], 'road')
        self.assertEqual(map_role(feature('t', [(0, 0), (10, 0)], road_class='track', track_grade='grade1'), 'road')['action'], 'footpath')
        self.assertEqual(map_role(feature('p', [(0, 0), (10, 0)], road_class='service', service='parking_aisle'), 'road')['action'], 'skip')

    def test_directional_and_invalid_lanes(self):
        for oneway in (1, -1):
            result = map_role(feature('r', [(0, 0), (10, 0)], oneway=oneway, lanes=2), 'road')
            self.assertEqual((result['fwd_lanes'], result['bkw_lanes']), (2, 0))
        f = feature('r', [(0, 0), (10, 0)], lanes=3, lanes_forward=2)
        result = map_role(f, 'road')
        self.assertEqual((result['fwd_lanes'], result['bkw_lanes']), (2, 1))
        for lanes in (-1, 0, 2.5, 9):
            self.assertEqual(map_role(feature('r', [(0, 0), (10, 0)], lanes=lanes), 'road')['action'], 'skip')


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
        f = feature('r', [(0, 0), (20, 0)], oneway=-1)
        strokes, _ = build_strokes([road(f)], self.box)
        self.assertEqual(strokes[0]['points_xz'], [(20, 0), (0, 0)])
        self.assertEqual((strokes[0]['fwd_lanes'], strokes[0]['bkw_lanes']), (1, 0))

    def test_opposing_oneway_pieces_do_not_join(self):
        a = feature('a', [(0, 0), (20, 0)], ['a', 'joint'], oneway=1)
        b = feature('b', [(40, 0), (20, 0)], ['b', 'joint'], oneway=1)
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
