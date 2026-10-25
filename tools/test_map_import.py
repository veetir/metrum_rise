# SPDX-License-Identifier: GPL-2.0-only

"""Small synthetic importer regressions; no OSM extracts, Godot or network needed."""

import copy
import json
import math
from pathlib import Path
import random
import sqlite3
import struct
import tempfile
import unittest

from tools.map_import.cli import validate_outcome
from tools.map_import.geometry import build_strokes, clip_line, distance, point_segment_distance, radius_audit, simplify
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


class RoadFeatureIntegrationTest(unittest.TestCase):
    def test_through_roads_establish_profiles_before_connecting_links(self):
        for road_class in ('motorway', 'trunk', 'primary', 'secondary', 'tertiary'):
            main = road(feature('main', [(0, 0), (100, 0)], road_class=road_class))[1]
            link = road(feature('link', [(0, 0), (20, 20)], road_class=road_class+'_link', oneway=1, lanes=1))[1]
            self.assertLess(main['priority'], link['priority'])
            self.assertEqual((link['fwd_lanes'], link['bkw_lanes']), (1, 0))

    def test_bus_permissions_follow_the_saved_edge_direction(self):
        from unittest.mock import patch
        from tools.map_import.features import prepare
        nodes = {0: [0, 0, 0], 1: [60, 0, 0]}
        source = {'id': 'main', 'source_ids': ['main'], 'points_xz': [(0, 0), (60, 0)],
                  'fwd_lanes': 3, 'bkw_lanes': 2, 'bus_masks': [4, 0]}
        for reverse in (False, True):
            a, b = (1, 0) if reverse else (0, 1)
            edges = [{'id': 0, 'a': a, 'b': b, 'fwd': 2 if reverse else 3,
                      'bkw': 3 if reverse else 2, 'points': [nodes[a], nodes[b]]}]
            with patch('tools.map_import.features.read_save', return_value=(nodes, edges)), \
                 patch('tools.map_import.features.prepare_rules', return_value={'actions': [], 'source_audit': []}):
                result = prepare('unused', {'strokes': [source]}, [])
            self.assertEqual(result['actions'], [
                {'kind': 'bus_lanes', 'edge': 0, 'masks': [0, 4] if reverse else [4, 0]}])

    def test_feature_actions_use_the_source_arm_and_preserve_unsupported_controls(self):
        from unittest.mock import patch
        from tools.map_import.features import prepare
        nodes = {0: [-60, 0, 0], 1: [0, 0, 0], 2: [60, 0, 0], 3: [0, 0, 60]}
        source = {'source_ids': ['osm/way/main'], 'fwd_lanes': 2, 'bkw_lanes': 2, 'bus_masks': [2, 2]}
        edges = [{'id': 0, 'a': 0, 'b': 1, 'fwd': 2, 'bkw': 2, 'points': [nodes[0], nodes[1]], 'source': source},
                 {'id': 1, 'a': 1, 'b': 2, 'fwd': 1, 'bkw': 1, 'points': [nodes[1], nodes[2]], 'source': {'source_ids': ['osm/way/main']}},
                 {'id': 2, 'a': 1, 'b': 3, 'fwd': 1, 'bkw': 1, 'points': [nodes[1], nodes[3]], 'source': {'source_ids': ['osm/way/side']}}]
        main = feature('osm/way/main', [(-60, 0), (-10, 0), (60, 0)], ['a', 'osm/node/mark', 'b'])
        mark = feature('osm/node/mark', [(-10, 0)], crossing_marked=True)
        control = feature('osm/node/signal', [(0, 0)], junction_control='traffic_signals')
        with patch('tools.map_import.features.read_save', return_value=(nodes, edges)), \
             patch('tools.map_import.features.source_index', return_value=None), \
             patch('tools.map_import.features.attribute'), \
             patch('tools.map_import.features.prepare_rules', return_value={'actions': [], 'source_audit': []}):
            result = prepare('unused', {}, [main, mark, control])
        self.assertEqual(result['actions'], [
            {'kind': 'bus_lanes', 'edge': 0, 'masks': [2, 2]},
            {'kind': 'crosswalk', 'node': 1, 'edge': 0, 'marked': True, 'source_id': 'osm/node/mark'}])
        self.assertTrue(any(r.get('reason') == 'TRAFFIC_06_authored_control_API_pending' for r in result['source_audit']))

    def test_crossing_way_only_authors_at_shared_source_nodes(self):
        from collections import defaultdict
        from tools.map_import.features import crossing_intents
        way = feature('osm/way/zebra', [(0, -10), (0, 0), (0, 10)], ['a', 'cross', 'b'], road_class='footway', crossing_marked=True)
        membership = defaultdict(set, {'a': {way['id']}, 'cross': {way['id'], 'osm/way/main'}, 'b': {way['id']}})
        intents, audit = crossing_intents([way], membership)
        self.assertEqual(len(intents), 1)
        self.assertEqual(intents[0]['points'], [(0, 0)])
        self.assertEqual(intents[0]['road_source_ids'], {'osm/way/main'})
        self.assertEqual(audit, [])

    def test_crossing_markings_override_signal_and_type(self):
        from tools.map_import.osm import marked_crossing
        self.assertTrue(marked_crossing({'highway': 'crossing', 'crossing': 'zebra'}))
        self.assertTrue(marked_crossing({'highway': 'footway', 'footway': 'crossing', 'crossing': 'marked'}))
        self.assertFalse(marked_crossing({'highway': 'cycleway', 'cycleway': 'crossing', 'crossing': 'unmarked'}))
        self.assertFalse(marked_crossing({'highway': 'crossing', 'crossing': 'traffic_signals', 'crossing:markings': 'no'}))
        self.assertIsNone(marked_crossing({'highway': 'traffic_signals'}))
        self.assertIsNone(marked_crossing({'highway': 'crossing'}))
        self.assertTrue(marked_crossing({'crossing:markings': 'ladder'}))

    def test_source_extent_survives_provenance_union_and_curve_fitting(self):
        from tools.map_import.curves import prepare_curves
        ground = feature('ground', [(0, 0), (40, 0)], ['a', 'joint'])
        bridge = feature('bridge', [(40, 0), (60, 0)], ['joint', 'b'], bridge=True, layer=1)
        ground['source_ids'] = ['ground', 'bridge']
        strokes, _ = build_strokes([road(ground), road(bridge)], (-100, -100, 100, 100))
        prepare_curves(strokes, [ground, bridge])
        self.assertEqual([(s['bridge'], s['layer']) for s in strokes], [(True, 1), (False, 0)])

    def test_carriageway_chains_end_at_structure_boundaries(self):
        from tools.map_import.medians import carriageway_chains
        a = feature('a', [(0, 0), (40, 0)], ['a', 'joint'], road_class='primary', name='main', oneway=1, lanes=3)
        b = feature('b', [(40, 0), (60, 0)], ['joint', 'b'], road_class='primary', name='main', oneway=1, lanes=2, bridge=True, layer=1)
        self.assertEqual(len(carriageway_chains([road(a), road(b)])), 2)

    def test_short_bridge_keeps_the_retained_direction_and_exact_source_joint(self):
        from tools.map_import.medians import merge_carriageways
        fs = [
            feature('a', [(0, 0), (-100, 80)], ['a', 'joint-a'], road_class='primary', name='main', oneway=1, lanes=3),
            feature('b', [(-92, 88), (8, 8)], ['joint-b', 'b'], road_class='primary', name='main', oneway=1, lanes=3),
            feature('c', [(-100, 80), (-110, 110)], ['joint-a', 'c'], road_class='primary', name='main', oneway=1, lanes=3, bridge=True, layer=1),
            feature('d', [(-102, 118), (-92, 88)], ['d', 'joint-b'], road_class='primary', name='main', oneway=1, lanes=3, bridge=True, layer=1)]
        result, _ = merge_carriageways(list(map(road, fs)), {})
        self.assertEqual(len(result), 2)
        a, b = [f for f, _ in result]
        self.assertEqual(a['points'][0], b['points'][-1])
        self.assertEqual(a['node_ids'][0], b['node_ids'][-1])

    def test_university_survey_keeps_six_lanes_and_bus_permissions(self):
        from tools.map_import.survey import apply_road_surveys
        from tools.map_import.medians import merge_carriageways
        a = feature('osm/way/1119987974', [(0, 0), (40, 0)], road_class='primary', name='main', oneway=1, lanes=2, bridge=True, layer=1)
        b = feature('osm/way/1119987976', [(40, 12), (0, 12)], road_class='primary', name='main', oneway=1, lanes=2, bridge=True, layer=1)
        roads = [road(a), road(b)]
        self.assertEqual(len(apply_road_surveys(roads)), 2)
        result, _ = merge_carriageways(roads, {})
        self.assertEqual(len(result), 1)
        m = result[0][1]
        self.assertEqual((m['fwd_lanes'], m['bkw_lanes']), (3, 3))
        self.assertEqual((m['median_style'], m['median_width_m'], m['bus_masks']), (3, 2.0, [4, 4]))

    def test_university_underpass_keeps_motor_road_and_separate_path(self):
        from tools.map_import.survey import apply_road_surveys
        roads = [road(feature('osm/way/'+i, [(0, 0), (100, 0)], road_class=kind))
                 for i, kind in [('1119987950', 'unclassified'), ('1119987951', 'unclassified'),
                                 ('987262495', 'cycleway')]]
        apply_road_surveys(roads)
        for f, m in roads[:2]:
            self.assertEqual((m['action'], m['fwd_lanes'], m['bkw_lanes']), ('road', 1, 1))
            self.assertEqual(f['properties']['road_class'], 'unclassified')
            self.assertEqual((f['properties']['tunnel'], f['properties']['layer']), (False, 0))
        self.assertEqual(roads[2][1]['action'], 'footpath')
        self.assertTrue(roads[2][0]['properties']['survey_underpass'])

    def test_untagged_motor_faces_are_detected_without_source_ids(self):
        from tools.map_import.roundabouts import detect_roundabouts
        for split in (False, True):
            points = [(round(15*math.cos(-i*math.tau/24), 3),
                       round(15*math.sin(-i*math.tau/24), 3)) for i in range(25)]
            ids = [str(i % 24) for i in range(25)]
            ring = [feature('arbitrary', points, ids, road_class='tertiary', oneway=1)]
            if split:
                ring = [feature(f'part/{i}', points[i:i+7], ids[i:i+7],
                                road_class='tertiary', oneway=1) for i in range(0, 24, 6)]
            arms = [feature(f'arm/{i}', [points[i], (points[i][0]*4, points[i][1]*4)],
                            [ids[i], f'outer/{i}']) for i in (0, 6, 12, 18)]
            neutral = copy.deepcopy(ring+arms)
            evidence = detect_roundabouts(ring+arms)
            self.assertEqual([r['status'] for r in evidence], ['detected'])
            self.assertTrue(all(f['properties']['roundabout'] for f in ring))
            self.assertFalse(any(f['properties'].get('roundabout') for f in neutral))
            self.assertEqual(collapse_roundabouts(ring+arms, list(map(road, arms)), {})[0]['status'], 'planned')

    def test_cycle_recognition_excludes_blocks_parking_and_turning_loops(self):
        from tools.map_import.roundabouts import detect_roundabouts
        points = [(15*math.cos(-i*math.tau/24), 15*math.sin(-i*math.tau/24)) for i in range(24)]
        points.append(points[0])
        ids = [str(i % 24) for i in range(25)]
        for cls, direction, arms in [('service', 1, 4), ('tertiary', 0, 4), ('tertiary', 1, 2)]:
            ring = feature('ring', points, ids, road_class=cls, oneway=direction)
            roads = [ring]+[feature(f'arm/{i}', [points[i*6], (points[i*6][0]*4, points[i*6][1]*4)],
                                    [ids[i*6], f'outer/{i}']) for i in range(arms)]
            evidence = detect_roundabouts(roads)
            self.assertFalse(ring['properties'].get('roundabout'))
            if arms == 2:
                self.assertEqual(evidence[0]['reason'], 'turning_loop_fewer_than_three_external_arms')
        block = feature('block', [(0, 0), (0, 20), (20, 20), (20, 0), (0, 0)],
                        ['a', 'b', 'c', 'd', 'a'], oneway=1)
        self.assertEqual(detect_roundabouts([block]), [])

    def test_osm_circular_and_implied_roundabout_direction(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'osm.json'
            path.write_text(json.dumps({'elements': [
                {'type': 'way', 'id': 1, 'tags': {'highway': 'secondary', 'junction': 'roundabout'}},
                {'type': 'way', 'id': 2, 'tags': {'highway': 'secondary', 'junction': 'circular', 'oneway': 'yes'}}]}))
            features = OsmAdapter().read(path)
            self.assertTrue(all(f.properties['roundabout'] for f in features))
            self.assertTrue(all(f.properties['oneway'] == 1 for f in features))

    def test_unnamed_service_loops_do_not_become_through_streets(self):
        from tools.map_import.service import filter_service_circulation
        main = feature('main', [(0, 0), (50, 0), (100, 0)], ['a', 'm', 'b'], name='Street')
        loop = feature('loop', [(0, 0), (0, 40), (100, 40), (100, 0)],
                       ['a', 'l', 'r', 'b'], road_class='service')
        spur = feature('spur', [(50, 0), (50, -40)], ['m', 's'], road_class='service')
        decisions = {}
        result, audit = filter_service_circulation(list(map(road, [main, loop, spur])), decisions)
        self.assertEqual([f['id'] for f, _ in result], ['main', 'spur'])
        self.assertEqual(audit[0]['source_ids'], ['loop'])
        self.assertEqual(decisions['loop', 'road']['reason'], 'local_service_circulation')
        other = feature('other', [(100, 0), (150, 0)], ['b', 'end'], name='Other street')
        result, audit = filter_service_circulation(list(map(road, [main, loop, other])), {})
        self.assertEqual(len(result), 3)
        self.assertEqual(audit, [])

    def test_bridge_profile_spans_dem_valley_and_precedes_approaches(self):
        from tools.map_import.structures import prepare_structure_profiles
        from unittest.mock import Mock
        bridge = dict(id='b', kind='road', priority=0, bridge=True, layer=1,
                      points_xz=[(0, 0), (25, 0), (50, 0), (75, 0), (100, 0)])
        ground = dict(id='a', kind='road', priority=0, bridge=False, layer=0,
                      points_xz=[(-100, 0), (0, 0)])
        terrain = Mock()
        terrain.at.side_effect = lambda p: {0: 90, 100: 92}.get(p[0], 80)
        result = prepare_structure_profiles([ground, bridge], terrain)
        self.assertIs(result[0], bridge)
        self.assertEqual([p[1] for p in bridge['points_xyz']], [90, 90.5, 91, 91.5, 92])
        self.assertEqual(bridge['endpoint_levels'][0], [0, 1])
        self.assertEqual(ground['endpoint_levels'][-1], [0, 1])
        self.assertNotIn('points_xyz', ground)
        self.assertEqual(terrain.at.call_count, 2)

    def test_two_lane_ring_explicitly_records_single_lane_limit(self):
        f = feature('ring', [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)], ['a', 'b', 'c', 'd', 'a'], roundabout=True, lanes=2)
        audit = collapse_roundabouts([f], [], {})
        self.assertEqual(audit[0]['diameter_m'], 23.5)
        self.assertEqual(audit[0]['source_lanes'], 2)
        self.assertEqual(audit[0]['imported_lanes'], 1)
        self.assertIn('spiral_exits', audit[0]['remaining'])

    def test_roundabout_staging_discards_only_the_replaced_inside_footprint(self):
        circle = feature('circle', [(10, 0), (0, 10), (-10, 0), (0, -10), (10, 0)],
                         ['a', 'b', 'c', 'd', 'a'], roundabout=True)
        arm = feature('arm', [(10, 0), (11, 0), (30, 0), (60, 0)], ['a', 'near', 'out', 'far'])
        roads = [road(arm)]
        collapse_roundabouts([circle], roads, {})
        self.assertEqual(arm['points'], [(0, 0), (30, 0), (60, 0)])
        self.assertEqual(arm['node_ids'][1:], ['out', 'far'])

    def test_open_and_cycle_rings_do_not_move_car_approaches(self):
        open_ring = feature('open', [(0, 0), (20, 0), (20, 20)], ['a', 'b', 'c'], roundabout=True)
        cycle = feature('cycle', [(0, 0), (20, 0), (20, 20), (0, 0)], ['a', 'b', 'c', 'a'], roundabout=True, road_class='cycleway')
        arm = feature('arm', [(-50, 0), (0, 0)], ['x', 'a'])
        roads = [road(arm)]
        original = copy.deepcopy(roads)
        self.assertEqual(collapse_roundabouts([cycle], roads, {}), [])
        self.assertEqual(collapse_roundabouts([open_ring], roads, {})[0]['status'], 'unsupported')
        self.assertEqual(roads, original)


class PocketLaneTest(unittest.TestCase):
    def test_bus_intent_removes_only_a_pocket_with_the_same_source_base(self):
        from tools.map_import.pockets import bus_section_pocket_removals
        edge = {'id': 7, 'source': {'bus_masks': [4, 4], 'fwd_lanes': 3, 'bkw_lanes': 3},
                'tapers': {1: {'pocket': True, 'from_fwd': 3, 'from_bkw': 3}}}
        action, = bus_section_pocket_removals([edge])
        self.assertEqual((action['edge'], action['at_start'], action['left'], action['right']),
                         (7, True, False, False))
        edge['tapers'][1]['from_bkw'] = 2
        self.assertEqual(bus_section_pocket_removals([edge]), [])
        edge['tapers'][1]['from_bkw'] = 3
        edge['source']['bus_masks'] = [0, 0]
        self.assertEqual(bus_section_pocket_removals([edge]), [])

    def test_compact_left_storage_fits_the_exact_clips_in_either_direction(self):
        from tools.map_import.pockets import fit_compact_left_pocket
        for at_start in (True, False):
            near = 0 if at_start else 1
            edge = {'length_m': 68.139946, 'start_clip': 15.3285, 'end_clip': 27.256926,
                    'tapers': {1: {'length_m': 27.256926, 'pocket': False}}}
            if not at_start:
                edge['start_clip'], edge['end_clip'] = edge['end_clip'], edge['start_clip']
                edge['tapers'] = {0: edge['tapers'][1]}
            action = {'at_start': at_start, 'left': True, 'right': False,
                      'pocket_m': 70, 'taper_m': 20}
            fit_compact_left_pocket(action, edge)
            self.assertEqual(action['pocket_m'], 12)
            self.assertGreaterEqual(action['taper_m'], 12)
            self.assertLessEqual(action['pocket_m'] + action['taper_m'] + 15.3285 + 27.256926 + .5,
                                 edge['length_m'])
            self.assertEqual(edge['tapers'][1-near]['length_m'], 27.256926)

    def test_compact_fit_does_not_reinterpret_a_protected_near_taper_or_widened_road(self):
        from tools.map_import.pockets import fit_compact_left_pocket
        edge = {'length_m': 68, 'tapers': {0: {'length_m': 28, 'pocket': False}}}
        for left, right in ((True, False), (True, True), (False, True)):
            action = {'at_start': True, 'left': left, 'right': right, 'pocket_m': 70, 'taper_m': 20}
            fit_compact_left_pocket(action, edge)
            self.assertEqual((action['pocket_m'], action['taper_m']), (70, 20))

    def test_directional_turn_tags_keep_travel_order(self):
        from tools.map_import.pockets import turn_lanes
        p = {'oneway': 0, 'lanes_forward': 3, 'lanes_backward': 2,
             'turn_lanes_forward': 'left|through|right',
             'turn_lanes_backward': 'through;left|right'}
        rules = turn_lanes(p)
        self.assertEqual([r['direction'] for r in rules], [1, -1])
        self.assertEqual(rules[1]['lanes'], [('through', 'left'), ('right',)])
        self.assertFalse(any(r['reason'] for r in rules))

    def test_osm_whitelist_keeps_untagged_approach_connections(self):
        from unittest.mock import patch
        from tools.map_import.pockets import prepare_rules
        edges = [
            {'id': 0, 'a': 0, 'b': 1, 'fwd': 1, 'bkw': 0, 'points': [[-50, 0, 0], [0, 0, 0]]},
            {'id': 1, 'a': 1, 'b': 2, 'fwd': 1, 'bkw': 0, 'points': [[0, 0, 0], [50, 0, 0]]},
            {'id': 2, 'a': 1, 'b': 3, 'fwd': 1, 'bkw': 1, 'points': [[0, 0, 0], [0, 0, -50]]},
        ]
        audit = {'source_audit': [{'edge': 0, 'junction': 1, 'at_start': False,
                                  'lanes': [('through',)], 'source_gap_m': 0, 'source_id': 'osm/test'}]}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'city.sqlite'
            with sqlite3.connect(path) as c:
                c.execute('CREATE TABLE saved_lanes(lane_id INTEGER PRIMARY KEY,edge_id,lane_idx,node_id,from_lane_id,to_lane_id)')
                c.executemany('INSERT INTO saved_lanes VALUES (?,?,?,?,?,?)', [
                    (10, 0, 0, -1, -1, -1), (11, 1, 0, -1, -1, -1),
                    (12, 2, 0, -1, -1, -1), (13, 2, -1, -1, -1, -1),
                    (20, -1, 0, 1, 10, 11), (21, -1, 0, 1, 10, 12), (22, -1, 0, 1, 13, 11),
                ])
            with patch('tools.map_import.pockets.read_save', return_value=({}, edges)), patch('tools.map_import.pockets.prepare', return_value=audit):
                result = prepare_rules(path, {}, [])
        self.assertEqual(result['actions'][0]['rules'], [(0, 0, 1, 0), (2, -1, 1, 0)])
        self.assertEqual(result['actions'][0]['restricted_native_connectors'], 1)

    def test_turn_sector_matches_game_boundary(self):
        from tools.map_import.pockets import movement
        self.assertEqual(movement((1, 0), (1, 1)), 'through')
        self.assertEqual(movement((1, 0), (0, 1)), 'right')
        self.assertEqual(movement((1, 0), (0, -1)), 'left')

    def test_reverse_one_way_and_bad_count_are_explicit(self):
        from tools.map_import.pockets import turn_lanes
        rules = turn_lanes({'oneway': -1, 'lanes': 4, 'turn_lanes': 'left|through|through|right'})
        self.assertEqual(rules[0]['direction'], -1)
        self.assertEqual(rules[0]['reason'], '')
        rules = turn_lanes({'oneway': 1, 'lanes': 3, 'turn_lanes': 'through|right'})
        self.assertEqual(rules[0]['reason'], 'tag_lane_count_mismatch')


class ConnectivityDirectionTest(unittest.TestCase):
    def test_one_way_terminal_is_distinct_from_physical_disconnect(self):
        from tools.map_import.connectivity import directed_components, topology
        edges = [{'id': 0, 'a': 0, 'b': 1, 'fwd': 1, 'bkw': 1},
                 {'id': 1, 'a': 1, 'b': 2, 'fwd': 1, 'bkw': 0}]
        self.assertEqual(len(topology({}, edges)[0]), 1)
        self.assertEqual(directed_components(edges)['strong_component_sizes'], [2, 1])
        edges[1]['bkw'] = 1
        self.assertEqual(directed_components(edges)['strong_components'], 1)


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

    def test_minor_roads_preserve_explicit_oneway_and_default_untagged_roads(self):
        for kind in ('service', 'residential', 'unclassified', 'living_street', 'tertiary'):
            f = feature('r', [(0, 0), (50, 0)], road_class=kind, oneway=-1, lanes=3)
            self.assertEqual((map_role(f, 'road')['fwd_lanes'], map_role(f, 'road')['bkw_lanes']), (3, 0))
            f['properties']['oneway'] = 0
            self.assertEqual((map_role(f, 'road')['fwd_lanes'], map_role(f, 'road')['bkw_lanes']), (1, 1))


class WaterBedTest(unittest.TestCase):
    def test_beds_cover_complete_edited_fill_and_preserve_other_lakes(self):
        from tools.map_import.water_beds import prepare_initial
        with tempfile.TemporaryDirectory() as directory:
            world, output = (Path(directory)/name for name in ('world.sqlite', 'beds.sqlite'))
            heights = [5.0]*35
            for i in (15, 16, 17):
                heights[i] = 4.95
            heights[19] = 4.9  # Separate, unedited lake.
            with sqlite3.connect(world) as c:
                c.executescript('CREATE TABLE world_definition_meta(width_m,height_m,terrain_cell_m,terrain_chunk_m);'
                                'INSERT INTO world_definition_meta VALUES(60,40,10,30);'
                                'CREATE TABLE world_terrain_chunks(chunk_x,chunk_z,width_samples,height_samples,source_height_blob_f32_le);'
                                'CREATE TABLE world_lake_fills(x,z,y);CREATE TABLE world_open_water_fills(x,z,y);')
                for z in range(2):
                    for x in range(3):
                        width, height = min(3, 7-x*3), min(3, 5-z*3)
                        values = [heights[(z*3+row)*7+x*3+col] for row in range(height) for col in range(width)]
                        c.execute('INSERT INTO world_terrain_chunks VALUES(?,?,?,?,?)',
                                  (x, z, width, height, struct.pack('<'+'f'*len(values), *values)))
                c.execute('INSERT INTO world_lake_fills VALUES(-10,0,99.5)')
                c.execute('INSERT INTO world_lake_fills VALUES(20,0,98.03)')
            original = world.read_bytes()
            edits = [{'seed_xz': [-10,0], 'new_seed_xz': [-10,0], 'new_surface_m': 99.03}]
            result = prepare_initial(world, edits, output)
            self.assertEqual(result['samples'], 3)  # Includes both sides of a road crop at x=-5.
            self.assertEqual(world.read_bytes(), original)
            fills = [{'kind': 'auto', 'seed_xz': [-10,0], 'surface_m': 99.03},
                     {'kind': 'lake', 'seed_xz': [20,0], 'surface_m': 98.03}]
            before, after = (WaterMask.load(p, fills) for p in (world, output))
            self.assertEqual([v > .001 for v in before.depth], [v > .001 for v in after.depth])
            for i in range(35):
                if i in (15, 16, 17):
                    self.assertAlmostEqual(after.heights[i]+after.depth[i], 99.03, places=4)
                    self.assertGreater(after.depth[i], 2)
                else:
                    self.assertEqual(after.heights[i], before.heights[i])
                    self.assertEqual(after.depth[i], before.depth[i])

    def test_city_repair_preserves_level_mask_and_all_other_rows(self):
        from tools.map_import.water_beds import repair_save
        with tempfile.TemporaryDirectory() as directory:
            source, output, repeated = (Path(directory)/name for name in ('source.sqlite', 'fixed.sqlite', 'again.sqlite'))
            heights = (4.95, 5.0, 4.9, 4.85, 4.95)
            depths = (.03, 0, .03, 2.03, .001)
            with sqlite3.connect(source) as c:
                c.executescript('CREATE TABLE terrain_state(width,height,height_blob_f32_le);'
                                'CREATE TABLE water_state(width,height,baseline_depth_blob_f32_le);'
                                "CREATE TABLE roads(id,geometry);INSERT INTO roads VALUES(7, x'1234');")
                c.execute('INSERT INTO terrain_state VALUES(5,1,?)', (struct.pack('<5f', *heights),))
                c.execute('INSERT INTO water_state VALUES(5,1,?)', (struct.pack('<5f', *depths),))
            original = source.read_bytes()
            edits = [{'new_surface_m': 99.03}]
            self.assertEqual(repair_save(source, output, edits)['samples'], 1)
            self.assertEqual(source.read_bytes(), original)
            with sqlite3.connect(output) as c:
                new_heights = struct.unpack('<5f', c.execute('SELECT height_blob_f32_le FROM terrain_state').fetchone()[0])
                new_depths = struct.unpack('<5f', c.execute('SELECT baseline_depth_blob_f32_le FROM water_state').fetchone()[0])
                self.assertEqual(c.execute('SELECT * FROM roads').fetchall(), [(7, b'\x12\x34')])
            for i in range(5):
                self.assertEqual(new_depths[i] > .001, struct.unpack('<f', struct.pack('<f', depths[i]))[0] > .001)
                self.assertAlmostEqual(new_heights[i]*20+new_depths[i], heights[i]*20+depths[i], places=4)
                if i:
                    self.assertEqual(new_heights[i], struct.unpack('<f', struct.pack('<f', heights[i]))[0])
                    self.assertEqual(new_depths[i], struct.unpack('<f', struct.pack('<f', depths[i]))[0])
            self.assertEqual(repair_save(output, repeated, edits)['samples'], 0)
            with self.assertRaises(ValueError):
                repair_save(source, source, edits)
            with self.assertRaises(ValueError):
                repair_save(source, output, edits)

    def test_tiff_lzw_dictionary_and_clear_codes(self):
        from tools.map_import.source_dem import lzw
        bits = ''.join(format(code,'09b') for code in (256,65,66,258,260,257))
        bits += '0'*((-len(bits))%8)
        data = int(bits,2).to_bytes(len(bits)//8,'big')
        self.assertEqual(lzw(data),b'ABABABA')


class CurveTest(unittest.TestCase):
    def test_a_divided_corridor_keeps_one_tangent_across_its_name_boundary(self):
        from tools.map_import.curves import name_boundary_controls, smooth
        base = {'kind': 'road', 'road_class': 'primary', 'fwd_lanes': 2, 'bkw_lanes': 2, 'median_style': 2, 'median_width_m': 4}
        west = {**base, 'id': 'west', 'names': ['West'], 'points_xz': [(-100, 40), (-12, 6), (-2, 1), (0, 0)]}
        east = {**base, 'id': 'east', 'names': ['East'], 'points_xz': [(0, 0), (1, -2), (8, -12), (50, -50)]}
        joined = name_boundary_controls([west, east])
        a = smooth(joined['west'][0], .01, joined['west'][1])
        b = smooth(joined['east'][0], .01, joined['east'][1])
        u, v = tuple(-x for x in a[-2]), b[1]
        self.assertGreater(sum(x*y for x,y in zip(u,v))/math.hypot(*u)/math.hypot(*v), .99999)
        self.assertEqual((a[0], b[-1]), ((-100, 40), (50, -50)))
        self.assertNotIn((-2, 1), joined['west'][0])
        self.assertNotIn((1, -2), joined['east'][0])
        self.assertEqual(name_boundary_controls([east, west]), joined)
        self.assertEqual(name_boundary_controls([west, east], [(0,0)]), {})
        third = {**base, 'id': 'third', 'names': ['Third'], 'points_xz': [(0,0), (0,100)]}
        self.assertEqual(name_boundary_controls([west,east,third]), {})

    def test_explicit_straight_throat_survives_curve_fitting(self):
        from tools.map_import.curves import smooth, straight_throat
        source = [(-100, -40), (-45, 0), (0, 0), (42, 0), (110, 30)]
        points, tangents = straight_throat(source, (0, 0), 75)
        cooked = smooth(points, fixed_tangents=tangents)
        self.assertEqual((cooked[0], cooked[-1]), (source[0], source[-1]))
        throat = [p for p in cooked if -75 <= p[0] <= 75]
        self.assertGreater(len(throat), 30)
        self.assertTrue(all(p[1] == 0 for p in throat))
        self.assertIn((0, 0), cooked)
        self.assertEqual(straight_throat(source, (99, 99), 75), (source, {}))

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

    def test_roundabout_approaches_stage_atomic_player_replacement(self):
        circle = feature('circle', [(0, 0), (20, 0), (20, 20), (0, 20), (0, 0)], ['a', 'b', 'c', 'd', 'a'], roundabout=True)
        fs = [feature('west', [(-100, 0), (0, 0)], ['w', 'a']),
              feature('east', [(20, 20), (100, 20)], ['c', 'e'])]
        roads = list(map(road, fs))
        decisions = {('circle', 'road'): map_role(circle, 'road')}
        audit = collapse_roundabouts([circle], roads, decisions)
        self.assertEqual(audit[0]['centre_xz'], (10, 10))
        self.assertEqual(roads[0][0]['node_ids'][-1], roads[1][0]['node_ids'][0])
        self.assertEqual(roads[0][0]['points'][-1], (10, 10))
        self.assertEqual(decisions['circle', 'road']['reason'], 'roundabout_player_replacement')

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

    def test_section_joint_prefers_paired_midline_over_an_unshifted_chain_end(self):
        from tools.map_import.medians import merge_carriageways
        fs = [feature('a', [(100, -5), (102, -5), (104, -5), (150, -5), (400, -5)], ['join', 'a2', 'a4', 'a50', 'end'], road_class='primary', name='Main', oneway=1),
              feature('a_return', [(400, 5), (160, 5)], road_class='primary', name='Main', oneway=1),
              feature('b', [(0, -5), (100, -5)], ['start', 'join'], road_class='primary', name='Main', oneway=1, survey_cross_section='section'),
              feature('b_return', [(100, 5), (0, 5)], road_class='primary', name='Main', oneway=1, survey_cross_section='section')]
        result, _ = merge_carriageways(list(map(road, fs)), {})
        by_id = {f['id']: f for f, _ in result}
        self.assertEqual(by_id['a']['points'][0], (100, 0))
        self.assertEqual(by_id['b']['points'][-1], (100, 0))
        self.assertGreaterEqual(distance(by_id['a']['points'][1], (100, 0)), 8.25)
        self.assertNotIn((102, -5), by_id['a']['points'])
        self.assertNotIn((104, -5), by_id['a']['points'])
        self.assertEqual(merge_carriageways(list(map(road, reversed(fs))), {})[0], result)
        # A different named axis retains the source junction authority; its paired
        # sample must not move the other road's physical crossing onto that axis.
        for f in fs[2:]:
            f['properties']['name'] = 'Other'
        result, _ = merge_carriageways(list(map(road, fs)), {})
        by_id = {f['id']: f for f, _ in result}
        self.assertEqual(by_id['a']['points'][0], (100, -5))
        self.assertEqual(by_id['b']['points'][-1], (100, -5))

    def test_paired_sections_keep_their_local_directional_lane_counts(self):
        from tools.map_import.medians import merge_carriageways
        fs = [feature('a1', [(0, 0), (50, 0)], ['a0', 'aj'], road_class='primary', name='Main', oneway=1, lanes=3),
              feature('a2', [(50, 0), (100, 0)], ['aj', 'ae'], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('b1', [(50, 12), (0, 12)], ['bj', 'b0'], road_class='primary', name='Main', oneway=1, lanes=1),
              feature('b2', [(100, 12), (50, 12)], ['be', 'bj'], road_class='primary', name='Main', oneway=1, lanes=2)]
        result, _ = merge_carriageways(list(map(road, fs)), {})
        mapped = {f['id']: (m['fwd_lanes'], m['bkw_lanes']) for f, m in result}
        self.assertEqual(mapped, {'a1': (3, 1), 'a2': (2, 2)})

    def test_unpaired_one_lane_head_does_not_narrow_the_paired_carriageway(self):
        from tools.map_import.medians import merge_carriageways
        fs = [feature('a_head', [(-8, -10), (0, 0)], ['head', 'join'], road_class='primary', name='Main', oneway=1, lanes=1),
              feature('a_main', [(0, 0), (100, 0)], ['join', 'east'], road_class='primary', name='Main', oneway=1, lanes=2),
              feature('b', [(100, 10), (0, 10)], road_class='primary', name='Main', oneway=1, lanes=2)]
        result, audit = merge_carriageways(copy.deepcopy(list(map(road, fs))), {})
        mapped = {f['id']: m for f, m in result}
        self.assertEqual((mapped['a_main']['fwd_lanes'], mapped['a_main']['bkw_lanes']), (2, 2))
        self.assertEqual((mapped['a_head']['fwd_lanes'], mapped['a_head']['bkw_lanes']), (1, 0))
        self.assertTrue(all((a['fwd_lanes'], a['bkw_lanes']) == (2, 2) for a in audit))
        self.assertEqual(merge_carriageways(list(map(road, reversed(fs))), {}), (result, audit))

    def test_retained_carriageway_continues_across_a_road_name_and_axis_change(self):
        from tools.map_import.medians import merge_carriageways
        fs = [feature('a', [(-100, 0), (0, 0)], ['west_a', 'join_a'], road_class='primary', name='West', oneway=1),
              feature('b', [(0, 10), (-100, 10)], ['join_b', 'west_b'], road_class='primary', name='West', oneway=1),
              feature('c', [(0, 0), (60, -100)], ['join_a', 'north_a'], road_class='primary', name='North', oneway=1),
              feature('d', [(70, -100), (0, 10)], ['north_b', 'join_b'], road_class='primary', name='North', oneway=1)]
        result, audit = merge_carriageways(list(map(road, fs)), {})
        self.assertEqual([f['id'] for f, _ in result], ['a', 'c'])
        self.assertEqual(result[0][0]['points'][-1], result[1][0]['points'][0])
        self.assertEqual([a['status'] for a in audit], ['merged', 'suppressed', 'merged', 'suppressed'])
        self.assertEqual(merge_carriageways(list(map(road, reversed(fs))), {}), (result, audit))

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

    def test_projected_median_junction_is_pinned_before_curve_fitting(self):
        from tools.map_import.curves import prepare_curves
        fs = [feature('main', [(-100, -30), (0, 0), (100, 0), (200, 30)]),
              feature('side', [(50, 0), (50, 80)])]
        strokes, _, _ = prepare_streets(list(map(road, fs)), (-300, -300, 300, 300), 1)
        strokes = prepare_curves(strokes, fs)
        main = next(s for s in strokes if 'main' in s['source_ids'])
        side = next(s for s in strokes if 'side' in s['source_ids'])
        self.assertIn(tuple(side['points_xz'][0]), main['points_xz'])
        self.assertIn((50, 0), main['junctions_xz'])

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
