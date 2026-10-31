// SPDX-License-Identifier: GPL-2.0-only

//! Deterministic physical supply and width/exclusion regressions on minimal subsystem fixtures.

use super::places::*;
use super::*;
use crate::simulation::buildings::allocator::BuildingAllocator;
use crate::simulation::network::{
    graph::RegionGraph,
    roadbed::{RoadbedSystem, fixtures},
    types::{EdgeClass, NodeType},
};
use godot::prelude::Vector3;

fn street(width: f32) -> (RegionGraph, RoadbedSystem, BuildingAllocator) {
    let mut graph = RegionGraph::new();
    let a = graph.add_node(Vector3::ZERO, NodeType::Junction);
    let b = graph.add_node(Vector3::new(120.0, 0.0, 0.0), NodeType::Junction);
    let edge = fixtures::connect(&mut graph, a, b, width, false);
    let e = graph.edge_mut(edge);
    e.fwd_lanes = 1;
    e.bkw_lanes = 1;
    e.speed_limit = crate::config::DEFAULT_URBAN_ROAD_SPEED_MS;
    graph.rebuild_intersection_clips();
    let mut roads = RoadbedSystem::new(RegionGraph::CHUNK_SIZE);
    roads.compile_dirty(&graph);
    (graph, roads, BuildingAllocator::new())
}

#[test]
fn deterministic_one_side_bays_preserve_road_products() {
    let (graph, roads, allocator) = street(7.0);
    let before = fixtures::ProductProbe::of(
        &graph,
        &roads,
        &crate::simulation::network::lanes::LaneSystem::new(),
    );
    let rule = graph.parking_rule(0);
    assert_eq!(rule.sides, [SideRule::None, SideRule::Unmarked]);
    let context = kerb_context(&graph, &allocator, 0);
    let a = kerb_bays(&graph, &roads, &allocator, 0, 1, rule, &context);
    assert!(!a.is_empty());
    assert_eq!(
        a,
        kerb_bays(&graph, &roads, &allocator, 0, 1, rule, &context)
    );
    assert!(kerb_bays(&graph, &roads, &allocator, 0, 0, rule, &context).is_empty());
    assert_eq!(
        fixtures::ProductProbe::of(
            &graph,
            &roads,
            &crate::simulation::network::lanes::LaneSystem::new()
        ),
        before
    );
    for bay in a {
        assert!((bay.position[2] + 2.35).abs() < 0.001);
    }
}

#[test]
fn width_driveway_and_class_exclusions() {
    let (mut graph, roads, allocator) = street(5.49);
    let rule = graph.parking_rule(0);
    let context = kerb_context(&graph, &allocator, 0);
    assert!(kerb_bays(&graph, &roads, &allocator, 0, 1, rule, &context).is_empty());
    let (g, r, a) = street(7.0);
    graph = g;
    let rule = graph.parking_rule(0);
    let mut context = kerb_context(&graph, &a, 0);
    context.driveways[1].push((30.0, 40.0));
    let bays = kerb_bays(&graph, &r, &a, 0, 1, rule, &context);
    assert!(!bays.is_empty());
    assert!(
        bays.iter()
            .all(|b| b.position[0] + 3.0 <= 30.0 || b.position[0] - 3.0 >= 40.0)
    );
    let mut both = rule;
    both.sides = [SideRule::Unmarked; 2];
    assert!(kerb_bays(&graph, &r, &a, 0, 1, both, &context).is_empty());
    for class in [EdgeClass::Bridge, EdgeClass::Tunnel] {
        graph.edge_mut(0).class = class;
        assert!(kerb_bays(&graph, &r, &a, 0, 1, rule, &context).is_empty());
        assert_eq!(
            ParkingRule::default_for(graph.edge(0)).sides,
            [SideRule::None; 2]
        );
    }
}

#[test]
fn crossings_and_junction_mouths_are_clear() {
    let graph = fixtures::t_junction(false);
    let allocator = BuildingAllocator::new();
    let mut roads = RoadbedSystem::new(RegionGraph::CHUNK_SIZE);
    roads.compile_dirty(&graph);
    for edge in 0..graph.edge_count() {
        let context = kerb_context(&graph, &allocator, edge);
        let rule = ParkingRule {
            sides: [SideRule::None, SideRule::Unmarked],
            ..Default::default()
        };
        let bays = kerb_bays(&graph, &roads, &allocator, edge, 1, rule, &context);
        let span = roads.span_piece(edge).unwrap();
        for bay in bays {
            let station = BuildingAllocator::project_point_to_polyline_s(
                &graph.edge(edge).physical_geometry,
                godot::prelude::Vector2::new(bay.position[0], bay.position[2]),
            );
            assert!(station - 3.0 >= span.sections()[0].s_m);
            assert!(station + 3.0 <= span.sections().last().unwrap().s_m);
            for end in [
                crate::simulation::network::roadbed::EdgeEnd::Start,
                crate::simulation::network::roadbed::EdgeEnd::End,
            ] {
                if let Some((lo, hi)) = span.crossing_strip_m(end) {
                    assert!(station + 3.0 <= lo - 5.0 || station - 3.0 >= hi + 5.0);
                }
            }
        }
    }
}

#[test]
fn fewer_mouths_and_direction_ties_choose_exactly_one_side() {
    let (mut graph, _, _) = street(7.0);
    for (mouths, side) in [([0, 1], 0), ([1, 0], 1), ([2, 2], 1)] {
        let mut rule = graph.parking_rule(0);
        rule.choose_side(mouths);
        assert_eq!(
            rule.sides
                .iter()
                .filter(|&&s| s == SideRule::Unmarked)
                .count(),
            1
        );
        assert_eq!(rule.sides[side], SideRule::Unmarked);
    }
    graph.set_node_type(0, NodeType::Border);
    assert_eq!(graph.parking_rule(0).sides, [SideRule::None; 2]);
    for (f, b) in [(2, 2), (2, 1), (1, 0)] {
        let e = graph.edge_mut(0);
        e.fwd_lanes = f;
        e.bkw_lanes = b;
        assert_eq!(ParkingRule::default_for(e).sides, [SideRule::None; 2]);
    }
}

#[test]
fn structure_footprints_exclude_whole_bays() {
    let (graph, roads, mut allocator) = street(7.0);
    let context = kerb_context(&graph, &allocator, 0);
    let structure = [
        godot::prelude::Vector2::new(-5.0, -4.0),
        godot::prelude::Vector2::new(125.0, -4.0),
        godot::prelude::Vector2::new(125.0, 4.0),
        godot::prelude::Vector2::new(-5.0, 4.0),
    ];
    let site = crate::simulation::buildings::allocator::BuildingSiteClient {
        footprint_world: structure.to_vec(),
        lot_footprint_world: structure,
        support_height_m: 0.0,
        surfaces: Vec::new(),
        structure_world: vec![structure],
        planting_world: Vec::new(),
    };
    allocator.site_cells.insert(0, &site);
    allocator.building_sites.push(site);
    assert!(
        kerb_bays(
            &graph,
            &roads,
            &allocator,
            0,
            1,
            graph.parking_rule(0),
            &context
        )
        .is_empty()
    );
}

#[test]
fn width_taper_sections_never_supply_bays() {
    let (mut graph, _, allocator) = street(7.0);
    let end = graph.add_node(Vector3::new(240.0, 0.0, 0.0), NodeType::Junction);
    fixtures::connect(&mut graph, 1, end, 10.0, false);
    graph.rebuild_intersection_clips();
    let mut roads = RoadbedSystem::new(RegionGraph::CHUNK_SIZE);
    roads.compile_dirty(&graph);
    let span = roads.span_piece(0).unwrap();
    let context = kerb_context(&graph, &allocator, 0);
    let bays = kerb_bays(
        &graph,
        &roads,
        &allocator,
        0,
        1,
        graph.parking_rule(0),
        &context,
    );
    assert!(!bays.is_empty());
    let varied: Vec<_> = span
        .sections()
        .iter()
        .filter(|s| {
            s.bands
                .iter()
                .filter(|b| b.kind == crate::simulation::network::roadbed::BandKind::Carriageway)
                .any(|b| b.lateral_start_m < -3.5 || b.lateral_end_m > 3.5)
        })
        .collect();
    // Width transitions are node-owned in this roadbed; no span bay enters the node.
    assert!(roads.node_piece(1).is_some());
    for bay in bays {
        for section in &varied {
            assert!((bay.position[0] - section.center_xz.x as f32).abs() > 3.0);
        }
    }
}
