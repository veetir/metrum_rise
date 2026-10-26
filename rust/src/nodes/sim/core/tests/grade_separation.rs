// SPDX-License-Identifier: GPL-2.0-only

//! ROAD-68 layered crossing transactions, preview identity, agents and undo.

use super::super::road_commit::RoadCommitRequest;
use super::super::road_edit_plan::RoadEditPlan;
use super::*;
use crate::simulation::economy::agents::{MODE_CAR, TRANSIT_NETWORK};
use crate::simulation::network::surface::RoadSurfaceSystem;
use std::time::Instant;

fn fixture() -> SimCore {
    let mut core = SimCore::new(WorldConfig {
        width_m: 512.0,
        height_m: 512.0,
        terrain_cell_m: 4.0,
        terrain_chunk_m: 128.0,
        ..WorldConfig::default()
    });
    core.create_blank_world_internal(512.0, 512.0, 4.0, 128.0, 0.0)
        .unwrap();
    core
}

fn build(
    core: &mut SimCore,
    points: Vec<Vector3>,
    lanes: i32,
    level: i16,
    plan: Option<std::sync::Arc<RoadEditPlan>>,
) {
    let result = core.commit_road(
        RoadCommitRequest {
            crossing_level: level,
            median: None,
            points,
            fwd_lanes: lanes,
            bkw_lanes: lanes,
            snap_to_existing_roads: true,
            edit_plan: plan,
        },
        Instant::now(),
        Default::default(),
    );
    assert!(
        result.committed,
        "{}: {}",
        result.rejection_reason, result.rejection
    );
}

fn lower() -> Vec<Vector3> {
    vec![Vector3::new(-80.0, 0.0, 0.0), Vector3::new(80.0, 0.0, 0.0)]
}
fn upper(height: f32) -> Vec<Vector3> {
    vec![
        Vector3::new(0.0, height, -80.0),
        Vector3::new(0.0, height, 80.0),
    ]
}

#[test]
fn crossing_preview_includes_structural_neighbors_outside_the_topology_snap_halo() {
    let mut core = SimCore::new(WorldConfig {
        width_m: 512.0,
        height_m: 512.0,
        terrain_cell_m: 4.0,
        terrain_chunk_m: 64.0,
        ..WorldConfig::default()
    });
    core.create_blank_world_internal(512.0, 512.0, 4.0, 64.0, 0.0)
        .unwrap();
    build(&mut core, lower(), 1, 0, None);
    // Outside the old topology bounds, but inside the compiler's structural halo. Although
    // unchanged and not actually crossed, its recompile dirties farther terrain chunks.
    build(
        &mut core,
        vec![
            Vector3::new(40.0, 0.0, 100.0),
            Vector3::new(40.0, 0.0, 220.0),
        ],
        1,
        0,
        None,
    );
    let retained = core.region_graph.edge(1).physical_geometry.clone();
    build(&mut core, upper(6.0), 1, 1, None);
    assert_eq!(core.region_graph.edge(1).physical_geometry, retained);
}

#[test]
fn preview_commit_and_reverse_build_order_keep_the_crossing_unrelated() {
    for foot in [false, true] {
        for upper_first in [false, true] {
            let mut core = fixture();
            let lanes = if foot { 0 } else { 1 };
            let height = if foot { 3.8 } else { 6.0 };
            if upper_first {
                build(&mut core, upper(height), 1, 1, None);
            }
            let retained_upper =
                upper_first.then(|| core.region_graph.edge(0).physical_geometry.clone());
            build(&mut core, lower(), lanes, 0, None);
            let retained = retained_upper
                .unwrap_or_else(|| core.region_graph.edge(0).physical_geometry.clone());
            if !upper_first {
                let plan = RoadEditPlan::compile_road(
                    &core,
                    RoadPreviewRequest {
                        crossing_level: 1,
                        median: None,
                        enqueued_at: None,
                        include_terrain: false,
                        request_id: 1,
                        surface_generation: core.road_tool_surface_generation,
                        points: upper(height),
                        fwd_lanes: 1,
                        bkw_lanes: 1,
                        snap_to_existing_roads: true,
                    },
                );
                assert_eq!(plan.road_status(&core), "ready");
                build(
                    &mut core,
                    upper(height),
                    1,
                    1,
                    Some(std::sync::Arc::new(plan)),
                );
            }
            assert_eq!(core.region_graph.edge_count(), 2);
            assert_eq!(core.region_graph.node_count(), 4);
            let high = usize::from(!upper_first);
            let low = 1 - high;
            assert_eq!(core.region_graph.edge(high).crossing_level, 1);
            assert_eq!(core.region_graph.edge(low).crossing_level, 0);
            assert_eq!(
                core.region_graph.edge(high).physical_geometry,
                upper(height)
            );
            assert!(
                core.region_graph
                    .edge(low)
                    .physical_geometry
                    .iter()
                    .all(|p| p.y == 0.0 && p.z == 0.0)
            );
            assert_eq!(core.region_graph.edge(0).physical_geometry, retained);
            assert!(
                core.transit_network
                    .road_surface
                    .published_generation_matches_source()
            );
            assert!(core.undo_action_internal());
            assert_eq!(core.region_graph.edge_count(), 1);
        }
    }
}

#[test]
fn stale_preview_layer_is_rebuilt_and_insufficient_clearance_is_atomic() {
    let mut core = fixture();
    build(&mut core, lower(), 1, 0, None);
    let request = |height, level| RoadPreviewRequest {
        crossing_level: level,
        median: None,
        enqueued_at: None,
        include_terrain: false,
        request_id: 1,
        surface_generation: core.road_tool_surface_generation,
        points: upper(height),
        fwd_lanes: 1,
        bkw_lanes: 1,
        snap_to_existing_roads: true,
    };
    let refused = RoadEditPlan::compile_road(&core, request(5.0, 1));
    assert_eq!(
        refused.validation().invalid_reason,
        "grade_separation_clearance"
    );
    let plan = std::sync::Arc::new(RoadEditPlan::compile_road(&core, request(6.0, 1)));
    let history = core.undo_stack.len();
    let result = core.commit_road(
        RoadCommitRequest {
            crossing_level: 1,
            median: None,
            points: upper(5.0),
            fwd_lanes: 1,
            bkw_lanes: 1,
            snap_to_existing_roads: true,
            edit_plan: Some(plan.clone()),
        },
        Instant::now(),
        Default::default(),
    );
    assert!(!result.committed);
    assert_eq!(result.rejection_reason, "grade_separation_clearance");
    assert_eq!(core.undo_stack.len(), history);
    assert_eq!(core.region_graph.edge_count(), 1);
    build(&mut core, upper(6.0), 1, 2, Some(plan));
    assert_eq!(core.region_graph.edge(1).crossing_level, 2);
}

#[test]
fn in_place_edit_and_undo_keep_agents_on_their_own_roads_and_refuse_a_lowered_deck() {
    let mut core = fixture();
    build(&mut core, lower(), 1, 0, None);
    build(&mut core, upper(6.0), 1, 1, None);
    let mut agents = Vec::new();
    for edge in 0..2 {
        let lane = core.transit_network.lane_system.edge_lanes[&edge]
            .iter()
            .copied()
            .find(|&id| {
                let lane = &core.transit_network.lane_system.lanes[id];
                lane.lane_type == LaneType::Vehicle && lane.is_fwd
            })
            .unwrap();
        let start = core.region_graph.edge(edge).start_node;
        let agent = core
            .agents
            .spawn_border_arrival_agent(usize::MAX, start, 0.0, 0.0);
        core.agents.agents.transit[agent] = TRANSIT_NETWORK;
        core.agents.agents.transit_mode[agent] = MODE_CAR;
        core.agents.agents.current_edge[agent] = edge;
        core.agents.agents.current_lane_id[agent] = lane;
        core.agents.agents.lane_distance[agent] = 60.0;
        agents.push((agent, edge));
    }
    assert!(
        core.change_road_cross_section(0, 2, 2, None),
        "{:?}",
        core.last_road_rejection
    );
    assert!(core.undo_action_internal());
    assert_eq!(core.region_graph.edge(0).width, 7.0);
    assert_eq!(core.region_graph.edge(1).crossing_level, 1);
    for (agent, edge) in agents {
        let lane = core.agents.agents.current_lane_id[agent];
        assert!(core.transit_network.lane_system.edge_lanes[&edge].contains(&lane));
    }
    let node = core.region_graph.edge(1).start_node;
    let history = core.undo_stack.len();
    assert!(!core.move_network_node_internal(node as i32, Vector3::new(0.0, 0.0, -80.0)));
    assert_eq!(core.region_graph.node(node).pos.y, 6.0);
    assert_eq!(core.undo_stack.len(), history);
}

#[test]
fn raised_crossing_connects_to_ordinary_straight_approaches_at_authored_endpoints() {
    let mut core = fixture();
    build(&mut core, lower(), 1, 0, None);
    for (start, end) in [(-160.0, -80.0), (80.0, 160.0)] {
        build(
            &mut core,
            vec![Vector3::new(0.0, 0.0, start), Vector3::new(0.0, 0.0, end)],
            1,
            0,
            None,
        );
    }
    build(
        &mut core,
        vec![
            Vector3::new(0.0, 0.0, -80.0),
            Vector3::new(0.0, 6.0, -40.0),
            Vector3::new(0.0, 6.0, 40.0),
            Vector3::new(0.0, 0.0, 80.0),
        ],
        1,
        1,
        None,
    );
    let graph = &core.region_graph;
    let start = graph.edge(1).start_node;
    let end = graph.edge(2).end_node;
    assert!(
        core.transit_network
            .cch_graph
            .find_path(start, end, usize::MAX, usize::MAX, graph, TransitFlags::CAR)
            .is_some()
    );
    assert!(
        core.transit_network
            .cch_graph
            .find_path(
                start,
                graph.edge(0).start_node,
                usize::MAX,
                usize::MAX,
                graph,
                TransitFlags::CAR
            )
            .is_none()
    );
}

#[test]
fn source_ground_approach_to_a_deck_terminal_keeps_mixed_earthworks() {
    let mut core = fixture();
    build(&mut core, upper(6.0), 1, 1, None);
    let raw = vec![
        Vector3::new(0.0, 0.0, -200.0),
        Vector3::new(0.0, 0.0, -140.0),
        Vector3::new(0.0, 6.0, -80.0),
    ];
    let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
        &raw,
        &core.heightmap,
        &core.region_graph,
        &core.transit_network.road_surface,
        true,
        0,
    );
    assert_eq!(prepared.class, EdgeClass::Standard);
    assert_eq!(prepared.points, raw);
    build(&mut core, raw, 1, 0, None);
    assert_eq!(core.region_graph.edge(1).class, EdgeClass::Standard);
    assert_eq!(
        core.region_graph.edge(0).start_node,
        core.region_graph.edge(1).end_node
    );
}
