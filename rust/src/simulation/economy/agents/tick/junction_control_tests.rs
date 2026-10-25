// SPDX-License-Identifier: GPL-2.0-only

//! Automatic priority, stop and conflict-admission regressions.

use crate::simulation::economy::agents::AgentSystem;
use crate::simulation::economy::agents::{MODE_CAR, TRANSIT_NETWORK};
use crate::simulation::network::graph::Edge;
use crate::simulation::network::lanes::control::{ApproachControl, approach_control};
use crate::simulation::network::types::{NodeType, TransitFlags};
use crate::simulation::network::{TransitNetwork, graph::RegionGraph, lanes::LaneType};
use godot::prelude::{Vector2, Vector3};

fn fixture(major: bool, stop: bool) -> (TransitNetwork, RegionGraph, [usize; 4]) {
    let mut graph = RegionGraph::new();
    let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
    for (arm, pos) in [
        Vector3::new(-100.0, 0.0, 0.0),
        Vector3::new(100.0, 0.0, 0.0),
        Vector3::new(0.0, 0.0, -100.0),
        Vector3::new(0.0, 0.0, 100.0),
    ]
    .into_iter()
    .enumerate()
    {
        let end = graph.add_node(pos, NodeType::Junction);
        let speed = if major && arm < 2 {
            if stop { 60.0 / 3.6 } else { 50.0 / 3.6 }
        } else {
            30.0 / 3.6
        };
        graph.add_edge(Edge {
            start_node: end,
            end_node: center,
            geometry: vec![pos, Vector3::ZERO],
            physical_geometry: vec![pos, Vector3::ZERO],
            physical_length: 100.0,
            width: 7.0,
            fwd_lanes: 1,
            bkw_lanes: 1,
            speed_limit: speed,
            allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
            ..Default::default()
        });
    }
    graph.rebuild_adjacency_list();
    graph.rebuild_intersection_clips();
    let mut network = TransitNetwork::new();
    network.lane_system.rebuild(&mut graph);
    let incoming = std::array::from_fn(|edge| {
        *network.lane_system.edge_lanes[&edge]
            .iter()
            .find(|&&id| {
                let l = &network.lane_system.lanes[id];
                l.is_fwd && l.lane_type == LaneType::Vehicle
            })
            .unwrap()
    });
    (network, graph, incoming)
}
fn car(
    agents: &mut AgentSystem,
    network: &TransitNetwork,
    lanes: &[usize; 4],
    from: usize,
    to: usize,
) -> usize {
    let lane = lanes[from];
    let i = agents.spawn_debug_car(
        vec![(from + 1) as u32, 0, (to + 1) as u32],
        &network.lane_system,
        lane,
        Vector2::ZERO,
    );
    agents.current_node[i] = (from + 1) as u32;
    agents.current_path_index[i] = 1;
    agents.lane_distance[i] = network.lane_system.lanes[lane].length;
    agents.speed[i] = 0.0;
    agents.transit[i] = TRANSIT_NETWORK;
    agents.transit_mode[i] = MODE_CAR;
    i
}
fn prepare(agents: &mut AgentSystem, network: &TransitNetwork, graph: &RegionGraph) {
    agents.invalidate_lane_bucket_snapshot();
    agents.prepare_lane_buckets_for_tick(network, agents.agents.len());
    agents.prepare_junction_admission(network, graph, 0.1);
}
fn connector(network: &TransitNetwork, source: usize, to: usize) -> usize {
    *network.lane_system.lanes[source]
        .next_lanes
        .iter()
        .find(|&&id| {
            network.lane_system.lanes[id]
                .next_lanes
                .first()
                .is_some_and(|&out| network.lane_system.lanes[out].edge_id == to)
        })
        .unwrap()
}
#[test]
fn automatic_priority_beats_right_hand_and_storage_order() {
    for reverse in [false, true] {
        let (network, graph, lanes) = fixture(true, false);
        let mut agents = AgentSystem::new();
        let (a, b) = if reverse {
            let b = car(&mut agents, &network, &lanes, 3, 2);
            let a = car(&mut agents, &network, &lanes, 0, 1);
            (a, b)
        } else {
            let a = car(&mut agents, &network, &lanes, 0, 1);
            let b = car(&mut agents, &network, &lanes, 3, 2);
            (a, b)
        };
        prepare(&mut agents, &network, &graph);
        assert!(
            agents
                .junction_admission
                .allows(a, connector(&network, lanes[0], 1), &network)
        );
        assert!(
            !agents
                .junction_admission
                .allows(b, connector(&network, lanes[3], 2), &network)
        );
        assert_eq!(approach_control(&graph, 0, 0), ApproachControl::Priority);
        assert_eq!(approach_control(&graph, 0, 3), ApproachControl::Yield);
    }
}
#[test]
fn equal_roads_yield_to_the_right() {
    let (network, graph, lanes) = fixture(false, false);
    let mut agents = AgentSystem::new();
    let a = car(&mut agents, &network, &lanes, 0, 1);
    let b = car(&mut agents, &network, &lanes, 3, 2);
    prepare(&mut agents, &network, &graph);
    assert!(
        !agents
            .junction_admission
            .allows(a, connector(&network, lanes[0], 1), &network)
    );
    assert!(
        agents
            .junction_admission
            .allows(b, connector(&network, lanes[3], 2), &network)
    );
}
#[test]
fn four_way_right_hand_cycle_grants_a_member_and_then_drains() {
    let (mut network, mut graph, lanes) = fixture(false, false);
    let mut agents = AgentSystem::new();
    for (from, to) in [(0, 1), (1, 0), (2, 3), (3, 2)] {
        car(&mut agents, &network, &lanes, from, to);
    }
    prepare(&mut agents, &network, &graph);
    let winners: Vec<_> = (0..4)
        .filter(|&i| {
            agents.junction_admission.allows(
                i,
                connector(&network, lanes[i], [1, 0, 3, 2][i]),
                &network,
            )
        })
        .collect();
    assert_eq!(winners, vec![0]);
    let allocator = crate::simulation::buildings::allocator::BuildingAllocator::new();
    let time = crate::simulation::core::time::TimeSystem::new();
    for _ in 0..1000 {
        agents.tick(&allocator, &mut network, &mut graph, 0.1, &time);
    }
    for (i, lane) in lanes.into_iter().enumerate() {
        assert_ne!(
            agents.current_lane_id[i], lane,
            "car {i} must clear its approach"
        );
    }
}
#[test]
fn side_road_stop_requires_a_full_stop_but_releases_afterwards() {
    let (network, graph, lanes) = fixture(true, true);
    assert_eq!(approach_control(&graph, 0, 2), ApproachControl::Stop);
    let mut agents = AgentSystem::new();
    let a = car(&mut agents, &network, &lanes, 2, 3);
    let c = connector(&network, lanes[2], 3);
    agents.speed[a] = 1.0;
    prepare(&mut agents, &network, &graph);
    assert!(!agents.junction_admission.allows(a, c, &network));
    agents.speed[a] = 0.0;
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(a, c, &network));
}
#[test]
fn occupied_conflicting_movement_blocks_an_approach_until_clear() {
    let (network, graph, lanes) = fixture(true, false);
    let mut agents = AgentSystem::new();
    let a = car(&mut agents, &network, &lanes, 0, 1);
    let c = connector(&network, lanes[0], 1);
    let other = network
        .lane_system
        .controlled_movement(c)
        .unwrap()
        .conflicts[0];
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(a, c, &network));
    let blocker = car(&mut agents, &network, &lanes, 3, 2);
    agents
        .agents
        .enter_lane(blocker, &network.lane_system, other, 1.0, 0);
    prepare(&mut agents, &network, &graph);
    assert!(!agents.junction_admission.allows(a, c, &network));
    let out = network.lane_system.lanes[other].next_lanes[0];
    agents
        .agents
        .enter_lane(blocker, &network.lane_system, out, 20.0, 0);
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(a, c, &network));
}
#[test]
fn an_occupied_crossing_does_not_cancel_a_waiting_major_roads_priority() {
    let (network, graph, lanes) = fixture(true, false);
    let mut agents = AgentSystem::new();
    let major = car(&mut agents, &network, &lanes, 0, 1);
    let minor = car(&mut agents, &network, &lanes, 3, 2);
    let crossing = car(&mut agents, &network, &lanes, 3, 2);
    let a = connector(&network, lanes[0], 1);
    let b = connector(&network, lanes[3], 2);
    let distance = crate::config::CAR_LENGTH + crate::config::IDM_S_MIN + 1.0;
    assert!(distance < network.lane_system.lanes[b].length);
    agents
        .agents
        .enter_lane(crossing, &network.lane_system, b, distance, 0);
    prepare(&mut agents, &network, &graph);
    assert!(super::traffic::lane_entry_slot_clear(
        b,
        &agents.lane_buckets
    ));
    assert!(!agents.junction_admission.allows(major, a, &network));
    assert!(!agents.junction_admission.allows(minor, b, &network));

    let out = network.lane_system.lanes[b].next_lanes[0];
    agents
        .agents
        .enter_lane(crossing, &network.lane_system, out, 20.0, 0);
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(major, a, &network));
    assert!(!agents.junction_admission.allows(minor, b, &network));

    let out = network.lane_system.lanes[a].next_lanes[0];
    agents
        .agents
        .enter_lane(major, &network.lane_system, out, 20.0, 0);
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(minor, b, &network));
}

#[test]
fn incremental_rebuild_retires_controls_and_preserves_geometry() {
    let (mut network, mut graph, lanes) = fixture(true, false);
    let old = connector(&network, lanes[0], 1);
    let geometry = network.lane_system.lanes[lanes[0]].geometry.clone();
    network
        .lane_system
        .rebuild_edges_incremental(&mut graph, &[0].into_iter().collect());
    assert!(network.lane_system.controlled_movement(old).is_none());
    let new_lane = *network.lane_system.edge_lanes[&0]
        .iter()
        .find(|&&l| {
            network.lane_system.lanes[l].is_fwd
                && network.lane_system.lanes[l].lane_type == LaneType::Vehicle
        })
        .unwrap();
    assert_eq!(network.lane_system.lanes[new_lane].geometry, geometry);
    let new = connector(&network, new_lane, 1);
    assert!(network.lane_system.controlled_movement(new).is_some());
}

#[test]
fn conflicting_same_tick_claim_is_checked_after_snapshot_grant() {
    use super::claims::LaneClaimContext;
    use crate::simulation::economy::agents::data::MovementClaimMode;
    use std::sync::atomic::{AtomicUsize, Ordering};
    let (network, graph, lanes) = fixture(true, false);
    let mut agents = AgentSystem::new();
    let driver = car(&mut agents, &network, &lanes, 0, 1);
    let vehicle = connector(&network, lanes[0], 1);
    prepare(&mut agents, &network, &graph);
    let owners: Vec<_> = (0..network.lane_system.lanes.len())
        .map(|_| AtomicUsize::new(usize::MAX))
        .collect();
    let modes = [MovementClaimMode::Dynamic];
    let context =
        LaneClaimContext::new(&owners, &modes).with_junctions(&agents.junction_admission, &network);
    assert!(context.connector_allowed(driver, vehicle));
    let conflicting = network
        .lane_system
        .controlled_movement(vehicle)
        .unwrap()
        .conflicts[0];
    owners[conflicting].store(42, Ordering::Relaxed);
    assert!(!context.connector_allowed(driver, vehicle));
}

#[test]
fn oncoming_straight_movement_precedes_an_equal_road_left_turn() {
    let (network, graph, lanes) = fixture(false, false);
    let mut agents = AgentSystem::new();
    let left = car(&mut agents, &network, &lanes, 0, 2);
    let straight = car(&mut agents, &network, &lanes, 1, 0);
    prepare(&mut agents, &network, &graph);
    assert!(
        !agents
            .junction_admission
            .allows(left, connector(&network, lanes[0], 2), &network)
    );
    assert!(
        agents
            .junction_admission
            .allows(straight, connector(&network, lanes[1], 0), &network)
    );
}

#[test]
fn an_added_turn_pocket_does_not_promote_an_equal_road() {
    use crate::simulation::network::graph::RoadLaneTaper;
    let (_, mut graph, _) = fixture(false, false);
    let edge = graph.edge_mut(0);
    edge.fwd_lanes = 2;
    edge.width = 10.5;
    edge.lane_tapers = Some(Box::new([
        Some(RoadLaneTaper {
            length_m: 30.0,
            from_fwd: 1,
            from_bkw: 1,
            from_median_m: 0.0,
            left_pocket: true,
            right_pocket: false,
        }),
        None,
    ]));
    assert_eq!(
        approach_control(&graph, 0, 0),
        ApproachControl::Uncontrolled
    );
}


mod zone_support;
mod zone_roundabout;
mod zone_ramps;
mod zone_links;
mod zone_lanes;
mod zone_cycles;
mod zone_tail;
mod border_hold;
mod zone_storage;
mod zone_cache;
mod zone_step;
