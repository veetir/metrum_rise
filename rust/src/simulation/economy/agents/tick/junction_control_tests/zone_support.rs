// SPDX-License-Identifier: GPL-2.0-only

//! Fixed-step safety oracle and finite-fleet drainage, with enough idle rows for Rayon dispatch.

use super::*;
use crate::config::CAR_LENGTH;
use crate::simulation::economy::agents::TRANSIT_INTERSECTION;

/// Advances a generated fleet, asserting pair-zone exclusion including outgoing body tails.
pub(super) fn drain(
    mut agents: AgentSystem,
    mut network: TransitNetwork,
    mut graph: RegionGraph,
    targets: &[usize],
    workers: usize,
) -> Vec<(usize, u32, u32, u8)> {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(workers)
        .build()
        .unwrap();
    pool.install(|| {
        let count = targets.len();
        let mut previous = vec![usize::MAX; count];
        let mut tail = vec![usize::MAX; count];
        let mut reached = vec![false; count];
        for _ in 0..4096 {
            agents.spawn_housed_agent(usize::MAX, 0.0, 0.0);
        }
        let allocator = crate::simulation::buildings::allocator::BuildingAllocator::new();
        let time = crate::simulation::core::time::TimeSystem::new();
        let mut nodes: Vec<_> = network.lane_system.node_lanes.keys().copied().collect();
        nodes.sort_unstable();
        for step in 0..4000 {
            previous.copy_from_slice(&agents.current_lane_id[..count]);
            agents.tick(&allocator, &mut network, &mut graph, 0.05, &time);
            for i in 0..count {
                let old = previous[i];
                if old < network.lane_system.lanes.len()
                    && network.lane_system.lanes[old].edge_id == usize::MAX
                    && agents.current_lane_id[i] != old
                    && network.lane_system.lanes[old].next_lanes.first().copied()
                        == Some(agents.current_lane_id[i])
                {
                    tail[i] = old;
                }
                if tail[i] != usize::MAX {
                    let lane = &network.lane_system.lanes[tail[i]];
                    if lane.next_lanes.first().copied() != Some(agents.current_lane_id[i])
                        || agents.lane_distance[i] > CAR_LENGTH
                    {
                        tail[i] = usize::MAX;
                    }
                }
                reached[i] |=
                    agents.current_edge[i] == targets[i] && agents.lane_distance[i] > CAR_LENGTH;
            }
            for &node in &nodes {
                for zone in network.lane_system.junction_zones(node) {
                    let occupies = |i: usize, connector: usize, begin: f32, end: f32| {
                        let d = if agents.current_lane_id[i] == connector
                            && agents.transit[i] == TRANSIT_INTERSECTION
                        {
                            agents.lane_distance[i]
                        } else if tail[i] == connector {
                            network.lane_system.lanes[connector].length + agents.lane_distance[i]
                        } else {
                            return false;
                        };
                        d > begin && d - CAR_LENGTH < end
                    };
                    for a in 0..count {
                        if !occupies(a, zone.a, zone.movement.enter, zone.movement.exit) {
                            continue;
                        }
                        for b in 0..count {
                            assert!(
                                a == b
                                    || !occupies(
                                        b,
                                        zone.b,
                                        zone.movement.other_enter,
                                        zone.movement.other_exit
                                    ),
                                "zone {} at node {node}: cars {a}/{b} overlap at step {step}; a={:?} b={:?} zone={zone:?}",
                                zone.movement.id,
                                (agents.current_lane_id[a], agents.lane_distance[a], tail[a]),
                                (agents.current_lane_id[b], agents.lane_distance[b], tail[b])
                            );
                        }
                    }
                }
            }
            if reached.iter().all(|&r| r) {
                return (0..count)
                    .map(|i| {
                        (
                            agents.current_lane_id[i],
                            agents.lane_distance[i].to_bits(),
                            agents.speed[i].to_bits(),
                            agents.transit[i],
                        )
                    })
                    .collect();
            }
        }
        let stuck: Vec<_> = (0..count)
            .filter(|&i| !reached[i])
            .map(|i| {
                (
                    i,
                    agents.current_lane_id[i],
                    agents.lane_distance[i],
                    agents.speed[i],
                )
            })
            .collect();
        panic!("finite fleet failed to drain: {stuck:?}");
    })
}

/// Builds routed finite fleets on generated lanes, keeping each source queue physically spaced.
pub(super) fn routed_fleet(
    mut graph: RegionGraph,
    routes: &[(usize, bool, usize, bool)],
    workers: usize,
) -> Vec<(usize, u32, u32, u8)> {
    graph.rebuild_adjacency_list();
    graph.rebuild_intersection_clips();
    let mut network = TransitNetwork::new();
    network.lane_system.rebuild(&mut graph);
    network.cch_graph = crate::simulation::pathing::cch::CchGraph::build(&graph);
    let mut agents = AgentSystem::new();
    let mut targets = Vec::new();
    let mut queues = vec![0usize; network.lane_system.lanes.len()];
    for _ in 0..6 {
        for &(source, forward, target, target_forward) in routes {
            let road = graph.edge(source);
            let from = if forward {
                road.start_node
            } else {
                road.end_node
            };
            let arrival = graph.edge(target);
            let to = if target_forward {
                arrival.end_node
            } else {
                arrival.start_node
            };
            let (_, _, path) = network
                .cch_graph
                .find_path(from, to, usize::MAX, usize::MAX, &graph, TransitFlags::CAR)
                .unwrap();
            assert_eq!(
                path[1],
                if forward {
                    road.end_node
                } else {
                    road.start_node
                }
            );
            let next_edge = graph
                .node_adjacency(path[1])
                .iter()
                .copied()
                .find(|&e| {
                    let r = graph.edge(e);
                    (r.start_node == path[1] && r.end_node == path[2])
                        || (r.end_node == path[1] && r.start_node == path[2])
                })
                .unwrap();
            let lane = *network.lane_system.edge_lanes[&source]
                .iter()
                .find(|&&id| {
                    let l = &network.lane_system.lanes[id];
                    l.lane_type == LaneType::Vehicle
                        && l.is_fwd == forward
                        && l.next_lanes.iter().any(|&c| {
                            let x = &network.lane_system.lanes[c];
                            x.edge_id == next_edge
                                || x.next_lanes
                                    .iter()
                                    .any(|&n| network.lane_system.lanes[n].edge_id == next_edge)
                        })
                })
                .unwrap();
            let i = agents.spawn_debug_car(path, &network.lane_system, lane, Vector2::ZERO);
            agents.current_node[i] = from;
            agents.current_path_index[i] = 1;
            agents.lane_distance[i] = network.lane_system.lanes[lane].length
                - 1.0
                - queues[lane] as f32 * (CAR_LENGTH + crate::config::IDM_S_MIN + 0.5);
            assert!(agents.lane_distance[i] >= 0.0);
            queues[lane] += 1;
            agents.speed[i] = 0.0;
            targets.push(target);
        }
    }
    drain(agents, network, graph, &targets, workers)
}
