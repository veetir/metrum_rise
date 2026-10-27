// SPDX-License-Identifier: GPL-2.0-only

//! Lane graph navigation helpers used by agent movement and planning.

use crate::simulation::economy::agents::ACCESS_PLAN_VALID;
use crate::simulation::network::TransitNetwork;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::lanes::LaneType;

/// Returns the road node where a road lane begins for its travel direction.
pub(super) fn lane_origin_node(
    lane_id: usize,
    transit_network: &TransitNetwork,
    graph: &RegionGraph,
) -> Option<u32> {
    let lane = transit_network.lane_system.lanes.get(lane_id)?;
    if lane.edge_id == usize::MAX || lane.edge_id >= graph.edge_count() {
        return None;
    }
    let edge = graph.edge(lane.edge_id);
    Some(if lane.is_fwd {
        edge.start_node
    } else {
        edge.end_node
    })
}

/// Returns the road node where a road lane ends for its travel direction.
pub(super) fn lane_terminal_node(
    lane_id: usize,
    transit_network: &TransitNetwork,
    graph: &RegionGraph,
) -> Option<u32> {
    let lane = transit_network.lane_system.lanes.get(lane_id)?;
    if lane.edge_id == usize::MAX || lane.edge_id >= graph.edge_count() {
        return None;
    }
    let edge = graph.edge(lane.edge_id);
    Some(if lane.is_fwd {
        edge.end_node
    } else {
        edge.start_node
    })
}

fn connector_targets_edge(
    conn_lane_id: usize,
    target_edge_id: usize,
    transit_network: &TransitNetwork,
) -> bool {
    let Some(conn_lane) = transit_network.lane_system.lanes.get(conn_lane_id) else {
        return false;
    };
    if conn_lane.edge_id != usize::MAX {
        return false;
    }
    let Some(&target_lane_id) = conn_lane.next_lanes.first() else {
        return false;
    };
    transit_network
        .lane_system
        .lanes
        .get(target_lane_id)
        .is_some_and(|target_lane| target_lane.edge_id == target_edge_id)
}

fn connector_targets_lane(
    conn_lane_id: usize,
    target_lane_id: usize,
    transit_network: &TransitNetwork,
) -> bool {
    transit_network
        .lane_system
        .lanes
        .get(conn_lane_id)
        .is_some_and(|conn_lane| {
            conn_lane.edge_id == usize::MAX
                && conn_lane.next_lanes.first().copied() == Some(target_lane_id)
        })
}

/// Fills `out` with connector lanes that lead from `from_lane_id` to `target_edge_id`.
pub(super) fn collect_connector_lanes_to_edge(
    from_lane_id: usize,
    target_edge_id: usize,
    transit_network: &TransitNetwork,
    out: &mut Vec<usize>,
) -> bool {
    out.clear();
    let Some(from_lane) = transit_network.lane_system.lanes.get(from_lane_id) else {
        return false;
    };

    for &conn_lane_id in &from_lane.next_lanes {
        if connector_targets_edge(conn_lane_id, target_edge_id, transit_network) {
            out.push(conn_lane_id);
        }
    }
    if out.len() > 1 {
        let shortest = out
            .iter()
            .map(|&lane_id| transit_network.lane_system.lanes[lane_id].length)
            .min_by(f32::total_cmp)
            .unwrap_or(f32::INFINITY);
        out.retain(|&lane_id| {
            (transit_network.lane_system.lanes[lane_id].length - shortest).abs() <= 0.001
        });
    }
    !out.is_empty()
}

/// Fills `out` with connector lanes that lead from `from_lane_id` to `target_lane_id`.
pub(super) fn collect_connector_lanes_to_lane(
    from_lane_id: usize,
    target_lane_id: usize,
    transit_network: &TransitNetwork,
    out: &mut Vec<usize>,
) -> bool {
    out.clear();
    let Some(from_lane) = transit_network.lane_system.lanes.get(from_lane_id) else {
        return false;
    };

    for &conn_lane_id in &from_lane.next_lanes {
        if connector_targets_lane(conn_lane_id, target_lane_id, transit_network) {
            out.push(conn_lane_id);
        }
    }
    !out.is_empty()
}

/// Returns the vehicle lane of `target_edge_id` that `from_lane_id` runs straight into at a
/// pass-through node with no connectors.
pub(super) fn direct_vehicle_lane_to_edge(
    from_lane_id: usize,
    target_edge_id: usize,
    transit_network: &TransitNetwork,
) -> Option<usize> {
    let from_lane = transit_network.lane_system.lanes.get(from_lane_id)?;
    if from_lane.lane_type != LaneType::Vehicle {
        return None;
    }

    from_lane.next_lanes.iter().copied().find(|&next_lane_id| {
        transit_network
            .lane_system
            .lanes
            .get(next_lane_id)
            .is_some_and(|lane| {
                lane.edge_id == target_edge_id && lane.lane_type == LaneType::Vehicle
            })
    })
}

/// Returns whether `from_lane_id` runs straight into the road lane `target_lane_id`.
pub(super) fn direct_vehicle_lane_to_lane(
    from_lane_id: usize,
    target_lane_id: usize,
    transit_network: &TransitNetwork,
) -> bool {
    let Some(from_lane) = transit_network.lane_system.lanes.get(from_lane_id) else {
        return false;
    };
    if from_lane.lane_type != LaneType::Vehicle {
        return false;
    }
    from_lane.next_lanes.contains(&target_lane_id)
        && transit_network
            .lane_system
            .lanes
            .get(target_lane_id)
            .is_some_and(|lane| lane.lane_type == LaneType::Vehicle && lane.edge_id != usize::MAX)
}

/// Seed of the stable choice among equally short connectors at a junction.
#[inline(always)]
pub(super) fn connector_choice_seed(
    agent_idx: usize,
    from_lane_id: usize,
    target_id: usize,
    node_id: u32,
    path_idx: usize,
) -> u64 {
    (agent_idx as u64)
        ^ ((from_lane_id as u64) << 11)
        ^ ((target_id as u64) << 23)
        ^ ((node_id as u64) << 37)
        ^ ((path_idx as u64) << 3)
}

fn connection_lane_to_edge(
    from_lane_id: usize,
    target_edge_id: usize,
    transit_network: &TransitNetwork,
) -> Option<usize> {
    let from_lane = transit_network.lane_system.lanes.get(from_lane_id)?;
    from_lane
        .next_lanes
        .iter()
        .copied()
        .find(|&conn_lane_id| connector_targets_edge(conn_lane_id, target_edge_id, transit_network))
}

fn connection_lane_to_lane_unclaimed(
    from_lane_id: usize,
    target_lane_id: usize,
    transit_network: &TransitNetwork,
) -> Option<usize> {
    let from_lane = transit_network.lane_system.lanes.get(from_lane_id)?;
    from_lane
        .next_lanes
        .iter()
        .copied()
        .find(|&conn_lane_id| connector_targets_lane(conn_lane_id, target_lane_id, transit_network))
}

/// Returns the connector lane that should be used after the current road lane.
#[allow(clippy::too_many_arguments)]
pub(super) fn planned_next_connector(
    lane_id: usize,
    current_node: u32,
    path: &[u32],
    path_idx: usize,
    access_flags: u8,
    planned_detach_node: u32,
    planned_detach_lane_id: usize,
    transit_network: &TransitNetwork,
    graph: &RegionGraph,
) -> Option<usize> {
    let lane = transit_network.lane_system.lanes.get(lane_id)?;
    if lane.edge_id == usize::MAX {
        return None;
    }
    let terminal_node = lane_terminal_node(lane_id, transit_network, graph)?;

    if (access_flags & ACCESS_PLAN_VALID) != 0
        && terminal_node == planned_detach_node
        && planned_detach_lane_id != usize::MAX
        && lane_origin_node(planned_detach_lane_id, transit_network, graph)
            == Some(planned_detach_node)
    {
        return connection_lane_to_lane_unclaimed(lane_id, planned_detach_lane_id, transit_network)
            .or_else(|| {
                let edge = transit_network
                    .lane_system
                    .lanes
                    .get(planned_detach_lane_id)?
                    .edge_id;
                connection_lane_to_edge(lane_id, edge, transit_network)
            });
    }

    let next_idx = if path.get(path_idx).copied() == Some(terminal_node) {
        path_idx + 1
    } else {
        path_idx
    };
    let next_node = *path.get(next_idx)?;
    let target_edge = graph.get_edge_between_nodes(terminal_node, next_node)?;
    let current_edge = lane.edge_id;
    if target_edge == current_edge && current_node != terminal_node {
        return None;
    }
    connection_lane_to_edge(lane_id, target_edge, transit_network)
}

/// Tests an existing direct continuation or connector without constructing a candidate list.
pub(super) fn vehicle_lane_reaches_edge(
    lane_id: usize,
    edge: usize,
    network: &TransitNetwork,
) -> bool {
    network.lane_system.lanes.get(lane_id).is_some_and(|lane| {
        lane.next_lanes.iter().any(|&next| {
            let next = &network.lane_system.lanes[next];
            next.edge_id == edge
                || (next.edge_id == usize::MAX
                    && next
                        .next_lanes
                        .first()
                        .is_some_and(|&out| network.lane_system.lanes[out].edge_id == edge))
        })
    })
}

/// Tests whether another same-direction approach lane can drive the requested road turn.
pub(super) fn sibling_lane_reaches_edge(
    lane_id: usize,
    edge: usize,
    network: &TransitNetwork,
) -> bool {
    let lanes = &network.lane_system;
    let Some(source) = lanes.lanes.get(lane_id) else {
        return false;
    };
    lanes
        .edge_lanes
        .get(&source.edge_id)
        .is_some_and(|siblings| {
            siblings.iter().any(|&id| {
                let lane = &lanes.lanes[id];
                lane.lane_type == LaneType::Vehicle
                    && lane.is_fwd == source.is_fwd
                    && vehicle_lane_reaches_edge(id, edge, network)
            })
        })
}

/// Returns the junction node only inside a multi-lane road's preparation window. This
/// O(1) guard runs before callers read path/access columns or scan sibling successors.
#[inline]
pub(super) fn junction_approach_node(
    lane_id: usize,
    lane_d: f32,
    network: &TransitNetwork,
    graph: &RegionGraph,
) -> Option<u32> {
    if !network.lane_system.needs_turn_preparation(lane_id) {
        return None;
    }
    let lane = network.lane_system.lanes.get(lane_id)?;
    if lane.edge_id == usize::MAX || lane.lane_type != LaneType::Vehicle {
        return None;
    }
    let edge = graph.get_edge(lane.edge_id)?;
    let count = if lane.is_fwd {
        edge.fwd_lanes
    } else {
        edge.bkw_lanes
    };
    if count <= 1 || lane.length - lane_d > f32::from(count - 1) * 70.0 + 30.0 {
        return None;
    }
    Some(if lane.is_fwd {
        edge.end_node
    } else {
        edge.start_node
    })
}

/// Chooses the nearest lane permitting the next route movement at a node returned by
/// `junction_approach_node`. Returning the current lane prevents discretionary weaving.
/// The allocation-free query is O(D + L * C): local node adjacency, siblings and successors.
#[allow(clippy::too_many_arguments)]
pub(super) fn junction_approach_lane(
    lane_id: usize,
    terminal: u32,
    path: &[u32],
    path_idx: usize,
    access_flags: u8,
    detach_node: u32,
    detach_lane_id: usize,
    network: &TransitNetwork,
    graph: &RegionGraph,
) -> Option<usize> {
    let lanes = &network.lane_system;
    let lane = lanes.lanes.get(lane_id)?;
    let target_edge = if access_flags & ACCESS_PLAN_VALID != 0
        && terminal == detach_node
        && lane_origin_node(detach_lane_id, network, graph) == Some(detach_node)
    {
        lanes.lanes.get(detach_lane_id)?.edge_id
    } else {
        let next = path_idx + usize::from(path.get(path_idx).copied() == Some(terminal));
        graph.get_edge_between_nodes(terminal, *path.get(next)?)?
    };
    lanes
        .edge_lanes
        .get(&lane.edge_id)?
        .iter()
        .copied()
        .filter(|&candidate| {
            let sibling = &lanes.lanes[candidate];
            sibling.is_fwd == lane.is_fwd
                && sibling.lane_type == LaneType::Vehicle
                && vehicle_lane_reaches_edge(candidate, target_edge, network)
        })
        .min_by_key(|&candidate| {
            let index = lanes.lanes[candidate].lane_idx;
            (
                (i16::from(index) - i16::from(lane.lane_idx)).unsigned_abs(),
                index.unsigned_abs(),
                candidate,
            )
        })
}
