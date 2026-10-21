// SPDX-License-Identifier: GPL-2.0-only

//! Rebuild-time, local junction conflict and automatic right-of-way products.

use super::{Lane, LaneSystem, LaneType};
use crate::simulation::network::graph::{Edge, RegionGraph};
use crate::simulation::network::types::NodeType;
use godot::prelude::{Vector2, Vector3};

/// Automatic approach signing; derived from the same hierarchy as movement priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ApproachControl {
    /// Equal roads use the right-hand rule.
    Uncontrolled,
    /// Highest road hierarchy at a mixed-priority junction.
    Priority,
    /// Minor approach must give way.
    Yield,
    /// Low-speed side road joining a road of at least 60 km/h must stop first.
    Stop,
}

/// Static movement admission data, indexed by the connector's ordinary lane ID.
pub(crate) struct ControlledMovement {
    /// Physical approach lane supplying this connector.
    pub(crate) incoming: usize,
    /// Swept trajectories that cannot be occupied concurrently.
    pub(crate) conflicts: Vec<usize>,
    /// Conflicting movements with statutory precedence over this movement.
    pub(crate) yields_to: Vec<usize>,
    /// Requires a stopped snapshot at the approach mouth before admission.
    pub(crate) stop: bool,
}

// Structural EdgeClass describes bridges/tunnels, not a road hierarchy. Until the importer
// authors priority, use design speed, through-lane count, then carriageway width. Pockets
// retain the original section's rank, so adding a turning lane cannot change right of way.
fn rank(edge: &Edge) -> (bool, i32, u16, i32) {
    let (lanes, width) = edge.pocket_taper().map_or(
        (
            u16::from(edge.fwd_lanes) + u16::from(edge.bkw_lanes),
            edge.width,
        ),
        |(_, taper)| {
            let lanes = u16::from(taper.from_fwd) + u16::from(taper.from_bkw);
            (
                lanes,
                f32::from(lanes) * crate::config::LANE_WIDTH + taper.from_median_m,
            )
        },
    );
    (
        edge.roundabout,
        (edge.speed_limit * 1000.0).round() as i32,
        lanes,
        (width * 1000.0).round() as i32,
    )
}

/// Derives B1/B5/B6 signing from local incident roads, without scanning the network.
pub(crate) fn approach_control(graph: &RegionGraph, node: u32, edge: usize) -> ApproachControl {
    if graph.node(node).node_type == NodeType::LaneTransition {
        return ApproachControl::Uncontrolled;
    }
    if graph.edge(edge).roundabout {
        return ApproachControl::Uncontrolled;
    }
    if graph
        .node_adjacency(node)
        .iter()
        .any(|&id| !graph.edge(id).deleted && graph.edge(id).roundabout)
    {
        return ApproachControl::Yield;
    }
    let mut min = rank(graph.edge(edge));
    let mut max = min;
    let mut degree = 0;
    for &id in graph.node_adjacency(node) {
        let road = graph.edge(id);
        if !road.deleted && road.fwd_lanes + road.bkw_lanes > 0 {
            let other = rank(road);
            min = min.min(other);
            max = max.max(other);
            degree += 1;
        }
    }
    if degree < 3 || min == max {
        ApproachControl::Uncontrolled
    } else if rank(graph.edge(edge)) == max {
        ApproachControl::Priority
    } else if max.1 >= 16_667 && graph.edge(edge).speed_limit <= 8.334 {
        ApproachControl::Stop
    } else {
        ApproachControl::Yield
    }
}

fn flat(p: Vector3) -> Vector2 {
    Vector2::new(p.x, p.z)
}
fn cross(a: Vector2, b: Vector2) -> f32 {
    a.x * b.y - a.y * b.x
}

fn conflicts(a: &Lane, b: &Lane) -> bool {
    // Swept vehicle width with clearance. This is compile-time geometry work;
    // movement uses the resulting connector-ID adjacency lists only.
    let clearance: f32 = 2.2;
    a.geometry.windows(2).any(|p| {
        b.geometry.windows(2).any(|q| {
            super::geometry::segment_distance_squared_xz(
                flat(p[0]),
                flat(p[1]),
                flat(q[0]),
                flat(q[1]),
            ) < clearance * clearance
        })
    })
}

impl LaneSystem {
    /// Compiles only this node; incremental rebuilds preserve remote control products.
    pub(super) fn rebuild_node_control(&mut self, graph: &RegionGraph, node: usize) {
        let Some(ids) = self.node_lanes.get(&node) else {
            return;
        };
        self.controlled.resize_with(self.lanes.len(), || None);
        if graph.node(node as u32).node_type == NodeType::LaneTransition {
            return;
        }
        let mut incoming = std::collections::HashMap::new();
        for &edge in graph.node_adjacency(node as u32) {
            if let Some(lanes) = self.edge_lanes.get(&edge) {
                for &lane in lanes {
                    for &next in &self.lanes[lane].next_lanes {
                        incoming.insert(next, lane);
                    }
                }
            }
        }
        for &id in ids {
            let lane = &self.lanes[id];
            if lane.lane_type != LaneType::Vehicle {
                continue;
            }
            let Some(&source) = incoming.get(&id) else {
                continue;
            };
            let mut data = ControlledMovement {
                incoming: source,
                conflicts: Vec::new(),
                yields_to: Vec::new(),
                stop: approach_control(graph, node as u32, self.lanes[source].edge_id)
                    == ApproachControl::Stop,
            };
            for &other in ids {
                if other == id {
                    continue;
                }
                let b = &self.lanes[other];
                if b.lane_type != LaneType::Vehicle {
                    continue;
                }
                let Some(&from) = incoming.get(&other) else {
                    continue;
                };
                if !conflicts(lane, b) {
                    continue;
                }
                data.conflicts.push(other);
                let yields = {
                    let a_rank = rank(graph.edge(self.lanes[source].edge_id));
                    let b_rank = rank(graph.edge(self.lanes[from].edge_id));
                    if a_rank != b_rank {
                        a_rank < b_rank
                    } else {
                        let a_dir = -super::lane_use::outward(
                            graph,
                            self.lanes[source].edge_id,
                            node as u32,
                        );
                        let b_dir =
                            -super::lane_use::outward(graph, self.lanes[from].edge_id, node as u32);
                        let side = cross(a_dir, b_dir);
                        // Only numerical collinearity bypasses the right-hand rule.
                        if side.abs() > 1.0e-5 {
                            side < 0.0
                        } else if a_dir.dot(b_dir) < 0.0 {
                            // Turning left yields to oncoming straight/right movements.
                            let left = |connector: &Lane, source: usize| {
                                super::lane_use::is_left_movement(
                                    graph,
                                    self.lanes[source].edge_id,
                                    self.lanes[connector.next_lanes[0]].edge_id,
                                    node as u32,
                                )
                            };
                            let a_left = left(lane, source);
                            let b_left = left(b, from);
                            if a_left != b_left { a_left } else { id > other }
                        } else {
                            id > other
                        }
                    }
                };
                if yields {
                    data.yields_to.push(other);
                }
            }
            self.controlled[id] = Some(data);
        }
    }

    /// Static control for a connector, absent for sidewalk corners and direct continuations.
    pub(crate) fn controlled_movement(&self, lane: usize) -> Option<&ControlledMovement> {
        self.controlled.get(lane)?.as_ref()
    }
}
