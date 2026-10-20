// SPDX-License-Identifier: GPL-2.0-only

//! Deterministic default movement allocation, evaluated only while rebuilding junction lanes.

use super::super::graph::RegionGraph;
use crate::config;
use godot::prelude::*;

const LEFT: u8 = 1;
const STRAIGHT: u8 = 2;
const RIGHT: u8 = 4;

// Tangents point out from the node. Physical centerlines keep classification independent of
// lane offsets, clipping, median width and the order in which lanes are rebuilt.
/// Unit centreline tangent pointing away from a junction, shared by lane use and priority.
pub(super) fn outward(graph: &RegionGraph, edge: usize, node: u32) -> Vector2 {
    let edge = graph.edge(edge);
    let points = &edge.physical_geometry;
    let direction = if edge.start_node == node {
        points
            .windows(2)
            .map(|p| p[1] - p[0])
            .find(|d| d.x * d.x + d.z * d.z > 1e-8)
    } else {
        points
            .windows(2)
            .rev()
            .map(|p| p[0] - p[1])
            .find(|d| d.x * d.x + d.z * d.z > 1e-8)
    }
    .unwrap_or_else(|| {
        let other = if edge.start_node == node {
            edge.end_node
        } else {
            edge.start_node
        };
        graph.node(other).pos - graph.node(node).pos
    });
    Vector2::new(direction.x, direction.z).normalized()
}

fn movement(incoming: Vector2, outgoing: Vector2) -> u8 {
    // The inclusive 45-degree straight sector has a fixed boundary, with no best-arm heuristic.
    if incoming.dot(outgoing) >= std::f32::consts::FRAC_1_SQRT_2 {
        STRAIGHT
    } else if incoming.cross(outgoing) < 0.0 {
        LEFT
    } else {
        RIGHT
    }
}

/// Uses the same inclusive straight sector for priority and lane allocation.
pub(super) fn is_left_movement(
    graph: &RegionGraph,
    incoming: usize,
    outgoing: usize,
    node: u32,
) -> bool {
    movement(
        -outward(graph, incoming, node),
        outward(graph, outgoing, node),
    ) == LEFT
}

fn lane_rank(index: i8, count: usize) -> usize {
    let center_rank = if index >= 0 {
        index as usize
    } else {
        (-i16::from(index) - 1) as usize
    };
    if config::DRIVE_ON_LEFT {
        count - 1 - center_rank
    } else {
        center_rank
    }
}

fn lane_mask(rank: usize, count: usize, available: u8) -> u8 {
    if count == 1 || available.count_ones() <= 1 {
        return available;
    }
    match available {
        // Two shared lanes preserve all three movements without a dedicated middle lane.
        7 if count == 2 => {
            if rank == 0 {
                LEFT | STRAIGHT
            } else {
                STRAIGHT | RIGHT
            }
        }
        7 => {
            if rank == 0 {
                LEFT
            } else if rank == count - 1 {
                RIGHT
            } else {
                STRAIGHT
            }
        }
        3 => {
            if rank == 0 {
                LEFT
            } else {
                STRAIGHT
            }
        }
        6 => {
            if rank == count - 1 {
                RIGHT
            } else {
                STRAIGHT
            }
        }
        5 => {
            if rank < count.div_ceil(2) {
                LEFT
            } else {
                RIGHT
            }
        }
        _ => available,
    }
}

// Overlapping integer rank intervals produce monotone fans/merges for unequal lane counts.
// Left/right turns use the corresponding outer target group; straight uses the whole group.
fn target_matches(rank: usize, count: usize, target: usize, targets: usize, turn: u8) -> bool {
    let span = if turn == STRAIGHT {
        targets
    } else {
        count.min(targets)
    };
    let start = if turn == RIGHT { targets - span } else { 0 };
    target >= start
        && target < start + span
        && (rank + 1) * span > (target - start) * count
        && (target - start + 1) * count > rank * span
}

/// Rebuild-only default policy for one approach. Whitelists bypass this policy entirely.
pub(super) struct DefaultLaneUse {
    incoming: Vector2,
    available: u8,
    count: usize,
    node: u32,
    left_pocket: bool,
    right_pocket: bool,
}

impl DefaultLaneUse {
    /// Classifies outgoing vehicle arms once per approach lane, using only node-local adjacency.
    pub(super) fn new(graph: &RegionGraph, edge: usize, node: u32) -> Self {
        let incoming = -outward(graph, edge, node);
        let approach = graph.edge(edge);
        let count = usize::from(if approach.end_node == node {
            approach.fwd_lanes
        } else {
            approach.bkw_lanes
        });
        let mut available = 0;
        for &other in graph.node_adjacency(node) {
            let arm = graph.edge(other);
            let outgoing = if arm.start_node == node {
                arm.fwd_lanes
            } else {
                arm.bkw_lanes
            };
            if other != edge && !arm.deleted && outgoing > 0 {
                available |= movement(incoming, outward(graph, other, node));
            }
        }
        let pockets = approach.pocket_taper().filter(|(end, _)| {
            if *end == 0 {
                approach.end_node == node
            } else {
                approach.start_node == node
            }
        });
        Self {
            left_pocket: pockets.is_some_and(|(_, t)| t.left_pocket),
            right_pocket: pockets.is_some_and(|(_, t)| t.right_pocket),
            incoming,
            available,
            count,
            node,
        }
    }

    fn mask(&self, rank: usize) -> u8 {
        let left = usize::from(self.left_pocket && self.available & LEFT != 0);
        let right = usize::from(self.right_pocket && self.available & RIGHT != 0);
        if left != 0 && rank == 0 {
            return LEFT;
        }
        if right != 0 && rank + 1 == self.count {
            return RIGHT;
        }
        let remaining = self.available
            & !(if left != 0 { LEFT } else { 0 })
            & !(if right != 0 { RIGHT } else { 0 });
        let mask = lane_mask(
            rank - left,
            self.count - left - right,
            if remaining == 0 {
                self.available
            } else {
                remaining
            },
        );
        if self.left_pocket || self.right_pocket {
            mask | (self.available & STRAIGHT)
        } else {
            mask
        }
    }

    pub(super) fn supports_pockets(&self, left: bool, right: bool) -> bool {
        (!left || self.available & LEFT != 0) && (!right || self.available & RIGHT != 0)
    }

    /// Single-movement and single-lane approaches need no advance movement selection.
    pub(super) fn needs_lane_choice(&self) -> bool {
        self.count > 1 && self.available.count_ones() > 1
    }

    /// Tests a lane pair without allocating or querying unrelated roads.
    pub(super) fn allows(&self, graph: &RegionGraph, from: i8, edge: usize, to: i8) -> bool {
        let turn = movement(self.incoming, outward(graph, edge, self.node));
        let rank = lane_rank(from, self.count);
        if self.mask(rank) & turn == 0 {
            return false;
        }
        let mut group_rank = 0;
        let mut group_count = 0;
        for candidate in 0..self.count {
            if self.mask(candidate) & turn != 0 {
                group_rank += usize::from(candidate < rank);
                group_count += 1;
            }
        }
        let target = graph.edge(edge);
        let target_count = usize::from(if target.start_node == self.node {
            target.fwd_lanes
        } else {
            target.bkw_lanes
        });
        target_matches(
            group_rank,
            group_count,
            lane_rank(to, target_count),
            target_count,
            turn,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pockets_preserve_straight_travel_in_the_base_lanes() {
        for (left, right, expected) in [
            (true, false, vec![LEFT, STRAIGHT, STRAIGHT | RIGHT]),
            (false, true, vec![LEFT | STRAIGHT, STRAIGHT, RIGHT]),
            (true, true, vec![LEFT, STRAIGHT, STRAIGHT, RIGHT]),
        ] {
            let use_rules = DefaultLaneUse {
                incoming: Vector2::new(1.0, 0.0),
                available: 7,
                count: expected.len(),
                node: 0,
                left_pocket: left,
                right_pocket: right,
            };
            assert_eq!(
                (0..expected.len())
                    .map(|rank| use_rules.mask(rank))
                    .collect::<Vec<_>>(),
                expected
            );
        }
    }

    #[test]
    fn lane_use_shared_and_dedicated_one_to_four_lanes() {
        for (count, expected) in [
            (1, vec![7]),
            (2, vec![3, 6]),
            (3, vec![1, 2, 4]),
            (4, vec![1, 2, 2, 4]),
        ] {
            assert_eq!(
                (0..count)
                    .map(|rank| lane_mask(rank, count, 7))
                    .collect::<Vec<_>>(),
                expected
            );
            for available in 1..=7 {
                let masks: Vec<_> = (0..count)
                    .map(|rank| lane_mask(rank, count, available))
                    .collect();
                assert!(
                    masks
                        .iter()
                        .all(|&mask| mask != 0 && mask & !available == 0)
                );
                assert_eq!(masks.into_iter().fold(0, |a, b| a | b), available);
            }
        }
        assert_eq!([lane_mask(0, 2, 5), lane_mask(1, 2, 5)], [LEFT, RIGHT]);
        assert_eq!([lane_mask(0, 2, 3), lane_mask(1, 2, 3)], [LEFT, STRAIGHT]);
        assert_eq!([lane_mask(0, 2, 6), lane_mask(1, 2, 6)], [STRAIGHT, RIGHT]);
    }

    #[test]
    fn lane_use_angle_sectors_include_skewed_and_sloped_arms() {
        for (degrees, expected) in [
            (-160.0_f32, LEFT),
            (-90.0, LEFT),
            (-46.0, LEFT),
            (-44.0, STRAIGHT),
            (0.0, STRAIGHT),
            (44.0, STRAIGHT),
            (46.0, RIGHT),
            (90.0, RIGHT),
            (160.0, RIGHT),
        ] {
            let angle = degrees.to_radians();
            assert_eq!(
                movement(Vector2::RIGHT, Vector2::new(angle.cos(), angle.sin())),
                expected
            );
        }
    }

    #[test]
    fn lane_use_target_fans_never_cross_and_every_source_has_an_exit() {
        for count in 1..=4 {
            for targets in 1..=4 {
                for turn in [LEFT, STRAIGHT, RIGHT] {
                    let pairs: Vec<_> = (0..count)
                        .flat_map(|rank| {
                            (0..targets).filter_map(move |target| {
                                target_matches(rank, count, target, targets, turn)
                                    .then_some((rank, target))
                            })
                        })
                        .collect();
                    for rank in 0..count {
                        assert!(pairs.iter().any(|&(r, _)| r == rank));
                    }
                    for &(a, b) in &pairs {
                        for &(c, d) in &pairs {
                            assert!(!(a < c && b > d));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod junction_tests {
    use super::*;
    use crate::simulation::network::graph::Edge;
    use crate::simulation::network::lanes::{LaneSystem, LaneType};
    use crate::simulation::network::types::{NodeType, TransitFlags, TransitType};

    #[test]
    fn lane_use_t_cross_and_skew_preserve_every_legal_road_turn() {
        for count in 1..=4 {
            for arms in [3, 4] {
                for skew in [false, true] {
                    let mut graph = RegionGraph::new();
                    let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
                    let positions = if skew {
                        [
                            Vector3::new(-100.0, 0.0, 0.0),
                            Vector3::new(90.0, 4.0, 20.0),
                            Vector3::new(-25.0, 0.0, -100.0),
                            Vector3::new(35.0, 0.0, 100.0),
                        ]
                    } else {
                        [
                            Vector3::LEFT * 100.0,
                            Vector3::RIGHT * 100.0,
                            Vector3::FORWARD * 100.0,
                            Vector3::BACK * 100.0,
                        ]
                    };
                    for point in positions.into_iter().take(arms) {
                        let end = graph.add_node(point, NodeType::Junction);
                        graph.add_edge(Edge {
                            start_node: end,
                            end_node: center,
                            primary_type: TransitType::Road,
                            allowed_types: TransitFlags::CAR,
                            fwd_lanes: count,
                            bkw_lanes: count,
                            width: f32::from(count) * config::LANE_WIDTH * 2.0,
                            physical_length: point.length(),
                            geometry: vec![point, Vector3::ZERO],
                            physical_geometry: vec![point, Vector3::ZERO],
                            ..Default::default()
                        });
                    }
                    graph.rebuild_all_indices();
                    graph.rebuild_intersection_clips();
                    let mut lanes = LaneSystem::new();
                    lanes.rebuild(&mut graph);
                    for from in 0..arms {
                        let policy = DefaultLaneUse::new(&graph, from, center);
                        let mut reachable = vec![false; arms];
                        for &id in &lanes.edge_lanes[&from] {
                            let source = &lanes.lanes[id];
                            if source.lane_type != LaneType::Vehicle {
                                continue;
                            }
                            assert_eq!(
                                lanes.needs_turn_preparation(id),
                                source.is_fwd && policy.needs_lane_choice(),
                            );
                            if !source.is_fwd {
                                continue;
                            }
                            assert!(!source.next_lanes.is_empty());
                            for &conn in &source.next_lanes {
                                let target = &lanes.lanes[lanes.lanes[conn].next_lanes[0]];
                                assert_ne!(from, target.edge_id, "NET-07");
                                assert!(policy.allows(
                                    &graph,
                                    source.lane_idx,
                                    target.edge_id,
                                    target.lane_idx
                                ));
                                reachable[target.edge_id] = true;
                            }
                        }
                        for (to, reached) in reachable.into_iter().enumerate() {
                            assert_eq!(
                                reached,
                                from != to,
                                "{count} lanes, {arms} arms, skew={skew}: {from}->{to}"
                            );
                        }
                    }
                    // An explicit connection from the normally right-turning lane to the far
                    // left target still overrides defaults; all unspecified turns are blocked.
                    graph.add_lane_connection(center, 0, count as i8 - 1, 2, -1);
                    lanes.rebuild(&mut graph);
                    let connections: Vec<_> = lanes.node_lanes[&(center as usize)]
                        .iter()
                        .filter(|&&id| lanes.lanes[id].lane_type == LaneType::Vehicle)
                        .collect();
                    assert_eq!(connections.len(), 1);
                    lanes.rebuild_edges_incremental(
                        &mut graph,
                        &std::collections::HashSet::from([0]),
                    );
                    for from in 0..arms {
                        for &id in &lanes.edge_lanes[&from] {
                            let lane = &lanes.lanes[id];
                            if lane.lane_type == LaneType::Vehicle {
                                assert_eq!(
                                    lanes.needs_turn_preparation(id),
                                    lane.is_fwd && count > 1
                                );
                            }
                        }
                    }
                    lanes.clear();
                    assert!(lanes.turn_preparation.is_empty());
                }
            }
        }
    }
}
