// SPDX-License-Identifier: GPL-2.0-only

//! Vehicle junction connectors, including continuous divided bend corridors.

use super::super::graph::RegionGraph;
use super::super::types::TransitFlags;
use super::geometry::build_cum_dist;
use super::lane_use::DefaultLaneUse;
use super::{Lane, LaneSuccessors, LaneType};
use crate::config;
use godot::prelude::*;
use std::collections::HashMap;

const VEHICLE_CONNECTION_MAX_STEP_M: f32 = 1.0;
const VEHICLE_CONNECTION_MIN_STEPS: usize = 8;
const VEHICLE_CONNECTION_MAX_STEPS: usize = 64;
const VEHICLE_CONNECTION_CONTROL_DISTANCE_FACTOR: f32 = 0.35;
const VEHICLE_TRUE_JUNCTION_CONTROL_DISTANCE_FACTOR: f32 = 0.55;
const VEHICLE_DIRECT_PASS_THROUGH_EPS_M: f32 = 0.05;
const VEHICLE_DIRECT_PASS_THROUGH_DOT: f32 = 0.98;

/// Builds vehicle intersection connection lanes at a single node.
pub fn build_vehicle_connections_at_node(
    lanes: &mut Vec<Lane>,
    lane_map: &HashMap<(usize, bool, i8), usize>,
    graph: &RegionGraph,
    node_id: usize,
    node_lanes: &mut std::collections::HashMap<usize, Vec<usize>>,
    turn_preparation: &mut Vec<bool>,
) {
    let mut inbound: Vec<(usize, i8, usize)> = Vec::new();
    let mut outbound: Vec<(usize, i8, usize)> = Vec::new();
    // Arms that carry vehicle lanes. Footpaths and other lane-less arms join the node for
    // pedestrians only: a road continuing past a footpath is a pass-through for cars.
    let mut vehicle_deg = 0;

    for &e_idx in graph.node_adjacency(node_id as u32) {
        let edge = graph.edge(e_idx);
        if edge.deleted {
            continue;
        }
        if edge.fwd_lanes + edge.bkw_lanes > 0 {
            vehicle_deg += 1;
        }

        if edge.start_node as usize == node_id {
            for l in 0..edge.fwd_lanes {
                if let Some(&lid) = lane_map.get(&(e_idx, true, l as i8)) {
                    outbound.push((e_idx, l as i8, lid));
                }
            }
            for l in 0..edge.bkw_lanes {
                if let Some(&lid) = lane_map.get(&(e_idx, false, -(l as i8) - 1)) {
                    inbound.push((e_idx, -(l as i8) - 1, lid));
                }
            }
        }

        if edge.end_node as usize == node_id {
            for l in 0..edge.fwd_lanes {
                if let Some(&lid) = lane_map.get(&(e_idx, true, l as i8)) {
                    inbound.push((e_idx, l as i8, lid));
                }
            }
            for l in 0..edge.bkw_lanes {
                if let Some(&lid) = lane_map.get(&(e_idx, false, -(l as i8) - 1)) {
                    outbound.push((e_idx, -(l as i8) - 1, lid));
                }
            }
        }
    }

    let lane_conns = &graph.node(node_id as u32).lane_connections;
    let node_has_any_vehicle_conn = node_whitelists_vehicle_turns(graph, node_id);
    let node_allows_direct_pass_through = vehicle_deg == 2 && !node_has_any_vehicle_conn;
    let node_uses_junction_span = vehicle_deg >= 3 || node_has_any_vehicle_conn;

    turn_preparation.resize(lanes.len(), false);
    for &(in_edge_id, in_lane_idx, in_lane_id) in &inbound {
        let explicit = lane_conns.get(&(in_edge_id, in_lane_idx));
        let defaults = (!node_has_any_vehicle_conn)
            .then(|| DefaultLaneUse::new(graph, in_edge_id, node_id as u32));
        turn_preparation[in_lane_id] = defaults.as_ref().map_or_else(
            || {
                let lane = &lanes[in_lane_id];
                let edge = graph.edge(in_edge_id);
                (if lane.is_fwd {
                    edge.fwd_lanes
                } else {
                    edge.bkw_lanes
                }) > 1
            },
            DefaultLaneUse::needs_lane_choice,
        );
        let allows_u_turn = graph.open_node_allows_u_turn(node_id as u32);
        for &(out_edge_id, out_lane_idx, out_lid) in &outbound {
            let allowed = if let Some(defaults) = &defaults {
                if out_edge_id == in_edge_id {
                    allows_u_turn
                } else {
                    defaults.allows(graph, in_lane_idx, out_edge_id, out_lane_idx)
                }
            } else {
                explicit.is_some_and(|rules| rules.contains(&(out_edge_id, out_lane_idx)))
            };
            if !allowed {
                continue;
            }
            let p0 = *lanes[in_lane_id].geometry.last().unwrap();
            let p1_base = {
                let g = &lanes[in_lane_id].geometry;
                if g.len() >= 2 {
                    let d = g[g.len() - 1] - g[g.len() - 2];
                    if d.length_squared() > 0.00001 {
                        d.normalized()
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    }
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                }
            };
            let p3 = lanes[out_lid].geometry[0];
            let p2_base = {
                let g = &lanes[out_lid].geometry;
                if g.len() >= 2 {
                    let d = g[1] - g[0];
                    if d.length_squared() > 0.00001 {
                        d.normalized()
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    }
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                }
            };

            let dist = p0.distance_to(p3);
            // On the inside of a slight bend the offset lanes overlap: the outgoing lane starts
            // behind the incoming lane's end, and any connector between them would run backwards.
            let overlaps = (p3.x - p0.x) * p1_base.x + (p3.z - p0.z) * p1_base.z <= 0.0;
            if node_allows_direct_pass_through
                && (dist <= VEHICLE_DIRECT_PASS_THROUGH_EPS_M || overlaps)
                && flat_direction_dot(p1_base, p2_base) >= VEHICLE_DIRECT_PASS_THROUGH_DOT
            {
                lanes[in_lane_id].next_lanes.push(out_lid);
                continue;
            }

            let distance_basis = vehicle_connection_distance_basis_m(
                graph,
                node_id,
                node_uses_junction_span,
                dist,
                p0,
                p3,
            );
            let control_factor = if node_uses_junction_span {
                VEHICLE_TRUE_JUNCTION_CONTROL_DISTANCE_FACTOR
            } else {
                VEHICLE_CONNECTION_CONTROL_DISTANCE_FACTOR
            };
            let cd = distance_basis * control_factor;
            let p1 = p0 + p1_base * cd;
            let p2 = p3 - p2_base * cd;

            let steps = vehicle_connection_steps_for_curve(
                p0,
                p1,
                p2,
                p3,
                distance_basis.max(dist + cd * 2.0),
            );
            let mut conn_geom = Vec::with_capacity(steps + 1);
            let mut conn_len = 0.0;
            let median_bend = graph.edge(in_edge_id).median.is_some()
                && graph.median_continues_at_node(in_edge_id, node_id as u32);
            let divided_frame = median_bend.then(|| {
                MedianBendFrame::new(
                    graph,
                    node_id,
                    &lanes[in_lane_id],
                    &lanes[out_lid],
                    p1_base,
                    p2_base,
                )
            });
            for k in 0..=steps {
                let t = k as f32 / steps as f32;
                let point = divided_frame.as_ref().map_or_else(
                    || vehicle_connection_point(p0, p1, p2, p3, t),
                    |frame| frame.point(t),
                );
                conn_geom.push(point);
                if k > 0 {
                    conn_len += conn_geom[k - 1].distance_to(point);
                }
            }

            let conn_cum = build_cum_dist(&conn_geom);
            let conn_id = lanes.len();
            lanes.push(Lane {
                edge_id: usize::MAX,
                is_fwd: true,
                lane_idx: 0,
                geometry: conn_geom,
                length: conn_len,
                frontage_delay_penalty_s: 0.0,
                live_frontage_delay_s: 0.0,
                cum_dist: conn_cum,
                lane_type: LaneType::Vehicle,
                next_lanes: LaneSuccessors::one(out_lid),
                node_id,
            });
            node_lanes.entry(node_id).or_default().push(conn_id);
            lanes[in_lane_id].next_lanes.push(conn_id);
        }
    }
}

// Two offset approach ribbons meet on the same bisector cross-section as the median. A
// single cubic can cut the island on the inside of a bend. This construction keeps the lane
// offset throughout each half, including lane changes between the two directional groups.
struct MedianBendFrame {
    node: Vector3,
    start: Vector3,
    end: Vector3,
    start_normal: Vector3,
    mid_normal: Vector3,
    end_normal: Vector3,
    start_offset: f32,
    end_offset: f32,
}

impl MedianBendFrame {
    fn new(
        graph: &RegionGraph,
        node: usize,
        inbound: &Lane,
        outbound: &Lane,
        incoming: Vector3,
        outgoing: Vector3,
    ) -> Self {
        let normal = |direction: Vector3| Vector3::new(-direction.z, 0.0, direction.x).normalized();
        let offset = |lane: &Lane| {
            let edge = graph.edge(lane.edge_id);
            let index = if lane.lane_idx >= 0 {
                lane.lane_idx
            } else {
                -lane.lane_idx - 1
            };
            let side = if config::DRIVE_ON_LEFT { -1.0 } else { 1.0 };
            edge.median_center_offset() * if lane.is_fwd { 1.0 } else { -1.0 }
                + side * (edge.median_half_width() + (f32::from(index) + 0.5) * config::LANE_WIDTH)
        };
        let start_normal = normal(incoming);
        let end_normal = normal(outgoing);
        Self {
            node: graph.node(node as u32).pos,
            start: *inbound.geometry.last().unwrap(),
            end: outbound.geometry[0],
            start_normal,
            mid_normal: (start_normal + end_normal).normalized(),
            end_normal,
            start_offset: offset(inbound),
            end_offset: offset(outbound),
        }
    }

    fn point(&self, t: f32) -> Vector3 {
        let middle_offset = (self.start_offset + self.end_offset) * 0.5;
        let (a, b, normal_a, normal_b, offset_a, offset_b, u) = if t <= 0.5 {
            (
                self.start - self.start_normal * self.start_offset,
                self.node,
                self.start_normal,
                self.mid_normal,
                self.start_offset,
                middle_offset,
                t * 2.0,
            )
        } else {
            (
                self.node,
                self.end - self.end_normal * self.end_offset,
                self.mid_normal,
                self.end_normal,
                middle_offset,
                self.end_offset,
                t * 2.0 - 1.0,
            )
        };
        let normal = normal_a.lerp(normal_b, u).normalized();
        let mut point = a.lerp(b, u) + normal * (offset_a + (offset_b - offset_a) * u);
        point.y = self.start.y + (self.end.y - self.start.y) * t;
        point
    }
}

/// Returns `true` when the node has a user vehicle lane connection, which puts every turn at
/// the node in whitelist mode. Pedestrian rules (`RegionGraph::is_vehicle_turn_rule`: sidewalk
/// lane `±100` or a footpath edge) never do.
pub(crate) fn node_whitelists_vehicle_turns(graph: &RegionGraph, node_id: usize) -> bool {
    graph
        .node(node_id as u32)
        .lane_connections
        .keys()
        .any(|&(edge_idx, lane_idx)| graph.is_vehicle_turn_rule(edge_idx, lane_idx))
}

fn vehicle_connection_distance_basis_m(
    graph: &RegionGraph,
    node_id: usize,
    use_junction_span: bool,
    chord_distance_m: f32,
    start: Vector3,
    end: Vector3,
) -> f32 {
    if !use_junction_span {
        return chord_distance_m;
    }

    let node_pos = graph.node(node_id as u32).pos;
    let center_span_m = start.distance_to(node_pos) + end.distance_to(node_pos);
    chord_distance_m
        .max(center_span_m)
        .max(node_vehicle_junction_span_m(graph, node_id))
}

fn node_vehicle_junction_span_m(graph: &RegionGraph, node_id: usize) -> f32 {
    let mut max_half_width_m = config::LANE_WIDTH;
    for &edge_id in graph.node_adjacency(node_id as u32) {
        let edge = graph.edge(edge_id);
        if edge.deleted {
            continue;
        }
        let sidewalk_width_m = if (edge.allowed_types & TransitFlags::FOOT) != 0 {
            config::SIDEWALK_WIDTH
        } else {
            0.0
        };
        let half_width_m = edge.width.max(config::LANE_WIDTH) * 0.5 + sidewalk_width_m;
        max_half_width_m = max_half_width_m.max(half_width_m);
    }

    (max_half_width_m * 2.0).max(config::LANE_WIDTH * 3.0)
}

fn vehicle_connection_steps(chord_distance_m: f32) -> usize {
    ((chord_distance_m / VEHICLE_CONNECTION_MAX_STEP_M).ceil() as usize)
        .clamp(VEHICLE_CONNECTION_MIN_STEPS, VEHICLE_CONNECTION_MAX_STEPS)
}

fn vehicle_connection_steps_for_curve(
    p0: Vector3,
    p1: Vector3,
    p2: Vector3,
    p3: Vector3,
    distance_basis_m: f32,
) -> usize {
    let mut steps = vehicle_connection_steps(distance_basis_m);
    loop {
        if steps >= VEHICLE_CONNECTION_MAX_STEPS
            || vehicle_connection_max_sample_chord_m(p0, p1, p2, p3, steps)
                <= VEHICLE_CONNECTION_MAX_STEP_M + 1.0e-4
        {
            return steps;
        }
        steps = (steps * 2).min(VEHICLE_CONNECTION_MAX_STEPS);
    }
}

fn vehicle_connection_max_sample_chord_m(
    p0: Vector3,
    p1: Vector3,
    p2: Vector3,
    p3: Vector3,
    steps: usize,
) -> f32 {
    let mut max_chord = 0.0_f32;
    let mut prev = vehicle_connection_point(p0, p1, p2, p3, 0.0);
    for k in 1..=steps {
        let t = k as f32 / steps as f32;
        let point = vehicle_connection_point(p0, p1, p2, p3, t);
        max_chord = max_chord.max(prev.distance_to(point));
        prev = point;
    }
    max_chord
}

fn vehicle_connection_point(p0: Vector3, p1: Vector3, p2: Vector3, p3: Vector3, t: f32) -> Vector3 {
    let mut point = (1.0 - t).powi(3) * p0
        + 3.0 * (1.0 - t).powi(2) * t * p1
        + 3.0 * (1.0 - t) * t.powi(2) * p2
        + t.powi(3) * p3;
    point.y = p0.y + (p3.y - p0.y) * t;
    point
}

fn flat_direction_dot(a: Vector3, b: Vector3) -> f32 {
    let a = Vector2::new(a.x, a.z);
    let b = Vector2::new(b.x, b.z);
    if a.length_squared() <= 1.0e-8 || b.length_squared() <= 1.0e-8 {
        return -1.0;
    }
    a.normalized().dot(b.normalized()).clamp(-1.0, 1.0)
}
