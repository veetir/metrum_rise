// SPDX-License-Identifier: GPL-2.0-only

//! Rebuild-time vehicle trajectory invariants, including the Prisma pocket approach.

use super::*;
use crate::simulation::network::graph::{MedianStyle, RoadMedian};
use crate::simulation::network::pockets::{
    PocketLayout, PocketRequest, author_lane_count_transitions,
};

fn median() -> RoadMedian {
    RoadMedian {
        style: MedianStyle::KerbedGrass,
        width_m: 5.0,
        start_setback_m: 0.0,
        end_setback_m: 0.0,
    }
}

fn arm(graph: &mut RegionGraph, a: u32, b: u32, count: u8, divided: bool) -> usize {
    let id = add_road_edge(graph, a, b);
    graph
        .set_edge_cross_section(id, count, count, divided.then(median))
        .unwrap();
    id
}

fn flat(p: Vector3) -> Vector2 {
    Vector2::new(p.x, p.z)
}

fn cross(a: Vector2, b: Vector2) -> f32 {
    a.x * b.y - a.y * b.x
}

fn check_connectors(graph: &mut RegionGraph) -> (usize, usize) {
    graph.rebuild_intersection_clips();
    let mut lanes = LaneSystem::new();
    lanes.rebuild(graph);
    let mut turns = 0;
    let mut shifts = 0;
    for source in lanes
        .lanes
        .iter()
        .filter(|l| l.edge_id != usize::MAX && l.lane_type == LaneType::Vehicle)
    {
        let incoming = flat(
            source.geometry[source.geometry.len() - 1] - source.geometry[source.geometry.len() - 2],
        )
        .normalized();
        for &next in &source.next_lanes {
            let connector = &lanes.lanes[next];
            if connector.edge_id != usize::MAX {
                continue;
            }
            let target = &lanes.lanes[connector.next_lanes[0]];
            // U-turns need a separate loop construction and are outside single-turn invariants.
            if target.edge_id == source.edge_id {
                continue;
            }
            let outgoing = flat(target.geometry[1] - target.geometry[0]).normalized();
            let start = flat(connector.geometry[0]);
            let end = flat(*connector.geometry.last().unwrap());
            let delta = end - start;
            let determinant = cross(incoming, outgoing);
            let single_turn = determinant.abs() > 0.05;
            let bound = if single_turn {
                turns += 1;
                let a = cross(delta, outgoing) / determinant;
                let b = cross(incoming, delta) / determinant;
                assert!(
                    a > 0.0 && b > 0.0,
                    "turn mouths must bracket tangent intersection: {a}, {b}"
                );
                // The largest tangent circle fitting these mouths determines the required radius.
                let half = incoming.dot(outgoing).clamp(-1.0, 1.0).acos() * 0.5;
                let mut bound = half.tan() / a.min(b);
                if graph.edge(source.edge_id).median.is_some()
                    && graph.median_continues_at_node(source.edge_id, connector.node_id as u32)
                {
                    let clearance = |lane: &super::super::Lane| {
                        let rank = if lane.lane_idx >= 0 {
                            i16::from(lane.lane_idx)
                        } else {
                            -i16::from(lane.lane_idx) - 1
                        };
                        (f32::from(rank) + 0.5) * crate::config::LANE_WIDTH * 0.5
                    };
                    bound =
                        bound.max((1.0 - half.cos()) / clearance(source).min(clearance(target)));
                }
                bound
            } else {
                shifts += 1;
                let longitudinal = delta.dot(incoming).abs();
                let lateral = cross(incoming, delta).abs();
                12.0 * lateral / longitudinal.max(1.0).powi(2) + 0.005
            };
            let mut directions = Vec::with_capacity(connector.geometry.len() + 1);
            directions.push(incoming);
            for pair in connector.geometry.windows(2) {
                directions.push(flat(pair[1] - pair[0]));
            }
            directions.push(outgoing);
            assert!(
                incoming.dot(directions[1].normalized()) > 0.99,
                "entry tangent discontinuity {next}"
            );
            assert!(
                outgoing.dot(directions[directions.len() - 2].normalized()) > 0.99,
                "exit tangent discontinuity {next}"
            );
            // Exclude synthetic one-metre tangent extensions from curvature magnitude checks.
            for (index, pair) in directions.windows(2).enumerate() {
                let (a, b) = (pair[0], pair[1]);
                let signed = cross(a, b);
                if single_turn {
                    assert!(
                        signed * determinant.signum() >= -0.0001,
                        "counter-steer on connector {next}, edges {} -> {}, sample {index}, cross {signed}, turn {determinant}",
                        source.edge_id,
                        target.edge_id
                    );
                }
                if index > 0 && index + 2 < directions.len() {
                    let curvature =
                        2.0 * signed.abs() / (a.length() * b.length() * (a + b).length());
                    assert!(
                        curvature <= bound + 0.001,
                        "curvature spike {curvature} > {bound} on connector {next}, edges {} -> {}",
                        source.edge_id,
                        target.edge_id
                    );
                }
            }
        }
    }
    (turns, shifts)
}

#[test]
fn junction_trajectories_are_tangent_and_have_no_countersteer() {
    for divided in [false, true] {
        for count in 1..=4 {
            for (arms, skew) in [(3, 0.0), (4, 0.0), (3, 65.0), (4, 65.0)] {
                let mut graph = RegionGraph::new();
                let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
                for p in [
                    Vector3::new(-240.0, 0.0, 0.0),
                    Vector3::new(240.0, 0.0, 0.0),
                    Vector3::new(skew, 0.0, 180.0),
                    Vector3::new(-skew, 0.0, -180.0),
                ]
                .into_iter()
                .take(arms)
                {
                    let node = graph.add_node(p, NodeType::Junction);
                    arm(&mut graph, node, center, count, divided);
                }
                let (turns, straight) = check_connectors(&mut graph);
                assert!(turns > 0 && straight > 0);
            }
        }
    }
}

#[test]
fn prisma_pockets_and_transition_trajectories_have_bounded_curvature() {
    let mut graph = RegionGraph::new();
    let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
    for (p, count, divided) in [
        (Vector3::new(-240.0, 0.0, 0.0), 2, true),
        (Vector3::new(240.0, 0.0, 0.0), 2, true),
        (Vector3::new(0.0, 0.0, 180.0), 1, false),
        (Vector3::new(0.0, 0.0, -180.0), 1, false),
    ] {
        let node = graph.add_node(p, NodeType::Junction);
        arm(&mut graph, node, center, count, divided);
    }
    graph.rebuild_intersection_clips();
    PocketLayout::prepare(
        &graph,
        PocketRequest {
            edge: 0,
            at_start: false,
            left: true,
            right: true,
            length: 80.0,
            taper: 35.0,
        },
    )
    .unwrap()
    .apply(&mut graph);
    let (turns, shifts) = check_connectors(&mut graph);
    assert!(turns > 0 && shifts > 0);
}

#[test]
fn lane_gain_drop_trajectories_are_tangent_without_curvature_spikes() {
    for divided in [false, true] {
        for (left, right) in [(1, 2), (2, 1), (1, 4), (4, 2)] {
            let mut graph = RegionGraph::new();
            let a = graph.add_node(Vector3::new(-180.0, 0.0, 0.0), NodeType::Junction);
            let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
            let b = graph.add_node(Vector3::new(180.0, 0.0, 0.0), NodeType::Junction);
            arm(&mut graph, a, center, left, divided);
            arm(&mut graph, center, b, right, divided);
            author_lane_count_transitions(
                &mut graph,
                &HashSet::from([center]),
                &mut HashSet::new(),
            );
            assert!(check_connectors(&mut graph).1 > 0);
        }
    }
}

#[test]
fn divided_bend_trajectories_remain_tangent() {
    for degrees in [15.0_f32, 30.0, 60.0, 90.0] {
        let angle = degrees.to_radians();
        let mut graph = RegionGraph::new();
        let a = graph.add_node(Vector3::new(-180.0, 0.0, 0.0), NodeType::Junction);
        let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
        let b = graph.add_node(
            Vector3::new(180.0 * angle.cos(), 0.0, 180.0 * angle.sin()),
            NodeType::Junction,
        );
        arm(&mut graph, a, center, 2, true);
        arm(&mut graph, center, b, 2, true);
        assert!(check_connectors(&mut graph).0 > 0);
    }
}
