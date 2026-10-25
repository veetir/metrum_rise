// SPDX-License-Identifier: GPL-2.0-only

//! Edit-time clearance between explicitly layered road corridors.

mod profile;

use super::span::SpanQuad;
use super::{RoadPreviewValidation, RoadSurfaceBand, RoadSurfaceSection, RoadSurfaceSystem};
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::types::TransitFlags;
use glam::{DVec2, DVec3};
use godot::prelude::Vector3;
use std::collections::HashSet;

// Reserved for the structural opening, independent of the ordinary bridge's thinner slab.
const DECK_DEPTH_M: f64 = 0.6;
const VEHICLE_CLEARANCE_M: f64 = 4.8;
const FOOT_CYCLE_CLEARANCE_M: f64 = 2.8;

impl RoadSurfaceSystem {
    /// Restores exact clearance after local junction profiling, from higher levels down.
    /// Uses actual compiled mouth widths/crossfall and the existing 12% envelope. Fixed
    /// joints that cannot clear remain unchanged and fail the normal exact validator.
    /// Work is bounded by dirty edges and their R-tree neighbors; no simulation-tick work.
    pub(crate) fn conform_layered_clearance_profiles(
        &self,
        graph: &mut RegionGraph,
        dirty: &HashSet<usize>,
    ) {
        let mut ordered = dirty
            .iter()
            .copied()
            .filter(|&id| {
                id < graph.edge_count()
                    && !graph.edge(id).deleted
                    && graph.edge(id).class
                        == crate::simulation::network::types::EdgeClass::Standard
            })
            .collect::<Vec<_>>();
        ordered.sort_unstable_by_key(|&id| (std::cmp::Reverse(graph.edge(id).crossing_level), id));
        for id in ordered {
            let level = graph.edge(id).crossing_level;
            let neighbors = self.crossing_neighbors(graph, id);
            if !neighbors
                .iter()
                .any(|&other| graph.edge(other).crossing_level > level)
            {
                continue;
            }
            let mut ignored = neighbors
                .into_iter()
                .filter(|&other| graph.edge(other).crossing_level <= level)
                .collect::<HashSet<_>>();
            ignored.insert(id);
            if self
                .crossing_clearance_violation(graph, id, graph, &ignored)
                .is_none()
            {
                continue;
            }
            let Ok(profile) = self.fit_existing_under_crossing(graph, id) else {
                continue;
            };
            graph.remove_from_spatial_index(id);
            let edge = graph.edge_mut(id);
            edge.physical_geometry = profile.points;
            let (cost, length) =
                crate::simulation::pathing::cost::CostCalculator::calculate_costs(edge);
            edge.base_cost = cost;
            edge.physical_length = length;
            graph.add_to_spatial_index(id);
        }
    }

    /// Applies the same corridor clearance guard to synchronous tool feedback.
    pub(crate) fn validate_prepared_crossing_clearance(
        &self,
        prepared: &super::PreparedRoadInput,
        fwd: u8,
        bkw: u8,
        source: &RegionGraph,
        validation: RoadPreviewValidation,
    ) -> RoadPreviewValidation {
        if !validation.is_valid || prepared.points.len() < 2 {
            return validation;
        }
        if prepared.crossing_level != 0 && validation.max_grade > 0.16 {
            return validation.with_invalid_reason("grade_separation_grade");
        }
        let mut candidate = RegionGraph::new();
        let start = candidate.add_node(
            prepared.points[0],
            crate::simulation::network::types::NodeType::Junction,
        );
        let end = candidate.add_node(
            *prepared.points.last().unwrap(),
            crate::simulation::network::types::NodeType::Junction,
        );
        let mut edge = crate::simulation::network::build_surface_edge(
            start,
            end,
            prepared.points.clone(),
            fwd,
            bkw,
            prepared.class,
        );
        edge.crossing_level = prepared.crossing_level;
        edge.median = prepared.median;
        edge.width += prepared.median.map_or(0.0, |median| median.width_m);
        let id = candidate.add_edge(edge);
        // This isolated quick graph has no source incident arms yet. Exact topology solves
        // their shared junction mouths and checks the remaining span-owned overlap below.
        let mut connected = HashSet::new();
        for &endpoint in [
            prepared.points.first().unwrap(),
            prepared.points.last().unwrap(),
        ] {
            for other in source.get_edges_near_point(endpoint, 0.001) {
                if edges_share_endpoint(&candidate, id, source, other) {
                    connected.insert(other);
                }
            }
        }
        self.crossing_clearance_violation(&candidate, id, source, &connected)
            .unwrap_or(validation)
    }

    /// Checks one edited corridor against indexed local roads, never against the resident city.
    /// `ignored` are source IDs replaced by the candidate graph (for moved-node excerpts).
    /// O(log E + K log K + P + sum(P * Q * B * B)) time and O(K + (P + Q) * B) edit-time storage;
    /// triangle clipping uses fixed stack buffers and adds no per-agent or per-tick work.
    pub(crate) fn crossing_clearance_violation(
        &self,
        candidate: &RegionGraph,
        id: usize,
        source: &RegionGraph,
        ignored: &HashSet<usize>,
    ) -> Option<RoadPreviewValidation> {
        let edge = candidate.edge(id);
        if edge.deleted || !Self::is_surface_edge(edge) || edge.physical_geometry.len() < 2 {
            return None;
        }
        let mut min = Vector3::new(f32::INFINITY, 0.0, f32::INFINITY);
        let mut max = Vector3::new(f32::NEG_INFINITY, 0.0, f32::NEG_INFINITY);
        for point in &edge.physical_geometry {
            min.x = min.x.min(point.x);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.z = max.z.max(point.z);
        }
        // The existing index stores centrelines. Tool roads are bounded to four lanes per
        // direction plus a 12 m median; the halo covers both complete roadbeds. Legacy wider
        // roads crossing the centreline are still candidates regardless of this halo.
        let halo = Self::visual_roadbed_half_width_m(edge) + 24.0;
        min -= Vector3::new(halo, 0.0, halo);
        max += Vector3::new(halo, 0.0, halo);
        let mut neighbors = source.get_edges_near_aabb(min, max);
        neighbors.retain(|other| {
            !ignored.contains(other)
                && !source.edge(*other).deleted
                && Self::is_surface_edge(source.edge(*other))
                && source.edge(*other).crossing_level != edge.crossing_level
        });
        neighbors.sort_unstable();
        if neighbors.is_empty() {
            return None;
        }
        let sections = self.compile_edge_sections(candidate, id);
        if sections.len() < 2 {
            return None;
        }
        for other in neighbors {
            let other_edge = source.edge(other);
            let other_sections = self.compile_edge_sections(source, other);
            if other_sections.len() < 2 {
                continue;
            }
            // A deliberately shared endpoint is an at-grade junction across layer labels.
            // Its node-owned pavement has no vertical-clearance requirement. Use the same
            // clipped span ranges as rendering; all other interior overlap still needs clearance.
            let shared_endpoint = edges_share_endpoint(candidate, id, source, other);
            let owned = |graph: &RegionGraph, edge_id, sections: &[RoadSurfaceSection]| {
                if shared_endpoint {
                    self.visible_corridor_index_range_for_edge(graph, edge_id, sections)
                } else {
                    Some((0, sections.len().saturating_sub(1)))
                }
            };
            let Some((a_start, a_end)) = owned(candidate, id, &sections) else {
                continue;
            };
            let Some((b_start, b_end)) = owned(source, other, &other_sections) else {
                continue;
            };
            let candidate_above = edge.crossing_level > other_edge.crossing_level;
            let lower = if candidate_above { other_edge } else { edge };
            let required = if lower.allowed_types & TransitFlags::CAR != 0 {
                VEHICLE_CLEARANCE_M
            } else {
                FOOT_CYCLE_CLEARANCE_M
            };
            for a in sections[a_start..=a_end].windows(2) {
                for b in other_sections[b_start..=b_end].windows(2) {
                    let upper = if candidate_above { a } else { b };
                    let lower = if candidate_above { b } else { a };
                    if let Some(gap) = section_pair_gap(upper, lower) {
                        let clearance = gap - DECK_DEPTH_M;
                        if clearance + 1e-5 < required {
                            let mut refusal = RoadPreviewValidation::valid(0.0)
                                .with_invalid_reason("grade_separation_clearance");
                            refusal.clearance_m = clearance as f32;
                            refusal.required_clearance_m = required as f32;
                            return Some(refusal);
                        }
                    }
                }
            }
        }
        None
    }
}

fn edges_share_endpoint(a: &RegionGraph, aid: usize, b: &RegionGraph, bid: usize) -> bool {
    let a = &a.edge(aid).physical_geometry;
    let b = &b.edge(bid).physical_geometry;
    a.first().into_iter().chain(a.last()).any(|&p| {
        b.first()
            .into_iter()
            .chain(b.last())
            .any(|&q| p.distance_squared_to(q) <= 1e-6)
    })
}

fn corridor_corners(pair: &[RoadSurfaceSection]) -> [DVec3; 4] {
    let boundary = |section: &RoadSurfaceSection, left: bool| {
        let lateral = if left {
            section.bands.first().map_or(0.0, |b| b.lateral_start_m)
        } else {
            section.bands.last().map_or(0.0, |b| b.lateral_end_m)
        };
        RoadSurfaceSystem::section_boundary_world_point_static(
            section,
            lateral,
            section.center_height_m,
        )
    };
    [
        boundary(&pair[0], true),
        boundary(&pair[1], true),
        boundary(&pair[1], false),
        boundary(&pair[0], false),
    ]
}

fn band_quad(
    pair: &[RoadSurfaceSection],
    a: &RoadSurfaceBand,
    b: &RoadSurfaceBand,
) -> Option<SpanQuad> {
    let point = RoadSurfaceSystem::section_boundary_world_point_static;
    // Same order and fan diagonal as RoadSurfaceSpanOwnedRegion::corners/quad.
    SpanQuad::from_corners([
        point(&pair[0], a.lateral_start_m, a.height_start_m),
        point(&pair[1], b.lateral_start_m, b.height_start_m),
        point(&pair[1], b.lateral_end_m, b.height_end_m),
        point(&pair[0], a.lateral_end_m, a.height_end_m),
    ])
}

/// Returns the minimum paved-band vertical gap over the sections' exact XZ overlap.
pub(super) fn section_pair_gap(
    upper: &[RoadSurfaceSection],
    lower: &[RoadSurfaceSection],
) -> Option<f64> {
    if !overlap_bounds(&corridor_corners(upper), &corridor_corners(lower)) {
        return None;
    }
    let mut gap = f64::INFINITY;
    for (a, b) in upper[0].bands.iter().zip(&upper[1].bands) {
        let Some(upper) = band_quad(upper, a, b) else {
            continue;
        };
        for (a, b) in lower[0].bands.iter().zip(&lower[1].bands) {
            let Some(lower) = band_quad(lower, a, b) else {
                continue;
            };
            if !overlap_bounds(upper.points(), lower.points()) {
                continue;
            }
            for &a in upper.triangles() {
                for &b in lower.triangles() {
                    if let Some(measured) = triangle_gap(a, b) {
                        gap = gap.min(measured);
                    }
                }
            }
        }
    }
    gap.is_finite().then_some(gap)
}

fn xz(point: DVec3) -> DVec2 {
    DVec2::new(point.x, point.z)
}

fn overlap_bounds(a: &[DVec3], b: &[DVec3]) -> bool {
    let bounds = |points: &[DVec3]| {
        points.iter().fold(
            (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY)),
            |(min, max), &p| (min.min(xz(p)), max.max(xz(p))),
        )
    };
    let (amin, amax) = bounds(a);
    let (bmin, bmax) = bounds(b);
    amin.x <= bmax.x && bmin.x <= amax.x && amin.y <= bmax.y && bmin.y <= amax.y
}

// Affine heights attain their minimum difference at a vertex of the clipped intersection.
fn triangle_gap(upper: [DVec3; 3], lower: [DVec3; 3]) -> Option<f64> {
    let (input, count) = triangle_overlap(upper, lower)?;
    let height = |triangle: [DVec3; 3], point: DVec2| {
        let a = xz(triangle[0]);
        let ab = xz(triangle[1]) - a;
        let ac = xz(triangle[2]) - a;
        let determinant = ab.perp_dot(ac);
        if determinant.abs() < 1e-10 {
            return None;
        }
        let offset = point - a;
        Some(
            triangle[0].y
                + offset.perp_dot(ac) / determinant * (triangle[1].y - triangle[0].y)
                + ab.perp_dot(offset) / determinant * (triangle[2].y - triangle[0].y),
        )
    };
    let mut gap = f64::INFINITY;
    for point in &input[..count] {
        gap = gap.min(height(upper, *point)? - height(lower, *point)?);
    }
    Some(gap)
}

/// Clips XZ triangles into a fixed stack buffer, returning its populated vertex count.
pub(super) fn triangle_overlap(
    upper: [DVec3; 3],
    lower: [DVec3; 3],
) -> Option<([DVec2; 8], usize)> {
    let mut input = [DVec2::ZERO; 8];
    input[..3].copy_from_slice(&upper.map(xz));
    let mut count = 3;
    let orientation = (xz(lower[1]) - xz(lower[0])).perp_dot(xz(lower[2]) - xz(lower[0]));
    if orientation.abs() < 1e-10 {
        return None;
    }
    for side in 0..3 {
        let a = xz(lower[side]);
        let direction = xz(lower[(side + 1) % 3]) - a;
        let signed = |p: DVec2| direction.perp_dot(p - a) * orientation.signum();
        let mut output = [DVec2::ZERO; 8];
        let mut length = 0;
        let mut append = |point| {
            if length == 0 || output[length - 1] != point {
                output[length] = point;
                length += 1;
            }
        };
        for i in 0..count {
            let p = input[i];
            let q = input[(i + 1) % count];
            let dp = signed(p);
            let dq = signed(q);
            if dp >= 0.0 {
                append(p);
            }
            if (dp >= 0.0) != (dq >= 0.0) {
                append(p.lerp(q, dp / (dp - dq)));
            }
        }
        if length > 1 && output[0] == output[length - 1] {
            length -= 1;
        }
        input = output;
        count = length;
        if count == 0 {
            return None;
        }
    }
    if count < 3 {
        return None;
    }
    let origin = input[0];
    let area = (1..count - 1)
        .map(|i| (input[i] - origin).perp_dot(input[i + 1] - origin))
        .sum::<f64>();
    // End-to-end roadbeds share an edge, not a covered corridor. This permits authored
    // straight approach connections between levels while interior overlap still needs clearance.
    if area.abs() < 1e-12 {
        return None;
    }
    Some((input, count))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::buildings::allocator::BuildingAllocator;
    use crate::simulation::core::config::WorldConfig;
    use crate::simulation::network::graph::RoadMedian;
    use crate::simulation::network::types::{EdgeClass, NodeType};
    use crate::simulation::network::{TransitNetwork, build_surface_edge, topology};
    use crate::simulation::zoning::ZoningSystem;

    fn crossing(height: f32, foot: bool) -> RegionGraph {
        let mut graph = RegionGraph::new();
        for (points, lanes, level) in [
            (
                [Vector3::new(-80.0, 0.0, 0.0), Vector3::new(80.0, 0.0, 0.0)],
                if foot { 0 } else { 1 },
                0,
            ),
            (
                [
                    Vector3::new(0.0, height, -80.0),
                    Vector3::new(0.0, height, 80.0),
                ],
                2,
                1,
            ),
        ] {
            let start = graph.add_node(points[0], NodeType::Junction);
            let end = graph.add_node(points[1], NodeType::Junction);
            let mut edge = build_surface_edge(
                start,
                end,
                points.to_vec(),
                lanes,
                lanes,
                EdgeClass::Standard,
            );
            edge.crossing_level = level;
            edge.base_cost =
                crate::simulation::pathing::cost::CostCalculator::calculate_costs(&edge).0;
            graph.add_edge(edge);
        }
        graph
    }

    #[test]
    fn explicit_layers_keep_topology_lanes_and_routing_disconnected_even_at_identical_heights() {
        for height in [0.0, 3.8, 6.0] {
            let mut graph = crossing(height, false);
            let mut net = TransitNetwork::new();
            let mut zoning = ZoningSystem::new(&WorldConfig::default());
            let mut allocator = BuildingAllocator::new();
            topology::process_intersections(&mut net, &mut graph, 1, &mut zoning, &mut allocator);
            assert_eq!((graph.edge_count(), graph.node_count()), (2, 4));
            net.lane_system.rebuild(&mut graph);
            net.rebuild_cch_and_check(&graph);
            for flags in [TransitFlags::CAR, TransitFlags::FOOT] {
                assert!(
                    net.cch_graph
                        .find_path(0, 1, usize::MAX, usize::MAX, &graph, flags)
                        .is_some()
                );
                assert!(
                    net.cch_graph
                        .find_path(2, 3, usize::MAX, usize::MAX, &graph, flags)
                        .is_some()
                );
                assert!(
                    net.cch_graph
                        .find_path(0, 2, usize::MAX, usize::MAX, &graph, flags)
                        .is_none()
                );
            }
            for lane in &net.lane_system.lanes {
                for &next in &lane.next_lanes {
                    let next = &net.lane_system.lanes[next];
                    if lane.edge_id != usize::MAX && next.edge_id == usize::MAX {
                        for &target in &next.next_lanes {
                            assert_eq!(lane.edge_id, net.lane_system.lanes[target].edge_id);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn shared_endpoint_uses_junction_ownership_but_retains_interior_clearance() {
        let surface = RoadSurfaceSystem::new(128.0);
        let mut graph = RegionGraph::new();
        let points = [
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::ZERO,
            Vector3::new(100.0, 0.0, 10.0),
        ];
        let nodes = points.map(|p| graph.add_node(p, NodeType::Junction));
        for i in 0..2 {
            let mut edge = build_surface_edge(
                nodes[i],
                nodes[i + 1],
                points[i..=i + 1].to_vec(),
                1,
                1,
                EdgeClass::Standard,
            );
            edge.crossing_level = i as i16;
            graph.add_edge(edge);
        }
        graph.rebuild_intersection_clips();
        assert!(
            surface
                .crossing_clearance_violation(&graph, 1, &graph, &HashSet::from([1]))
                .is_none()
        );

        // Sharing a junction must not exempt a second crossing farther along a looping road.
        let mut graph = RegionGraph::new();
        let start = graph.add_node(Vector3::new(-80.0, 0.0, 0.0), NodeType::Junction);
        let joint = graph.add_node(Vector3::new(0.0, 6.0, -80.0), NodeType::Junction);
        let end = graph.add_node(Vector3::new(0.0, 6.0, 80.0), NodeType::Junction);
        graph.add_edge(build_surface_edge(
            start,
            joint,
            vec![
                graph.node(start).pos,
                Vector3::new(80.0, 0.0, 0.0),
                Vector3::new(80.0, 6.0, -80.0),
                graph.node(joint).pos,
            ],
            1,
            1,
            EdgeClass::Standard,
        ));
        let mut edge = build_surface_edge(
            joint,
            end,
            vec![
                graph.node(joint).pos,
                Vector3::new(0.0, 3.0, 0.0),
                graph.node(end).pos,
            ],
            1,
            1,
            EdgeClass::Standard,
        );
        edge.crossing_level = 1;
        graph.add_edge(edge);
        graph.rebuild_intersection_clips();
        assert!(
            surface
                .crossing_clearance_violation(&graph, 1, &graph, &HashSet::from([1]))
                .is_some()
        );
    }

    #[test]
    fn same_layer_crossings_form_a_junction_regardless_of_height_and_splits_inherit_the_layer() {
        let mut graph = crossing(6.0, false);
        graph.edge_mut(0).crossing_level = 1;
        let mut net = TransitNetwork::new();
        let mut zoning = ZoningSystem::new(&WorldConfig::default());
        let mut allocator = BuildingAllocator::new();
        topology::process_intersections(&mut net, &mut graph, 1, &mut zoning, &mut allocator);
        assert_eq!((graph.edge_count(), graph.node_count()), (4, 5));
        assert!(graph.edges().iter().all(|edge| edge.crossing_level == 1));
    }

    #[test]
    fn pocket_split_guard_uses_authored_layers_instead_of_height() {
        let mut graph = crossing(3.8, false);
        graph.edge_mut(0).fwd_lanes = 2;
        graph.edge_mut(0).width = 10.5;
        graph.edge_mut(0).lane_tapers = Some(Box::new([
            Some(crate::simulation::network::graph::RoadLaneTaper {
                length_m: 20.0,
                from_fwd: 1,
                from_bkw: 1,
                from_median_m: 0.0,
                left_pocket: false,
                right_pocket: true,
            }),
            None,
        ]));
        let points = &graph.edge(1).geometry;
        assert!(crate::simulation::network::pockets::stroke_splits_pocket(
            &graph, points, 0
        ));
        assert!(!crate::simulation::network::pockets::stroke_splits_pocket(
            &graph, points, 1
        ));
        let transition = graph.edge(0).start_node;
        graph.set_node_type(transition, NodeType::LaneTransition);
        let endpoint = graph.node(transition).pos;
        assert!(crate::simulation::network::pockets::stroke_splits_pocket(
            &graph,
            &[endpoint, endpoint + Vector3::new(0.0, 3.8, -40.0)],
            1
        ));
    }

    #[test]
    fn clearance_uses_lower_transport_and_authoritative_vertical_order() {
        let surface = RoadSurfaceSystem::new(128.0);
        for (height, foot, valid) in [
            (6.0, false, true),
            (5.0, false, false),
            (3.8, true, true),
            (3.0, true, false),
            (-6.0, false, false),
        ] {
            let graph = crossing(height, foot);
            let result =
                surface.crossing_clearance_violation(&graph, 1, &graph, &HashSet::from([1]));
            assert_eq!(
                result.is_none(),
                valid,
                "height={height} foot={foot}: {result:?}"
            );
            if let Some(result) = result {
                assert_eq!(result.required_clearance_m, if foot { 2.8 } else { 4.8 });
            }
        }
    }

    #[test]
    fn clearance_checks_the_full_sloped_cross_section_including_median_and_sidewalks() {
        let surface = RoadSurfaceSystem::new(128.0);
        let mut graph = crossing(6.0, false);
        graph
            .set_edge_cross_section(0, 2, 2, RoadMedian::from_tool(2, 12.0).unwrap())
            .unwrap();
        graph.remove_from_spatial_index(1);
        // Upper road has ample clearance on the centreline but descends across the wide lower road.
        let points = vec![
            Vector3::new(0.0, -2.0, -80.0),
            Vector3::new(0.0, 14.0, 80.0),
        ];
        graph.edge_mut(1).geometry = points.clone();
        graph.edge_mut(1).physical_geometry = points;
        graph.add_to_spatial_index(1);
        let refusal = surface
            .crossing_clearance_violation(&graph, 1, &graph, &HashSet::from([1]))
            .unwrap();
        assert!(refusal.clearance_m < 4.8, "{refusal:?}");
    }

    #[test]
    fn node_move_preflight_matches_the_updated_graph_near_a_lane_taper() {
        let surface = RoadSurfaceSystem::new(128.0);
        let mut graph = crossing(6.0, false);
        graph.move_node(0, Vector3::new(-80.0, 0.0, 70.0));
        graph.move_node(1, Vector3::new(80.0, 0.0, 70.0));
        let junction = graph.edge(1).end_node;
        let end = graph.add_node(Vector3::new(0.0, 6.0, 160.0), NodeType::Junction);
        let mut continuation = build_surface_edge(
            junction,
            end,
            vec![graph.node(junction).pos, graph.node(end).pos],
            1,
            1,
            EdgeClass::Standard,
        );
        continuation.crossing_level = 1;
        graph.add_edge(continuation);
        graph.set_node_type(junction, NodeType::LaneTransition);
        graph.edge_mut(1).lane_tapers = Some(Box::new([
            None,
            Some(crate::simulation::network::graph::RoadLaneTaper {
                length_m: 30.0,
                from_fwd: 1,
                from_bkw: 1,
                from_median_m: 0.0,
                left_pocket: false,
                right_pocket: false,
            }),
        ]));
        graph.rebuild_intersection_clips();
        for x in [0.0, 4.0] {
            for y in [2.0, 4.0, 6.0, 8.0] {
                let position = Vector3::new(x, y, 80.0);
                let planned = surface.node_move_crossing_refusal(&graph, junction, position);
                let mut updated = graph.clone();
                updated.move_node(junction, position);
                updated.rebuild_intersection_clips_for_nodes(&HashSet::from([2, junction, end]));
                let actual = [1, 2].into_iter().find_map(|id| {
                    surface.crossing_clearance_violation(
                        &updated,
                        id,
                        &updated,
                        &HashSet::from([id]),
                    )
                });
                assert_eq!(
                    planned.is_some(),
                    actual.is_some(),
                    "position={position:?}: planned={planned:?}, actual={actual:?}"
                );
            }
        }
    }

    #[test]
    fn sloped_bend_clearance_uses_band_diagonals_instead_of_the_outer_footprint_diagonal() {
        use super::super::RoadSurfaceBandKind;
        let section = |center_xz, lateral_xz, y, limits: &[f32]| RoadSurfaceSection {
            edge_idx: 0,
            s_m: 0.0,
            center_xz,
            center_height_m: y,
            tangent_xz: DVec2::X,
            lateral_xz,
            bands: limits
                .windows(2)
                .map(|pair| RoadSurfaceBand {
                    kind: RoadSurfaceBandKind::Carriageway,
                    lateral_start_m: pair[0],
                    lateral_end_m: pair[1],
                    height_start_m: y,
                    height_end_m: y,
                })
                .collect(),
        };
        let upper = [
            section(DVec2::ZERO, DVec2::Y, 6.0, &[-7.0, -3.5, 0.0, 3.5, 7.0]),
            section(
                DVec2::new(12.0, 6.0),
                DVec2::new(-3.0_f64.sqrt() * 0.5, 0.5),
                7.2,
                &[-7.0, -3.5, 0.0, 3.5, 7.0],
            ),
        ];
        let lower = [
            section(DVec2::new(3.9, 5.0 / 6.0), DVec2::Y, 0.0, &[-0.1, 0.1]),
            section(DVec2::new(4.1, 5.0 / 6.0), DVec2::Y, 0.0, &[-0.1, 0.1]),
        ];
        // The rendered band is y = 6 + x / 10 here. A fan across the entire footprint
        // instead reads about 6.61 m at the centre, overstating clearance by 0.21 m.
        let measured = section_pair_gap(&upper, &lower).unwrap();
        assert!((measured - 6.39).abs() < 1e-6, "{measured}");
    }

    #[test]
    #[ignore = "manual release locality timing of the edit-time clearance guard"]
    fn benchmark_grade_separation_clearance_locality() {
        use std::hint::black_box;
        use std::time::Instant;
        let surface = RoadSurfaceSystem::new(128.0);
        for background in [0, 1_000, 10_000, 100_000] {
            let mut graph = crossing(6.0, false);
            graph
                .set_edge_cross_section(0, 2, 2, RoadMedian::from_tool(2, 5.0).unwrap())
                .unwrap();
            // Setup/index construction is outside the timed repeated validation.
            for index in 0..background {
                let a = Vector3::new(
                    1_000.0 + (index % 512) as f32 * 32.0,
                    0.0,
                    1_000.0 + (index / 512) as f32 * 32.0,
                );
                let b = a + Vector3::new(16.0, 0.0, 0.0);
                let start = graph.add_node(a, NodeType::Junction);
                let end = graph.add_node(b, NodeType::Junction);
                graph.add_edge(build_surface_edge(
                    start,
                    end,
                    vec![a, b],
                    1,
                    1,
                    EdgeClass::Standard,
                ));
            }
            assert_eq!(graph.get_edges_near_point(Vector3::ZERO, 100.0).len(), 2);
            let ignored = HashSet::from([1]);
            for _ in 0..10 {
                black_box(surface.crossing_clearance_violation(&graph, 1, &graph, &ignored));
            }
            let mut samples = Vec::with_capacity(200);
            for _ in 0..200 {
                let started = Instant::now();
                let result =
                    surface.crossing_clearance_violation(black_box(&graph), 1, &graph, &ignored);
                samples.push(started.elapsed().as_secs_f64() * 1000.0);
                assert!(result.is_none());
            }
            samples.sort_by(f64::total_cmp);
            eprintln!(
                "GRADE_CLEARANCE background={background} local_edges=2 samples=200 median_ms={:.6} p95_ms={:.6}",
                samples[100], samples[190]
            );
        }
    }

    #[test]
    fn triangle_clipping_handles_coincident_edges_and_affine_height_extrema() {
        let upper = [
            DVec3::new(0.0, 8.0, 0.0),
            DVec3::new(10.0, 9.0, 0.0),
            DVec3::new(0.0, 8.0, 10.0),
        ];
        let lower = upper.map(|p| DVec3::new(p.x, 0.0, p.z));
        assert_eq!(triangle_gap(upper, lower), Some(8.0));
        assert_eq!(
            triangle_gap(upper, [lower[2], lower[1], lower[0]]),
            Some(8.0)
        );
    }
}
