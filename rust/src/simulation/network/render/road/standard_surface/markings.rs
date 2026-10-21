// SPDX-License-Identifier: GPL-2.0-only

//! Lane-divider marking placement and crosswalk-mouth clearance.

use super::super::crosswalks::CROSSWALK_STRIPE_LEN;
use super::super::{
    MARKING_RENDER_Z_BIAS_M, MIN_SEGMENT_LEN, MeshLayer, NetworkMeshData, NetworkMeshOwner,
    marking_dash_color,
};
use super::coverage::CompiledSurfaceCoverage;
use super::geometry::{emit_surface_quad, section_world_point_at_lateral_offset};
use crate::config;
use crate::simulation::network::graph::{Edge, RegionGraph};
use crate::simulation::network::lanes::{APPROACH_SOLID_LENGTH_M, Lane, LaneSystem, LaneType};
use crate::simulation::network::surface::{
    RoadSurfaceBandKind, RoadSurfaceSection, RoadSurfaceSystem,
};
use crate::simulation::network::types::{NodeType, TransitType};
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::{Color, Vector2, Vector3};
use std::collections::{BTreeSet, HashMap};

const LANE_MARKING_CROSSWALK_CLEARANCE_M: f32 = 0.25;
// Traficom, Liite 14 (2023), K0 table: centre line K1 and lane line K2_1 are 1 m dashes with
// 3 m gaps at 50 km/h or less, and 3 m dashes with 9 m gaps above; 0.10 m wide, or 0.15 m
// (K1_3, K2_1_3) above 50 km/h. Every longitudinal line is white since the 2020 Road Traffic Act.
const FINNISH_LINE_SPEED_THRESHOLD_KMH: f32 = 50.0;
const FINNISH_LINE_WIDTH_M: f32 = 0.10;
const FINNISH_FAST_LINE_WIDTH_M: f32 = 0.15;
// Marking UV.y carries the dash pattern for road_marking.gdshader: 2 solid, then 1 m / 3 m,
// then 3 m / 9 m. Zebra paint is now part of the asphalt surface.
const MARKING_PATTERN_SHORT_DASH: f32 = 3.0;
const MARKING_PATTERN_LONG_DASH: f32 = 4.0;
// One lane each way at urban speed stays unmarked, as Finnish residential streets are. The road
// tool has no street hierarchy, so a 1+1 road at 50 km/h (its default) counts as residential.
const UNMARKED_STREET_MAX_KMH: f32 = 50.0;

#[derive(Clone, Copy)]
struct FinnishLaneLine {
    width_m: f32,
    pattern: f32,
}

impl FinnishLaneLine {
    fn for_edge(edge: &Edge) -> Option<Self> {
        let speed_kmh = (edge.speed_limit * 3.6).round();
        if edge.fwd_lanes == 1 && edge.bkw_lanes == 1 && speed_kmh <= UNMARKED_STREET_MAX_KMH {
            return None;
        }
        Some(if speed_kmh <= FINNISH_LINE_SPEED_THRESHOLD_KMH {
            Self {
                width_m: FINNISH_LINE_WIDTH_M,
                pattern: MARKING_PATTERN_SHORT_DASH,
            }
        } else {
            Self {
                width_m: FINNISH_FAST_LINE_WIDTH_M,
                pattern: MARKING_PATTERN_LONG_DASH,
            }
        })
    }
}
const CROSSWALK_MOUTH_CENTER_MATCH_TOLERANCE_M: f32 = 0.25;

pub(in crate::simulation::network::render::road) fn emit_compiled_lane_markings(
    mesh: &mut NetworkMeshData,
    graph: &RegionGraph,
    lane_system: &LaneSystem,
    road_surface: &RoadSurfaceSystem,
    terrain: &TerrainSystem,
    coverage: &CompiledSurfaceCoverage,
) {
    let crosswalk_endpoint_flags =
        lane_marking_crosswalk_endpoint_flags_by_edge(graph, lane_system, &coverage.edge_indices);
    for &edge_idx in &coverage.edge_indices {
        mesh.set_owner(NetworkMeshOwner::Edge(edge_idx));
        let edge = graph.edge(edge_idx);
        if edge.deleted || edge.primary_type != TransitType::Road {
            continue;
        }
        let total_lanes = edge.fwd_lanes as usize + edge.bkw_lanes as usize;
        emit_approach_arrows(mesh, graph, lane_system, edge_idx);
        if edge.bus_lanes != [0; 2] {
            super::bus::emit_bus_markings(
                mesh,
                graph,
                road_surface,
                terrain,
                edge_idx,
                crosswalk_endpoint_flags
                    .get(&edge_idx)
                    .copied()
                    .unwrap_or((false, false)),
            );
        }
        if total_lanes <= 1 {
            continue;
        }

        let Some(sections) = road_surface.compiled_sections().get(&edge_idx) else {
            continue;
        };
        let ranges =
            road_surface.visible_section_ranges_for_edge(graph, terrain, edge_idx, sections);
        if ranges.is_empty() {
            continue;
        }
        let marking_s_range = edge_lane_marking_s_range(
            edge,
            sections,
            crosswalk_endpoint_flags
                .get(&edge_idx)
                .copied()
                .unwrap_or((false, false)),
        );
        if marking_s_range.1 <= marking_s_range.0 + MIN_SEGMENT_LEN {
            continue;
        }

        let mut solid = (false, false);
        if let Some(ids) = lane_system.edge_lanes.get(&edge_idx) {
            for &id in ids {
                if lane_system.needs_turn_preparation(id) {
                    if lane_system.lanes[id].is_fwd {
                        solid.1 = true;
                    } else {
                        solid.0 = true;
                    }
                }
            }
        }
        let dashed_range = |start: bool, end: bool| {
            (
                if start {
                    edge.start_clip + APPROACH_SOLID_LENGTH_M
                } else {
                    marking_s_range.0
                },
                if end {
                    sections.last().map_or(0.0, |s| s.s_m) - edge.end_clip - APPROACH_SOLID_LENGTH_M
                } else {
                    marking_s_range.1
                },
            )
        };
        if edge.median.is_some() {
            let line = FinnishLaneLine {
                width_m: FINNISH_LINE_WIDTH_M,
                pattern: MARKING_PATTERN_SHORT_DASH,
            };
            let side = if config::DRIVE_ON_LEFT { -1.0 } else { 1.0 };
            for (count, sign) in [(edge.fwd_lanes, side), (edge.bkw_lanes, -side)] {
                for divider in 1..count {
                    let offset = edge.median_center_offset()
                        + sign
                            * (edge.median_half_width() + f32::from(divider) * config::LANE_WIDTH);
                    for &(start, end) in &ranges {
                        if end > start {
                            emit_lane_marking_sections(
                                mesh,
                                &sections[start..=end],
                                1,
                                1,
                                marking_s_range,
                                line,
                                Some(offset),
                                dashed_range(sign != side && solid.0, sign == side && solid.1),
                            );
                        }
                    }
                }
            }
            continue;
        }

        let Some(line) = FinnishLaneLine::for_edge(edge) else {
            continue;
        };
        for divider in 1..total_lanes {
            for (start_index, end_index) in &ranges {
                if *end_index <= *start_index {
                    continue;
                }
                emit_lane_marking_sections(
                    mesh,
                    &sections[*start_index..=*end_index],
                    divider,
                    total_lanes,
                    marking_s_range,
                    line,
                    None,
                    dashed_range(solid.0, solid.1),
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_lane_marking_sections(
    mesh: &mut NetworkMeshData,
    sections: &[RoadSurfaceSection],
    divider: usize,
    total_lanes: usize,
    marking_s_range: (f32, f32),
    line: FinnishLaneLine,
    lateral_offset_m: Option<f32>,
    dashed_range: (f32, f32),
) {
    if sections.len() < 2 {
        return;
    }

    let lane_fraction = divider as f32 / total_lanes as f32;
    for pair in sections.windows(2) {
        let segment_start_s = pair[0].s_m;
        let segment_end_s = pair[1].s_m;
        if segment_end_s <= segment_start_s + MIN_SEGMENT_LEN {
            continue;
        }
        let clipped_start_s = segment_start_s.max(marking_s_range.0);
        let clipped_end_s = segment_end_s.min(marking_s_range.1);
        if clipped_end_s <= clipped_start_s + MIN_SEGMENT_LEN {
            continue;
        }

        let Some((left_a, right_a)) = carriageway_bounds(&pair[0]) else {
            continue;
        };
        let Some((left_b, right_b)) = carriageway_bounds(&pair[1]) else {
            continue;
        };
        let lateral_a = lateral_offset_m.unwrap_or(left_a + (right_a - left_a) * lane_fraction);
        let lateral_b = lateral_offset_m.unwrap_or(left_b + (right_b - left_b) * lane_fraction);
        let Some(start) = section_world_point_at_lateral_offset(&pair[0], lateral_a) else {
            continue;
        };
        let Some(end) = section_world_point_at_lateral_offset(&pair[1], lateral_b) else {
            continue;
        };
        let mut cuts = [
            clipped_start_s,
            clipped_end_s,
            dashed_range.0.clamp(clipped_start_s, clipped_end_s),
            dashed_range.1.clamp(clipped_start_s, clipped_end_s),
        ];
        cuts.sort_by(f32::total_cmp);
        for cut in cuts.windows(2) {
            if cut[1] <= cut[0] + MIN_SEGMENT_LEN {
                continue;
            }
            let point = |s: f32| {
                start.lerp(
                    end,
                    ((s - segment_start_s) / (segment_end_s - segment_start_s)).clamp(0.0, 1.0),
                )
            };
            let middle = (cut[0] + cut[1]) * 0.5;
            emit_marking_segment(
                mesh,
                point(cut[0]),
                point(cut[1]),
                cut[0],
                cut[1],
                line.width_m * 0.5,
                if middle < dashed_range.0 || middle > dashed_range.1 {
                    2.0
                } else {
                    line.pattern
                },
                marking_dash_color(),
            );
        }
    }
}

fn arrow_mask(lane: &Lane, lanes: &LaneSystem, node_type: NodeType) -> u8 {
    let Some(input) = lane.geometry.windows(2).rev().find_map(|pair| {
        Vector2::new(pair[1].x - pair[0].x, pair[1].z - pair[0].z).try_normalized()
    }) else {
        return 0;
    };
    let mut mask = 0;
    let mut target = |id: usize| {
        let Some(out) = lanes.lanes.get(id).filter(|out| {
            out.edge_id != usize::MAX && out.edge_id != lane.edge_id && out.geometry.len() >= 2
        }) else {
            return;
        };
        let Some(output) = out.geometry.windows(2).find_map(|pair| {
            Vector2::new(pair[1].x - pair[0].x, pair[1].z - pair[0].z).try_normalized()
        }) else {
            return;
        };
        // A shallow ramp fork is a directional choice even below the ordinary junction's
        // 45-degree threshold. Collinear through lanes keep their straight arrows.
        mask |= if input.dot(output) >= std::f32::consts::FRAC_1_SQRT_2
            && (node_type != NodeType::Ramp || input.cross(output).abs() <= 1.0e-5)
        {
            2
        } else if input.cross(output) < 0.0 {
            1
        } else {
            4
        };
    };
    for &id in &lane.next_lanes {
        if lanes.lanes[id].edge_id == usize::MAX {
            for &next in &lanes.lanes[id].next_lanes {
                target(next);
            }
        } else {
            target(id);
        }
    }
    mask
}

fn lane_point(lane: &Lane, d: f32) -> Vector3 {
    let index = lane
        .cum_dist
        .partition_point(|&s| s <= d)
        .saturating_sub(1)
        .min(lane.geometry.len() - 2);
    let fraction = ((d - lane.cum_dist[index])
        / (lane.cum_dist[index + 1] - lane.cum_dist[index]).max(1e-5))
    .clamp(0.0, 1.0);
    lane.geometry[index].lerp(lane.geometry[index + 1], fraction)
}

fn emit_approach_arrows(
    mesh: &mut NetworkMeshData,
    graph: &RegionGraph,
    lanes: &LaneSystem,
    edge: usize,
) {
    let Some(ids) = lanes.edge_lanes.get(&edge) else {
        return;
    };
    for &id in ids {
        let lane = &lanes.lanes[id];
        let road = graph.edge(edge);
        let end = if lane.is_fwd {
            road.end_node
        } else {
            road.start_node
        };
        if lane.lane_type != LaneType::Vehicle || graph.node_adjacency(end).len() < 3 {
            continue;
        }
        let mask = arrow_mask(lane, lanes, graph.node(end).node_type);
        if mask == 0 {
            continue;
        }
        for back in [12.0, 42.0] {
            if lane.length < back + 8.0 {
                continue;
            }
            let center = lane_point(lane, lane.length - back);
            let Some(forward) = (lane_point(lane, lane.length - back + 1.0)
                - lane_point(lane, lane.length - back - 1.0))
            .try_normalized() else {
                continue;
            };
            let right = Vector3::new(-forward.z, 0.0, forward.x);
            let point = |x: f32, y: f32| center + right * x + forward * y;
            let mut stroke = |a: (f32, f32), b: (f32, f32), width: f32| {
                emit_marking_segment(
                    mesh,
                    point(a.0, a.1),
                    point(b.0, b.1),
                    0.0,
                    1.0,
                    width * 0.5,
                    2.0,
                    marking_dash_color(),
                );
            };
            stroke((0.0, -2.5), (0.0, 0.0), 0.20);
            if mask & 2 != 0 {
                stroke((0.0, 0.0), (0.0, 2.5), 0.20);
                stroke((-0.65, 1.55), (0.0, 2.5), 0.23);
                stroke((0.65, 1.55), (0.0, 2.5), 0.23);
            }
            for (bit, side) in [(1, -1.0), (4, 1.0)] {
                if mask & bit == 0 {
                    continue;
                }
                stroke((0.0, 0.0), (side * 1.05, 0.9), 0.20);
                stroke((side * 1.05, 0.9), (side * 1.05, 0.0), 0.23);
                stroke((side * 1.05, 0.9), (side * 0.15, 1.1), 0.23);
            }
        }
    }
}

// Shared by line paint and reserved-lane surfacing so neither obscures a zebra corridor.
pub(super) fn edge_lane_marking_s_range(
    edge: &Edge,
    sections: &[RoadSurfaceSection],
    crosswalk_endpoint_flags: (bool, bool),
) -> (f32, f32) {
    let total_s = sections.last().map_or(0.0, |section| section.s_m);
    let mut start_s: f32 = 0.0;
    let mut end_s = total_s;
    let (has_start_crosswalk, has_end_crosswalk) = crosswalk_endpoint_flags;
    let crosswalk_gap_m =
        config::CROSSWALK_INSET + CROSSWALK_STRIPE_LEN * 0.5 + LANE_MARKING_CROSSWALK_CLEARANCE_M;

    if has_start_crosswalk {
        start_s = start_s.max(edge.start_clip + crosswalk_gap_m);
    }
    if has_end_crosswalk {
        end_s = end_s.min(total_s - edge.end_clip - crosswalk_gap_m);
    }

    if edge.lane_tapers.is_some() {
        start_s = start_s.max(edge.start_clip);
        end_s = end_s.min(total_s - edge.end_clip);
    }

    (start_s.clamp(0.0, total_s), end_s.clamp(0.0, total_s))
}

fn lane_marking_crosswalk_endpoint_flags_by_edge(
    graph: &RegionGraph,
    lane_system: &LaneSystem,
    edge_indices: &[usize],
) -> HashMap<usize, (bool, bool)> {
    let mut selected_edges = BTreeSet::new();
    let mut endpoint_nodes = BTreeSet::new();
    for &edge_idx in edge_indices {
        if edge_idx >= graph.edge_count() {
            continue;
        }
        let edge = graph.edge(edge_idx);
        if edge.deleted || edge.primary_type != TransitType::Road {
            continue;
        }
        selected_edges.insert(edge_idx);
        endpoint_nodes.insert(graph.get_valid_node(edge.start_node));
        endpoint_nodes.insert(graph.get_valid_node(edge.end_node));
    }

    let mut flags_by_edge = HashMap::new();
    for node_id in endpoint_nodes {
        let Some(lane_ids) = lane_system.node_lanes.get(&(node_id as usize)) else {
            continue;
        };
        for &lane_id in lane_ids {
            let Some(lane) = lane_system.lanes.get(lane_id) else {
                continue;
            };
            let Some(crosswalk) = lane_system.crosswalk_marking(lane_id) else {
                continue;
            };
            if lane.edge_id != usize::MAX
                || lane.lane_type != LaneType::Foot
                || !selected_edges.contains(&crosswalk.edge_id)
                || crosswalk.edge_id >= graph.edge_count()
            {
                continue;
            }

            let edge = graph.edge(crosswalk.edge_id);
            if edge.deleted || edge.primary_type != TransitType::Road {
                continue;
            }
            let center = crosswalk_center_xz(crosswalk);
            let at_start = graph.get_valid_node(edge.start_node) == node_id
                && crosswalk_mouth_center(edge, true).is_some_and(|mouth| {
                    mouth.distance_to(center) <= CROSSWALK_MOUTH_CENTER_MATCH_TOLERANCE_M
                });
            let at_end = graph.get_valid_node(edge.end_node) == node_id
                && crosswalk_mouth_center(edge, false).is_some_and(|mouth| {
                    mouth.distance_to(center) <= CROSSWALK_MOUTH_CENTER_MATCH_TOLERANCE_M
                });
            if at_start || at_end {
                let endpoint_flags = flags_by_edge
                    .entry(crosswalk.edge_id)
                    .or_insert((false, false));
                endpoint_flags.0 |= at_start;
                endpoint_flags.1 |= at_end;
            }
        }
    }

    flags_by_edge
}

fn crosswalk_mouth_center(edge: &Edge, from_start: bool) -> Option<Vector2> {
    let distance_m = if from_start {
        edge.start_clip
    } else {
        edge.end_clip
    } + config::CROSSWALK_INSET;
    let point = walk_edge_geometry_from_end(&edge.geometry, distance_m, from_start)?;
    Some(Vector2::new(point.x, point.z))
}

fn crosswalk_center_xz(crosswalk: crate::simulation::network::lanes::CrosswalkMarking) -> Vector2 {
    Vector2::new(
        (crosswalk.start.x + crosswalk.end.x) * 0.5,
        (crosswalk.start.z + crosswalk.end.z) * 0.5,
    )
}

fn walk_edge_geometry_from_end(
    points: &[Vector3],
    distance_m: f32,
    from_start: bool,
) -> Option<Vector3> {
    let first = points.first().copied()?;
    let last = points.last().copied()?;
    if distance_m <= 0.0 {
        return Some(if from_start { first } else { last });
    }

    let mut remaining = distance_m;
    if from_start {
        for pair in points.windows(2) {
            let segment_len = pair[0].distance_to(pair[1]);
            if remaining <= segment_len || pair[1] == last {
                let t = if segment_len <= f32::EPSILON {
                    0.0
                } else {
                    (remaining / segment_len).clamp(0.0, 1.0)
                };
                return Some(pair[0].lerp(pair[1], t));
            }
            remaining -= segment_len;
        }
        Some(last)
    } else {
        for index in (1..points.len()).rev() {
            let start = points[index];
            let end = points[index - 1];
            let segment_len = start.distance_to(end);
            if remaining <= segment_len || index == 1 {
                let t = if segment_len <= f32::EPSILON {
                    0.0
                } else {
                    (remaining / segment_len).clamp(0.0, 1.0)
                };
                return Some(start.lerp(end, t));
            }
            remaining -= segment_len;
        }
        Some(first)
    }
}

fn carriageway_bounds(section: &RoadSurfaceSection) -> Option<(f32, f32)> {
    let mut carriageway = section
        .bands
        .iter()
        .filter(|band| band.kind == RoadSurfaceBandKind::Carriageway);
    let first_band = carriageway.next()?;
    let last_band = carriageway.next_back().unwrap_or(first_band);
    Some((first_band.lateral_start_m, last_band.lateral_end_m))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn emit_marking_segment(
    mesh: &mut NetworkMeshData,
    start: Vector3,
    end: Vector3,
    uv_start: f32,
    uv_end: f32,
    half_width: f32,
    pattern: f32,
    color: Color,
) {
    let delta = Vector2::new(end.x - start.x, end.z - start.z);
    let length = delta.length();
    if length < MIN_SEGMENT_LEN {
        return;
    }

    let tangent = delta / length;
    let side = Vector2::new(-tangent.y, tangent.x);
    let center_start = start + Vector3::new(0.0, MARKING_RENDER_Z_BIAS_M, 0.0);
    let center_end = end + Vector3::new(0.0, MARKING_RENDER_Z_BIAS_M, 0.0);
    let eo = side * half_width;
    let a_l = Vector3::new(center_start.x + eo.x, center_start.y, center_start.z + eo.y);
    let a_r = Vector3::new(center_start.x - eo.x, center_start.y, center_start.z - eo.y);
    let b_l = Vector3::new(center_end.x + eo.x, center_end.y, center_end.z + eo.y);
    let b_r = Vector3::new(center_end.x - eo.x, center_end.y, center_end.z - eo.y);
    emit_surface_quad(
        mesh,
        MeshLayer::Marking,
        [a_l, a_r, b_r, b_l],
        [
            Vector2::new(uv_start, pattern),
            Vector2::new(uv_start, pattern),
            Vector2::new(uv_end, pattern),
            Vector2::new(uv_end, pattern),
        ],
        color,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shallow_ramp_exits_keep_directional_arrows_in_both_stored_directions() {
        for sign in [1.0, -1.0] {
            let mut lanes = LaneSystem::new();
            let mut incoming = Lane {
                edge_id: 0,
                geometry: vec![Vector3::new(-10.0 * sign, 0.0, 0.0), Vector3::ZERO],
                ..Default::default()
            };
            incoming.next_lanes.push(1);
            lanes.lanes.push(incoming);
            lanes.lanes.push(Lane {
                edge_id: 1,
                geometry: vec![Vector3::ZERO, Vector3::new(10.0 * sign, 0.0, 5.0 * sign)],
                ..Default::default()
            });
            assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Junction), 2);
            assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Ramp), 4);
            lanes.lanes[1].geometry[1].z = 0.0;
            assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Ramp), 2);
        }
    }

    #[test]
    fn arrows_follow_actual_connectors_and_whitelist_removal() {
        let mut lanes = LaneSystem::new();
        let source = Lane {
            edge_id: 0,
            geometry: vec![Vector3::new(-10.0, 0.0, 0.0), Vector3::ZERO],
            ..Default::default()
        };
        lanes.lanes.push(source);
        for (id, direction) in [
            (1, Vector3::new(0.0, 0.0, -10.0)),
            (2, Vector3::new(10.0, 0.0, 0.0)),
            (3, Vector3::new(0.0, 0.0, 10.0)),
        ] {
            lanes.lanes.push(Lane {
                edge_id: id,
                geometry: vec![Vector3::ZERO, direction],
                ..Default::default()
            });
        }
        for id in 1..=3 {
            let mut connector = Lane::default();
            connector.next_lanes.push(id);
            let next = lanes.lanes.len();
            lanes.lanes.push(connector);
            lanes.lanes[0].next_lanes.push(next);
        }
        assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Junction), 7);
        lanes.lanes[0].geometry.push(Vector3::ZERO);
        lanes.lanes[1].geometry.insert(0, Vector3::ZERO);
        assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Junction), 7);
        lanes.lanes[0].next_lanes.clear();
        lanes.lanes[0].next_lanes.push(5);
        assert_eq!(arrow_mask(&lanes.lanes[0], &lanes, NodeType::Junction), 2);
    }
}
