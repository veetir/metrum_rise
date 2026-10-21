// SPDX-License-Identifier: GPL-2.0-only

//! Reserved-lane red surfacing and BUS lettering on the authoritative sampled road ribbon.

use super::super::{MeshLayer, NetworkMeshData};
use super::geometry::{emit_surface_quad, section_world_point_at_lateral_offset};
use super::markings::{edge_lane_marking_s_range, emit_marking_segment};
use crate::config;
use crate::simulation::network::graph::{LaneUse, RegionGraph};
use crate::simulation::network::surface::{RoadSurfaceSection, RoadSurfaceSystem};
use crate::simulation::terrain::TerrainSystem;
use godot::prelude::{Color, Vector2, Vector3};

pub(super) fn emit_bus_markings(
    mesh: &mut NetworkMeshData,
    graph: &RegionGraph,
    surface: &RoadSurfaceSystem,
    terrain: &TerrainSystem,
    id: usize,
    crosswalks: (bool, bool),
) {
    let edge = graph.edge(id);
    if edge.bus_lanes == [0; 2] {
        return;
    }
    let Some(sections) = surface.compiled_sections().get(&id) else {
        return;
    };
    let marking_range = edge_lane_marking_s_range(edge, sections, crosswalks);
    let ranges = surface.visible_section_ranges_for_edge(graph, terrain, id, sections);
    let side = if config::DRIVE_ON_LEFT { -1.0 } else { 1.0 };
    for (is_fwd, count, sign) in [(true, edge.fwd_lanes, side), (false, edge.bkw_lanes, -side)] {
        for rank in 0..count {
            if edge.lane_use(is_fwd, rank) != LaneUse::BusOnly {
                continue;
            }
            let offset = edge.median_center_offset()
                + sign * (edge.median_half_width() + (f32::from(rank) + 0.5) * config::LANE_WIDTH);
            for &(first, last) in &ranges {
                if last <= first {
                    continue;
                }
                let span = &sections[first..=last];
                let first_s = span[0].s_m.max(marking_range.0);
                let last_s = span.last().unwrap().s_m.min(marking_range.1);
                if last_s <= first_s {
                    continue;
                }
                for pair in span.windows(2) {
                    let start_s = pair[0].s_m.max(first_s);
                    let end_s = pair[1].s_m.min(last_s);
                    if end_s <= start_s {
                        continue;
                    }
                    let fractions =
                        [start_s, end_s].map(|s| (s - pair[0].s_m) / (pair[1].s_m - pair[0].s_m));
                    let corners = [(0, -1.0), (0, 1.0), (1, 1.0), (1, -1.0)].map(|(end, side)| {
                        let lateral = offset + side * (config::LANE_WIDTH * 0.5 - 0.08);
                        section_world_point_at_lateral_offset(&pair[0], lateral)
                            .zip(section_world_point_at_lateral_offset(&pair[1], lateral))
                            .map(|(a, b)| a.lerp(b, fractions[end]))
                    });
                    if let [Some(a), Some(b), Some(c), Some(d)] = corners {
                        emit_surface_quad(
                            mesh,
                            MeshLayer::Marking,
                            [a, b, c, d].map(|p| p + Vector3::UP * 0.009),
                            [Vector2::new(0.0, 2.0); 4],
                            Color::from_rgba(0.48, 0.09, 0.055, 1.0),
                        );
                    }
                }
                // Stations are edge-relative and independent of chunk residency or frame order.
                let begin = ((first_s + 8.0) / 60.0).ceil() as usize;
                let end = ((last_s - 8.0) / 60.0).floor() as usize;
                if begin.max(1) > end && last_s - first_s >= 16.0 {
                    emit_bus_word(
                        mesh,
                        span,
                        (first_s + last_s) * 0.5,
                        offset,
                        if is_fwd { 1.0 } else { -1.0 },
                    );
                }
                for station in begin.max(1)..=end {
                    let s = station as f32 * 60.0;
                    if s < first_s + 8.0 || s > last_s - 8.0 {
                        continue;
                    }
                    emit_bus_word(mesh, span, s, offset, if is_fwd { 1.0 } else { -1.0 });
                }
            }
        }
    }
}

fn point(sections: &[RoadSurfaceSection], s: f32, offset: f32) -> Option<Vector3> {
    let index = sections
        .partition_point(|section| section.s_m < s)
        .clamp(1, sections.len() - 1);
    let a = &sections[index - 1];
    let b = &sections[index];
    let t = ((s - a.s_m) / (b.s_m - a.s_m)).clamp(0.0, 1.0);
    Some(
        section_world_point_at_lateral_offset(a, offset)?
            .lerp(section_world_point_at_lateral_offset(b, offset)?, t)
            + Vector3::UP * 0.004,
    )
}

fn emit_bus_word(
    mesh: &mut NetworkMeshData,
    sections: &[RoadSurfaceSection],
    s: f32,
    offset: f32,
    direction: f32,
) {
    // A compact stroke alphabet, across the lane, elongated along the direction of travel.
    const B: &[(f32, f32, f32, f32)] = &[
        (0., 0., 0., 1.),
        (0., 1., 0.7, 1.),
        (0.7, 1., 0.7, 0.5),
        (0., 0.5, 0.7, 0.5),
        (0.7, 0.5, 0.7, 0.),
        (0., 0., 0.7, 0.),
    ];
    const U: &[(f32, f32, f32, f32)] = &[(0., 1., 0., 0.), (0., 0., 0.7, 0.), (0.7, 0., 0.7, 1.)];
    const S: &[(f32, f32, f32, f32)] = &[
        (0.7, 1., 0., 1.),
        (0., 1., 0., 0.5),
        (0., 0.5, 0.7, 0.5),
        (0.7, 0.5, 0.7, 0.),
        (0.7, 0., 0., 0.),
    ];
    for (letter, strokes) in [B, U, S].into_iter().enumerate() {
        for &(ax, ay, bx, by) in strokes {
            let p = |x: f32, y: f32| {
                point(
                    sections,
                    s + direction * (y - 0.5) * 3.5,
                    offset + direction * (letter as f32 * 0.9 + x - 1.25),
                )
            };
            if let (Some(a), Some(b)) = (p(ax, ay), p(bx, by)) {
                emit_marking_segment(mesh, a, b, 0.0, 1.0, 0.075, 2.0, Color::WHITE);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::graph::Edge;
    use crate::simulation::network::lanes::LaneSystem;
    use crate::simulation::network::render::road::RoadRenderer;
    use crate::simulation::network::types::{NodeType, TransitFlags, TransitType};

    #[test]
    fn bus_paint_leaves_the_actual_zebra_corridors_clear() {
        let mut graph = RegionGraph::new();
        let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
        for pos in [
            Vector3::LEFT,
            Vector3::RIGHT,
            Vector3::FORWARD,
            Vector3::BACK,
        ] {
            let pos = pos * 100.0;
            let end = graph.add_node(pos, NodeType::Junction);
            let points = vec![Vector3::ZERO, pos];
            graph.add_edge(Edge {
                start_node: center,
                end_node: end,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                fwd_lanes: 2,
                bkw_lanes: 2,
                bus_lanes: [2, 2],
                width: 14.0,
                physical_length: 100.0,
                speed_limit: 14.0,
                geometry: points.clone(),
                physical_geometry: points,
                ..Default::default()
            });
        }
        graph.rebuild_intersection_clips();
        let terrain = TerrainSystem::with_chunking(257, 257, 1.0, 32, 0.0);
        let mut surface = RoadSurfaceSystem::new(128.0);
        surface.compile_dirty(&graph, &terrain);
        let mut lanes = LaneSystem::new();
        lanes.rebuild(&mut graph);
        let mesh = RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
        let crossings: Vec<_> = (0..lanes.lanes.len())
            .filter_map(|id| lanes.crosswalk_marking(id))
            .collect();
        assert!(crossings.len() >= 4);
        for crossing in crossings {
            for t in [0.125, 0.875] {
                let sample = crossing.start.lerp(crossing.end, t);
                let sample = Vector2::new(sample.x, sample.z);
                for (triangle, colors) in mesh
                    .marking_vertices
                    .chunks_exact(3)
                    .zip(mesh.marking_colors.chunks_exact(3))
                {
                    if colors[0].r <= colors[0].g * 3.0 {
                        continue;
                    }
                    let p = [triangle[0], triangle[1], triangle[2]].map(|v| Vector2::new(v.x, v.z));
                    let sides = [0, 1, 2].map(|i| (p[(i + 1) % 3] - p[i]).cross(sample - p[i]));
                    assert!(
                        !(sides.iter().all(|&s| s >= -1e-4) || sides.iter().all(|&s| s <= 1e-4)),
                        "Bus surfacing covers zebra on edge {} at {sample:?}",
                        crossing.edge_id
                    );
                }
            }
        }
    }

    #[test]
    fn bus_paint_is_deterministic_and_leaves_solved_roadbed_unchanged() {
        for length in [35., 200.] {
            let mut graph = RegionGraph::new();
            let start = graph.add_node(Vector3::new(-length * 0.5, 0., 0.), NodeType::Junction);
            let end = graph.add_node(Vector3::new(length * 0.5, 0., 0.), NodeType::Junction);
            let points = vec![graph.node(start).pos, graph.node(end).pos];
            let id = graph.add_edge(Edge {
                start_node: start,
                end_node: end,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                width: 14.,
                fwd_lanes: 2,
                bkw_lanes: 2,
                physical_length: length,
                speed_limit: 14.,
                geometry: points.clone(),
                physical_geometry: points,
                ..Default::default()
            });
            graph.rebuild_intersection_clips();
            let terrain = TerrainSystem::with_chunking(257, 257, 1., 32, 0.);
            let mut surface = RoadSurfaceSystem::new(128.);
            surface.compile_dirty(&graph, &terrain);
            let mut lanes = LaneSystem::new();
            lanes.rebuild(&mut graph);
            let plain =
                RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
            graph.edge_mut(id).bus_lanes = [2, 2];
            lanes.rebuild(&mut graph);
            let bus =
                RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
            let again =
                RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
            assert_eq!(plain.road_vertices, bus.road_vertices);
            assert_eq!(plain.curb_vertices, bus.curb_vertices);
            assert_eq!(bus.marking_vertices, again.marking_vertices);
            assert_eq!(bus.marking_colors, again.marking_colors);
            assert!(bus.marking_colors.iter().any(|c| c.r > c.g * 3.));
            assert!(bus.marking_colors.contains(&Color::WHITE));
            assert!(
                bus.marking_vertices
                    .iter()
                    .all(|p| p.is_finite() && p.z.abs() <= 7.)
            );
            assert!(bus.seal().render_payload_validated());
        }
    }
}
