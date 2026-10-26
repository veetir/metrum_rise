// SPDX-License-Identifier: GPL-2.0-only

//! Emission of compiled road, curb, sidewalk, and raised-step top surfaces.

use super::super::{
    MeshLayer, NetworkMeshData, NetworkMeshOwner, curb_color, footpath_color, footpath_node_color,
    road_color, road_space_color, sidewalk_color,
};
use super::bridge::emit_compiled_bridge_concrete;
use super::coverage::CompiledSurfaceCoverage;
use super::earthwork::emit_compiled_earthwork_mesh;
use super::geometry::{
    emit_node_top_surface_polygons, emit_road_space_quad, emit_surface_triangles,
    emit_vertical_surface_polygon, emit_vertical_surface_triangles,
};
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::surface::RoadSurfaceSystem;
use crate::simulation::network::types::{EdgeClass, TransitType};
use crate::simulation::terrain::TerrainSystem;

pub(in crate::simulation::network::render::road) fn emit_compiled_surface_mesh(
    mesh: &mut NetworkMeshData,
    graph: &RegionGraph,
    road_surface: &RoadSurfaceSystem,
    terrain: &TerrainSystem,
    coverage: &CompiledSurfaceCoverage,
) {
    super::super::material_partition::prepare(mesh, graph, road_surface, coverage);
    emit_compiled_earthwork_mesh(mesh, graph, road_surface, terrain, coverage);

    for &edge_idx in &coverage.edge_indices {
        mesh.set_owner(NetworkMeshOwner::Edge(edge_idx));
        let edge = graph.edge(edge_idx);
        let Some(piece) = road_surface.compiled_visual_span_pieces().get(&edge_idx) else {
            continue;
        };
        for quad in piece.curb_surface_polygons() {
            emit_surface_triangles(
                mesh,
                MeshLayer::Curb,
                quad.triangles().iter().copied(),
                curb_color(),
            );
        }
        for face in piece.raised_step_face_polygons() {
            emit_vertical_surface_triangles(mesh, face.triangles().iter().copied(), curb_color());
        }
        if edge.primary_type == TransitType::Foot {
            // Footpaths draw their own path surface (road_path.gdshader) in road space.
            let color = footpath_color(edge);
            for (quad, corners, road_space) in piece.sidewalk_surface_polygons_in_road_space() {
                emit_road_space_quad(
                    mesh,
                    MeshLayer::Sidewalk,
                    &quad,
                    &corners,
                    &road_space,
                    color,
                );
            }
        } else if let Some(median) = edge.median {
            super::median::emit_span_nonroad(mesh, piece, median);
        } else {
            for quad in piece.sidewalk_surface_polygons() {
                emit_surface_triangles(
                    mesh,
                    MeshLayer::Sidewalk,
                    quad.triangles().iter().copied(),
                    sidewalk_color(),
                );
            }
        }
        let road_space_color = road_space_color(edge);
        let emit_asphalt = |(quad, corners, road_space)| {
            emit_road_space_quad(
                mesh,
                MeshLayer::Road,
                &quad,
                &corners,
                &road_space,
                road_space_color,
            );
        };
        let asphalt = piece.road_surface_polygons_in_road_space();
        if edge.pocket_taper().is_some() {
            asphalt
                .map(|(quad, corners, mut road_space)| {
                    for uv in &mut road_space {
                        let section = edge.lane_section_at(uv[0]);
                        let center = (section.left + section.right) * 0.5;
                        let median_center = (section.median_left + section.median_right) * 0.5;
                        let median_half = (section.median_right - section.median_left) * 0.5;
                        let side = if uv[1] < median_center { -1.0 } else { 1.0 };
                        // The shader's carriageway is centred. A right pocket shifts only one
                        // kerb; using the graph centreline makes its asphalt look like gravel.
                        uv[1] -= center + side * median_half;
                    }
                    (quad, corners, road_space)
                })
                .for_each(emit_asphalt);
        } else if edge.median.is_some() {
            let center = edge.median_center_offset();
            let half = edge.median_half_width();
            asphalt
                .map(|(quad, corners, mut road_space)| {
                    // Tyre wear is measured in lane space, excluding the island.
                    let side = if road_space.iter().map(|uv| uv[1]).sum::<f32>() * 0.25 < center {
                        -1.0
                    } else {
                        1.0
                    };
                    for uv in &mut road_space {
                        uv[1] -= center + side * half;
                    }
                    (quad, corners, road_space)
                })
                .for_each(emit_asphalt);
        } else {
            asphalt.for_each(emit_asphalt);
        }
        if let Some(tunnel) = &piece.short_tunnel {
            super::bridge::emit_shell_faces(
                mesh,
                MeshLayer::Concrete,
                &tunnel.concrete,
                super::super::concrete_color(),
            );
        }
        if let Some(crossing) = &piece.filled_crossing {
            super::bridge::emit_filled_crossing(mesh, crossing);
        } else if edge.class == EdgeClass::Bridge {
            let Some(sections) = road_surface.compiled_sections().get(&edge_idx) else {
                continue;
            };
            for (start_index, end_index) in
                road_surface.visible_section_ranges_for_edge(graph, terrain, edge_idx, sections)
            {
                if end_index <= start_index {
                    continue;
                }
                emit_compiled_bridge_concrete(mesh, terrain, &sections[start_index..=end_index]);
            }
        }
    }

    for &node_id in &coverage.node_ids {
        mesh.set_owner(NetworkMeshOwner::Node(node_id));
        let Some(piece) = road_surface.compiled_visual_node_pieces().get(&node_id) else {
            continue;
        };
        emit_node_top_surface_polygons(
            mesh,
            MeshLayer::Curb,
            piece.curb_surface_polygons(),
            curb_color(),
        );
        for polygon in &piece.raised_step_face_polygons {
            emit_vertical_surface_polygon(mesh, polygon, curb_color());
        }
        // Where only footpaths meet, the node piece is path surface too, without road space.
        let footpath_node = graph
            .node_adjacency(node_id)
            .iter()
            .map(|&edge_idx| graph.edge(edge_idx))
            .filter(|edge| !edge.deleted)
            .all(|edge| edge.primary_type == TransitType::Foot);
        let median = graph
            .node_adjacency(node_id)
            .iter()
            .copied()
            .filter_map(|id| graph.edge(id).median.map(|median| (id, median)))
            .min_by_key(|&(id, _)| id)
            .map(|(_, median)| median);
        if let Some(median) = median {
            super::median::emit_node_nonroad(mesh, piece, median);
        } else {
            emit_node_top_surface_polygons(
                mesh,
                MeshLayer::Sidewalk,
                piece.sidewalk_surface_polygons(),
                if footpath_node {
                    footpath_node_color()
                } else {
                    sidewalk_color()
                },
            );
        }
        emit_node_top_surface_polygons(
            mesh,
            MeshLayer::Road,
            piece.road_surface_polygons(),
            road_color(),
        );
    }
}
