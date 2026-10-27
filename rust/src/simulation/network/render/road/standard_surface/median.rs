// SPDX-License-Identifier: GPL-2.0-only

//! Median material routing and paint on the authoritative band polygons.

use super::super::{
    MARKING_RENDER_Z_BIAS_M, MeshLayer, NetworkMeshData, concrete_color, marking_dash_color,
    push_triangle, road_color, sidewalk_color,
};
use super::geometry::{emit_road_space_quad, emit_surface_triangles};
use crate::simulation::network::graph::{MedianStyle, RoadMedian};
use crate::simulation::network::surface::{
    RoadSurfaceBandKind, RoadSurfaceSpanRegionRole, RoadSurfaceVisualNodePiece,
    RoadSurfaceVisualSpanPiece,
};
use godot::prelude::{Color, Vector2, Vector3};

fn material(median: RoadMedian) -> (MeshLayer, Color) {
    match median.style {
        MedianStyle::Painted => (MeshLayer::Road, road_color()),
        MedianStyle::KerbedGrass => (MeshLayer::MedianGrass, Color::WHITE),
        MedianStyle::KerbedPaving => (MeshLayer::Concrete, concrete_color()),
    }
}

pub(super) fn emit_span_nonroad(
    mesh: &mut NetworkMeshData,
    piece: &RoadSurfaceVisualSpanPiece,
    median: RoadMedian,
) {
    for region in piece
        .span_owned_regions
        .iter()
        .filter(|region| region.role() == RoadSurfaceSpanRegionRole::NonRoad)
    {
        let quad = piece.region_quad(region);
        if region.owner().kind != RoadSurfaceBandKind::Median {
            emit_surface_triangles(
                mesh,
                MeshLayer::Sidewalk,
                quad.triangles().iter().copied(),
                sidewalk_color(),
            );
            continue;
        }
        let (layer, color) = material(median);
        let corners = region.corners(&piece.sections);
        let uv = region.road_space_corners(&piece.sections);
        emit_road_space_quad(mesh, layer, &quad, &corners, &uv, color);
        if median.style == MedianStyle::Painted {
            for (a, b) in [(0, 1), (3, 2)] {
                let vector = |i: usize| {
                    Vector3::new(
                        corners[i].x as f32,
                        corners[i].y as f32,
                        corners[i].z as f32,
                    )
                };
                super::markings::emit_marking_segment(
                    mesh,
                    vector(a),
                    vector(b),
                    uv[a][0],
                    uv[b][0],
                    0.05,
                    2.0,
                    marking_dash_color(),
                );
            }
            for triangle in quad.triangles() {
                let hatch = triangle.map(|point| {
                    let i = corners
                        .iter()
                        .position(|&corner| corner == point)
                        .unwrap_or(0);
                    Vector2::new(uv[i][0] + uv[i][1], 5.0)
                });
                emit_hatch_triangle(mesh, *triangle, hatch);
            }
        }
    }
}

pub(super) fn emit_node_nonroad(
    mesh: &mut NetworkMeshData,
    piece: &RoadSurfaceVisualNodePiece,
    median: RoadMedian,
) {
    for region in &piece.owned_regions {
        if matches!(
            region.kind,
            RoadSurfaceBandKind::Carriageway | RoadSurfaceBandKind::CurbOrShoulder
        ) {
            continue;
        }
        let (layer, color) = if region.kind == RoadSurfaceBandKind::Median {
            material(median)
        } else {
            (MeshLayer::Sidewalk, sidewalk_color())
        };
        emit_surface_triangles(mesh, layer, region.polygon.triangles(), color);
        if region.kind == RoadSurfaceBandKind::Median && median.style == MedianStyle::Painted {
            for triangle in region.polygon.triangles() {
                emit_hatch_triangle(
                    mesh,
                    triangle,
                    triangle.map(|p| Vector2::new((p.x + p.z) as f32, 5.0)),
                );
            }
        }
    }
}

fn emit_hatch_triangle(
    mesh: &mut NetworkMeshData,
    points: [crate::simulation::network::surface::RoadVec3; 3],
    uv: [Vector2; 3],
) {
    let points =
        points.map(|p| Vector3::new(p.x as f32, p.y as f32 + MARKING_RENDER_Z_BIAS_M, p.z as f32));
    push_triangle(mesh, MeshLayer::Marking, points, uv, marking_dash_color());
}
