// SPDX-License-Identifier: GPL-3.0-or-later

//! Road-corridor conflict checks for parcel placement.

use super::overlap::rectangles_overlap_on_axes;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::zoning::parcels::{
    OVERLAP_EPSILON_M, ParcelGeometry, ROAD_OVERLAP_QUERY_PAD_M,
};
use godot::prelude::{Vector2, Vector3};

pub(crate) fn geometry_overlaps_road(graph: &RegionGraph, geometry: &ParcelGeometry) -> bool {
    let min = Vector3::new(
        geometry.aabb_min.x - ROAD_OVERLAP_QUERY_PAD_M,
        0.0,
        geometry.aabb_min.y - ROAD_OVERLAP_QUERY_PAD_M,
    );
    let max = Vector3::new(
        geometry.aabb_max.x + ROAD_OVERLAP_QUERY_PAD_M,
        0.0,
        geometry.aabb_max.y + ROAD_OVERLAP_QUERY_PAD_M,
    );
    graph
        .get_edges_near_aabb(min, max)
        .into_iter()
        .any(|edge_idx| {
            edge_idx != geometry.edge_idx && geometry_overlaps_road_edge(graph, geometry, edge_idx)
        })
}

fn geometry_overlaps_road_edge(
    graph: &RegionGraph,
    geometry: &ParcelGeometry,
    edge_idx: usize,
) -> bool {
    let edge = graph.edge(edge_idx);
    if edge.deleted || edge.physical_geometry.len() < 2 {
        return false;
    }
    let half_widths =
        [false, true].map(|positive| edge.width * 0.5 + edge.sidewalk_widths.on_lateral(positive));
    edge.physical_geometry.windows(2).any(|window| {
        // A pocket edge's corridor lies about its carriageway's axis (`ROAD-95`, D17).
        let (start, end) = edge.carriageway_segment(
            Vector2::new(window[0].x, window[0].z),
            Vector2::new(window[1].x, window[1].z),
        );
        geometry_overlaps_road_band_segment(geometry, start, end, half_widths)
    })
}

pub(crate) fn geometry_overlaps_road_corridor_segment(
    geometry: &ParcelGeometry,
    start: Vector2,
    end: Vector2,
    half_width: f32,
) -> bool {
    geometry_overlaps_road_band_segment(geometry, start, end, [half_width; 2])
}

fn geometry_overlaps_road_band_segment(
    geometry: &ParcelGeometry,
    start: Vector2,
    end: Vector2,
    [left_m, right_m]: [f32; 2],
) -> bool {
    let segment = end - start;
    if segment.length_squared() <= OVERLAP_EPSILON_M * OVERLAP_EPSILON_M {
        return false;
    }
    let tangent = segment.normalized();
    let normal = Vector2::new(tangent.y, -tangent.x);
    let road_corners = [
        start - normal * right_m,
        end - normal * right_m,
        end + normal * left_m,
        start + normal * left_m,
    ];
    rectangles_overlap_on_axes(
        &geometry.corners,
        &road_corners,
        [geometry.tangent, geometry.normal, tangent, normal],
    )
}
