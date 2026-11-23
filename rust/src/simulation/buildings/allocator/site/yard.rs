// SPDX-License-Identifier: GPL-3.0-or-later

//! Automatic road-following yard planes, with continuous cone banks to the stored floor.

use super::derive::level_support_footprint_local;
use crate::assets::AssetManifest;
use crate::simulation::buildings::allocator::BuildingAllocator;
use crate::simulation::buildings::allocator::entrance::building_local_xz_basis;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::roadbed::RoadbedSystem;
use crate::simulation::parking::ground::DRIVE_GRADE;
use crate::simulation::terrain::graded::SiteTerrace;
use godot::prelude::Vector2;

/// One bounded plane is sufficient for moderate grades. Required contacts and D still prove
/// its visible result; clamping its grade does not waive an infeasible street contact.
/// O(frontage polyline knots + asset support vertices), placement/edit/load only.
#[allow(clippy::too_many_arguments)]
pub(crate) fn derive_yard_terrace(
    manifest: &AssetManifest,
    lot: [Vector2; 4],
    center: Vector2,
    facing: Vector2,
    edge_idx: usize,
    side: i8,
    graph: &RegionGraph,
    roads: &RoadbedSystem,
) -> Option<SiteTerrace> {
    let plane = frontage_yard_plane(lot, center, facing, edge_idx, side, graph, roads)?;
    let (bx, bz) = building_local_xz_basis(facing, manifest.building_frontage_forward());
    let half_width = lot
        .iter()
        .map(|p| (*p - center).dot(bx).abs())
        .fold(0.0_f32, f32::max);
    let half_depth = lot
        .iter()
        .map(|p| (*p - center).dot(bz).abs())
        .fold(0.0_f32, f32::max);
    Some(SiteTerrace {
        footprint_world: lot.to_vec(),
        plane,
        level_footprint_world: level_support_footprint_local(manifest, half_width, half_depth)
            .into_iter()
            .map(|p| center + bx * p.x + bz * p.y)
            .collect(),
    })
}

/// Road-only plane inputs, without allocating asset polygons for an unchanged edited site.
#[allow(clippy::too_many_arguments)]
pub(super) fn frontage_yard_plane(
    lot: [Vector2; 4],
    center: Vector2,
    facing: Vector2,
    edge_idx: usize,
    side: i8,
    graph: &RegionGraph,
    roads: &RoadbedSystem,
) -> Option<[f64; 3]> {
    let edge = graph.get_edge(edge_idx)?;
    let along = facing.orthogonal();
    let half_width = lot
        .iter()
        .map(|p| (*p - center).dot(along).abs())
        .fold(0.0_f32, f32::max);
    let half_depth = lot
        .iter()
        .map(|p| (*p - center).dot(facing).abs())
        .fold(0.0_f32, f32::max);
    let front = center + facing * half_depth;
    let sample = |p: Vector2| {
        let t = BuildingAllocator::project_point_to_polyline_s(&edge.physical_geometry, p)
            / edge.physical_length;
        let foot = BuildingAllocator::claimed_road_side_connection_pos(graph, edge_idx, side, t)?;
        roads.exposed_height_at(foot.x, foot.y)
    };
    let (a, b) = (
        sample(front - along * half_width)?,
        sample(front + along * half_width)?,
    );
    if a.to_bits() == b.to_bits() || half_width <= 0.0 {
        // The existing primitive and all its float operations stay unchanged on flat ground.
        return None;
    }
    let grade = (f64::from(b - a) / f64::from(2.0 * half_width))
        .clamp(-f64::from(DRIVE_GRADE), f64::from(DRIVE_GRADE));
    let (gx, gz) = (f64::from(along.x) * grade, f64::from(along.y) * grade);
    // A curved vertical profile need not pass through the secant's midpoint. The local
    // contact is the actual midpoint road surface, not the average of distant endpoints.
    let middle = f64::from(sample(front)?);
    Some([
        gx,
        gz,
        middle - gx * f64::from(front.x) - gz * f64::from(front.y),
    ])
}
