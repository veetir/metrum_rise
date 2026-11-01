// SPDX-License-Identifier: GPL-2.0-only

//! Public anchor bays with fixed driveway–aisle–bay swept-footprint proofs.

use super::{
    Bay,
    geometry::{precise, rectangle, structure_collision},
    validation::{AnchorBay, validate_bay},
};
use crate::assets::AnchorType;
use crate::simulation::buildings::allocator::{BuildingAllocator, building_local_xz_basis};
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::roadbed::RoadbedSystem;
use godot::prelude::Vector2;

/// One place per valid parking anchor, at asset/edit boundaries only. For A bays, D driveways,
/// S support vertices and L local structure vertices: O(A² D + A D (S + L + K overlay cost)),
/// K only AABB-overlapping support/structure contours per tested footprint. Bay, sweep and
/// turn checks share this filter; disjoint support never runs an overlay. Unsupported sweeps
/// return diagnostics and publish no bay. No work depends on the city's total building count.
pub(crate) fn lot_bays(
    allocator: &BuildingAllocator,
    index: usize,
    graph: &RegionGraph,
    roads: &RoadbedSystem,
) -> (Vec<(u32, Bay)>, Vec<String>) {
    let building = &allocator.buildings[index];
    let Some(asset) = allocator.registry.get(&building.asset_id) else {
        return (Vec::new(), Vec::new());
    };
    let Some(site) = allocator.building_sites.get(index) else {
        return (Vec::new(), Vec::new());
    };
    let anchors = &asset.manifest.anchors;
    if !anchors.iter().any(|a| a.anchor_type == AnchorType::Parking) {
        return (Vec::new(), Vec::new());
    }
    let (bx, bz) = building_local_xz_basis(
        building.facing_dir,
        asset.manifest.building_frontage_forward(),
    );
    let center = Vector2::new(building.center_x, building.center_y);
    let position = |p: [f32; 3]| center + bx * p[0] + bz * p[2];
    let direction = |p: [f32; 3]| (bx * p[0] + bz * p[2]).normalized();
    let bays: Vec<_> = anchors
        .iter()
        .enumerate()
        .filter(|(_, a)| a.anchor_type == AnchorType::Parking)
        .map(|(i, a)| {
            (
                i as u32,
                position(a.position),
                direction(a.forward),
                a.width_m.unwrap_or(2.5),
                a.length_m.unwrap_or(5.0),
            )
        })
        .collect();
    let footprints: Vec<_> = bays
        .iter()
        .map(|(_, p, f, w, l)| rectangle(*p, *f, *w, *l))
        .collect();
    let mut support: Vec<_> = site
        .surfaces
        .iter()
        .map(|s| precise(&s.vertices_world))
        .collect();
    // Driveway access may cross its frontage's sidewalk/carriageway, never another building.
    if let Some(span) = roads.span_piece(building.edge_idx) {
        let mesh = span.mesh();
        support.extend(mesh.triangles().iter().map(|tri| {
            tri.iter()
                .map(|&i| {
                    let p = mesh.positions()[i as usize];
                    [p.x, p.z]
                })
                .collect()
        }));
    }
    let geometry: Vec<_> = bays
        .iter()
        .map(|&(_, pos, forward, width, length)| AnchorBay {
            pos,
            forward,
            width,
            length,
        })
        .collect();
    let mut result = Vec::new();
    let mut issues = Vec::new();
    for (slot, &(anchor, pos, forward, width, length)) in bays.iter().enumerate() {
        let fail = |reason: &str| {
            format!(
                "{} parking anchor {}: {}",
                building.asset_id, anchor, reason
            )
        };
        let paths: Vec<_> = anchors
            .iter()
            .filter(|a| a.anchor_type == AnchorType::Driveway)
            .filter_map(|driveway| {
                if driveway.width_m.unwrap_or(3.0) < 1.8 {
                    return None;
                }
                let start = position(driveway.position);
                let inward = direction(driveway.forward);
                if inward.dot(-building.facing_dir) < 0.9 {
                    return None;
                }
                let edge = graph.get_edge(building.edge_idx).filter(|e| !e.deleted)?;
                let station =
                    BuildingAllocator::project_point_to_polyline_s(&edge.physical_geometry, start);
                let connection = BuildingAllocator::claimed_road_side_connection_pos(
                    graph,
                    building.edge_idx,
                    building.side,
                    station / edge.physical_length,
                )?;
                // Placement already validates frontage support; repeat its exact connection proof.
                roads.height_at(connection.x, connection.y)?;
                let approach = start + inward * 2.5;
                Some([connection, approach])
            })
            .collect();
        let checked = validate_bay(
            slot,
            &geometry,
            &footprints,
            &support,
            anchors
                .iter()
                .any(|a| a.anchor_type == AnchorType::Driveway),
            &paths,
            |foot| structure_collision(allocator, foot),
        );
        if let Ok(path) = checked {
            let swept_path = path.map(|p| [p.x, p.y]);
            result.push((
                anchor,
                Bay {
                    identity: [0, u64::from(anchor)],
                    position: [pos.x, site.support_height_m + 0.02, pos.y],
                    path: swept_path,
                    heading: (-forward.x).atan2(-forward.y),
                    width,
                    length,
                },
            ));
        } else {
            issues.push(fail(checked.unwrap_err()));
        }
    }
    (result, issues)
}
