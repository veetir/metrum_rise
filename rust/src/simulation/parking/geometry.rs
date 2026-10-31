// SPDX-License-Identifier: GPL-2.0-only

//! Edit-boundary footprint checks using the existing polygon overlay and site spatial index.

use crate::simulation::agriculture::PolygonFootprint;
use crate::simulation::buildings::allocator::BuildingAllocator;
use godot::prelude::Vector2;
use i_overlay::core::{fill_rule::FillRule, overlay_rule::OverlayRule};
use i_overlay::float::single::SingleFloatOverlay;

pub(super) fn rectangle(
    center: Vector2,
    forward: Vector2,
    width: f32,
    length: f32,
) -> [Vector2; 4] {
    let along = forward.normalized() * (length * 0.5);
    let across = Vector2::new(-forward.y, forward.x) * (width * 0.5);
    [
        center - along - across,
        center + along - across,
        center + along + across,
        center - along + across,
    ]
}

pub(super) fn precise(points: &[Vector2]) -> Vec<[f64; 2]> {
    points
        .iter()
        .map(|p| [f64::from(p.x), f64::from(p.y)])
        .collect()
}

// Union coverage through ordered differences handles disconnected paving and holes exactly.
// This runs at edit/load boundaries, not in departure queries or movement.
pub(super) fn covered(points: &[Vector2], surfaces: &[Vec<[f64; 2]>]) -> bool {
    let mut remaining = vec![vec![precise(points)]];
    for surface in surfaces {
        remaining = remaining.overlay(surface, OverlayRule::Difference, FillRule::EvenOdd);
        if remaining.is_empty() {
            return true;
        }
    }
    false
}

pub(super) fn overlaps(a: &[Vector2], b: &[Vector2]) -> bool {
    PolygonFootprint::new(a).overlaps(&PolygonFootprint::new(b))
}

/// Only existing site-cell candidates are tested: O(local structures × footprint overlay).
pub(super) fn structure_collision(allocator: &BuildingAllocator, points: &[Vector2]) -> bool {
    let footprint = PolygonFootprint::new(points);
    let mut collision = false;
    allocator.site_cells.visit(
        [
            footprint.min.x,
            footprint.min.y,
            footprint.max.x,
            footprint.max.y,
        ],
        |idx| {
            if let Some(site) = allocator.building_sites.get(idx) {
                collision |= site
                    .structure_world
                    .iter()
                    .any(|structure| footprint.overlaps(&PolygonFootprint::new(structure)));
            }
        },
    );
    collision
}
