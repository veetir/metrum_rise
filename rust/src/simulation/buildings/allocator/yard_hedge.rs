// SPDX-License-Identifier: GPL-2.0-only

//! Yard hedge events: the rows a placed building's asset asks for, and the buildings removed.
//!
//! The allocator only plans and queues; SimCore lays and removes the plants, because they are
//! vegetation edits and the allocator does not own vegetation.

use super::BuildingAllocator;
use super::entrance::building_local_xz_pos;
use crate::assets::asset::{
    AnchorType, YardHedgeKind, YardLot, plan_yard_hedge, structure_footprint,
};
use godot::prelude::Vector2;

/// Stable identity of a zoned building's yard: its parcel and that parcel's build generation.
/// Building indices are swap-removed, so they cannot name what a yard hedge belongs to.
pub(crate) type YardKey = (u64, u32);

/// One planned hedge row in world metres, with whether each end may join a hedge it meets.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct YardHedgeRowWorld {
    pub(crate) from: Vector2,
    pub(crate) to: Vector2,
    pub(crate) join_from: bool,
    pub(crate) join_to: bool,
}

/// A yard change SimCore applies after the allocator step that made it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum YardHedgeEvent {
    /// A building was placed whose asset lines its yard with `hedge` along `rows`.
    Placed {
        key: YardKey,
        hedge: YardHedgeKind,
        rows: Vec<YardHedgeRowWorld>,
    },
    /// A zoned building was removed.
    Removed(YardKey),
}

impl BuildingAllocator {
    /// Queues the yard hedge of the building at `building_idx`, if its asset authors one and it
    /// stands on a parcel. O(rows * samples * surface and wall vertices) from the asset's own geometry.
    pub(crate) fn queue_yard_hedge(&mut self, building_idx: usize, zone_cell_m: f32) {
        let Some(building) = self.buildings.get(building_idx) else {
            return;
        };
        let Some(manifest) = self.registry.get(&building.asset_id).map(|entry| &entry.manifest)
        else {
            return;
        };
        let Some(yard) = manifest.building.as_ref().and_then(|data| data.yard_hedge.as_ref())
        else {
            return;
        };
        if building.parcel_id == 0 {
            return;
        }
        let frontage = manifest.building_frontage_forward();
        let surfaces: Vec<Vec<[f32; 2]>> = manifest
            .site_surfaces
            .iter()
            .map(|surface| surface.vertices.clone())
            .collect();
        let structures: Vec<_> = manifest
            .mesh_parts
            .iter()
            .filter_map(structure_footprint)
            .collect();
        let entrance = manifest
            .anchors
            .iter()
            .find(|anchor| anchor.anchor_type == AnchorType::Entrance && anchor.name == "main")
            .map(|anchor| [anchor.position[0], anchor.position[2]]);
        let lot = YardLot {
            half_width_m: f32::from(building.width_cells) * zone_cell_m * 0.5,
            half_depth_m: f32::from(building.depth_cells) * zone_cell_m * 0.5,
            frontage: [frontage[0], frontage[2]],
            surfaces: &surfaces,
            entrance,
            structures: &structures,
        };
        let world = |p: [f32; 2]| building_local_xz_pos(building, [p[0], 0.0, p[1]], frontage);
        let rows = plan_yard_hedge(&lot, &yard.edges)
            .into_iter()
            .map(|row| YardHedgeRowWorld {
                from: world(row.from),
                to: world(row.to),
                join_from: row.join_from,
                join_to: row.join_to,
            })
            .collect();
        let event = YardHedgeEvent::Placed {
            key: (building.parcel_id, building.build_generation),
            hedge: yard.hedge,
            rows,
        };
        self.pending_yard_hedges.push(event);
    }

    // Queues the removal of a zoned building's yard. SimCore ignores a yard it laid no hedge for.
    pub(super) fn queue_yard_removal(&mut self, building_idx: usize) {
        if let Some(building) = self.buildings.get(building_idx)
            && building.parcel_id != 0
        {
            let key = (building.parcel_id, building.build_generation);
            self.pending_yard_hedges.push(YardHedgeEvent::Removed(key));
        }
    }
}
