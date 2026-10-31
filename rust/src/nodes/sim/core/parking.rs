// SPDX-License-Identifier: GPL-2.0-only

//! Local physical parking publication at existing road and site edit boundaries.

use super::SimCore;
use crate::simulation::parking::lots::lot_bays;
use rayon::prelude::*;
use std::collections::BTreeSet;

impl SimCore {
    /// O(affected edges' geometry + local candidates × lot validation), never a city-wide supply rebuild.
    pub(crate) fn reconcile_parking_frontages(&mut self, edges: &BTreeSet<usize>) {
        if edges.is_empty() {
            return;
        }
        if self.allocator.building_sites.len() != self.allocator.buildings.len() {
            self.allocator
                .rebuild_building_site_clients(self.zoning.config.zone_cell_m);
        }
        // The site-cell index is maintained by site lifecycle helpers. Parking never rebuilds
        // the economy's zone/vacancy index, whose order affects admission.
        let mut buildings = Vec::new();
        for &edge in edges {
            if let Some(record) = self.region_graph.get_edge(edge) {
                for pair in record.physical_geometry.windows(2) {
                    self.allocator.site_cells.visit(
                        [
                            pair[0].x.min(pair[1].x) - 8.0,
                            pair[0].z.min(pair[1].z) - 8.0,
                            pair[0].x.max(pair[1].x) + 8.0,
                            pair[0].z.max(pair[1].z) + 8.0,
                        ],
                        |i| buildings.push(i),
                    );
                }
            }
        }
        buildings.sort_unstable();
        buildings.dedup();
        let lots: Vec<_> = crate::thread_pools::install(|| {
            buildings
                .into_par_iter()
                .filter_map(|index| {
                    let handle = self.allocator.building_handles.handle(index);
                    if handle.is_none() {
                        return None;
                    }
                    let (bays, issues) = lot_bays(
                        &self.allocator,
                        index,
                        &self.region_graph,
                        &self.transit_network.roadbed,
                    );
                    Some((handle, bays, issues))
                })
                .collect()
        });
        // Geometry is independent; live occupancy reconciliation commits in source order.
        for (handle, bays, issues) in lots {
            self.parking.replace_building(handle.to_bits(), &bays);
            if issues.is_empty() {
                self.parking.diagnostics.remove(&handle.to_bits());
            } else {
                self.parking.diagnostics.insert(handle.to_bits(), issues);
            }
        }
        self.build_parked_chunks();
    }

    /// O(changed buildings × lot validation), consuming the allocator's
    /// stable-handle site journal. Removed handles remove their old source lists even after swaps.
    pub(crate) fn publish_parking_sites(&mut self) {
        let mut changes = std::mem::take(&mut self.allocator.parking_site_changes);
        if changes.is_empty() {
            self.allocator.parking_site_changes = changes;
            return;
        }
        changes.sort_unstable();
        changes.dedup();
        let lots: Vec<_> = crate::thread_pools::install(|| {
            changes
                .par_iter()
                .map(|&handle| {
                    let (bays, issues) =
                        if let Some(i) = self.allocator.building_handles.resolve(handle) {
                            lot_bays(
                                &self.allocator,
                                i,
                                &self.region_graph,
                                &self.transit_network.roadbed,
                            )
                        } else {
                            (Vec::new(), Vec::new())
                        };
                    (handle, bays, issues)
                })
                .collect()
        });
        for (handle, bays, issues) in lots {
            self.parking.replace_building(handle.to_bits(), &bays);
            if issues.is_empty() {
                self.parking.diagnostics.remove(&handle.to_bits());
            } else {
                self.parking.diagnostics.insert(handle.to_bits(), issues);
            }
        }
        changes.clear();
        self.allocator.parking_site_changes = changes;
        self.build_parked_chunks();
    }
}

impl SimCore {
    /// Ground dirty parked instances once with the existing vehicle-contact solve, O(dirty cars × Q).
    pub(crate) fn build_parked_chunks(&mut self) {
        let mut parking = std::mem::take(&mut self.parking);
        parking.build_dirty_chunks_with(|model, pos, heading| {
            let (s, c) = heading.sin_cos();
            let (y, basis) = self.vehicle_ground_pose_exact(
                model,
                godot::prelude::Vector2::new(pos[0], pos[2]),
                godot::prelude::Vector3::new(s, 0.0, c),
            );
            let [x_axis, y_axis, z_axis] = basis;
            [
                x_axis.x, y_axis.x, z_axis.x, pos[0], x_axis.y, y_axis.y, z_axis.y, y, x_axis.z,
                y_axis.z, z_axis.z, pos[2],
            ]
        });
        self.parking = parking;
    }
}
