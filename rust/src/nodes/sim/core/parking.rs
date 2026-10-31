// SPDX-License-Identifier: GPL-2.0-only

//! Local physical parking publication at existing road and site edit boundaries.

use super::SimCore;
use crate::simulation::parking::{
    PlaceSource, SideRule,
    lots::lot_bays,
    places::{kerb_bays, kerb_context},
};
use rayon::prelude::*;
use std::collections::BTreeSet;

impl SimCore {
    /// O(affected edges' geometry + local site candidates), never a city-wide supply rebuild.
    pub(crate) fn refresh_parking_roads(&mut self, edges: &BTreeSet<usize>) {
        if edges.is_empty() {
            return;
        }
        self.allocator
            .prepare_building_site_query_index(self.zoning.config.zone_cell_m);
        let products: Vec<_> = crate::thread_pools::install(|| {
            edges
                .par_iter()
                .map(|&edge| {
                    if self.region_graph.get_edge(edge).is_none_or(|e| e.deleted) {
                        return (edge, None, [Vec::new(), Vec::new()]);
                    }
                    let context = kerb_context(&self.region_graph, &self.allocator, edge);
                    let mut rule = self.region_graph.parking_rule(edge);
                    rule.choose_side(context.mouths);
                    let bays = [0, 1].map(|side| {
                        kerb_bays(
                            &self.region_graph,
                            &self.transit_network.roadbed,
                            &self.allocator,
                            edge,
                            side,
                            rule,
                            &context,
                        )
                    });
                    (edge, Some(rule), bays)
                })
                .collect()
        });
        let mut supply = Vec::new();
        for (edge, rule, bays) in products {
            if let Some(rule) = rule {
                self.region_graph.set_parking_rule(edge, rule);
            }
            for (side, bays) in bays.into_iter().enumerate() {
                supply.push((
                    PlaceSource::Edge {
                        edge,
                        side: if side == 0 { -1 } else { 1 },
                    },
                    bays,
                ));
            }
        }
        self.parking.replace_kerbs(supply);
        self.build_parked_chunks();
    }

    /// O(changed buildings' anchors + local affected road geometry), consuming the allocator's
    /// stable-handle site journal. Removed handles remove their old source lists even after swaps.
    pub(crate) fn publish_parking_sites(&mut self) {
        let mut changes = std::mem::take(&mut self.allocator.parking_site_changes);
        if changes.is_empty() {
            self.allocator.parking_site_changes = changes;
            return;
        }
        changes.sort_unstable();
        changes.dedup();
        let mut edges = BTreeSet::new();
        for handle in changes.drain(..) {
            let (bays, issues) = if let Some(i) = self.allocator.building_handles.resolve(handle) {
                edges.insert(self.allocator.buildings[i].edge_idx);
                lot_bays(
                    &self.allocator,
                    i,
                    &self.region_graph,
                    &self.transit_network.roadbed,
                )
            } else {
                (Vec::new(), Vec::new())
            };
            self.parking.replace_building(handle.to_bits(), &bays);
            if issues.is_empty() {
                self.parking.diagnostics.remove(&handle.to_bits());
            } else {
                self.parking.diagnostics.insert(handle.to_bits(), issues);
            }
        }
        self.allocator.parking_site_changes = changes;
        // Old/new site bounds also catch demolished driveways and neighbouring kerb footprints.
        for change in &self.last_site_terrain_changes {
            let [a, b, c, d] = change.bounds;
            edges.extend(self.region_graph.get_edges_near_aabb(
                godot::prelude::Vector3::new(a - 8.0, 0.0, b - 8.0),
                godot::prelude::Vector3::new(c + 8.0, 0.0, d + 8.0),
            ));
        }
        self.refresh_parking_roads(&edges);
        self.build_parked_chunks();
    }

    /// Player property edit: O(local edge geometry), undo stores permission only, never cars.
    pub(crate) fn set_parking_side(&mut self, edge: usize, side: usize, value: SideRule) {
        self.adopt_pending_cell_lots();
        if side > 1 || self.region_graph.get_edge(edge).is_none_or(|e| e.deleted) {
            return;
        }
        self.push_network_undo_for_local_topology(
            std::collections::HashSet::from([edge]),
            std::collections::HashSet::new(),
        );
        let mut rule = self.region_graph.parking_rule(edge);
        rule.automatic = false;
        rule.sides[side] = value;
        self.region_graph.set_parking_rule(edge, rule);
        self.refresh_parking_roads(&BTreeSet::from([edge]));
        self.bump_road_tool_query_generation();
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
