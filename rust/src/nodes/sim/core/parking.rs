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
        let mut points = Vec::new();
        for (handle, bays, issues) in lots {
            points.extend(self.parking.source_points(handle.to_bits()));
            points.extend(bays.iter().map(|(_, b)| [b.position[0], b.position[2]]));
            if let Some(b) = self.allocator.building_handles.resolve(handle) {
                let e = &self.allocator.entrances[b];
                points.push([e.door_pos.x, e.door_pos.y]);
            } else {
                self.parking.remove_destination_candidates(handle.to_bits());
            }
            self.parking.replace_building(handle.to_bits(), &bays);
            if issues.is_empty() {
                self.parking.diagnostics.remove(&handle.to_bits());
            } else {
                self.parking.diagnostics.insert(handle.to_bits(), issues);
            }
        }
        points.extend(
            edges
                .iter()
                .filter_map(|&edge| self.region_graph.get_edge(edge))
                .flat_map(|edge| {
                    edge.physical_geometry.windows(2).flat_map(|w| {
                        let steps = (w[0].distance_to(w[1]) / 200.0).ceil().max(1.0) as usize;
                        (0..=steps).map(move |i| {
                            let p = w[0].lerp(w[1], i as f32 / steps as f32);
                            [p.x, p.z]
                        })
                    })
                }),
        );
        self.rebuild_parking_candidates_near(&points);
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
        let mut points = Vec::new();
        for (handle, bays, issues) in lots {
            points.extend(self.parking.source_points(handle.to_bits()));
            points.extend(bays.iter().map(|(_, b)| [b.position[0], b.position[2]]));
            if let Some(b) = self.allocator.building_handles.resolve(handle) {
                let e = &self.allocator.entrances[b];
                points.push([e.door_pos.x, e.door_pos.y]);
            } else {
                self.parking.remove_destination_candidates(handle.to_bits());
            }
            self.parking.replace_building(handle.to_bits(), &bays);
            if issues.is_empty() {
                self.parking.diagnostics.remove(&handle.to_bits());
            } else {
                self.parking.diagnostics.insert(handle.to_bits(), issues);
            }
        }
        self.rebuild_parking_candidates_near(&points);
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

impl SimCore {
    /// Consumes household lifecycle events, never scans sleeping households or cars. O(events
    /// + affected members + local catchment entries); allocations are lifecycle preparation.
    pub(crate) fn publish_parking_households(&mut self) {
        use crate::simulation::zoning::profiles::{ZoneDensity, load_builtin_profile_registry};
        let mut events = std::mem::take(&mut self.households.parking_changes);
        if events.is_empty() {
            self.households.parking_changes = events;
            return;
        }
        let profiles = load_builtin_profile_registry().expect("built-in zoning profiles");
        for &handle in &events {
            let owner = handle.to_bits();
            let Some(h) = self.households.handles.resolve(handle) else {
                self.parking.remove_household_car(owner);
                continue;
            };
            let household = &self.households.households[h];
            let Some(home) = self.allocator.buildings.get(household.home_building_id) else {
                continue;
            };
            let home_handle = self
                .allocator
                .building_handles
                .handle(household.home_building_id)
                .to_bits();
            let density = profiles
                .density_for_runtime_id(home.zone_profile_runtime_id)
                .unwrap_or(ZoneDensity::Low);
            self.parking.assign_household(
                owner,
                density,
                [home.center_x, home.center_y],
                household.adult_count + household.elder_count > 0,
            );
            let car = self.parking.household_car(owner);
            self.parking.park_new_car(car, home_handle);
            let members: Vec<_> = self.agents.household_members(h).collect();
            for member in members {
                self.agents.agents.parking_trip[member].vehicle = car;
            }
        }
        events.clear();
        self.households.parking_changes = events;
    }

    /// Rebuilds only door catchments in the existing 512 m building chunks around changed
    /// supply/doors. O(local chunk buildings + local bounded searches), independent of background.
    pub(crate) fn rebuild_parking_candidates_near(&mut self, points: &[[f32; 2]]) {
        let mut buildings = Vec::new();
        for &point in points {
            let (cx, cz) = crate::simulation::network::graph::RegionGraph::get_chunk_coords(
                godot::prelude::Vector3::new(point[0], 0.0, point[1]),
            );
            for z in cz - 2..=cz + 2 {
                for x in cx - 2..=cx + 2 {
                    if let Some(rows) = self.allocator.building_chunks.get(&(x, z)) {
                        buildings.extend(rows.iter().copied().filter(|&b| {
                            self.allocator.entrances.get(b).is_some_and(|e| {
                                (e.door_pos.x - point[0]).hypot(e.door_pos.y - point[1]) <= 512.0
                            })
                        }));
                    }
                }
            }
        }
        buildings.sort_unstable();
        buildings.dedup();
        self.parking.rebuild_candidates(
            &buildings,
            &self.allocator,
            &self.transit_network,
            &self.region_graph,
        );
        for &point in points {
            self.parking.rehouse_near(point);
        }
    }
}

impl SimCore {
    /// Prepares only requested route capacity at the frame boundary, before agent movement.
    /// O(requested route storage); this is not part of a routing or movement loop.
    pub(crate) fn prepare_parking_route_capacity(&mut self) {
        use rayon::prelude::*;
        self.agents
            .agents
            .parking_trip
            .par_iter_mut()
            .zip(self.agents.agents.current_path.par_iter_mut())
            .for_each(|(trip, route)| {
                if trip.route_capacity > route.capacity() {
                    route.reserve(trip.route_capacity - route.len());
                }
                trip.route_capacity = 0;
            });
    }
}

impl SimCore {
    /// Drains explicit retry requests into the saved timing wheel; no polling timer.
    /// 30 s for unavailable routes, next eligible slot for requested route-buffer preparation.
    /// O(retries × log scheduled events), outside agent movement.
    pub(crate) fn schedule_parking_retries(&mut self) {
        let mut requests = std::mem::take(&mut self.agents.parking_retries);
        for (agent, seconds) in requests.drain(..) {
            let slots = (f64::from(seconds)
                * crate::simulation::core::scheduler::SLOTS_PER_MINUTE as f64
                / self.time.seconds_per_minute())
            .ceil()
            .max(1.0) as u64;
            self.schedule_trip(
                self.time.absolute_slot() + slots,
                super::trips::TripEvent::ParkingRetry { agent },
            );
        }
        self.agents.parking_retries = requests;
    }
}
