// SPDX-License-Identifier: GPL-2.0-only

//! Parking endpoint quotes and reusable-route materialization using ordinary exact routing.

use super::super::super::{ACCESS_PLAN_VALID, ACCESS_ZERO_HOP_NODE_PATH, MODE_CAR};
use super::candidate::{PlannedTripCandidate, RouteStart, TripQueryScratch, parking_candidate};
use super::types::BuiltTripPlan;
use crate::simulation::buildings::allocator::BuildingEntrance;
use crate::simulation::network::{TransitNetwork, graph::RegionGraph};
use std::sync::atomic::AtomicU32;

/// Exact endpoint quote. O(Q) route work for four endpoint pairs, independent of parking
/// candidate count. Drive costs use free-flow weights and fixed swept paths, never live traffic.
#[allow(clippy::too_many_arguments)]
pub(crate) fn quote_parking_leg(
    mode: u8,
    origin: &BuildingEntrance,
    destination: &BuildingEntrance,
    maneuvers: [f32; 2],
    network: &TransitNetwork,
    graph: &RegionGraph,
    count: &AtomicU32,
) -> Option<ParkingLegQuote> {
    parking_candidate(mode, origin, destination, maneuvers, network, graph, count)
        .map(ParkingLegQuote)
}

/// Proven endpoint pair; no persistent route allocation until the phase starts.
#[derive(Clone)]
pub(crate) struct ParkingLegQuote(PlannedTripCandidate);

impl ParkingLegQuote {
    /// Physical travel seconds, excluding the unchanged mode-choice preference penalty.
    pub(crate) fn seconds(&self) -> f32 {
        self.0.total_cost_s
    }

    /// Writes the next leg into the agent's reusable route buffer; O(route length + Q).
    /// Capacity is prepared at a lifecycle boundary; overflow returns the needed size without
    /// allocating or truncating. The caller schedules preparation before retrying.
    pub(crate) fn materialize(
        &self,
        target: usize,
        activity: u8,
        route: &mut Vec<u32>,
        network: &TransitNetwork,
        graph: &RegionGraph,
    ) -> Result<BuiltTripPlan, usize> {
        let c = &self.0;
        let direct = c.planned_attach_lane_id == c.planned_detach_lane_id
            && c.planned_attach_lane_d <= c.planned_detach_lane_d + 1e-6;
        let zero = !direct
            && c.planned_attach_node == c.planned_detach_node
            && (c.mode != MODE_CAR
                || network.cch_graph.vehicle_turn_permitted(
                    c.planned_attach_node,
                    network.lane_system.lanes[c.planned_attach_lane_id].edge_id,
                    network.lane_system.lanes[c.planned_detach_lane_id].edge_id,
                ));
        route.clear();
        if !direct && !zero {
            TripQueryScratch::with_routes(|scratch| {
                scratch
                    .network_path(
                        c.mode,
                        c.planned_attach_node,
                        c.planned_detach_node,
                        RouteStart::Lane(c.planned_attach_lane_id),
                        c.planned_detach_lane_id,
                        network,
                        graph,
                        |path| {
                            if route.capacity() < path.len() {
                                return Err(path.len());
                            }
                            route.extend_from_slice(path);
                            Ok(())
                        },
                    )
                    .map_err(|_| 0_usize)?
                    .1
            })?;
        }
        Ok(BuiltTripPlan {
            mode: c.mode,
            target_building: target,
            activity,
            planned_attach_node: c.planned_attach_node,
            planned_detach_node: c.planned_detach_node,
            planned_attach_lane_id: c.planned_attach_lane_id,
            planned_detach_lane_id: c.planned_detach_lane_id,
            planned_attach_lane_d: c.planned_attach_lane_d,
            planned_detach_lane_d: c.planned_detach_lane_d,
            current_path: std::mem::take(route),
            access_flags: ACCESS_PLAN_VALID | if zero { ACCESS_ZERO_HOP_NODE_PATH } else { 0 },
        })
    }
}
