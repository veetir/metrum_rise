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
    parking_candidate(
        mode,
        origin,
        destination,
        maneuvers,
        network,
        graph,
        count,
        None,
    )
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

    /// Cost with the existing car preference overhead; walking wins an exact tie.
    pub(crate) fn mode_choice_seconds(&self) -> f32 {
        self.0.mode_choice_cost_s
    }

    /// Writes the next leg into the agent's reusable route buffer; O(route length + Q).
    /// Capacity is prepared at a lifecycle boundary; overflow returns the needed size without
    /// allocating or truncating. The caller schedules preparation before retrying.
    pub(crate) fn materialize(
        &self,
        target: usize,
        activity: u8,
        route: &mut Vec<u32>,
        prepared_nodes: usize,
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
                            if prepared_nodes < path.len() {
                                return Err(path.len());
                            }
                            assert!(
                                route.capacity() >= path.len(),
                                "parking route preparation must precede movement"
                            );
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

/// Quotes recovery from the actual road/foot-lane position. O(Q), four endpoint pairs;
/// no relocation and no allocation. Connector recovery waits for the existing road-lane exit.
#[allow(clippy::too_many_arguments)]
pub(crate) fn quote_parking_leg_from_lane(
    mode: u8,
    lane_id: usize,
    distance: f32,
    destination: &BuildingEntrance,
    pull_in_s: f32,
    network: &TransitNetwork,
    graph: &RegionGraph,
    count: &AtomicU32,
) -> Option<ParkingLegQuote> {
    let lane = network.lane_system.lanes.get(lane_id)?;
    let edge = graph.get_edge(lane.edge_id)?;
    let point = crate::simulation::buildings::allocator::BuildingAllocator::sample_pos_on_lane(
        lane, distance,
    );
    let mut origin = BuildingEntrance {
        edge_idx: lane.edge_id,
        side: if lane.is_fwd { -1 } else { 1 },
        door_pos: point,
        curb_pos: point,
        entrance_s_m:
            crate::simulation::buildings::allocator::BuildingAllocator::project_point_to_polyline_s(
                &edge.physical_geometry,
                point,
            ),
        ..Default::default()
    };
    if mode == MODE_CAR {
        if lane.is_fwd {
            origin.car_lane_fwd = lane_id;
        } else {
            origin.car_lane_bkw = lane_id;
        }
    } else if lane.is_fwd {
        origin.foot_lane_fwd = lane_id;
    } else {
        origin.foot_lane_bkw = lane_id;
    }
    parking_candidate(
        mode,
        &origin,
        destination,
        [0.0, pull_in_s],
        network,
        graph,
        count,
        Some((lane_id, distance)),
    )
    .map(ParkingLegQuote)
}

/// Proven complete car itinerary without claims or route-vector allocation.
/// Destination selection is O(k); the three exact route quotes cost O(Q), independent of k.
pub(crate) struct ParkingQuote {
    /// The household car's actual occupied bay.
    pub(crate) source: crate::simulation::core::handles::Handle<crate::simulation::parking::Place>,
    /// First free fitting bay in the destination's published walking catchment.
    pub(crate) destination:
        crate::simulation::core::handles::Handle<crate::simulation::parking::Place>,
    /// First pedestrian leg, ready for the ordinary reusable route materializer.
    pub(crate) to_car: ParkingLegQuote,
    /// Physical travel seconds for all five legs, excluding the mode-choice preference.
    pub(crate) seconds: f32,
    /// Complete cost with the unchanged driving preference penalty.
    pub(crate) mode_choice_seconds: f32,
}

/// Quotes all five resident legs at current supply and custody, without reserving. O(k + Q)
/// with prepared routing scratch and no per-candidate route queries or allocations. Callers
/// pass adult eligibility separately; a child can own a car but cannot borrow it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn quote_parking_trip(
    car: crate::simulation::core::handles::Handle<crate::simulation::parking::Vehicle>,
    borrower: crate::simulation::economy::agents::AgentHandle,
    eligible: bool,
    origin: usize,
    destination: usize,
    parking: &crate::simulation::parking::ParkingSystem,
    allocator: &crate::simulation::buildings::allocator::BuildingAllocator,
    network: &TransitNetwork,
    graph: &RegionGraph,
    count: &AtomicU32,
) -> Option<ParkingQuote> {
    use super::super::super::MODE_WALK;
    if !eligible || !parking.available_to(car, borrower) {
        return None;
    }
    let source = parking.vehicle_place(car)?;
    parking.walk_to_car_seconds(allocator.building_handles.handle(origin).to_bits(), source)?;
    let candidate =
        parking.first_free_candidate(allocator.building_handles.handle(destination).to_bits())?;
    let from = parking.place_entrance(source, allocator, network, graph)?;
    let to = parking.place_entrance(candidate.place, allocator, network, graph)?;
    let drive = quote_parking_leg(
        MODE_CAR,
        &from,
        &to,
        [
            parking.maneuver_seconds(source),
            parking.maneuver_seconds(candidate.place),
        ],
        network,
        graph,
        count,
    )?;
    let to_car = quote_parking_leg(
        MODE_WALK,
        allocator.entrances.get(origin)?,
        &from,
        [0.0; 2],
        network,
        graph,
        count,
    )?;
    let to_door = quote_parking_leg(
        MODE_WALK,
        &to,
        allocator.entrances.get(destination)?,
        [0.0; 2],
        network,
        graph,
        count,
    )?;
    let walking = to_car.seconds() + to_door.seconds();
    Some(ParkingQuote {
        source,
        destination: candidate.place,
        seconds: walking + drive.seconds(),
        mode_choice_seconds: walking + drive.mode_choice_seconds(),
        to_car,
    })
}


/// Estimates the same complete itinerary offered at departure, without claiming supply.
/// O(k + Q), with walking winning a mode-cost tie; physical seconds exclude preference overhead.
#[allow(clippy::too_many_arguments)]
pub(crate) fn estimate_parking_trip_seconds(
    origin: usize, destination: usize,
    car: crate::simulation::core::handles::Handle<crate::simulation::parking::Vehicle>,
    eligible: bool, parking: &crate::simulation::parking::ParkingSystem,
    allocator: &crate::simulation::buildings::allocator::BuildingAllocator,
    network: &TransitNetwork, graph: &RegionGraph, count: &AtomicU32,
) -> Option<u16> {
    let walk = quote_parking_leg(super::super::super::MODE_WALK, allocator.entrances.get(origin)?, allocator.entrances.get(destination)?, [0.0;2], network, graph, count);
    let borrower = parking.borrowed_by(car).unwrap_or(crate::simulation::core::handles::Handle::NONE);
    let car = quote_parking_trip(car, borrower, eligible, origin, destination, parking, allocator, network, graph, count);
    let seconds = match (walk, car) {
        (Some(walk), Some(car)) if walk.seconds() > car.mode_choice_seconds => car.seconds,
        (Some(walk), _) => walk.seconds(),
        (None, Some(car)) => car.seconds,
        _ => return None,
    };
    Some(seconds.ceil().min(f32::from(u16::MAX)) as u16)
}
