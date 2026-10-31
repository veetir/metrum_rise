// SPDX-License-Identifier: GPL-2.0-only

//! Resident parking itinerary regressions on a minimal straight-road fixture.

use super::support::{create_test_building, register_test_asset, setup_straight_road};
use super::*;
use crate::simulation::parking::{Bay, ParkingSystem, ParkingTripPhase, PlaceSource};

fn fixture() -> (
    AgentSystem,
    ParkingSystem,
    BuildingAllocator,
    TransitNetwork,
    RegionGraph,
) {
    let (network, graph, _, _) = setup_straight_road();
    let mut allocator = BuildingAllocator::new();
    let home_asset = register_test_asset(&mut allocator, "parking", "home", ZoneClass::Residential);
    let work_asset = register_test_asset(&mut allocator, "parking", "work", ZoneClass::Commercial);
    for (x, asset, zone) in [
        (45.0, home_asset, ZoneType::Residential),
        (455.0, work_asset, ZoneType::Commercial),
    ] {
        let mut b = create_test_building(0, -1);
        b.center_x = x;
        b.center_y = -12.0;
        b.facing_dir = Vector2::new(0.0, -1.0);
        b.asset_id = asset;
        b.zone_type = zone;
        allocator.buildings.push(b);
    }
    allocator
        .building_handles
        .register_appended(allocator.buildings.len());
    allocator.rebuild_entrance_cache(&graph, &network.lane_system);
    let mut parking = ParkingSystem::default();
    for b in 0..2 {
        let e = &allocator.entrances[b];
        let pos = e.door_pos + Vector2::new(5.0, 0.0);
        parking.add_place(
            PlaceSource::Building {
                building: allocator.building_handles.handle(b).to_bits(),
                anchor: 0,
            },
            Bay {
                identity: [0, b as u64],
                position: [pos.x, 0.0, pos.y],
                path: [
                    [e.curb_pos.x, e.curb_pos.y],
                    [pos.x, e.curb_pos.y],
                    [pos.x, pos.y + 3.0],
                    [pos.x, pos.y],
                ],
                heading: 0.0,
                width: 2.5,
                length: 5.0,
            },
        );
    }
    parking.rebuild_candidates(&[0, 1], &allocator, &network, &graph);
    let home = allocator.building_handles.handle(0).to_bits();
    let car = parking.assign_household(
        0,
        crate::simulation::zoning::profiles::ZoneDensity::Low,
        [45.0, -12.0],
        true,
    );
    parking.park_new_car(car, home);
    assert!(parking.vehicle_place(car).is_some());
    let mut agents = AgentSystem::new();
    for _ in 0..2 {
        let i = agents.spawn_housed_agent(0, 45.0, -12.0);
        agents.agents.parking_trip[i].vehicle = car;
        agents.agents.planned_target_building[i] = 1;
        agents.agents.planned_activity[i] = 1;
    }
    (agents, parking, allocator, network, graph)
}

#[test]
fn resident_itinerary_never_drives_to_a_door_and_one_borrower() {
    let (mut agents, mut parking, allocator, mut network, mut graph) = fixture();
    let mut phases = [false; 5];
    let clock = test_clock(0, 0);
    for _ in 0..8000 {
        agents.tick_with_parking(
            &allocator,
            &mut network,
            &mut graph,
            0.1,
            &clock,
            &mut parking,
        );
        let phase = agents.agents.parking_trip[0].phase;
        match phase {
            ParkingTripPhase::WalkToCar => phases[0] = true,
            ParkingTripPhase::PullOut => phases[1] = true,
            ParkingTripPhase::Drive => phases[2] = true,
            ParkingTripPhase::PullIn => phases[3] = true,
            ParkingTripPhase::WalkToDoor => phases[4] = true,
            ParkingTripPhase::None => {}
        }
        assert_ne!(
            agents.agents.transit_mode[1], MODE_CAR,
            "second borrower drove"
        );
        for i in 0..2 {
            if agents.agents.transit[i] == TRANSIT_IN_BUILDING
                && agents.agents.current_building[i] == 1
            {
                assert_eq!(
                    agents.agents.transit_mode[i], MODE_WALK,
                    "car leg ended at a door"
                );
            }
        }
        if agents.agents.current_building[0] == 1 && agents.agents.transit[0] == TRANSIT_IN_BUILDING
        {
            break;
        }
    }
    assert_eq!(
        phases,
        [true; 5],
        "missing itinerary leg: {phases:?}, phase={:?} transit={} lane={} pos=({}, {})",
        agents.agents.parking_trip[0].phase,
        agents.agents.transit[0],
        agents.agents.current_lane_id[0],
        agents.agents.pos_x[0],
        agents.agents.pos_y[0]
    );
    assert_eq!(agents.agents.current_building[0], 1);
    let car = agents.agents.parking_trip[0].vehicle;
    assert!(parking.vehicle_place(car).is_some());
}

#[test]
fn full_destination_falls_back_to_walking_without_relocation() {
    let (mut agents, mut parking, allocator, mut network, mut graph) = fixture();
    parking.fill_test_fleet();
    let before = (agents.agents.pos_x[0], agents.agents.pos_y[0]);
    agents.tick_with_parking(
        &allocator,
        &mut network,
        &mut graph,
        0.1,
        &test_clock(0, 0),
        &mut parking,
    );
    assert_eq!(agents.agents.transit_mode[0], MODE_WALK);
    assert_eq!(agents.agents.parking_trip[0].phase, ParkingTripPhase::None);
    let e = &allocator.entrances[0];
    assert!(
        (agents.agents.pos_x[0] - e.door_pos.x).hypot(agents.agents.pos_y[0] - e.door_pos.y) < 0.01
    );
    assert!((agents.agents.pos_x[0] - before.0).abs() < 2.0);
}
