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

#[test]
fn removing_reserved_supply_replans_at_the_actual_position() {
    for phase in [
        ParkingTripPhase::WalkToCar,
        ParkingTripPhase::PullOut,
        ParkingTripPhase::Drive,
        ParkingTripPhase::PullIn,
    ] {
        let (mut agents, mut parking, allocator, mut network, mut graph) = fixture();
        let e = &allocator.entrances[1];
        let pos = e.door_pos + Vector2::new(13.0, 0.0);
        let replacement = parking.add_place(
            PlaceSource::Building {
                building: allocator.building_handles.handle(1).to_bits(),
                anchor: 1,
            },
            Bay {
                identity: [0, 9],
                position: [pos.x, 0.0, pos.y],
                path: [
                    [e.curb_pos.x + 8.0, e.curb_pos.y],
                    [pos.x, e.curb_pos.y],
                    [pos.x, pos.y + 3.0],
                    [pos.x, pos.y],
                ],
                heading: 0.0,
                width: 2.5,
                length: 5.0,
            },
        );
        parking.rebuild_candidates(&[0, 1], &allocator, &network, &graph);
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
            if agents.parking_trip[0].phase == phase {
                break;
            }
        }
        assert_eq!(agents.parking_trip[0].phase, phase);
        let removed = agents.parking_trip[0].destination;
        assert_ne!(removed, replacement);
        parking.remove_place(removed);
        parking.rebuild_candidates(&[0, 1], &allocator, &network, &graph);
        let car = agents.parking_trip[0].vehicle;
        let mut reached = false;
        for _ in 0..10000 {
            let before = Vector2::new(agents.pos_x[0], agents.pos_y[0]);
            agents.tick_with_parking(
                &allocator,
                &mut network,
                &mut graph,
                0.1,
                &clock,
                &mut parking,
            );
            let after = Vector2::new(agents.pos_x[0], agents.pos_y[0]);
            assert!(
                before.distance_to(after) < 8.0,
                "recovery relocated a driver from {before:?} to {after:?} in {phase:?}"
            );
            if agents.current_building[0] == 1 && agents.transit[0] == TRANSIT_IN_BUILDING {
                reached = true;
                break;
            }
        }
        assert!(
            reached,
            "recovery stalled from {phase:?}: {:?}",
            agents.parking_trip[0]
        );
        assert_eq!(parking.vehicle_place(car), Some(replacement));
        assert_eq!(agents.transit_mode[0], MODE_WALK);
    }
}

#[test]
fn towing_before_borrowing_walks_to_the_cars_actual_new_place() {
    let (mut agents, mut parking, allocator, mut network, mut graph) = fixture();
    let e = &allocator.entrances[0];
    let pos = e.door_pos + Vector2::new(13.0, 0.0);
    let replacement = parking.add_place(
        PlaceSource::Building {
            building: allocator.building_handles.handle(0).to_bits(),
            anchor: 1,
        },
        Bay {
            identity: [0, 9],
            position: [pos.x, 0.0, pos.y],
            path: [
                [e.curb_pos.x + 8.0, e.curb_pos.y],
                [pos.x, e.curb_pos.y],
                [pos.x, pos.y + 3.0],
                [pos.x, pos.y],
            ],
            heading: 0.0,
            width: 2.5,
            length: 5.0,
        },
    );
    parking.rebuild_candidates(&[0, 1], &allocator, &network, &graph);
    let clock = test_clock(0, 0);
    agents.tick_with_parking(
        &allocator,
        &mut network,
        &mut graph,
        0.1,
        &clock,
        &mut parking,
    );
    assert_eq!(agents.parking_trip[0].phase, ParkingTripPhase::WalkToCar);
    let car = agents.parking_trip[0].vehicle;
    let old = agents.parking_trip[0].source;
    parking.remove_place(old);
    assert_eq!(parking.vehicle_place(car), Some(replacement));
    parking.rebuild_candidates(&[0, 1], &allocator, &network, &graph);
    let mut retrieved = false;
    for _ in 0..10000 {
        let before = Vector2::new(agents.pos_x[0], agents.pos_y[0]);
        agents.tick_with_parking(
            &allocator,
            &mut network,
            &mut graph,
            0.1,
            &clock,
            &mut parking,
        );
        assert!(before.distance_to(Vector2::new(agents.pos_x[0], agents.pos_y[0])) < 8.0);
        if agents.parking_trip[0].source == replacement && agents.transit_mode[0] == MODE_CAR {
            retrieved = true;
        }
        if agents.current_building[0] == 1 && agents.transit[0] == TRANSIT_IN_BUILDING {
            break;
        }
    }
    assert!(
        retrieved,
        "borrower did not retrieve the towed car: {:?}",
        agents.parking_trip[0]
    );
    assert_eq!(agents.current_building[0], 1);
}

#[test]
fn candidate_walk_bound_uses_network_time_and_publication_rebuilds_it() {
    let (agents, mut parking, mut allocator, network, graph) = fixture();
    allocator.entrances[0].door_pos = Vector2::new(45.0, -100.0);
    let home = allocator.building_handles.handle(0).to_bits();
    let remote = parking.add_place(
        PlaceSource::Building {
            building: allocator.building_handles.handle(1).to_bits(),
            anchor: 8,
        },
        Bay {
            identity: [0, 8],
            position: [300.0, 0.0, -100.0],
            path: [
                [300.0, -4.0],
                [300.0, -20.0],
                [300.0, -95.0],
                [300.0, -100.0],
            ],
            heading: 0.0,
            width: 2.5,
            length: 5.0,
        },
    );
    assert!(
        allocator.entrances[0]
            .door_pos
            .distance_to(Vector2::new(300.0, -100.0))
            < 400.0
    );
    parking.rebuild_candidates(&[0], &allocator, &network, &graph);
    assert!(
        parking.walk_to_car_seconds(home, remote).is_none(),
        "straight-line bound admitted a walk over 400 m at walking pace"
    );
    let near = parking.add_place(
        PlaceSource::Building {
            building: home,
            anchor: 9,
        },
        Bay {
            identity: [0, 9],
            position: [75.0, 0.0, -100.0],
            path: [[75.0, -4.0], [75.0, -20.0], [75.0, -95.0], [75.0, -100.0]],
            heading: 0.0,
            width: 2.5,
            length: 5.0,
        },
    );
    assert!(parking.first_free_candidate(home).is_none());
    parking.rebuild_candidates(&[0], &allocator, &network, &graph);
    assert_eq!(parking.first_free_candidate(home).unwrap().place, near);
    parking.remove_place(near);
    parking.rebuild_candidates(&[0], &allocator, &network, &graph);
    assert!(parking.first_free_candidate(home).is_none());
    assert!(
        parking
            .vehicle_place(agents.parking_trip[0].vehicle)
            .is_some()
    );
}

#[test]
fn children_cannot_borrow_a_free_household_car() {
    let (mut agents, mut parking, allocator, mut network, mut graph) = fixture();
    agents.age_group.fill(AGE_CHILD);
    agents.tick_with_parking(
        &allocator,
        &mut network,
        &mut graph,
        0.1,
        &test_clock(0, 0),
        &mut parking,
    );
    for i in 0..agents.len() {
        assert!(!agents.has_car[i]);
        assert_eq!(agents.transit_mode[i], MODE_WALK);
        assert_eq!(agents.parking_trip[i].phase, ParkingTripPhase::None);
    }
    assert!(
        parking
            .borrowed_by(agents.parking_trip[0].vehicle)
            .is_none()
    );
}
