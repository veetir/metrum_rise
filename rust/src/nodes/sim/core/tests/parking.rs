// SPDX-License-Identifier: GPL-2.0-only

//! PARK-01 real edit, towing, lot and SQLite integration regressions.

use super::*;
use crate::nodes::sim::benchmark::road_edit::RoadEditMetrics;
use crate::nodes::sim::core::RoadCommitRequest;
use std::time::Instant;

fn core() -> SimCore {
    let mut core = SimCore::new(
        WorldConfig::new(512.0, 512.0, 16.0, 10.0)
            .with_chunking(128.0, 0.0)
            .with_terrain_resolution(2.0),
    );
    core.treasury.balance = 1_000_000.0;
    commit(&mut core, (-120.0, 0.0), (120.0, 0.0));
    core
}
fn commit(core: &mut SimCore, a: (f32, f32), b: (f32, f32)) {
    core.precompute_road_mesh_data();
    core.allocator
        .prepare_building_site_query_index(core.config.zone_cell_m);
    let outcome = core.commit_road(
        RoadCommitRequest {
            points: vec![Vector3::new(a.0, 0.0, a.1), Vector3::new(b.0, 0.0, b.1)],
            fwd_lanes: 1,
            bkw_lanes: 1,
            snap_to_existing_roads: true,
            preview: None,
        },
        Instant::now(),
        RoadEditMetrics::default(),
    );
    assert!(outcome.committed, "{}", outcome.rejection);
    core.adopt_pending_cell_lots();
    core.transit_network.settle_cch();
}
fn register_lot(core: &mut SimCore) -> String {
    let mut manifest = AssetManifest::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../godot/bootstrap/mods/parking/assets/building.parking.small_lot/asset.toml"
    )))
    .unwrap();
    manifest.mesh_parts[0].imported_bounds = Some([[-0.8, 0.0, -0.12], [0.8, 0.16, 0.12]]);
    let id = manifest.qualified_id("parking");
    core.allocator
        .registry
        .register("parking", manifest, String::new());
    id
}

#[test]
fn parking_lot_frontage_split_and_undo_reconcile_live_cars() {
    use crate::simulation::parking::PlaceSource;
    let mut c = core();
    let id = register_lot(&mut c);
    c.place_service_building_internal(&id, -60.0, -30.0)
        .unwrap();
    c.publish_pending_building_site_changes();
    c.parking.fill_test_fleet();
    let owner = c.allocator.building_handles.handle(0).to_bits();
    let product = |c: &SimCore| {
        (2..8)
            .flat_map(|anchor| {
                c.parking.source_bays(PlaceSource::Building {
                    building: owner,
                    anchor,
                })
            })
            .collect::<Vec<_>>()
    };
    let before = product(&c);
    assert_eq!(before.len(), 6);
    commit(&mut c, (30.0, 0.0), (30.0, 80.0));
    c.precompute_road_mesh_data();
    c.publish_road_tool_view();
    c.publish_pending_building_site_changes();
    assert_eq!(product(&c), before);
    assert_eq!(c.parking.counts(), (6, 6, 0));
    assert!(c.undo_action_internal());
    c.precompute_road_mesh_data();
    c.publish_road_tool_view();
    c.publish_pending_building_site_changes();
    assert_eq!(product(&c), before);
    assert_eq!(c.parking.counts(), (6, 6, 0));
}

#[test]
fn parking_lot_is_public_neutral_and_round_trips_with_cars() {
    let mut c = core();
    let id = register_lot(&mut c);
    let before = c.parking.counts().0;
    let result = c.place_service_building_internal(&id, 0.0, -30.0);
    assert!(result.is_ok(), "{result:?}");
    c.publish_pending_building_site_changes();
    assert!(
        c.parking.diagnostics.is_empty(),
        "{:?}",
        c.parking.diagnostics
    );
    assert!(c.parking.counts().0 >= before + 3);
    assert_eq!(c.allocator.household_capacity(0), 0);
    assert_eq!(c.allocator.registry.worker_capacity(&id), 0);
    c.parking.fill_test_fleet();
    let count = c.parking.counts().1;
    let path = temp_save_path("parking");
    c.save_game_settled_at(path.to_str().unwrap(), None, 0)
        .unwrap();
    c.load_game_internal(path.to_str().unwrap()).unwrap();
    assert_eq!(c.parking.counts().1, count);
    assert!(c.bulldoze_building(0));
    c.publish_pending_building_site_changes();
    assert_eq!(c.parking.counts(), (0, count, count));
    assert!(c.undo_action_internal());
    c.publish_pending_building_site_changes();
    assert_eq!(c.parking.counts(), (6, count, 0));
    std::fs::remove_file(path).unwrap();
}

#[test]
fn parking_invalid_anchor_sweep_reports_diagnostic_and_tows_on_reload() {
    let mut c = core();
    let id = register_lot(&mut c);
    c.place_service_building_internal(&id, 0.0, -30.0).unwrap();
    c.publish_pending_building_site_changes();
    c.parking.fill_test_fleet();
    let cars = c.parking.counts().1;
    let mut manifest = c.allocator.registry.get(&id).unwrap().manifest.clone();
    for anchor in &mut manifest.anchors {
        if anchor.anchor_type == AnchorType::Parking {
            anchor.position = [0.0, 0.0, -8.0];
        }
    }
    c.replace_asset_registry(vec![("parking".to_owned(), manifest, String::new())]);
    assert!(!c.parking.diagnostics.is_empty());
    assert_eq!(c.parking.counts().1, cars);
    let json = c.parking.save_json(|s| s).unwrap();
    crate::simulation::parking::ParkingSystem::load_json(&json).unwrap();
}

#[test]
fn parking_road_demolition_also_retires_its_lot_driveway_supply() {
    let mut c = core();
    let id = register_lot(&mut c);
    c.place_service_building_internal(&id, 0.0, -30.0).unwrap();
    c.publish_pending_building_site_changes();
    c.parking.fill_test_fleet();
    let cars = c.parking.counts().1;
    assert!(c.bulldoze_road_edge_for_test(0));
    assert_eq!(c.parking.counts(), (0, cars, cars));
    let path = temp_save_path("parking_road_removed");
    c.save_game_settled_at(path.to_str().unwrap(), None, 0)
        .unwrap();
    c.load_game_internal(path.to_str().unwrap()).unwrap();
    assert_eq!(c.parking.counts(), (0, cars, cars));
    std::fs::remove_file(path).unwrap();
}

#[test]
fn parking_resident_save_round_trip_and_handoff_at_every_leg() {
    use crate::simulation::parking::ParkingTripPhase;
    let mut c = SimCore::new(
        WorldConfig::new(2048.0, 1024.0, 16.0, 10.0)
            .with_chunking(128.0, 0.0)
            .with_terrain_resolution(2.0),
    );
    c.treasury.balance = 1_000_000.0;
    commit(&mut c, (-600.0, 0.0), (600.0, 0.0));
    let id = register_lot(&mut c);
    let mut manifest = c.allocator.registry.get(&id).unwrap().manifest.clone();
    manifest.building.as_mut().unwrap().household_capacity = Some(4);
    c.allocator
        .registry
        .register("parking", manifest, String::new());
    c.place_service_building_internal(&id, -450.0, -30.0)
        .unwrap();
    c.place_service_building_internal(&id, 450.0, -30.0)
        .unwrap();
    c.publish_pending_building_site_changes();
    // Travel along the lot's legal driving side; loop/end-detach regressions belong to TRAFFIC-13/15.
    let eastbound = c.allocator.entrances[0].side == -1;
    let (home, target) = if eastbound { (0, 1) } else { (1, 0) };
    let agent = c.debug_add_parking_commuter(home, target).unwrap();
    assert!(
        c.parking
            .vehicle_place(c.agents.parking_trip[agent].vehicle)
            .is_some(),
        "new resident car must be parked"
    );
    assert!(
        c.parking
            .first_free_candidate(c.allocator.building_handles.handle(target).to_bits())
            .is_some(),
        "destination catchment must be published on placement"
    );
    c.agents.release_departure(agent);
    let mut seen = [false; 5];
    let path = temp_save_path("parking-resident-mid-trip");
    let mut arrived = false;
    for _ in 0..12000 {
        crate::thread_pools::install(|| {
            c.agents.tick_with_parking(
                &c.allocator,
                &mut c.transit_network,
                &mut c.region_graph,
                0.1,
                &c.time,
                &mut c.parking,
            )
        });
        c.schedule_parking_retries();
        c.parking.apply_supply_events();
        let phase = c.agents.parking_trip[agent].phase;
        let slot = match phase {
            ParkingTripPhase::WalkToCar => Some(0),
            ParkingTripPhase::PullOut => Some(1),
            ParkingTripPhase::Drive => Some(2),
            ParkingTripPhase::PullIn => Some(3),
            ParkingTripPhase::WalkToDoor => Some(4),
            _ => None,
        };
        if let Some(slot) = slot
            && !seen[slot]
        {
            seen[slot] = true;
            let before = serde_json::to_value(c.agents.parking_trip[agent]).unwrap();
            let position = (
                c.agents.pos_x[agent].to_bits(),
                c.agents.pos_y[agent].to_bits(),
            );
            c.save_game_settled_at(path.to_str().unwrap(), None, 0)
                .unwrap();
            c.load_game_internal(path.to_str().unwrap()).unwrap();
            assert_eq!(
                serde_json::to_value(c.agents.parking_trip[agent]).unwrap(),
                before
            );
            assert_eq!(
                (
                    c.agents.pos_x[agent].to_bits(),
                    c.agents.pos_y[agent].to_bits()
                ),
                position
            );
            let snapshot = c.build_snapshot();
            c.build_parked_chunks();
            let moving = snapshot
                .car_transforms
                .values()
                .map(|v| v.len() / 12)
                .sum::<usize>();
            let parked = c
                .parking
                .publications(true)
                .iter()
                .map(|(_, chunk)| chunk.identities.values().map(Vec::len).sum::<usize>())
                .sum::<usize>();
            assert_eq!(
                moving + parked,
                1,
                "handoff drew a car twice or omitted it at {phase:?}"
            );
        }
        if c.agents.current_building[agent] == target
            && c.agents.transit[agent] == crate::simulation::economy::agents::TRANSIT_IN_BUILDING
        {
            arrived = true;
            break;
        }
    }
    assert_eq!(seen, [true; 5]);
    assert!(arrived);
    std::fs::remove_file(path).unwrap();
}

#[test]
fn parking_home_demolition_cannot_start_a_door_to_door_car_leg() {
    let mut c = SimCore::new(
        WorldConfig::new(2048.0, 1024.0, 16.0, 10.0)
            .with_chunking(128.0, 0.0)
            .with_terrain_resolution(2.0),
    );
    c.treasury.balance = 1_000_000.0;
    commit(&mut c, (-600.0, 0.0), (600.0, 0.0));
    let id = register_lot(&mut c);
    let mut manifest = c.allocator.registry.get(&id).unwrap().manifest.clone();
    manifest.building.as_mut().unwrap().household_capacity = Some(4);
    c.allocator
        .registry
        .register("parking", manifest, String::new());
    c.place_service_building_internal(&id, -450.0, -30.0)
        .unwrap();
    c.place_service_building_internal(&id, 450.0, -30.0)
        .unwrap();
    c.publish_pending_building_site_changes();
    let (home, target) = if c.allocator.entrances[0].side == -1 {
        (0, 1)
    } else {
        (1, 0)
    };
    let agent = c.debug_add_parking_commuter(home, target).unwrap();
    c.agents.has_car[agent] = true;
    assert!(!c.agents.parking_trip[agent].vehicle.is_none());
    assert!(c.bulldoze_building(home));
    assert_eq!(
        c.agents.transit_mode[agent],
        crate::simulation::economy::agents::MODE_WALK
    );
}
