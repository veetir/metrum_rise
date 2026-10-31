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
