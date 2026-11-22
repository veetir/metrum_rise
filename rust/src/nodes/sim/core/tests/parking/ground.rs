// SPDX-License-Identifier: GPL-3.0-or-later

//! Visible graded ground in real lot publication, terrain-stroke release and undo.

use super::*;
use crate::assets::AnchorType;
use crate::nodes::sim::editing::TerrainBrushMode;
use crate::simulation::buildings::allocator::building_local_xz_basis;
use godot::prelude::Vector2;

// One street-facing bay `x` m from the lot's centre line. No imported structure bounds or
// gameplay constructor beyond the lot/road whose interaction is tested.
fn street_lot(c: &mut SimCore, x: f32) -> usize {
    let mut manifest = AssetManifest::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../godot/bootstrap/mods/parking/assets/building.parking.medium_lot/asset.toml"
    )))
    .unwrap();
    manifest.asset_id = "building.parking.street_ground_test".into();
    manifest.mesh_parts.clear();
    manifest
        .anchors
        .retain(|a| a.anchor_type == AnchorType::Parking);
    manifest.anchors.truncate(1);
    manifest.anchors[0].position = [x, 0.0, 15.0];
    manifest.anchors[0].forward = [0.0, 0.0, -1.0];
    manifest.site_surfaces[0].vertices =
        vec![[-10.0, -15.0], [10.0, -15.0], [10.0, 15.0], [-10.0, 15.0]];
    let id = manifest.qualified_id("test");
    c.allocator
        .registry
        .register("test", manifest, String::new());
    let index = c.place_service_building_internal(&id, 0.0, -30.0).unwrap();
    c.publish_pending_building_site_changes();
    index
}

// The cache reuses rendered GL0 vertices; compare its entire product bitwise with the
// authoritative point query, on both sides of the street/yard and at non-lattice positions.
fn assert_cached_surface(core: &SimCore) {
    core.with_graded_surface(|surface, scratch| {
        let mut block = crate::simulation::terrain::graded::LatticeBlock::default();
        for x in -12..=12 {
            for z in -18..=2 {
                let (x, z) = (x as f64 + 0.17, z as f64 * 1.7 + 0.23);
                let expected = surface.surface_height_at(x, z, scratch);
                let cached = surface.surface_height_in_block(x, z, &mut block, scratch);
                assert_eq!(cached.to_bits(), expected.to_bits(), "({x}, {z})");
            }
        }
    });
}

// Wheels on the level pad.
const LEVEL_X: f32 = 7.5;

#[test]
fn parking_surface_cache_requires_published_site_inputs() {
    let mut c = core();
    let index = street_lot(&mut c, LEVEL_X);
    assert!(
        c.terrain_query_index
            .current_for(&c.transit_network.roadbed, &c.allocator)
    );
    let before = c.prove_parking_lot(index).bays;
    c.allocator.reindex_site_clients();
    assert!(
        !c.terrain_query_index
            .current_for(&c.transit_network.roadbed, &c.allocator)
    );
    assert_eq!(
        c.prove_parking_lot(index).bays,
        before,
        "the live fallback is exact"
    );
    c.record_graded_site_edit(&[], false);
    assert!(
        c.terrain_query_index
            .current_for(&c.transit_network.roadbed, &c.allocator)
    );
    assert_cached_surface(&c);
}

#[test]
fn parking_visible_cut_is_refused_and_the_inspector_receives_its_measurement() {
    let mut c = core();
    let index = street_lot(&mut c, LEVEL_X);
    let owner = c.allocator.building_handles.handle(index).to_bits();
    assert_eq!(
        c.parking.building_place_count(owner),
        1,
        "{:?}",
        c.parking_rejections(index)
    );
    let b = &c.allocator.buildings[index];
    let (bx, _) = building_local_xz_basis(b.facing_dir, [0.0, 0.0, 1.0]);
    let proof = crate::simulation::parking::lots::lot_bays(
        &c.allocator,
        index,
        &c.region_graph,
        &c.transit_network.roadbed,
        |p| p.dot(bx) * 0.5,
    );
    assert!(proof.0.is_empty());
    assert_eq!(proof.1.len(), 1);
    assert!(proof.1[0].contains("slopes 50.0 %"), "{:?}", proof.1);
    // The ordinary inspector API exposes exactly the proof's issue, not just a reduced count.
    c.parking.diagnostics.insert(owner, proof.1.clone());
    assert_eq!(&*c.parking_rejections(index), &proof.1);
}

#[test]
fn parking_gameplay_brush_refuses_a_steepened_spot_and_undo_restores_it() {
    // With 1 m source cells the pad's one-cell protection halo ends 1 m outside the pad. A spot
    // whose outer wheels stand beyond the halo stands on ground the brush can lower.
    let mut c = SimCore::new(
        WorldConfig::new(512.0, 512.0, 16.0, 10.0)
            .with_chunking(128.0, 0.0)
            .with_terrain_resolution(1.0),
    );
    c.treasury.balance = 1_000_000.0;
    commit(&mut c, (-120.0, 0.0), (120.0, 0.0));
    let index = street_lot(&mut c, 8.7);
    let owner = c.allocator.building_handles.handle(index).to_bits();
    assert_eq!(
        c.parking.building_place_count(owner),
        1,
        "{:?}",
        c.parking_rejections(index)
    );
    let before = c.prove_parking_lot(index).bays;
    let b = &c.allocator.buildings[index];
    let (bx, bz) = building_local_xz_basis(b.facing_dir, [0.0, 0.0, 1.0]);
    let cursor = Vector2::new(b.center_x, b.center_y) + bx * 12.0 + bz * 10.0;
    assert_cached_surface(&c);
    c.begin_game_terrain_stroke(
        TerrainBrushMode::Lower,
        cursor,
        10.0,
        2.0,
        Vector3::ZERO,
        Vector3::ZERO,
    );
    for _ in 0..8 {
        c.game_terrain_stroke_step(cursor, 0.5);
    }
    assert!(c.finish_game_terrain_stroke());
    assert_cached_surface(&c);
    let fresh = c.prove_parking_lot(index);
    assert!(
        fresh.ground_refused,
        "the stroke must tilt the spot: {:?}",
        fresh.issues
    );
    assert_eq!(c.parking.building_place_count(owner), 0);
    let reasons = c.parking_rejections(index).into_owned();
    assert_eq!(reasons, fresh.issues);
    assert_eq!(reasons.len(), 1, "{reasons:?}");
    assert!(
        reasons[0].contains("not drivable: the ground under this spot slopes"),
        "{reasons:?}"
    );

    // A save from before the ground check (format 125) can still hold the now-undrivable row.
    // Loading must remove it using the same proof without changing the stored floor or relying
    // on an inspector visit.
    let floor = c.allocator.buildings[index].support_height_m;
    c.parking.replace_building(owner, &before);
    let path = temp_save_path("parking-undrivable-ground");
    c.save_game_settled_at(path.to_str().unwrap(), None, 0)
        .unwrap();
    rusqlite::Connection::open(&path)
        .unwrap()
        .execute_batch(&format!(
            "UPDATE save_meta SET version = {};",
            crate::simulation::save::schema::PRE_GROUND_SAVE_VERSION
        ))
        .unwrap();
    let mut loaded = core();
    loaded.allocator.registry = c.allocator.registry.clone();
    loaded.load_game_internal(path.to_str().unwrap()).unwrap();
    std::fs::remove_file(path).unwrap();
    assert_eq!(loaded.parking.building_place_count(owner), 0);
    assert_eq!(
        loaded.allocator.buildings[index].support_height_m.to_bits(),
        floor.to_bits()
    );
    assert_eq!(
        loaded.parking_rejections(index).as_ref(),
        reasons.as_slice()
    );
    c.parking.replace_building(owner, &fresh.bays);

    assert!(c.undo_action_internal());
    assert_eq!(
        c.parking.building_place_count(owner),
        1,
        "{:?}",
        c.parking_rejections(index)
    );
    assert!(c.parking_rejections(index).is_empty());
    assert_eq!(c.prove_parking_lot(index).bays, before);
}
