// SPDX-License-Identifier: GPL-3.0-or-later

//! Terrain authoring changes growth feasibility while retaining authoritative cell paint.

use super::*;
use godot::prelude::Vector2;

#[test]
fn terrain_stroke_revalidates_cell_growth_and_undo_restores_feasibility() {
    for angle in [0.0, 0.35] {
        let mut core = painted_fixture(angle);
        core.benchmark_mode = false;
        core.prepare_cell_lots_internal();
        let candidates = core.allocator.collect_demand_spawn_candidates_by_use(
            &core.zoning,
            &core.region_graph,
            core.demand.runtime_catalog(),
            &[],
            environment(&core),
        );
        let action = candidates.residential[0].action.clone();
        let lot = core
            .zoning
            .parcel_by_raw_id(action.parcel_id)
            .unwrap()
            .clone();
        let paint = core.zoning.cells.saved_cells();
        let frames: Vec<_> = core.zoning.cells.saved_frames().collect();
        let preview = core.preview_cell_selection_internal(
            CellSelectionShape::Cell,
            &[DVec2::new(angle.cos(), angle.sin()).perp() * -15.0],
        );
        let undo_count = core.undo_stack.len();
        let revision = core.zoning.cell_external_revision();
        let terrain_generation = core.heightmap.source_generation();
        // The 64 m tie-in can pass a narrow mound. Put the broad mound beyond the lot so its
        // fringe blocks the tie-in while the brush remains clear of the frontage road.
        let away = DVec2::new(angle.cos(), angle.sin()).perp() * 110.0;
        let center = lot.center() + Vector2::new(away.x as f32, away.y as f32);
        core.start_terrain_stroke_internal();
        core.sculpt_terrain_stroke_step_internal(Vector2::new(center.x, center.y), 120.0, 40.0);
        assert!(core.end_terrain_stroke_internal());
        assert_eq!(core.undo_stack.len(), undo_count + 1);
        assert!(core.heightmap.source_generation() > terrain_generation);
        assert!(core.zoning.cell_external_revision() > revision);
        assert!(!core.apply_cell_selection_internal(&preview, 1));
        assert_eq!(core.zoning.cells.saved_cells(), paint);
        for (id, frame) in &frames {
            assert_eq!(core.zoning.cells.frame(*id), Some(*frame));
        }
        let after = core.zoning.parcel_by_raw_id(action.parcel_id).unwrap();
        assert_eq!(after.corners(), lot.corners());
        assert_eq!(after.cell_lot(), lot.cell_lot());
        assert_eq!(
            after.yard.as_ref().unwrap().verdict,
            crate::simulation::buildings::allocator::YardVerdict::Impossible,
            "terrain release publishes quality before an hourly planner query"
        );
        let blocked = core.allocator.collect_demand_spawn_candidates_by_use(
            &core.zoning,
            &core.region_graph,
            core.demand.runtime_catalog(),
            &[],
            environment(&core),
        );
        assert!(
            !blocked
                .residential
                .iter()
                .any(|c| c.action.parcel_id == action.parcel_id)
        );
        assert!(
            core.allocator
                .execute_demand_spawn_action(
                    &action,
                    &mut core.zoning,
                    &core.region_graph,
                    &core.transit_network.roadbed,
                    &core.heightmap,
                    core.demand.runtime_catalog(),
                    core.demand.runtime_tuning(),
                )
                .is_err(),
            "a queued action must revalidate the sculpted site"
        );
        assert!(core.allocator.buildings.is_empty());
        assert!(core.undo_action_internal());
        core.rebuild_network_surface_terrain_internal();
        core.prepare_cell_lots_internal();
        assert_eq!(core.zoning.cells.saved_cells(), paint);
        let restored = core.allocator.collect_demand_spawn_candidates_by_use(
            &core.zoning,
            &core.region_graph,
            core.demand.runtime_catalog(),
            &[],
            environment(&core),
        );
        assert!(
            restored
                .residential
                .iter()
                .any(|c| c.action.parcel_id == action.parcel_id)
        );
        let index = core
            .allocator
            .execute_demand_spawn_action(
                &action,
                &mut core.zoning,
                &core.region_graph,
                &core.transit_network.roadbed,
                &core.heightmap,
                core.demand.runtime_catalog(),
                core.demand.runtime_tuning(),
            )
            .unwrap();
        assert_eq!(core.allocator.buildings[index].parcel_id, action.parcel_id);
        assert_eq!(
            core.zoning
                .parcel_by_raw_id(action.parcel_id)
                .unwrap()
                .corners(),
            lot.corners()
        );
    }
}

#[test]
fn zoning_paint_release_publishes_a_steep_lot_verdict() {
    use crate::simulation::buildings::allocator::YardVerdict;
    let mut core = painted_fixture(0.0);
    core.prepare_cell_lots_internal();
    // Erase/repaint one whole existing lot; the release must rebuild its verdict immediately.
    let lot = core
        .zoning
        .parcels()
        .iter()
        .find(|p| p.is_available())
        .unwrap()
        .clone();
    let preview = core.preview_cell_selection_internal(
        CellSelectionShape::Marquee,
        &[
            DVec2::new(
                lot.aabb_min().x as f64 + 0.01,
                lot.aabb_min().y as f64 + 0.01,
            ),
            DVec2::new(
                lot.aabb_max().x as f64 - 0.01,
                lot.aabb_max().y as f64 - 0.01,
            ),
        ],
    );
    assert!(core.apply_cell_selection_internal(&preview, 0));
    let preview = core.preview_cell_selection_internal(
        CellSelectionShape::Marquee,
        &[
            DVec2::new(
                lot.aabb_min().x as f64 + 0.01,
                lot.aabb_min().y as f64 + 0.01,
            ),
            DVec2::new(
                lot.aabb_max().x as f64 - 0.01,
                lot.aabb_max().y as f64 - 0.01,
            ),
        ],
    );
    // Keep the road fixed while every source ring lies 200 normalized units above it.
    let bounds = [
        lot.aabb_min().x - 75.0,
        lot.aabb_min().y - 75.0,
        lot.aabb_max().x + 75.0,
        lot.aabb_max().y + 75.0,
    ];
    let rect = core
        .heightmap
        .grid_rect_for_world_bounds(bounds[0], bounds[1], bounds[2], bounds[3])
        .unwrap();
    for z in rect.2..=rect.3 {
        for x in rect.0..=rect.1 {
            core.heightmap.set_height(x, z, 200.0);
        }
    }
    // The preview is stale after ground editing, so regenerate it as the tool does.
    let _ = preview;
    core.record_graded_base_edit(bounds.map(f64::from));
    let preview = core.preview_cell_selection_internal(
        CellSelectionShape::Marquee,
        &[
            DVec2::new(
                lot.aabb_min().x as f64 + 0.01,
                lot.aabb_min().y as f64 + 0.01,
            ),
            DVec2::new(
                lot.aabb_max().x as f64 - 0.01,
                lot.aabb_max().y as f64 - 0.01,
            ),
        ],
    );
    assert!(core.apply_cell_selection_internal(&preview, 1));
    let lot = core.zoning.parcel_at(lot.center()).unwrap();
    let verdict = lot
        .yard
        .as_ref()
        .expect("released paint has a stored verdict");
    assert_eq!(verdict.verdict, YardVerdict::Impossible);
    assert!(!verdict.reason.is_empty());
}

#[test]
#[ignore = "matched unprofiled paint-release timing with eight workers"]
fn benchmark_zoning_paint_release_yards() {
    use std::time::Instant;
    let mut core = test_core();
    road_terrain_plan::commit_ready(
        &mut core,
        vec![
            Vector3::new(-1024.0, 0.0, 0.0),
            Vector3::new(1024.0, 0.0, 0.0),
        ],
    );
    let assets = if std::env::var("METRUM_PAINT_ASSETS").as_deref() == Ok("houses") {
        let fixture: Fixture = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../benchmarks/fixtures/kuopio-terrain/residential-buildability.json"
        )))
        .unwrap();
        register_houses(&mut core, fixture.assets);
        2
    } else {
        register_test_asset(&mut core.allocator, "paint_yard", ZoneType::Residential);
        1
    };
    let path = [DVec2::new(-1000.0, -65.0), DVec2::new(1000.0, 65.0)];
    let preview = core.preview_cell_selection_internal(CellSelectionShape::Marquee, &path);
    let before = core.allocator.site_feasibility_solve_count();
    let started = Instant::now();
    assert!(core.apply_cell_selection_internal(&preview, 1));
    let elapsed = started.elapsed().as_secs_f64() * 1000.0;
    let lots = core.zoning.parcels().len();
    assert!((180..=220).contains(&lots), "painted {lots} lots");
    assert!(core.zoning.parcels().iter().all(|p| p.yard.is_some()));
    eprintln!(
        "paint_release_yards lots={lots} assets={assets} release_ms={elapsed:.3} solves={}",
        core.allocator.site_feasibility_solve_count() - before
    );
}
