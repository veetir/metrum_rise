// SPDX-License-Identifier: GPL-3.0-or-later

//! Portable hillside roadside-site and feasibility invalidation regressions.

mod cell_zoning;

use super::*;
use crate::simulation::buildings::allocator::BuildingSiteEnvironment;

#[derive(serde::Deserialize)]
struct Fixture {
    width: usize,
    height: usize,
    cell_m: f32,
    x0: usize,
    z0: usize,
    samples: Vec<Vec<f32>>,
    nodes: Vec<(u32, f32, f32, f32)>,
    edges: Vec<FixtureEdge>,
    parcels: Vec<(u64, usize, i8, f32, f32, f32, u16)>,
    assets: Vec<String>,
}

#[derive(serde::Deserialize)]
struct FixtureEdge {
    id: usize,
    start: u32,
    end: u32,
    width: f32,
    start_clip: f32,
    end_clip: f32,
    // Control polyline and physical profile, as a save holds them.
    points: Vec<[f32; 3]>,
    physical: Vec<[f32; 3]>,
}

fn fixture() -> SimCore {
    let f: Fixture = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../benchmarks/fixtures/kuopio-terrain/residential-buildability.json"
    )))
    .unwrap();
    let mut core = test_core();
    core.config.width_m = (f.width - 1) as f32 * f.cell_m;
    core.config.height_m = (f.height - 1) as f32 * f.cell_m;
    core.config.terrain_cell_m = f.cell_m;
    core.config.terrain_chunk_m = 512.0;
    core.heightmap = TerrainSystem::from_world_config(&core.config);
    core.transit_network = TransitNetwork::new_for_world(&core.config);
    core.zoning = ZoningSystem::new(&core.config);
    for (z, row) in f.samples.iter().enumerate() {
        for (x, &height) in row.iter().enumerate() {
            core.heightmap.set_height(f.x0 + x, f.z0 + z, height);
        }
    }
    for (id, x, y, z) in f.nodes {
        assert_eq!(
            core.region_graph
                .add_node(Vector3::new(x, y, z), NodeType::Junction),
            id
        );
    }
    let points = |points: &[[f32; 3]]| -> Vec<_> {
        points
            .iter()
            .map(|p| Vector3::new(p[0], p[1], p[2]))
            .collect()
    };
    for e in f.edges {
        let mut edge = Edge {
            start_node: e.start,
            end_node: e.end,
            width: e.width,
            primary_type: TransitType::Road,
            allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
            sidewalks: crate::simulation::network::graph::SidewalkSides::BOTH,
            fwd_lanes: 1,
            bkw_lanes: 1,
            start_clip: e.start_clip,
            end_clip: e.end_clip,
            geometry: points(&e.points),
            physical_geometry: points(&e.physical),
            ..Default::default()
        };
        (edge.base_cost, edge.physical_length) =
            crate::simulation::pathing::cost::CostCalculator::calculate_costs(&edge);
        assert_eq!(core.region_graph.add_edge(edge), e.id);
        core.transit_network.roadbed.mark_edge_dirty(e.id);
    }
    core.region_graph.rebuild_adjacency_list();
    core.precompute_road_mesh_data();
    // Regenerated with `ROAD-68-E`'s load rebuild (open question 8): every node builds as saved.
    let roadbed = &core.transit_network.roadbed;
    assert!(
        roadbed.is_current() && roadbed.failed_nodes().is_empty(),
        "{:?}",
        roadbed.failed_nodes()
    );
    for (id, edge, side, t, width, depth, profile) in f.parcels {
        core.zoning
            .restore_parcel_from_attachment(
                id,
                edge,
                side,
                t,
                width,
                depth,
                profile,
                &core.region_graph,
            )
            .unwrap();
    }
    register_houses(&mut core, f.assets);
    core
}

fn register_houses(core: &mut SimCore, assets: Vec<String>) {
    for asset in assets {
        let mut manifest: AssetManifest = toml::from_str(&asset).unwrap();
        // Keep this feasibility fixture's structural box geometry explicit and engine-independent.
        for part in &mut manifest.mesh_parts {
            part.imported_bounds = Some([[-0.75, 0.0, -0.75], [0.75, 1.0, 0.75]]);
        }
        core.allocator
            .registry
            .register("kenney", manifest, String::new());
    }
}

fn flat_fixture() -> SimCore {
    let mut core = test_core();
    road_terrain_plan::commit_ready(
        &mut core,
        vec![Vector3::new(-96.0, 0.0, 0.0), Vector3::new(96.0, 0.0, 0.0)],
    );
    let f: Fixture = serde_json::from_str(include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../benchmarks/fixtures/kuopio-terrain/residential-buildability.json"
    )))
    .unwrap();
    register_houses(&mut core, f.assets);
    core.zoning
        .restore_parcel_from_attachment(7, 0, -1, 0.5, 20.0, 20.0, 1, &core.region_graph)
        .unwrap();
    core
}

fn environment(core: &SimCore) -> BuildingSiteEnvironment<'_> {
    BuildingSiteEnvironment {
        road_surface: &core.transit_network.roadbed,
        terrain: &core.heightmap,
    }
}

#[test]
fn missing_assets_allow_zoning_and_growth_waits_for_a_compatible_initial_asset() {
    let mut core = flat_fixture();
    let mut house = core
        .allocator
        .registry
        .get("kenney:building.residential.family_single_house")
        .unwrap()
        .manifest
        .clone();
    let parcel = core.zoning.parcel_by_raw_id(7).unwrap();
    let geometry = core
        .zoning
        .parcel_geometry_at(parcel.center().x, parcel.center().y)
        .unwrap();
    core.allocator.registry.clear();
    for profile in core.zoning.profiles.profiles() {
        assert_eq!(
            core.allocator.zoning_site_feasibility(
                &geometry,
                profile.runtime_id,
                &core.zoning,
                &core.region_graph,
                environment(&core),
            ),
            Ok(()),
            "missing assets must not block {}",
            profile.id
        );
    }
    core.zoning
        .place_or_rezone_parcel_at(
            geometry.center.x,
            geometry.center.y,
            2,
            geometry.frontage_m,
            geometry.depth_m,
            &core.region_graph,
        )
        .unwrap();
    let candidates = |core: &SimCore| {
        core.allocator.collect_demand_spawn_candidates_by_use(
            &core.zoning,
            &core.region_graph,
            core.demand.runtime_catalog(),
            &[],
            environment(core),
        )
    };
    let empty = candidates(&core);
    assert!(
        empty.residential.is_empty() && empty.commercial.is_empty() && empty.industrial.is_empty()
    );
    for (density, level, width, can_grow) in [
        ("low", 1, 2, false),
        ("medium", 2, 2, false),
        ("medium", 1, 20, false),
        ("medium", 1, 2, true),
    ] {
        let data = house.building.as_mut().unwrap();
        data.density = Some(density.to_owned());
        data.level = level;
        data.lot_width_cells = width;
        core.allocator.registry.clear();
        core.allocator
            .registry
            .register("kenney", house.clone(), String::new());
        let solves = core.allocator.site_feasibility_solve_count();
        assert_eq!(
            core.allocator.zoning_site_feasibility(
                &geometry,
                2,
                &core.zoning,
                &core.region_graph,
                environment(&core),
            ),
            Ok(())
        );
        assert_eq!(
            core.allocator.site_feasibility_solve_count(),
            solves,
            "asset changes cannot change the parcel terrain calculation"
        );
        assert_eq!(
            candidates(&core)
                .residential
                .iter()
                .any(|c| c.action.parcel_id == 7),
            can_grow,
            "density={density}, level={level}, width={width}"
        );
        assert_eq!(
            core.zoning
                .parcel_by_raw_id(7)
                .unwrap()
                .zone_profile_runtime_id(),
            2
        );
        assert!(core.allocator.buildings.is_empty());
    }
}

#[test]
fn legacy_hillside_houses_retain_their_floors_and_terrain_feasibility_cache() {
    hillside_houses();
}

// The hillside's five empty lots, checked for terrain-neutral feasibility across all profiles and
// built, the small house on parcel 7: with the tie-in rays at the graded terrain's 64 m cap
// (`docs/terrain.md` T12 (a)) it grades into the slope that refused it at the exact cut's 20 m
// margin, where the family house was the constructible one.
fn hillside_houses() -> SimCore {
    let mut core = fixture();
    let before = (
        core.zoning.parcels().len(),
        core.allocator.buildings.len(),
        core.heightmap.source_generation(),
        core.heightmap.source_generation(),
    );
    for id in [6, 7, 8, 9, 16] {
        let p = core.zoning.parcel_by_raw_id(id).unwrap();
        let geometry = core
            .zoning
            .parcel_geometry_at(p.center().x, p.center().y)
            .unwrap();
        let verdict = core.allocator.zoning_site_feasibility(
            &geometry,
            1,
            &core.zoning,
            &core.region_graph,
            environment(&core),
        );
        for profile in core.zoning.profiles.profiles() {
            assert_eq!(
                core.allocator.zoning_site_feasibility(
                    &geometry,
                    profile.runtime_id,
                    &core.zoning,
                    &core.region_graph,
                    environment(&core)
                ),
                verdict,
                "parcel {id}, profile {}",
                profile.id
            );
        }
    }
    assert_eq!(
        before,
        (
            core.zoning.parcels().len(),
            core.allocator.buildings.len(),
            core.heightmap.source_generation(),
            core.heightmap.source_generation()
        ),
        "zoning must not engineer the ground"
    );
    // Keep every established hillside site for terrain bake coverage. New construction's
    // stricter street-contact verdict is tested independently below.
    for id in [6, 7, 8, 9, 16] {
        let asset = if id == 7 {
            "kenney:building.residential.small_single_family_house"
        } else {
            "kenney:building.residential.family_single_house"
        };
        core.allocator.restore_legacy_site_fixture(
            asset,
            Some(id),
            godot::prelude::Vector2::ZERO,
            &mut core.zoning,
            &core.region_graph,
            &core.transit_network.roadbed,
            core.demand.runtime_catalog(),
            core.demand.runtime_tuning(),
        );
        core.publish_pending_building_site_changes();
    }
    assert_eq!(core.allocator.buildings.len(), 5);
    core
}

// The graded terrain after the hillside houses (`TERRAIN-07-C`, re-pointed from the refined CDT
// patches): every site's tiles bake without errors at every level, the pads hold their support
// heights bitwise and the yards take the partition's materials.
#[test]
fn hillside_houses_terrain_builds_clean() {
    let mut core = hillside_houses();
    core.allocator
        .prepare_building_site_query_index(core.config.zone_cell_m);
    use crate::simulation::terrain::graded::site_tests::assert_allocator_sites_bake;
    let (baked, pads, _) = assert_allocator_sites_bake(
        &core.heightmap,
        &core.transit_network.roadbed,
        &core.allocator,
    );
    assert!(baked > 0 && pads > 1_000, "{baked} tiles, {pads} pad samples");
}

#[test]
fn feasibility_invalidates_changed_terrain_and_assets() {
    use crate::simulation::buildings::allocator::YardVerdict;
    let mut core = flat_fixture();
    let check = |core: &SimCore| {
        let assets = core
            .allocator
            .collect_spawn_asset_candidates_by_profile(&core.zoning, core.demand.runtime_catalog());
        let p = core.zoning.parcel_by_raw_id(7).unwrap();
        core.allocator.evaluate_lot_yard(
            p,
            &core.zoning,
            &core.region_graph,
            assets.get(&p.zone_profile_runtime_id()),
            environment(core),
        )
    };
    assert_eq!(check(&core).verdict, YardVerdict::Good);
    let solves = core.allocator.site_feasibility_solve_count();
    assert_eq!(check(&core).verdict, YardVerdict::Good);
    assert_eq!(core.allocator.site_feasibility_solve_count(), solves);
    core.heightmap.set_height(0, 0, 30.0);
    assert_eq!(check(&core).verdict, YardVerdict::Good);
    assert_eq!(
        core.allocator.site_feasibility_solve_count(),
        solves,
        "remote terrain keeps local cache"
    );
    core.transit_network.roadbed.mark_edge_dirty(0);
    assert_eq!(
        check(&core).verdict,
        YardVerdict::Impossible,
        "stale roads cannot authorize growth"
    );
    core.precompute_road_mesh_data();
    assert_eq!(check(&core).verdict, YardVerdict::Good);
    let p = core.zoning.parcel_by_raw_id(7).unwrap();
    let rect = core
        .heightmap
        .grid_rect_for_world_bounds(
            p.aabb_min().x - 75.0,
            p.aabb_min().y - 75.0,
            p.aabb_max().x + 75.0,
            p.aabb_max().y + 75.0,
        )
        .unwrap();
    // Every ray now sees 200 normalized units, far beyond a 64 m × 1:2 cone + 3 m.
    for z in rect.2..=rect.3 {
        for x in rect.0..=rect.1 {
            core.heightmap.set_height(x, z, 200.0);
        }
    }
    assert_eq!(check(&core).verdict, YardVerdict::Impossible);
    let before = core.allocator.buildings.len();
    assert!(
        core.allocator
            .execute_demand_spawn_action(
                &DemandSpawnAction {
                    parcel_id: 7,
                    asset_id: "kenney:building.residential.family_single_house".into(),
                },
                &mut core.zoning,
                &core.region_graph,
                &core.transit_network.roadbed,
                &core.heightmap,
                core.demand.runtime_catalog(),
                core.demand.runtime_tuning()
            )
            .is_err()
    );
    assert_eq!(core.allocator.buildings.len(), before);
    core.allocator.registry.clear();
    assert_eq!(check(&core).verdict, YardVerdict::Impossible);
    let p = core.zoning.parcel_by_raw_id(7).unwrap();
    let geometry = core
        .zoning
        .parcel_geometry_at(p.center().x, p.center().y)
        .unwrap();
    assert!(
        core.allocator
            .zoning_site_feasibility(
                &geometry,
                1,
                &core.zoning,
                &core.region_graph,
                environment(&core)
            )
            .is_ok(),
        "terrain quality is displayed after paint, never silently refuses zoning"
    );
}

#[test]
fn hillside_placement_records_fixed_street_contact_verdicts() {
    use crate::simulation::buildings::allocator::YardVerdict;
    let mut core = fixture();
    core.refresh_dirty_lot_yards();
    let records: Vec<_> = [6, 7, 8, 9, 16]
        .map(|id| {
            let yard = core
                .zoning
                .parcel_by_raw_id(id)
                .unwrap()
                .yard
                .as_ref()
                .unwrap();
            (id, yard.verdict)
        })
        .into();
    assert_eq!(
        records,
        vec![
            (6, YardVerdict::Good),
            (7, YardVerdict::Rough),
            (8, YardVerdict::Rough),
            (9, YardVerdict::Good),
            (16, YardVerdict::Rough)
        ]
    );
}
