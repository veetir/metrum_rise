// SPDX-License-Identifier: GPL-2.0-only

//! Opt-in read-only audits of imported junction geometry using production compilation.

use super::*;
use std::collections::BTreeSet;

#[test]
fn kys_crossfall_uses_control_supports_without_rewriting_physical_relief() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/kys_slip_bend.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let original = graph.edges().to_vec();
    let plane = graph.solved_junction_endpoint_profile_plane(0).unwrap();
    let pos = graph.node(0).pos;
    let grade = (plane.height_at_xz(pos.x + 1.0, pos.z) - pos.y)
        .hypot(plane.height_at_xz(pos.x, pos.z + 1.0) - pos.y);
    assert!(
        grade < 0.07,
        "KYS control grade is 6.63%, not the eased profile's 245%: {grade}"
    );
    let config = WorldConfig::new(2500.0, 2500.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_partial_for_geometry_audit(&graph, &terrain);
    for sections in surface.compiled_sections().values() {
        for section in sections.iter() {
            for band in &section.bands {
                assert!((band.height_start_m - section.center_height_m).abs() < 0.5);
                assert!((band.height_end_m - section.center_height_m).abs() < 0.5);
            }
        }
    }
    for (edge, original) in graph.edges().iter().zip(original) {
        assert_eq!(edge.physical_geometry, original.physical_geometry);
        assert_eq!(edge.geometry, original.geometry);
    }
}

#[test]
#[ignore = "read-only imported-map audit; set METRUM_JUNCTION_SAVE and METRUM_JUNCTION_OUTPUT"]
fn audit_imported_junction_geometry() {
    let path = std::env::var("METRUM_JUNCTION_SAVE").unwrap();
    let output = std::path::PathBuf::from(std::env::var("METRUM_JUNCTION_OUTPUT").unwrap());
    fs::create_dir_all(&output).unwrap();
    let conn =
        rusqlite::Connection::open_with_flags(&path, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
            .unwrap();
    let config = conn.query_row(
        "SELECT width_m,height_m,terrain_cell_m,terrain_chunk_m,terrain_base_elevation_m,env_cell_m,zone_cell_m FROM world_config LIMIT 1", [],
        |r| Ok(WorldConfig::new(r.get(0)?,r.get(1)?,r.get(5)?,r.get(6)?)
            .with_terrain_resolution(r.get(2)?).with_chunking(r.get(3)?,r.get(4)?)),
    ).unwrap();
    let mut terrain = world::load_terrain(&conn, &config).unwrap();
    let mut graph = network::load_graph(&conn).unwrap();
    let mut transit = TransitNetwork::new_for_world(&config);
    transit
        .road_surface
        .compile_partial_for_geometry_audit(&graph, &terrain);
    let surface = &transit.road_surface;
    let missing_spans = (0..graph.edge_count())
        .filter(|id| !surface.compiled_visual_span_pieces().contains_key(id))
        .collect::<Vec<_>>();
    let mut walls = Vec::new();
    for (&id, piece) in surface.compiled_visual_node_pieces() {
        let max_step = piece
            .raised_step_face_polygons
            .iter()
            .flat_map(|p| {
                p.points_world.iter().enumerate().flat_map(move |(i, a)| {
                    p.points_world[i + 1..].iter().filter_map(move |b| {
                        ((a.x - b.x).hypot(a.z - b.z) < 0.001).then_some((a.y - b.y).abs())
                    })
                })
            })
            .fold(0.0_f64, f64::max);
        if max_step > 0.25 {
            walls.push((id, max_step));
        }
    }
    walls.sort_by_key(|v| v.0);
    let mut terrain_failures = Vec::new();
    for &id in surface.compiled_visual_node_pieces().keys() {
        let pos = graph.node(id).pos;
        if let Err(err) = surface.terrain_cdt_road_loops_for_world_bounds(
            &graph,
            pos.x - 24.0,
            pos.z - 24.0,
            pos.x + 24.0,
            pos.z + 24.0,
        ) {
            terrain_failures.push((id, format!("{err:?}")));
        }
    }
    terrain_failures.sort_by_key(|v| v.0);
    let mut outliers = Vec::new();
    let mut selected = BTreeSet::from([12, 47, 123, 136, 137, 140, 45, 61, 62, 321, 322]);
    for (&edge, sections) in surface.compiled_sections() {
        let delta = sections
            .iter()
            .flat_map(|s| {
                s.bands.iter().map(move |b| {
                    (b.height_start_m - s.center_height_m)
                        .abs()
                        .max((b.height_end_m - s.center_height_m).abs())
                })
            })
            .fold(0.0_f32, f32::max);
        if delta > 1.0 {
            outliers.push((edge, delta));
            selected.insert(edge);
        }
    }
    outliers.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    for &node in surface.failed_node_ids() {
        selected.extend(graph.node_adjacency(node));
    }
    let mut steep_source_profiles = Vec::new();
    for (id, edge) in graph.edges().iter().enumerate() {
        let maximum = edge
            .physical_geometry
            .windows(2)
            .filter_map(|p| {
                let run = (p[1].x - p[0].x).hypot(p[1].z - p[0].z);
                (run > f32::EPSILON).then_some((p[1].y - p[0].y).abs() / run)
            })
            .fold(0.0_f32, f32::max);
        if maximum > 0.5 {
            steep_source_profiles.push((id, maximum));
        }
    }
    let mut nodes = Vec::new();
    for id in 0..graph.node_count() as u32 {
        if let Some(plane) = graph.solved_junction_endpoint_profile_plane(id) {
            let pos = graph.node(id).pos;
            let gx = plane.height_at_xz(pos.x + 1.0, pos.z) - pos.y;
            let gz = plane.height_at_xz(pos.x, pos.z + 1.0) - pos.y;
            if gx.hypot(gz) > 0.2 {
                nodes.push(serde_json::json!({"id":id,"pos":[pos.x,pos.y,pos.z],"grade":gx.hypot(gz),"edges":graph.node_adjacency(id)}));
            }
        }
    }
    let report = serde_json::json!({"save":path,"published":surface.published_generation_matches_source(),
        "failure":surface.last_compile_failure_label(),"failed_nodes":surface.failed_node_ids(),
        "missing_spans":missing_spans,"oversized_kerb_steps":walls,"terrain_export_failures":terrain_failures,
        "steep_source_profiles":steep_source_profiles,"section_outliers":outliers,"steep_node_planes":nodes,
        "spans":surface.compiled_visual_span_pieces().len(),"nodes":surface.compiled_visual_node_pieces().len()});
    fs::write(
        output.join("audit.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    eprintln!("{}", report);
    if !selected.is_empty() {
        fs::write(
            output.join("geometry.txt"),
            surface.build_edge_geometry_debug_dump(
                &graph,
                &terrain,
                &selected.into_iter().collect::<Vec<_>>(),
            ),
        )
        .unwrap();
    }
    let mut transit = TransitNetwork::new_for_world(&config);
    let loaded = network::rebuild_loaded_graph_runtime(&mut graph, &mut transit, &mut terrain);
    eprintln!("load_runtime={loaded:?}");
}

#[test]
fn overlapping_bioteknia_profiles_resolve_one_continuous_longitudinal_grade() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/bioteknia_overlap.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let original = graph.edges().to_vec();
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &[5], &[], false);
    let sections = &surface.compiled_sections()[&5];
    assert!(sections.len() > 2);
    for pair in sections.windows(2) {
        let run = (pair[1].center_xz - pair[0].center_xz).length();
        assert!(
            f64::from((pair[1].center_height_m - pair[0].center_height_m).abs())
                <= run * 0.16 + 0.0001,
            "overlapping junction profiles must not leave a 331% ramp: {pair:?}"
        );
    }
    for pair in sections.windows(2) {
        for (a, b) in pair[0].bands.iter().zip(&pair[1].bands) {
            if a.kind != crate::simulation::network::surface::RoadSurfaceBandKind::Carriageway {
                continue;
            }
            for ((lateral_a, height_a), (lateral_b, height_b)) in [
                (
                    (a.lateral_start_m, a.height_start_m),
                    (b.lateral_start_m, b.height_start_m),
                ),
                (
                    (a.lateral_end_m, a.height_end_m),
                    (b.lateral_end_m, b.height_end_m),
                ),
            ] {
                let xz_a = pair[0].center_xz + pair[0].lateral_xz * f64::from(lateral_a);
                let xz_b = pair[1].center_xz + pair[1].lateral_xz * f64::from(lateral_b);
                assert!(
                    f64::from((height_b - height_a).abs()) <= (xz_b - xz_a).length() * 0.5 + 0.001,
                    "crossfall blending must not create a wall beside a valid centreline: stations={:?}, heights={:?}, run={}",
                    [pair[0].s_m, pair[1].s_m],
                    [height_a, height_b],
                    (xz_b - xz_a).length()
                );
            }
        }
    }
    for (edge, original) in graph.edges().iter().zip(original) {
        assert_eq!(edge.physical_geometry, original.physical_geometry);
        assert_eq!(edge.geometry, original.geometry);
    }
}
