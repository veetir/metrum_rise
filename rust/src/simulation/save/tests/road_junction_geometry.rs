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
fn university_import_parking_uses_the_certified_common_grade_for_folded_carriers() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/university_import_parking.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let original = graph.edges().to_vec();
    let config = WorldConfig::new(3000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 80.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_partial_for_geometry_audit(&graph, &terrain);
    assert!(
        surface.failed_node_ids().is_empty(),
        "{:?}",
        surface.last_compile_failure_label()
    );
    assert!(surface.compiled_visual_node_pieces().contains_key(&3));
    for (edge, original) in graph.edges().iter().zip(original) {
        assert_eq!(edge.geometry, original.geometry);
        assert_eq!(edge.physical_geometry, original.physical_geometry);
        assert_eq!(edge.median, original.median);
        assert_eq!(
            (edge.fwd_lanes, edge.bkw_lanes),
            (original.fwd_lanes, original.bkw_lanes)
        );
    }
}

#[test]
#[ignore = "read-only imported-map audit; set METRUM_JUNCTION_SAVE and METRUM_JUNCTION_OUTPUT"]
fn audit_imported_junction_geometry() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
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
    let probe = std::env::var("METRUM_JUNCTION_PROBE_NODES");
    if let Ok(selection) = &probe {
        let nodes = selection
            .split(',')
            .map(|id| id.parse::<u32>().unwrap())
            .collect::<Vec<_>>();
        let mut edges = nodes
            .iter()
            .flat_map(|&id| graph.node_adjacency(id))
            .copied()
            .collect::<Vec<_>>();
        edges.push(992);
        edges.sort_unstable();
        edges.dedup();
        transit.road_surface.compile_owners_for_geometry_audit(
            &graph,
            &terrain,
            &edges,
            &nodes,
            std::env::var("METRUM_JUNCTION_PROBE_CHORDS").as_deref() == Ok("1"),
        );
        fs::write(
            output.join("geometry.txt"),
            transit
                .road_surface
                .build_edge_geometry_debug_dump(&graph, &terrain, &edges),
        )
        .unwrap();
        eprintln!(
            "probe spans={:?} nodes={:?}",
            transit.road_surface.compiled_visual_span_pieces().keys(),
            transit.road_surface.compiled_visual_node_pieces().keys()
        );
        for &id in &nodes {
            let pos = graph.node(id).pos;
            eprintln!(
                "probe terrain node={id}: {:?}",
                transit
                    .road_surface
                    .terrain_cdt_road_loops_for_world_bounds(
                        &graph,
                        pos.x - 24.0,
                        pos.z - 24.0,
                        pos.x + 24.0,
                        pos.z + 24.0
                    )
                    .map(|_| ())
            );
        }
        if std::env::var("METRUM_JUNCTION_MESH").as_deref() == Ok("1") {
            let lanes = crate::simulation::network::lanes::LaneSystem::new();
            let mesh = crate::simulation::network::render::road::RoadRenderer
                .generate_mesh_data_with_surface(&graph, &lanes, &terrain, &transit.road_surface)
                .seal();
            let points = |v: &[Vector3]| v.iter().map(|p| [p.x, p.y, p.z]).collect::<Vec<_>>();
            fs::write(output.join("mesh.json"), serde_json::to_vec(&serde_json::json!({"road":points(&mesh.road_vertices),"curb":points(&mesh.curb_vertices),"sidewalk":points(&mesh.sidewalk_vertices),"steps":points(&mesh.raised_step_vertices),"concrete":points(&mesh.concrete_vertices),"earthwork":points(&mesh.earthwork_vertices),"grass":mesh.median_grass.as_ref().map(|m| points(&m.vertices))})).unwrap()).unwrap();
        }
        if std::env::var("METRUM_JUNCTION_TERRAIN_PATCHES").is_err() {
            return;
        }
    }
    if probe.is_err() {
        transit
            .road_surface
            .compile_partial_for_geometry_audit(&graph, &terrain);
    }
    if let Ok(selection) = std::env::var("METRUM_JUNCTION_TERRAIN_PATCHES") {
        if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
            crate::debug::set_mask(crate::debug::Channel::Road.bit());
        }
        let surface = &transit.road_surface;
        let margins =
            surface.terrain_render_patch_grading_margins_for_visible_roads(&graph, &terrain, 2.0);
        let keys = if selection == "all" {
            margins.keys().copied().collect::<Vec<_>>()
        } else {
            selection
                .split(';')
                .map(|pair| {
                    let (x, z) = pair.split_once(',').unwrap();
                    (x.parse().unwrap(), z.parse().unwrap())
                })
                .collect()
        };
        let mut failures = Vec::new();
        let mut window_count = 0;
        for (x, z) in keys {
            let patch = terrain.visual_patch_snapshot(x, z).unwrap();
            let margin = crate::simulation::terrain::terrain_cdt_road_query_margin_m(
                &terrain,
                2.0,
                margins.get(&(x, z)).copied().unwrap_or_default(),
            );
            let query = surface.terrain_cdt_road_loops_for_world_bounds(
                &graph,
                patch.world_origin_x - margin,
                patch.world_origin_z - margin,
                patch.world_origin_x + patch.world_size_x + margin,
                patch.world_origin_z + patch.world_size_z + margin,
            );
            let (loops, sources) = match query {
                Ok(query) => query,
                Err(error) => {
                    eprintln!("terrain patch {x},{z}: clip export {error:?}");
                    failures.push(format!("{x},{z}: clip export {error:?}"));
                    continue;
                }
            };
            let input =
                crate::nodes::simulation_node::SimulationNode::planned_road_terrain_patch_input(
                    &terrain,
                    &patch,
                    &loops,
                    sources,
                    true,
                    margin,
                    2.0,
                    None,
                    crate::simulation::network::surface::RoadSurfaceView::new(&graph, surface),
                );
            window_count += input.windows.len();
            let entries =
                crate::nodes::sim::core::SimCore::build_refined_terrain_patch_cache_entries(vec![
                    input,
                ]);
            for entry in entries {
                if let Some(error) =
                    crate::nodes::simulation_node::SimulationNode::cached_refined_cdt_failure_label(
                        &entry,
                    )
                {
                    failures.push(format!("{x},{z}: {error}"));
                    fs::write(
                        output.join(format!("terrain-{x}-{z}.txt")),
                        format!(
                            "key={:?} error={error}\nwindows={:#?}",
                            entry.key,
                            entry
                                .windows
                                .iter()
                                .map(|window| (
                                    &window.key,
                                    window
                                        .mesh_result
                                        .as_ref()
                                        .map(|mesh| (&mesh.stats, &mesh.road_seam_face_samples))
                                ))
                                .collect::<Vec<_>>()
                        ),
                    )
                    .unwrap();
                    eprintln!("terrain patch {x},{z}: {error}");
                }
            }
            eprintln!(
                "terrain patch {x},{z}: windows={window_count} failures={}",
                failures.len()
            );
        }
        fs::write(
            output.join("terrain-results.json"),
            serde_json::to_vec_pretty(
                &serde_json::json!({"windows":window_count,"failures":failures}),
            )
            .unwrap(),
        )
        .unwrap();
        assert!(failures.is_empty(), "{failures:?}");
        return;
    }
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
    selected.extend(missing_spans.iter().copied());
    for &(node, _) in &walls {
        selected.extend(graph.node_adjacency(node));
    }
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
    let mut steep_visible_span_profiles = Vec::new();
    for (&id, piece) in surface.compiled_visual_span_pieces() {
        let max_grade = piece
            .span_owned_regions
            .iter()
            .filter(|region| {
                region.owner().kind
                    == crate::simulation::network::surface::RoadSurfaceBandKind::Carriageway
            })
            .filter_map(|region| {
                let a = &piece.sections[region.start_section_index()];
                let b = &piece.sections[region.end_section_index()];
                let run = (a.center_xz - b.center_xz).length();
                (run > f64::EPSILON)
                    .then_some(f64::from((a.center_height_m - b.center_height_m).abs()) / run)
            })
            .fold(0.0_f64, f64::max);
        if max_grade > 0.5 {
            steep_visible_span_profiles.push((id, max_grade));
        }
    }
    steep_visible_span_profiles.sort_by_key(|item| item.0);
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
    let material_mesh = if std::env::var("METRUM_JUNCTION_MESH").as_deref() == Ok("1") {
        let lanes = crate::simulation::network::lanes::LaneSystem::new();
        let mesh = crate::simulation::network::render::road::RoadRenderer
            .generate_mesh_data_with_surface(&graph, &lanes, &terrain, surface)
            .seal();
        let valid = mesh.render_payload_validated();
        eprintln!(
            "compiled_material_mesh validated={valid} vertices={}",
            mesh.vertex_count()
        );
        assert!(
            valid,
            "full imported material mesh must compile without rejected unions or invalid buffers"
        );
        Some(serde_json::json!({"validated":valid,"vertices":mesh.vertex_count()}))
    } else {
        None
    };
    let report = serde_json::json!({"save":path,"published":surface.published_generation_matches_source(),
        "material_mesh":material_mesh,"failure":surface.last_compile_failure_label(),"failed_nodes":surface.failed_node_ids(),
        "missing_spans":missing_spans,"oversized_kerb_steps":walls,"terrain_export_failures":terrain_failures,
        "steep_source_profiles":steep_source_profiles,"steep_visible_span_profiles":steep_visible_span_profiles,"section_outliers":outliers,"steep_node_planes":nodes,
        "junction_grade_conflicts":surface.junction_grade_conflicts_for_geometry_audit(&graph),
        "mouth_height_discontinuities":surface.mouth_height_discontinuities_for_geometry_audit(&graph),
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
    assert!(
        loaded.is_ok(),
        "the full save runtime must load: {loaded:?}"
    );
    assert_eq!(
        report["published"], true,
        "the sweep must inspect a fully published map"
    );
    for signature in [
        "missing_spans",
        "failed_nodes",
        "oversized_kerb_steps",
        "terrain_export_failures",
        "junction_grade_conflicts",
        "mouth_height_discontinuities",
    ] {
        assert!(
            report[signature].as_array().unwrap().is_empty(),
            "remaining {signature}: {}",
            report[signature]
        );
    }
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

#[test]
fn imported_tight_terminal_curve_publishes_a_complete_span() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/tight_terminal_curve.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let original = graph.edges().to_vec();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &[0, 1, 2, 3], &[2], false);
    assert!(surface.compiled_visual_span_pieces().contains_key(&1));
    assert!(
        surface.compiled_visual_node_pieces().contains_key(&2),
        "the adjacent junction retains its declared kerb contact"
    );
    for (edge, original) in graph.edges().iter().zip(original) {
        assert_eq!(edge.physical_geometry, original.physical_geometry);
        assert_eq!(edge.geometry, original.geometry);
    }
}

#[test]
fn imported_short_approaches_keep_ordinary_kerb_steps() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/graded_continuation.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_partial_for_geometry_audit(&graph, &terrain);
    let piece = &surface.compiled_visual_node_pieces()[&0];
    for face in &piece.raised_step_face_polygons {
        for (index, a) in face.points_world.iter().enumerate() {
            for b in &face.points_world[index + 1..] {
                if (a.x - b.x).hypot(a.z - b.z) < 0.001 {
                    assert!(
                        (a.y - b.y).abs() <= 0.121,
                        "ordinary kerb: {:?}",
                        face.points_world
                    );
                }
            }
        }
    }
}

#[test]
fn bioteknia_grade_conformity_survives_dirty_compilation_and_undo() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/bioteknia_overlap.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_dirty(&graph, &terrain);
    assert!(
        surface.published_generation_matches_source(),
        "{:?}",
        surface.last_compile_failure_label()
    );
    let spans = surface.compiled_visual_span_pieces().clone();
    let nodes = surface.compiled_visual_node_pieces().clone();
    let edge_ids = (0..graph.edge_count()).collect();
    let node_ids = (0..graph.node_count() as u32).collect();
    let undo = surface.capture_topology_undo(&edge_ids, &node_ids).unwrap();
    surface.mark_edge_dirty(&graph, 3);
    surface.compile_dirty(&graph, &terrain);
    assert_eq!(surface.compiled_visual_span_pieces(), &spans);
    assert_eq!(surface.compiled_visual_node_pieces(), &nodes);
    assert!(surface.restore_topology_undo(undo, &edge_ids, &node_ids));
    surface.mark_edge_dirty(&graph, 3);
    surface.compile_dirty(&graph, &terrain);
    assert_eq!(surface.compiled_visual_span_pieces(), &spans);
    assert_eq!(surface.compiled_visual_node_pieces(), &nodes);
}

#[test]
fn bioteknia_complete_mouth_profile_matches_junction_grade() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/bioteknia_overlap.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_owners_for_geometry_audit(
        &graph,
        &terrain,
        &[0, 1, 2, 3, 4, 5, 6, 7, 8],
        &[0, 2, 4, 5],
        false,
    );
    assert!(surface.compiled_visual_node_pieces().contains_key(&4));
    assert_eq!(
        surface.mouth_height_discontinuities_for_geometry_audit(&graph),
        vec![],
        "the full transverse mouth grade must match, including interior material boundaries"
    );
}

#[test]
fn curved_terminal_cap_keeps_the_approach_grade_at_its_mouth() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/terminal_cap_grade.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &[1], &[2], false);
    assert!(surface.compiled_visual_node_pieces().contains_key(&2));
    assert_eq!(
        surface.mouth_height_discontinuities_for_geometry_audit(&graph),
        vec![],
        "a generated terminal cap inherits the curved approach's grade, not a flat node elevation"
    );
}

#[test]
fn terrain_window_keeps_overlapping_junction_contributors_complete() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_query_cohort.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_owners_for_geometry_audit(
        &graph,
        &terrain,
        &(0..8).collect::<Vec<_>>(),
        &[1, 3, 4, 5, 7],
        false,
    );
    for node in [1, 3, 4, 5, 7] {
        assert!(
            surface.compiled_visual_node_pieces().contains_key(&node),
            "required junction {node} must be compiled"
        );
    }
    assert_eq!(
        surface.junction_grade_conflicts_for_geometry_audit(&graph),
        Vec::new(),
        "connected overlapping asphalt must have one grade"
    );
    let pivot = graph.node(7).pos;
    let export = surface.terrain_cdt_road_loops_for_world_bounds(
        &graph,
        pivot.x - 24.0,
        pivot.z - 24.0,
        pivot.x + 24.0,
        pivot.z + 24.0,
    );
    assert!(
        export.is_ok(),
        "partial-window selection must not expose a buried boundary outside the window: {export:?}"
    );
    let lanes = crate::simulation::network::lanes::LaneSystem::new();
    let mesh = crate::simulation::network::render::road::RoadRenderer
        .generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface)
        .seal();
    assert!(
        mesh.render_payload_validated(),
        "material union must compile into a publishable mesh"
    );
    let overlaps = crate::simulation::network::surface::RoadSurfaceSystem::material_mesh_overlaps_for_geometry_audit(&mesh);
    if let Some(&(index, road, _, _)) = overlaps.first() {
        let nonroad = mesh
            .curb_vertices
            .as_chunks::<3>()
            .0
            .iter()
            .chain(mesh.sidewalk_vertices.as_chunks::<3>().0.iter())
            .nth(index)
            .unwrap();
        panic!(
            "foreign sidewalks/kerbs cannot cross connected asphalt: {overlaps:?} nonroad={nonroad:?} road={:?}",
            mesh.road_vertices.as_chunks::<3>().0[road]
        );
    }
    let chunks = surface
        .surface_chunk_cache()
        .keys()
        .chain(surface.earthwork_chunk_cache().keys())
        .copied()
        .collect();
    let mut lanes = lanes;
    let chunked = crate::simulation::network::render::road::RoadRenderer
        .generate_mesh_chunks_with_surface(&graph, &mut lanes, &terrain, &surface, &chunks);
    let mut assembled = crate::simulation::network::render::NetworkMeshData::new();
    let (ox, oz) = surface.chunk_origin_m();
    for ((x, z), mesh) in chunked {
        let offset = Vector3::new(
            ox + x as f32 * surface.chunk_span_m(),
            0.0,
            oz + z as f32 * surface.chunk_span_m(),
        );
        assembled
            .road_vertices
            .extend(mesh.road_vertices.iter().map(|v| *v + offset));
        assembled
            .curb_vertices
            .extend(mesh.curb_vertices.iter().map(|v| *v + offset));
        assembled
            .sidewalk_vertices
            .extend(mesh.sidewalk_vertices.iter().map(|v| *v + offset));
    }
    let overlaps = crate::simulation::network::surface::RoadSurfaceSystem::material_mesh_overlaps_for_geometry_audit(&assembled);
    assert!(
        overlaps.is_empty(),
        "chunk publication must retain the material partition: {overlaps:?}"
    );
}

#[test]
fn shared_grade_refinement_preserves_source_contacts_and_facet_quality() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_grade_breaks.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(6000.0, 6000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_owners_for_geometry_audit(
        &graph,
        &terrain,
        &[
            2, 4, 5, 6, 7, 8, 9, 10, 11, 14, 15, 17, 21, 22, 27, 28, 31, 32, 33, 36, 37, 38, 44,
            47, 48, 51, 52,
        ],
        &[
            3, 8, 9, 11, 12, 13, 14, 16, 19, 58, 27, 54, 31, 42, 43, 33, 37,
        ],
        false,
    );
    for node in [
        3, 8, 9, 11, 12, 13, 14, 16, 19, 58, 27, 54, 31, 42, 43, 33, 37,
    ] {
        assert!(surface.compiled_visual_node_pieces().contains_key(&node));
    }
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
}

#[test]
fn shared_grade_cohort_survives_dirty_compilation_undo_and_preview() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_query_cohort.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(4000.0, 4000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 89.7 / crate::config::HEIGHT_SCALE);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    assert!(
        surface.compile_dirty(&graph, &terrain),
        "{:?}",
        surface.last_compile_failure_label()
    );
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
    let spans = surface.compiled_visual_span_pieces().clone();
    let nodes = surface.compiled_visual_node_pieces().clone();
    let edge_ids = std::collections::HashSet::from([3]);
    let node_ids = std::collections::HashSet::from([4, 5]);
    let undo = surface.capture_topology_undo(&edge_ids, &node_ids).unwrap();
    surface.mark_edge_dirty(&graph, 3);
    assert!(
        surface.compile_dirty(&graph, &terrain),
        "{:?}",
        surface.last_compile_failure_label()
    );
    assert_eq!(surface.compiled_visual_span_pieces(), &spans);
    assert_eq!(surface.compiled_visual_node_pieces(), &nodes);
    assert!(surface.restore_topology_undo(undo, &edge_ids, &node_ids));
    assert!(
        surface.compile_dirty(&graph, &terrain),
        "{:?}",
        surface.last_compile_failure_label()
    );
    assert_eq!(surface.compiled_visual_span_pieces(), &spans);
    assert_eq!(surface.compiled_visual_node_pieces(), &nodes);
    let start = graph.node(0).pos;
    let direction = (start - graph.node(1).pos).normalized();
    let end = Vector3::new(
        start.x + direction.x * 80.0,
        89.7,
        start.z + direction.z * 80.0,
    );
    let preview = surface.compile_preview_surface_mesh_only_with_existing_surface(
        &[start, end],
        1,
        1,
        &terrain,
        &graph,
        &surface,
    );
    assert!(
        preview.is_valid,
        "a terminal extension must retain the whole remapped grade cohort: {:?}",
        preview.validation
    );
    assert_eq!(surface.compiled_visual_span_pieces(), &spans);
    assert_eq!(surface.compiled_visual_node_pieces(), &nodes);
}

#[test]
fn refined_junction_steps_keep_declared_contacts_on_retained_top_edges() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_refined_steps.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(6000.0, 6000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_partial_for_geometry_audit(&graph, &terrain);
    // Exact source neighbourhoods around imported nodes 1150 and 1628. The
    // source-ID mapping is retained in the SQL header; source geometry is unchanged.
    for node in [25, 27, 28, 31, 32, 43, 44] {
        assert!(
            surface.compiled_visual_node_pieces().contains_key(&node),
            "junction {node} must retain its source-declared kerb step"
        );
    }
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
}

#[test]
fn refined_footprint_uses_one_canonical_grade_at_source_and_generated_vertices() {
    if std::env::var("METRUM_DEBUG").as_deref() == Ok("1") {
        crate::debug::set_mask(crate::debug::Channel::Road.bit());
    }
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_refined_boundary.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(6000.0, 6000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    surface.compile_partial_for_geometry_audit(&graph, &terrain);
    // The refined boundary of imported node 1629 coincides with source contacts.
    for node in [6, 8] {
        assert!(
            surface.compiled_visual_node_pieces().contains_key(&node),
            "source and generated boundary contacts must have one canonical height"
        );
    }
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
}

#[test]
fn restored_junction_footprints_keep_connected_grades_coherent() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_grade_breaks.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(6000.0, 6000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    let nodes = [19, 20, 21, 58];
    let edges = nodes
        .iter()
        .flat_map(|&n| graph.node_adjacency(n))
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &edges, &nodes, false);
    for node in nodes {
        assert!(surface.compiled_visual_node_pieces().contains_key(&node));
    }
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
}

#[test]
fn retained_shared_grade_plans_require_valid_source_ownership() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_grade_source_plans.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(6000.0, 6000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    let nodes = [2, 12, 20, 16, 18];
    let edges = nodes
        .iter()
        .flat_map(|&n| graph.node_adjacency(n))
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &edges, &nodes, false);
    for node in nodes {
        assert!(surface.compiled_visual_node_pieces().contains_key(&node));
    }
    assert!(
        surface
            .junction_grade_conflicts_for_geometry_audit(&graph)
            .is_empty()
    );
}

#[test]
fn shared_grade_terrain_boundaries_keep_a_certified_source_owner() {
    let conn = rusqlite::Connection::open_in_memory().unwrap();
    conn.execute_batch(SCHEMA).unwrap();
    conn.execute_batch(include_str!("data/junction_terrain_owners.sql"))
        .unwrap();
    let graph = network::load_graph(&conn).unwrap();
    let config = WorldConfig::new(8000.0, 8000.0, 40.0, 10.0)
        .with_terrain_resolution(10.0)
        .with_chunking(510.0, 100.0);
    let terrain = TerrainSystem::from_world_config(&config);
    let mut surface = crate::simulation::network::surface::RoadSurfaceSystem::new(510.0);
    // Frontier incidence is intentionally omitted from this exact source excerpt.
    let nodes = [
        1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 14, 15, 16, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        34, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 52,
    ];
    let edges = (0..graph.edge_count()).collect::<Vec<_>>();
    surface.compile_owners_for_geometry_audit(&graph, &terrain, &edges, &nodes, false);
    for node in [1, 2, 7, 8, 23, 24, 25, 36, 43, 45, 50] {
        let p = graph.node(node).pos;
        let result = surface.terrain_cdt_road_loops_for_world_bounds(
            &graph,
            p.x - 24.0,
            p.z - 24.0,
            p.x + 24.0,
            p.z + 24.0,
        );
        assert!(result.is_ok(), "source node {node}: {result:?}");
    }
}
