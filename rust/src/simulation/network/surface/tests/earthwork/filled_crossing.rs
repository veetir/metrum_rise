// SPDX-License-Identifier: GPL-2.0-only

//! Filled crossing ownership, clearance, legacy isolation and local rebuild regressions.

use super::*;
use crate::simulation::network::build_surface_edge;

fn fixture(foot: bool, skew: bool) -> (RegionGraph, TerrainSystem, RoadSurfaceSystem) {
    let mut graph = RegionGraph::new();
    for (points, lanes, level, class) in [
        (
            [
                Vector3::new(-80.0, 0.0, if skew { -30.0 } else { 0.0 }),
                Vector3::new(80.0, 0.0, if skew { 30.0 } else { 0.0 }),
            ],
            if foot { 0 } else { 1 },
            0,
            EdgeClass::Standard,
        ),
        (
            [Vector3::new(0.0, 6.0, -80.0), Vector3::new(0.0, 6.0, 80.0)],
            2,
            1,
            EdgeClass::Bridge,
        ),
    ] {
        let start = graph.add_node(points[0], NodeType::Junction);
        let end = graph.add_node(points[1], NodeType::Junction);
        let mut edge = build_surface_edge(start, end, points.to_vec(), lanes, lanes, class);
        edge.crossing_level = level;
        graph.add_edge(edge);
    }
    let terrain = flat_terrain(257, 257);
    let mut surface = RoadSurfaceSystem::new(16.0);
    surface.compile_dirty(&graph, &terrain);
    assert!(surface.published_generation_matches_source());
    (graph, terrain, surface)
}

#[test]
fn filled_openings_leave_full_lower_roadbed_clear_and_keep_terrain_below() {
    for foot in [false, true] {
        for skew in [false, true] {
            let (graph, mut terrain, mut surface) = fixture(foot, skew);
            let crossing = surface.compiled_visual_span_pieces()[&1]
                .filled_crossing
                .as_ref()
                .unwrap();
            assert_eq!(crossing.lower_edges, [0]);
            assert!(!crossing.fill.is_empty());
            assert!(!crossing.concrete.is_empty());
            let lower = &surface.compiled_sections()[&0];
            // No grass face overlaps the lower paved corridor, including its sidewalks.
            for quad in &crossing.fill {
                for pair in lower.windows(2) {
                    let p = |s: &RoadSurfaceSection, right: bool| {
                        let band = if right {
                            s.bands.last().unwrap()
                        } else {
                            &s.bands[0]
                        };
                        RoadSurfaceSystem::section_boundary_world_point_static(
                            s,
                            if right {
                                band.lateral_end_m
                            } else {
                                band.lateral_start_m
                            },
                            s.center_height_m,
                        )
                    };
                    let corridor = [
                        p(&pair[0], false),
                        p(&pair[1], false),
                        p(&pair[1], true),
                        p(&pair[0], true),
                    ];
                    for a in [[0, 1, 2], [0, 2, 3]] {
                        for b in [[0, 1, 2], [0, 2, 3]] {
                            assert!(
                                super::super::super::grade_separation::triangle_overlap(
                                    a.map(|i| quad[i]),
                                    b.map(|i| corridor[i])
                                )
                                .is_none(),
                                "fill must not close the lower corridor"
                            );
                        }
                    }
                }
            }
            surface.rebuild_all_earthworks(&graph, &mut terrain);
            assert!(
                terrain.sample_visual_height_world(0.0, 0.0) * crate::config::HEIGHT_SCALE < 0.1
            );
            if !skew {
                assert!(
                    terrain.sample_visual_height_world(0.0, 40.0) * crate::config::HEIGHT_SCALE
                        > 5.0,
                    "solid fill supports the interior pavement away from the opening"
                );
                assert!(
                    surface
                        .sample_visible_surface_height(&graph, &terrain, 15.0, 40.0)
                        .unwrap()
                        > 1.0,
                    "the filled slope participates in existing ground picking"
                );
                assert!(
                    terrain.sample_visual_height_world(15.0, 40.0) * crate::config::HEIGHT_SCALE
                        > 1.0,
                    "the shared stamper supports the filled slope outside the opening"
                );
                assert_eq!(
                    terrain.sample_height_world(15.0, 40.0),
                    0.0,
                    "source terrain stays immutable"
                );
            }
        }
    }
}

#[test]
fn lower_width_delete_and_restore_rebuild_upper_owner() {
    let (mut graph, terrain, mut surface) = fixture(false, false);
    let old = surface.compiled_visual_span_pieces()[&1]
        .filled_crossing
        .clone();
    graph.edge_mut(0).width += 6.0;
    surface.mark_edge_dirty(&graph, 0);
    surface.compile_dirty(&graph, &terrain);
    assert_ne!(
        old,
        surface.compiled_visual_span_pieces()[&1].filled_crossing
    );
    let edges = HashSet::from([0]);
    let nodes = HashSet::new();
    let undo = surface.capture_topology_undo(&edges, &nodes).unwrap();
    graph.edge_mut(0).deleted = true;
    surface.mark_edge_dirty(&graph, 0);
    surface.compile_dirty(&graph, &terrain);
    assert!(
        surface.compiled_visual_span_pieces()[&1]
            .filled_crossing
            .as_ref()
            .is_some_and(|crossing| crossing.lower_edges.is_empty() && !crossing.fill.is_empty())
    );
    graph.edge_mut(0).deleted = false;
    assert!(surface.restore_topology_undo(undo, &edges, &nodes));
    surface.compile_dirty(&graph, &terrain);
    assert!(
        surface.compiled_visual_span_pieces()[&1]
            .filled_crossing
            .is_some()
    );
}

#[test]
fn same_layer_valley_bridge_keeps_prior_span_exactly() {
    let (mut graph, terrain, mut surface) = fixture(false, false);
    graph.edge_mut(1).crossing_level = 0;
    surface.mark_edge_dirty(&graph, 1);
    surface.compile_dirty(&graph, &terrain);
    assert!(
        surface.compiled_visual_span_pieces()[&1]
            .filled_crossing
            .is_none()
    );
    let old_sections = surface.compiled_sections()[&1].clone();
    graph.edge_mut(0).deleted = true;
    surface.mark_edge_dirty(&graph, 0);
    surface.compile_dirty(&graph, &terrain);
    assert_eq!(old_sections, surface.compiled_sections()[&1]);
}

#[test]
fn automatic_over_under_profiles_preserve_knots_and_pass_exact_clearance() {
    for over in [true, false] {
        for foot in [false, true] {
            let (mut graph, terrain, surface) = fixture(over && foot, false);
            graph.edge_mut(1).deleted = true;
            let raw = [
                Vector3::new(0.0, 0.0, -140.0),
                Vector3::new(0.0, 0.0, 140.0),
            ];
            let lanes = if !over && foot { (0, 0) } else { (1, 1) };
            let profile = surface
                .prepare_crossing_profile(&graph, &raw, lanes, None, over, None)
                .unwrap();
            assert_eq!(profile.points.first(), raw.first());
            assert_eq!(profile.points.last(), raw.last());
            assert_eq!(profile.level, if over { 1 } else { -1 });
            let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
                &profile.points,
                &terrain,
                &graph,
                &surface,
                true,
                profile.level,
            );
            assert_eq!(
                prepared.points, profile.points,
                "do not erase structural knots"
            );
            assert_eq!(prepared.class, EdgeClass::Standard);
            let validation = surface.validate_prepared_road_surface(
                &prepared.points,
                prepared.class,
                lanes.0,
                lanes.1,
                &terrain,
            );
            let validation = surface.validate_prepared_crossing_clearance(
                &prepared, lanes.0, lanes.1, &graph, validation,
            );
            assert!(validation.is_valid, "{validation:?}");
        }
    }
}

#[test]
fn fill_and_abutment_winding_faces_the_visible_volume() {
    let (_, _, surface) = fixture(false, false);
    let piece = &surface.compiled_visual_span_pieces()[&1];
    let crossing = piece.filled_crossing.as_ref().unwrap();
    for face in &crossing.fill {
        let normal = (face[2] - face[0]).cross(face[1] - face[0]);
        assert!(normal.y >= -1e-6, "grass faces must face up/outward");
    }
    for face in &crossing.concrete {
        if face.iter().all(|p| (p.z - face[0].z).abs() < 1e-6)
            && face.iter().any(|p| p.y < 0.1)
            && face.iter().any(|p| p.y > 5.0)
        {
            let normal = (face[2] - face[0]).cross(face[1] - face[0]);
            assert!(
                normal.z * -face[0].z >= 0.0,
                "abutments and wings face the opening"
            );
        }
    }
}

#[test]
fn filled_grass_stops_at_the_span_owned_mouths() {
    let (mut graph, terrain, mut surface) = fixture(false, false);
    let joint = graph.edge(1).start_node;
    let end = graph.add_node(Vector3::new(-80.0, 6.0, -80.0), NodeType::Junction);
    let mut approach = build_surface_edge(
        joint,
        end,
        vec![graph.node(joint).pos, graph.node(end).pos],
        1,
        1,
        EdgeClass::Bridge,
    );
    approach.crossing_level = 1;
    let approach = graph.add_edge(approach);
    surface.mark_edge_dirty(&graph, approach);
    surface.mark_edge_dirty(&graph, 1);
    surface.compile_dirty(&graph, &terrain);
    let crossing = surface.compiled_visual_span_pieces()[&1]
        .filled_crossing
        .as_ref()
        .unwrap();
    let sections = &surface.compiled_sections()[&1];
    let (start, _) = surface
        .visible_corridor_index_range_for_edge(&graph, 1, sections)
        .unwrap();
    let mouth_z = sections[start].center_xz.y;
    assert!(mouth_z > -79.0, "the bend owns its pavement mouth");
    assert!(
        crossing
            .fill
            .iter()
            .flatten()
            .all(|p| p.z >= mouth_z - 0.001)
    );
    assert!(surface.node_piece_uses_earthworks(&graph, graph.edge(1).start_node, &terrain));
}

#[test]
fn source_level_crossing_keeps_ordinary_junctions_out_of_the_clearance_envelope() {
    let (mut graph, _, surface) = fixture(false, false);
    graph.edge_mut(1).deleted = true;
    graph.edge_mut(0).crossing_level = 1;
    let a = graph.add_node(Vector3::new(-80.0, 0.0, -60.0), NodeType::Junction);
    let b = graph.add_node(Vector3::new(80.0, 0.0, -60.0), NodeType::Junction);
    graph.add_edge(build_surface_edge(
        a,
        b,
        vec![graph.node(a).pos, graph.node(b).pos],
        1,
        1,
        EdgeClass::Standard,
    ));
    let raw = [
        Vector3::new(0.0, 0.0, -140.0),
        Vector3::new(0.0, 0.0, 140.0),
    ];
    let fixed = surface
        .prepare_crossing_profile(&graph, &raw, (1, 1), None, false, Some(0))
        .unwrap();
    assert_eq!(fixed.level, 0);
    assert_eq!(
        fixed.edges,
        [0],
        "only the explicitly higher road owns an opening"
    );
    assert_eq!(fixed.points.first(), raw.first());
    assert_eq!(fixed.points.last(), raw.last());
    let free = surface
        .prepare_crossing_profile(&graph, &raw, (1, 1), None, false, None)
        .unwrap();
    assert!(
        free.edges.contains(&2),
        "the player's unconstrained Under selection retains its semantics"
    );
    assert_eq!(free.level, -1);
}

#[test]
fn under_an_upper_level_can_create_a_level_zero_open_cut() {
    let (mut graph, terrain, surface) = fixture(false, false);
    graph.edge_mut(1).deleted = true;
    graph.edge_mut(0).crossing_level = 1;
    let raw = [
        Vector3::new(0.0, 0.0, -140.0),
        Vector3::new(0.0, 0.0, 140.0),
    ];
    let profile = surface
        .prepare_crossing_profile(&graph, &raw, (1, 1), None, false, None)
        .unwrap();
    assert_eq!(profile.level, 0);
    let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
        &profile.points,
        &terrain,
        &graph,
        &surface,
        true,
        profile.level,
    );
    assert_eq!(prepared.class, EdgeClass::Standard);
    assert!(prepared.points.iter().any(|p| p.y < -5.0));
}

#[test]
fn a_surface_branch_can_join_an_existing_open_underpass_cut() {
    let (mut graph, terrain, mut surface) = fixture(false, false);
    // The existing ordinary lower owner is below the original DEM, as after Under.
    for point in &mut graph.edge_mut(0).geometry {
        point.y = -4.0;
    }
    graph.edge_mut(0).physical_geometry = graph.edge(0).geometry.clone();
    graph.edge_mut(0).no_building_spawn = true;
    for node in [graph.edge(0).start_node, graph.edge(0).end_node] {
        graph.nodes[node as usize].pos.y = -4.0;
    }
    surface.mark_edge_dirty(&graph, 0);
    surface.compile_dirty(&graph, &terrain);
    let raw = [
        Vector3::new(-25.0, -4.0, 0.0),
        Vector3::new(-25.0, 0.0, 30.0),
        Vector3::new(-25.0, 0.0, 80.0),
    ];
    let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
        &raw, &terrain, &graph, &surface, true, 0,
    );
    assert_eq!(prepared.class, EdgeClass::Standard);
    assert_eq!(prepared.points.first(), raw.first());
    assert_eq!(prepared.points.last(), raw.last());
    // A point with no road support still follows the ordinary tunnel rules.
    let detached = [Vector3::new(-25.0, -4.0, 8.0), raw[1], raw[2]];
    let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
        &detached, &terrain, &graph, &surface, true, 0,
    );
    assert_eq!(prepared.class, EdgeClass::Tunnel);
    let raw = [
        Vector3::new(-25.0, -4.0, 0.0),
        Vector3::new(0.0, 0.0, 40.0),
        Vector3::new(25.0, 0.0, 100.0),
    ];
    let profile = surface
        .prepare_crossing_profile(&graph, &raw, (0, 0), None, false, Some(0))
        .unwrap();
    let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_at_level(
        &profile.points,
        &terrain,
        &graph,
        &surface,
        true,
        0,
    );
    assert_eq!(prepared.class, EdgeClass::Standard);
    assert_eq!(
        prepared.points, profile.points,
        "an open-cut endpoint must not erase Under clearance knots"
    );
}

#[test]
fn crossing_envelope_bounds_steep_raw_approaches_without_moving_joints() {
    let (mut graph, _, surface) = fixture(false, false);
    graph.edge_mut(1).deleted = true;
    for over in [true, false] {
        let sign = if over { 1.0 } else { -1.0 };
        let raw = [
            Vector3::new(0.0, 0.0, -200.0),
            Vector3::new(0.0, sign * 10.0, -60.0),
            Vector3::new(0.0, 0.0, -40.0),
            Vector3::new(0.0, 0.0, 200.0),
        ];
        let profile = surface
            .prepare_crossing_profile(&graph, &raw, (1, 1), None, over, None)
            .unwrap();
        assert_eq!(profile.points.first(), raw.first());
        assert_eq!(profile.points.last(), raw.last());
        for pair in profile.points.windows(2) {
            let run = Vector2::new(pair[1].x - pair[0].x, pair[1].z - pair[0].z).length();
            assert!((pair[1].y - pair[0].y).abs() <= run * 0.12001);
        }
    }
}
