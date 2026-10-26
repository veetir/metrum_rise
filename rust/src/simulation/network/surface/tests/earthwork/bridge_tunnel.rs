// SPDX-License-Identifier: GPL-2.0-only

//! Bridge and tunnel earthwork policy tests.

use super::*;

fn assert_structural_terminals_are_span_owned(surface: &RoadSurfaceSystem, start: u32, end: u32) {
    assert!(
        surface.published_generation_matches_source(),
        "span-owned structural terminals must be a complete published generation"
    );
    for node_id in [start, end] {
        assert!(
            !surface.compiled_visual_node_pieces().contains_key(&node_id),
            "bridge/tunnel terminal {node_id} must be omitted by policy before node compilation"
        );
    }
}

#[test]
fn structural_terminal_policy_removes_and_restores_caps_incrementally() {
    let terrain = flat_terrain(97, 33);
    let mut graph = RegionGraph::new();
    let start = graph.add_node(Vector3::new(-24.0, 0.0, 0.0), NodeType::Junction);
    let end = graph.add_node(Vector3::new(24.0, 0.0, 0.0), NodeType::Junction);
    let edge_idx = graph.add_edge(test_edge(
        start,
        end,
        vec![Vector3::new(-24.0, 0.0, 0.0), Vector3::new(24.0, 0.0, 0.0)],
        10.0,
        EdgeClass::Standard,
        TransitType::Road,
        TransitFlags::CAR | TransitFlags::FOOT,
    ));

    let mut surface = RoadSurfaceSystem::new(16.0);
    surface.compile_dirty(&graph, &terrain);
    assert!(surface.compiled_visual_node_pieces().contains_key(&start));
    assert!(surface.compiled_visual_node_pieces().contains_key(&end));

    graph.edge_mut(edge_idx).class = EdgeClass::Bridge;
    surface.mark_edge_dirty(&graph, edge_idx);
    surface.mark_node_dirty(&graph, start);
    surface.mark_node_dirty(&graph, end);
    surface.compile_dirty(&graph, &terrain);
    assert_structural_terminals_are_span_owned(&surface, start, end);

    graph.edge_mut(edge_idx).class = EdgeClass::Standard;
    surface.mark_edge_dirty(&graph, edge_idx);
    surface.mark_node_dirty(&graph, start);
    surface.mark_node_dirty(&graph, end);
    surface.compile_dirty(&graph, &terrain);
    assert!(surface.published_generation_matches_source());
    assert!(surface.compiled_visual_node_pieces().contains_key(&start));
    assert!(surface.compiled_visual_node_pieces().contains_key(&end));
}

#[test]
fn bridge_earthworks_do_not_stamp_terrain() {
    let mut terrain = flat_terrain(97, 33);
    let mut graph = RegionGraph::new();
    let start = graph.add_node(Vector3::new(-24.0, 6.0, 0.0), NodeType::Junction);
    let end = graph.add_node(Vector3::new(24.0, 6.0, 0.0), NodeType::Junction);
    let edge_idx = graph.add_edge(test_edge(
        start,
        end,
        vec![
            Vector3::new(-24.0, 6.0, 0.0),
            Vector3::new(0.0, 6.0, 0.0),
            Vector3::new(24.0, 6.0, 0.0),
        ],
        10.0,
        EdgeClass::Bridge,
        TransitType::Road,
        TransitFlags::CAR | TransitFlags::FOOT,
    ));

    let mut surface = RoadSurfaceSystem::new(16.0);
    surface.rebuild_all_earthworks(&graph, &mut terrain);
    let span_piece = surface
        .compiled_visual_span_pieces()
        .get(&edge_idx)
        .expect("bridge span should compile");
    assert_structural_terminals_are_span_owned(&surface, start, end);
    assert!(span_piece.span_earthwork_support_regions.is_empty());
    assert_eq!(span_piece.earthwork_surface_polygons().len(), 0);
    assert!(span_piece.render_earthwork_faces.is_empty());

    let span_center = terrain.sample_visual_height_world(0.0, 0.0) * crate::config::HEIGHT_SCALE;
    let start_terminal =
        terrain.sample_visual_height_world(-20.0, 0.0) * crate::config::HEIGHT_SCALE;
    let end_terminal = terrain.sample_visual_height_world(20.0, 0.0) * crate::config::HEIGHT_SCALE;
    assert!(span_center.abs() <= 0.01);
    assert!(start_terminal.abs() <= 0.01);
    assert!(end_terminal.abs() <= 0.01);
}

#[test]
fn bridge_ramp_earthworks_do_not_raise_elevated_terminal_terrain() {
    let mut terrain = flat_terrain(129, 33);
    let mut graph = RegionGraph::new();
    let start = graph.add_node(Vector3::new(-48.0, 0.0, 0.0), NodeType::Junction);
    let end = graph.add_node(Vector3::new(48.0, 7.5, 0.0), NodeType::Junction);
    let edge_idx = graph.add_edge(test_edge(
        start,
        end,
        vec![Vector3::new(-48.0, 0.0, 0.0), Vector3::new(48.0, 7.5, 0.0)],
        10.0,
        EdgeClass::Bridge,
        TransitType::Road,
        TransitFlags::CAR | TransitFlags::FOOT,
    ));

    let mut surface = RoadSurfaceSystem::new(16.0);
    surface.rebuild_all_earthworks(&graph, &mut terrain);
    let span_piece = surface
        .compiled_visual_span_pieces()
        .get(&edge_idx)
        .expect("bridge ramp span should compile");
    assert_structural_terminals_are_span_owned(&surface, start, end);

    assert!(
        !span_piece.span_earthwork_support_regions.is_empty(),
        "the grounded end of a bridge ramp must own a terrain-clipped abutment"
    );
    assert!(
        !span_piece.terrain_clip_boundary_loops().is_empty(),
        "the bridge abutment must remove coplanar terrain beneath the deck"
    );
    assert!(span_piece.render_earthwork_faces.is_empty());
    for (x, expected) in [(-48.0, 0.0), (0.0, 0.0), (48.0, 0.0)] {
        let visual_height =
            terrain.sample_visual_height_world(x, 0.0) * crate::config::HEIGHT_SCALE;
        assert!(
            (visual_height - expected).abs() <= 0.01,
            "bridge ramp must not raise visual terrain at x={x}: {visual_height:.3}"
        );
    }
}

#[test]
fn tunnel_earthworks_only_stamp_portals() {
    let mut terrain = flat_terrain(97, 33);
    let mut graph = RegionGraph::new();
    let start = graph.add_node(Vector3::new(-24.0, 0.0, 0.0), NodeType::Junction);
    let end = graph.add_node(Vector3::new(24.0, 0.0, 0.0), NodeType::Junction);
    let edge_idx = graph.add_edge(test_edge(
        start,
        end,
        vec![
            Vector3::new(-24.0, 0.0, 0.0),
            Vector3::new(-10.0, -6.0, 0.0),
            Vector3::new(10.0, -6.0, 0.0),
            Vector3::new(24.0, 0.0, 0.0),
        ],
        10.0,
        EdgeClass::Tunnel,
        TransitType::Road,
        TransitFlags::CAR | TransitFlags::FOOT,
    ));

    let mut surface = RoadSurfaceSystem::new(16.0);
    surface.rebuild_all_earthworks(&graph, &mut terrain);
    let span_piece = surface
        .compiled_visual_span_pieces()
        .get(&edge_idx)
        .expect("tunnel span should compile");
    assert_structural_terminals_are_span_owned(&surface, start, end);
    assert!(!span_piece.span_earthwork_support_regions.is_empty());
    assert!(
        span_piece.render_earthwork_faces.is_empty(),
        "the concrete shell and stitched terrain replace generic skirts"
    );
    assert!(
        !span_piece
            .short_tunnel
            .as_ref()
            .unwrap()
            .concrete
            .is_empty()
    );
    assert!(
        span_piece
            .span_earthwork_support_regions
            .iter()
            .all(|region| {
                let sections = &span_piece.sections;
                !(region.start_s_m(sections) < 24.0 && region.end_s_m(sections) > 24.0)
            }),
        "tunnel support regions must stay at visible portals instead of owning buried midspan terrain"
    );
    assert!(
        span_piece
            .terrain_clip_boundary_loops()
            .iter()
            .flat_map(|boundary| &boundary.source_edges)
            .all(|edge| {
                matches!(
                    edge.source,
                    RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
                        edge_class: EdgeClass::Tunnel,
                        support_policy: RoadSurfaceEarthworkSupportPolicy::TunnelOpenCut
                            | RoadSurfaceEarthworkSupportPolicy::TunnelPortalRoof,
                        ..
                    }
                )
            }),
        "open cuts and roof rails preserve their structural source"
    );

    let center = terrain.sample_visual_height_world(0.0, 0.0) * crate::config::HEIGHT_SCALE;
    let portal = terrain.sample_visual_height_world(-20.0, 0.0) * crate::config::HEIGHT_SCALE;
    assert!(center.abs() <= 0.01);
    assert!(portal <= -0.1);
}

#[test]
fn bridge_abutment_ranges_stop_before_fully_airborne_sections() {
    let terrain = flat_terrain(129, 65);
    let surface = RoadSurfaceSystem::new(16.0);
    let ground = Vector3::new(-48.0, 0.0, 0.0);
    let air = Vector3::new(48.0, 7.5, 0.0);
    for points in [vec![ground, air], vec![air, ground]] {
        let mut graph = RegionGraph::new();
        let start = graph.add_node(points[0], NodeType::Junction);
        let end = graph.add_node(points[1], NodeType::Junction);
        let edge_idx = graph.add_edge(test_edge(
            start,
            end,
            points,
            10.0,
            EdgeClass::Bridge,
            TransitType::Road,
            TransitFlags::CAR | TransitFlags::FOOT,
        ));
        let sections = surface.compile_edge_sections(&graph, edge_idx);
        let ranges = surface.earthwork_section_ranges_for_edge(
            &graph,
            edge_idx,
            graph.edge(edge_idx),
            &sections,
            &terrain,
        );
        assert!(
            !ranges.is_empty(),
            "the grounded abutment still needs a cutout"
        );
        for (first, last) in ranges {
            assert!(
                sections[first..=last]
                    .iter()
                    .all(|section| surface.bridge_section_contacts_terrain(section, &terrain)),
                "a bridge cutout must not remove terrain beneath an airborne section"
            );
        }
    }
}

#[test]
fn short_tunnel_hill_has_open_portals_and_closed_roof_from_section_widths() {
    for lanes in [1, 2] {
        let mut terrain = TerrainSystem::with_chunking(257, 129, 1.0, 64, 0.0);
        for z in 0..129 {
            for x in 0..257 {
                let world_x = x as f32 - 128.0;
                terrain.set_height(
                    x,
                    z,
                    ((64.0 - world_x.abs()).max(0.0) * 0.3) / crate::config::HEIGHT_SCALE,
                );
            }
        }
        let mut graph = RegionGraph::new();
        let points = vec![
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(100.0, 0.0, 0.0),
        ];
        let a = graph.add_node(points[0], NodeType::Junction);
        let b = graph.add_node(points[1], NodeType::Junction);
        let allow_walk = lanes == 2;
        let mut edge = crate::simulation::network::build_surface_edge(
            a,
            b,
            points,
            lanes,
            lanes,
            EdgeClass::Tunnel,
        );
        edge.allowed_types = TransitFlags::CAR | if allow_walk { TransitFlags::FOOT } else { 0 };
        let id = graph.add_edge(edge);
        let mut surface = RoadSurfaceSystem::new(64.0);
        surface.rebuild_all_earthworks(&graph, &mut terrain);
        assert!(surface.published_generation_matches_source());
        let piece = &surface.compiled_visual_span_pieces()[&id];
        let tunnel = piece.short_tunnel.as_ref().unwrap();
        assert!(tunnel.concrete.len() > 20);
        let cutouts = piece.terrain_clip_boundary_loops();
        assert_eq!(cutouts.len(), 2, "only the two open approaches cut terrain");
        assert!(
            cutouts
                .iter()
                .all(|boundary| boundary.points_world.iter().any(|p| p.y > 5.0)),
            "portal terrain handoff belongs to the roof, never the floor"
        );

        assert!(
            piece.render_earthwork_faces.is_empty(),
            "no duplicate skirts across the owned portal shell"
        );
        assert!(
            surface
                .sample_visible_surface_height(&graph, &terrain, 0.0, 0.0)
                .is_none()
        );
        assert!(
            (terrain.sample_visual_height_world(0.0, 0.0) * crate::config::HEIGHT_SCALE - 19.2)
                .abs()
                < 0.01
        );
        assert!(piece.span_earthwork_support_regions.iter().all(|region| {
            let pair =
                &piece.sections[region.start_section_index()..=region.start_section_index() + 1];
            pair.iter().all(|section| section.center_xz.x.abs() > 20.0)
        }));
        let mut paths = crate::simulation::network::lanes::LaneSystem::new();
        paths.rebuild(&mut graph);
        paths.sync_heights_to_visible_surface(&graph, &terrain, &surface);
        assert_eq!(
            paths
                .lanes
                .iter()
                .any(|lane| lane.lane_type == crate::simulation::network::lanes::LaneType::Foot),
            allow_walk
        );
        assert!(
            paths
                .lanes
                .iter()
                .filter(|lane| lane.edge_id == id)
                .flat_map(|lane| &lane.geometry)
                .all(|p| p.y.abs() < 0.25),
            "every permitted mode follows the bore pavement"
        );
        let bounds = graph.paved_bounds(id, 100.0);
        let lateral = tunnel
            .concrete
            .iter()
            .flatten()
            .map(|p| p.z.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            lateral > f64::from(bounds[1]),
            "walls must clear every paved strip"
        );
        assert!(
            lateral < f64::from(bounds[1]) + 4.0,
            "wings follow the queried section width"
        );
    }
}

#[test]
fn tunnel_quick_exact_cover_parity_includes_off_centre_hills_and_refusals() {
    let mut terrain = TerrainSystem::with_chunking(257, 129, 1.0, 64, 0.0);
    for z in 0..129 {
        for x in 0..257 {
            terrain.set_height(
                x,
                z,
                ((64.0 - (x as f32 - 128.0).abs()).max(0.0) * 0.3) / crate::config::HEIGHT_SCALE,
            );
        }
    }
    let graph = RegionGraph::new();
    let surface = RoadSurfaceSystem::new(64.0);
    for height in [0.0, 30.0] {
        let points = [
            Vector3::new(-100.0, height, 0.0),
            Vector3::new(80.0, height, 0.0),
            Vector3::new(100.0, height, 0.0),
        ];
        let prepared = RoadSurfaceSystem::prepare_road_input_for_tool_with_tunnel(
            &points, &terrain, &graph, &surface, false, 0, true,
        );
        let quick =
            surface.validate_prepared_road_candidate_fast(&prepared, 1, 1, &terrain, &graph);
        let exact = surface.validate_prepared_road_surface(
            &prepared.points,
            prepared.class,
            1,
            1,
            &terrain,
        );
        assert_eq!(quick.is_valid, height == 0.0, "{quick:?}");
        assert_eq!(quick.is_valid, exact.is_valid);
        assert_eq!(quick.invalid_reason, exact.invalid_reason);
        assert_eq!(quick.clearance_m, exact.clearance_m);
    }
}

#[test]
fn tunnel_portal_does_not_stamp_floor_samples_under_the_roof_on_coarse_terrain() {
    for hill_shift in [0.0, 2.0] {
        let mut terrain = TerrainSystem::with_chunking(161, 161, 4.0, 128, 0.0);
        let (cx, cz) = terrain.world_to_grid_coords(hill_shift, 0.0);
        terrain.sculpt(cx, cz, 100.0 / terrain.cell_size_m(), 1.4);
        let mut graph = RegionGraph::new();
        let points = vec![
            Vector3::new(-150.0, 0.0, 0.0),
            Vector3::new(150.0, 0.0, 0.0),
        ];
        let a = graph.add_node(points[0], NodeType::Junction);
        let b = graph.add_node(points[1], NodeType::Junction);
        let edge = graph.add_edge(crate::simulation::network::build_surface_edge(
            a,
            b,
            points,
            1,
            1,
            EdgeClass::Tunnel,
        ));
        let mut surface = RoadSurfaceSystem::new(128.0);
        surface.rebuild_all_earthworks(&graph, &mut terrain);
        let piece = &surface.compiled_visual_span_pieces()[&edge];
        let (start, end) = RoadSurfaceSystem::tunnel_bore_ranges(&piece.sections, &terrain)[0];
        let portal_x = piece.sections[start].center_xz.x as f32;
        let (first, _) = terrain.world_to_grid_coords(portal_x, 0.0);
        let (last, _) = terrain.world_to_grid_coords(piece.sections[end].center_xz.x as f32, 0.0);
        for x in first.ceil() as usize..=last.floor() as usize {
            let (wx, _) = terrain.grid_to_world_coords(x, 0);
            assert_eq!(
                terrain.sample_visual_height_world(wx, 0.0),
                terrain.sample_height_world(wx, 0.0),
                "floor stamps must leave covered DEM samples unchanged, including an aligned roof rail: x={wx}, hill_shift={hill_shift}"
            );
        }
        assert!(
            piece
                .short_tunnel
                .as_ref()
                .unwrap()
                .concrete
                .iter()
                .any(|quad| {
                    [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]]
                        .into_iter()
                        .any(|tri| {
                            RoadSurfaceSystem::triangle_height_at_xz(
                                &tri,
                                super::super::super::backend::RoadVec2::new(
                                    f64::from(portal_x - 4.0),
                                    6.0,
                                ),
                            )
                            .is_some_and(|height| height.abs() < 0.25)
                        })
                }),
            "concrete apron closes the widened cutout beside the pavement"
        );
        let bounds = graph.paved_bounds(edge, 80.0);
        let boundaries = piece.terrain_clip_boundary_loops();
        for end in [-1.0, 1.0] {
            assert!(
                boundaries
                    .iter()
                    .flat_map(|boundary| &boundary.points_world)
                    .any(|point| {
                        point.x * end > 0.0 && point.z.abs() > f64::from(bounds[1]) + 2.0
                    }),
                "the terrain cut follows the flared retaining wings at both ends"
            );
        }
    }
}
