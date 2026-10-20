// SPDX-License-Identifier: GPL-2.0-only

//! Cross-section regressions against the pre-median road products.

use super::*;
use crate::simulation::network::graph::Edge;
use crate::simulation::network::lanes::LaneSystem;
use crate::simulation::network::surface::RoadSurfaceSystem;
use crate::simulation::network::types::{EdgeClass, NodeType, TransitFlags, TransitType};
use crate::simulation::terrain::TerrainSystem;

fn fixture(arms: usize, slope: bool) -> (RegionGraph, TerrainSystem) {
    let mut graph = RegionGraph::new();
    let mut terrain = TerrainSystem::with_chunking(257, 257, 1.0, 32, 0.0);
    let height = |x: f32| if slope { x * 0.025 } else { 0.0 };
    if slope {
        for z in 0..257 {
            for x in 0..257 {
                terrain.set_height(x, z, height(x as f32 - 128.0));
            }
        }
    }
    let center = graph.add_node(Vector3::ZERO, NodeType::Junction);
    for (x, z) in [(80.0, 0.0), (-60.0, 50.0), (-50.0, -65.0), (15.0, 80.0)]
        .into_iter()
        .take(arms)
    {
        let end = graph.add_node(Vector3::new(x, height(x), z), NodeType::Junction);
        let points = vec![Vector3::ZERO, graph.node(end).pos];
        graph.add_edge(Edge {
            start_node: center,
            end_node: end,
            primary_type: TransitType::Road,
            allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
            class: EdgeClass::Standard,
            width: 14.0,
            fwd_lanes: 2,
            bkw_lanes: 2,
            speed_limit: 50.0 / 3.6,
            physical_length: points[1].length(),
            physical_geometry: points.clone(),
            geometry: points,
            ..Default::default()
        });
    }
    graph.rebuild_all_indices();
    graph.rebuild_intersection_clips();
    (graph, terrain)
}

fn digest(value: impl std::fmt::Debug) -> u64 {
    format!("{value:?}")
        .bytes()
        .fold(0xcbf29ce484222325, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3)
        })
}

// Preserve the recorded visible-pose debug representation while ignoring new ownership metadata.
struct VisiblePose<'a, K> {
    name: &'static str,
    position: &'a Vector3,
    yaw: f32,
    kind: &'a K,
}

impl<K: std::fmt::Debug> std::fmt::Debug for VisiblePose<'_, K> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct(self.name)
            .field("position", self.position)
            .field("yaw", &self.yaw)
            .field("kind", self.kind)
            .finish()
    }
}

#[test]
fn median_free_road_products_match_pre_median_fixtures() {
    let mut actual = Vec::new();
    let mut roadbed_actual = Vec::new();
    for (arms, slope) in [(1, false), (2, false), (3, false), (4, false), (3, true)] {
        let (mut graph, terrain) = fixture(arms, slope);
        let mut surface = RoadSurfaceSystem::new(128.0);
        assert!(surface.compile_dirty(&graph, &terrain));
        let mut lanes = LaneSystem::new();
        lanes.rebuild(&mut graph);
        let mesh =
            road::RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
        assert!(
            mesh.median_grass.is_none(),
            "legacy chunks must not allocate optional grass buffers"
        );
        let mut hash = Vec::new();
        macro_rules! layer {
            ($($field:ident),*) => { $(hash.push(digest(&mesh.$field));)* };
        }
        layer!(
            earthwork_vertices,
            earthwork_normals,
            earthwork_uvs,
            earthwork_colors,
            curb_vertices,
            curb_normals,
            curb_uvs,
            curb_colors,
            raised_step_vertices,
            raised_step_normals,
            raised_step_uvs,
            raised_step_colors,
            sidewalk_vertices,
            sidewalk_normals,
            sidewalk_uvs,
            sidewalk_colors,
            road_vertices,
            road_normals,
            road_uvs,
            road_colors,
            road_crossings,
            marking_vertices,
            marking_normals,
            marking_uvs,
            marking_colors,
            concrete_vertices,
            concrete_normals,
            concrete_uvs,
            concrete_colors
        );
        hash.push(digest(
            mesh.street_lights
                .iter()
                .map(|light| VisiblePose {
                    name: "StreetLight",
                    position: &light.position,
                    yaw: light.yaw,
                    kind: &light.kind,
                })
                .collect::<Vec<_>>(),
        ));
        hash.push(digest(
            mesh.road_signs
                .iter()
                .map(|sign| VisiblePose {
                    name: "RoadSign",
                    position: &sign.position,
                    yaw: sign.yaw,
                    kind: &sign.kind,
                })
                .collect::<Vec<_>>(),
        ));
        // Junction paint and connector defaults intentionally change in ROAD-67. The separate
        // road_pockets_legacy_test bridge pins paint-free roadbed hashes to 6ec0b858 on load.
        roadbed_actual.push(digest(&hash));
        for lane in &lanes.lanes {
            hash.push(digest((
                &lane.geometry,
                lane.edge_id,
                lane.is_fwd,
                lane.lane_idx,
                lane.next_lanes.iter().collect::<Vec<_>>(),
            )));
        }
        let crossings = (0..lanes.lanes.len())
            .filter_map(|id| lanes.crosswalk(id).map(|crossing| (id, crossing)))
            .collect::<BTreeMap<_, _>>();
        hash.push(digest(crossings));
        let sections = surface
            .compiled_sections()
            .iter()
            .collect::<BTreeMap<_, _>>();
        hash.push(digest(sections));
        actual.push(digest(hash));
    }
    // Render product hashes include ROAD-67 arrows and solid approach lines; terminal/bend
    // fixtures remain identical to ff3f9882. The bridge separately checks every roadbed layer.
    assert_eq!(
        roadbed_actual,
        vec![
            1451289510056481026,
            11020531984807302493,
            7303861726095382493,
            17719816073361374844,
            17143503236276419293
        ]
    );
    // The combined hashes include ROAD-67's restricted connectors and the tangent circular
    // turn follow-up. The unchanged roadbed hashes above remain the geometry acceptance gate.
    assert_eq!(
        actual,
        vec![
            7042786987200519549,
            16177303824768118247,
            17072291034510577630,
            437264782030988095,
            9851663238608887122
        ]
    );
}

#[test]
fn median_bands_lanes_and_junctions_compile_for_each_style() {
    use crate::simulation::network::graph::{MedianStyle, RoadMedian};
    use crate::simulation::network::lanes::LaneType;
    use crate::simulation::network::surface::RoadSurfaceBandKind as K;
    let mut failed = Vec::new();
    for (style, width) in [
        (MedianStyle::Painted, 1.0),
        (MedianStyle::KerbedGrass, 5.0),
        (MedianStyle::KerbedPaving, 2.0),
    ] {
        for (fwd, bkw) in [(1, 1), (2, 2), (1, 2)] {
            for (arms, slope) in [(1, false), (2, false), (3, false), (4, false), (3, true)] {
                let (mut graph, terrain) = fixture(arms, slope);
                let median = RoadMedian {
                    style,
                    width_m: width,
                    start_setback_m: 0.0,
                    end_setback_m: 0.0,
                };
                graph
                    .set_edge_cross_section(0, fwd, bkw, Some(median))
                    .unwrap();
                // The two-arm case keeps the median through its bend, with opposite stored axes.
                if arms == 2 {
                    graph
                        .set_edge_cross_section(1, bkw, fwd, Some(median))
                        .unwrap();
                }
                graph.rebuild_intersection_clips();
                let mut surface = RoadSurfaceSystem::new(128.0);
                if !surface.compile_dirty(&graph, &terrain) {
                    failed.push(format!(
                        "{style:?}, lanes={fwd}/{bkw}, arms={arms}, slope={slope}: {:?}",
                        surface.last_compile_failure_label()
                    ));
                    continue;
                }
                let sections = &surface.compiled_sections()[&0];
                let middle = sections
                    .iter()
                    .min_by(|a, b| (a.s_m - 40.0).abs().total_cmp(&(b.s_m - 40.0).abs()))
                    .unwrap();
                assert_eq!(
                    middle.bands.iter().map(|b| b.kind).collect::<Vec<_>>(),
                    vec![
                        K::Sidewalk,
                        K::CurbOrShoulder,
                        K::Carriageway,
                        K::CurbOrShoulder,
                        K::Median,
                        K::CurbOrShoulder,
                        K::Carriageway,
                        K::CurbOrShoulder,
                        K::Sidewalk
                    ]
                );
                let center = graph.edge(0).median_center_offset();
                assert!((middle.bands[6].lateral_start_m - center - width * 0.5).abs() < 0.001);
                assert!((middle.bands[2].lateral_end_m - center + width * 0.5).abs() < 0.001);
                let mut lanes = LaneSystem::new();
                lanes.rebuild(&mut graph);
                if arms == 2 {
                    assert!((0..lanes.lanes.len()).all(|id| lanes.lanes[id].node_id != 0 || lanes.crosswalk(id).is_none()), "continuous median must not have a default crossing");
                }
                for lane in lanes
                    .lanes
                    .iter()
                    .filter(|lane| lane.edge_id == 0 && lane.lane_type == LaneType::Vehicle)
                {
                    let nearest = lane
                        .geometry
                        .iter()
                        .map(|p| (p.z - center).abs())
                        .fold(f32::MAX, f32::min);
                    assert!(nearest >= width * 0.5 + crate::config::LANE_WIDTH * 0.5 - 0.001);
                }
                for lane in lanes
                    .lanes
                    .iter()
                    .filter(|lane| lane.node_id == 0 && lane.lane_type == LaneType::Vehicle)
                {
                    for pair in lane.geometry.windows(2) {
                        for sample in 0..=8 {
                            let p = pair[0].lerp(pair[1], sample as f32 / 8.0);
                            assert!(
                                !over_median(&surface, p),
                                "connector crosses non-carriageway {style:?}, arms={arms}, p={p:?}"
                            );
                        }
                    }
                }
                let mesh = road::RoadRenderer
                    .generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
                assert!(mesh.render_payload_is_valid());
                if style == MedianStyle::KerbedGrass {
                    assert!(
                        mesh.median_grass
                            .as_ref()
                            .is_some_and(|grass| !grass.vertices.is_empty())
                    );
                    assert!(
                    mesh.street_lights
                        .iter()
                        .any(|light| light.kind == road::street_lights::StreetLightKind::Median)
                );
                }
            }
        }
    }
    assert!(failed.is_empty(), "{}", failed.join("\n"));
}

#[test]
fn median_grass_owner_extraction_and_preview_keep_every_attribute() {
    use crate::simulation::network::graph::RoadMedian;
    use std::collections::HashSet;
    let (mut graph, terrain) = fixture(1, false);
    graph
        .set_edge_cross_section(0, 1, 1, RoadMedian::from_tool(2, 5.0).unwrap())
        .unwrap();
    graph.rebuild_intersection_clips();
    let mut surface = RoadSurfaceSystem::new(128.0);
    assert!(surface.compile_dirty(&graph, &terrain));
    let mut lanes = LaneSystem::new();
    lanes.rebuild(&mut graph);
    let mesh =
        road::RoadRenderer.generate_mesh_data_with_surface(&graph, &lanes, &terrain, &surface);
    let expected = mesh.median_grass.as_deref().unwrap();
    let owners = HashSet::from([
        NetworkMeshOwner::Edge(0),
        NetworkMeshOwner::Node(0),
        NetworkMeshOwner::Node(1),
    ]);
    for retained in [
        mesh.without_owners(&HashSet::new()),
        mesh.preview_partition(&owners, &HashSet::new(), &[], Vector2::ZERO, true),
    ] {
        let grass = retained.median_grass.as_deref().unwrap();
        assert_eq!(
            (&grass.vertices, &grass.normals, &grass.uvs, &grass.colors),
            (
                &expected.vertices,
                &expected.normals,
                &expected.uvs,
                &expected.colors
            )
        );
        assert!(retained.render_payload_is_valid());
    }
    assert!(mesh.without_owners(&owners).is_empty());
    assert!(
        mesh.preview_partition(&owners, &HashSet::new(), &[], Vector2::ZERO, false)
            .is_empty()
    );
}

#[test]
fn explicit_crossing_opens_a_continuous_median_before_the_zebra() {
    use crate::simulation::network::graph::RoadMedian;
    let (mut graph, terrain) = fixture(2, false);
    let median = RoadMedian::from_tool(2, 5.0).unwrap();
    for id in 0..2 {
        graph.set_edge_cross_section(id, 1, 1, median).unwrap();
    }
    graph.set_crosswalk_override(0, 0, true);
    graph.rebuild_intersection_clips();
    let mut surface = RoadSurfaceSystem::new(128.0);
    assert!(
        surface.compile_dirty(&graph, &terrain),
        "{:?}",
        surface.last_compile_failure_label()
    );
    let mut lanes = LaneSystem::new();
    lanes.rebuild(&mut graph);
    let markings: Vec<_> = (0..lanes.lanes.len())
        .filter(|&id| lanes.lanes[id].node_id == 0)
        .filter_map(|id| lanes.crosswalk_marking(id))
        .collect();
    assert!(!markings.is_empty());
    for marking in markings {
        for sample in 0..=32 {
            assert!(!over_median(
                &surface,
                marking.start.lerp(marking.end, sample as f32 / 32.0)
            ));
        }
    }
}

#[test]
fn median_setbacks_and_divided_to_undivided_handoffs_compile() {
    use crate::simulation::network::graph::RoadMedian;
    for arms in [2, 3] {
        for style in [1, 2, 3] {
            let (mut graph, terrain) = fixture(arms, false);
            let width = [0.0, 1.0, 5.0, 2.0][style as usize];
            let mut median = RoadMedian::from_tool(style, width).unwrap().unwrap();
            median.start_setback_m = 3.0;
            median.end_setback_m = 2.0;
            graph.set_edge_cross_section(0, 1, 1, Some(median)).unwrap();
            graph.rebuild_intersection_clips();
            let mut surface = RoadSurfaceSystem::new(128.0);
            assert!(
                surface.compile_dirty(&graph, &terrain),
                "style={style}, arms={arms}: {:?}",
                surface.last_compile_failure_label()
            );
            let edge = graph.edge(0);
            let zebra_far = edge.start_clip
                + crate::config::CROSSWALK_INSET
                + crate::config::CROSSWALK_LENGTH_M * 0.5;
            for section in surface.compiled_sections()[&0].iter() {
                let band = &section.bands[4];
                if section.s_m <= zebra_far + median.start_setback_m {
                    assert_eq!(band.lateral_start_m, band.lateral_end_m);
                }
            }
        }
    }
}

// Test-only strict triangle containment: existing terminal turns may leave the asphalt cap,
// but no connector may enter an actual median band, on either a span or a bend.
fn over_median(surface: &RoadSurfaceSystem, point: Vector3) -> bool {
    use crate::simulation::network::surface::{RoadSurfaceBandKind, RoadVec3};
    let inside = |triangle: [RoadVec3; 3]| {
        let p = (f64::from(point.x), f64::from(point.z));
        let cross =
            |a: RoadVec3, b: RoadVec3| (b.x - a.x) * (p.1 - a.z) - (b.z - a.z) * (p.0 - a.x);
        let a = cross(triangle[0], triangle[1]);
        let b = cross(triangle[1], triangle[2]);
        let c = cross(triangle[2], triangle[0]);
        a > 1e-6 && b > 1e-6 && c > 1e-6 || a < -1e-6 && b < -1e-6 && c < -1e-6
    };
    surface.compiled_visual_span_pieces().values().any(|piece| {
        piece
            .span_owned_regions
            .iter()
            .filter(|r| r.owner().kind == RoadSurfaceBandKind::Median)
            .any(|r| piece.region_quad(r).triangles().iter().copied().any(inside))
    }) || surface.compiled_visual_node_pieces().values().any(|piece| {
        piece
            .owned_regions
            .iter()
            .filter(|r| r.kind == RoadSurfaceBandKind::Median)
            .any(|r| r.polygon.triangles().any(inside))
    })
}
