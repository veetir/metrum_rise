// SPDX-License-Identifier: GPL-2.0-only

//! Exact ordered compiled-product capture, shared unchanged by baseline and candidate binaries.

use super::*;
use crate::simulation::network::{
    build_surface_edge,
    graph::{MedianStyle, RoadLaneTaper, RoadMedian},
    types::{EdgeClass, NodeType, TransitFlags},
};
use sha2::{Digest, Sha256};

type Records = BTreeMap<String, Vec<u64>>;

fn f32s(values: impl IntoIterator<Item = f32>) -> Vec<u64> {
    values.into_iter().map(|v| u64::from(v.to_bits())).collect()
}

fn capture(name: &str, fixture: &mut Fixture, records: &mut Records) {
    fixture.graph =
        crate::simulation::save::tests::road_sections::migrate_compile_fixture(&fixture.graph);
    fixture.network.road_surface.clear();
    fixture
        .network
        .road_surface
        .compile_dirty(&fixture.graph, &fixture.terrain);
    fixture.network.lane_system.rebuild(&mut fixture.graph);
    fixture.network.lane_system.sync_heights_to_visible_surface(
        &fixture.graph,
        &fixture.terrain,
        &fixture.network.road_surface,
    );
    let mesh = fixture.mesh();
    macro_rules! floats {
        ($field:ident, $($axis:ident),+) => {
            records.insert(format!("{name}/mesh/{}", stringify!($field)), f32s(mesh.$field.iter().flat_map(|v| [$(v.$axis),+])));
        };
    }
    floats!(earthwork_vertices, x, y, z);
    floats!(earthwork_normals, x, y, z);
    floats!(earthwork_uvs, x, y);
    floats!(earthwork_colors, r, g, b, a);
    floats!(curb_vertices, x, y, z);
    floats!(curb_normals, x, y, z);
    floats!(curb_uvs, x, y);
    floats!(curb_colors, r, g, b, a);
    floats!(raised_step_vertices, x, y, z);
    floats!(raised_step_normals, x, y, z);
    floats!(raised_step_uvs, x, y);
    floats!(raised_step_colors, r, g, b, a);
    floats!(sidewalk_vertices, x, y, z);
    floats!(sidewalk_normals, x, y, z);
    floats!(sidewalk_uvs, x, y);
    floats!(sidewalk_colors, r, g, b, a);
    floats!(road_vertices, x, y, z);
    floats!(road_normals, x, y, z);
    floats!(road_uvs, x, y);
    floats!(road_colors, r, g, b, a);
    floats!(marking_vertices, x, y, z);
    floats!(marking_normals, x, y, z);
    floats!(marking_uvs, x, y);
    floats!(marking_colors, r, g, b, a);
    floats!(concrete_vertices, x, y, z);
    floats!(concrete_normals, x, y, z);
    floats!(concrete_uvs, x, y);
    floats!(concrete_colors, r, g, b, a);
    floats!(road_crossings, x, y, z, w);
    if let Some(grass) = &mesh.median_grass {
        records.insert(
            format!("{name}/mesh/grass"),
            f32s(grass.vertices.iter().flat_map(|v| [v.x, v.y, v.z])),
        );
        records.insert(
            format!("{name}/mesh/grass_normals"),
            f32s(grass.normals.iter().flat_map(|v| [v.x, v.y, v.z])),
        );
        records.insert(
            format!("{name}/mesh/grass_uvs"),
            f32s(grass.uvs.iter().flat_map(|v| [v.x, v.y])),
        );
        records.insert(
            format!("{name}/mesh/grass_colors"),
            f32s(grass.colors.iter().flat_map(|v| [v.r, v.g, v.b, v.a])),
        );
    }
    for (i, light) in mesh.street_lights.iter().enumerate() {
        let mut row = vec![light.owner_edge as u64, light.kind as u64];
        row.extend(f32s([
            light.position.x,
            light.position.y,
            light.position.z,
            light.yaw,
        ]));
        records.insert(format!("{name}/light/{i}"), row);
    }
    for (i, sign) in mesh.road_signs.iter().enumerate() {
        let mut row = vec![sign.owner_edge as u64, sign.kind as u64];
        row.extend(f32s([
            sign.position.x,
            sign.position.y,
            sign.position.z,
            sign.yaw,
        ]));
        records.insert(format!("{name}/sign/{i}"), row);
    }
    for (&edge, sections) in fixture.network.road_surface.compiled_sections() {
        let mut row = Vec::new();
        for section in sections.iter() {
            row.extend(f32s([section.s_m, section.center_height_m]));
            row.extend([
                section.center_xz.x.to_bits(),
                section.center_xz.y.to_bits(),
                section.tangent_xz.x.to_bits(),
                section.tangent_xz.y.to_bits(),
                section.lateral_xz.x.to_bits(),
                section.lateral_xz.y.to_bits(),
            ]);
            row.push(section.bands.len() as u64);
            for band in &section.bands {
                row.push(band.kind as u64);
                row.extend(f32s([
                    band.lateral_start_m,
                    band.lateral_end_m,
                    band.height_start_m,
                    band.height_end_m,
                ]));
            }
        }
        records.insert(format!("{name}/edge/{edge}/sections"), row);
    }
    let polygon = |polygon: &crate::simulation::network::surface::RoadSurfaceVisualPolygon| {
        let mut row: Vec<_> = polygon
            .points_world
            .iter()
            .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
            .collect();
        row.push(u64::MAX);
        row.extend(polygon.triangles().flat_map(|t| {
            t.into_iter()
                .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
        }));
        row
    };
    macro_rules! clip_loops {
        ($owner:expr, $loops:expr) => {
            for (i, clip) in $loops.iter().enumerate() {
                let mut row: Vec<_> = clip
                    .points_world
                    .iter()
                    .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
                    .collect();
                row.push(u64::MAX);
                for edge in &clip.source_edges {
                    row.extend([
                        edge.start.x.to_bits(),
                        edge.start.y.to_bits(),
                        edge.start.z.to_bits(),
                        edge.end.x.to_bits(),
                        edge.end.y.to_bits(),
                        edge.end.z.to_bits(),
                        edge.kind as u64,
                    ]);
                }
                records.insert(format!("{name}/{}/clip/{i}", $owner), row);
            }
        };
    }
    for (&id, piece) in fixture.network.road_surface.compiled_visual_span_pieces() {
        let owner = format!("span/{id}");
        clip_loops!(owner, piece.terrain_clip_boundary_loops());
        for (i, p) in piece.outer_boundary_loops.iter().enumerate() {
            records.insert(format!("{name}/{owner}/outer/{i}"), polygon(p));
        }
        for (i, p) in piece.earthwork_outer_boundary_loops.iter().enumerate() {
            records.insert(format!("{name}/{owner}/earth_boundary/{i}"), polygon(p));
        }
        for (i, face) in piece.render_earthwork_faces.iter().enumerate() {
            let mut row = vec![face.kind as u64];
            row.extend([
                face.inner_start.x.to_bits(),
                face.inner_start.y.to_bits(),
                face.inner_start.z.to_bits(),
                face.inner_end.x.to_bits(),
                face.inner_end.y.to_bits(),
                face.inner_end.z.to_bits(),
            ]);
            row.extend(polygon(&face.polygon));
            records.insert(format!("{name}/{owner}/earth_face/{i}"), row);
        }
        if let Some(filled) = &piece.filled_crossing {
            records.insert(
                format!("{name}/{owner}/filled_lower"),
                filled.lower_edges.iter().map(|&v| v as u64).collect(),
            );
            for (label, quads) in [("fill", &filled.fill), ("concrete", &filled.concrete)] {
                records.insert(
                    format!("{name}/{owner}/{label}"),
                    quads
                        .iter()
                        .flatten()
                        .flat_map(|v| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()])
                        .collect(),
                );
            }
        }
    }
    for (&id, piece) in fixture.network.road_surface.compiled_visual_node_pieces() {
        let owner = format!("node/{id}");
        clip_loops!(owner, piece.terrain_clip_boundary_loops);
        for (i, p) in piece.outer_boundary_loops.iter().enumerate() {
            records.insert(format!("{name}/{owner}/outer/{i}"), polygon(p));
        }
        for (i, p) in piece.surface_polygons().enumerate() {
            records.insert(format!("{name}/{owner}/surface/{i}"), polygon(p));
        }
        for (i, p) in piece.raised_step_face_polygons.iter().enumerate() {
            records.insert(format!("{name}/{owner}/step/{i}"), polygon(p));
        }
        for (i, face) in piece.render_earthwork_faces.iter().enumerate() {
            records.insert(
                format!("{name}/{owner}/earth_face/{i}"),
                polygon(&face.polygon),
            );
        }
    }
    for (id, edge) in fixture.graph.edges().iter().enumerate() {
        records.insert(
            format!("{name}/edge/{id}/clips"),
            f32s([edge.start_clip, edge.end_clip]),
        );
        for side in [-1, 1] {
            for station in [0.0, 0.25, 0.5, 0.75, 1.0] {
                if let Some((center, outward, setback)) =
                    BuildingAllocator::explicit_frontage_transform(
                        &fixture.graph,
                        id,
                        station,
                        side,
                        10.0,
                    )
                {
                    records.insert(
                        format!("{name}/edge/{id}/frontage/{side}/{station}"),
                        f32s([center.x, center.y, outward.x, outward.y, setback]),
                    );
                }
            }
        }
    }
    for (id, lane) in fixture.network.lane_system.lanes.iter().enumerate() {
        let mut row = vec![
            lane.edge_id as u64,
            lane.node_id as u64,
            lane.is_fwd as u64,
            lane.lane_idx as u64,
        ];
        row.extend(f32s([lane.length]));
        row.extend(f32s(lane.geometry.iter().flat_map(|v| [v.x, v.y, v.z])));
        row.extend(f32s(lane.cum_dist.iter().copied()));
        row.extend(lane.next_lanes.iter().map(|&v| v as u64));
        records.insert(format!("{name}/lane/{id}"), row);
        if let Some(marking) = fixture.network.lane_system.crosswalk_marking(id) {
            let mut row = vec![marking.edge_id as u64];
            row.extend(f32s([
                marking.start.x,
                marking.start.y,
                marking.start.z,
                marking.end.x,
                marking.end.y,
                marking.end.z,
            ]));
            records.insert(format!("{name}/crossing/{id}"), row);
        }
    }
}

fn add(
    fixture: &mut Fixture,
    points: &[Vector3],
    fwd: u8,
    bkw: u8,
    modes: u8,
    width: f32,
    class: EdgeClass,
) -> usize {
    let start = fixture.graph.add_node(points[0], NodeType::Junction);
    let end = fixture
        .graph
        .add_node(*points.last().unwrap(), NodeType::Junction);
    let mut edge = build_surface_edge(start, end, points.to_vec(), fwd, bkw, class);
    edge.width = width;
    edge.allowed_types = modes;
    edge.speed_limit = 40.0 / 3.6;
    fixture.graph.add_edge(edge)
}

#[test]
fn exact_compiled_legacy_baseline() {
    let mut records = Records::new();
    let modes = TransitFlags::CAR | TransitFlags::FOOT;
    for (name, sloped, points, fwd, bkw, flags, width, class) in [
        (
            "straight",
            false,
            vec![(20., 30.), (160., 30.)],
            1,
            1,
            modes,
            7.,
            EdgeClass::Standard,
        ),
        (
            "bend",
            false,
            vec![(20., 30.), (80., 30.), (130., 80.)],
            1,
            1,
            modes,
            7.,
            EdgeClass::Standard,
        ),
        (
            "sloped",
            true,
            vec![(20., 30.), (160., 30.)],
            1,
            1,
            modes,
            7.,
            EdgeClass::Standard,
        ),
        (
            "foot",
            false,
            vec![(20., 30.), (160., 30.)],
            0,
            0,
            TransitFlags::FOOT,
            1.25,
            EdgeClass::Standard,
        ),
        (
            "no_foot",
            false,
            vec![(20., 30.), (160., 30.)],
            1,
            1,
            TransitFlags::CAR,
            7.,
            EdgeClass::Standard,
        ),
        (
            "asymmetric",
            false,
            vec![(20., 30.), (160., 30.)],
            2,
            1,
            modes,
            10.5,
            EdgeClass::Standard,
        ),
        (
            "exceptional_width",
            false,
            vec![(20., 30.), (160., 30.)],
            1,
            1,
            modes,
            8.125,
            EdgeClass::Standard,
        ),
        (
            "bridge",
            false,
            vec![(20., 30.), (160., 30.)],
            1,
            1,
            modes,
            7.,
            EdgeClass::Bridge,
        ),
    ] {
        let mut fixture = Fixture::new(sloped);
        let points: Vec<_> = points
            .into_iter()
            .map(|(x, z)| {
                let mut p = fixture.point(x, z);
                if class == EdgeClass::Bridge {
                    p.y += 8.;
                }
                p
            })
            .collect();
        add(&mut fixture, &points, fwd, bkw, flags, width, class);
        capture(name, &mut fixture, &mut records);
    }
    for (name, cross, foot, zebra) in [
        ("t", false, false, true),
        ("cross", true, false, true),
        ("mixed", false, true, true),
        ("unmarked", true, false, false),
    ] {
        let mut fixture = Fixture::new(false);
        fixture.add(&[fixture.point(20., 100.), fixture.point(220., 100.)], 1, 1);
        fixture.add(
            &[
                fixture.point(120., if cross { 20. } else { 100. }),
                fixture.point(120., 220.),
            ],
            if foot { 0 } else { 1 },
            if foot { 0 } else { 1 },
        );
        if !zebra {
            for node in 0..fixture.graph.node_count() {
                for edge in fixture.graph.node_adjacency(node as u32).to_vec() {
                    fixture
                        .graph
                        .set_crosswalk_override(node as u32, edge, false);
                }
            }
        }
        capture(name, &mut fixture, &mut records);
    }
    for (name, style, width) in [
        ("painted", MedianStyle::Painted, 1.),
        ("grass", MedianStyle::KerbedGrass, 4.),
        ("paved", MedianStyle::KerbedPaving, 2.),
    ] {
        let mut fixture = Fixture::new(false);
        let points = [fixture.point(20., 100.), fixture.point(220., 100.)];
        let id = add(
            &mut fixture,
            &points,
            1,
            1,
            modes,
            7. + width,
            EdgeClass::Standard,
        );
        fixture.graph.edge_mut(id).median = Some(RoadMedian {
            width_m: width,
            style,
            start_setback_m: 2.,
            end_setback_m: 3.,
        });
        capture(name, &mut fixture, &mut records);
    }
    for (name, left, right) in [
        ("left_pocket", true, false),
        ("right_pocket", false, true),
        ("taper", false, false),
    ] {
        let mut fixture = Fixture::new(false);
        let points = [fixture.point(20., 100.), fixture.point(220., 100.)];
        let id = add(
            &mut fixture,
            &points,
            2,
            1,
            modes,
            14.5,
            EdgeClass::Standard,
        );
        let edge = fixture.graph.edge_mut(id);
        edge.median = Some(RoadMedian {
            width_m: 4.,
            style: MedianStyle::KerbedGrass,
            start_setback_m: 0.,
            end_setback_m: 0.,
        });
        edge.lane_tapers = Some(Box::new([
            Some(RoadLaneTaper {
                length_m: 30.,
                from_fwd: 1,
                from_bkw: 1,
                from_median_m: 4.,
                left_pocket: left,
                right_pocket: right,
            }),
            None,
        ]));
        fixture.graph.set_node_type(0, NodeType::LaneTransition);
        capture(name, &mut fixture, &mut records);
    }
    {
        let mut fixture = Fixture::new(false);
        let points = [fixture.point(20., 100.), fixture.point(220., 100.)];
        let id = add(&mut fixture, &points, 2, 0, modes, 7., EdgeClass::Standard);
        fixture.graph.edge_mut(id).bus_lanes = [1, 0];
        capture("bus", &mut fixture, &mut records);
    }
    {
        let mut fixture = Fixture::new(false);
        let lower = [fixture.point(120., 20.), fixture.point(120., 220.)];
        add(&mut fixture, &lower, 1, 1, modes, 7., EdgeClass::Standard);
        let upper = [Vector3::new(20., 8., 120.), Vector3::new(220., 8., 120.)];
        let id = add(&mut fixture, &upper, 1, 1, modes, 7., EdgeClass::Bridge);
        fixture.graph.edge_mut(id).crossing_level = 1;
        capture("filled_crossing", &mut fixture, &mut records);
    }
    {
        use crate::simulation::network::roundabouts::{RoundaboutLayout, RoundaboutRequest};
        let mut fixture = Fixture::new(false);
        RoundaboutLayout::prepare(
            &fixture.graph,
            RoundaboutRequest {
                center: fixture.point(120., 120.),
                diameter: 40.,
                angles: vec![0., std::f32::consts::FRAC_PI_2, std::f32::consts::PI],
                replace: None,
            },
        )
        .unwrap()
        .apply(&mut fixture.graph);
        fixture.graph.rebuild_intersection_clips();
        capture("ring", &mut fixture, &mut records);
    }
    {
        use crate::simulation::network::ramps::{RampLayout, RampRequest};
        let mut fixture = Fixture::new(false);
        let id = add(
            &mut fixture,
            &[Vector3::new(-600., 0., 0.), Vector3::new(600., 0., 0.)],
            2,
            2,
            TransitFlags::CAR,
            19.,
            EdgeClass::Standard,
        );
        fixture.graph.edge_mut(id).median = RoadMedian::from_tool(2, 5.).unwrap();
        RampLayout::prepare(
            &fixture.graph,
            RampRequest {
                edge: id,
                station: 600.,
                forward: true,
                exit: true,
                points: vec![Vector3::ZERO, Vector3::new(200., 0., 100.)],
                length: 80.,
                taper: 35.,
            },
        )
        .unwrap()
        .apply(&mut fixture.graph);
        fixture.graph.rebuild_intersection_clips();
        capture("ramp", &mut fixture, &mut records);
    }
    let path = std::env::var("ROAD_SECTION_COMPARE").ok();
    if let Some(path) = path {
        let expected: Records = serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        for key in expected.keys().chain(records.keys()) {
            if expected.get(key) != records.get(key) {
                let index = expected
                    .get(key)
                    .zip(records.get(key))
                    .and_then(|(a, b)| a.iter().zip(b).position(|(a, b)| a != b));
                panic!(
                    "first differing owner/field {key}, ordered word {index:?}: baseline {:?}, candidate {:?}",
                    expected.get(key).map(Vec::len),
                    records.get(key).map(Vec::len)
                );
            }
        }
    }
    if let Ok(path) = std::env::var("ROAD_SECTION_CAPTURE") {
        std::fs::write(path, serde_json::to_vec(&records).unwrap()).unwrap();
    }
    let manifest: BTreeMap<_, _> = records
        .iter()
        .map(|(key, words)| {
            let mut hash = Sha256::new();
            for word in words {
                hash.update(word.to_le_bytes());
            }
            (
                key.clone(),
                hash.finalize()
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<String>(),
            )
        })
        .collect();
    let expected: BTreeMap<String, String> =
        serde_json::from_str(include_str!("legacy_baseline_hashes.json")).unwrap();
    for key in expected.keys().chain(manifest.keys()) {
        assert_eq!(
            expected.get(key),
            manifest.get(key),
            "first differing owner/field {key}"
        );
    }
    if let Ok(path) = std::env::var("ROAD_SECTION_MANIFEST") {
        std::fs::write(path, serde_json::to_vec_pretty(&manifest).unwrap()).unwrap();
    }
}
