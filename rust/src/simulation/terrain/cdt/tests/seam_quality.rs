// SPDX-License-Identifier: GPL-2.0-only

//! Seam simplification and source-provenance quality tests.

use super::*;

#[test]
fn cdt_stops_after_reaching_a_near_degenerate_three_vertex_seam_loop() {
    let source = test_span_boundary_source(77, TerrainCdtRoadBandKind::Sidewalk, 4);
    let points = vec![
        TerrainCdtVertex::new(1.0, 0.12, 1.0),
        TerrainCdtVertex::new(1.004, 0.12, 1.0),
        TerrainCdtVertex::new(1.008, 0.12, 1.0),
        TerrainCdtVertex::new(5.0, 0.12, 1.000_001),
    ];
    let source_edges = points
        .iter()
        .copied()
        .enumerate()
        .map(|(index, start)| TerrainCdtRoadLoopSourceEdge {
            start,
            end: points[(index + 1) % points.len()],
            source,
        })
        .collect::<Vec<_>>();

    let quality = harden_terrain_cdt_road_loop_seams(
        points.clone(),
        &source_edges,
        TerrainCdtPatch::new(0.0, 0.0, 10.0, 10.0, [0.0; 4]),
    );

    assert_eq!(quality.points, vec![points[0], points[2], points[3]]);
    assert_eq!(quality.edge_sources.len(), 3);
    assert_eq!(quality.merged_subbudget_seam_edges, 1);
}

#[test]
fn cdt_merges_subbudget_same_authority_seam_fragments_before_triangulation() {
    let source_a = test_span_boundary_source_range(
        78,
        TerrainCdtRoadBandKind::Sidewalk,
        5,
        15,
        16,
        10.0,
        10.004,
    );
    let source_b = test_span_boundary_source_range(
        78,
        TerrainCdtRoadBandKind::Sidewalk,
        5,
        16,
        17,
        10.004,
        12.0,
    );
    let source_c = test_span_boundary_source_range(
        78,
        TerrainCdtRoadBandKind::Sidewalk,
        5,
        17,
        18,
        12.0,
        14.0,
    );
    let road = vec![
        TerrainCdtVertex::new(3.0, 0.12, 3.0),
        TerrainCdtVertex::new(5.0, 0.12, 3.0),
        TerrainCdtVertex::new(5.004, 0.12, 3.0),
        TerrainCdtVertex::new(7.0, 0.12, 3.0),
        TerrainCdtVertex::new(7.0, 0.12, 7.0),
        TerrainCdtVertex::new(3.0, 0.12, 7.0),
    ];
    let source_edges = vec![
        TerrainCdtRoadLoopSourceEdge {
            start: road[0],
            end: road[1],
            source: source_a,
        },
        TerrainCdtRoadLoopSourceEdge {
            start: road[1],
            end: road[2],
            source: source_b,
        },
        TerrainCdtRoadLoopSourceEdge {
            start: road[2],
            end: road[3],
            source: source_c,
        },
        TerrainCdtRoadLoopSourceEdge {
            start: road[3],
            end: road[4],
            source: source_c,
        },
        TerrainCdtRoadLoopSourceEdge {
            start: road[4],
            end: road[5],
            source: source_c,
        },
        TerrainCdtRoadLoopSourceEdge {
            start: road[5],
            end: road[0],
            source: source_c,
        },
    ];
    let input = TerrainCdtInput::new(
        TerrainCdtPatch::new(0.0, 0.0, 10.0, 10.0, [0.0; 4]),
        vec![TerrainCdtRoadLoop::new_with_source_edges(
            78,
            0,
            road,
            source_edges,
        )],
        Vec::new(),
    );

    let mesh = build_road_touched_terrain_patch(input)
        .expect("source-compatible seam fragments should merge before Spade input");

    assert_eq!(mesh.stats.invalid_constraint_edges, 0);
    assert_eq!(mesh.stats.merged_subbudget_seam_edges, 1);
    assert_eq!(mesh.stats.blocking_degenerate_seam_edges, 0);
    assert_eq!(mesh.seam_quality_samples.len(), 1);
    let sample = mesh.seam_quality_samples[0];
    assert_eq!(
        sample.kind,
        TerrainCdtSeamQualityKind::MergedSubbudgetSeamEdge
    );
    assert!(sample.length_m > MIN_SOURCE_OWNED_SEAM_EDGE_LENGTH_M as f32);
    match sample.source {
        TerrainCdtRoadBoundarySource::SpanSupportBoundary {
            edge_idx,
            source_band_index,
            start_section_index,
            end_section_index,
            start_s_m,
            end_s_m,
            ..
        } => {
            assert_eq!(edge_idx, 78);
            assert_eq!(source_band_index, 5);
            assert_eq!(start_section_index, 15);
            assert_eq!(end_section_index, 17);
            assert_eq!(start_s_m, 10.0);
            assert_eq!(end_s_m, 12.0);
        }
        other => panic!("merged seam must preserve span authority, got {other:?}"),
    }
}

#[test]
fn seam_hardening_keeps_noncollinear_submillimetre_road_corners() {
    let points = vec![
        TerrainCdtVertex::new(3.0, 0.12, 3.0),
        TerrainCdtVertex::new(5.0, 0.12, 3.0),
        TerrainCdtVertex::new(5.004, 0.12, 2.9995),
        TerrainCdtVertex::new(7.0, 0.12, 3.0),
        TerrainCdtVertex::new(7.0, 0.12, 7.0),
        TerrainCdtVertex::new(3.0, 0.12, 7.0),
    ];
    let road = sourced_road_loop(
        79,
        0,
        points.clone(),
        test_span_boundary_source(79, TerrainCdtRoadBandKind::Sidewalk, 5),
    );
    let quality = harden_terrain_cdt_road_loop_seams(
        points.clone(),
        &road.source_edges,
        TerrainCdtPatch::new(0.0, 0.0, 10.0, 10.0, [0.0; 4]),
    );
    assert_eq!(
        quality.points, points,
        "short curves must retain the exact road/terrain cut"
    );
    assert_eq!(quality.merged_subbudget_seam_edges, 0);
}

#[test]
fn cdt_seam_hardening_preserves_a_mergeable_minimum_triangle() {
    let points = vec![
        TerrainCdtVertex::new(0.0, 0.12, 0.0),
        TerrainCdtVertex::new(0.02, 0.12, 0.0),
        TerrainCdtVertex::new(2.0, 0.12, 0.02),
    ];
    let source = test_span_boundary_source(7, TerrainCdtRoadBandKind::Sidewalk, 5);
    let edges = (0..points.len())
        .map(|i| TerrainCdtRoadLoopSourceEdge {
            start: points[i],
            end: points[(i + 1) % points.len()],
            source,
        })
        .collect::<Vec<_>>();
    let quality = harden_terrain_cdt_road_loop_seams(
        points.clone(),
        &edges,
        TerrainCdtPatch::new(-1.0, -1.0, 3.0, 1.0, [0.0; 4]),
    );
    assert_eq!(quality.points, points);
    assert_eq!(quality.merged_subbudget_seam_edges, 0);
    assert_eq!(quality.edge_sources, vec![Some(source); 3]);
    assert_eq!(quality.accepted_seam_edges, 3);
}
