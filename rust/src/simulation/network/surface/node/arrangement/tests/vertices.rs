// SPDX-License-Identifier: GPL-2.0-only

//! Arrangement vertex identity and height-context tests.

use super::*;

#[test]
fn duplicate_arrangement_vertex_key_merges_matching_owner_source_context() {
    let mut arrangement = NodeArrangement::new(42, RoadSurfaceVisualNodePieceKind::JunctionN);
    let point = RoadVec2::new(12.345, -6.789);

    let first = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::Sidewalk, 2)],
            height_field_id(RoadSurfaceBandKind::Sidewalk, 2),
            [seam_source(2)],
        )
        .expect("first vertex should insert");
    let second = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::Sidewalk, 2)],
            height_field_id(RoadSurfaceBandKind::Sidewalk, 2),
            [seam_source(2)],
        )
        .expect("matching context vertex should merge");

    assert_eq!(first, second);
    let vertex = &arrangement.vertices()[first.0];
    assert_eq!(vertex.owners, vec![owner(RoadSurfaceBandKind::Sidewalk, 2)]);
    assert_eq!(vertex.seam_sources, vec![seam_source(2)]);
}

#[test]
fn duplicate_arrangement_vertex_key_merges_same_height_field_and_quantized_height() {
    let mut arrangement = NodeArrangement::new(42, RoadSurfaceVisualNodePieceKind::JunctionN);
    let point = RoadVec2::new(12.345, -6.789);
    let field_id = height_field_id(RoadSurfaceBandKind::Sidewalk, 2);

    let first = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::Sidewalk, 2)],
            field_id,
            [seam_source(2)],
        )
        .expect("first vertex should insert");
    let second = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::Sidewalk, 2)],
            field_id,
            [NodeSeamSource::SidewalkOuter { owner_index: 2 }],
        )
        .expect("same height-field and solved height should share the canonical vertex");

    assert_eq!(first, second);
    let vertex = &arrangement.vertices()[first.0];
    assert_eq!(vertex.height_field_id(), field_id);
    assert_eq!(
        vertex.seam_sources,
        vec![
            NodeSeamSource::SidewalkOuter { owner_index: 2 },
            seam_source(2)
        ]
    );
}

#[test]
fn duplicate_arrangement_vertex_key_keeps_distinct_material_height_field_contexts() {
    let mut arrangement = NodeArrangement::new(42, RoadSurfaceVisualNodePieceKind::JunctionN);
    let point = RoadVec2::new(12.345, -6.789);

    let first = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::Sidewalk, 2)],
            height_field_id(RoadSurfaceBandKind::Sidewalk, 2),
            [seam_source(2)],
        )
        .expect("first vertex should insert");
    let second = arrangement
        .insert_vertex(
            point,
            3.25,
            [owner(RoadSurfaceBandKind::CurbOrShoulder, 1)],
            height_field_id(RoadSurfaceBandKind::CurbOrShoulder, 1),
            [seam_source(1)],
        )
        .expect("distinct height source context should keep its own vertex");

    assert_ne!(first, second);
    assert_eq!(arrangement.vertices().len(), 2);
}

#[test]
fn junctionn_arrangement_vertices_preserve_node_grade_authority() {
    let carriageway = owner(RoadSurfaceBandKind::Carriageway, 0);
    let sidewalk = owner(RoadSurfaceBandKind::Sidewalk, 1);
    let heights = two_region_height_solution(carriageway, sidewalk, Vec::new(), Vec::new());
    let arrangement = NodeArrangement::from_height_solution(&heights)
        .expect("heighted JunctionN should arrange with grade authority");

    assert!(!arrangement.vertices().is_empty());
    assert!(arrangement.vertices().iter().all(|vertex| {
        matches!(
            vertex.grade_authority().decision,
            NodeGradeCarrierDecision::SourceCarrier { .. }
        )
    }));
}

#[test]
fn arrangement_contour_removes_immediate_backtracking_spur() {
    let sidewalk = owner(RoadSurfaceBandKind::Sidewalk, 3);
    let heights = NodeHeightSolution {
        node_id: 41,
        piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
        regions: vec![test_height_region_with_seams(
            RoadSurfaceBandKind::Sidewalk,
            sidewalk,
            vec![
                height_vertex(0.0, 0.0, 0.0),
                height_vertex(1.0, 0.0, 0.0),
                height_vertex(1.001, 0.05, 0.0),
                height_vertex(1.0, 0.0, 0.0),
                height_vertex(0.0, 1.0, 0.0),
            ],
            Vec::new(),
        )],
    };

    let arrangement = NodeArrangement::from_height_solution(&heights)
        .expect("zero-area backtracking spur should canonicalize before CDT");

    assert_eq!(arrangement.regions()[0].outer_loop().len(), 3);
}

#[test]
fn arrangement_contour_preserves_same_millimetre_distinct_canonical_keys() {
    let sidewalk = owner(RoadSurfaceBandKind::Sidewalk, 3);
    let heights = NodeHeightSolution {
        node_id: 42,
        piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
        regions: vec![test_height_region_with_seams(
            RoadSurfaceBandKind::Sidewalk,
            sidewalk,
            vec![
                height_vertex(0.0000, 0.0, 0.0),
                height_vertex(0.0004, 0.4, 0.0),
                height_vertex(0.0004, 0.0, 0.0),
                height_vertex(1.0000, 0.0, 0.0),
                height_vertex(1.0000, 1.0, 0.0),
                height_vertex(0.0000, 1.0, 0.0),
            ],
            Vec::new(),
        )],
    };

    let arrangement = NodeArrangement::from_height_solution(&heights)
        .expect("same-millimetre canonical vertices must remain distinct");

    assert_eq!(
        arrangement.regions()[0].outer_loop().len(),
        6,
        "arrangement cleanup must not collapse distinct canonical XZ keys just because they share a millimetre bucket"
    );
}

#[test]
fn arrangement_rejects_heighted_vertex_without_node_grade_authority() {
    let owner = owner(RoadSurfaceBandKind::Sidewalk, 4);
    let field = NodeBandHeightFieldId::new(0, 4, RoadSurfaceBandKind::Sidewalk);
    let mut shape = vec![
        height_vertex(0.0, 0.0, 1.0),
        height_vertex(1.0, 0.0, 1.0),
        height_vertex(0.0, 1.0, 1.0),
    ];
    for vertex in &mut shape {
        vertex.height_field_id = field;
    }
    let heights = NodeHeightSolution {
        node_id: 81,
        piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
        regions: vec![NodeHeightedRegion {
            kind: RoadSurfaceBandKind::Sidewalk,
            owner,
            height_field_id: field,
            shape: vec![shape],
            area_m2: 0.5,
            seam_constraints: Vec::new(),
        }],
    };

    assert!(matches!(
        NodeArrangement::from_height_solution(&heights),
        Err(NodeArrangementError::MissingGradeAuthority {
            owner: missing_owner,
            height_field_id,
            ..
        }) if missing_owner == owner && height_field_id == field
    ));
}

#[test]
fn duplicate_arrangement_vertex_key_rejects_same_context_height_conflict() {
    let mut arrangement = NodeArrangement::new(7, RoadSurfaceVisualNodePieceKind::JunctionN);
    let point = RoadVec2::new(0.0, 0.0);

    arrangement
        .insert_vertex(
            point,
            1.0,
            [owner(RoadSurfaceBandKind::Sidewalk, 0)],
            height_field_id(RoadSurfaceBandKind::Sidewalk, 0),
            [seam_source(0)],
        )
        .expect("first vertex should insert");

    let result = arrangement.insert_vertex(
        point,
        1.01,
        [owner(RoadSurfaceBandKind::Sidewalk, 0)],
        height_field_id(RoadSurfaceBandKind::Sidewalk, 0),
        [seam_source(0)],
    );

    assert!(matches!(
        result,
        Err(NodeArrangementError::DuplicateVertexHeightConflict { .. })
    ));
}

#[test]
fn arrangement_vertex_requires_explicit_owner() {
    let mut arrangement = NodeArrangement::new(9, RoadSurfaceVisualNodePieceKind::Terminal);

    let result = arrangement.insert_vertex(
        RoadVec2::new(1.0, 2.0),
        0.0,
        [],
        height_field_id(RoadSurfaceBandKind::Sidewalk, 0),
        [seam_source(0)],
    );

    assert!(matches!(
        result,
        Err(NodeArrangementError::EmptyOwnerSet { .. })
    ));
}

#[test]
fn a_micro_contour_reaches_cdt_disposal_without_rejecting_its_junction() {
    let sidewalk = owner(RoadSurfaceBandKind::Sidewalk, 1);
    let main = test_height_region_with_seams(
        RoadSurfaceBandKind::Sidewalk,
        sidewalk,
        vec![
            height_vertex(0.0, 0.0, 106.8),
            height_vertex(1.0, 0.0, 106.8),
            height_vertex(1.0, 1.0, 106.8),
            height_vertex(0.0, 1.0, 106.8),
        ],
        Vec::new(),
    );
    let mut micro = test_height_region_with_seams(
        RoadSurfaceBandKind::Sidewalk,
        sidewalk,
        [
            [808.973452, 1131.612277],
            [808.974254, 1131.613079],
            [811.196066, 1133.833983],
            [808.974307, 1131.613132],
        ]
        .into_iter()
        .map(|[x, z]| height_vertex(x, z, 106.8))
        .collect(),
        Vec::new(),
    );
    micro.area_m2 = 2.4e-8;
    let heights = NodeHeightSolution {
        node_id: 1035,
        piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
        regions: vec![micro, main],
    };
    let mut arrangement = NodeArrangement::from_height_solution(&heights)
        .expect("numeric contour cleanup must not invalidate the entire junction");
    assert_eq!(
        arrangement.regions().len(),
        2,
        "retain source seams through arrangement construction"
    );
    let triangles=crate::simulation::network::surface::RoadSurfaceSystem::build_node_triangulation_from_arrangement(&arrangement)
        .expect("the existing CDT area budget discards only the microscopic top region");
    assert_eq!(triangles.regions.len(), 1);
    assert_eq!(triangles.regions[0].source_region_index, 1);
    arrangement.attach_triangulation(&triangles).unwrap();
    assert!(
        arrangement
            .faces()
            .iter()
            .all(|face| face.region().index() == 1)
    );
    let mut missing = triangles.clone();
    missing.regions.clear();
    assert!(
        matches!(
            arrangement.attach_triangulation(&missing),
            Err(NodeArrangementError::TriangulationRegionCountMismatch { .. })
        ),
        "discarding a real region still fails attachment"
    );
}

#[test]
fn terrain_footprint_keeps_a_refined_non_affine_boundary_grade() {
    use crate::simulation::network::surface::node::triangulation::{
        NodeTriangulatedTriangle, NodeTriangulatedVertex,
    };
    use crate::simulation::network::surface::{RoadSurfaceSystem, RoadVec3};
    let owner = owner(RoadSurfaceBandKind::Carriageway, 0);
    let heights = NodeHeightSolution {
        node_id: 11,
        piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
        regions: vec![test_height_region_with_seams(
            RoadSurfaceBandKind::Carriageway,
            owner,
            vec![
                height_vertex(0.0, 0.0, 10.0),
                height_vertex(1.0, 0.0, 10.0),
                height_vertex(1.0, 1.0, 10.0),
                height_vertex(0.0, 1.0, 10.0),
            ],
            Vec::new(),
        )],
    };
    let mut arrangement = NodeArrangement::from_height_solution(&heights).unwrap();
    let mut triangles =
        RoadSurfaceSystem::build_node_triangulation_from_arrangement(&arrangement).unwrap();
    let region = &mut triangles.regions[0];
    let corner = |x, z| {
        region
            .vertices
            .iter()
            .position(|v| v.point_world.x == x && v.point_world.z == z)
            .unwrap()
    };
    let [a, b, c, d] = [
        corner(0.0, 0.0),
        corner(1.0, 0.0),
        corner(1.0, 1.0),
        corner(0.0, 1.0),
    ];
    let middle = region.vertices.len();
    let midpoint = RoadVec3::new(0.5, 10.25, 0.0);
    region.vertices.push(NodeTriangulatedVertex {
        point_world: midpoint,
        height_field_id: region.height_field_id,
        grade_authority: NodeGradeVertexAuthority::new(
            RoadVec2::new(0.5, 0.0),
            10.25,
            owner,
            region.height_field_id,
            NodeGradeCarrierDecision::SourceCarrier { authority: None },
        ),
    });
    region.triangles = [[a, middle, d], [middle, c, d], [middle, b, c]]
        .map(|vertices| NodeTriangulatedTriangle { vertices })
        .to_vec();
    region.boundary_constraints = vec![[a, middle], [middle, b], [b, c], [c, d], [d, a]];
    arrangement.attach_triangulation(&triangles).unwrap();
    let footprint = vec![vec![vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]]];
    let result =
        RoadSurfaceSystem::node_surface_regions_from_arrangement(&arrangement, &footprint).unwrap();
    let boundary = &result.earthwork_boundary_segments[0];
    let bottom = boundary
        .iter()
        .filter(|e| e.inner_start.z == 0.0 && e.inner_end.z == 0.0)
        .collect::<Vec<_>>();
    assert_eq!(
        bottom.len(),
        2,
        "the terrain cut must retain both affine grade intervals"
    );
    assert!(
        bottom
            .iter()
            .all(|e| e.inner_start == midpoint || e.inner_end == midpoint)
    );
    assert!(
        result.outer_boundary_loops[0]
            .points_world
            .contains(&midpoint)
    );
}
