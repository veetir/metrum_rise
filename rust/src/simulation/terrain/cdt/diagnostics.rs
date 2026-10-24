// SPDX-License-Identifier: GPL-2.0-only

//! Mesh diagnostics and source-provenance summaries.

use std::collections::{BTreeMap, HashSet};

use super::*;

pub(super) struct TerrainCdtDiagnostics {
    #[cfg(test)]
    pub(super) emitted_faces: Vec<TerrainCdtEmittedFace>,
    pub(super) terrain_triangles: Vec<[usize; 3]>,
    pub(super) terrain_triangle_sources: Vec<Vec<TerrainCdtRoadBoundarySource>>,
    pub(super) retaining_wall_triangles: Vec<[usize; 3]>,
    pub(super) retaining_wall_triangle_sources: Vec<Vec<TerrainCdtRoadBoundarySource>>,
    pub(super) max_face_y_delta_m: f32,
    pub(super) max_face_slope_ratio: f32,
    pub(super) longest_triangle_edge_m: f32,
    pub(super) road_seam_faces: usize,
    pub(super) road_seam_max_y_delta_m: f32,
    pub(super) road_seam_max_slope_ratio: f32,
    pub(super) retaining_wall_faces: usize,
    pub(super) retaining_wall_max_y_delta_m: f32,
    pub(super) retaining_wall_max_slope_ratio: f32,
    pub(super) road_seam_face_samples: Vec<TerrainCdtFaceSample>,
    pub(super) retaining_wall_face_samples: Vec<TerrainCdtFaceSample>,
}

#[derive(Clone, Copy)]
struct TerrainFaceMetrics {
    max_y_delta_m: f32,
    max_slope_ratio: f32,
    longest_edge_m: f32,
}

#[derive(Clone, Copy, Default)]
struct RoadConstraintVertex {
    boundary: bool,
    materials: RoadBoundaryMaterials,
}

#[derive(Clone, Copy, Default)]
struct RoadBoundaryMaterials {
    node: Option<(u32, TerrainCdtNodePieceKind)>,
    owners: [bool; 4],
    span: Option<u64>,
    span_owners: [bool; 4],
    span_handoff: bool,
    incompatible: bool,
}

impl RoadBoundaryMaterials {
    fn add_source(&mut self, source: TerrainCdtRoadBoundarySource) {
        let mut other = Self::default();
        let (kind, span_owner) = match source {
            TerrainCdtRoadBoundarySource::NodeFootprintBoundary {
                node_id,
                node_kind,
                owner_kind,
                boundary_source: Some(_),
                ..
            } => {
                other.node = Some((node_id, node_kind));
                (owner_kind, false)
            }
            TerrainCdtRoadBoundarySource::SpanSupportBoundary {
                edge_idx,
                edge_class: TerrainCdtEdgeClass::Standard,
                support_policy: TerrainCdtEarthworkSupportPolicy::StandardFullGroundedSpan,
                band_kind,
                start_section_index,
                end_section_index,
                start_s_m,
                end_s_m,
                ..
            } => {
                other.span = Some(edge_idx);
                other.span_handoff =
                    start_section_index == end_section_index && start_s_m == end_s_m;
                (band_kind, true)
            }
            _ => {
                self.incompatible = true;
                return;
            }
        };
        let slot = match kind {
            TerrainCdtRoadBandKind::Carriageway => 0,
            TerrainCdtRoadBandKind::CurbOrShoulder => 1,
            TerrainCdtRoadBandKind::Sidewalk => 2,
            TerrainCdtRoadBandKind::Median => 3,
            _ => {
                self.incompatible = true;
                return;
            }
        };
        if span_owner {
            other.span_owners[slot] = true;
        } else {
            other.owners[slot] = true;
        }
        self.merge(other);
    }

    fn merge(&mut self, other: Self) {
        self.incompatible |= other.incompatible;
        if let Some(node) = other.node {
            self.incompatible |= self.node.is_some_and(|current| current != node);
            self.node = Some(node);
        }
        if let Some(span) = other.span {
            self.incompatible |= self.span.is_some_and(|current| current != span);
            self.span = Some(span);
        }
        self.span_handoff |= other.span_handoff;
        // Opposite carriageways are distinct band owners of the same span material.
        for (owner, incoming) in self.span_owners.iter_mut().zip(other.span_owners) {
            *owner |= incoming;
        }
        // A junction's material boundary changes band owner at each approach and
        // corner. All incident rails still certify the same node's kerb return;
        // an owner index is not a distinct terrain grade or a foreign junction.
        for (owner, incoming) in self.owners.iter_mut().zip(other.owners) {
            *owner |= incoming;
        }
    }
}

pub(super) fn terrain_face_diagnostics(
    vertices: &[TerrainCdtVertex],
    triangles: &[[usize; 3]],
    road_constraint_sources: &BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
    retaining_wall_required_sources: &[TerrainCdtRoadBoundarySource],
) -> TerrainCdtDiagnostics {
    let mut diagnostics = TerrainCdtDiagnostics {
        #[cfg(test)]
        emitted_faces: Vec::new(),
        terrain_triangles: Vec::with_capacity(triangles.len()),
        terrain_triangle_sources: Vec::with_capacity(triangles.len()),
        retaining_wall_triangles: Vec::new(),
        retaining_wall_triangle_sources: Vec::new(),
        max_face_y_delta_m: 0.0,
        max_face_slope_ratio: 0.0,
        longest_triangle_edge_m: 0.0,
        road_seam_faces: 0,
        road_seam_max_y_delta_m: 0.0,
        road_seam_max_slope_ratio: 0.0,
        retaining_wall_faces: 0,
        retaining_wall_max_y_delta_m: 0.0,
        retaining_wall_max_slope_ratio: 0.0,
        road_seam_face_samples: Vec::new(),
        retaining_wall_face_samples: Vec::new(),
    };
    let mut road_constraint_vertices = vec![RoadConstraintVertex::default(); vertices.len()];
    for (edge, source) in road_constraint_sources {
        for &index in edge {
            road_constraint_vertices[index].boundary = true;
            road_constraint_vertices[index]
                .materials
                .add_source(source.boundary_source);
        }
    }

    for triangle in triangles {
        let points = [
            vertices[triangle[0]],
            vertices[triangle[1]],
            vertices[triangle[2]],
        ];
        let sources = terrain_triangle_road_sources(
            triangle,
            road_constraint_sources,
            &road_constraint_vertices,
        );
        let touches_road_seam = !sources.is_empty();
        let metrics = terrain_face_metrics(points);
        let kerb_return = triangle
            .iter()
            .all(|&i| road_constraint_vertices[i].boundary)
            && {
                let mut materials = RoadBoundaryMaterials::default();
                for &index in triangle {
                    materials.merge(road_constraint_vertices[index].materials);
                }
                source_owned_kerb_return(points, &materials)
            };
        let kind = if kerb_return {
            // The concrete bucket also owns finite-width returns at normal kerb corners.
            // Their 120 mm step is not a slope of the surrounding source terrain.
            TerrainCdtTieInKind::RetainingWall
        } else {
            classify_terrain_tie_in_face(
                &sources,
                retaining_wall_required_sources,
                metrics.max_slope_ratio,
            )
        };
        #[cfg(test)]
        diagnostics.emitted_faces.push(TerrainCdtEmittedFace {
            triangle: *triangle,
            kind,
            sources: sources.clone(),
        });
        diagnostics.max_face_y_delta_m = diagnostics.max_face_y_delta_m.max(metrics.max_y_delta_m);
        diagnostics.max_face_slope_ratio = diagnostics
            .max_face_slope_ratio
            .max(metrics.max_slope_ratio);
        diagnostics.longest_triangle_edge_m = diagnostics
            .longest_triangle_edge_m
            .max(metrics.longest_edge_m);

        match kind {
            TerrainCdtTieInKind::OrdinaryTerrain => {
                diagnostics.terrain_triangles.push(*triangle);
                diagnostics.terrain_triangle_sources.push(sources.clone());
            }
            TerrainCdtTieInKind::RetainingWall => {
                diagnostics.retaining_wall_triangles.push(*triangle);
                diagnostics
                    .retaining_wall_triangle_sources
                    .push(sources.clone());
                diagnostics.retaining_wall_faces += 1;
                diagnostics.retaining_wall_max_y_delta_m = diagnostics
                    .retaining_wall_max_y_delta_m
                    .max(metrics.max_y_delta_m);
                diagnostics.retaining_wall_max_slope_ratio = diagnostics
                    .retaining_wall_max_slope_ratio
                    .max(metrics.max_slope_ratio);
            }
        }

        if !touches_road_seam {
            continue;
        }

        let sample = terrain_face_sample(points, kind, sources, metrics);
        if kind == TerrainCdtTieInKind::RetainingWall {
            insert_road_seam_face_sample(
                &mut diagnostics.retaining_wall_face_samples,
                sample.clone(),
            );
        }

        diagnostics.road_seam_faces += 1;
        diagnostics.road_seam_max_y_delta_m = diagnostics
            .road_seam_max_y_delta_m
            .max(metrics.max_y_delta_m);
        diagnostics.road_seam_max_slope_ratio = diagnostics
            .road_seam_max_slope_ratio
            .max(metrics.max_slope_ratio);
        insert_road_seam_face_sample(&mut diagnostics.road_seam_face_samples, sample);
    }

    diagnostics
}

fn terrain_triangle_road_sources(
    triangle: &[usize; 3],
    road_constraint_sources: &BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
    road_constraint_vertices: &[RoadConstraintVertex],
) -> Vec<TerrainCdtRoadBoundarySource> {
    if !triangle_edges(triangle).iter().any(|edge| {
        road_constraint_vertices[edge.0].boundary && road_constraint_vertices[edge.1].boundary
    }) {
        return Vec::new();
    }
    let mut sources = triangle_edges(triangle)
        .iter()
        .filter_map(|edge| {
            road_constraint_sources
                .get(&[edge.0, edge.1])
                .map(|source| source.boundary_source)
        })
        .collect::<Vec<_>>();
    sort_dedup_terrain_cdt_boundary_sources(&mut sources);
    sources
}

fn source_owned_kerb_return(
    points: [TerrainCdtVertex; 3],
    materials: &RoadBoundaryMaterials,
) -> bool {
    if materials.incompatible
        || materials.node.is_none()
        || materials.span.is_some() && !materials.span_handoff
        || (materials.owners[1..].iter().all(|&present| !present)
            && materials.span_owners[1..].iter().all(|&present| !present))
    {
        return false;
    }
    let step = crate::simulation::network::surface::CURB_STEP_HEIGHT_M;
    for i in 0..3 {
        let j = (i + 1) % 3;
        if edge_length_xz_m(points[i], points[j]) > CDT_EPSILON_M {
            continue;
        }
        let (low, high) = if points[i].height_m < points[j].height_m {
            (i, j)
        } else {
            (j, i)
        };
        if !same_height(points[high].height_m - points[low].height_m, step) {
            continue;
        }
        // Prove that removing the declared material step leaves a valid ground grade.
        // The third vertex may belong to either side of the same kerb return.
        let other = 3 - low - high;
        for other_raised in [false, true] {
            let mut base = points.map(|mut point| {
                point.height_m =
                    (quantized_coord(f64::from(point.height_m)) as f64 * CDT_EPSILON_M) as f32;
                point
            });
            let lower_material = |height: f32| {
                ((quantized_coord(f64::from(height)) - quantized_coord(f64::from(step))) as f64
                    * CDT_EPSILON_M) as f32
            };
            // The source contract is in millimetre keys. Subtracting 0.12 directly in
            // f32 can leave a full ULP across a micrometre-wide kerb corner at 100 m elevation.
            base[high].height_m = lower_material(base[high].height_m);
            if other_raised {
                base[other].height_m = lower_material(base[other].height_m);
            }
            if terrain_face_plane_slope_ratio(base) <= MAX_TERRAIN_TIE_IN_SLOPE_RATIO {
                return true;
            }
        }
    }
    false
}

fn classify_terrain_tie_in_face(
    sources: &[TerrainCdtRoadBoundarySource],
    retaining_wall_required_sources: &[TerrainCdtRoadBoundarySource],
    max_slope_ratio: f32,
) -> TerrainCdtTieInKind {
    if sources.is_empty() {
        return TerrainCdtTieInKind::OrdinaryTerrain;
    }
    if !terrain_cdt_sources_allow_retaining_wall(sources) {
        return TerrainCdtTieInKind::OrdinaryTerrain;
    }
    if terrain_sources_include_retaining_wall_required_source(
        sources,
        retaining_wall_required_sources,
    ) {
        return TerrainCdtTieInKind::RetainingWall;
    }
    if max_slope_ratio > MAX_TERRAIN_TIE_IN_SLOPE_RATIO {
        TerrainCdtTieInKind::RetainingWall
    } else {
        TerrainCdtTieInKind::OrdinaryTerrain
    }
}

fn terrain_cdt_sources_allow_retaining_wall(sources: &[TerrainCdtRoadBoundarySource]) -> bool {
    sources
        .iter()
        .copied()
        .any(terrain_cdt_boundary_source_allows_retaining_wall)
}

pub(super) fn terrain_cdt_boundary_source_allows_retaining_wall(
    source: TerrainCdtRoadBoundarySource,
) -> bool {
    match source {
        TerrainCdtRoadBoundarySource::SpanSupportBoundary {
            edge_class,
            support_policy,
            ..
        } => {
            edge_class != TerrainCdtEdgeClass::Standard
                || support_policy != TerrainCdtEarthworkSupportPolicy::StandardFullGroundedSpan
        }
        TerrainCdtRoadBoundarySource::NodeFootprintBoundary { .. }
        | TerrainCdtRoadBoundarySource::NodeSameMaterialBoundaryHandoff { .. } => false,
        TerrainCdtRoadBoundarySource::BuildingSiteBoundary { .. } => false,
        TerrainCdtRoadBoundarySource::SyntheticTestBoundary { .. } => true,
    }
}

pub(super) fn terrain_cdt_boundary_source_requires_widened_sample_retaining_wall(
    source: TerrainCdtRoadBoundarySource,
) -> bool {
    match source {
        TerrainCdtRoadBoundarySource::SpanSupportBoundary {
            edge_class: TerrainCdtEdgeClass::Tunnel,
            support_policy: TerrainCdtEarthworkSupportPolicy::TunnelVisiblePortals,
            ..
        }
        | TerrainCdtRoadBoundarySource::SyntheticTestBoundary { .. } => true,
        TerrainCdtRoadBoundarySource::SpanSupportBoundary { .. }
        | TerrainCdtRoadBoundarySource::NodeFootprintBoundary { .. }
        | TerrainCdtRoadBoundarySource::NodeSameMaterialBoundaryHandoff { .. }
        | TerrainCdtRoadBoundarySource::BuildingSiteBoundary { .. } => false,
    }
}

fn terrain_sources_include_retaining_wall_required_source(
    sources: &[TerrainCdtRoadBoundarySource],
    retaining_wall_required_sources: &[TerrainCdtRoadBoundarySource],
) -> bool {
    sources.iter().copied().any(|source| {
        retaining_wall_required_sources
            .binary_search_by(|required_source| {
                terrain_cdt_boundary_source_cmp(*required_source, source)
            })
            .is_ok()
    })
}

pub(super) fn widening_tie_in_sample_against_any_road_loop(
    sample: TerrainCdtVertex,
    road_loops: &[CanonicalTerrainCdtRoadLoop],
) -> Option<TerrainCdtTieInSample> {
    let mut best: Option<TerrainCdtTieInSample> = None;
    for road_loop in road_loops {
        let Some(candidate) = widening_tie_in_sample(sample, road_loop) else {
            continue;
        };
        if best.as_ref().is_none_or(|current| {
            candidate
                .slope_ratio
                .total_cmp(&current.slope_ratio)
                .then_with(|| candidate.height_delta_m.total_cmp(&current.height_delta_m))
                .then_with(|| current.distance_m.total_cmp(&candidate.distance_m))
                .is_gt()
        }) {
            best = Some(candidate);
        }
    }
    best
}

pub(super) fn widening_tie_in_sample(
    sample: TerrainCdtVertex,
    road_loop: &CanonicalTerrainCdtRoadLoop,
) -> Option<TerrainCdtTieInSample> {
    let max_height_delta_m = (sample.height_m - road_loop.min_height_m)
        .abs()
        .max((sample.height_m - road_loop.max_height_m).abs());
    if max_height_delta_m <= MIN_TIE_IN_HEIGHT_DELTA_M {
        return None;
    }
    let bounds_dx = if sample.x < road_loop.min_x {
        road_loop.min_x - sample.x
    } else if sample.x > road_loop.max_x {
        sample.x - road_loop.max_x
    } else {
        0.0
    };
    let bounds_dz = if sample.z < road_loop.min_z {
        road_loop.min_z - sample.z
    } else if sample.z > road_loop.max_z {
        sample.z - road_loop.max_z
    } else {
        0.0
    };
    let maximum_required_distance_m =
        f64::from(max_height_delta_m / MAX_TERRAIN_TIE_IN_SLOPE_RATIO.max(f32::EPSILON));
    let maximum_candidate_distance_m = maximum_required_distance_m - CDT_EPSILON_M;
    if maximum_candidate_distance_m <= 0.0
        || bounds_dx * bounds_dx + bounds_dz * bounds_dz
            >= maximum_candidate_distance_m * maximum_candidate_distance_m
    {
        return None;
    }
    let (distance_m, seam_point, seam_source) =
        closest_sourced_loop_edge_distance_point_and_source(
            sample,
            road_loop,
            maximum_required_distance_m,
        )?;
    let height_delta_m = (sample.height_m - seam_point.height_m).abs();
    if height_delta_m <= MIN_TIE_IN_HEIGHT_DELTA_M {
        return None;
    }
    if distance_m <= CDT_EPSILON_M {
        return Some(TerrainCdtTieInSample {
            source_sample: sample,
            seam_point,
            seam_source,
            distance_m: 0.0,
            required_distance_m: height_delta_m / MAX_TERRAIN_TIE_IN_SLOPE_RATIO,
            height_delta_m,
            slope_ratio: f32::INFINITY,
        });
    }
    let distance_m_f32 = distance_m as f32;
    let slope_ratio = height_delta_m / distance_m_f32;
    let required_distance_m = height_delta_m / MAX_TERRAIN_TIE_IN_SLOPE_RATIO;
    (distance_m < f64::from(required_distance_m) - CDT_EPSILON_M).then_some(TerrainCdtTieInSample {
        source_sample: sample,
        seam_point,
        seam_source,
        distance_m: distance_m_f32,
        required_distance_m,
        height_delta_m,
        slope_ratio,
    })
}

fn closest_sourced_loop_edge_distance_point_and_source(
    point: TerrainCdtVertex,
    road_loop: &CanonicalTerrainCdtRoadLoop,
    maximum_distance_m: f64,
) -> Option<(f64, TerrainCdtVertex, TerrainCdtRoadBoundarySource)> {
    if road_loop.sourced_edges.is_empty() {
        return None;
    }

    let mut closest_distance_m = maximum_distance_m;
    let mut closest_point = TerrainCdtVertex::new(0.0, 0.0, 0.0);
    let mut closest_source = None;
    for edge in &road_loop.sourced_edges {
        let bounds_dx = if point.x < edge.min_x {
            edge.min_x - point.x
        } else if point.x > edge.max_x {
            point.x - edge.max_x
        } else {
            0.0
        };
        let bounds_dz = if point.z < edge.min_z {
            edge.min_z - point.z
        } else if point.z > edge.max_z {
            point.z - edge.max_z
        } else {
            0.0
        };
        let comparison_distance_m = closest_distance_m + CDT_EPSILON_M;
        if bounds_dx * bounds_dx + bounds_dz * bounds_dz
            > comparison_distance_m * comparison_distance_m
        {
            continue;
        }
        let t = if edge.length_squared_m <= CDT_EPSILON_M * CDT_EPSILON_M {
            0.0
        } else {
            (((point.x - edge.start_x) * edge.delta_x + (point.z - edge.start_z) * edge.delta_z)
                / edge.length_squared_m)
                .clamp(0.0, 1.0)
        };
        let closest_x = edge.start_x + edge.delta_x * t;
        let closest_z = edge.start_z + edge.delta_z * t;
        let dx = point.x - closest_x;
        let dz = point.z - closest_z;
        let distance_squared_m = dx * dx + dz * dz;
        if distance_squared_m > comparison_distance_m * comparison_distance_m {
            continue;
        }
        let distance_m = distance_squared_m.sqrt();
        let height_m = (f64::from(edge.start_height_m) + f64::from(edge.delta_height_m) * t) as f32;
        let candidate_point = TerrainCdtVertex::new(closest_x, height_m, closest_z);
        if terrain_cdt_closer_loop_point(
            distance_m,
            candidate_point,
            edge.source,
            closest_distance_m,
            closest_point,
            closest_source,
        ) {
            closest_distance_m = distance_m;
            closest_point = candidate_point;
            closest_source = Some(edge.source);
        }
    }

    Some((closest_distance_m, closest_point, closest_source?))
}

fn terrain_cdt_closer_loop_point(
    candidate_distance_m: f64,
    candidate_point: TerrainCdtVertex,
    candidate_source: TerrainCdtRoadBoundarySource,
    current_distance_m: f64,
    current_point: TerrainCdtVertex,
    current_source: Option<TerrainCdtRoadBoundarySource>,
) -> bool {
    if candidate_distance_m < current_distance_m - CDT_EPSILON_M {
        return true;
    }
    if (candidate_distance_m - current_distance_m).abs() > CDT_EPSILON_M {
        return false;
    }
    let geometry_ordering = terrain_cdt_vertex_geometry_cmp(candidate_point, current_point);
    if !geometry_ordering.is_eq() {
        return geometry_ordering.is_lt();
    }
    current_source
        .is_none_or(|source| terrain_cdt_boundary_source_cmp(candidate_source, source).is_lt())
}

fn terrain_cdt_vertex_geometry_cmp(a: TerrainCdtVertex, b: TerrainCdtVertex) -> std::cmp::Ordering {
    terrain_cdt_vertex_key(a).cmp(&terrain_cdt_vertex_key(b))
}

pub(super) fn triangle_edges(triangle: &[usize; 3]) -> [(usize, usize); 3] {
    [
        normalize_edge(triangle[0], triangle[1]),
        normalize_edge(triangle[1], triangle[2]),
        normalize_edge(triangle[2], triangle[0]),
    ]
}

fn terrain_face_sample(
    points: [TerrainCdtVertex; 3],
    kind: TerrainCdtTieInKind,
    sources: Vec<TerrainCdtRoadBoundarySource>,
    metrics: TerrainFaceMetrics,
) -> TerrainCdtFaceSample {
    let mut min_x = points[0].x;
    let mut min_z = points[0].z;
    let mut max_x = points[0].x;
    let mut max_z = points[0].z;
    let mut min_y_m = points[0].height_m;
    let mut max_y_m = points[0].height_m;

    for point in points {
        min_x = min_x.min(point.x);
        min_z = min_z.min(point.z);
        max_x = max_x.max(point.x);
        max_z = max_z.max(point.z);
        min_y_m = min_y_m.min(point.height_m);
        max_y_m = max_y_m.max(point.height_m);
    }

    TerrainCdtFaceSample {
        kind,
        vertices: points,
        centroid: centroid(points),
        sources,
        min_x,
        min_z,
        max_x,
        max_z,
        min_y_m,
        max_y_m,
        max_y_delta_m: metrics.max_y_delta_m,
        max_slope_ratio: metrics.max_slope_ratio,
    }
}

fn terrain_face_metrics(points: [TerrainCdtVertex; 3]) -> TerrainFaceMetrics {
    let min_height_m = points[0]
        .height_m
        .min(points[1].height_m)
        .min(points[2].height_m);
    let max_height_m = points[0]
        .height_m
        .max(points[1].height_m)
        .max(points[2].height_m);
    let edge_length_squared = |a: TerrainCdtVertex, b: TerrainCdtVertex| {
        let dx = b.x - a.x;
        let dz = b.z - a.z;
        dx * dx + dz * dz
    };
    let longest_edge_squared_m = edge_length_squared(points[0], points[1])
        .max(edge_length_squared(points[1], points[2]))
        .max(edge_length_squared(points[2], points[0]));
    TerrainFaceMetrics {
        max_y_delta_m: max_height_m - min_height_m,
        max_slope_ratio: terrain_face_plane_slope_ratio(points),
        longest_edge_m: longest_edge_squared_m.sqrt() as f32,
    }
}

fn terrain_face_plane_slope_ratio(points: [TerrainCdtVertex; 3]) -> f32 {
    let ax = points[1].x - points[0].x;
    let ay = f64::from(points[1].height_m - points[0].height_m);
    let az = points[1].z - points[0].z;
    let bx = points[2].x - points[0].x;
    let by = f64::from(points[2].height_m - points[0].height_m);
    let bz = points[2].z - points[0].z;

    let normal_x = ay * bz - az * by;
    let normal_y = az * bx - ax * bz;
    let normal_z = ax * by - ay * bx;
    let horizontal_normal = (normal_x * normal_x + normal_z * normal_z).sqrt();
    if horizontal_normal <= CDT_EPSILON_M * CDT_EPSILON_M {
        return 0.0;
    }
    if normal_y.abs() <= CDT_EPSILON_M * CDT_EPSILON_M {
        return 1_000_000.0;
    }
    (horizontal_normal / normal_y.abs()) as f32
}

fn insert_road_seam_face_sample(
    samples: &mut Vec<TerrainCdtFaceSample>,
    sample: TerrainCdtFaceSample,
) {
    samples.push(sample);
    samples.sort_by(|a, b| {
        b.max_slope_ratio
            .total_cmp(&a.max_slope_ratio)
            .then_with(|| b.max_y_delta_m.total_cmp(&a.max_y_delta_m))
            .then_with(|| terrain_cdt_boundary_sources_cmp(&a.sources, &b.sources))
            .then_with(|| a.centroid.x.total_cmp(&b.centroid.x))
            .then_with(|| a.centroid.z.total_cmp(&b.centroid.z))
    });
    samples.truncate(MAX_ROAD_SEAM_FACE_SAMPLES);
}

pub(super) fn insert_tie_in_widened_sample(
    samples: &mut Vec<TerrainCdtTieInSample>,
    sample: TerrainCdtTieInSample,
) {
    samples.push(sample);
    samples.sort_by(|a, b| {
        b.slope_ratio
            .total_cmp(&a.slope_ratio)
            .then_with(|| b.height_delta_m.total_cmp(&a.height_delta_m))
            .then_with(|| terrain_cdt_boundary_source_cmp(a.seam_source, b.seam_source))
            .then_with(|| a.source_sample.x.total_cmp(&b.source_sample.x))
            .then_with(|| a.source_sample.z.total_cmp(&b.source_sample.z))
    });
    samples.truncate(MAX_TIE_IN_SAMPLE_DIAGNOSTICS);
}

pub(super) fn insert_invalid_constraint_sample(
    samples: &mut Vec<TerrainCdtInvalidConstraintSample>,
    edge: [usize; 2],
    vertices: &[TerrainCdtVertex],
    road_constraint_sources: &BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
) {
    let Some(&start) = vertices.get(edge[0]) else {
        return;
    };
    let Some(&end) = vertices.get(edge[1]) else {
        return;
    };
    let source = road_constraint_sources.get(&edge);
    samples.push(TerrainCdtInvalidConstraintSample {
        start,
        end,
        road_owned: source.is_some(),
        stable_piece_id: source.map_or(0, |source| source.stable_piece_id),
        local_loop_index: source.map_or(u32::MAX, |source| source.local_loop_index),
        local_edge_index: source.map_or(u32::MAX, |source| source.local_edge_index),
        source: source.map(|source| source.boundary_source),
    });
    samples.sort_by(|a, b| {
        b.road_owned
            .cmp(&a.road_owned)
            .then_with(|| a.stable_piece_id.cmp(&b.stable_piece_id))
            .then_with(|| a.local_loop_index.cmp(&b.local_loop_index))
            .then_with(|| a.local_edge_index.cmp(&b.local_edge_index))
            .then_with(|| terrain_cdt_optional_boundary_source_cmp(a.source, b.source))
            .then_with(|| a.start.x.total_cmp(&b.start.x))
            .then_with(|| a.start.z.total_cmp(&b.start.z))
            .then_with(|| a.end.x.total_cmp(&b.end.x))
            .then_with(|| a.end.z.total_cmp(&b.end.z))
    });
    samples.truncate(MAX_INVALID_CONSTRAINT_SAMPLES);
}

pub(super) fn unpreserved_road_constraint_samples(
    road_constraint_edges: &[[usize; 2]],
    accepted_edges: &HashSet<(usize, usize)>,
    vertices: &[TerrainCdtVertex],
    road_constraint_sources: &BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
) -> Vec<TerrainCdtInvalidConstraintSample> {
    let mut samples = Vec::new();
    for edge in road_constraint_edges {
        if accepted_edges.contains(&normalize_edge(edge[0], edge[1])) {
            continue;
        }
        insert_invalid_constraint_sample(
            &mut samples,
            normalize_edge_array(edge[0], edge[1]),
            vertices,
            road_constraint_sources,
        );
    }
    samples
}

pub(super) fn sort_dedup_terrain_cdt_boundary_sources(
    sources: &mut Vec<TerrainCdtRoadBoundarySource>,
) {
    sources.sort_by(|a, b| terrain_cdt_boundary_source_cmp(*a, *b));
    sources.dedup_by(|a, b| terrain_cdt_boundary_source_cmp(*a, *b).is_eq());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kerb_returns_require_one_source_node_and_the_declared_step() {
        let points = [
            TerrainCdtVertex::new(0.0, 20.0, 0.0),
            TerrainCdtVertex::new(0.0004, 20.12, 0.0),
            TerrainCdtVertex::new(0.0, 20.125, 3.5),
        ];
        let source = TerrainCdtRoadBoundarySource::NodeFootprintBoundary {
            node_id: 3,
            node_kind: TerrainCdtNodePieceKind::JunctionN,
            owner_kind: TerrainCdtRoadBandKind::CurbOrShoulder,
            owner_index: 4,
            boundary_source: Some(TerrainCdtNodeFootprintBoundarySegmentSource {
                start: TerrainCdtNodeFootprintBoundaryVertexSource::CanonicalBoundaryPoint {
                    x_key: 0,
                    z_key: 0,
                    y_mm: 20_000,
                },
                end: TerrainCdtNodeFootprintBoundaryVertexSource::CanonicalBoundaryPoint {
                    x_key: 400,
                    z_key: 0,
                    y_mm: 20_120,
                },
            }),
        };
        let materials = |sources: &[TerrainCdtRoadBoundarySource]| {
            let mut materials = RoadBoundaryMaterials::default();
            for &source in sources {
                materials.add_source(source);
            }
            materials
        };
        assert!(source_owned_kerb_return(points, &materials(&[source])));
        let elevated = [
            TerrainCdtVertex::new(-666.099032, 102.288, 1666.876382),
            TerrainCdtVertex::new(-666.099033, 102.408, 1666.876382),
            TerrainCdtVertex::new(-665.385927, 102.219, 1669.740489),
        ];
        assert!(
            source_owned_kerb_return(elevated, &materials(&[source])),
            "material-step proof must be invariant under the source elevation"
        );
        let span = |edge_idx, source_band_index, handoff| {
            TerrainCdtRoadBoundarySource::SpanSupportBoundary {
                edge_idx,
                edge_class: TerrainCdtEdgeClass::Standard,
                support_policy: TerrainCdtEarthworkSupportPolicy::StandardFullGroundedSpan,
                source_band_index,
                band_kind: TerrainCdtRoadBandKind::Carriageway,
                role: TerrainCdtSpanRegionRole::Asphalt,
                start_section_index: 5,
                end_section_index: if handoff { 5 } else { 6 },
                start_s_m: 11.0,
                end_s_m: if handoff { 11.0 } else { 12.0 },
            }
        };
        assert!(source_owned_kerb_return(
            elevated,
            &materials(&[source, span(929, 2, true), span(929, 3, true)])
        ));
        assert!(!source_owned_kerb_return(
            elevated,
            &materials(&[source, span(929, 2, false)])
        ));
        assert!(!source_owned_kerb_return(
            elevated,
            &materials(&[source, span(929, 2, true), span(930, 2, true)])
        ));
        let mut adjacent_owner = source;
        if let TerrainCdtRoadBoundarySource::NodeFootprintBoundary { owner_index, .. } =
            &mut adjacent_owner
        {
            *owner_index += 1;
        }
        assert!(source_owned_kerb_return(
            points,
            &materials(&[source, adjacent_owner]),
        ));
        // Exact source corners from the four imported-map tiles that exposed this.
        for points in [
            [
                TerrainCdtVertex::new(-1044.675267, 99.575, 2134.690785),
                TerrainCdtVertex::new(-1044.675360, 99.455, 2134.690743),
                TerrainCdtVertex::new(-1044.674984, 99.575, 2134.690173),
            ],
            [
                TerrainCdtVertex::new(-836.277558, 95.774, 2734.670418),
                TerrainCdtVertex::new(-836.277557, 95.654, 2734.670417),
                TerrainCdtVertex::new(-836.253684, 95.774, 2734.694337),
            ],
            [
                TerrainCdtVertex::new(898.785735, 105.121, 1286.329947),
                TerrainCdtVertex::new(898.791567, 105.241, 1286.374503),
                TerrainCdtVertex::new(898.785734, 105.241, 1286.329946),
            ],
            [
                TerrainCdtVertex::new(-71.956634, 94.768, 1137.966879),
                TerrainCdtVertex::new(-71.945855, 94.768, 1137.978493),
                TerrainCdtVertex::new(-71.956635, 94.648, 1137.966880),
            ],
        ] {
            assert!(source_owned_kerb_return(
                points,
                &materials(&[source, adjacent_owner]),
            ));
        }
        let mut cliff = points;
        cliff[1].height_m += 0.5;
        assert!(!source_owned_kerb_return(cliff, &materials(&[source])));
        let mut gap = points;
        gap[1].x = 0.002;
        assert!(!source_owned_kerb_return(gap, &materials(&[source])));
        let mut unrelated = source;
        if let TerrainCdtRoadBoundarySource::NodeFootprintBoundary { node_id, .. } = &mut unrelated
        {
            *node_id += 1;
        }
        assert!(!source_owned_kerb_return(
            points,
            &materials(&[source, unrelated])
        ));
        let mut unsourced = source;
        if let TerrainCdtRoadBoundarySource::NodeFootprintBoundary {
            boundary_source, ..
        } = &mut unsourced
        {
            *boundary_source = None;
        }
        assert!(!source_owned_kerb_return(points, &materials(&[unsourced])));
    }
}
