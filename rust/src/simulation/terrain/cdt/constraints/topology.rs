// SPDX-License-Identifier: GPL-2.0-only

//! Patch rails, constraint noding, and deterministic segment geometry.

use rstar::primitives::{GeomWithData, Line};
use rstar::{AABB, RTree};
use std::collections::{BTreeMap, BTreeSet, HashMap};

use super::super::*;

/// Existing tile-local constraint index, shared by guide noding and boundary exposure.
pub(in crate::simulation::terrain::cdt) type TerrainCdtRoadConstraintIndex =
    RTree<GeomWithData<Line<[f64; 2]>, usize>>;

pub(in crate::simulation::terrain::cdt) fn push_patch_boundary_constraints(
    patch: TerrainCdtPatch,
    vertices: &[TerrainCdtVertex],
    constraint_set: &mut BTreeSet<[usize; 2]>,
) {
    let mut left = Vec::new();
    let mut top = Vec::new();
    let mut right = Vec::new();
    let mut bottom = Vec::new();

    for (index, vertex) in vertices.iter().copied().enumerate() {
        if same_coord(vertex.x, patch.min_x)
            && vertex.z >= patch.min_z - CDT_EPSILON_M
            && vertex.z <= patch.max_z + CDT_EPSILON_M
        {
            left.push((quantized_xz_coord(vertex.z), index));
        }
        if same_coord(vertex.z, patch.max_z)
            && vertex.x >= patch.min_x - CDT_EPSILON_M
            && vertex.x <= patch.max_x + CDT_EPSILON_M
        {
            top.push((quantized_xz_coord(vertex.x), index));
        }
        if same_coord(vertex.x, patch.max_x)
            && vertex.z >= patch.min_z - CDT_EPSILON_M
            && vertex.z <= patch.max_z + CDT_EPSILON_M
        {
            right.push((-quantized_xz_coord(vertex.z), index));
        }
        if same_coord(vertex.z, patch.min_z)
            && vertex.x >= patch.min_x - CDT_EPSILON_M
            && vertex.x <= patch.max_x + CDT_EPSILON_M
        {
            bottom.push((-quantized_xz_coord(vertex.x), index));
        }
    }

    push_sorted_boundary_side(&mut left, constraint_set);
    push_sorted_boundary_side(&mut top, constraint_set);
    push_sorted_boundary_side(&mut right, constraint_set);
    push_sorted_boundary_side(&mut bottom, constraint_set);
}

fn push_sorted_boundary_side(
    side: &mut Vec<(i64, usize)>,
    constraint_set: &mut BTreeSet<[usize; 2]>,
) {
    side.sort_unstable();
    side.dedup_by_key(|(_, index)| *index);
    for pair in side.windows(2) {
        insert_constraint([pair[0].1, pair[1].1], constraint_set);
    }
}

pub(in crate::simulation::terrain::cdt) fn insert_constraint(
    edge: [usize; 2],
    constraint_set: &mut BTreeSet<[usize; 2]>,
) {
    let edge = normalize_edge_array(edge[0], edge[1]);
    if edge[0] != edge[1] {
        constraint_set.insert(edge);
    }
}

// Spade accepts a constrained graph but does not node crossing or T-touching
// constraints for us. i_overlay owns roadbed area union; this patch-local pass
// only canonicalizes the final CDT constraint graph. Determinism comes from
// sorted original road loops, quantized XZ vertex lookup, and BTreeSet edge
// emission. Edge/edge noding is O(E^2) with bbox rejection; source/guide incidence
// uses the bounded edge index below. Both operate on one dirty terrain tile,
// outside the per-tick simulation hot path.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TerrainCdtRoadConstraintSplit {
    t: f64,
    vertex_index: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TerrainCdtSourceSampleConstraintHit {
    edge_index: usize,
    t: f64,
    height_m: f32,
}

pub(in crate::simulation::terrain::cdt) fn node_road_constraint_edges(
    vertices: &mut Vec<TerrainCdtVertex>,
    vertex_lookup: &mut HashMap<(i64, i64), usize>,
    patch: TerrainCdtPatch,
    source_sample_vertex_indices: &[usize],
    road_constraint_edges: &mut Vec<[usize; 2]>,
    road_constraint_sources: &mut BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
) -> usize {
    if road_constraint_edges.len() < 2 {
        return 0;
    }

    let original_edges = road_constraint_edges.clone();
    let original_sources = road_constraint_sources.clone();
    let mut constraint_vertex_site_owned_only =
        terrain_cdt_constraint_vertex_site_ownership(&original_edges, &original_sources);
    let mut invalid_constraint_edges = 0usize;
    let mut split_points = original_edges
        .iter()
        .map(|edge| {
            vec![
                TerrainCdtRoadConstraintSplit {
                    t: 0.0,
                    vertex_index: edge[0],
                },
                TerrainCdtRoadConstraintSplit {
                    t: 1.0,
                    vertex_index: edge[1],
                },
            ]
        })
        .collect::<Vec<_>>();

    for first_index in 0..original_edges.len() {
        for second_index in first_index + 1..original_edges.len() {
            let first_edge = original_edges[first_index];
            let second_edge = original_edges[second_index];
            if first_edge == second_edge {
                continue;
            }
            let first_start = vertices[first_edge[0]];
            let first_end = vertices[first_edge[1]];
            let second_start = vertices[second_edge[0]];
            let second_end = vertices[second_edge[1]];
            if !segment_bounds_overlap(first_start, first_end, second_start, second_end) {
                continue;
            }

            for intersection in
                segment_intersections(first_start, first_end, second_start, second_end)
                    .into_iter()
                    .flatten()
            {
                let first_t =
                    segment_parameter(first_start, first_end, intersection.x, intersection.z);
                let second_t =
                    segment_parameter(second_start, second_end, intersection.x, intersection.z);
                if !unit_interval_contains(first_t) || !unit_interval_contains(second_t) {
                    continue;
                }
                let first_height =
                    interpolated_segment_height(first_start, first_end, clamp_unit(first_t));
                let second_height =
                    interpolated_segment_height(second_start, second_end, clamp_unit(second_t));
                let first_source = original_sources
                    .get(&first_edge)
                    .map(|source| source.boundary_source);
                let second_source = original_sources
                    .get(&second_edge)
                    .map(|source| source.boundary_source);
                let Some(intersection_height) = shared_road_constraint_height_for_sources(
                    first_height,
                    second_height,
                    first_source,
                    second_source,
                ) else {
                    crate::debug_log!(
                        "road",
                        "terrain_constraint_height_conflict point={intersection:?} first=({first_start:?},{first_end:?},{first_source:?}) second=({second_start:?},{second_end:?},{second_source:?})"
                    );
                    invalid_constraint_edges += 1;
                    continue;
                };
                let site_owned_only = first_source
                    .is_some_and(terrain_cdt_boundary_source_is_building_site)
                    && second_source.is_some_and(terrain_cdt_boundary_source_is_building_site);
                let Some(vertex_index) = insert_road_constraint_vertex(
                    TerrainCdtVertex::new(intersection.x, intersection_height, intersection.z),
                    site_owned_only,
                    vertices,
                    vertex_lookup,
                    &mut constraint_vertex_site_owned_only,
                ) else {
                    crate::debug_log!(
                        "road",
                        "terrain_constraint_vertex_conflict point={intersection:?} height={intersection_height} first=({first_start:?},{first_end:?},{first_source:?}) second=({second_start:?},{second_end:?},{second_source:?})"
                    );
                    invalid_constraint_edges += 1;
                    continue;
                };
                split_points[first_index].push(TerrainCdtRoadConstraintSplit {
                    t: clamp_unit(first_t),
                    vertex_index,
                });
                split_points[second_index].push(TerrainCdtRoadConstraintSplit {
                    t: clamp_unit(second_t),
                    vertex_index,
                });
            }
        }
    }

    split_road_constraints_at_source_samples(
        &original_edges,
        vertices,
        source_sample_vertex_indices,
        &mut split_points,
    );

    let mut noded_edges = BTreeSet::new();
    road_constraint_sources.clear();

    for (edge, splits) in original_edges.iter().copied().zip(split_points.iter_mut()) {
        sort_dedup_constraint_splits(splits);
        let source = original_sources.get(&edge).copied();
        for pair in splits.windows(2) {
            let noded_edge = normalize_edge_array(pair[0].vertex_index, pair[1].vertex_index);
            if noded_edge[0] == noded_edge[1]
                || edge_lies_on_patch_boundary(
                    vertices[noded_edge[0]],
                    vertices[noded_edge[1]],
                    patch,
                )
            {
                continue;
            }
            noded_edges.insert(noded_edge);
            if let Some(source) = source {
                road_constraint_sources
                    .entry(noded_edge)
                    .and_modify(|existing| {
                        if terrain_cdt_road_constraint_source_cmp(source, *existing).is_lt() {
                            *existing = source;
                        }
                    })
                    .or_insert(source);
            }
        }
    }

    *road_constraint_edges = noded_edges.into_iter().collect();
    invalid_constraint_edges
}

fn terrain_cdt_constraint_vertex_site_ownership(
    edges: &[[usize; 2]],
    sources: &BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
) -> BTreeMap<usize, bool> {
    let mut site_owned_only_by_vertex = BTreeMap::new();
    for edge in edges {
        let site_owned_edge = sources
            .get(edge)
            .map(|source| source.boundary_source)
            .is_some_and(terrain_cdt_boundary_source_is_building_site);
        for vertex_index in edge {
            site_owned_only_by_vertex
                .entry(*vertex_index)
                .and_modify(|site_owned_only| *site_owned_only &= site_owned_edge)
                .or_insert(site_owned_edge);
        }
    }
    site_owned_only_by_vertex
}

pub(in crate::simulation::terrain::cdt) fn retain_exposed_road_constraint_edges(
    vertices: &[TerrainCdtVertex],
    road_loops: &[CanonicalTerrainCdtRoadLoop],
    road_constraint_edges: &mut Vec<[usize; 2]>,
    road_constraint_sources: &mut BTreeMap<[usize; 2], TerrainCdtRoadConstraintSource>,
) -> usize {
    let mut internal_edges = Vec::new();
    road_constraint_edges.retain(|edge| {
        let exposed =
            road_constraint_edge_exposes_terrain(vertices[edge[0]], vertices[edge[1]], road_loops);
        if !exposed {
            internal_edges.push(*edge);
        }
        exposed
    });
    for edge in &internal_edges {
        road_constraint_sources.remove(edge);
    }
    internal_edges.len()
}

fn road_constraint_edge_exposes_terrain(
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
    road_loops: &[CanonicalTerrainCdtRoadLoop],
) -> bool {
    let dx = end.x - start.x;
    let dz = end.z - start.z;
    let length = dx.hypot(dz);
    if length <= CDT_EPSILON_M {
        return false;
    }
    let mid_x = (start.x + end.x) * 0.5;
    let mid_z = (start.z + end.z) * 0.5;
    let probe_distance = MIN_SOURCE_OWNED_SEAM_EDGE_LENGTH_M.min(length * 0.25);
    let nx = -dz / length * probe_distance;
    let nz = dx / length * probe_distance;
    let height_m = (start.height_m + end.height_m) * 0.5;
    let left_probe = TerrainCdtVertex::new(mid_x + nx, height_m, mid_z + nz);
    let right_probe = TerrainCdtVertex::new(mid_x - nx, height_m, mid_z - nz);
    road_exterior_support_point(left_probe, road_loops)
        || road_exterior_support_point(right_probe, road_loops)
}

fn split_road_constraints_at_source_samples(
    original_edges: &[[usize; 2]],
    vertices: &mut [TerrainCdtVertex],
    source_sample_vertex_indices: &[usize],
    split_points: &mut [Vec<TerrainCdtRoadConstraintSplit>],
) -> TerrainCdtRoadConstraintIndex {
    // Guides and DEM samples share this path. Query only incident road constraints:
    // O(E log E + S log E + sum(H log H)) for H local hits per sample, instead of S*E.
    let index = RTree::bulk_load(
        original_edges
            .iter()
            .enumerate()
            .map(|(id, edge)| {
                let start = vertices[edge[0]];
                let end = vertices[edge[1]];
                GeomWithData::new(Line::new([start.x, start.z], [end.x, end.z]), id)
            })
            .collect(),
    );
    let mut hits = Vec::new();
    for &vertex_index in source_sample_vertex_indices {
        let Some(vertex) = vertices.get(vertex_index).copied() else {
            continue;
        };
        hits.clear();
        let bounds = AABB::from_corners(
            [vertex.x - CDT_EPSILON_M, vertex.z - CDT_EPSILON_M],
            [vertex.x + CDT_EPSILON_M, vertex.z + CDT_EPSILON_M],
        );
        for candidate in index.locate_in_envelope_intersecting(&bounds) {
            let edge_index = candidate.data;
            let edge = original_edges[edge_index];
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            let start = vertices[edge[0]];
            let end = vertices[edge[1]];
            if !point_bounds_overlap_segment(vertex, start, end) {
                continue;
            }
            let Some(t) = source_sample_parameter_near_road_constraint(start, end, vertex) else {
                continue;
            };
            hits.push(TerrainCdtSourceSampleConstraintHit {
                edge_index,
                t,
                height_m: interpolated_segment_height(start, end, t),
            });
        }

        if hits.is_empty() {
            continue;
        }
        // Preserve the exhaustive path's height authority and split order exactly.
        hits.sort_unstable_by_key(|hit| hit.edge_index);
        let height_m = hits[0].height_m;
        if !hits.iter().all(|hit| same_height(hit.height_m, height_m)) {
            continue;
        }
        vertices[vertex_index].height_m = height_m;
        for hit in &hits {
            split_points[hit.edge_index].push(TerrainCdtRoadConstraintSplit {
                t: hit.t,
                vertex_index,
            });
        }
    }
    index
}

// DEM/guide samples have the existing millimetre incidence budget. Authoritative
// road vertices use exact topology identity below; the two contracts are distinct.
fn source_sample_parameter_near_road_constraint(
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
    sample: TerrainCdtVertex,
) -> Option<f64> {
    if !point_bounds_overlap_segment(sample, start, end) {
        return None;
    }
    let t = segment_parameter(start, end, sample.x, sample.z);
    if !unit_interval_contains(t) {
        return None;
    }
    let t = clamp_unit(t);
    let closest = interpolate_vertex(start, end, t);
    (quantized_coord(closest.x) == quantized_coord(sample.x)
        && quantized_coord(closest.z) == quantized_coord(sample.z))
    .then_some(t)
}

pub(in crate::simulation::terrain::cdt) fn source_sample_parameter_on_road_constraint(
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
    sample: TerrainCdtVertex,
) -> Option<f64> {
    if !point_bounds_overlap_segment(sample, start, end) {
        return None;
    }
    if same_xz(sample, start) {
        return Some(0.0);
    }
    if same_xz(sample, end) {
        return Some(1.0);
    }
    let t = segment_parameter(start, end, sample.x, sample.z);
    if !unit_interval_contains(t) {
        return None;
    }
    let t = clamp_unit(t);
    let closest = interpolate_vertex(start, end, t);
    same_xz(closest, sample).then_some(t)
}

pub(in crate::simulation::terrain::cdt) fn point_bounds_overlap_segment(
    point: TerrainCdtVertex,
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
) -> bool {
    point.x >= start.x.min(end.x) - CDT_EPSILON_M
        && point.x <= start.x.max(end.x) + CDT_EPSILON_M
        && point.z >= start.z.min(end.z) - CDT_EPSILON_M
        && point.z <= start.z.max(end.z) + CDT_EPSILON_M
}

fn insert_road_constraint_vertex(
    vertex: TerrainCdtVertex,
    site_owned_only: bool,
    vertices: &mut Vec<TerrainCdtVertex>,
    vertex_lookup: &mut HashMap<(i64, i64), usize>,
    constraint_vertex_site_owned_only: &mut BTreeMap<usize, bool>,
) -> Option<usize> {
    let key = terrain_cdt_vertex_xz_key(vertex);
    if let Some(index) = vertex_lookup.get(&key) {
        let existing_site_owned_only = constraint_vertex_site_owned_only.get(index).copied();
        let Some(existing_site_owned_only) = existing_site_owned_only else {
            vertices[*index].height_m = vertex.height_m;
            constraint_vertex_site_owned_only.insert(*index, site_owned_only);
            return Some(*index);
        };
        let merged = merge_road_vertex_height(
            TerrainCdtRoadVertexHeight {
                height_m: vertices[*index].height_m,
                site_owned_only: existing_site_owned_only,
            },
            TerrainCdtRoadVertexHeight {
                height_m: vertex.height_m,
                site_owned_only,
            },
        )?;
        vertices[*index].height_m = merged.height_m;
        constraint_vertex_site_owned_only.insert(*index, merged.site_owned_only);
        return Some(*index);
    }
    let index = vertices.len();
    vertices.push(vertex);
    vertex_lookup.insert(key, index);
    constraint_vertex_site_owned_only.insert(index, site_owned_only);
    Some(index)
}

fn shared_road_constraint_height_for_sources(
    first_height_m: f32,
    second_height_m: f32,
    first_source: Option<TerrainCdtRoadBoundarySource>,
    second_source: Option<TerrainCdtRoadBoundarySource>,
) -> Option<f32> {
    if let Some(height_m) = shared_road_constraint_height(first_height_m, second_height_m) {
        return Some(height_m);
    }
    match (
        first_source.is_some_and(terrain_cdt_boundary_source_is_building_site),
        second_source.is_some_and(terrain_cdt_boundary_source_is_building_site),
    ) {
        (true, false) => Some(second_height_m),
        (false, true) => Some(first_height_m),
        _ => None,
    }
}

fn terrain_cdt_boundary_source_is_building_site(source: TerrainCdtRoadBoundarySource) -> bool {
    matches!(
        source,
        TerrainCdtRoadBoundarySource::BuildingSiteBoundary { .. }
    )
}

fn sort_dedup_constraint_splits(splits: &mut Vec<TerrainCdtRoadConstraintSplit>) {
    splits.sort_by(|a, b| {
        a.t.total_cmp(&b.t)
            .then_with(|| a.vertex_index.cmp(&b.vertex_index))
    });
    // t is dimensionless, not metres. Even metrically close intersections can occupy
    // different canonical vertices; dropping one leaves its other incident edge
    // crossing this constraint. Vertex insertion already owns identity merging.
    splits.dedup_by_key(|split| split.vertex_index);
}

// Bounds f64 subtraction, projection and interpolation construction error at this
// coordinate magnitude. This is an arithmetic error bound, not a metric overlap buffer.
fn coordinate_construction_roundoff_m(points: &[TerrainCdtVertex]) -> f64 {
    points
        .iter()
        .map(|p| p.x.abs().max(p.z.abs()))
        .fold(1.0_f64, f64::max)
        * f64::EPSILON
        * 16.0
}

pub(in crate::simulation::terrain::cdt) fn segment_intersections(
    first_start: TerrainCdtVertex,
    first_end: TerrainCdtVertex,
    second_start: TerrainCdtVertex,
    second_end: TerrainCdtVertex,
) -> [Option<TerrainCdtVertex>; 2] {
    if !segment_bounds_overlap(first_start, first_end, second_start, second_end) {
        return [None, None];
    }
    let key = |p: TerrainCdtVertex| {
        (
            i128::from(quantized_xz_coord(p.x)),
            i128::from(quantized_xz_coord(p.z)),
        )
    };
    let points = [first_start, first_end, second_start, second_end];
    let roundoff = coordinate_construction_roundoff_m(&points);
    let off_grid = points.iter().any(|p| {
        (p.x - quantized_xz_coord(p.x) as f64 * CDT_XZ_IDENTITY_GRID_M).abs() > roundoff
            || (p.z - quantized_xz_coord(p.z) as f64 * CDT_XZ_IDENTITY_GRID_M).abs() > roundoff
    });
    if off_grid {
        let ux = first_end.x - first_start.x;
        let uz = first_end.z - first_start.z;
        let vx = second_end.x - second_start.x;
        let vz = second_end.z - second_start.z;
        let length = ux.hypot(uz);
        // Generated clip/interpolation points can lie between source-grid keys. Keep
        // their true straight carrier when its residual is only construction roundoff.
        if length > 0.0
            && cross_xz(ux, uz, vx, vz).abs() <= roundoff * (length + vx.hypot(vz))
            && cross_xz(
                second_start.x - first_start.x,
                second_start.z - first_start.z,
                ux,
                uz,
            )
            .abs()
                <= roundoff * length
        {
            let a = segment_parameter(first_start, first_end, second_start.x, second_start.z);
            let b = segment_parameter(first_start, first_end, second_end.x, second_end.z);
            let start = a.min(b).max(0.0);
            let end = a.max(b).min(1.0);
            if start > end {
                return [None, None];
            }
            let mut first = interpolate_vertex(first_start, first_end, start);
            let mut second = interpolate_vertex(first_start, first_end, end);
            first.height_m = 0.0;
            second.height_m = 0.0;
            return [Some(first), (!same_xz(first, second)).then_some(second)];
        }
    }
    let (ax, az) = key(first_start);
    let (bx, bz) = key(first_end);
    let (cx, cz) = key(second_start);
    let (dx, dz) = key(second_end);
    let (ux, uz, vx, vz) = (bx - ax, bz - az, dx - cx, dz - cz);
    let length_squared = ux * ux + uz * uz;
    if length_squared == 0 || vx == 0 && vz == 0 {
        return [None, None];
    }
    let cross = ux * vz - uz * vx;
    let (wx, wz) = (cx - ax, cz - az);
    let interpolate = |numerator: i128, denominator: i128| {
        let t = numerator as f64 / denominator as f64;
        let mut point = interpolate_vertex(first_start, first_end, t);
        point.height_m = 0.0;
        point
    };
    // Exact source-grid predicates distinguish even one-micrometre kerb returns.
    // Neither a metric collinearity buffer nor a parameter margin may invent overlap.
    if cross != 0 {
        let first_numerator = wx * vz - wz * vx;
        let second_numerator = wx * uz - wz * ux;
        let within = |n: i128| {
            if cross > 0 {
                (0..=cross).contains(&n)
            } else {
                (cross..=0).contains(&n)
            }
        };
        return if within(first_numerator) && within(second_numerator) {
            [Some(interpolate(first_numerator, cross)), None]
        } else {
            [None, None]
        };
    }
    if wx * uz - wz * ux != 0 {
        return [None, None];
    }
    let start_parameter = wx * ux + wz * uz;
    let end_parameter = (dx - ax) * ux + (dz - az) * uz;
    let overlap_start = start_parameter.min(end_parameter).max(0);
    let overlap_end = start_parameter.max(end_parameter).min(length_squared);
    if overlap_start > overlap_end {
        return [None, None];
    }
    let first = interpolate(overlap_start, length_squared);
    let second = interpolate(overlap_end, length_squared);
    [Some(first), (!same_xz(first, second)).then_some(second)]
}

pub(in crate::simulation::terrain::cdt) fn segment_bounds_overlap(
    first_start: TerrainCdtVertex,
    first_end: TerrainCdtVertex,
    second_start: TerrainCdtVertex,
    second_end: TerrainCdtVertex,
) -> bool {
    first_start.x.min(first_end.x) <= second_start.x.max(second_end.x) + CDT_EPSILON_M
        && second_start.x.min(second_end.x) <= first_start.x.max(first_end.x) + CDT_EPSILON_M
        && first_start.z.min(first_end.z) <= second_start.z.max(second_end.z) + CDT_EPSILON_M
        && second_start.z.min(second_end.z) <= first_start.z.max(first_end.z) + CDT_EPSILON_M
}

pub(in crate::simulation::terrain::cdt) fn segment_parameter(
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
    x: f64,
    z: f64,
) -> f64 {
    let dx = end.x - start.x;
    let dz = end.z - start.z;
    let length_squared = dx * dx + dz * dz;
    if same_xz(start, end) {
        return 0.0;
    }
    ((x - start.x) * dx + (z - start.z) * dz) / length_squared
}

pub(in crate::simulation::terrain::cdt) fn interpolated_segment_height(
    start: TerrainCdtVertex,
    end: TerrainCdtVertex,
    t: f64,
) -> f32 {
    (f64::from(start.height_m) + f64::from(end.height_m - start.height_m) * t) as f32
}

pub(in crate::simulation::terrain::cdt) fn unit_interval_contains(value: f64) -> bool {
    (-CDT_EPSILON_M..=1.0 + CDT_EPSILON_M).contains(&value)
}

pub(in crate::simulation::terrain::cdt) fn clamp_unit(value: f64) -> f64 {
    value.clamp(0.0, 1.0)
}

pub(in crate::simulation::terrain::cdt) fn cross_xz(ax: f64, az: f64, bx: f64, bz: f64) -> f64 {
    ax * bz - az * bx
}

pub(in crate::simulation::terrain::cdt) fn edge_lies_on_patch_boundary(
    a: TerrainCdtVertex,
    b: TerrainCdtVertex,
    patch: TerrainCdtPatch,
) -> bool {
    (same_coord(a.x, patch.min_x) && same_coord(b.x, patch.min_x))
        || (same_coord(a.x, patch.max_x) && same_coord(b.x, patch.max_x))
        || (same_coord(a.z, patch.min_z) && same_coord(b.z, patch.min_z))
        || (same_coord(a.z, patch.max_z) && same_coord(b.z, patch.max_z))
}

pub(in crate::simulation::terrain::cdt) fn edge_length_xz_m(
    a: TerrainCdtVertex,
    b: TerrainCdtVertex,
) -> f64 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    (dx * dx + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn indexed_guide_noding_matches_exhaustive_edge_order() {
        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        for z in 0..128 {
            for delta in [0.0, if z % 2 == 0 { 0.0005 } else { 1.0 }] {
                let index = vertices.len();
                vertices.extend([
                    TerrainCdtVertex::new(0.0, z as f32 + delta, f64::from(z)),
                    TerrainCdtVertex::new(10.0, z as f32 + 1.0 + delta, f64::from(z)),
                ]);
                edges.push([index, index + 1]);
            }
        }
        let first_sample = vertices.len();
        for z in -1..129 {
            for x in [-0.002, 0.0, 2.5, 5.0, 10.0, 10.002] {
                for offset in [-0.002, 0.0, 0.0005] {
                    vertices.push(TerrainCdtVertex::new(x, -10.0, f64::from(z) + offset));
                }
            }
        }
        let samples = (first_sample..vertices.len()).collect::<Vec<_>>();
        let mut expected_vertices = vertices.clone();
        let mut expected_splits = vec![Vec::new(); edges.len()];
        for &vertex_index in &samples {
            let hits = edges
                .iter()
                .enumerate()
                .filter_map(|(edge_index, edge)| {
                    let start = expected_vertices[edge[0]];
                    let end = expected_vertices[edge[1]];
                    let t = source_sample_parameter_near_road_constraint(
                        start,
                        end,
                        expected_vertices[vertex_index],
                    )?;
                    Some((edge_index, t, interpolated_segment_height(start, end, t)))
                })
                .collect::<Vec<_>>();
            let Some(&(_, _, height)) = hits.first() else {
                continue;
            };
            if hits.iter().all(|hit| same_height(hit.2, height)) {
                expected_vertices[vertex_index].height_m = height;
                for (edge_index, t, _) in hits {
                    expected_splits[edge_index]
                        .push(TerrainCdtRoadConstraintSplit { t, vertex_index });
                }
            }
        }
        let mut splits = vec![Vec::new(); edges.len()];
        split_road_constraints_at_source_samples(&edges, &mut vertices, &samples, &mut splits);
        assert_eq!(vertices, expected_vertices);
        assert_eq!(splits, expected_splits);
    }
}
