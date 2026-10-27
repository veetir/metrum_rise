// SPDX-License-Identifier: GPL-2.0-only

//! Cross-owner carriageway precedence for connected, same-level roadbeds.

use super::backend::godot_vec3_to_road;
use super::*;
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::types::{EdgeClass, TransitType};
use std::collections::BTreeSet;

/// Source owner of a visible roadbed material.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RoadMaterialOwner {
    /// A span's source bands.
    Span(usize),
    /// A junction's source bands.
    Node(u32),
}

#[derive(Debug)]
enum MaterialSource {
    Span(Arc<RoadSurfaceVisualSpanPiece>),
    Node(Arc<RoadSurfaceVisualNodePiece>),
}

impl MaterialSource {
    fn grid(&self) -> &SurfaceTriangleGrid {
        match self {
            Self::Span(piece) => &piece.surface_query,
            Self::Node(piece) => &piece.surface_query,
        }
    }

    fn triangle(&self, item: u32) -> ([RoadVec3; 3], bool) {
        match self {
            Self::Span(piece) => piece.surface_query_triangle(item),
            Self::Node(piece) => piece.surface_query_triangle(item),
        }
    }
}

/// Local source references and lazy unions on the existing 32 m query tiles.
/// Union triangulation distinguishes exterior kerbs from internal asphalt diagonals.
#[derive(Debug)]
pub(crate) struct RoadSurfaceMaterialMask {
    sources: Vec<MaterialSource>,
    tiles: HashMap<SurfaceChunkKey, Option<MaterialTile>>,
}

/// A carriageway union triangle and its actual exterior boundary edges.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RoadMaterialClipTriangle {
    /// Counterclockwise XZ positions on the road overlay grid.
    pub(crate) points: [RoadVec3; 3],
    /// Edge `i` runs from point `i` to `(i + 1) % 3`.
    pub(crate) exterior_edges: [bool; 3],
}

#[derive(Debug)]
struct MaterialTile {
    triangles: Vec<RoadMaterialClipTriangle>,
    grid: SurfaceTriangleGrid,
}

impl RoadSurfaceMaterialMask {
    /// Collects a local union, retaining real exterior kerbs and removing internal walls.
    /// A failed union invalidates the mesh payload instead of publishing unpartitioned bands.
    pub(crate) fn collect_clips(
        &mut self,
        bounds: [f64; 4],
        items: &mut Vec<u32>,
        triangles: &mut Vec<RoadMaterialClipTriangle>,
    ) -> Option<()> {
        triangles.clear();
        let min = RoadSurfaceSystem::query_chunk_coords_for_world(bounds[0], bounds[1]);
        let max = RoadSurfaceSystem::query_chunk_coords_for_world(bounds[2], bounds[3]);
        for x in min.0..=max.0 {
            for z in min.1..=max.1 {
                let tile = self
                    .tiles
                    .entry((x, z))
                    .or_insert_with(|| {
                        let (a, b) = RoadSurfaceSystem::query_chunk_world_bounds((x, z));
                        MaterialTile::build(&self.sources, [a.x, a.z, b.x, b.z], items)
                    })
                    .as_ref()?;
                items.clear();
                items.extend(tile.grid.items_in_bounds(bounds));
                items.sort_unstable();
                items.dedup();
                for &item in items.iter() {
                    let triangle = tile.triangles[item as usize];
                    if triangle_bounds_touch(triangle.points, bounds) {
                        triangles.push(triangle);
                    }
                }
            }
        }
        triangles.sort_unstable_by_key(|t| {
            (
                t.points.map(keys::SurfaceXzKey::from_world_xz),
                t.exterior_edges,
            )
        });
        triangles.dedup_by_key(|t| {
            (
                t.points.map(keys::SurfaceXzKey::from_world_xz),
                t.exterior_edges,
            )
        });
        Some(())
    }
}

impl MaterialTile {
    fn build(sources: &[MaterialSource], bounds: [f64; 4], items: &mut Vec<u32>) -> Option<Self> {
        let mut contours = Vec::new();
        for source in sources {
            items.clear();
            items.extend(source.grid().items_in_bounds(bounds));
            items.sort_unstable();
            items.dedup();
            for &item in items.iter() {
                let (triangle, carriageway) = source.triangle(item);
                if !carriageway || !triangle_bounds_touch(triangle, bounds) {
                    continue;
                }
                // Match uploaded asphalt positions before the overlay's micrometre rounding.
                let mut contour: Vec<_> = triangle
                    .into_iter()
                    .map(|p| [f64::from(p.x as f32), f64::from(p.z as f32)])
                    .collect();
                if RoadSurfaceSystem::overlay_contour_area_f64(&contour) < 0.0 {
                    contour.reverse();
                }
                contours.push(contour);
            }
        }
        let shapes = RoadSurfaceSystem::overlay_union_contours(&contours)?;
        let mut triangles = Vec::new();
        for shape in shapes {
            let rings: Vec<Vec<_>> = shape
                .into_iter()
                .map(|ring| {
                    ring.into_iter()
                        .map(|p| RoadVec3::new(p[0], 0.0, p[1]))
                        .collect()
                })
                .collect();
            let mut boundary = BTreeSet::new();
            for ring in &rings {
                for i in 0..ring.len() {
                    boundary.insert(keys::SurfaceXzSegmentKey::new(
                        keys::SurfaceXzKey::from_world_xz(ring[i]),
                        keys::SurfaceXzKey::from_world_xz(ring[(i + 1) % ring.len()]),
                    ));
                }
            }
            for mut points in RoadSurfaceSystem::triangulate_constrained_rings_xz(&rings)? {
                if (points[1] - points[0]).cross(points[2] - points[0]).y > 0.0 {
                    points.swap(1, 2);
                }
                let exterior_edges = std::array::from_fn(|i| {
                    boundary.contains(&keys::SurfaceXzSegmentKey::new(
                        keys::SurfaceXzKey::from_world_xz(points[i]),
                        keys::SurfaceXzKey::from_world_xz(points[(i + 1) % 3]),
                    ))
                });
                triangles.push(RoadMaterialClipTriangle {
                    points,
                    exterior_edges,
                });
            }
        }
        let grid =
            SurfaceTriangleGrid::from_triangles(triangles.iter().map(|t| t.points), |i| i as u32);
        Some(Self { triangles, grid })
    }
}

impl RoadSurfaceSystem {
    /// Reports independently graded overlapping source-connected node carriageways.
    #[cfg(test)]
    pub(crate) fn junction_grade_conflicts_for_geometry_audit(
        &self,
        graph: &RegionGraph,
    ) -> Vec<(u32, u32)> {
        let mut conflicts = BTreeSet::new();
        for (&node, a) in &self.compiled_visual_node_pieces {
            let Some(level) = owner_level(graph, RoadMaterialOwner::Node(node)) else {
                continue;
            };
            for &id in graph.node_adjacency(node) {
                let e = graph.edge(id);
                if e.deleted
                    || e.class != EdgeClass::Standard
                    || e.primary_type != TransitType::Road
                    || e.crossing_level != level
                {
                    continue;
                }
                let start = graph.get_valid_node(e.start_node);
                let other = if start == node {
                    graph.get_valid_node(e.end_node)
                } else {
                    start
                };
                if other <= node
                    || owner_level(graph, RoadMaterialOwner::Node(other)) != Some(level)
                {
                    continue;
                }
                if let Some(b) = self.compiled_visual_node_pieces.get(&other)
                    && node_grade_overlap(a, b, false) == Some(true)
                {
                    conflicts.insert((node, other));
                }
            }
        }
        conflicts.into_iter().collect()
    }

    /// Validates final uploaded nonroad tops against asphalt, allowing only the upload's
    /// coordinate-rounding strip. Used by the read-only imported-map acceptance sweep.
    #[cfg(test)]
    pub(crate) fn material_mesh_overlaps_for_geometry_audit(
        mesh: &crate::simulation::network::render::NetworkMeshData,
    ) -> Vec<(usize, usize, f64, f64)> {
        let convert = |t: &[godot::prelude::Vector3; 3]| {
            [t[0], t[1], t[2]]
                .map(|p| RoadVec3::new(f64::from(p.x), f64::from(p.y), f64::from(p.z)))
        };
        let roads: Vec<_> = mesh
            .road_vertices
            .as_chunks::<3>()
            .0
            .iter()
            .map(convert)
            .collect();
        let grid = SurfaceTriangleGrid::from_triangles(roads.iter().copied(), |i| i as u32);
        let mut failures = Vec::new();
        for (index, triangle) in mesh
            .curb_vertices
            .as_chunks::<3>()
            .0
            .iter()
            .chain(mesh.sidewalk_vertices.as_chunks::<3>().0.iter())
            .map(convert)
            .enumerate()
        {
            let bounds = triangle_bounds(triangle);
            let mut candidates: Vec<_> = grid.items_in_bounds(bounds).collect();
            candidates.sort_unstable();
            candidates.dedup();
            for item in candidates {
                let road = roads[item as usize];
                let area = triangle_overlap_area(triangle, road);
                // Both uploaded triangles have rounded coordinates. Bound their boundary
                // strips using a full f32 ULP per endpoint plus the overlay's 1 µm grid.
                let coordinate = triangle
                    .into_iter()
                    .chain(road)
                    .map(|p| p.x.abs().max(p.z.abs()))
                    .fold(1.0_f64, f64::max);
                let perimeter = [triangle, road]
                    .iter()
                    .map(|t| {
                        (0..3)
                            .map(|i| {
                                RoadVec2::new(t[i].x - t[(i + 1) % 3].x, t[i].z - t[(i + 1) % 3].z)
                                    .length()
                            })
                            .sum::<f64>()
                    })
                    .sum::<f64>();
                let budget = perimeter * (coordinate * f64::from(f32::EPSILON) + 1e-6);
                if area > budget {
                    failures.push((index, item as usize, area, budget));
                }
            }
        }
        failures
    }

    /// Tests the same cross-owner material precedence without allocating query storage.
    pub(in crate::simulation::network::surface) fn material_nonroad_is_occluded(
        &self,
        graph: &RegionGraph,
        owner: RoadMaterialOwner,
        point: RoadVec2,
        include_edge: &impl Fn(usize) -> bool,
        include_node: &impl Fn(u32) -> bool,
    ) -> bool {
        let Some(level) = owner_level(graph, owner) else {
            return false;
        };
        let target = match owner {
            RoadMaterialOwner::Span(id) => self
                .compiled_visual_span_pieces
                .get(&id)
                .filter(|piece| piece.filled_crossing.is_none())
                .map(|p| MaterialSourceRef::Span(p)),
            RoadMaterialOwner::Node(id) => self
                .compiled_visual_node_pieces
                .get(&id)
                .map(|p| MaterialSourceRef::Node(p)),
        };
        let Some(target) = target else {
            return false;
        };
        let chunk = Self::query_chunk_coords_for_world(point.x, point.y);
        let edges = self
            .query_chunk_spans
            .get(&chunk)
            .into_iter()
            .flat_map(|set| set.iter().copied())
            .filter(|id| include_edge(*id))
            .map(RoadMaterialOwner::Span);
        let nodes = self
            .query_chunk_nodes
            .get(&chunk)
            .into_iter()
            .flat_map(|set| set.iter().copied())
            .filter(|id| include_node(*id))
            .map(RoadMaterialOwner::Node);
        for other in edges.chain(nodes) {
            if other == owner
                || owner_level(graph, other) != Some(level)
                || !(owners_connected(graph, owner, other, level)
                    || self.owners_share_grade_cohort(graph, owner, other)
                    || self.owners_share_grade_cohort(graph, other, owner))
            {
                continue;
            }
            let source = match other {
                RoadMaterialOwner::Span(id) => self
                    .compiled_visual_span_pieces
                    .get(&id)
                    .filter(|piece| piece.filled_crossing.is_none())
                    .map(|p| MaterialSourceRef::Span(p)),
                RoadMaterialOwner::Node(id) => self
                    .compiled_visual_node_pieces
                    .get(&id)
                    .map(|p| MaterialSourceRef::Node(p)),
            };
            let Some(source) = source else {
                continue;
            };
            for &item in source.grid().cell_items(point) {
                let (triangle, carriageway) = source.triangle(item);
                if !carriageway || !inside_triangle(triangle, point) {
                    continue;
                }
                if target.grid().cell_items(point).iter().any(|&item| {
                    let (nonroad, carriageway) = target.triangle(item);
                    !carriageway && material_triangles_overlap(triangle, nonroad)
                }) {
                    return true;
                }
            }
        }
        false
    }

    /// Resolves connected carriageways that overlap this owner's nonroad materials.
    /// Work follows local indexed contributors and their existing triangle-grid cells.
    pub(crate) fn material_partition_mask(
        &self,
        graph: &RegionGraph,
        owner: RoadMaterialOwner,
        mask_carriageway: bool,
    ) -> Option<RoadSurfaceMaterialMask> {
        let level = owner_level(graph, owner)?;
        if mask_carriageway
            && !owner_nodes(graph, owner).iter().any(|node| {
                self.node_material_grade_fields
                    .get(node)
                    .is_some_and(|field| field.cohort_nodes().is_some())
            })
        {
            return None;
        }
        let target = self.material_source(owner)?;
        let bounds = target.grid().bounds_xz;
        let (edges, nodes) = self.collect_spatial_query_contributors_for_bounds(
            bounds[0], bounds[1], bounds[2], bounds[3],
        );
        let mut sources = Vec::new();
        for other in edges
            .into_iter()
            .map(RoadMaterialOwner::Span)
            .chain(nodes.into_iter().map(RoadMaterialOwner::Node))
        {
            if other == owner
                || owner_level(graph, other) != Some(level)
                || !(owners_connected(graph, owner, other, level)
                    || self.owners_share_grade_cohort(graph, owner, other)
                    || self.owners_share_grade_cohort(graph, other, owner))
            {
                continue;
            }
            if mask_carriageway && !self.asphalt_owner_precedes(graph, other, owner) {
                continue;
            }
            let Some(source) = self.material_source(other) else {
                continue;
            };
            let overlaps = source.grid().items_in_bounds(bounds).any(|item| {
                let (triangle, carriageway) = source.triangle(item);
                if !carriageway || !triangle_bounds_overlap(triangle, bounds) {
                    return false;
                }
                let local_bounds = triangle_bounds(triangle);
                target.grid().items_in_bounds(local_bounds).any(|item| {
                    let (nonroad, target_road) = target.triangle(item);
                    mask_carriageway == target_road && material_triangles_overlap(triangle, nonroad)
                })
            });
            if overlaps {
                sources.push(source);
            }
        }
        (!sources.is_empty()).then_some(RoadSurfaceMaterialMask {
            sources,
            tiles: HashMap::new(),
        })
    }

    fn owners_share_grade_cohort(
        &self,
        graph: &RegionGraph,
        a: RoadMaterialOwner,
        b: RoadMaterialOwner,
    ) -> bool {
        let a_nodes = owner_nodes(graph, a);
        let b_nodes = owner_nodes(graph, b);
        a_nodes.into_iter().any(|a| {
            self.node_material_grade_fields
                .get(&a)
                .and_then(|field| field.cohort_nodes())
                .is_some_and(|members| b_nodes.iter().all(|b| members.binary_search(b).is_ok()))
        })
    }

    fn asphalt_owner_precedes(
        &self,
        graph: &RegionGraph,
        source: RoadMaterialOwner,
        target: RoadMaterialOwner,
    ) -> bool {
        let RoadMaterialOwner::Node(node) = source else {
            return false;
        };
        if !self
            .node_material_grade_fields
            .get(&node)
            .is_some_and(|field| field.cohort_nodes().is_some())
        {
            return false;
        }
        match target {
            RoadMaterialOwner::Span(_) => {
                owner_nodes(graph, target).contains(&node)
                    || self.owners_share_grade_cohort(graph, source, target)
            }
            RoadMaterialOwner::Node(other) => {
                self.owners_share_grade_cohort(graph, source, target)
                    && (
                        keys::SurfaceXzKey::from_world_xz(godot_vec3_to_road(graph.node(node).pos)),
                        node,
                    ) < (
                        keys::SurfaceXzKey::from_world_xz(godot_vec3_to_road(
                            graph.node(other).pos,
                        )),
                        other,
                    )
            }
        }
    }

    fn material_source(&self, owner: RoadMaterialOwner) -> Option<MaterialSource> {
        match owner {
            RoadMaterialOwner::Span(id) => {
                let piece = self.compiled_visual_span_pieces.get(&id)?;
                piece
                    .filled_crossing
                    .is_none()
                    .then(|| MaterialSource::Span(Arc::clone(piece)))
            }
            RoadMaterialOwner::Node(id) => self
                .compiled_visual_node_pieces
                .get(&id)
                .map(|piece| MaterialSource::Node(Arc::clone(piece))),
        }
    }
}

#[derive(Clone, Copy)]
enum MaterialSourceRef<'a> {
    Span(&'a RoadSurfaceVisualSpanPiece),
    Node(&'a RoadSurfaceVisualNodePiece),
}

impl MaterialSourceRef<'_> {
    fn grid(&self) -> &SurfaceTriangleGrid {
        match self {
            Self::Span(p) => &p.surface_query,
            Self::Node(p) => &p.surface_query,
        }
    }
    fn triangle(&self, item: u32) -> ([RoadVec3; 3], bool) {
        match self {
            Self::Span(p) => p.surface_query_triangle(item),
            Self::Node(p) => p.surface_query_triangle(item),
        }
    }
}

fn inside_triangle(triangle: [RoadVec3; 3], point: RoadVec2) -> bool {
    let triangle = triangle.map(|p| RoadVec2::new(p.x, p.z));
    let sides = std::array::from_fn::<_, 3, _>(|i| {
        (triangle[(i + 1) % 3] - triangle[i]).perp_dot(point - triangle[i])
    });
    sides.iter().all(|&side| side >= 0.0) || sides.iter().all(|&side| side <= 0.0)
}

fn owner_nodes(graph: &RegionGraph, owner: RoadMaterialOwner) -> [u32; 2] {
    match owner {
        RoadMaterialOwner::Node(id) => [id, id],
        RoadMaterialOwner::Span(id) => {
            let edge = graph.edge(id);
            [
                graph.get_valid_node(edge.start_node),
                graph.get_valid_node(edge.end_node),
            ]
        }
    }
}

/// Returns the source crossing level only for an ordinary same-level roadbed owner.
pub(in crate::simulation::network::surface) fn owner_level(
    graph: &RegionGraph,
    owner: RoadMaterialOwner,
) -> Option<i16> {
    match owner {
        RoadMaterialOwner::Span(id) => {
            let edge = graph.edge(id);
            (!edge.deleted
                && edge.class == EdgeClass::Standard
                && edge.primary_type == TransitType::Road)
                .then_some(edge.crossing_level)
        }
        RoadMaterialOwner::Node(id) => {
            let mut level = None;
            for &id in graph.node_adjacency(id) {
                let edge = graph.edge(id);
                if edge.deleted {
                    continue;
                }
                if edge.primary_type != TransitType::Road
                    || level.is_some_and(|level| level != edge.crossing_level)
                {
                    return None;
                }
                level = Some(edge.crossing_level);
            }
            level
        }
    }
}

fn owners_connected(
    graph: &RegionGraph,
    a: RoadMaterialOwner,
    b: RoadMaterialOwner,
    level: i16,
) -> bool {
    let a = owner_nodes(graph, a);
    let b = owner_nodes(graph, b);
    a.into_iter().any(|node| {
        b.contains(&node)
            || graph.node_adjacency(node).iter().any(|&id| {
                let edge = graph.edge(id);
                !edge.deleted
                    && edge.class == EdgeClass::Standard
                    && edge.primary_type == TransitType::Road
                    && edge.crossing_level == level
                    && b.contains(&graph.get_valid_node(
                        if graph.get_valid_node(edge.start_node) == node {
                            edge.end_node
                        } else {
                            edge.start_node
                        },
                    ))
            })
    })
}

fn triangle_bounds(triangle: [RoadVec3; 3]) -> [f64; 4] {
    triangle.into_iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |b, p| [b[0].min(p.x), b[1].min(p.z), b[2].max(p.x), b[3].max(p.z)],
    )
}

fn triangle_bounds_touch(triangle: [RoadVec3; 3], b: [f64; 4]) -> bool {
    let a = triangle_bounds(triangle);
    a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1]
}

fn triangle_bounds_overlap(triangle: [RoadVec3; 3], b: [f64; 4]) -> bool {
    let a = triangle_bounds(triangle);
    a[0] < b[2] && a[2] > b[0] && a[1] < b[3] && a[3] > b[1]
}

// Positive material domains use the same micrometre-grid area floor as node overlays.
fn material_triangles_overlap(a: [RoadVec3; 3], b: [RoadVec3; 3]) -> bool {
    triangle_overlap_area(a, b) > f64::from(NODE_OVERLAY_MIN_AREA_M2)
}

// Convex triangle intersection on the source geometry, with fixed local scratch.
fn triangle_overlap_area(a: [RoadVec3; 3], b: [RoadVec3; 3]) -> f64 {
    let (points, len) = triangle_overlap_points(a, b);
    (1..len.saturating_sub(1))
        .map(|i| (points[i] - points[0]).perp_dot(points[i + 1] - points[0]))
        .sum::<f64>()
        .abs()
        * 0.5
}

fn triangle_overlap_points(a: [RoadVec3; 3], b: [RoadVec3; 3]) -> ([RoadVec2; 8], usize) {
    if !triangle_bounds_overlap(a, triangle_bounds(b)) {
        return ([RoadVec2::ZERO; 8], 0);
    }
    let mut clip = b.map(|p| RoadVec2::new(p.x, p.z));
    if (clip[1] - clip[0]).perp_dot(clip[2] - clip[0]) < 0.0 {
        clip.swap(1, 2);
    }
    let mut points = [RoadVec2::ZERO; 8];
    points[..3].copy_from_slice(&a.map(|p| RoadVec2::new(p.x, p.z)));
    let mut len = 3;
    for i in 0..3 {
        let origin = clip[i];
        let axis = clip[(i + 1) % 3] - origin;
        let mut output = [RoadVec2::ZERO; 8];
        let mut count = 0;
        for j in 0..len {
            let p = points[j];
            let q = points[(j + 1) % len];
            let x = axis.perp_dot(p - origin);
            let y = axis.perp_dot(q - origin);
            if x >= 0.0 {
                output[count] = p;
                count += 1;
            }
            if (x > 0.0 && y < 0.0) || (x < 0.0 && y > 0.0) {
                output[count] = p.lerp(q, x / (x - y));
                count += 1;
            }
        }
        points = output;
        len = count;
        if len < 3 {
            return (points, 0);
        }
    }
    (points, len)
}

/// Tests actual carriageway intersections using the pieces' existing triangle grids.
/// The flag distinguishes a positive overlap from an incompatible independently solved grade.
pub(in crate::simulation::network::surface) fn node_grade_overlap(
    a: &RoadSurfaceVisualNodePiece,
    b: &RoadSurfaceVisualNodePiece,
    full_roadbed: bool,
) -> Option<bool> {
    let mut overlaps = false;
    for item in a.surface_query.items_in_bounds(b.surface_query.bounds_xz) {
        let (triangle, road) = a.surface_query_triangle(item);
        if !full_roadbed && !road {
            continue;
        }
        for other in b.surface_query.items_in_bounds(triangle_bounds(triangle)) {
            let (other, other_road) = b.surface_query_triangle(other);
            if (!full_roadbed && !other_road) || !material_triangles_overlap(triangle, other) {
                continue;
            }
            overlaps = true;
            let (points, len) = triangle_overlap_points(triangle, other);
            if road
                && other_road
                && points[..len].iter().any(|&point| {
                    RoadSurfaceSystem::triangle_height_at_xz(&triangle, point)
                        .zip(RoadSurfaceSystem::triangle_height_at_xz(&other, point))
                        .is_some_and(|(a, b)| (a - b).abs() > 0.001)
                })
            {
                return Some(true);
            }
        }
    }
    overlaps.then_some(false)
}
