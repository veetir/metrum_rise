// SPDX-License-Identifier: GPL-2.0-only

//! Source-bound material CDT refinement at the shared junction grade facets.

use super::*;
use std::collections::BTreeSet;

impl NodeMaterialGradeField {
    /// Refines material triangles at proved shared-grade facet breaks without changing ownership.
    pub(in crate::simulation::network::surface) fn refine_triangulation(
        &self,
        solution: &mut crate::simulation::network::surface::node::triangulation::NodeTriangulationSolution,
        arrangement: &crate::simulation::network::surface::node::arrangement::NodeArrangement,
    ) -> Option<()> {
        use crate::simulation::network::surface::node::triangulation::{
            NodeTriangulatedTriangle, NodeTriangulatedVertex,
        };
        use crate::simulation::network::surface::{
            RoadSurfaceSystem, RoadVec3, SurfaceTriangleGrid,
        };
        let grade_triangles = self
            .geometry
            .triangles
            .iter()
            .map(|t| {
                [
                    RoadVec3::new(t.a_xz.x, t.a_height_m, t.a_xz.y),
                    RoadVec3::new(t.b_xz.x, t.b_height_m, t.b_xz.y),
                    RoadVec3::new(t.c_xz.x, t.c_height_m, t.c_xz.y),
                ]
            })
            .collect::<Vec<_>>();
        let grade_grid =
            SurfaceTriangleGrid::from_triangles(grade_triangles.iter().copied(), |i| i as u32);
        let mut candidates = Vec::new();
        for region in &mut solution.regions {
            let levels = region
                .vertices
                .iter()
                .filter_map(|v| match v.grade_authority.decision {
                    NodeGradeCarrierDecision::DeclaredMaterialContact { level_mm, .. } => {
                        Some(level_mm)
                    }
                    _ => None,
                })
                .collect::<BTreeSet<_>>();
            if levels.len() != 1 {
                return None;
            }
            let level_mm = *levels.first()?;
            for vertex in &mut region.vertices {
                if vertex.grade_authority.source_provenance.is_none()
                    && !matches!(
                        vertex.grade_authority.decision,
                        NodeGradeCarrierDecision::DeclaredMaterialContact { .. }
                    )
                {
                    let p = RoadVec2::new(vertex.point_world.x, vertex.point_world.z);
                    let base = self.height_at(p);
                    let height = base + level_mm as f64 / 1000.0;
                    vertex.point_world.y = height;
                    vertex.grade_authority = NodeGradeVertexAuthority::new(
                        p,
                        height,
                        region.owner,
                        region.height_field_id,
                        NodeGradeCarrierDecision::DeclaredMaterialContact {
                            base_height_mm: SurfaceHeightMmKey::from_m_f64(base).as_i64(),
                            level_mm,
                        },
                    );
                }
            }
            let source = arrangement.regions().get(region.source_region_index)?;
            let mut shape = std::iter::once(source.outer_loop())
                .chain(source.holes().iter().map(Vec::as_slice))
                .map(|ring| {
                    ring.iter()
                        .map(|id| {
                            let p = arrangement.vertices()[id.index()].point_xz();
                            [p.x, p.y]
                        })
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>();
            for (i, ring) in shape.iter_mut().enumerate() {
                if (RoadSurfaceSystem::overlay_contour_area_f64(ring) > 0.0) != (i == 0) {
                    ring.reverse();
                }
            }
            let bounds = shape.iter().flatten().fold(
                [
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ],
                |b, p| {
                    [
                        b[0].min(p[0]),
                        b[1].min(p[1]),
                        b[2].max(p[0]),
                        b[3].max(p[1]),
                    ]
                },
            );
            candidates.clear();
            candidates.extend(grade_grid.items_in_bounds(bounds));
            candidates.sort_unstable();
            candidates.dedup();
            let mut lookup = region
                .vertices
                .iter()
                .enumerate()
                .map(|(i, v)| (SurfaceXzKey::from_world_xz(v.point_world), i))
                .collect::<BTreeMap<_, _>>();
            let owner = region.owner;
            let height_field_id = region.height_field_id;
            let mut insert_vertex =
                |key: SurfaceXzKey, vertices: &mut Vec<NodeTriangulatedVertex>| {
                    *lookup.entry(key).or_insert_with(|| {
                        let p = key.to_road_xz();
                        let base = self.height_at(p);
                        let height = base + level_mm as f64 / 1000.0;
                        let i = vertices.len();
                        vertices.push(NodeTriangulatedVertex {
                            point_world: RoadVec3::new(p.x, height, p.y),
                            height_field_id,
                            grade_authority: NodeGradeVertexAuthority::new(
                                p,
                                height,
                                owner,
                                height_field_id,
                                NodeGradeCarrierDecision::DeclaredMaterialContact {
                                    base_height_mm: SurfaceHeightMmKey::from_m_f64(base).as_i64(),
                                    level_mm,
                                },
                            ),
                        });
                        i
                    })
                };
            region.triangles.clear();
            let mut constraints = BTreeSet::new();
            // Node the complete facet graph and material boundary in one overlay.
            // Independent per-facet intersections can round a shared intersection
            // differently, leaving a crack or overlapping boundary intervals.
            let mut cuts = BTreeSet::new();
            for &candidate in &candidates {
                let points = grade_triangles[candidate as usize].map(SurfaceXzKey::from_world_xz);
                for i in 0..3 {
                    let mut edge = [points[i], points[(i + 1) % 3]];
                    edge.sort_unstable();
                    cuts.insert(edge);
                }
            }
            let cuts = cuts
                .into_iter()
                .map(|edge| {
                    edge.map(|p| {
                        let p = p.to_road_xz();
                        [p.x, p.y]
                    })
                })
                .collect::<Vec<_>>();
            let mut boundary_uses = BTreeMap::<[usize; 2], usize>::new();
            for cell in RoadSurfaceSystem::overlay_slice_shape_by_segments(&shape, &cuts)? {
                let rings = cell
                    .iter()
                    .map(|ring| {
                        ring.iter()
                            .map(|p| {
                                let xz = RoadVec2::new(p[0], p[1]);
                                RoadVec3::new(
                                    p[0],
                                    self.height_at(xz) + level_mm as f64 / 1000.0,
                                    p[1],
                                )
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();
                for triangle in RoadSurfaceSystem::triangulate_constrained_rings_xz(&rings)? {
                    if !canonical_facet_triangle_has_area(triangle) {
                        continue;
                    }
                    let vertices = triangle.map(|p| {
                        insert_vertex(SurfaceXzKey::from_world_xz(p), &mut region.vertices)
                    });
                    if vertices[0] != vertices[1]
                        && vertices[1] != vertices[2]
                        && vertices[0] != vertices[2]
                    {
                        region.triangles.push(NodeTriangulatedTriangle { vertices });
                    }
                }
                for ring in cell {
                    for i in 0..ring.len() {
                        let mut edge = [ring[i], ring[(i + 1) % ring.len()]].map(|p| {
                            insert_vertex(SurfaceXzKey::from_overlay_point(p), &mut region.vertices)
                        });
                        edge.sort_unstable();
                        if edge[0] != edge[1] {
                            *boundary_uses.entry(edge).or_default() += 1;
                        }
                    }
                }
            }
            for (edge, count) in boundary_uses {
                if count > 2 {
                    return None;
                }
                if count == 1 {
                    constraints.insert(edge);
                }
            }
            // Long grade facets can meet a kerb at a very small angle. Bisect the
            // longest edge of a rejected aspect-ratio face, on both incident faces,
            // without moving either material or grade boundaries. Each split halves
            // face area; the existing blocking-area floor bounds the local product.
            let edge_key = |mut edge: [usize; 2]| {
                edge.sort_unstable();
                edge
            };
            let edges = |t: &NodeTriangulatedTriangle| {
                std::array::from_fn::<_, 3, _>(|i| {
                    edge_key([t.vertices[i], t.vertices[(i + 1) % 3]])
                })
            };
            let mut incidence = BTreeMap::<[usize; 2], std::collections::BTreeSet<usize>>::new();
            let mut pending = std::collections::VecDeque::new();
            for (i, t) in region.triangles.iter().enumerate() {
                for edge in edges(t) {
                    incidence.entry(edge).or_default().insert(i);
                }
                pending.push_back(i);
            }
            while let Some(i) = pending.pop_front() {
                let triangle = region.triangles[i];
                let points = triangle.vertices.map(|v| region.vertices[v].point_world);
                if !crate::simulation::network::surface::node::validation::needs_aspect_refinement(
                    points,
                ) {
                    continue;
                }
                let edge = edges(&triangle).into_iter().max_by(|a, b| {
                    let length = |e: &[usize; 2]| {
                        let a = region.vertices[e[0]].point_world;
                        let b = region.vertices[e[1]].point_world;
                        (b.x - a.x).hypot(b.z - a.z)
                    };
                    length(a).total_cmp(&length(b)).then(a.cmp(b))
                })?;
                let a = region.vertices[edge[0]].point_world;
                let b = region.vertices[edge[1]].point_world;
                let midpoint =
                    SurfaceXzKey::from_road_xz(RoadVec2::new((a.x + b.x) * 0.5, (a.z + b.z) * 0.5));
                let middle = insert_vertex(midpoint, &mut region.vertices);
                if edge.contains(&middle) {
                    return None;
                }
                let adjacent = incidence.get(&edge)?.iter().copied().collect::<Vec<_>>();
                for index in adjacent {
                    let old = region.triangles[index];
                    for e in edges(&old) {
                        incidence.get_mut(&e)?.remove(&index);
                    }
                    let start = (0..3).find(|&j| {
                        edge_key([old.vertices[j], old.vertices[(j + 1) % 3]]) == edge
                    })?;
                    let [a, b, c] = std::array::from_fn(|j| old.vertices[(start + j) % 3]);
                    region.triangles[index] = NodeTriangulatedTriangle {
                        vertices: [a, middle, c],
                    };
                    let new = region.triangles.len();
                    region.triangles.push(NodeTriangulatedTriangle {
                        vertices: [middle, b, c],
                    });
                    for i in [index, new] {
                        for e in edges(&region.triangles[i]) {
                            incidence.entry(e).or_default().insert(i);
                        }
                        pending.push_back(i);
                    }
                }
                if constraints.remove(&edge) {
                    constraints.insert(edge_key([edge[0], middle]));
                    constraints.insert(edge_key([middle, edge[1]]));
                }
            }
            // f64 world coordinates can give a collinear grid triangle a tiny
            // signed area. It must not count as a third face on a real mesh edge.
            for (i, triangle) in region.triangles.iter().enumerate() {
                if !canonical_facet_triangle_has_area(
                    triangle.vertices.map(|v| region.vertices[v].point_world),
                ) {
                    for edge in edges(triangle) {
                        incidence.get_mut(&edge)?.remove(&i);
                    }
                }
            }
            region.triangles.retain(|t| {
                canonical_facet_triangle_has_area(
                    t.vertices.map(|v| region.vertices[v].point_world),
                )
            });
            discard_zero_area_boundary_spurs(&mut constraints, &incidence);
            region.boundary_constraints = constraints.into_iter().collect();
        }
        // Grade intersections can canonically node a material boundary that was
        // sub-micrometre off its neighbour before CDT refinement. Publish the step
        // from the two retained declared-grade proofs, including its exact extent.
        let mut boundaries =
            BTreeMap::<[SurfaceXzKey; 2], Vec<[NodeGradeVertexAuthority; 2]>>::new();
        for region in &solution.regions {
            // The retained top mesh is authoritative after integer slicing. A
            // sub-grid boundary sliver can leave a cut classified as an internal
            // cell edge even though the adjacent material retains that same edge.
            // Both endpoints must still prove the same declared grade and level.
            let edges = region
                .triangles
                .iter()
                .flat_map(|t| (0..3).map(move |i| [t.vertices[i], t.vertices[(i + 1) % 3]]))
                .chain(region.boundary_constraints.iter().copied());
            for [a, b] in edges {
                let mut contacts = [
                    region.vertices[a].grade_authority,
                    region.vertices[b].grade_authority,
                ];
                contacts.sort_unstable_by_key(|c| c.key);
                boundaries
                    .entry(contacts.map(|c| c.key))
                    .or_default()
                    .push(contacts);
            }
        }
        let mut steps = solution
            .explicit_vertical_step_segments
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();
        for contacts in boundaries.values() {
            for (i, a) in contacts.iter().enumerate() {
                for b in &contacts[i + 1..] {
                    if a[0].shares_declared_material_base(b[0])
                        && a[1].shares_declared_material_base(b[1])
                        && a[0].height_key != b[0].height_key
                        && a[1].height_key != b[1].height_key
                        && let Some(step) = crate::simulation::network::surface::node::arrangement::NodeExplicitVerticalStepSegment::new(
                            crate::simulation::network::surface::node::arrangement::NodeArrangementKey::from_surface_key(a[0].key),
                            crate::simulation::network::surface::node::arrangement::NodeArrangementKey::from_surface_key(a[1].key),
                            a[0].owner, b[0].owner)
                    {
                        steps.insert(step);
                    }
                }
            }
        }
        solution.explicit_vertical_step_segments = steps.into_iter().collect();
        Some(())
    }
}

// Only exact zero area on the shared topology grid is discarded. Real grid
// slivers remain subject to the unchanged area, aspect and coverage validators.
fn canonical_facet_triangle_has_area(
    points: [crate::simulation::network::surface::RoadVec3; 3],
) -> bool {
    let keys = points.map(|p| SurfaceXzKey::from_world_xz(p).raw_tuple());
    SurfaceXzKey::raw_tuple_triangle_area_m2_abs(keys[0], keys[1], keys[2]) > 0.0
}

// Canonical rounding can collapse the tip of a microscopic boundary triangle onto
// its neighbour, leaving a dangling constraint with no incident face. It encloses
// exactly zero area. Keep its source vertex, but discard this empty topological spur.
// An edge used by any retained face cannot be removed by this cleanup.
fn discard_zero_area_boundary_spurs(
    constraints: &mut BTreeSet<[usize; 2]>,
    incidence: &BTreeMap<[usize; 2], BTreeSet<usize>>,
) {
    let mut by_vertex = BTreeMap::<usize, BTreeSet<[usize; 2]>>::new();
    for &edge in constraints.iter() {
        for vertex in edge {
            by_vertex.entry(vertex).or_default().insert(edge);
        }
    }
    let mut pending = by_vertex
        .iter()
        .filter_map(|(&v, e)| (e.len() == 1).then_some(v))
        .collect::<std::collections::VecDeque<_>>();
    while let Some(vertex) = pending.pop_front() {
        let edges = &by_vertex[&vertex];
        if edges.len() != 1 {
            continue;
        }
        let edge = *edges.first().unwrap();
        if incidence.get(&edge).is_some_and(|faces| !faces.is_empty()) {
            continue;
        }
        constraints.remove(&edge);
        for v in edge {
            let edges = by_vertex.get_mut(&v).unwrap();
            edges.remove(&edge);
            if edges.len() == 1 {
                pending.push_back(v);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_facet_filter_preserves_real_grid_slivers() {
        use crate::simulation::network::surface::RoadVec3;
        let point = |(x, z)| {
            let p = SurfaceXzKey::from_raw_keys(x, z).to_road_xz();
            RoadVec3::new(p.x, 0.0, p.y)
        };
        assert!(!canonical_facet_triangle_has_area([
            point((-660117015, 2097996184)),
            point((-660116927, 2097996206)),
            point((-660116971, 2097996195))
        ]));
        assert!(canonical_facet_triangle_has_area([
            point((-660117015, 2097996184)),
            point((-660116927, 2097996206)),
            point((-660116971, 2097996196))
        ]));
    }

    #[test]
    fn boundary_spur_cleanup_keeps_every_face_edge() {
        let face_edges = BTreeMap::from([
            ([0, 1], BTreeSet::from([0])),
            ([0, 2], BTreeSet::from([0])),
            ([1, 2], BTreeSet::from([0])),
        ]);
        let mut boundary = BTreeSet::from([[0, 1], [0, 2], [1, 2], [1, 3], [3, 4]]);
        discard_zero_area_boundary_spurs(&mut boundary, &face_edges);
        assert_eq!(boundary, BTreeSet::from([[0, 1], [0, 2], [1, 2]]));
        let mut open_real_boundary = BTreeSet::from([[0, 1], [1, 2]]);
        discard_zero_area_boundary_spurs(&mut open_real_boundary, &face_edges);
        assert_eq!(open_real_boundary, BTreeSet::from([[0, 1], [1, 2]]));
    }
}
