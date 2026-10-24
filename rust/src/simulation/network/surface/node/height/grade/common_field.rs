// SPDX-License-Identifier: GPL-2.0-only

//! Continuous junction grade interpolated from its centre and exact span-mouth supports.

mod coverage;
mod refinement;

use super::super::super::input::NodeArrangementInput;
use super::super::model::NodeBandHeightTriangle;
use super::*;
use crate::simulation::network::surface::{RoadSurfaceSystem, RoadSurfaceVisualNodePieceKind};
use spade::{DelaunayTriangulation, Point2, Triangulation};

/// Shared piecewise affine grade; material offsets are applied separately.
#[derive(Debug)]
pub(in crate::simulation::network::surface) struct NodeMaterialGradeField {
    geometry: std::sync::Arc<NodeMaterialGradeGeometry>,
    cohort_nodes: Option<std::sync::Arc<[u32]>>,
    source_plans: Option<std::sync::Arc<BTreeMap<u32, NodeArrangementInput>>>,
}

#[derive(Debug)]
struct NodeMaterialGradeGeometry {
    supports: BTreeMap<SurfaceXzKey, f64>,
    triangles: Vec<NodeBandHeightTriangle>,
    hull: Vec<[(RoadVec2, f64); 2]>,
    origin: (RoadVec2, f64),
    gradient: RoadVec2,
}

impl NodeMaterialGradeField {
    /// Checks the exact source certificate before reusing a cached grade.
    pub(in crate::simulation::network::surface::node) fn same_source(&self, other: &Self) -> bool {
        self.cohort_nodes == other.cohort_nodes
            && self.source_plans == other.source_plans
            && self.geometry.supports == other.geometry.supports
            && self.geometry.origin == other.geometry.origin
            && self.geometry.gradient == other.geometry.gradient
    }

    /// Reconstructs the shared grade only when all ordinary source material levels are proved.
    pub(in crate::simulation::network::surface::node::height) fn for_input(
        input: &NodeArrangementInput,
    ) -> Option<Self> {
        Self::from_input(input, &super::contacts::declared_levels(input)?)
    }

    /// Returns exact crossings of a transverse band with the solved grade triangles.
    pub(in crate::simulation::network::surface::node::height) fn band_breaks(
        &self,
        a: RoadVec2,
        b: RoadVec2,
    ) -> Vec<f64> {
        let axis = b - a;
        let a_key = SurfaceXzKey::from_road_xz(a);
        let b_key = SurfaceXzKey::from_road_xz(b);
        let mut breaks = vec![0.0, 1.0];
        for triangle in &self.geometry.triangles {
            let vertices = [triangle.a_xz, triangle.b_xz, triangle.c_xz];
            for i in 0..3 {
                let c = vertices[i];
                let other = vertices[(i + 1) % 3] - c;
                let determinant = axis.perp_dot(other);
                if determinant == 0.0 {
                    continue;
                }
                let delta = c - a;
                let t = delta.perp_dot(other) / determinant;
                let u = delta.perp_dot(axis) / determinant;
                let key = SurfaceXzKey::from_road_xz(a + axis * t);
                if t > 0.0 && t < 1.0 && (0.0..=1.0).contains(&u) && key != a_key && key != b_key {
                    breaks.push(t);
                }
            }
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup_by_key(|t| SurfaceXzKey::from_road_xz(a + axis * *t));
        breaks
    }

    /// Builds one deterministic grade from the junction centre and every material boundary along each mouth.
    pub(super) fn from_input(
        input: &NodeArrangementInput,
        levels: &BTreeMap<NodeBandHeightFieldId, f64>,
    ) -> Option<Self> {
        let mut supports = BTreeMap::<SurfaceXzKey, f64>::new();
        let mut center = None;
        for mouth in &input.mouths {
            let first = mouth
                .band_intervals
                .iter()
                .find(|band| band.band_kind == RoadSurfaceBandKind::Carriageway)?;
            let last = mouth
                .band_intervals
                .iter()
                .rfind(|band| band.band_kind == RoadSurfaceBandKind::Carriageway)?;
            center.get_or_insert((first.endpoint_start_world + last.endpoint_end_world) * 0.5);
            for band in &mouth.band_intervals {
                let level = levels[&NodeBandHeightFieldId::new(
                    mouth.order_index,
                    band.band_index,
                    band.band_kind,
                )];
                for point in [band.mouth_start_world, band.mouth_end_world] {
                    let key = SurfaceXzKey::from_road_xz(RoadVec2::new(point.x, point.z));
                    let height = point.y - level;
                    if let Some(previous) = supports.insert(key, height)
                        && SurfaceHeightMmKey::from_m_f64(previous)
                            != SurfaceHeightMmKey::from_m_f64(height)
                    {
                        return None;
                    }
                }
            }
        }
        let center = center?;
        supports
            .entry(SurfaceXzKey::from_road_xz(RoadVec2::new(
                center.x, center.z,
            )))
            .or_insert(center.y);
        let origin = (RoadVec2::new(center.x, center.z), center.y);
        let gradient = fit_gradient(&supports, origin)?;
        // A common affine source grade already has the legacy exact solution. Keep
        // its geometry and carrier identities instead of replacing them with new proofs.
        // Independently rounded source/support keys may differ by one millimetre.
        let planar = input.mouths.iter().all(|mouth| {
            mouth.band_intervals.iter().all(|band| {
                let level = levels[&NodeBandHeightFieldId::new(
                    mouth.order_index,
                    band.band_index,
                    band.band_kind,
                )];
                [
                    band.mouth_start_world,
                    band.mouth_end_world,
                    band.endpoint_start_world,
                    band.endpoint_end_world,
                ]
                .into_iter()
                .chain(band.start_path_world.iter().copied())
                .chain(band.end_path_world.iter().copied())
                .all(|point| {
                    let plane = origin.1 + gradient.dot(RoadVec2::new(point.x, point.z) - origin.0);
                    SurfaceHeightMmKey::from_m_f64(point.y - level)
                        .as_i64()
                        .abs_diff(SurfaceHeightMmKey::from_m_f64(plane).as_i64())
                        <= 1
                })
            })
        });
        if planar {
            return None;
        }
        Self::with_supports(supports, origin, gradient, None)
    }

    fn with_supports(
        supports: BTreeMap<SurfaceXzKey, f64>,
        origin: (RoadVec2, f64),
        gradient: RoadVec2,
        cohort_nodes: Option<std::sync::Arc<[u32]>>,
    ) -> Option<Self> {
        let mut cdt = DelaunayTriangulation::<Point2<f64>>::new();
        for key in supports.keys() {
            let point = key.to_road_xz();
            cdt.insert(Point2::new(point.x, point.y)).ok()?;
        }
        let sample = |point: Point2<f64>| {
            let xz = RoadVec2::new(point.x, point.y);
            (xz, supports[&SurfaceXzKey::from_road_xz(xz)])
        };
        let triangles = cdt
            .inner_faces()
            .map(|face| {
                let [a, b, c] = face.positions().map(sample);
                NodeBandHeightTriangle {
                    a_xz: a.0,
                    a_height_m: a.1,
                    b_xz: b.0,
                    b_height_m: b.1,
                    c_xz: c.0,
                    c_height_m: c.1,
                }
            })
            .collect::<Vec<_>>();
        if triangles.is_empty() {
            return None;
        }
        let hull = cdt
            .convex_hull()
            .map(|edge| edge.positions().map(sample))
            .collect();
        Some(Self {
            geometry: std::sync::Arc::new(NodeMaterialGradeGeometry {
                supports,
                triangles,
                hull,
                origin,
                gradient,
            }),
            cohort_nodes,
            source_plans: None,
        })
    }

    /// Solves one grade from saved centres and the cohort's exterior mouth supports.
    /// Interior handoffs are compiler products, so they do not constrain the shared solve.
    pub(in crate::simulation::network::surface) fn for_cohort(
        graph: &crate::simulation::network::graph::RegionGraph,
        inputs: &[(u32, NodeArrangementInput)],
        internal_edges: &std::collections::BTreeSet<usize>,
        coverage_bounds: [f64; 4],
    ) -> Option<Self> {
        let mut supports = BTreeMap::new();
        let mut centres = std::collections::BTreeSet::new();
        for &(node, _) in inputs {
            let point = graph.node(node).pos;
            let key =
                SurfaceXzKey::from_road_xz(RoadVec2::new(f64::from(point.x), f64::from(point.z)));
            let height = f64::from(point.y);
            if let Some(previous) = supports.insert(key, height)
                && SurfaceHeightMmKey::from_m_f64(previous)
                    != SurfaceHeightMmKey::from_m_f64(height)
            {
                return None;
            }
            centres.insert(key);
        }
        for (node, input) in inputs {
            let levels = super::contacts::declared_levels(input).or_else(|| {
                crate::debug_log!("road", "cohort_grade_material_levels_failed node={}", node);
                None
            })?;
            for mouth in &input.mouths {
                let internal = internal_edges.contains(&mouth.edge_idx);
                let terminal = input.piece_kind == crate::simulation::network::surface::RoadSurfaceVisualNodePieceKind::Terminal;
                if internal && !terminal {
                    continue;
                }
                for band in &mouth.band_intervals {
                    let level = levels[&NodeBandHeightFieldId::new(
                        mouth.order_index,
                        band.band_index,
                        band.band_kind,
                    )];
                    let points = if internal {
                        [band.endpoint_start_world, band.endpoint_end_world]
                    } else {
                        [band.mouth_start_world, band.mouth_end_world]
                    };
                    for point in points {
                        let key = SurfaceXzKey::from_road_xz(RoadVec2::new(point.x, point.z));
                        let height = point.y - level;
                        if let Some(previous) = supports.get_mut(&key) {
                            if SurfaceHeightMmKey::from_m_f64(*previous)
                                .as_i64()
                                .abs_diff(SurfaceHeightMmKey::from_m_f64(height).as_i64())
                                > 1
                            {
                                crate::debug_log!(
                                    "road",
                                    "cohort_grade_support_conflict node={} key={:?} existing={} incoming={} centre={}",
                                    node,
                                    key,
                                    *previous,
                                    height,
                                    centres.contains(&key)
                                );
                                return None;
                            }
                            if !centres.contains(&key) {
                                *previous = previous.min(height);
                            }
                        } else {
                            supports.insert(key, height);
                        }
                    }
                }
            }
        }
        let point = graph.node(inputs.first()?.0).pos;
        let origin = (
            RoadVec2::new(f64::from(point.x), f64::from(point.z)),
            f64::from(point.y),
        );
        let gradient = fit_gradient(&supports, origin)?;
        let mut nodes = inputs.iter().map(|(node, _)| *node).collect::<Vec<_>>();
        nodes.sort_unstable();
        nodes.dedup();
        let mut field = Self::with_supports(supports, origin, gradient, Some(nodes.into()))?
            .with_coverage_bounds(coverage_bounds)?;
        // Coalescing a pre-conformance snapshot can leave a real material residual
        // or an unowned contact. Retain geometry only when the same ownership
        // checks as the publishing compiler certify that source plan.
        let plan_grade = std::sync::Arc::new(Self {
            geometry: std::sync::Arc::clone(&field.geometry),
            cohort_nodes: field.cohort_nodes.clone(),
            source_plans: None,
        });
        let plans = RoadSurfaceSystem::collect_compile_work(
            inputs,
            crate::simulation::network::surface::PARALLEL_NODE_COMPILE_MIN_ITEMS,
            |(node, mut input)| {
                input.coalesce_grade_subdivisions();
                let (mut rails, _) =
                    RoadSurfaceSystem::build_node_rail_contours_from_input_with_profile(
                        &input, false,
                    )
                    .ok()?;
                rails.material_grade = Some(std::sync::Arc::clone(&plan_grade));
                let ownership =
                    RoadSurfaceSystem::build_node_boolean_ownership_from_rails(&rails).ok()?;
                if crate::simulation::network::surface::node::validation::NodeValidationReport::from_owned_region_arrangement_diagnostics(&ownership.owned_region_arrangement).is_some() {
                    return None;
                }
                Some((node, input))
            },
        );
        field.source_plans = Some(std::sync::Arc::new(plans.into_iter().flatten().collect()));
        Some(field)
    }

    /// Returns the source plan captured by this solve. Dirty mouth compilation
    /// replaces the grade and its source plans together before rebuilding geometry.
    pub(in crate::simulation::network::surface::node) fn source_plan(
        &self,
        node: u32,
    ) -> Option<&NodeArrangementInput> {
        self.source_plans.as_ref()?.get(&node)
    }

    /// Retains the bounded dependency cohort alongside its shared immutable geometry.
    pub(in crate::simulation::network::surface) fn cohort_nodes(&self) -> Option<&[u32]> {
        self.cohort_nodes.as_deref()
    }

    /// Rebinds preview-local node identities without copying the grade triangulation.
    pub(in crate::simulation::network::surface) fn with_remapped_cohort_nodes(
        &self,
        node_map: &std::collections::HashMap<u32, u32>,
        edge_map: &std::collections::HashMap<usize, usize>,
    ) -> Option<Self> {
        let mut nodes = self
            .cohort_nodes()?
            .iter()
            .map(|node| node_map.get(node).copied())
            .collect::<Option<Vec<_>>>()?;
        nodes.sort_unstable();
        nodes.dedup();
        let source_plans = if let Some(plans) = &self.source_plans {
            Some(std::sync::Arc::new(
                plans
                    .iter()
                    .map(|(node, input)| {
                        let mut input = input.clone();
                        input.node_id = *node_map.get(node)?;
                        for mouth in &mut input.mouths {
                            mouth.edge_idx = *edge_map.get(&mouth.edge_idx)?;
                        }
                        Some((input.node_id, input))
                    })
                    .collect::<Option<BTreeMap<_, _>>>()?,
            ))
        } else {
            None
        };
        Some(Self {
            geometry: std::sync::Arc::clone(&self.geometry),
            cohort_nodes: Some(nodes.into()),
            source_plans,
        })
    }

    /// Evaluates the common grade only on the ownership solver's proved source carriers.
    pub(in crate::simulation::network::surface::node::height) fn height_regions(
        &self,
        input: &NodeArrangementInput,
        ownership: &crate::simulation::network::surface::node::ownership::NodeBooleanOwnership,
    ) -> Option<Vec<NodeHeightedRegion>> {
        use super::super::evaluate::{
            height_carrier_provenance_by_key, heighted_shape_area_m2,
            resolved_height_authority_key, source_noded_contour_points,
        };
        let levels = super::contacts::declared_levels(input)?;
        let provenance = height_carrier_provenance_by_key(ownership);
        let mut regions = Vec::new();
        for region in &ownership.owned_regions {
            let id = NodeBandHeightFieldId::new(
                region.source_mouth_order_index,
                region.source_band_index?,
                region.kind,
            );
            let level = if let Some(&level) = levels.get(&id) {
                level
            } else if input.piece_kind == RoadSurfaceVisualNodePieceKind::Terminal {
                let mut matching = levels
                    .iter()
                    .filter_map(|(id, &level)| (id.kind() == region.kind).then_some(level));
                let level = matching.next()?;
                if matching.any(|other| other != level) {
                    return None;
                }
                level
            } else {
                return None;
            };
            let shape = region
                .shape
                .iter()
                .map(|contour| {
                    source_noded_contour_points(contour, region)
                        .into_iter()
                        .map(|p| {
                            let point = RoadVec2::new(p[0], p[1]);
                            let source = *provenance.get(&resolved_height_authority_key(
                                region.owner,
                                id,
                                region.claim_priority,
                                point,
                            ))?;
                            let base = self.height_at(point);
                            let height = base + level;
                            Some(NodeHeightedVertex {
                                point_xz: point,
                                height_m: height,
                                height_field_id: id,
                                height_authority: None,
                                source_provenance: Some(source),
                                grade_authority: Some(
                                    NodeGradeVertexAuthority::new_with_source_provenance(
                                        point,
                                        height,
                                        region.owner,
                                        id,
                                        NodeGradeCarrierDecision::DeclaredMaterialContact {
                                            base_height_mm: SurfaceHeightMmKey::from_m_f64(base)
                                                .as_i64(),
                                            level_mm: SurfaceHeightMmKey::from_m_f64(level)
                                                .as_i64(),
                                        },
                                        Some(source),
                                    ),
                                ),
                            })
                        })
                        .collect::<Option<Vec<_>>>()
                })
                .collect::<Option<Vec<_>>>()?;
            let area_m2 = heighted_shape_area_m2(&shape);
            if shape.first()?.len() < 3 {
                let source_area = RoadSurfaceSystem::overlay_shape_area_m2(&region.shape);
                let budget =
                    RoadSurfaceSystem::overlay_numeric_area_budget_for_shape(&region.shape);
                if area_m2 == 0.0 && source_area <= budget {
                    continue;
                }
                crate::debug_log!(
                    "road",
                    "cohort_grade_degenerate_source node={} owner={:?} source_area={} stored_area={} budget={} shape={:?}",
                    input.node_id,
                    region.owner,
                    source_area,
                    region.area_m2,
                    budget,
                    region.shape
                );
                return None;
            }
            regions.push(NodeHeightedRegion {
                kind: region.kind,
                owner: region.owner,
                height_field_id: id,
                shape,
                area_m2,
                seam_constraints: region.seam_constraints.clone(),
            });
        }
        Some(regions)
    }

    /// Samples the continuous grade, extending its affine trend outside the mouth hull.
    pub(in crate::simulation::network::surface) fn height_at(&self, point: RoadVec2) -> f64 {
        if let Some(&height) = self
            .geometry
            .supports
            .get(&SurfaceXzKey::from_road_xz(point))
        {
            return height;
        }
        if let Some(height) = self
            .geometry
            .triangles
            .iter()
            .find_map(|triangle| triangle.height_at_exact(point))
        {
            return height;
        }
        // Convex-boundary projection is unique. Interpolating the boundary residual
        // preserves continuity at hull edges and vertices, including outer kerb corners.
        let trend = |point: RoadVec2| {
            self.geometry.origin.1 + self.geometry.gradient.dot(point - self.geometry.origin.0)
        };
        let mut nearest = None;
        for &[a, b] in &self.geometry.hull {
            let axis = b.0 - a.0;
            let t = ((point - a.0).dot(axis) / axis.length_squared()).clamp(0.0, 1.0);
            let projection = a.0 + axis * t;
            let distance = (point - projection).length_squared();
            let residual = (a.1 - trend(a.0)) * (1.0 - t) + (b.1 - trend(b.0)) * t;
            if nearest.is_none_or(|(previous, _)| distance < previous) {
                nearest = Some((distance, residual));
            }
        }
        trend(point) + nearest.map_or(0.0, |(_, height)| height)
    }
}

fn fit_gradient(
    supports: &BTreeMap<SurfaceXzKey, f64>,
    origin: (RoadVec2, f64),
) -> Option<RoadVec2> {
    let (mut xx, mut xz, mut zz, mut xy, mut zy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for (key, &height) in supports {
        let delta = key.to_road_xz() - origin.0;
        xx += delta.x * delta.x;
        xz += delta.x * delta.y;
        zz += delta.y * delta.y;
        xy += delta.x * (height - origin.1);
        zy += delta.y * (height - origin.1);
    }
    let det = xx * zz - xz * xz;
    if det <= f64::EPSILON * (xx * zz).max(1.0) {
        return None;
    }
    Some(RoadVec2::new(
        (xy * zz - zy * xz) / det,
        (xx * zy - xz * xy) / det,
    ))
}

#[cfg(test)]
mod guide_grade_tests {
    use super::*;
    use crate::simulation::network::surface::node::arrangement::NodeArrangement;
    use crate::simulation::network::surface::node::height::NodeHeightSolution;
    use crate::simulation::network::surface::node::input::{NodeInputBandInterval, NodeInputMouth};
    use crate::simulation::network::surface::{
        IncidentEdgeSide, RoadSurfaceSystem, RoadSurfaceVisualNodePieceKind, RoadVec3,
    };

    #[test]
    fn cdt_interior_guides_sample_the_resolved_non_affine_grade() {
        let kind = RoadSurfaceBandKind::Carriageway;
        let corners = [
            RoadVec3::new(-20.0, -0.2, -20.0),
            RoadVec3::new(20.0, 0.2, -20.0),
            RoadVec3::new(20.0, 0.2, 20.0),
            RoadVec3::new(-20.0, -0.2, 20.0),
        ];
        let input = NodeArrangementInput {
            node_id: 0,
            piece_kind: RoadSurfaceVisualNodePieceKind::JunctionN,
            mouths: (0..4)
                .map(|i| NodeInputMouth {
                    order_index: i,
                    edge_idx: i,
                    side: IncidentEdgeSide::Start,
                    direction_xz: RoadVec2::X,
                    direction_angle_ccw: 0.0,
                    conflict_handoff_distance_m: 1.0,
                    mouth_rails: Vec::new(),
                    endpoint_rails: Vec::new(),
                    boundary_rails: Vec::new(),
                    uses_explicit_band_domain_paths: false,
                    band_intervals: vec![NodeInputBandInterval {
                        band_index: 0,
                        band_kind: kind,
                        mouth_start_world: corners[i],
                        mouth_end_world: corners[(i + 1) % 4],
                        endpoint_start_world: RoadVec3::new(0.0, 0.5, 0.0),
                        endpoint_end_world: RoadVec3::new(0.0, 0.5, 0.0),
                        start_path_world: Vec::new(),
                        end_path_world: Vec::new(),
                    }],
                })
                .collect(),
        };
        let levels = (0..4)
            .map(|i| (NodeBandHeightFieldId::new(i, 0, kind), 0.0))
            .collect();
        let grade = NodeMaterialGradeField::from_input(&input, &levels).unwrap();
        let owner = NodeBandOwner::new(kind, 0);
        let field = NodeBandHeightFieldId::new(0, 0, kind);
        let heights = NodeHeightSolution {
            node_id: 0,
            piece_kind: input.piece_kind,
            regions: vec![NodeHeightedRegion {
                kind,
                owner,
                height_field_id: field,
                area_m2: 1600.0,
                seam_constraints: Vec::new(),
                shape: vec![
                    corners
                        .iter()
                        .map(|p| NodeHeightedVertex {
                            point_xz: RoadVec2::new(p.x, p.z),
                            height_m: p.y,
                            height_field_id: field,
                            height_authority: None,
                            source_provenance: None,
                            grade_authority: Some(NodeGradeVertexAuthority::new(
                                RoadVec2::new(p.x, p.z),
                                p.y,
                                owner,
                                field,
                                NodeGradeCarrierDecision::DeclaredMaterialContact {
                                    base_height_mm: (p.y * 1000.0).round() as i64,
                                    level_mm: 0,
                                },
                            )),
                        })
                        .collect(),
                ],
            }],
        };
        let arrangement = NodeArrangement::from_height_solution(&heights).unwrap();
        let compile = |grade| {
            RoadSurfaceSystem::build_node_triangulation_from_arrangement_with_grade(
                &arrangement,
                grade,
                None,
            )
            .unwrap()
        };
        let plane_guides = compile(None);
        assert!(
            plane_guides.regions[0].vertices.iter().any(|v| {
                let p = v.point_world;
                (p.y - grade.height_at(RoadVec2::new(p.x, p.z))).abs() > 0.05
            }),
            "the fixture must expose the independently fitted guide plane"
        );
        let resolved = compile(Some(&grade));
        assert!(resolved.regions[0].vertices.len() > corners.len());
        for vertex in &resolved.regions[0].vertices {
            let p = vertex.point_world;
            assert!(
                (p.y - grade.height_at(RoadVec2::new(p.x, p.z))).abs() < 1e-6,
                "derived guide must sample the resolved grade: {p:?}"
            );
        }
    }
}
