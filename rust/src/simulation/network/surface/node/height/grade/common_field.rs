// SPDX-License-Identifier: GPL-2.0-only

//! Continuous junction grade interpolated from its centre and exact span-mouth supports.

use super::super::super::input::NodeArrangementInput;
use super::super::model::NodeBandHeightTriangle;
use super::*;
use spade::{DelaunayTriangulation, Point2, Triangulation};

/// Shared piecewise affine grade; material offsets are applied separately.
#[derive(Debug)]
pub(in crate::simulation::network::surface) struct NodeMaterialGradeField {
    supports: BTreeMap<SurfaceXzKey, f64>,
    triangles: Vec<NodeBandHeightTriangle>,
    hull: Vec<[(RoadVec2, f64); 2]>,
    origin: (RoadVec2, f64),
    gradient: RoadVec2,
}

impl NodeMaterialGradeField {
    /// Checks the exact source certificate before reusing a cached grade.
    pub(in crate::simulation::network::surface::node) fn same_source(&self, other: &Self) -> bool {
        self.supports == other.supports
            && self.origin == other.origin
            && self.gradient == other.gradient
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
        for triangle in &self.triangles {
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
        let (mut xx, mut xz, mut zz, mut xy, mut zy) = (0.0, 0.0, 0.0, 0.0, 0.0);
        for (key, &height) in &supports {
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
        let gradient = RoadVec2::new((xy * zz - zy * xz) / det, (xx * zy - xz * xy) / det);
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
            supports,
            triangles,
            hull,
            origin,
            gradient,
        })
    }

    /// Samples the continuous grade, extending its affine trend outside the mouth hull.
    pub(in crate::simulation::network::surface::node) fn height_at(&self, point: RoadVec2) -> f64 {
        if let Some(&height) = self.supports.get(&SurfaceXzKey::from_road_xz(point)) {
            return height;
        }
        if let Some(height) = self
            .triangles
            .iter()
            .find_map(|triangle| triangle.height_at_exact(point))
        {
            return height;
        }
        // Convex-boundary projection is unique. Interpolating the boundary residual
        // preserves continuity at hull edges and vertices, including outer kerb corners.
        let trend = |point: RoadVec2| self.origin.1 + self.gradient.dot(point - self.origin.0);
        let mut nearest = None;
        for &[a, b] in &self.hull {
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
