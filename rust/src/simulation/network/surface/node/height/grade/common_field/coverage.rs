// SPDX-License-Identifier: GPL-2.0-only

//! Source-constrained grade coverage over the complete affected roadbed domain.

use super::*;
use crate::simulation::network::surface::SurfaceCdt;
use std::collections::BTreeSet;

impl NodeMaterialGradeField {
    /// Keeps every original facet and source height. The source grade's extrapolation
    /// supplies only the enclosing corners, so the complete roadbed has an explicit
    /// affine triangulation instead of unrepresented nearest-hull grade breaks.
    pub(in crate::simulation::network::surface::node::height) fn with_coverage_bounds(
        &self,
        bounds: [f64; 4],
    ) -> Option<Self> {
        let mut supports = self.geometry.supports.clone();
        let mut min = SurfaceXzKey::from_road_xz(RoadVec2::new(bounds[0], bounds[1])).raw_tuple();
        let mut max = SurfaceXzKey::from_road_xz(RoadVec2::new(bounds[2], bounds[3])).raw_tuple();
        for key in supports.keys() {
            let (x, z) = key.raw_tuple();
            min = (min.0.min(x), min.1.min(z));
            max = (max.0.max(x), max.1.max(z));
        }
        for (x, z) in [
            (min.0, min.1),
            (min.0, max.1),
            (max.0, min.1),
            (max.0, max.1),
        ] {
            let key = SurfaceXzKey::from_raw_keys(x, z);
            supports
                .entry(key)
                .or_insert_with(|| self.height_at(key.to_road_xz()));
        }
        if supports.len() == self.geometry.supports.len() {
            return Some(Self {
                geometry: std::sync::Arc::clone(&self.geometry),
                cohort_nodes: self.cohort_nodes.clone(),
            });
        }
        let keys = supports.keys().copied().collect::<Vec<_>>();
        let indices = keys
            .iter()
            .enumerate()
            .map(|(i, &key)| (key, i))
            .collect::<BTreeMap<_, _>>();
        let points = keys
            .iter()
            .map(|key| {
                let p = key.to_road_xz();
                Point2::new(p.x, p.y)
            })
            .collect();
        let mut constraints = BTreeSet::new();
        for triangle in &self.geometry.triangles {
            let vertices = [triangle.a_xz, triangle.b_xz, triangle.c_xz]
                .map(|p| indices[&SurfaceXzKey::from_road_xz(p)]);
            for i in 0..3 {
                let mut edge = [vertices[i], vertices[(i + 1) % 3]];
                edge.sort_unstable();
                constraints.insert(edge);
            }
        }
        let mut rejected = 0;
        let cdt = SurfaceCdt::try_bulk_load_cdt(points, constraints.into_iter().collect(), |_| {
            rejected += 1
        })
        .ok()?;
        if rejected != 0 {
            return None;
        }
        let sample = |p: Point2<f64>| {
            let xz = RoadVec2::new(p.x, p.y);
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
            .collect();
        let hull = cdt
            .convex_hull()
            .map(|edge| edge.positions().map(sample))
            .collect();
        Some(Self {
            geometry: std::sync::Arc::new(NodeMaterialGradeGeometry {
                supports,
                triangles,
                hull,
                origin: self.geometry.origin,
                gradient: self.geometry.gradient,
            }),
            cohort_nodes: self.cohort_nodes.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grade_coverage_enclosure_preserves_source_facets_and_anchors() {
        let supports = [
            ((-2, -2), 0.0),
            ((2, -2), 0.0),
            ((0, 2), 1.0),
            ((0, 0), 0.3),
        ]
        .into_iter()
        .map(|((x, z), h)| {
            (
                SurfaceXzKey::from_road_xz(RoadVec2::new(x as f64, z as f64)),
                h,
            )
        })
        .collect::<BTreeMap<_, _>>();
        let origin = (RoadVec2::ZERO, 0.3);
        let gradient = fit_gradient(&supports, origin).unwrap();
        let field = NodeMaterialGradeField::with_supports(supports.clone(), origin, gradient, None)
            .unwrap();
        let facets = |field: &NodeMaterialGradeField| {
            field
                .geometry
                .triangles
                .iter()
                .map(|t| {
                    let mut keys = [t.a_xz, t.b_xz, t.c_xz].map(SurfaceXzKey::from_road_xz);
                    keys.sort_unstable();
                    keys
                })
                .collect::<BTreeSet<_>>()
        };
        let before = facets(&field);
        let corners = [
            RoadVec2::new(-4.0, -4.0),
            RoadVec2::new(-4.0, 4.0),
            RoadVec2::new(4.0, -4.0),
            RoadVec2::new(4.0, 4.0),
        ];
        let heights = corners.map(|p| field.height_at(p));
        let covered = field.with_coverage_bounds([-4.0, -4.0, 4.0, 4.0]).unwrap();
        assert!(before.is_subset(&facets(&covered)));
        for (point, height) in supports {
            assert_eq!(covered.height_at(point.to_road_xz()), height);
        }
        for (point, height) in corners.into_iter().zip(heights) {
            assert_eq!(covered.height_at(point), height);
        }
    }
}
