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
    pub(in crate::simulation::network::surface::node) fn for_input(
        input: &NodeArrangementInput,
    ) -> Option<Self> {
        Self::from_input(input, &super::contacts::declared_levels(input)?)
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
