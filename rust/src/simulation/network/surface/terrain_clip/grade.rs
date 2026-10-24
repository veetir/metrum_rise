// SPDX-License-Identifier: GPL-2.0-only

//! Source-bound grade equivalence for overlapping terrain boundary owners.

use super::super::{
    RoadSurfaceEarthworkFaceSource, RoadSurfaceSystem, RoadSurfaceTerrainClipSourceEdge,
    keys::SurfaceHeightMmKey,
};
use crate::simulation::network::graph::RegionGraph;
use crate::simulation::network::types::EdgeClass;

impl RoadSurfaceSystem {
    /// Certifies a material level against a retained solve, without changing source ownership.
    /// Work is bounded to the source's one or two endpoint-local grade fields.
    pub(in crate::simulation::network::surface) fn terrain_clip_source_grade(
        &self,
        graph: &RegionGraph,
        edge: &RoadSurfaceTerrainClipSourceEdge,
    ) -> Option<(u32, i64)> {
        let nodes = match edge.source {
            RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id,
                boundary_source: Some(_),
                ..
            }
            | RoadSurfaceEarthworkFaceSource::NodeSameMaterialBoundaryHandoff {
                node_id,
                boundary_source: Some(_),
                ..
            } => [Some(node_id), None],
            RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
                edge_idx,
                edge_class: EdgeClass::Standard,
                ..
            } => {
                let source = graph.edge(edge_idx);
                [
                    Some(graph.get_valid_node(source.start_node)),
                    Some(graph.get_valid_node(source.end_node)),
                ]
            }
            _ => return None,
        };
        let mut selected = None;
        for node in nodes.into_iter().flatten() {
            let Some(field) = self.node_material_grade_fields.get(&node) else {
                continue;
            };
            let base =
                |p: super::super::RoadVec3| field.height_at(super::super::RoadVec2::new(p.x, p.z));
            let level = ((edge.start.y - base(edge.start)) * 1000.0).round() as i64;
            let step = (f64::from(super::super::CURB_STEP_HEIGHT_M) * 1000.0).round() as i64;
            if level != 0 && level != step {
                continue;
            }
            if ![edge.start, edge.end].into_iter().all(|p| {
                SurfaceHeightMmKey::from_m_f64(p.y).as_i64().abs_diff(
                    SurfaceHeightMmKey::from_m_f64(base(p) + level as f64 / 1000.0).as_i64(),
                ) <= super::heights::TERRAIN_CLIP_DUST_HEIGHT_TIE_TOLERANCE_MM
            }) {
                continue;
            }
            let root = field
                .cohort_nodes()
                .and_then(|nodes| nodes.first())
                .copied()
                .unwrap_or(node);
            let candidate = (root, level);
            if selected.is_some_and(|previous| previous != candidate) {
                return None;
            }
            selected = Some(candidate);
        }
        selected
    }
}
