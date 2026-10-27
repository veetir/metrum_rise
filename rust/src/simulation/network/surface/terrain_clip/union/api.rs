// SPDX-License-Identifier: GPL-2.0-only

//! Terrain-clip union public entry points.

use super::*;

impl RoadSurfaceSystem {
    #[cfg(test)]
    pub(in crate::simulation::network::surface) fn union_terrain_clip_boundary_export(
        boundary_loops: &[RoadSurfaceTerrainClipLoop],
    ) -> Result<RoadSurfaceTerrainClipExport, RoadSurfaceTerrainClipExportError> {
        let boundary_loop_refs = boundary_loops.iter().collect::<Vec<_>>();
        Self::union_terrain_clip_boundary_refs_export(&boundary_loop_refs, &[])
    }

    pub(in crate::simulation::network::surface) fn union_terrain_clip_boundary_refs_export(
        boundary_loops: &[&RoadSurfaceTerrainClipLoop],
        source_grades: &[Vec<Option<(u32, i64)>>],
    ) -> Result<RoadSurfaceTerrainClipExport, RoadSurfaceTerrainClipExportError> {
        let contours =
            Self::union_terrain_clip_boundary_contours_with_sources(boundary_loops, source_grades)?;
        let loops = contours
            .iter()
            .map(|contour| contour.boundary_loop.clone())
            .collect::<Vec<_>>();
        let loop_topologies = contours
            .iter()
            .map(|contour| contour.topology)
            .collect::<Vec<_>>();
        Ok(RoadSurfaceTerrainClipExport {
            loops,
            loop_topologies,
        })
    }

    #[cfg(test)]
    pub(in crate::simulation::network::surface) fn union_terrain_clip_boundary_loops_with_sources(
        boundary_loops: &[RoadSurfaceTerrainClipLoop],
    ) -> Result<Vec<RoadSurfaceTerrainClipLoop>, RoadSurfaceTerrainClipExportError> {
        let boundary_loop_refs = boundary_loops.iter().collect::<Vec<_>>();
        Self::union_terrain_clip_boundary_contours_with_sources(&boundary_loop_refs, &[]).map(
            |contours| {
                contours
                    .into_iter()
                    .map(|contour| contour.boundary_loop)
                    .collect()
            },
        )
    }

    fn union_terrain_clip_boundary_contours_with_sources(
        boundary_loops: &[&RoadSurfaceTerrainClipLoop],
        source_grades: &[Vec<Option<(u32, i64)>>],
    ) -> Result<Vec<TerrainClipOutputContour>, RoadSurfaceTerrainClipExportError> {
        if boundary_loops.is_empty() {
            return Ok(Vec::new());
        }

        let contours = Self::overlay_contours_from_terrain_clip_boundary_loops(boundary_loops);
        // Terrain clip input is occupied road footprint. Node exporters may emit final outer
        // contours with either winding at vertical-step boundaries, while real holes still cancel
        // against enclosing contours under the non-zero rule.
        let Some(mut shapes) = Self::overlay_union_terrain_contours(&contours) else {
            return Err(RoadSurfaceTerrainClipExportError::OverlayUnionFailed {
                source_loop_count: boundary_loops.len(),
            });
        };
        Self::sort_overlay_shapes(&mut shapes);
        let mut source_edges = Self::terrain_clip_source_edges_from_boundary_loops(boundary_loops);
        if !source_grades.is_empty() {
            for edge in &mut source_edges {
                edge.grade_authority = source_grades
                    .get(edge.source_index)
                    .and_then(|grades| grades.get(edge.edge_index))
                    .copied()
                    .flatten();
            }
        }
        Self::terrain_clip_boundary_contours_from_overlay_shapes_with_source_edges(
            &shapes,
            &source_edges,
        )
    }
}
