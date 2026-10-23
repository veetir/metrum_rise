// SPDX-License-Identifier: GPL-2.0-only

//! Terrain-clip output edge sourcing.

use super::super::{
    NodeFootprintBoundarySegmentSource, NodeFootprintBoundaryVertexSource, NodeOverlayPoint,
    RoadSurfaceSystem, RoadSurfaceVisualNodePieceKind, backend::RoadVec3,
    earthwork::RoadSurfaceEarthworkFaceSource, keys::SurfaceHeightMmKey,
};
use super::heights::interval_height_at;
use super::model::*;
use std::collections::{BTreeMap, BTreeSet};

enum TerrainClipOutputSourceSelection {
    Missing,
    Ambiguous(String),
    Source(TerrainClipSourceEdge),
}

impl RoadSurfaceSystem {
    pub(super) fn append_terrain_clip_sourced_segment_points(
        out: &mut Vec<RoadSurfaceTerrainClipSourceEdge>,
        mut points: Vec<RoadVec3>,
        source_edges: &[TerrainClipSourceEdge],
    ) -> Result<(), TerrainClipOutputSourceError> {
        Self::dedup_terrain_clip_top_envelope_points(&mut points);
        for segment in points.windows(2) {
            let start = segment[0];
            let end = segment[1];
            if Self::world_points_same_for_boundary(start, end) {
                continue;
            }
            if Self::canonical_numeric_dust_boundary_point(start, end).is_some() {
                continue;
            }
            let source = Self::terrain_clip_output_source_for_points(start, end, source_edges)?;
            Self::append_terrain_clip_source_edge(
                out,
                RoadSurfaceTerrainClipSourceEdge {
                    start,
                    end,
                    kind: source.kind,
                    source: source.source,
                },
            );
        }
        Ok(())
    }

    pub(super) fn append_terrain_clip_prepared_segment_points(
        out: &mut Vec<RoadSurfaceTerrainClipSourceEdge>,
        mut points: Vec<RoadVec3>,
        segment_start: NodeOverlayPoint,
        segment_end: NodeOverlayPoint,
        prepared_sources: &[TerrainClipPreparedSource],
        source_edges: &[TerrainClipSourceEdge],
    ) -> Result<(), TerrainClipOutputSourceError> {
        Self::dedup_terrain_clip_top_envelope_points(&mut points);
        let mut start_t = points.first().and_then(|point| {
            Self::overlay_line_parameter([point.x, point.z], segment_start, segment_end)
        });
        for segment in points.windows(2) {
            let start = segment[0];
            let end = segment[1];
            let end_t = Self::overlay_line_parameter([end.x, end.z], segment_start, segment_end);
            if Self::world_points_same_for_boundary(start, end)
                || Self::canonical_numeric_dust_boundary_point(start, end).is_some()
            {
                start_t = end_t;
                continue;
            }
            let source = match (start_t, end_t) {
                (Some(start_t), Some(end_t)) => {
                    Self::terrain_clip_output_source_for_prepared_points(
                        start,
                        end,
                        start_t,
                        end_t,
                        prepared_sources,
                        source_edges,
                    )?
                }
                _ => Self::terrain_clip_output_source_for_points(start, end, source_edges)?,
            };
            Self::append_terrain_clip_source_edge(
                out,
                RoadSurfaceTerrainClipSourceEdge {
                    start,
                    end,
                    kind: source.kind,
                    source: source.source,
                },
            );
            start_t = end_t;
        }
        Ok(())
    }

    fn terrain_clip_output_source_for_prepared_points(
        start: RoadVec3,
        end: RoadVec3,
        start_t: f64,
        end_t: f64,
        prepared_sources: &[TerrainClipPreparedSource],
        source_edges: &[TerrainClipSourceEdge],
    ) -> Result<TerrainClipSourceEdge, TerrainClipOutputSourceError> {
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_source_for_prepared_segment(
                start,
                end,
                start_t,
                end_t,
                prepared_sources,
            ),
            start,
            end,
        )? {
            return Ok(source);
        }
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_source_for_endpoint_segment(start, end, source_edges),
            start,
            end,
        )? {
            return Ok(source);
        }
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_dust_connector_source(start, end, source_edges),
            start,
            end,
        )? {
            return Ok(source);
        }
        Err(TerrainClipOutputSourceError::Missing { start, end })
    }

    fn terrain_clip_output_source_for_points(
        start: RoadVec3,
        end: RoadVec3,
        source_edges: &[TerrainClipSourceEdge],
    ) -> Result<TerrainClipSourceEdge, TerrainClipOutputSourceError> {
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_source_for_segment(start, end, source_edges),
            start,
            end,
        )? {
            return Ok(source);
        }
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_source_for_endpoint_segment(start, end, source_edges),
            start,
            end,
        )? {
            return Ok(source);
        }
        if let Some(source) = Self::terrain_clip_output_source_result(
            Self::terrain_clip_output_dust_connector_source(start, end, source_edges),
            start,
            end,
        )? {
            return Ok(source);
        }
        Err(TerrainClipOutputSourceError::Missing { start, end })
    }

    fn terrain_clip_output_source_result(
        selection: TerrainClipOutputSourceSelection,
        start: RoadVec3,
        end: RoadVec3,
    ) -> Result<Option<TerrainClipSourceEdge>, TerrainClipOutputSourceError> {
        match selection {
            TerrainClipOutputSourceSelection::Missing => Ok(None),
            TerrainClipOutputSourceSelection::Source(source) => Ok(Some(source)),
            TerrainClipOutputSourceSelection::Ambiguous(context) => {
                Err(TerrainClipOutputSourceError::Ambiguous {
                    start,
                    end,
                    context,
                })
            }
        }
    }

    pub(super) fn close_terrain_clip_source_edges(edges: &mut [RoadSurfaceTerrainClipSourceEdge]) {
        if edges.len() < 2 {
            return;
        }
        let first_start = edges[0].start;
        let last_index = edges.len() - 1;
        let last_end = edges[last_index].end;
        if Self::world_points_same_for_boundary(first_start, last_end) {
            let shared = if last_end.y > first_start.y {
                last_end
            } else {
                first_start
            };
            edges[0].start = shared;
            edges[last_index].end = shared;
        } else if let Some(shared) =
            Self::canonical_numeric_dust_boundary_point(first_start, last_end)
        {
            edges[0].start = shared;
            edges[last_index].end = shared;
        }
    }

    /// Retains every sourced endpoint, including a finite material-step corner return.
    pub(super) fn complete_terrain_clip_source_edge_chain(
        mut edges: Vec<RoadSurfaceTerrainClipSourceEdge>,
    ) -> Result<Vec<RoadSurfaceTerrainClipSourceEdge>, (RoadVec3, RoadVec3)> {
        for i in 0..edges.len() {
            let next = (i + 1) % edges.len();
            let a = edges[i].end;
            let b = edges[next].start;
            let ah = SurfaceHeightMmKey::from_m_f64(a.y).as_i64();
            let bh = SurfaceHeightMmKey::from_m_f64(b.y).as_i64();
            // Dust-chain recovery already proves this one-millimetre height tie. A run
            // may skip several 0.1 mm edges; preserve the established CDT incidence budget.
            if (a.x - b.x).hypot(a.z - b.z) <= f64::from(super::super::SAMPLE_EPSILON_M)
                && ah.abs_diff(bh) <= super::heights::TERRAIN_CLIP_DUST_HEIGHT_TIE_TOLERANCE_MM
            {
                let key = Self::terrain_clip_world_key(a).min(Self::terrain_clip_world_key(b));
                let point = key.to_road_xz();
                let shared = RoadVec3::new(point.x, ah.max(bh) as f64 / 1000.0, point.y);
                edges[i].end = shared;
                edges[next].start = shared;
            }
        }
        let gaps = edges
            .iter()
            .enumerate()
            .filter(|(i, edge)| {
                !Self::world_points_same_for_boundary(edge.end, edges[(i + 1) % edges.len()].start)
            })
            .count();
        if gaps == 0 {
            return Ok(edges);
        }
        let mut completed = Vec::with_capacity(edges.len() + gaps);
        for (i, &edge) in edges.iter().enumerate() {
            completed.push(edge);
            let next = edges[(i + 1) % edges.len()];
            if Self::world_points_same_for_boundary(edge.end, next.start) {
                continue;
            }
            let connector = Self::terrain_clip_node_material_corner_connector(edge, next)
                .ok_or((edge.end, next.start))?;
            completed.push(connector);
        }
        Ok(completed)
    }

    fn terrain_clip_node_material_corner_connector(
        a: RoadSurfaceTerrainClipSourceEdge,
        b: RoadSurfaceTerrainClipSourceEdge,
    ) -> Option<RoadSurfaceTerrainClipSourceEdge> {
        use super::super::band_semantics::raised_step_band_rank;
        let width = f64::from(super::super::NODE_OVERLAY_NUMERIC_DUST_WIDTH_M);
        if (a.end.x - b.start.x).hypot(a.end.z - b.start.z) > width {
            return None;
        }
        let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
            node_id,
            kind,
            owner_kind: ak,
            owner_index: ai,
            boundary_source: Some(ap),
        } = a.source
        else {
            return None;
        };
        let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
            node_id: bn,
            kind: bk,
            owner_kind: bkind,
            owner_index: bi,
            boundary_source: Some(bp),
        } = b.source
        else {
            return None;
        };
        if node_id != bn || kind != bk {
            return None;
        }
        let ah = SurfaceHeightMmKey::from_m_f64(a.end.y).as_i64();
        let bh = SurfaceHeightMmKey::from_m_f64(b.start.y).as_i64();
        let proof = Some(NodeFootprintBoundarySegmentSource {
            start: ap.end,
            end: bp.start,
        });
        let (source, edge_kind) = if ah == bh && ak == bkind {
            (
                RoadSurfaceEarthworkFaceSource::NodeSameMaterialBoundaryHandoff {
                    node_id,
                    kind,
                    owner_kind: ak,
                    owner_index_a: ai.min(bi),
                    owner_index_b: ai.max(bi),
                    boundary_source: proof,
                },
                a.kind,
            )
        } else {
            let step_mm = (f64::from(super::super::CURB_STEP_HEIGHT_M) * 1000.0).round() as u64;
            if ah.abs_diff(bh) != step_mm {
                return None;
            }
            let (low_kind, high_kind, high_owner, edge_kind) = if ah < bh {
                (ak, bkind, bi, b.kind)
            } else {
                (bkind, ak, ai, a.kind)
            };
            if raised_step_band_rank(high_kind)? <= raised_step_band_rank(low_kind)? {
                return None;
            }
            (
                RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                    node_id,
                    kind,
                    owner_kind: high_kind,
                    owner_index: high_owner,
                    boundary_source: proof,
                },
                edge_kind,
            )
        };
        Some(RoadSurfaceTerrainClipSourceEdge {
            start: a.end,
            end: b.start,
            kind: edge_kind,
            source,
        })
    }

    fn append_terrain_clip_source_edge(
        out: &mut Vec<RoadSurfaceTerrainClipSourceEdge>,
        mut edge: RoadSurfaceTerrainClipSourceEdge,
    ) {
        if Self::world_points_same_for_boundary(edge.start, edge.end) {
            return;
        }
        if Self::canonical_numeric_dust_boundary_point(edge.start, edge.end).is_some() {
            return;
        }
        if let Some(last) = out.last_mut()
            && Self::world_points_same_for_boundary(last.end, edge.start)
        {
            let shared = if edge.start.y > last.end.y {
                edge.start
            } else {
                last.end
            };
            last.end = shared;
            edge.start = shared;
        }
        if let Some(last) = out.last_mut()
            && let Some(shared) = Self::canonical_numeric_dust_boundary_point(last.end, edge.start)
        {
            last.end = shared;
            edge.start = shared;
        }
        out.push(edge);
    }

    fn terrain_clip_output_source_for_segment(
        start: RoadVec3,
        end: RoadVec3,
        source_edges: &[TerrainClipSourceEdge],
    ) -> TerrainClipOutputSourceSelection {
        let start_overlay = [start.x, start.z];
        let end_overlay = [end.x, end.z];
        let mut candidates = Vec::new();
        for &source_edge in source_edges {
            let Some(interval) = Self::terrain_clip_source_interval_on_segment(
                start_overlay,
                end_overlay,
                source_edge,
            ) else {
                continue;
            };
            if !Self::terrain_clip_interval_covers(interval, 0.0, 1.0) {
                continue;
            }
            if Self::overlay_heights_equal(interval_height_at(interval, 0.0), start.y)
                && Self::overlay_heights_equal(interval_height_at(interval, 1.0), end.y)
            {
                candidates.push(source_edge);
            }
        }
        Self::unique_terrain_clip_output_source(candidates, "covered_segment", Some((start, end)))
    }

    fn terrain_clip_output_source_for_prepared_segment(
        start: RoadVec3,
        end: RoadVec3,
        start_t: f64,
        end_t: f64,
        prepared_sources: &[TerrainClipPreparedSource],
    ) -> TerrainClipOutputSourceSelection {
        let coverage_start_t = start_t.min(end_t);
        let coverage_end_t = start_t.max(end_t);
        let mut candidates = Vec::with_capacity(prepared_sources.len());
        for prepared in prepared_sources {
            let interval = prepared.interval;
            if !Self::terrain_clip_interval_covers(interval, coverage_start_t, coverage_end_t) {
                continue;
            }
            if Self::overlay_heights_equal(interval_height_at(interval, start_t), start.y)
                && Self::overlay_heights_equal(interval_height_at(interval, end_t), end.y)
            {
                candidates.push(prepared.edge);
            }
        }
        Self::unique_terrain_clip_output_source(candidates, "covered_segment", Some((start, end)))
    }

    fn terrain_clip_output_source_for_endpoint_segment(
        start: RoadVec3,
        end: RoadVec3,
        source_edges: &[TerrainClipSourceEdge],
    ) -> TerrainClipOutputSourceSelection {
        let start_key = Self::terrain_clip_world_key(start);
        let end_key = Self::terrain_clip_world_key(end);
        let candidates = source_edges
            .iter()
            .copied()
            .filter(|source_edge| {
                let source_start_key = Self::terrain_clip_world_key(source_edge.start);
                let source_end_key = Self::terrain_clip_world_key(source_edge.end);
                if source_start_key == start_key && source_end_key == end_key {
                    Self::overlay_heights_equal(source_edge.start.y, start.y)
                        && Self::overlay_heights_equal(source_edge.end.y, end.y)
                } else if source_start_key == end_key && source_end_key == start_key {
                    Self::overlay_heights_equal(source_edge.start.y, end.y)
                        && Self::overlay_heights_equal(source_edge.end.y, start.y)
                } else {
                    false
                }
            })
            .collect::<Vec<_>>();
        Self::unique_terrain_clip_output_source(candidates, "endpoint_segment", Some((start, end)))
    }

    fn terrain_clip_output_dust_connector_source(
        start: RoadVec3,
        end: RoadVec3,
        source_edges: &[TerrainClipSourceEdge],
    ) -> TerrainClipOutputSourceSelection {
        let mut candidates = Self::terrain_clip_source_edges_at_world_xz_point(start, source_edges);
        candidates.extend(Self::terrain_clip_source_edges_at_world_xz_point(
            end,
            source_edges,
        ));
        if let Some(source) =
            Self::canonical_same_owner_dust_connector_output_source(&candidates, start, end)
        {
            return TerrainClipOutputSourceSelection::Source(source);
        }
        if let Some(source) =
            Self::canonical_span_handoff_connector_output_source(&candidates, start, end)
        {
            return TerrainClipOutputSourceSelection::Source(source);
        }
        Self::unique_terrain_clip_output_source(candidates, "dust_connector", Some((start, end)))
    }

    pub(super) fn terrain_clip_source_edges_at_world_xz_point(
        point: RoadVec3,
        source_edges: &[TerrainClipSourceEdge],
    ) -> Vec<TerrainClipSourceEdge> {
        let overlay_point = [point.x, point.z];
        source_edges
            .iter()
            .copied()
            .filter(|&source_edge| {
                let source_start = [source_edge.start.x, source_edge.start.z];
                let source_end = [source_edge.end.x, source_edge.end.z];
                let Some(_t) =
                    Self::overlay_segment_parameter(overlay_point, source_start, source_end)
                else {
                    return false;
                };
                true
            })
            .collect()
    }

    fn unique_terrain_clip_output_source(
        mut candidates: Vec<TerrainClipSourceEdge>,
        context: &'static str,
        canonical_segment: Option<(RoadVec3, RoadVec3)>,
    ) -> TerrainClipOutputSourceSelection {
        if candidates.is_empty() {
            return TerrainClipOutputSourceSelection::Missing;
        }
        candidates.sort_by(|a, b| terrain_clip_source_edge_ordering(*a, *b));
        let visible_count = candidates.partition_point(|candidate| {
            candidate.kind != RoadSurfaceTerrainClipEdgeKind::SpanHandoff
        });
        let provenance_candidates = if visible_count == 0 {
            candidates.as_slice()
        } else {
            &candidates[..visible_count]
        };
        let first = provenance_candidates[0];
        if !provenance_candidates
            .iter()
            .copied()
            .all(|candidate| terrain_clip_source_edges_same_provenance(candidate, first))
        {
            if let Some((start, end)) = canonical_segment
                && let Some(source) = Self::canonical_matching_span_handoff_output_source(
                    provenance_candidates,
                    start,
                    end,
                )
            {
                return TerrainClipOutputSourceSelection::Source(source);
            }
            if let Some((start, end)) = canonical_segment
                && let Some(source) = Self::canonical_same_owner_node_boundary_output_source(
                    provenance_candidates,
                    start,
                    end,
                )
            {
                return TerrainClipOutputSourceSelection::Source(source);
            }
            if let Some((start, end)) = canonical_segment
                && let Some(source) = Self::canonical_same_owner_span_support_output_source(
                    provenance_candidates,
                    start,
                    end,
                )
            {
                return TerrainClipOutputSourceSelection::Source(source);
            }
            let sources = provenance_candidates
                .iter()
                .take(6)
                .map(|candidate| {
                    format!(
                        "{:?}:{}:{}:{:?}",
                        candidate.kind,
                        candidate.source_index,
                        candidate.edge_index,
                        candidate.source
                    )
                })
                .collect::<Vec<_>>()
                .join("|");
            return TerrainClipOutputSourceSelection::Ambiguous(format!(
                "{context}_sources_disagree candidates={sources}"
            ));
        }
        TerrainClipOutputSourceSelection::Source(first)
    }

    fn canonical_matching_span_handoff_output_source(
        candidates: &[TerrainClipSourceEdge],
        start: RoadVec3,
        end: RoadVec3,
    ) -> Option<TerrainClipSourceEdge> {
        if candidates.is_empty()
            || !candidates
                .iter()
                .all(|candidate| candidate.kind == RoadSurfaceTerrainClipEdgeKind::SpanHandoff)
        {
            return None;
        }

        let first_key = Self::span_handoff_boundary_match_key(candidates[0].source)?;
        if !candidates.iter().copied().all(|candidate| {
            Self::span_handoff_boundary_match_key(candidate.source) == Some(first_key)
        }) {
            return None;
        }

        let first = candidates[0];
        Some(TerrainClipSourceEdge {
            start,
            end,
            kind: first.kind,
            source: first.source,
            source_index: first.source_index,
            edge_index: first.edge_index,
        })
    }

    fn span_handoff_boundary_match_key(
        source: RoadSurfaceEarthworkFaceSource,
    ) -> Option<(
        crate::simulation::network::types::EdgeClass,
        super::super::RoadSurfaceEarthworkSupportPolicy,
        super::super::RoadSurfaceBandKind,
        super::super::RoadSurfaceSpanRegionRole,
    )> {
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_class,
            support_policy,
            owner,
            role,
            ..
        } = source
        else {
            return None;
        };
        Some((edge_class, support_policy, owner.kind, role))
    }

    fn canonical_same_owner_node_boundary_output_source(
        candidates: &[TerrainClipSourceEdge],
        start: RoadVec3,
        end: RoadVec3,
    ) -> Option<TerrainClipSourceEdge> {
        let first = *candidates.first()?;
        let (node_id, kind, owner_kind, owner_indices) =
            Self::canonical_node_boundary_owner_set(candidates, first)?;
        let source = if owner_indices.len() == 1 {
            RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id,
                kind,
                owner_kind,
                owner_index: owner_indices[0],
                boundary_source: Some(NodeFootprintBoundarySegmentSource {
                    start: Self::canonical_output_boundary_vertex_source(start),
                    end: Self::canonical_output_boundary_vertex_source(end),
                }),
            }
        } else {
            RoadSurfaceEarthworkFaceSource::NodeSameMaterialBoundaryHandoff {
                node_id,
                kind,
                owner_kind,
                owner_index_a: owner_indices[0],
                owner_index_b: owner_indices[1],
                boundary_source: Some(NodeFootprintBoundarySegmentSource {
                    start: Self::canonical_output_boundary_vertex_source(start),
                    end: Self::canonical_output_boundary_vertex_source(end),
                }),
            }
        };
        Some(TerrainClipSourceEdge {
            start,
            end,
            kind: first.kind,
            source,
            source_index: first.source_index,
            edge_index: first.edge_index,
        })
    }

    fn canonical_same_owner_span_support_output_source(
        candidates: &[TerrainClipSourceEdge],
        start: RoadVec3,
        end: RoadVec3,
    ) -> Option<TerrainClipSourceEdge> {
        let first = *candidates.first()?;
        if first.kind == RoadSurfaceTerrainClipEdgeKind::SpanHandoff {
            return None;
        }
        let first_key = Self::span_support_boundary_owner_key(first)?;
        if !candidates
            .iter()
            .copied()
            .all(|candidate| Self::span_support_boundary_owner_key(candidate) == Some(first_key))
        {
            return None;
        }
        Some(TerrainClipSourceEdge {
            start,
            end,
            kind: first.kind,
            source: first.source,
            source_index: first.source_index,
            edge_index: first.edge_index,
        })
    }

    fn span_support_boundary_owner_key(
        candidate: TerrainClipSourceEdge,
    ) -> Option<(
        RoadSurfaceTerrainClipEdgeKind,
        usize,
        crate::simulation::network::types::EdgeClass,
        super::super::RoadSurfaceEarthworkSupportPolicy,
        super::super::RoadSurfaceSpanBandOwner,
        super::super::RoadSurfaceSpanRegionRole,
    )> {
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_idx,
            edge_class,
            support_policy,
            owner,
            role,
            ..
        } = candidate.source
        else {
            return None;
        };
        Some((
            candidate.kind,
            edge_idx,
            edge_class,
            support_policy,
            owner,
            role,
        ))
    }

    pub(super) fn canonical_same_owner_dust_connector_output_source(
        candidates: &[TerrainClipSourceEdge],
        start: RoadVec3,
        end: RoadVec3,
    ) -> Option<TerrainClipSourceEdge> {
        #[derive(Clone, Copy)]
        struct DustConnectorGroup {
            first: TerrainClipSourceEdge,
        }

        let mut visible_candidates = candidates
            .iter()
            .copied()
            .filter(|candidate| candidate.kind != RoadSurfaceTerrainClipEdgeKind::SpanHandoff)
            .collect::<Vec<_>>();
        visible_candidates.sort_by(|a, b| terrain_clip_source_edge_ordering(*a, *b));
        let first_kind = visible_candidates.first().map(|candidate| candidate.kind)?;
        let mut groups = BTreeMap::<_, DustConnectorGroup>::new();
        for candidate in visible_candidates
            .into_iter()
            .filter(|candidate| candidate.kind == first_kind)
        {
            let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id,
                kind,
                owner_kind,
                owner_index,
                ..
            } = candidate.source
            else {
                continue;
            };
            let key = (node_id, kind.sort_key(), owner_kind, owner_index);
            groups
                .entry(key)
                .or_insert(DustConnectorGroup { first: candidate });
        }
        let owner_groups = groups.into_values().collect::<Vec<_>>();
        let first = owner_groups.first()?.first;
        let candidates = owner_groups
            .iter()
            .map(|group| group.first)
            .collect::<Vec<_>>();
        let (node_id, kind, owner_kind, owner_indices) =
            Self::canonical_node_boundary_owner_set(&candidates, first)?;
        let source = if owner_indices.len() == 1 {
            RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id,
                kind,
                owner_kind,
                owner_index: owner_indices[0],
                boundary_source: Some(NodeFootprintBoundarySegmentSource {
                    start: Self::canonical_output_boundary_vertex_source(start),
                    end: Self::canonical_output_boundary_vertex_source(end),
                }),
            }
        } else {
            RoadSurfaceEarthworkFaceSource::NodeSameMaterialBoundaryHandoff {
                node_id,
                kind,
                owner_kind,
                owner_index_a: owner_indices[0],
                owner_index_b: owner_indices[1],
                boundary_source: Some(NodeFootprintBoundarySegmentSource {
                    start: Self::canonical_output_boundary_vertex_source(start),
                    end: Self::canonical_output_boundary_vertex_source(end),
                }),
            }
        };
        Some(TerrainClipSourceEdge {
            start,
            end,
            kind: first.kind,
            source,
            source_index: first.source_index,
            edge_index: first.edge_index,
        })
    }

    fn canonical_span_handoff_connector_output_source(
        candidates: &[TerrainClipSourceEdge],
        start: RoadVec3,
        end: RoadVec3,
    ) -> Option<TerrainClipSourceEdge> {
        let mut span_candidates = candidates
            .iter()
            .copied()
            .filter(|candidate| candidate.kind == RoadSurfaceTerrainClipEdgeKind::SpanHandoff)
            .collect::<Vec<_>>();
        if span_candidates.is_empty() || span_candidates.len() != candidates.len() {
            return None;
        }
        if !span_candidates.iter().all(|candidate| {
            matches!(
                candidate.source,
                RoadSurfaceEarthworkFaceSource::SpanSupportBoundary { .. }
            )
        }) {
            return None;
        }

        span_candidates.sort_by(|a, b| terrain_clip_source_edge_ordering(*a, *b));
        let first = span_candidates[0];
        Some(TerrainClipSourceEdge {
            start,
            end,
            kind: first.kind,
            source: first.source,
            source_index: first.source_index,
            edge_index: first.edge_index,
        })
    }

    fn canonical_node_boundary_owner_set(
        candidates: &[TerrainClipSourceEdge],
        first: TerrainClipSourceEdge,
    ) -> Option<(
        u32,
        RoadSurfaceVisualNodePieceKind,
        super::super::RoadSurfaceBandKind,
        Vec<usize>,
    )> {
        let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
            node_id,
            kind,
            owner_kind,
            ..
        } = first.source
        else {
            return None;
        };
        let mut owner_indices = BTreeSet::new();
        for candidate in candidates.iter().copied() {
            if candidate.kind != first.kind {
                return None;
            }
            let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id: candidate_node_id,
                kind: candidate_kind,
                owner_kind: candidate_owner_kind,
                owner_index,
                ..
            } = candidate.source
            else {
                return None;
            };
            if candidate_node_id != node_id
                || candidate_kind != kind
                || candidate_owner_kind != owner_kind
            {
                return None;
            }
            owner_indices.insert(owner_index);
        }
        let owner_indices = owner_indices.into_iter().collect::<Vec<_>>();
        (owner_indices.len() <= 2).then_some((node_id, kind, owner_kind, owner_indices))
    }

    fn canonical_output_boundary_vertex_source(
        point: RoadVec3,
    ) -> NodeFootprintBoundaryVertexSource {
        let key = Self::terrain_clip_world_key(point);
        NodeFootprintBoundaryVertexSource::CanonicalBoundaryPoint {
            x_key: key.x_key(),
            z_key: key.z_key(),
            y_mm: SurfaceHeightMmKey::from_m_f64(point.y).as_i64(),
        }
    }
}

pub(super) enum TerrainClipOutputSourceError {
    Missing {
        start: RoadVec3,
        end: RoadVec3,
    },
    Ambiguous {
        start: RoadVec3,
        end: RoadVec3,
        context: String,
    },
}

#[cfg(test)]
mod tests {
    use super::super::super::{
        RoadSurfaceBandKind, RoadSurfaceEarthworkSupportPolicy, RoadSurfaceSpanBandOwner,
        RoadSurfaceSpanRegionRole,
    };
    use super::*;
    use crate::simulation::network::types::EdgeClass;

    #[test]
    fn terrain_clip_material_corner_keeps_both_exact_source_endpoints() {
        let source =
            |owner_kind, owner_index, a, b| RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary {
                node_id: 796,
                kind: RoadSurfaceVisualNodePieceKind::JunctionN,
                owner_kind,
                owner_index,
                boundary_source: Some(NodeFootprintBoundarySegmentSource {
                    start: RoadSurfaceSystem::canonical_output_boundary_vertex_source(a),
                    end: RoadSurfaceSystem::canonical_output_boundary_vertex_source(b),
                }),
            };
        let low = RoadVec3::new(-647.867727, 95.777, 1458.000701);
        let high = RoadVec3::new(-647.8677256392291, 95.897, 1458.000702558227);
        let before = RoadVec3::new(-648.295579, 95.779, 1457.533843);
        let after = RoadVec3::new(-645.956995, 95.774, 1460.188692);
        let a = RoadSurfaceTerrainClipSourceEdge {
            start: before,
            end: low,
            kind: RoadSurfaceTerrainClipEdgeKind::FootprintBoundary,
            source: source(RoadSurfaceBandKind::Carriageway, 9, before, low),
        };
        let b = RoadSurfaceTerrainClipSourceEdge {
            start: high,
            end: after,
            kind: RoadSurfaceTerrainClipEdgeKind::SidewalkOuter,
            source: source(RoadSurfaceBandKind::Sidewalk, 11, high, after),
        };
        let cap = RoadSurfaceSystem::terrain_clip_node_material_corner_connector(a, b).unwrap();
        assert_eq!((cap.start, cap.end), (low, high));
        let mut cliff = b;
        cliff.start.y += 0.493;
        assert!(RoadSurfaceSystem::terrain_clip_node_material_corner_connector(a, cliff).is_none());
        let mut unrelated = b;
        if let RoadSurfaceEarthworkFaceSource::NodeFootprintBoundary { node_id, .. } =
            &mut unrelated.source
        {
            *node_id += 1;
        }
        assert!(
            RoadSurfaceSystem::terrain_clip_node_material_corner_connector(a, unrelated).is_none()
        );
        let mut gap = b;
        gap.start.x += 0.01;
        assert!(RoadSurfaceSystem::terrain_clip_node_material_corner_connector(a, gap).is_none());
    }

    #[test]
    fn terrain_clip_output_canonicalizes_matching_span_handoff_covered_segment_source() {
        let start = RoadVec3::new(0.0, 10.0, 0.0);
        let end = RoadVec3::new(1.0, 10.0, 0.0);
        let candidates = vec![
            span_handoff_source_edge(
                RoadVec3::new(-0.5, 10.0, 0.0),
                RoadVec3::new(1.5, 10.0, 0.0),
                0,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                0,
                38,
            ),
            span_handoff_source_edge(
                RoadVec3::new(-0.5, 10.0, 0.0),
                RoadVec3::new(1.5, 10.0, 0.0),
                1,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                1,
                5,
            ),
        ];

        let source = match RoadSurfaceSystem::terrain_clip_output_source_for_segment(
            start,
            end,
            &candidates,
        ) {
            TerrainClipOutputSourceSelection::Source(source) => source,
            TerrainClipOutputSourceSelection::Missing => {
                panic!("matching span handoff source must be present")
            }
            TerrainClipOutputSourceSelection::Ambiguous(context) => {
                panic!("matching span handoff source must be canonical: {context}")
            }
        };

        assert_eq!(source.start, start);
        assert_eq!(source.end, end);
        assert_eq!(source.kind, RoadSurfaceTerrainClipEdgeKind::SpanHandoff);
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_idx,
            owner,
            role,
            ..
        } = source.source
        else {
            panic!("span handoff segment must keep span support provenance");
        };
        assert_eq!(edge_idx, 0);
        assert_eq!(owner.source_band_index, 0);
        assert_eq!(owner.kind, RoadSurfaceBandKind::Sidewalk);
        assert_eq!(role, RoadSurfaceSpanRegionRole::NonRoad);
    }

    #[test]
    fn terrain_clip_output_canonicalizes_opposite_side_span_handoff_covered_segment_source() {
        let start = RoadVec3::new(-65.995, 0.153, 73.114);
        let end = RoadVec3::new(-64.678, 0.153, 72.818);
        let candidates = vec![
            span_handoff_source_edge(
                start,
                end,
                3,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                3,
                36,
            ),
            span_handoff_source_edge(
                start,
                end,
                7,
                5,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                7,
                0,
            ),
        ];

        let source = match RoadSurfaceSystem::terrain_clip_output_source_for_segment(
            start,
            end,
            &candidates,
        ) {
            TerrainClipOutputSourceSelection::Source(source) => source,
            TerrainClipOutputSourceSelection::Missing => {
                panic!("opposite-side span handoff source must be present")
            }
            TerrainClipOutputSourceSelection::Ambiguous(context) => {
                panic!("opposite-side span handoff source must be canonical: {context}")
            }
        };

        assert_eq!(source.start, start);
        assert_eq!(source.end, end);
        assert_eq!(source.kind, RoadSurfaceTerrainClipEdgeKind::SpanHandoff);
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_idx,
            owner,
            role,
            ..
        } = source.source
        else {
            panic!("span handoff segment must keep span support provenance");
        };
        assert_eq!(edge_idx, 3);
        assert_eq!(owner.kind, RoadSurfaceBandKind::Sidewalk);
        assert_eq!(role, RoadSurfaceSpanRegionRole::NonRoad);
    }

    #[test]
    fn terrain_clip_output_rejects_different_material_span_handoff_covered_segment_source() {
        let start = RoadVec3::new(0.0, 10.0, 0.0);
        let end = RoadVec3::new(1.0, 10.0, 0.0);
        let candidates = vec![
            span_handoff_source_edge(
                start,
                end,
                3,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                3,
                36,
            ),
            span_handoff_source_edge(
                start,
                end,
                7,
                4,
                RoadSurfaceBandKind::CurbOrShoulder,
                RoadSurfaceSpanRegionRole::CurbOrShoulder,
                7,
                0,
            ),
        ];

        let TerrainClipOutputSourceSelection::Ambiguous(context) =
            RoadSurfaceSystem::terrain_clip_output_source_for_segment(start, end, &candidates)
        else {
            panic!("different-material span handoff sources must stay ambiguous");
        };
        assert!(
            context.contains("sources_disagree"),
            "ambiguous span handoff diagnostic should name provenance disagreement: {context}"
        );
    }

    #[test]
    fn terrain_clip_output_canonicalizes_span_handoff_only_connector_source() {
        let start = RoadVec3::new(0.0, 10.0, 0.0);
        let end = RoadVec3::new(1.0, 10.0, 0.0);
        let candidates = vec![
            span_handoff_source_edge(
                RoadVec3::new(-0.5, 10.0, 0.0),
                RoadVec3::new(0.5, 10.0, 0.0),
                0,
                3,
                RoadSurfaceBandKind::Carriageway,
                RoadSurfaceSpanRegionRole::Asphalt,
                0,
                15,
            ),
            span_handoff_source_edge(
                RoadVec3::new(-0.5, 10.0, 0.0),
                RoadVec3::new(0.5, 10.0, 0.0),
                0,
                4,
                RoadSurfaceBandKind::CurbOrShoulder,
                RoadSurfaceSpanRegionRole::CurbOrShoulder,
                0,
                16,
            ),
            span_handoff_source_edge(
                RoadVec3::new(0.5, 10.0, 0.0),
                RoadVec3::new(1.5, 10.0, 0.0),
                4,
                2,
                RoadSurfaceBandKind::Carriageway,
                RoadSurfaceSpanRegionRole::Asphalt,
                4,
                3,
            ),
        ];

        let source = match RoadSurfaceSystem::terrain_clip_output_dust_connector_source(
            start,
            end,
            &candidates,
        ) {
            TerrainClipOutputSourceSelection::Source(source) => source,
            TerrainClipOutputSourceSelection::Missing => {
                panic!("span handoff connector source must be present")
            }
            TerrainClipOutputSourceSelection::Ambiguous(context) => {
                panic!("span handoff connector source must be canonical: {context}")
            }
        };

        assert_eq!(source.start, start);
        assert_eq!(source.end, end);
        assert_eq!(source.kind, RoadSurfaceTerrainClipEdgeKind::SpanHandoff);
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_idx,
            owner,
            role,
            start_section_index,
            end_section_index,
            ..
        } = source.source
        else {
            panic!("span handoff connector must keep span support provenance");
        };
        assert_eq!(edge_idx, 0);
        assert_eq!(owner.source_band_index, 3);
        assert_eq!(owner.kind, RoadSurfaceBandKind::Carriageway);
        assert_eq!(role, RoadSurfaceSpanRegionRole::Asphalt);
        assert_eq!(start_section_index, 0);
        assert_eq!(end_section_index, 0);
    }

    #[test]
    fn terrain_clip_output_canonicalizes_same_span_support_connector_source() {
        let start = RoadVec3::new(0.0, 10.0, 0.0);
        let end = RoadVec3::new(1.0, 10.0, 0.0);
        let candidates = vec![
            span_support_source_edge(
                RoadVec3::new(-0.5, 10.0, 0.0),
                RoadVec3::new(0.5, 10.0, 0.0),
                RoadSurfaceTerrainClipEdgeKind::SidewalkOuter,
                6,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                18,
                19,
                23.92626,
                24.041697,
                4,
                9,
            ),
            span_support_source_edge(
                RoadVec3::new(0.5, 10.0, 0.0),
                RoadVec3::new(1.5, 10.0, 0.0),
                RoadSurfaceTerrainClipEdgeKind::SidewalkOuter,
                6,
                0,
                RoadSurfaceBandKind::Sidewalk,
                RoadSurfaceSpanRegionRole::NonRoad,
                20,
                21,
                24.043888,
                26.04304,
                3,
                10,
            ),
        ];

        let source = match RoadSurfaceSystem::terrain_clip_output_dust_connector_source(
            start,
            end,
            &candidates,
        ) {
            TerrainClipOutputSourceSelection::Source(source) => source,
            TerrainClipOutputSourceSelection::Missing => {
                panic!("same span-support connector source must be present")
            }
            TerrainClipOutputSourceSelection::Ambiguous(context) => {
                panic!("same span-support connector source must be canonical: {context}")
            }
        };

        assert_eq!(source.start, start);
        assert_eq!(source.end, end);
        assert_eq!(source.kind, RoadSurfaceTerrainClipEdgeKind::SidewalkOuter);
        let RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
            edge_idx,
            owner,
            role,
            ..
        } = source.source
        else {
            panic!("visible span support connector must keep span support provenance");
        };
        assert_eq!(edge_idx, 6);
        assert_eq!(owner.source_band_index, 0);
        assert_eq!(owner.kind, RoadSurfaceBandKind::Sidewalk);
        assert_eq!(role, RoadSurfaceSpanRegionRole::NonRoad);
    }

    #[allow(clippy::too_many_arguments)]
    fn span_handoff_source_edge(
        start: RoadVec3,
        end: RoadVec3,
        edge_idx: usize,
        source_band_index: usize,
        kind: RoadSurfaceBandKind,
        role: RoadSurfaceSpanRegionRole,
        source_index: usize,
        edge_index: usize,
    ) -> TerrainClipSourceEdge {
        TerrainClipSourceEdge {
            start,
            end,
            kind: RoadSurfaceTerrainClipEdgeKind::SpanHandoff,
            source: RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
                edge_idx,
                edge_class: EdgeClass::Standard,
                support_policy: RoadSurfaceEarthworkSupportPolicy::StandardFullGroundedSpan,
                owner: RoadSurfaceSpanBandOwner {
                    source_band_index,
                    kind,
                },
                role,
                start_section_index: 0,
                end_section_index: 0,
                start_s_m: 0.0,
                end_s_m: 0.0,
            },
            source_index,
            edge_index,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn span_support_source_edge(
        start: RoadVec3,
        end: RoadVec3,
        kind: RoadSurfaceTerrainClipEdgeKind,
        edge_idx: usize,
        source_band_index: usize,
        band_kind: RoadSurfaceBandKind,
        role: RoadSurfaceSpanRegionRole,
        start_section_index: usize,
        end_section_index: usize,
        start_s_m: f32,
        end_s_m: f32,
        source_index: usize,
        edge_index: usize,
    ) -> TerrainClipSourceEdge {
        TerrainClipSourceEdge {
            start,
            end,
            kind,
            source: RoadSurfaceEarthworkFaceSource::SpanSupportBoundary {
                edge_idx,
                edge_class: EdgeClass::Standard,
                support_policy: RoadSurfaceEarthworkSupportPolicy::StandardFullGroundedSpan,
                owner: RoadSurfaceSpanBandOwner {
                    source_band_index,
                    kind: band_kind,
                },
                role,
                start_section_index,
                end_section_index,
                start_s_m,
                end_s_m,
            },
            source_index,
            edge_index,
        }
    }
}
