// SPDX-License-Identifier: GPL-2.0-only

//! Nonmutating local endpoint-pocket preview using the authoritative topology operation.

use super::*;
use crate::simulation::network::pockets::{PocketLayout, PocketRequest};
use crate::simulation::network::render::road::preview::{
    RoadJunctionPreview, RoadPreviewRenderInput,
};
use crate::simulation::network::surface::PlannedRoadSurfaceQuery;
use std::sync::Arc;

impl RoadSurfaceSystem {
    /// Ramp stubs share bounded surface and terrain replacement with other topology edits.
    pub(crate) fn compile_ramp_preview(
        &self,
        source: &RegionGraph,
        terrain: &TerrainSystem,
        request: crate::simulation::network::ramps::RampRequest,
    ) -> Result<(RoadJunctionPreview, Arc<PlannedRoadSurfaceQuery>), &'static str> {
        use crate::simulation::network::ramps::RampLayout;
        let layout = RampLayout::prepare(source, request.clone())?;
        self.compile_local_layout_preview(
            source,
            terrain,
            layout.originals.iter().map(|(id, _)| *id).collect(),
            |graph, _, edges| {
                let request = crate::simulation::network::ramps::RampRequest {
                    edge: edges[&request.edge],
                    ..request
                };
                Ok(RampLayout::prepare(graph, request)?.apply(graph))
            },
        )
    }

    /// Roundabouts use the same local surface, terrain ownership and render pipeline as endpoint edits.
    pub(crate) fn compile_roundabout_preview(
        &self,
        source: &RegionGraph,
        terrain: &TerrainSystem,
        request: crate::simulation::network::roundabouts::RoundaboutRequest,
    ) -> Result<(RoadJunctionPreview, Arc<PlannedRoadSurfaceQuery>), &'static str> {
        use crate::simulation::network::roundabouts::RoundaboutLayout;
        let layout = RoundaboutLayout::prepare(source, request.clone())?;
        self.compile_local_layout_preview(
            source,
            terrain,
            layout.originals.iter().map(|(id, _)| *id).collect(),
            |graph, nodes, _| {
                let mut local = request;
                local.replace = local.replace.map(|id| nodes[&id]);
                Ok(RoundaboutLayout::prepare(graph, local)?.apply(graph))
            },
        )
    }

    /// Compiles a fixed incidence halo, independent of background roads/buildings/agents.
    /// The same layout, junction finalizer and sampled bands are used by commit.
    pub(crate) fn compile_pocket_preview(
        &self,
        source: &RegionGraph,
        terrain: &TerrainSystem,
        request: PocketRequest,
    ) -> Result<(RoadJunctionPreview, Arc<PlannedRoadSurfaceQuery>), &'static str> {
        let layout = PocketLayout::prepare(source, request)?;
        self.compile_local_layout_preview(
            source,
            terrain,
            layout.originals.iter().map(|(id, _)| *id).collect(),
            |graph, _, edges| {
                let local_request = PocketRequest {
                    edge: *edges.get(&request.edge).ok_or("invalid_road")?,
                    ..request
                };
                Ok(PocketLayout::prepare(graph, local_request)?.apply(graph))
            },
        )
    }

    /// Shared bounded preview for topology presets; ordinary pocket previews retain the same compiler.
    pub(crate) fn compile_local_layout_preview(
        &self,
        source: &RegionGraph,
        terrain: &TerrainSystem,
        mut original_ids: Vec<usize>,
        apply: impl FnOnce(
            &mut RegionGraph,
            &HashMap<u32, u32>,
            &HashMap<usize, usize>,
        ) -> Result<Vec<usize>, &'static str>,
    ) -> Result<(RoadJunctionPreview, Arc<PlannedRoadSurfaceQuery>), &'static str> {
        let mut graph = RegionGraph::new();
        let mut nodes = HashMap::new();
        let mut edges = HashMap::new();
        original_ids.sort_unstable();
        for id in original_ids {
            Self::copy_validation_graph_edge(&mut graph, source, &mut nodes, &mut edges, id, None);
        }
        let required_edges: Vec<_> = edges.values().copied().collect();
        let required_nodes: Vec<_> = nodes.values().copied().collect();
        Self::complete_validation_surface_context(
            &mut graph,
            source,
            &mut nodes,
            &mut edges,
            &required_edges,
            &required_nodes,
            None,
        );
        let mut ordered_nodes: Vec<_> = nodes.iter().map(|(&a, &b)| (a, b)).collect();
        ordered_nodes.sort_unstable();
        for (source_id, local) in ordered_nodes {
            let node = source.node(source_id);
            for (&(from, lane), targets) in &node.lane_connections {
                let Some(&from) = edges.get(&from) else {
                    continue;
                };
                graph.nodes[local as usize]
                    .lane_connections
                    .insert((from, lane), Vec::new());
                for &(to, index) in targets {
                    if let Some(&to) = edges.get(&to) {
                        graph.add_lane_connection(local, from, lane, to, index);
                    }
                }
            }
            for (&id, &enabled) in &node.crosswalk_overrides {
                if let Some(&id) = edges.get(&id) {
                    graph.set_crosswalk_override(local, id, enabled);
                }
            }
        }
        let changed = apply(&mut graph, &nodes, &edges)?;
        let mut network = TransitNetwork::new();
        network.bulk_dirty_edges.extend(changed.iter().copied());
        let finalized = network.finalize_road_geometry(&mut graph);
        let mut surface = Self::new_with_chunk_grid(
            self.chunk_span_m,
            self.chunk_origin_x_m,
            self.chunk_origin_z_m,
        );
        surface.node_validation_logging_enabled = crate::debug::Channel::Road.enabled();
        let mut required: Vec<_> = finalized.dirty_edges.iter().copied().collect();
        required.sort_unstable();
        let mut required_nodes: Vec<_> = finalized.affected_nodes.iter().copied().collect();
        required_nodes.sort_unstable();
        surface.compile_validation_neighborhood_with_reason(
            &graph,
            terrain,
            &required,
            &required_nodes,
            RoadSurfaceCompileReason::CommitValidator,
        );
        if Self::explicit_surface_validation_failure(&surface, &required, &required_nodes).is_some()
        {
            crate::debug_log!(
                "road",
                "local_layout_preview_failed spans={:?} nodes={:?} reason={:?}",
                surface.last_failed_span_ids,
                surface.last_failed_node_ids,
                surface.last_compile_failure_label
            );
            return Err("surface_geometry_invalid");
        }
        let replaced_nodes: HashSet<u32> = nodes
            .iter()
            .filter_map(|(&source, &local)| required_nodes.contains(&local).then_some(source))
            .collect();
        // Prospective live identities preserve road/site ownership in the terrain compiler.
        // Only the local excerpt is copied; new nodes/edges append in the same order as commit.
        let mut edge_ids = vec![usize::MAX; graph.edge_count()];
        let mut node_ids = vec![u32::MAX; graph.node_count()];
        for (&source, &local) in &edges {
            edge_ids[local] = source;
        }
        for (&source, &local) in &nodes {
            node_ids[local as usize] = source;
        }
        let mut next_edge = source.edge_count();
        for id in &mut edge_ids {
            if *id == usize::MAX {
                *id = next_edge;
                next_edge += 1;
            }
        }
        let mut next_node = source.node_count() as u32;
        for id in &mut node_ids {
            if *id == u32::MAX {
                *id = next_node;
                next_node += 1;
            }
        }
        let replaced_edges = edges
            .iter()
            .filter_map(|(&source, &local)| {
                (graph.edge(local).deleted
                    || surface.compiled_visual_span_pieces.contains_key(&local))
                .then_some(source)
            })
            .collect();
        graph.set_placement_identities(edge_ids.clone(), node_ids.clone());
        let roads = Arc::new(PlannedRoadSurfaceQuery::capture(
            &graph,
            &surface,
            &edge_ids,
            &node_ids,
            &replaced_edges,
            &replaced_nodes,
        ));
        let scene = RoadPreviewRenderInput::new(
            graph,
            surface,
            edges,
            replaced_nodes.clone(),
            replaced_nodes,
            required.into_iter().collect(),
            required_nodes.into_iter().collect(),
        )
        .replace_whole_neighborhood()
        .render(terrain, self)
        .ok_or("surface_geometry_invalid")?;
        Ok((scene, roads))
    }
}
