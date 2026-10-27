// SPDX-License-Identifier: GPL-2.0-only

//! Ramp stub authoring API; the same request can be supplied by a future link-way importer.

use super::*;
use crate::simulation::network::ramps::{RampLayout, RampRequest};

fn request(
    edge: i64,
    station: f32,
    forward: bool,
    exit: bool,
    points: PackedVector3Array,
    length: f32,
    taper: f32,
) -> RampRequest {
    RampRequest {
        edge: edge as usize,
        station,
        forward,
        exit,
        points: points.as_slice().to_vec(),
        length,
        taper,
    }
}

#[godot_api(secondary)]
impl SimulationNode {
    /// Resolves the ramp tool's first click through the existing road R-tree selection.
    #[func]
    pub fn get_ramp_anchor(&self, point: Vector3) -> VarDictionary {
        let core = self.lock_core();
        let id = core.get_hovered_edge_internal(point.x, point.z);
        let Some(edge) = usize::try_from(id)
            .ok()
            .and_then(|id| core.region_graph.get_edge(id))
            .filter(|edge| !edge.deleted && edge.median.is_some() && edge.lane_tapers.is_none())
        else {
            return VarDictionary::new();
        };
        let start = core.region_graph.node(edge.start_node).pos;
        let end = core.region_graph.node(edge.end_node).pos;
        let delta = end - start;
        let length = delta.length();
        if length < 1.0 {
            return VarDictionary::new();
        }
        let station = (point - start).dot(delta / length).clamp(0.0, length);
        vdict! { "edge": id, "station": station, "position": start + delta * (station / length) }
    }

    /// Compiles a one-way ramp stub and its outer auxiliary lane on a straight divided edge.
    /// Points start at the main-road centreline and run outward, including for an entrance ramp.
    #[func]
    #[allow(clippy::too_many_arguments)]
    pub fn preview_ramp(
        &mut self,
        edge: i64,
        station: f32,
        forward: bool,
        exit: bool,
        points: PackedVector3Array,
        length: f32,
        taper: f32,
    ) -> VarDictionary {
        self.pocket_preview_revision = self.pocket_preview_revision.wrapping_add(1).max(1);
        let core = self.lock_core();
        let request = request(edge, station, forward, exit, points, length, taper);
        let prepared =
            RampLayout::prepare(&core.region_graph, request.clone()).and_then(|layout| {
                core.road_layout_corridor_refusal(layout.corridors())
                    .map_or(Ok(()), Err)
            });
        let preview = prepared.and_then(|()| {
            core.transit_network.road_surface.compile_ramp_preview(
                &core.region_graph,
                &core.heightmap,
                request,
            )
        });
        match preview {
            Ok((scene, roads)) => self.local_layout_preview_result(&core, scene, &roads),
            Err(reason) => vdict! { "is_valid": false, "invalid_reason": reason },
        }
    }

    /// Queues one undoable ramp transaction. Station is measured from the selected edge's start.
    #[func]
    #[allow(clippy::too_many_arguments)]
    pub fn place_ramp(
        &mut self,
        edge: i64,
        station: f32,
        forward: bool,
        exit: bool,
        points: PackedVector3Array,
        length: f32,
        taper: f32,
    ) -> i64 {
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        let _ = self
            .cmd_tx
            .send(crate::nodes::sim::core::SimCommand::PlaceRamp {
                request: request(edge, station, forward, exit, points, length, taper),
                completion,
            });
        self.road_commit_request_id
    }
}
