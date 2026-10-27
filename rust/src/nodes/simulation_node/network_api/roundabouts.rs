// SPDX-License-Identifier: GPL-2.0-only

//! Immutable single-lane roundabout requests for the tool and future map importers.

use super::*;
use crate::simulation::network::roundabouts::{RoundaboutLayout, RoundaboutRequest};

fn request(
    center: Vector3,
    diameter: f32,
    angles: PackedFloat32Array,
    replace_node: i32,
) -> RoundaboutRequest {
    RoundaboutRequest {
        center,
        diameter,
        angles: angles.as_slice().to_vec(),
        replace: u32::try_from(replace_node).ok(),
    }
}

#[godot_api(secondary)]
impl SimulationNode {
    /// Compiles a complete replacement preview. Negative node ID builds on empty ground;
    /// otherwise the existing junction's 3–6 arm angles are retained. Diameter spans asphalt.
    #[func]
    pub fn preview_roundabout(
        &mut self,
        center: Vector3,
        diameter: f32,
        angles: PackedFloat32Array,
        replace_node: i32,
    ) -> VarDictionary {
        self.pocket_preview_revision = self.pocket_preview_revision.wrapping_add(1).max(1);
        let core = self.lock_core();
        let request = request(center, diameter, angles, replace_node);
        let prepared = RoundaboutLayout::prepare(&core.region_graph, request.clone())
            .and_then(|layout| core.roundabout_layout_refusal(&layout).map_or(Ok(()), Err));
        let preview = prepared.and_then(|()| {
            core.transit_network
                .road_surface
                .compile_roundabout_preview(&core.region_graph, &core.heightmap, request)
        });
        match preview {
            Ok((scene, roads)) => self.local_layout_preview_result(&core, scene, &roads),
            Err(reason) => vdict! { "is_valid": false, "invalid_reason": reason },
        }
    }

    /// Queues one atomic preset, including all arms and ring edges. Poll ordinary road completion.
    #[func]
    pub fn place_roundabout(
        &mut self,
        center: Vector3,
        diameter: f32,
        angles: PackedFloat32Array,
        replace_node: i32,
    ) -> i64 {
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        let _ = self
            .cmd_tx
            .send(crate::nodes::sim::core::SimCommand::PlaceRoundabout {
                request: request(center, diameter, angles, replace_node),
                completion,
            });
        self.road_commit_request_id
    }
}
