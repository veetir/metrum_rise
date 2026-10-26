// SPDX-License-Identifier: GPL-2.0-only

//! Network, road-tool, and traffic-editing Godot API methods.

use super::*;

mod crossings;
mod ramps;
mod road_tool;
mod roundabouts;

#[godot_api(secondary)]
impl SimulationNode {
    /// Selects the separator for subsequent immutable tool requests: 0 none, 1 paint,
    /// 2 kerbed grass, 3 kerbed paving. Returns an empty string on success, else a reason.
    #[func]
    pub fn set_road_tool_median(&mut self, style: i32, width_m: f32) -> GString {
        match crate::simulation::network::graph::RoadMedian::from_tool(style, width_m) {
            Ok(median) => {
                self.road_tool_median = median;
                GString::new()
            }
            Err(reason) => reason.into(),
        }
    }

    /// Selects a tunnel profile without using the crossing layer to infer structure.
    /// Grounded endpoints retain the submitted interior heights in all validation paths.
    #[func]
    pub fn set_road_tool_tunnel(&mut self, tunnel: bool) {
        self.road_tool_tunnel = tunnel;
    }

    /// Selects an explicit crossing layer; higher layers must pass above lower layers.
    #[func]
    pub fn set_road_tool_crossing_level(&mut self, level: i32) -> GString {
        let Ok(level) = i16::try_from(level) else {
            return "crossing_level_out_of_range".into();
        };
        self.road_tool_crossing_level = level;
        GString::new()
    }

    /// Queues an in-place lane/median edit using the current tool selection; poll the returned
    /// identity with `get_road_commit_result`, exactly as for drawing a road.
    #[func]
    pub fn change_road_cross_section(&mut self, edge: i32, fwd: i32, bkw: i32) -> i64 {
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        if edge < 0 || !(0..=4).contains(&fwd) || !(0..=4).contains(&bkw) {
            let _ = completion.try_send(crate::nodes::sim::core::RoadCommitAck {
                committed: false,
                reason: "invalid_cross_section".into(),
                detail: "invalid_cross_section".into(),
            });
        } else {
            let _ = self.cmd_tx.send(
                crate::nodes::sim::core::SimCommand::ChangeRoadCrossSection {
                    edge: edge as usize,
                    lanes: (fwd as u8, bkw as u8),
                    median: self.road_tool_median,
                    completion,
                },
            );
        }
        self.road_commit_request_id
    }

    /// Queues bus-only lane masks (bit zero innermost, forward then backward).
    /// Importers translate busway/lanes:bus tags to these physical lane permissions.
    /// Cars may cross a reserved ribbon through legal right-turn junction connectors.
    #[func]
    pub fn change_road_bus_lanes(
        &mut self,
        edge: i32,
        forward_mask: i32,
        backward_mask: i32,
    ) -> i64 {
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        if let (Ok(edge), Ok(fwd), Ok(bkw)) = (
            usize::try_from(edge),
            u8::try_from(forward_mask),
            u8::try_from(backward_mask),
        ) {
            let _ = self
                .cmd_tx
                .send(crate::nodes::sim::core::SimCommand::ChangeRoadBusLanes {
                    edge,
                    masks: [fwd, bkw],
                    completion,
                });
        } else {
            let _ = completion.try_send(crate::nodes::sim::core::RoadCommitAck {
                committed: false,
                reason: "invalid_bus_lane_mask".into(),
                detail: "invalid_bus_lane_mask".into(),
            });
        }
        self.road_commit_request_id
    }

    /// Adds road-end turn lanes through a hidden transition node; poll normal road completion.
    #[func]
    pub fn change_road_pocket(
        &mut self,
        edge: i32,
        at_start: bool,
        left: bool,
        right: bool,
        pocket_m: f32,
        taper_m: f32,
    ) -> i64 {
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        if let Ok(edge) = usize::try_from(edge) {
            let _ = self
                .cmd_tx
                .send(crate::nodes::sim::core::SimCommand::ChangeRoadPocket {
                    edge,
                    at_start,
                    left,
                    right,
                    pocket_m,
                    taper_m,
                    completion,
                });
        } else {
            let _ = completion.try_send(crate::nodes::sim::core::RoadCommitAck {
                committed: false,
                reason: "invalid_road".into(),
                detail: "invalid_road".into(),
            });
        }
        self.road_commit_request_id
    }

    /// Resolves the nearest stored road end and reports its authored pocket dimensions.
    #[func]
    pub fn get_road_pocket_selection(&self, world_x: f32, world_z: f32) -> VarDictionary {
        let core = self.lock_core();
        let id = core.get_hovered_edge_internal(world_x, world_z);
        let Some(edge) = usize::try_from(id)
            .ok()
            .and_then(|id| core.region_graph.get_edge(id))
            .filter(|e| !e.deleted)
        else {
            return VarDictionary::new();
        };
        let p = Vector2::new(world_x, world_z);
        let start = core.region_graph.node(edge.start_node).pos;
        let end = core.region_graph.node(edge.end_node).pos;
        let mut at_start = p.distance_squared_to(Vector2::new(start.x, start.z))
            < p.distance_squared_to(Vector2::new(end.x, end.z));
        let mut left = false;
        let mut right = false;
        let mut length = 80.0;
        let mut taper = 35.0;
        if let Some((profile_end, profile)) = edge.pocket_taper() {
            at_start = profile_end == 1;
            left = profile.left_pocket;
            right = profile.right_pocket;
            taper = profile.length_m;
            length = edge.physical_length
                - profile.length_m
                - if at_start {
                    edge.start_clip
                } else {
                    edge.end_clip
                };
        }
        vdict! {"edge":id,"at_start":at_start,"left":left,"right":right,"pocket_m":length,"taper_m":taper}
    }

    /// Returns a nonmutating compiled endpoint preview; dimensions use the same operation as commit.
    #[func]
    pub fn preview_road_pocket(
        &mut self,
        edge: i32,
        at_start: bool,
        left: bool,
        right: bool,
        pocket_m: f32,
        taper_m: f32,
    ) -> VarDictionary {
        self.pocket_preview_revision = self.pocket_preview_revision.wrapping_add(1).max(1);
        let core = self.lock_core();
        let mut result = VarDictionary::new();
        let request = crate::simulation::network::pockets::PocketRequest {
            edge: usize::try_from(edge).unwrap_or(usize::MAX),
            at_start,
            left,
            right,
            length: pocket_m,
            taper: taper_m,
        };
        if let Ok(layout) =
            crate::simulation::network::pockets::PocketLayout::prepare(&core.region_graph, request)
            && let Some(reason) = core.pocket_layout_refusal(&layout)
        {
            result.set("is_valid", false);
            result.set("invalid_reason", reason);
            return result;
        }
        match core.transit_network.road_surface.compile_pocket_preview(
            &core.region_graph,
            &core.heightmap,
            request,
        ) {
            Ok((scene, roads)) => {
                result = self.local_layout_preview_result(&core, scene, &roads);
            }
            Err(reason) => {
                result.set("is_valid", false);
                result.set("invalid_reason", reason);
            }
        }
        result.set(
            "surface_generation",
            self.get_road_tool_surface_generation(),
        );
        result
    }

    /// Returns the authored lane/median selection of a live road for inspection.
    #[func]
    pub fn get_road_cross_section(&self, edge: i32) -> VarDictionary {
        let core = self.lock_core();
        let Some(edge) = usize::try_from(edge)
            .ok()
            .and_then(|id| core.region_graph.get_edge(id))
            .filter(|edge| !edge.deleted)
        else {
            return VarDictionary::new();
        };
        vdict! { "fwd_lanes": i32::from(edge.fwd_lanes), "bkw_lanes": i32::from(edge.bkw_lanes),
        "bus_forward_mask": i32::from(edge.bus_lanes[0]),
        "bus_backward_mask": i32::from(edge.bus_lanes[1]),
        "crossing_level": edge.crossing_level,
        "median_style": edge.median.map_or(0, |median| median.style as i32),
        "median_width": edge.median.map_or(0.0, |median| median.width_m) }
    }

    /// Returns the ID of the edge hovered by the mouse.
    #[func]
    pub fn get_hovered_edge(&self, world_x: f32, world_z: f32) -> i32 {
        self.lock_core().get_hovered_edge_internal(world_x, world_z)
    }

    /// Returns the 3D geometry for a road edge.
    #[func]
    pub fn get_edge_geometry_3d(&self, edge_idx: i32) -> PackedVector3Array {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return PackedVector3Array::new();
        }
        let edge = core.region_graph.edge(edge_idx as usize);
        PackedVector3Array::from_iter(edge.physical_geometry.iter().cloned())
    }

    /// Returns the width of a specific road edge.
    #[func]
    pub fn get_edge_width(&self, edge_idx: i32) -> f32 {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return 6.0;
        }
        core.region_graph.edge(edge_idx as usize).width
    }

    /// Adds a new road segment to the network.
    #[func]
    pub fn add_road(&mut self, points: PackedVector3Array, fwd_lanes: i32, bkw_lanes: i32) {
        self.add_road_with_snap(points, fwd_lanes, bkw_lanes, true);
    }

    /// Queues a road and returns its completion identity; this does not report acceptance.
    /// Only the latest request retains feedback, polled once with `get_road_commit_result`.
    #[func]
    pub fn add_road_with_snap(
        &mut self,
        points: PackedVector3Array,
        fwd_lanes: i32,
        bkw_lanes: i32,
        snap_to_existing_roads: bool,
    ) -> i64 {
        // Send to the background thread so the Godot main thread is never blocked
        // by lane rebuilding and zoning work. The command wakes the existing simulation thread
        // without waiting for, or advancing, its next movement tick.
        let road_debug = crate::debug::Channel::Road.enabled();
        let total_start = road_debug.then(Instant::now);
        let point_count = points.len();
        let clone_start = road_debug.then(Instant::now);
        let points = points.to_vec();
        let clone_ms = clone_start
            .map(|start| start.elapsed().as_secs_f64() * 1000.0)
            .unwrap_or(0.0);
        let edit_plan = self
            .road_preview_result
            .read()
            .expect("road preview result lock poisoned")
            .as_ref()
            .and_then(RoadPreviewSnapshot::edit_plan);
        let send_start = road_debug.then(Instant::now);
        let (completion, result) = std::sync::mpsc::sync_channel(1);
        self.road_commit_request_id = self.road_commit_request_id.wrapping_add(1).max(1);
        self.road_commit_result = Some(result);
        let send_ok = self
            .cmd_tx
            .send(crate::nodes::sim::core::SimCommand::AddRoad {
                tunnel: self.road_tool_tunnel,
                crossing_level: self.road_tool_crossing_level,
                median: self.road_tool_median,
                points,
                fwd_lanes,
                bkw_lanes,
                snap_to_existing_roads,
                edit_plan,
                enqueued_at: Instant::now(),
                completion,
            })
            .is_ok();
        let send_ms = send_start
            .map(|start| start.elapsed().as_secs_f64() * 1000.0)
            .unwrap_or(0.0);
        if road_debug {
            debug_log!(
                "road",
                "add_road_bridge points={} fwd_lanes={} bkw_lanes={} clone_ms={:.3} send_ms={:.3} send_ok={} total_ms={:.3}",
                point_count,
                fwd_lanes,
                bkw_lanes,
                clone_ms,
                send_ms,
                send_ok,
                total_start
                    .map(|start| start.elapsed().as_secs_f64() * 1000.0)
                    .unwrap_or(0.0)
            );
        }
        self.road_commit_request_id
    }

    /// Consumes the latest placement's acceptance or rejection without locking the simulation:
    /// `committed`, `reason` (the stable key the `events` report uses) and `detail`. Nil means
    /// pending/already consumed; a superseded id or disconnected worker is rejected.
    #[func]
    pub fn get_road_commit_result(&mut self, request_id: i64) -> Variant {
        if request_id != self.road_commit_request_id {
            return vdict! {
                "committed": false,
                "reason": "road_request_replaced",
                "detail": "road_request_replaced",
            }
            .to_variant();
        }
        let Some(receiver) = self.road_commit_result.as_ref() else {
            return Variant::nil();
        };
        let result = match receiver.try_recv() {
            Ok(ack) => vdict! {
                "committed": ack.committed,
                "reason": ack.reason,
                "detail": ack.detail,
            },
            Err(std::sync::mpsc::TryRecvError::Empty) => return Variant::nil(),
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                vdict! {
                    "committed": false,
                    "reason": "road_command_unavailable",
                    "detail": "road_command_unavailable",
                }
            }
        };
        self.road_commit_result = None;
        result.to_variant()
    }

    /// Returns the node ID of the nearest graph node near the border, or -1.
    #[func]
    pub fn check_border_candidate(&self, pos: Vector3) -> i64 {
        // Almost every road endpoint is far from the world edge. Reject those against the
        // immutable render snapshot so an in-progress surface compile cannot stall the main
        // thread merely to repeat this bounds check under the authoritative core lock.
        let terrain_world_size = self.snapshot.read().unwrap().terrain_world_size;
        let snapshot_has_extents = terrain_world_size.x > 0.0 && terrain_world_size.y > 0.0;
        if snapshot_has_extents
            && !Self::road_tool_is_near_border(
                pos,
                terrain_world_size.x * 0.5,
                terrain_world_size.y * 0.5,
                config::BORDER_DETECTION_THRESHOLD,
            )
        {
            debug_log!(
                "economy",
                "border candidate rejected at pos=({:.1}, {:.1}, {:.1}) because it is not near the map boundary",
                pos.x,
                pos.y,
                pos.z
            );
            return -1;
        }
        self.lock_core().check_border_candidate_internal(pos)
    }

    /// Marks the node at `node_id` as an external border connection.
    #[func]
    pub fn set_border_connection(&mut self, node_id: i32) {
        self.lock_core().set_border_connection_internal(node_id);
    }

    /// Returns the world-space positions of all active border nodes as a flat float array.
    #[func]
    pub fn get_border_nodes(&self) -> PackedFloat32Array {
        self.lock_core().get_border_nodes_internal()
    }

    /// Returns the classification of an edge as an integer (0=Standard, 1=Bridge, 2=Tunnel).
    /// Returns 0 if the edge index is invalid.
    #[func]
    pub fn get_edge_class(&self, edge_idx: i32) -> u8 {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return 0;
        }
        match core.region_graph.edge(edge_idx as usize).class {
            crate::simulation::network::types::EdgeClass::Bridge => 1,
            crate::simulation::network::types::EdgeClass::Tunnel => 2,
            _ => 0,
        }
    }

    /// Sets the classification of an edge (Standard, Bridge, Tunnel).
    #[func]
    pub fn set_edge_class(&mut self, edge_idx: i32, class_int: u8) {
        self.lock_core()
            .set_edge_class_internal(edge_idx, class_int);
    }

    /// Sets or clears the no-building-spawn flag on an edge. When true the building
    /// allocator skips this edge. Player-toggleable; also auto-set for speed ≥ 80 km/h.
    #[func]
    pub fn set_no_building_spawn(&mut self, edge_idx: i32, enabled: bool) {
        self.lock_core()
            .set_no_building_spawn_internal(edge_idx, enabled);
    }

    /// Sets the vehicle frontage-access policy on an edge.
    ///
    /// `0 = SameSideOnly`, `1 = BothSides`. Invalid values are ignored.
    #[func]
    pub fn set_vehicle_frontage_access(&mut self, edge_idx: i32, access_int: u8) {
        self.lock_core()
            .set_vehicle_frontage_access_internal(edge_idx, access_int);
    }

    /// Returns true if the given edge has the no-building-spawn flag set.
    #[func]
    pub fn get_no_building_spawn(&self, edge_idx: i32) -> bool {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return false;
        }
        core.region_graph.edge(edge_idx as usize).no_building_spawn
    }

    /// Returns the vehicle frontage-access policy on an edge.
    ///
    /// Returns `1` (`BothSides`) if the edge index is invalid.
    #[func]
    pub fn get_vehicle_frontage_access(&self, edge_idx: i32) -> u8 {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return 1;
        }
        match core
            .region_graph
            .edge(edge_idx as usize)
            .vehicle_frontage_access
        {
            crate::simulation::network::types::VehicleFrontageAccess::SameSideOnly => 0,
            crate::simulation::network::types::VehicleFrontageAccess::BothSides => 1,
        }
    }

    /// Returns the start and end node indices of an edge as `Vector2i(start, end)`.
    /// Returns `(-1, -1)` if the edge index is invalid.
    #[func]
    pub fn get_edge_nodes(&self, edge_idx: i32) -> Vector2i {
        let core = self.lock_core();
        if edge_idx < 0 || edge_idx as usize >= core.region_graph.edge_count() {
            return Vector2i::new(-1, -1);
        }
        let e = core.region_graph.edge(edge_idx as usize);
        Vector2i::new(e.start_node as i32, e.end_node as i32)
    }

    /// Returns no-build road line segments without waiting when the simulation is busy.
    #[func]
    pub fn try_get_no_building_spawn_lines(&self) -> VarDictionary {
        let mut dict = VarDictionary::new();
        dict.set("busy", true);
        let Some(core) = self.try_lock_core() else {
            return dict;
        };
        let mut lines = PackedVector3Array::new();
        for edge in core.region_graph.edges() {
            if edge.deleted || !edge.no_building_spawn {
                continue;
            }
            for segment in edge.physical_geometry.windows(2) {
                lines.push(segment[0]);
                lines.push(segment[1]);
            }
        }
        dict.set("busy", false);
        dict.set("line_vertices", lines);
        dict
    }

    /// Returns a generation-stamped road-mesh batch for the Godot chunk renderer.
    ///
    /// A full snapshot contains every occupied chunk and costs `O(total chunks + total vertices)`.
    /// An incremental response contains only accumulated upserts and removal tombstones since the
    /// last exact-generation acknowledgement. Each response also identifies whether it replaces
    /// all resident chunks and supplies the world-space chunk span and origin used by its local
    /// vertices.
    #[func]
    pub fn get_road_mesh_data(&self, full_snapshot: bool) -> VarDictionary {
        if full_snapshot {
            // A renderer rebuild is rare and necessarily uploads the whole network. Clone only
            // the Arc handles under the sim lock, then pack Godot arrays after releasing it.
            let (chunks, generation, chunk_span_m, chunk_origin_x_m, chunk_origin_z_m) = {
                let core = self.lock_core();
                let (chunk_origin_x_m, chunk_origin_z_m) =
                    core.transit_network.road_surface.chunk_origin_m();
                (
                    core.cached_road_mesh_chunks.clone(),
                    core.cached_road_mesh_generation,
                    core.transit_network.road_surface.chunk_span_m(),
                    chunk_origin_x_m,
                    chunk_origin_z_m,
                )
            };
            return SimCore::road_mesh_chunks_dict(
                &chunks,
                &BTreeSet::new(),
                true,
                generation,
                chunk_span_m,
                chunk_origin_x_m,
                chunk_origin_z_m,
            );
        }
        let (
            chunks,
            pending_chunks,
            full_replace,
            generation,
            chunk_span_m,
            chunk_origin_x_m,
            chunk_origin_z_m,
        ) = {
            let snapshot = self.snapshot.read().unwrap();
            (
                Arc::clone(&snapshot.road_mesh_chunks),
                Arc::clone(&snapshot.pending_road_mesh_chunks),
                snapshot.road_mesh_full_replace,
                snapshot.network_generation,
                snapshot.road_mesh_chunk_span_m,
                snapshot.road_mesh_chunk_origin_x_m,
                snapshot.road_mesh_chunk_origin_z_m,
            )
        };
        SimCore::road_mesh_chunks_dict(
            &chunks,
            &pending_chunks,
            full_replace,
            generation,
            chunk_span_m,
            chunk_origin_x_m,
            chunk_origin_z_m,
        )
    }

    /// Returns the ID of the closest network node.
    #[func]
    pub fn get_closest_node(&self, world_pos: Vector3, max_dist: f32) -> i32 {
        self.lock_core()
            .get_closest_node_internal(world_pos, max_dist)
    }

    /// Placeholder for cul-de-sac tools.
    #[func]
    pub fn set_node_cul_de_sac(&mut self, _node_id: i32, _enabled: bool, _radius: f32) {}

    /// Placeholder for cul-de-sac tools.
    #[func]
    pub fn has_cul_de_sac(&self, _node_id: i32) -> bool {
        false
    }

    /// Returns the number of road connections for a node.
    #[func]
    pub fn get_node_connection_count(&self, node_id: i32) -> i32 {
        self.lock_core().get_node_connection_count_internal(node_id)
    }

    /// Repositions a network node, returning false when reservations reject the move.
    #[func]
    pub fn move_network_node(&mut self, node_id: i32, pos: Vector3) -> bool {
        let moved = self.lock_core().move_network_node_internal(node_id, pos);
        if moved {
            self.refresh_snapshot_from_core();
        }
        moved
    }

    /// Returns all junction node positions, read from the pre-computed snapshot.
    ///
    /// Reading from the snapshot (RwLock) avoids acquiring the SimCore mutex, which
    /// would stall the Godot main thread while `add_road_internal` holds the lock.
    #[func]
    pub fn get_network_nodes(&self) -> PackedVector3Array {
        PackedVector3Array::from_iter(self.snapshot.read().unwrap().node_positions.iter().copied())
    }

    /// Configures a lane connection rule at a junction.
    #[func]
    pub fn set_lane_connection(
        &mut self,
        node_id: u32,
        from_edge: i32,
        from_lane: i32,
        to_edge: i32,
        to_lane: i32,
    ) {
        self.lock_core()
            .set_lane_connection_internal(node_id, from_edge, from_lane, to_edge, to_lane);
    }

    /// Clears all lane rules at a junction node.
    #[func]
    pub fn clear_lane_connections(&mut self, node_id: u32) {
        self.lock_core().clear_lane_connections_internal(node_id);
    }

    /// Builds an immutable per-arm zebra preview, including median and terrain replacement.
    #[func]
    pub fn preview_crosswalk(
        &mut self,
        node_id: u32,
        edge_id: i32,
        enabled: bool,
    ) -> VarDictionary {
        self.pocket_preview_revision = self.pocket_preview_revision.wrapping_add(1).max(1);
        let core = self.lock_core();
        let preview = usize::try_from(edge_id)
            .map_err(|_| "invalid_crosswalk_arm")
            .and_then(|edge| {
                core.transit_network.road_surface.compile_crosswalk_preview(
                    &core.region_graph,
                    &core.heightmap,
                    node_id,
                    edge,
                    enabled,
                )
            });
        match preview {
            Ok((scene, roads)) => self.local_layout_preview_result(&core, scene, &roads),
            Err(reason) => vdict! { "is_valid": false, "invalid_reason": reason },
        }
    }

    /// Toggles a user override for a crosswalk at a specific road mouth.
    #[func]
    pub fn set_crosswalk_override(&mut self, node_id: u32, edge_id: i32, enabled: bool) {
        let surface_changed = {
            let mut core = self.lock_core();
            let generation = core.road_tool_surface_generation;
            core.set_crosswalk_override_internal(node_id, edge_id, enabled);
            core.road_tool_surface_generation != generation
        };
        if surface_changed {
            self.refresh_snapshot_from_core();
        }
    }

    /// Returns true if a crosswalk exists natively or by user override.
    #[func]
    pub fn has_crosswalk(&self, node_id: u32, edge_id: i32) -> bool {
        self.lock_core().has_crosswalk_internal(node_id, edge_id)
    }

    /// Returns the world-space position of a node.
    #[func]
    pub fn get_node_pos(&self, node_id: u32) -> Vector3 {
        self.lock_core().get_node_pos_internal(node_id)
    }

    /// Returns information about all lanes entering/leaving a junction.
    #[func]
    pub fn get_node_lanes(&self, node_id: u32) -> VarArray {
        self.lock_core().get_node_lanes_internal(node_id)
    }

    /// Returns an array of current lane turn restrictions at a node.
    #[func]
    pub fn get_lane_connections_array(&self, node_id: u32) -> VarArray {
        self.lock_core()
            .get_lane_connections_array_internal(node_id)
    }

    /// Clears lane rules for a specific source lane.
    #[func]
    pub fn clear_lane_source(&mut self, node_id: u32, from_edge: i32, from_lane: i32) {
        self.lock_core()
            .clear_lane_source_internal(node_id, from_edge, from_lane);
    }
}

impl SimulationNode {
    fn local_layout_preview_result(
        &self,
        core: &crate::nodes::sim::core::SimCore,
        mut scene: crate::simulation::network::render::road::preview::RoadJunctionPreview,
        roads: &Arc<crate::simulation::network::surface::PlannedRoadSurfaceQuery>,
    ) -> VarDictionary {
        let mut result = VarDictionary::new();
        // Compilation is synchronous: a missing product is a refusal, never pending work.
        let terrain = core.compile_pocket_preview_terrain(roads);
        if let Some(reason) = terrain
            .as_ref()
            .and_then(|terrain| terrain.failure_reason())
        {
            result.set("is_valid", false);
            result.set("invalid_reason", "surface_geometry_invalid");
            result.set("detail", reason);
            return result;
        }
        let Some(patches) = terrain
            .as_ref()
            .and_then(|terrain| terrain.preview_patches())
        else {
            result.set("is_valid", false);
            result.set("is_pending", false);
            result.set(
                "invalid_reason",
                terrain
                    .as_ref()
                    .and_then(|terrain| terrain.failure_reason())
                    .unwrap_or("surface_preview_terrain_incomplete"),
            );
            return result;
        };
        let mut payloads = Array::<VarDictionary>::new();
        for patch in patches {
            let mut data = if patch.input_road_loops == 0 {
                Self::terrain_patch_dict(&patch.patch, &[])
            } else {
                Self::cached_refined_terrain_patch_dict(&patch, false, &[])
            };
            data.set(
                "terrain_revision",
                i64::try_from(self.pocket_preview_revision).unwrap_or(i64::MAX),
            );
            data.set(
                "surface_generation",
                i64::try_from(core.road_tool_surface_generation).unwrap_or(i64::MAX),
            );
            data.set("render_step_mm", i64::from(patch.key.render_step_mm));
            data.set(
                "terrain_requires_engineered_refinement",
                patch.input_road_loops > 0,
            );
            payloads.push(&data);
        }
        result.set("terrain_preview", vdict! { "patches": payloads });

        scene.source_mesh_generation = core.cached_road_mesh_generation;
        let meshes = scene
            .replacement_chunks
            .iter()
            .filter_map(|key| {
                core.cached_road_mesh_chunks
                    .get(key)
                    .map(|mesh| (*key, Arc::clone(mesh)))
            })
            .collect();
        scene.retain_existing(&meshes);
        scene.retained_revision = self.pocket_preview_revision;
        result.set("is_valid", true);
        result.set(
            "junction_preview",
            Self::road_junction_preview_dict(&scene, core.road_tool_surface_generation, 0),
        );
        result.set(
            "request_id",
            i64::try_from(self.pocket_preview_revision).unwrap_or(i64::MAX),
        );
        result.set(
            "surface_generation",
            i64::try_from(core.road_tool_surface_generation).unwrap_or(i64::MAX),
        );
        result
    }
}
