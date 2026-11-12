// SPDX-License-Identifier: GPL-2.0-only

//! Road-network spatial queries (closest-point identification, edge projection).

use crate::config::DEFAULT_ZONING_DEPTH;
use crate::nodes::sim::core::SimCore;
use crate::simulation::network::interaction;
use godot::prelude::*;

// How far a junction's marker ring reaches beyond half its widest carriageway, in metres.
const JUNCTION_MARKER_PAD_M: f32 = 1.5;

impl SimCore {
    /// Returns the ID of the edge closest to the given world position.
    pub fn get_hovered_edge_internal(&self, world_x: f32, world_z: f32) -> i32 {
        let max_depth = (DEFAULT_ZONING_DEPTH as f32 * self.config.zone_cell_m) + 10.0;
        interaction::get_closest_edge_xz(&self.region_graph, world_x, world_z, max_depth)
            .map_or(-1, |id| id as i32)
    }

    /// Returns the closest network point (node/edge) within range.
    pub fn get_closest_network_point_internal(
        &self,
        world_pos: Vector3,
        max_dist: f32,
    ) -> Option<Vector3> {
        interaction::get_closest_point_xz(&self.region_graph, world_pos, max_dist)
    }

    /// Returns the ID of the closest network node.
    pub fn get_closest_node_internal(&self, world_pos: Vector3, max_dist: f32) -> i32 {
        interaction::get_closest_node_xz(&self.region_graph, world_pos, max_dist)
            .map_or(-1, |id| id as i32)
    }

    /// The junctions (live nodes where three or more roads meet) inside the XZ rectangle `min` to
    /// `max`, in ascending node id order: id, position and a marker radius (half the widest
    /// road's carriageway plus `JUNCTION_MARKER_PAD_M`). The work follows the node grid cells the
    /// rectangle covers and each junction's own roads, never the whole network.
    pub fn junction_markers_internal(
        &self,
        min: Vector3,
        max: Vector3,
    ) -> Vec<(u32, Vector3, f32)> {
        let graph = &self.region_graph;
        let mut markers = Vec::new();
        for node_id in graph.get_nodes_near_aabb(min, max) {
            let pos = graph.node(node_id).pos;
            if pos.x < min.x || pos.x > max.x || pos.z < min.z || pos.z > max.z {
                continue;
            }
            if graph.get_valid_node(node_id) != node_id
                || node_id as usize >= graph.node_adjacency_count()
            {
                continue;
            }
            let (arms, widest) = graph
                .node_adjacency(node_id)
                .iter()
                .map(|&edge_idx| graph.edge(edge_idx))
                .filter(|edge| !edge.deleted)
                .fold((0, 0.0_f32), |(arms, widest), edge| {
                    (arms + 1, widest.max(edge.width))
                });
            if arms >= 3 {
                markers.push((node_id, pos, widest * 0.5 + JUNCTION_MARKER_PAD_M));
            }
        }
        markers
    }

    /// Returns the number of non-deleted road connections for a node.
    pub fn get_node_connection_count_internal(&self, node_id: i32) -> i32 {
        if node_id < 0 || node_id as usize >= self.region_graph.node_count() {
            return 0;
        }
        self.region_graph.live_node_connection_count(node_id as u32) as i32
    }

    /// Returns all live junction node positions.
    pub fn get_network_nodes_internal(&self) -> PackedVector3Array {
        let mut arr = PackedVector3Array::new();
        for (i, node) in self.region_graph.nodes().iter().enumerate() {
            if self.region_graph.is_live_canonical_node(i as u32) {
                arr.push(node.pos);
            }
        }
        arr
    }

    /// Returns the world-space position of a node.
    pub fn get_node_pos_internal(&self, node_id: u32) -> Vector3 {
        let valid_id = self.region_graph.get_valid_node(node_id);
        if (valid_id as usize) < self.region_graph.node_count() {
            self.region_graph.node(valid_id).pos
        } else {
            Vector3::ZERO
        }
    }

    /// Returns the world-space positions of all active border (external connection) nodes.
    pub fn get_border_nodes_internal(&self) -> PackedFloat32Array {
        let mut arr = PackedFloat32Array::new();
        for (i, node) in self.region_graph.nodes().iter().enumerate() {
            if node.node_type != crate::simulation::network::types::NodeType::Border {
                continue;
            }
            if !self.region_graph.is_live_canonical_node(i as u32) {
                continue;
            }
            arr.push(node.pos.x);
            arr.push(node.pos.y);
            arr.push(node.pos.z);
        }
        arr
    }
}
