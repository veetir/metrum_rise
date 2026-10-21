// SPDX-License-Identifier: GPL-2.0-only

//! Roundabout transactions reuse the local road undo, surface and lane publication gates.

use super::core::SimCore;
use crate::simulation::network::roundabouts::{RoundaboutLayout, RoundaboutRequest};
use std::collections::HashSet;

impl SimCore {
    /// Checks the existing indexed water and land guards without changing authoring state.
    pub(crate) fn roundabout_layout_refusal(
        &self,
        layout: &RoundaboutLayout,
    ) -> Option<&'static str> {
        for edge in layout.corridors() {
            let points = &edge.physical_geometry;
            let half =
                crate::simulation::network::surface::RoadSurfaceSystem::visual_roadbed_half_width_m(
                    edge,
                );
            if self
                .watermap
                .road_corridor_overlaps_visible_water(points, half)
            {
                return Some("water_requires_bridge");
            }
            if self.zoning.cells_overlap_road_corridor(points, half) {
                return Some("zoning_cell_overlap");
            }
            if self
                .allocator
                .field_clearance
                .overlaps_road_corridor(points, half)
            {
                return Some("field_overlap");
            }
            if !self
                .zoning
                .parcel_ids_overlapping_road_corridor(points, half)
                .is_empty()
            {
                return Some("parcel_overlap");
            }
        }
        None
    }

    /// Publishes one whole preset through the road transaction gate or rolls it back on refusal.
    pub(crate) fn place_roundabout(&mut self, request: RoundaboutRequest) -> bool {
        self.last_road_rejection = None;
        let layout = match RoundaboutLayout::prepare(&self.region_graph, request.clone()) {
            Ok(layout) => layout,
            Err(reason) => return self.refuse_cross_section(reason),
        };
        if let Some(reason) = self.roundabout_layout_refusal(&layout) {
            return self.refuse_cross_section(reason);
        }
        if let Err(reason) = self
            .transit_network
            .road_surface
            .compile_roundabout_preview(&self.region_graph, &self.heightmap, request)
        {
            return self.refuse_cross_section(reason);
        }
        let mut dirty: HashSet<_> = layout.originals.iter().map(|(id, _)| *id).collect();
        let mut nodes: HashSet<_> = layout
            .originals
            .iter()
            .flat_map(|(_, e)| [e.start_node, e.end_node])
            .collect();
        for &node in &nodes {
            dirty.extend(self.region_graph.node_adjacency(node).iter().copied());
        }
        nodes.extend(dirty.iter().flat_map(|&id| {
            let e = self.region_graph.edge(id);
            [e.start_node, e.end_node]
        }));
        self.transit_network.begin_road_edit();
        self.push_network_undo_for_local_topology(dirty.clone(), nodes.clone());
        self.transit_network
            .mark_surface_dirty_from_sets(&self.region_graph, &dirty, &nodes);
        dirty.extend(layout.apply(&mut self.region_graph));
        self.finish_road_section_edit(dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::core::config::WorldConfig;
    use godot::prelude::Vector3;

    fn core() -> SimCore {
        if std::env::var("METRUM_DEBUG").is_ok_and(|value| value == "1") {
            crate::debug::init();
        }
        SimCore::new(WorldConfig {
            width_m: 1024.0,
            height_m: 1024.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        })
    }

    fn request() -> RoundaboutRequest {
        RoundaboutRequest {
            center: Vector3::ZERO,
            diameter: 50.0,
            angles: vec![0.0, 1.4, 3.0, 4.7],
            replace: None,
        }
    }

    #[test]
    fn roundabout_preset_compiles_commits_and_undoes_in_one_step() {
        let mut core = core();
        let (preview, _) = core
            .transit_network
            .road_surface
            .compile_roundabout_preview(&core.region_graph, &core.heightmap, request())
            .unwrap();
        assert!(!preview.planned.is_empty());
        assert!(
            core.place_roundabout(request()),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(
            core.region_graph
                .edges()
                .iter()
                .filter(|e| e.roundabout)
                .count(),
            4
        );
        assert_eq!(
            core.region_graph
                .edges()
                .iter()
                .filter(|e| !e.deleted)
                .count(),
            8
        );
        for (key, mesh) in &preview.planned {
            let committed = &core.cached_road_mesh_chunks[key];
            assert_eq!(mesh.road_vertices, committed.road_vertices);
            assert_eq!(mesh.curb_vertices, committed.curb_vertices);
            assert_eq!(mesh.sidewalk_vertices, committed.sidewalk_vertices);
            assert_eq!(mesh.marking_vertices, committed.marking_vertices);
            assert_eq!(mesh.road_signs, committed.road_signs);
        }
        assert!(core.undo_action_internal());
        assert_eq!(core.region_graph.edge_count(), 0);
        assert!(core.transit_network.lane_system.edge_lanes.is_empty());
    }

    #[test]
    fn roundabout_crowded_geometry_refuses_without_mutation() {
        let mut core = core();
        let mut request = request();
        request.angles[1] = 0.01;
        assert!(!core.place_roundabout(request));
        assert_eq!(core.region_graph.edge_count(), 0);
        assert_eq!(core.region_graph.node_count(), 0);
    }

    #[test]
    fn rotated_roundabout_terrain_preserves_the_central_island_hole() {
        let mut core = core();
        let mut request = request();
        request.diameter = 60.0;
        request.angles = (0..4)
            .map(|i| 0.12 + i as f32 * std::f32::consts::FRAC_PI_2)
            .collect();
        let (_, roads) = core
            .transit_network
            .road_surface
            .compile_roundabout_preview(&core.region_graph, &core.heightmap, request.clone())
            .unwrap();
        let terrain = core.compile_pocket_preview_terrain(&roads).unwrap();
        let patches = terrain.preview_patches().unwrap();
        let covers = |world: godot::prelude::Vector2| {
            patches.iter().any(|patch| {
                let Some(mesh) = &patch.mesh_buffers else {
                    return false;
                };
                let point = world
                    - godot::prelude::Vector2::new(
                        patch.patch.world_origin_x + patch.patch.world_size_x * 0.5,
                        patch.patch.world_origin_z + patch.patch.world_size_z * 0.5,
                    );
                mesh.terrain_indices.as_chunks::<3>().0.iter().any(|ids| {
                    let v = ids.map(|id| {
                        let p = mesh.terrain_vertices[id as usize];
                        godot::prelude::Vector2::new(p.x, p.z)
                    });
                    let cross = [0, 1, 2].map(|i| (v[(i + 1) % 3] - v[i]).cross(point - v[i]));
                    cross.iter().all(|&x| x >= -0.001) || cross.iter().all(|&x| x <= 0.001)
                })
            })
        };
        assert!(
            covers(godot::prelude::Vector2::ZERO),
            "central grass must retain terrain triangles"
        );
        assert!(
            !covers(godot::prelude::Vector2::new(0.12_f32.cos(), 0.12_f32.sin()) * 28.25),
            "terrain must be cut out beneath the ring"
        );
        assert!(
            core.place_roundabout(request),
            "{:?}",
            core.last_road_rejection
        );
    }

    #[test]
    fn roundabout_replacement_retains_uneven_arms_and_original_edge_ids() {
        use crate::simulation::network::graph::Edge;
        use crate::simulation::network::types::{NodeType, TransitFlags, TransitType};
        let mut core = core();
        let center = core
            .region_graph
            .add_node(Vector3::ZERO, NodeType::Junction);
        for angle in [0.0_f32, 1.3, 2.8, 4.5, 5.5] {
            let p = Vector3::new(angle.cos(), 0.0, angle.sin()) * 100.0;
            let node = core.region_graph.add_node(p, NodeType::Junction);
            core.region_graph.add_edge(Edge {
                start_node: node,
                end_node: center,
                geometry: vec![p, Vector3::ZERO],
                physical_geometry: vec![p, Vector3::ZERO],
                physical_length: 100.0,
                fwd_lanes: 1,
                bkw_lanes: 1,
                width: 7.0,
                speed_limit: 50.0 / 3.6,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                ..Default::default()
            });
        }
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core.precompute_road_mesh_data();
        let lane = *core.transit_network.lane_system.edge_lanes[&0]
            .iter()
            .find(|&&id| {
                let lane = &core.transit_network.lane_system.lanes[id];
                lane.is_fwd
                    && lane.lane_type == crate::simulation::network::lanes::LaneType::Vehicle
            })
            .unwrap();
        let from = core.region_graph.edge(0).start_node;
        let to = core.region_graph.edge(1).start_node;
        let agent = core.agents.spawn_debug_car(
            vec![from, center, to],
            &core.transit_network.lane_system,
            lane,
            godot::prelude::Vector2::ZERO,
        );
        let old_lane = &core.transit_network.lane_system.lanes[lane];
        core.agents.agents.enter_lane(
            agent,
            &core.transit_network.lane_system,
            lane,
            old_lane.length,
            from,
        );
        let old_point = *old_lane.geometry.last().unwrap();
        let old_pose = godot::prelude::Vector2::new(old_point.x, old_point.z);
        let before: Vec<_> = core
            .region_graph
            .edges()
            .iter()
            .map(|e| e.physical_geometry.clone())
            .collect();
        let mut request = request();
        request.replace = Some(center);
        request.diameter = 60.0;
        let mut oversized = request.clone();
        oversized.diameter = 80.0;
        assert!(!core.place_roundabout(oversized));
        assert_eq!(core.region_graph.edge_count(), 5);
        assert!(
            core.place_roundabout(request),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(core.region_graph.node_adjacency(center).len(), 0);
        let new_lane = core.agents.agents.current_lane_id[agent];
        assert!(new_lane < core.transit_network.lane_system.lanes.len());
        assert!(
            core.agents.agents.current_path[agent].is_empty(),
            "retired centre must leave the live route plan"
        );
        let pose = godot::prelude::Vector2::new(
            core.agents.agents.pos_x[agent],
            core.agents.agents.pos_y[agent],
        );
        assert!(pose.distance_to(old_pose) <= 30.0);
        for (id, original) in before.iter().enumerate() {
            let edge = core.region_graph.edge(id);
            assert_eq!(edge.geometry[0], original[0]);
            assert!(edge.end_node != center);
            assert_eq!(core.region_graph.node_adjacency(edge.end_node).len(), 3);
        }
        assert!(core.undo_action_internal());
        assert!(
            core.agents.agents.current_lane_id[agent]
                < core.transit_network.lane_system.lanes.len()
        );
        assert_eq!(core.region_graph.edge_count(), 5);
        for (edge, before) in core.region_graph.edges().iter().zip(before) {
            assert_eq!(edge.physical_geometry, before);
        }
    }
}
