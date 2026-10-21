// SPDX-License-Identifier: GPL-2.0-only

//! Ramp transactions reuse the local road undo, surface and lane publication gates.

use super::core::SimCore;
use crate::simulation::network::ramps::{RampLayout, RampRequest};
use std::collections::HashSet;

impl SimCore {
    /// Publishes one whole preset through the road transaction gate or rolls it back on refusal.
    pub(crate) fn place_ramp(&mut self, request: RampRequest) -> bool {
        self.last_road_rejection = None;
        let layout = match RampLayout::prepare(&self.region_graph, request.clone()) {
            Ok(layout) => layout,
            Err(reason) => return self.refuse_cross_section(reason),
        };
        if let Some(reason) = self.road_layout_corridor_refusal(layout.corridors()) {
            return self.refuse_cross_section(reason);
        }
        if let Err(reason) = self.transit_network.road_surface.compile_ramp_preview(
            &self.region_graph,
            &self.heightmap,
            request,
        ) {
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
        let originals = layout.originals.clone();
        let changed = layout.apply(&mut self.region_graph);
        self.transit_network.remap_pocket_frontages(
            &originals,
            &changed[..3],
            &self.region_graph,
            &self.zoning,
            &mut self.allocator,
        );
        dirty.extend(changed);
        self.finish_road_section_edit(dirty)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::core::config::WorldConfig;
    use crate::simulation::network::graph::{Edge, RoadMedian};
    use crate::simulation::network::types::{NodeType, TransitFlags};
    use godot::prelude::Vector3;

    fn fixture() -> SimCore {
        let mut core = SimCore::new(WorldConfig {
            width_m: 1600.0,
            height_m: 1024.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        });
        let a = Vector3::new(-600.0, 0.0, 0.0);
        let b = Vector3::new(600.0, 0.0, 0.0);
        let start = core.region_graph.add_node(a, NodeType::Junction);
        let end = core.region_graph.add_node(b, NodeType::Junction);
        core.region_graph.add_edge(Edge {
            start_node: start,
            end_node: end,
            geometry: vec![a, b],
            physical_geometry: vec![a, b],
            physical_length: 1200.0,
            fwd_lanes: 2,
            bkw_lanes: 2,
            width: 19.0,
            median: RoadMedian::from_tool(2, 5.0).unwrap(),
            speed_limit: 100.0 / 3.6,
            allowed_types: TransitFlags::CAR,
            ..Default::default()
        });
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core
    }

    fn request(exit: bool, forward: bool) -> RampRequest {
        let sign = if forward { 1.0 } else { -1.0 };
        RampRequest {
            edge: 0,
            station: 600.0,
            forward,
            exit,
            points: vec![
                Vector3::ZERO,
                Vector3::new(sign * if exit { 200.0 } else { -200.0 }, 0.0, sign * 100.0),
            ],
            length: 80.0,
            taper: 35.0,
        }
    }

    #[test]
    fn ramp_preview_commit_and_single_undo() {
        for exit in [true, false] {
            for forward in [true, false] {
                let mut core = fixture();
                let (preview, _) = core
                    .transit_network
                    .road_surface
                    .compile_ramp_preview(
                        &core.region_graph,
                        &core.heightmap,
                        request(exit, forward),
                    )
                    .unwrap();
                assert!(
                    core.place_ramp(request(exit, forward)),
                    "{:?}",
                    core.last_road_rejection
                );
                assert_eq!(core.region_graph.edge_count(), 4);
                let aux = core.region_graph.edge(1);
                let fork_clip = if forward != exit {
                    aux.start_clip
                } else {
                    aux.end_clip
                };
                assert!(
                    (aux.physical_length - fork_clip - 35.0 - 80.0).abs() < 0.001,
                    "auxiliary={} mouth={fork_clip}",
                    aux.physical_length
                );

                let median_height = core
                    .transit_network
                    .road_surface
                    .sample_visible_surface_height(&core.region_graph, &core.heightmap, 0.0, 0.0)
                    .unwrap();
                assert!(
                    median_height > 0.05,
                    "separator must cross the fork: {median_height}"
                );
                for &id in &core.transit_network.lane_system.node_lanes[&3] {
                    let lane = &core.transit_network.lane_system.lanes[id];
                    if lane.lane_type != crate::simulation::network::lanes::LaneType::Vehicle {
                        continue;
                    }
                    for p in &lane.geometry {
                        let height = core
                            .transit_network
                            .road_surface
                            .sample_visible_surface_height(
                                &core.region_graph,
                                &core.heightmap,
                                p.x,
                                p.z,
                            )
                            .unwrap();
                        assert!(
                            (height - p.y).abs() < 0.05,
                            "connector crosses raised land: {p:?} height={height}"
                        );
                    }
                    assert!(
                        core.transit_network
                            .lane_system
                            .controlled_movement(id)
                            .is_none()
                    );
                }

                assert!(core.region_graph.median_continues_at_node(1, 3));
                for (key, mesh) in &preview.planned {
                    assert_eq!(
                        mesh.road_vertices,
                        core.cached_road_mesh_chunks[key].road_vertices
                    );
                }
                assert!(core.undo_action_internal());
                assert_eq!(core.region_graph.edge_count(), 1);
                assert_eq!(core.region_graph.node_count(), 2);
            }
        }
    }
    #[test]
    fn ramps_preserve_turn_contract_on_save_and_refuse_incompatible_edits() {
        use crate::simulation::pathing::cch::CchGraph;
        for exit in [true, false] {
            for forward in [true, false] {
                let mut core = fixture();
                assert!(core.place_ramp(request(exit, forward)));
                let (before, after) = if forward != exit { (0, 1) } else { (1, 2) };
                let (from, to) = if forward {
                    (before, after)
                } else {
                    (after, before)
                };
                let legal = if exit { (from, 3) } else { (3, to) };
                let forbidden = if exit { (to, 3) } else { (3, from) };
                let valid = |core: &SimCore, a, b| {
                    CchGraph::path_has_valid_vehicle_turns(&[3], a, b, &core.region_graph)
                };
                assert!(valid(&core, legal.0, legal.1));
                assert!(!valid(&core, forbidden.0, forbidden.1));
                assert!(!valid(&core, from, from));
                let path = std::env::temp_dir().join(format!(
                    "roads-rp-{}-{exit}-{forward}.sqlite",
                    std::process::id()
                ));
                core.save_game_internal(path.to_str().unwrap(), None)
                    .unwrap();
                core.load_game_internal(path.to_str().unwrap()).unwrap();
                std::fs::remove_file(path).unwrap();
                assert_eq!(core.region_graph.node(3).node_type, NodeType::Ramp);
                assert!(valid(&core, legal.0, legal.1));
                assert!(!valid(&core, forbidden.0, forbidden.1));
                assert!(!core.change_road_cross_section(3, 1, 1, None));
                assert!(!core.change_road_pocket(1, false, false, true, 80.0, 35.0));
                assert!(!core.change_road_pocket(0, false, false, false, 80.0, 35.0));
                assert!(!core.move_network_node_internal(3, Vector3::new(0.0, 1.0, 0.0)));
                core.clear_lane_connections_internal(3);
                assert!(valid(&core, legal.0, legal.1));
                assert!(!valid(&core, forbidden.0, forbidden.1));
            }
        }
    }

    #[test]
    fn refused_ramps_leave_topology_and_undo_untouched() {
        let mut core = fixture();
        for mode in 0..5 {
            let mut request = request(true, true);
            match mode {
                0 => request.station = f32::NAN,
                1 => request.station = 50.0,
                2 => request.points[1].z = -100.0,
                3 => request.points[1].y = 100.0,
                _ => request.taper = 0.0,
            }
            assert!(!core.place_ramp(request));
            assert_eq!(core.region_graph.edge_count(), 1);
            assert_eq!(core.region_graph.node_count(), 2);
        }
        assert!(!core.undo_action_internal());
    }
    #[test]
    fn separated_exit_and_entry_share_a_main_road_without_restricting_through_routes() {
        let mut core = fixture();
        let mut exit = request(true, true);
        exit.station = 400.0;
        exit.points = vec![
            Vector3::new(-200.0, 0.0, 0.0),
            Vector3::new(-50.0, 0.0, 100.0),
        ];
        assert!(core.place_ramp(exit), "{:?}", core.last_road_rejection);
        let mut entry = request(false, true);
        entry.edge = 2;
        entry.station = 400.0;
        entry.points = vec![
            Vector3::new(200.0, 0.0, 0.0),
            Vector3::new(50.0, 0.0, 100.0),
        ];
        assert!(core.place_ramp(entry), "{:?}", core.last_road_rejection);
        assert_eq!(core.region_graph.edge_count(), 7);
        let graph = &core.region_graph;
        assert!(
            crate::simulation::pathing::cch::CchGraph::path_has_valid_vehicle_turns(
                &[0, 2, 3, 6, 5, 1],
                usize::MAX,
                usize::MAX,
                graph
            )
        );
        assert!(core.undo_action_internal());
        assert_eq!(core.region_graph.edge_count(), 4);
        assert!(core.undo_action_internal());
        assert_eq!(core.region_graph.edge_count(), 1);
    }
}
