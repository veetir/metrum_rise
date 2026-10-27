// SPDX-License-Identifier: GPL-2.0-only

//! Road-end pocket authoring through ordinary split topology and local edit publication.

use super::core::SimCore;
#[cfg(test)]
use crate::simulation::network::types::{NodeType, TransitType};
use std::collections::HashSet;

impl SimCore {
    /// Authors one endpoint pocket transaction; all vehicle lanes retain node-to-node identity.
    pub(crate) fn change_road_pocket(
        &mut self,
        id: usize,
        at_start: bool,
        left: bool,
        right: bool,
        pocket_m: f32,
        taper_m: f32,
    ) -> bool {
        use crate::simulation::network::pockets::{PocketLayout, PocketRequest};
        self.last_road_rejection = None;
        let layout = match PocketLayout::prepare(
            &self.region_graph,
            PocketRequest {
                edge: id,
                at_start,
                left,
                right,
                length: pocket_m,
                taper: taper_m,
            },
        ) {
            Ok(layout) => layout,
            Err(reason) => return self.refuse_cross_section(reason),
        };
        if let Some(reason) = self.pocket_layout_refusal(&layout) {
            return self.refuse_cross_section(reason);
        }
        let originals = layout.originals.clone();
        let mut dirty: HashSet<_> = originals.iter().map(|(id, _)| *id).collect();
        let mut nodes: HashSet<_> = originals
            .iter()
            .flat_map(|(_, edge)| [edge.start_node, edge.end_node])
            .collect();
        for &node in &nodes {
            dirty.extend(self.region_graph.node_adjacency(node).iter().copied());
        }
        nodes.extend(dirty.iter().flat_map(|&id| {
            let edge = self.region_graph.edge(id);
            [edge.start_node, edge.end_node]
        }));
        self.transit_network.begin_road_edit();
        self.push_network_undo_for_local_topology(dirty.clone(), nodes.clone());
        self.transit_network
            .mark_surface_dirty_from_sets(&self.region_graph, &dirty, &nodes);
        let changed = layout.apply(&mut self.region_graph);
        dirty.extend(changed.iter().copied());
        self.transit_network.remap_pocket_frontages(
            &originals,
            &changed,
            &self.region_graph,
            &self.zoning,
            &mut self.allocator,
        );
        self.finish_road_section_edit(dirty)
    }
    pub(crate) fn pocket_layout_refusal(
        &self,
        layout: &crate::simulation::network::pockets::PocketLayout,
    ) -> Option<&'static str> {
        for (points, envelope) in layout.corridor_strips() {
            if self
                .watermap
                .road_corridor_overlaps_visible_water(&points, envelope)
            {
                return Some("water_requires_bridge");
            }
            if self.zoning.cells_overlap_road_corridor(&points, envelope) {
                return Some("zoning_cell_overlap");
            }
            if self
                .allocator
                .field_clearance
                .overlaps_road_corridor(&points, envelope)
            {
                return Some("field_overlap");
            }
            if !self
                .zoning
                .parcel_ids_overlapping_road_corridor(&points, envelope)
                .is_empty()
            {
                return Some("parcel_overlap");
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::core::config::WorldConfig;
    use crate::simulation::economy::agents::{MODE_CAR, TRANSIT_NETWORK};
    use crate::simulation::network::graph::{Edge, RoadMedian};
    use crate::simulation::network::lanes::LaneType;
    use crate::simulation::network::types::TransitFlags;
    use godot::prelude::Vector3;

    fn fixture() -> SimCore {
        let mut core = SimCore::new(WorldConfig {
            width_m: 1024.0,
            height_m: 1024.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        });
        let center = core
            .region_graph
            .add_node(Vector3::ZERO, NodeType::Junction);
        for (point, count, median) in [
            (
                Vector3::new(-240.0, 0.0, 0.0),
                2,
                RoadMedian::from_tool(2, 5.0).unwrap(),
            ),
            (Vector3::new(150.0, 0.0, 0.0), 1, None),
            (Vector3::new(0.0, 0.0, -150.0), 1, None),
            (Vector3::new(0.0, 0.0, 150.0), 1, None),
        ] {
            let end = core.region_graph.add_node(point, NodeType::Junction);
            let points = vec![point, Vector3::ZERO];
            core.region_graph.add_edge(Edge {
                start_node: end,
                end_node: center,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                fwd_lanes: count,
                bkw_lanes: count,
                width: f32::from(count) * 7.0 + median.map_or(0.0, |m| m.width_m),
                median,
                speed_limit: 50.0 / 3.6,
                physical_length: point.length(),
                physical_geometry: points.clone(),
                geometry: points,
                ..Default::default()
            });
        }
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core.precompute_road_mesh_data();
        core
    }

    #[test]
    fn pocket_split_keeps_agents_supported_and_undo_restores_topology() {
        let mut core = fixture();
        let ids = &core.transit_network.lane_system.edge_lanes[&0];
        let lane = *ids
            .iter()
            .find(|&&id| {
                let l = &core.transit_network.lane_system.lanes[id];
                l.lane_type == LaneType::Vehicle && l.is_fwd && l.lane_idx == 1
            })
            .unwrap();
        let agent = core.agents.spawn_border_arrival_agent(
            usize::MAX,
            core.region_graph.edge(0).start_node,
            -70.0,
            7.75,
        );
        core.agents.agents.transit[agent] = TRANSIT_NETWORK;
        core.agents.agents.transit_mode[agent] = MODE_CAR;
        core.agents.agents.current_edge[agent] = 0;
        core.agents.agents.current_lane_id[agent] = lane;
        core.agents.agents.lane_distance[agent] = 170.0;
        let taper_agent = core.agents.spawn_border_arrival_agent(
            usize::MAX,
            core.region_graph.edge(0).start_node,
            -115.0,
            7.75,
        );
        core.agents.agents.transit[taper_agent] = TRANSIT_NETWORK;
        core.agents.agents.transit_mode[taper_agent] = MODE_CAR;
        core.agents.agents.current_edge[taper_agent] = 0;
        core.agents.agents.current_lane_id[taper_agent] = lane;
        core.agents.agents.lane_distance[taper_agent] = 125.0;
        core.agents.agents.speed[taper_agent] = 3.0;

        assert!(
            core.change_road_pocket(0, false, true, true, 80.0, 35.0),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(core.region_graph.edge_count(), 5);
        let pocket = core.region_graph.edge(4);
        assert_eq!((pocket.fwd_lanes, pocket.bkw_lanes), (4, 2));
        assert!(pocket.end_clip >= crate::simulation::network::surface::RoadSurfaceSystem::visual_roadbed_half_width_m(pocket));
        for &id in &core.transit_network.lane_system.edge_lanes[&4] {
            let lane = &core.transit_network.lane_system.lanes[id];
            if lane.lane_type == LaneType::Vehicle {
                assert!(
                    (lane.length - 80.0).abs() < 0.001,
                    "requested pocket length: {}",
                    lane.length
                );
            }
        }
        let transition = pocket.start_node;
        assert_eq!(
            core.region_graph.node(transition).node_type,
            NodeType::LaneTransition
        );
        assert!(
            core.transit_network
                .road_surface
                .published_generation_matches_source()
        );
        let current = core.agents.agents.current_lane_id[agent];
        assert!(core.transit_network.lane_system.edge_lanes[&4].contains(&current));
        assert!((core.agents.agents.pos_x[agent] + 70.0).abs() < 1.0);
        let current = core.agents.agents.current_lane_id[taper_agent];
        assert_eq!(
            core.transit_network.lane_system.lanes[current].edge_id,
            usize::MAX
        );
        assert_eq!(
            core.agents.agents.transit[taper_agent],
            crate::simulation::economy::agents::TRANSIT_INTERSECTION
        );
        assert!((core.agents.agents.pos_x[taper_agent] + 115.0).abs() < 0.001);
        assert_eq!(core.agents.agents.speed[taper_agent], 3.0);

        for &id in &core.transit_network.lane_system.edge_lanes[&0] {
            let lane = &core.transit_network.lane_system.lanes[id];
            if lane.lane_type == LaneType::Vehicle && lane.is_fwd {
                assert!(!lane.next_lanes.is_empty());
            }
        }
        assert!(
            core.transit_network
                .lane_system
                .node_lanes
                .get(&(transition as usize))
                .is_none_or(|ids| ids.iter().all(|&id| core
                    .transit_network
                    .lane_system
                    .crosswalk(id)
                    .is_none()))
        );
        let transition_lanes = &core.transit_network.lane_system.node_lanes[&(transition as usize)];
        assert!(transition_lanes.iter().any(|&id| {
            let lane = &core.transit_network.lane_system.lanes[id];
            lane.lane_type == LaneType::Vehicle && lane.length >= 35.0
        }));
        for &edge in core.region_graph.node_adjacency(transition) {
            for &id in &core.transit_network.lane_system.edge_lanes[&edge] {
                let lane = &core.transit_network.lane_system.lanes[id];
                let terminal = if lane.is_fwd {
                    core.region_graph.edge(edge).end_node
                } else {
                    core.region_graph.edge(edge).start_node
                };
                if lane.lane_type == LaneType::Foot && terminal == transition {
                    assert!(
                        !lane.next_lanes.is_empty(),
                        "sidewalk disconnected at transition"
                    );
                }
            }
        }
        let origin = core.region_graph.node(transition).pos;
        for &id in transition_lanes {
            let lane = &core.transit_network.lane_system.lanes[id];
            if lane.lane_type != LaneType::Vehicle {
                continue;
            }
            for point in &lane.geometry {
                let section = core
                    .region_graph
                    .edge(4)
                    .lane_section_at(point.x - origin.x);
                assert!(
                    point.z >= section.left && point.z <= section.right,
                    "connector leaves road: {point:?} {section:?}"
                );
                assert!(
                    point.z <= section.median_left || point.z >= section.median_right,
                    "connector enters median: {point:?} {section:?}"
                );
            }
        }
        assert!(!core.change_road_cross_section(0, 1, 1, None));
        assert_eq!(core.region_graph.edge(0).fwd_lanes, 2);
        assert!(core.undo_action_internal());
        assert_eq!(core.region_graph.edge_count(), 4);
        assert_eq!(core.region_graph.edge(0).fwd_lanes, 2);
        let taper_current = core.agents.agents.current_lane_id[taper_agent];
        assert!(core.transit_network.lane_system.edge_lanes[&0].contains(&taper_current));
        assert!((core.agents.agents.pos_x[taper_agent] + 115.0).abs() < 0.001);
        let current = core.agents.agents.current_lane_id[agent];
        assert!(core.transit_network.lane_system.edge_lanes[&0].contains(&current));
    }

    #[test]
    fn invalid_pocket_is_transactionally_refused() {
        let mut core = fixture();
        let before = core.undo_stack.len();
        assert!(!core.change_road_pocket(0, false, true, true, 200.0, 60.0));
        assert_eq!(core.undo_stack.len(), before);
        assert_eq!(core.region_graph.edge_count(), 4);
        assert!(!core.change_road_pocket(1, false, true, false, 60.0, 20.0));
        assert_eq!(core.undo_stack.len(), before);
    }
    #[test]
    fn pocket_split_moves_authored_turn_and_crossing_references() {
        let mut core = fixture();
        let junction = core.region_graph.edge(0).end_node;
        core.region_graph.add_lane_connection(junction, 0, 0, 1, -1);
        core.region_graph.add_lane_connection(junction, 0, 1, 2, -1);
        core.region_graph.set_crosswalk_override(junction, 0, true);
        assert!(
            core.change_road_pocket(0, false, true, true, 80.0, 35.0),
            "{:?}",
            core.last_road_rejection
        );
        let node = core.region_graph.node(junction);
        assert_eq!(node.lane_connections.get(&(4, 1)), Some(&vec![(1, -1)]));
        assert_eq!(node.lane_connections.get(&(4, 2)), Some(&vec![(2, -1)]));
        assert!(!node.lane_connections.contains_key(&(0, 0)));
        assert_eq!(node.crosswalk_overrides.get(&4), Some(&true));
        assert!(!node.crosswalk_overrides.contains_key(&0));
    }

    #[test]
    fn midroad_count_edit_forks_and_merges_through_reserved_connectors() {
        let mut core = SimCore::new(WorldConfig {
            width_m: 512.0,
            height_m: 512.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        });
        let a = core
            .region_graph
            .add_node(Vector3::new(-180.0, 0.0, 0.0), NodeType::Junction);
        let mid = core
            .region_graph
            .add_node(Vector3::ZERO, NodeType::Junction);
        let b = core
            .region_graph
            .add_node(Vector3::new(180.0, 0.0, 0.0), NodeType::Junction);
        for (start, end) in [(a, mid), (mid, b)] {
            let points = vec![
                core.region_graph.node(start).pos,
                core.region_graph.node(end).pos,
            ];
            core.region_graph.add_edge(Edge {
                start_node: start,
                end_node: end,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                fwd_lanes: 1,
                bkw_lanes: 1,
                width: 7.0,
                speed_limit: 50.0 / 3.6,
                physical_length: 180.0,
                physical_geometry: points.clone(),
                geometry: points,
                ..Default::default()
            });
        }
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core.precompute_road_mesh_data();
        assert!(
            core.change_road_cross_section(1, 3, 3, None),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(
            core.region_graph.node(mid).node_type,
            NodeType::LaneTransition
        );
        assert_eq!(core.region_graph.edge(1).start_clip, 28.0);
        let start = &core.transit_network.road_surface.compiled_sections()[&1][0];
        let carriage: Vec<_> = start
            .bands
            .iter()
            .filter(|b| {
                b.kind == crate::simulation::network::surface::RoadSurfaceBandKind::Carriageway
            })
            .collect();
        assert_eq!(carriage.first().unwrap().lateral_start_m, -3.5);
        assert_eq!(carriage.last().unwrap().lateral_end_m, 3.5);

        let source = *core.transit_network.lane_system.edge_lanes[&0]
            .iter()
            .find(|&&id| {
                let l = &core.transit_network.lane_system.lanes[id];
                l.is_fwd && l.lane_type == LaneType::Vehicle
            })
            .unwrap();
        assert_eq!(
            core.transit_network.lane_system.lanes[source]
                .next_lanes
                .len(),
            3
        );
        for &connection in &core.transit_network.lane_system.lanes[source].next_lanes {
            let connector = &core.transit_network.lane_system.lanes[connection];
            assert_eq!(connector.edge_id, usize::MAX);
            assert!(connector.length + 0.001 >= 28.0);
            let target = connector.next_lanes[0];
            assert!(
                (core.transit_network.lane_system.lanes[target].geometry[0].x - 28.0).abs() < 0.001
            );
        }
    }
    #[test]
    fn kerb_pocket_uses_one_taper_on_an_undivided_road() {
        let mut core = fixture();
        assert!(core.change_road_cross_section(0, 2, 2, None));
        assert!(
            core.change_road_pocket(0, false, false, true, 80.0, 35.0),
            "{:?}",
            core.last_road_rejection
        );
        let start = &core.transit_network.road_surface.compiled_sections()[&4][0];
        let carriage: Vec<_> = start
            .bands
            .iter()
            .filter(|b| {
                b.kind == crate::simulation::network::surface::RoadSurfaceBandKind::Carriageway
            })
            .collect();
        assert_eq!(carriage.first().unwrap().lateral_start_m, -7.0);
        assert_eq!(carriage.last().unwrap().lateral_end_m, 7.0);
        let full = core.region_graph.edge(4).lane_section_at(35.0);
        assert_eq!((full.left, full.right), (-7.0, 10.5));
    }
    #[test]
    fn pocket_roles_resize_remove_and_preview_cover_both_stored_directions() {
        use crate::simulation::network::pockets::{PocketLayout, PocketRequest};
        for at_start in [false, true] {
            for (left, right) in [(true, false), (false, true), (true, true)] {
                for count in 1..=3 {
                    if count + u8::from(left) + u8::from(right) > 4 {
                        continue;
                    }
                    let mut core = fixture();
                    if at_start {
                        let edge = core.region_graph.edge_mut(0);
                        std::mem::swap(&mut edge.start_node, &mut edge.end_node);
                        edge.geometry.reverse();
                        edge.physical_geometry.reverse();
                    }
                    assert!(core.change_road_cross_section(
                        0,
                        count,
                        count,
                        RoadMedian::from_tool(2, 5.0).unwrap()
                    ));
                    let request = PocketRequest {
                        edge: 0,
                        at_start,
                        left,
                        right,
                        length: 80.0,
                        taper: 35.0,
                    };
                    let before_count = core.region_graph.edge_count();
                    let preview = core
                        .transit_network
                        .road_surface
                        .compile_pocket_preview(&core.region_graph, &core.heightmap, request)
                        .unwrap()
                        .0;
                    assert!(!preview.planned.is_empty());
                    assert_eq!(
                        core.region_graph.edge_count(),
                        before_count,
                        "preview is read-only"
                    );
                    let mut predicted = core.region_graph.clone();
                    let layout = PocketLayout::prepare(&predicted, request).unwrap();
                    let expected_ids = layout.apply(&mut predicted);
                    let mut network = crate::simulation::network::TransitNetwork::new();
                    network
                        .bulk_dirty_edges
                        .extend(expected_ids.iter().copied());
                    network.finalize_road_geometry(&mut predicted);
                    assert!(
                        core.change_road_pocket(0, at_start, left, right, 80.0, 35.0),
                        "{:?}",
                        core.last_road_rejection
                    );
                    for id in expected_ids {
                        let expected = predicted.edge(id);
                        let actual = core.region_graph.edge(id);
                        assert_eq!(actual.geometry, expected.geometry);
                        assert_eq!(actual.physical_geometry, expected.physical_geometry);
                        assert_eq!(
                            (actual.start_clip, actual.end_clip),
                            (expected.start_clip, expected.end_clip)
                        );
                        assert_eq!(actual.lane_tapers, expected.lane_tapers);
                    }
                    let pocket_id = if at_start { 0 } else { 4 };
                    let node = if at_start {
                        core.region_graph.edge(pocket_id).end_node
                    } else {
                        core.region_graph.edge(pocket_id).start_node
                    };
                    let old_pos = core.region_graph.node(node).pos;
                    assert!(
                        core.change_road_pocket(pocket_id, at_start, left, right, 90.0, 25.0),
                        "{:?}",
                        core.last_road_rejection
                    );
                    let pocket = core.region_graph.edge(pocket_id);
                    let body = pocket.physical_length
                        - 25.0
                        - if at_start {
                            pocket.start_clip
                        } else {
                            pocket.end_clip
                        };
                    assert!((body - 90.0).abs() < 0.001);
                    // Length+ taper stays constant; changing their sum moves the same hidden node.
                    assert!(core.region_graph.node(node).pos.distance_to(old_pos) < 0.001);
                    assert!(core.change_road_pocket(pocket_id, at_start, left, right, 100.0, 25.0));
                    assert!(core.region_graph.node(node).pos.distance_to(old_pos) > 9.9);
                    assert!(
                        core.change_road_pocket(pocket_id, at_start, false, false, 100.0, 25.0)
                    );
                    assert!(!core.region_graph.node_has_live_incident_edge(node));
                    assert!(core.region_graph.edge(4).deleted);
                    assert_eq!(
                        (
                            core.region_graph.edge(0).fwd_lanes,
                            core.region_graph.edge(0).bkw_lanes
                        ),
                        (count, count)
                    );
                    core.undo_action_internal();
                    assert!(core.region_graph.edge(pocket_id).pocket_taper().is_some());
                }
            }
        }
    }
    #[test]
    fn pocket_connector_survives_save_load_without_position_jump() {
        let mut core = fixture();
        assert!(core.change_road_pocket(0, false, true, true, 80.0, 35.0));
        let node = core.region_graph.edge(4).start_node;
        let lane = *core.transit_network.lane_system.node_lanes[&(node as usize)]
            .iter()
            .find(|&&id| core.transit_network.lane_system.lanes[id].lane_type == LaneType::Vehicle)
            .unwrap();
        let agent = core
            .agents
            .spawn_border_arrival_agent(usize::MAX, node, 0.0, 0.0);
        let d = core.transit_network.lane_system.lanes[lane].length * 0.45;
        core.agents.agents.transit_mode[agent] = MODE_CAR;
        core.agents
            .agents
            .enter_lane(agent, &core.transit_network.lane_system, lane, d, node);
        let p = crate::simulation::network::lanes::geometry::agent_lane_position(
            &core.transit_network.lane_system.lanes[lane],
            d,
            None,
        )
        .unwrap();
        core.agents.agents.pos_x[agent] = p.x;
        core.agents.agents.pos_y[agent] = p.z;
        let path = std::env::temp_dir().join(format!(
            "roads-b-pocket-connector-{}.sqlite",
            std::process::id()
        ));
        core.save_game_internal(path.to_str().unwrap(), None)
            .unwrap();
        core.load_game_internal(path.to_str().unwrap()).unwrap();
        std::fs::remove_file(&path).unwrap();
        let id = core.agents.agents.current_lane_id[agent];
        assert_ne!(id, usize::MAX);
        assert_eq!(
            core.transit_network.lane_system.lanes[id].edge_id,
            usize::MAX
        );
        assert!((core.agents.agents.pos_x[agent] - p.x).abs() < 0.001);
        assert!((core.agents.agents.pos_y[agent] - p.z).abs() < 0.001);
    }
    #[test]
    fn one_way_right_pocket_retains_the_through_lane_alignment() {
        let mut core = fixture();
        assert!(core.change_road_cross_section(0, 1, 0, None));
        assert!(
            core.change_road_pocket(0, false, false, true, 80.0, 35.0),
            "{:?}",
            core.last_road_rejection
        );
        let lanes = &core.transit_network.lane_system;
        let through = lanes.edge_lanes[&4]
            .iter()
            .find_map(|&id| {
                let lane = &lanes.lanes[id];
                (lane.lane_type == LaneType::Vehicle && lane.lane_idx == 0).then_some(lane)
            })
            .unwrap();
        assert!(through.geometry.iter().all(|p| p.z.abs() < 0.001));
    }
    #[test]
    fn mixed_lane_gain_and_drop_connect_every_incoming_lane() {
        use crate::simulation::network::graph::RegionGraph;
        use crate::simulation::network::{TransitNetwork, pockets::author_lane_count_transitions};
        let mut graph = RegionGraph::new();
        let a = graph.add_node(Vector3::new(-120.0, 0.0, 0.0), NodeType::Junction);
        let node = graph.add_node(Vector3::ZERO, NodeType::Junction);
        let b = graph.add_node(Vector3::new(120.0, 0.0, 0.0), NodeType::Junction);
        for (start, end, fwd, bkw) in [(a, node, 3, 1), (node, b, 2, 2)] {
            let geometry = vec![graph.node(start).pos, graph.node(end).pos];
            graph.add_edge(Edge {
                start_node: start,
                end_node: end,
                geometry: geometry.clone(),
                physical_geometry: geometry,
                physical_length: 120.0,
                width: 14.0,
                fwd_lanes: fwd,
                bkw_lanes: bkw,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                ..Default::default()
            });
        }
        let mut dirty = HashSet::from([0, 1]);
        author_lane_count_transitions(&mut graph, &HashSet::from([node]), &mut dirty);
        graph.rebuild_intersection_clips();
        let mut network = TransitNetwork::new();
        network.lane_system.rebuild(&mut graph);
        assert_eq!(graph.node(node).node_type, NodeType::LaneTransition);
        for id in [0, 1] {
            for &lane in &network.lane_system.edge_lanes[&id] {
                let lane = &network.lane_system.lanes[lane];
                if lane.lane_type == LaneType::Vehicle && lane.is_fwd == (id == 0) {
                    assert!(!lane.next_lanes.is_empty(), "every dropped lane must merge");
                }
            }
        }
    }
    #[test]
    fn opposing_end_pockets_keep_the_middle_road_and_remove_independently() {
        let mut core = fixture();
        let west = core.region_graph.edge(0).start_node;
        let p = Vector3::new(-440.0, 0.0, 0.0);
        core.region_graph.set_node_pos(west, p);
        core.region_graph.edge_mut(0).geometry[0] = p;
        core.region_graph.edge_mut(0).physical_geometry[0] = p;
        core.region_graph.edge_mut(0).physical_length = 440.0;
        for z in [-150.0, 150.0] {
            let end = core
                .region_graph
                .add_node(Vector3::new(-440.0, 0.0, z), NodeType::Junction);
            let geometry = vec![p, core.region_graph.node(end).pos];
            core.region_graph.add_edge(Edge {
                start_node: west,
                end_node: end,
                geometry: geometry.clone(),
                physical_geometry: geometry,
                physical_length: 150.0,
                width: 7.0,
                fwd_lanes: 1,
                bkw_lanes: 1,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                ..Default::default()
            });
        }
        core.region_graph.rebuild_all_indices();
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        assert!(core.change_road_pocket(0, false, true, true, 80.0, 35.0));
        assert!(
            core.change_road_pocket(0, true, true, true, 80.0, 35.0),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(
            (
                core.region_graph.edge(0).fwd_lanes,
                core.region_graph.edge(0).bkw_lanes
            ),
            (2, 4)
        );
        assert_eq!(
            (
                core.region_graph.edge(6).fwd_lanes,
                core.region_graph.edge(6).bkw_lanes
            ),
            (4, 2)
        );
        assert_eq!(
            (
                core.region_graph.edge(7).fwd_lanes,
                core.region_graph.edge(7).bkw_lanes
            ),
            (2, 2)
        );
        assert!(core.change_road_pocket(0, true, false, false, 80.0, 35.0));
        assert!(core.region_graph.edge(6).pocket_taper().is_some());
        assert!(core.region_graph.edge(7).deleted);
        assert_eq!(
            (
                core.region_graph.edge(0).fwd_lanes,
                core.region_graph.edge(0).bkw_lanes
            ),
            (2, 2)
        );
    }

    #[test]
    fn moving_a_pocket_and_undo_preserve_a_cars_world_station() {
        let mut core = fixture();
        assert!(core.change_road_pocket(0, false, true, true, 80.0, 35.0));
        let lane = *core.transit_network.lane_system.edge_lanes[&4]
            .iter()
            .find(|&&id| {
                let lane = &core.transit_network.lane_system.lanes[id];
                lane.lane_type == LaneType::Vehicle && lane.is_fwd && lane.lane_idx == 1
            })
            .unwrap();
        let origin = core.region_graph.edge(4).start_node;
        let agent = core
            .agents
            .spawn_border_arrival_agent(usize::MAX, origin, -70.0, 4.25);
        core.agents.agents.transit_mode[agent] = MODE_CAR;
        let d =
            crate::simulation::buildings::allocator::BuildingAllocator::project_point_to_polyline_s(
                &core.transit_network.lane_system.lanes[lane].geometry,
                godot::prelude::Vector2::new(-70.0, 4.25),
            );
        core.agents
            .agents
            .enter_lane(agent, &core.transit_network.lane_system, lane, d, origin);
        core.agents.agents.speed[agent] = 4.0;
        assert!(core.change_road_pocket(4, false, true, true, 100.0, 35.0));
        assert!((core.agents.agents.pos_x[agent] + 70.0).abs() < 0.001);
        assert!(core.undo_action_internal());
        assert!((core.agents.agents.pos_x[agent] + 70.0).abs() < 0.001);
        assert!((core.agents.agents.pos_y[agent] - 4.25).abs() < 0.001);
        assert_eq!(core.agents.agents.speed[agent], 4.0);
    }
    #[test]
    fn pocket_removal_preview_restores_terrain_beneath_the_retired_right_lane() {
        use crate::simulation::network::pockets::PocketRequest;
        use godot::prelude::Vector2;
        let mut core = fixture();
        assert!(core.change_road_pocket(0, false, true, true, 80.0, 35.0));
        core.allocator
            .prepare_building_site_query_index(core.config.zone_cell_m);
        let edge = (0..core.region_graph.edge_count())
            .find(|&id| core.region_graph.edge(id).pocket_taper().is_some())
            .unwrap();
        let mut covered = Vec::new();
        for right in [true, false] {
            let (_, roads) = core
                .transit_network
                .road_surface
                .compile_pocket_preview(
                    &core.region_graph,
                    &core.heightmap,
                    PocketRequest {
                        edge,
                        at_start: false,
                        left: right,
                        right,
                        length: 80.0,
                        taper: 35.0,
                    },
                )
                .unwrap();
            let plan = core.compile_pocket_preview_terrain(&roads).unwrap();
            let patches = plan.preview_patches().unwrap();
            assert!(!patches.is_empty());
            covered.push(patches.iter().any(|patch| {
                let Some(mesh) = &patch.mesh_buffers else {
                    return false;
                };
                let point = Vector2::new(
                    -50.0 - patch.patch.world_origin_x - patch.patch.world_size_x * 0.5,
                    12.5 - patch.patch.world_origin_z - patch.patch.world_size_z * 0.5,
                );
                mesh.terrain_indices.as_chunks::<3>().0.iter().any(|ids| {
                    let v = [ids[0], ids[1], ids[2]].map(|id| {
                        let p = mesh.terrain_vertices[id as usize];
                        Vector2::new(p.x, p.z)
                    });
                    let cross = [0, 1, 2].map(|i| (v[(i + 1) % 3] - v[i]).cross(point - v[i]));
                    cross.iter().all(|&x| x >= -0.001) || cross.iter().all(|&x| x <= 0.001)
                })
            }));
        }
        assert_eq!(
            covered,
            [false, true],
            "preview terrain must fill the old road cutout on removal"
        );
    }
}
