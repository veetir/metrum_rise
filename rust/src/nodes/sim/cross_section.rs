// SPDX-License-Identifier: GPL-2.0-only

//! Transactional in-place road cross-section edits, using the existing road undo/render gates.

use super::core::{RoadRejection, SimCore};
use crate::simulation::network::graph::RoadMedian;
use crate::simulation::network::lanes::LaneSystem;
use crate::simulation::network::surface::RoadSurfaceSystem;
use crate::simulation::network::types::EdgeClass;
use std::collections::HashSet;

impl SimCore {
    /// Changes one live road without replacing its topology. Local discovery is O(local degree),
    /// geometry O(affected samples); existing agent/access/CCH publication costs are unchanged.
    pub(crate) fn change_road_cross_section(
        &mut self,
        id: usize,
        fwd: u8,
        bkw: u8,
        median: Option<RoadMedian>,
    ) -> bool {
        self.last_road_rejection = None;
        let Some(edge) = self.region_graph.get_edge(id).filter(|edge| !edge.deleted) else {
            return self.refuse_cross_section("invalid_road");
        };
        let edits_pocket_base = [edge.start_node, edge.end_node].into_iter().any(|node| {
            self.region_graph.node(node).node_type
                == crate::simulation::network::types::NodeType::LaneTransition
                && self.region_graph.node_adjacency(node).iter().any(|&other| {
                    other != id && self.region_graph.edge(other).pocket_taper().is_some()
                })
        });
        if edge.pocket_taper().is_some() || edits_pocket_base {
            return self.refuse_cross_section("edit_pocket_controls");
        }
        if let Err(reason) = edge.validate_cross_section(fwd, bkw, median) {
            return self.refuse_cross_section(reason);
        }
        if fwd > 4 || bkw > 4 || (fwd == 0 && bkw == 0) {
            return self.refuse_cross_section("cross_section_requires_vehicle_lanes");
        }
        if edge.fwd_lanes == fwd && edge.bkw_lanes == bkw && edge.median == median {
            return true;
        }
        let nodes = HashSet::from([edge.start_node, edge.end_node]);
        let mut dirty = HashSet::from([id]);
        for &node in &nodes {
            dirty.extend(
                self.region_graph
                    .node_adjacency(node)
                    .iter()
                    .copied()
                    .filter(|&id| !self.region_graph.edge(id).deleted),
            );
        }
        let undo_nodes = dirty
            .iter()
            .flat_map(|&id| {
                let edge = self.region_graph.edge(id);
                [edge.start_node, edge.end_node]
            })
            .collect();
        self.transit_network.begin_road_edit();
        self.push_network_undo_for_local_topology(dirty.clone(), undo_nodes);
        self.zoning
            .mark_cell_lots_for_roads(&self.region_graph, dirty.iter().copied());
        self.transit_network
            .mark_surface_dirty_from_sets(&self.region_graph, &dirty, &nodes);
        if let Err(reason) = self
            .region_graph
            .set_edge_cross_section(id, fwd, bkw, median)
        {
            self.rollback_staged_road_edit();
            return self.refuse_cross_section(reason);
        }
        let edge = self.region_graph.edge(id);
        let half_width = RoadSurfaceSystem::visual_roadbed_half_width_m(edge);
        let mut geometry_violations = Vec::new();
        crate::simulation::network::validate::validate_edge(
            &self.region_graph,
            id,
            &crate::simulation::network::validate::RoadGeometryLimits::default(),
            &mut geometry_violations,
        );
        if geometry_violations.iter().any(|violation| {
            matches!(
                violation,
                crate::simulation::network::validate::RoadGeometryViolation::HorizontalRadius {
                    tier: crate::simulation::network::validate::RoadGeometryTier::Illegal,
                    ..
                }
            )
        }) {
            self.rollback_staged_road_edit();
            return self.refuse_cross_section("cross_section_too_wide_for_curve");
        }
        let points = &edge.physical_geometry;
        // Widening cannot silently consume painted lots, fields or existing building parcels.
        let refusal = if edge.class == EdgeClass::Standard
            && self
                .watermap
                .road_corridor_overlaps_visible_water(points, half_width)
        {
            Some("water_requires_bridge")
        } else if self.zoning.cells_overlap_road_corridor(points, half_width) {
            Some("zoning_cell_overlap")
        } else if self
            .allocator
            .field_clearance
            .overlaps_road_corridor(points, half_width)
        {
            Some("field_overlap")
        } else if !self
            .zoning
            .parcel_ids_overlapping_road_corridor(points, half_width)
            .is_empty()
        {
            Some("parcel_overlap")
        } else {
            None
        };
        if let Some(reason) = refusal {
            self.rollback_staged_road_edit();
            return self.refuse_cross_section(reason);
        }
        self.finish_road_section_edit(dirty)
    }

    pub(super) fn finish_road_section_edit(&mut self, mut dirty: HashSet<usize>) -> bool {
        self.transit_network
            .bulk_dirty_edges
            .extend(dirty.iter().copied());
        let finalized = self
            .transit_network
            .finalize_road_geometry(&mut self.region_graph);
        dirty.extend(finalized.dirty_edges);
        let mut ordered: Vec<_> = dirty.iter().copied().collect();
        ordered.sort_unstable();
        for id in ordered {
            if let Some(refusal) = self
                .transit_network
                .road_surface
                .crossing_clearance_violation(
                    &self.region_graph,
                    id,
                    &self.region_graph,
                    &HashSet::from([id]),
                )
            {
                self.rollback_staged_road_edit();
                self.last_road_rejection = Some(RoadRejection::from_validation(
                    refusal.invalid_reason,
                    &refusal,
                    &[],
                ));
                return false;
            }
        }
        self.zoning
            .mark_cell_lots_for_roads(&self.region_graph, dirty.iter().copied());
        self.mark_local_network_render_dirty();
        if !self.validate_staged_road_render_with_plan(None) {
            return false;
        }
        // Match the closure used by the lane builder before any old connector IDs are retired.
        let lane_edges = LaneSystem::incremental_rebuild_edge_closure(&self.region_graph, &dirty);
        self.agents.invalidate_lane_ids_for_edges(
            &lane_edges,
            &self.transit_network.lane_system,
            &self.region_graph,
        );
        self.transit_network
            .lane_system
            .rebuild_edges_incremental(&mut self.region_graph, &dirty);
        self.agents.reattach_invalidated_lanes_for_edges(
            &lane_edges,
            &self.transit_network.lane_system,
            &self.region_graph,
        );
        self.rebuild_building_entrances_internal();
        self.transit_network
            .rebuild_cch_and_check(&self.region_graph);
        self.transit_network.flow_fields.mark_all_dirty();
        self.precompute_road_mesh_data();
        self.terrain_dirty = true;
        true
    }

    pub(super) fn refuse_cross_section(&mut self, reason: &'static str) -> bool {
        self.last_road_rejection = Some(RoadRejection::new(reason, reason));
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::core::config::WorldConfig;
    use crate::simulation::economy::agents::{MODE_CAR, TRANSIT_NETWORK};
    use crate::simulation::network::graph::{Edge, MedianStyle};
    use crate::simulation::network::lanes::LaneType;
    use crate::simulation::network::types::{NodeType, TransitFlags, TransitType};
    use godot::prelude::Vector3;

    #[test]
    fn crossing_override_rebuilds_median_surface_and_undo_restores_continuity() {
        let mut core = SimCore::new(WorldConfig {
            width_m: 512.0,
            height_m: 512.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        });
        let center = core
            .region_graph
            .add_node(Vector3::ZERO, NodeType::Junction);
        for end in [
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(90.0, 0.0, 40.0),
        ] {
            let end_node = core.region_graph.add_node(end, NodeType::Junction);
            let points = vec![Vector3::ZERO, end];
            core.region_graph.add_edge(Edge {
                start_node: center,
                end_node,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
                fwd_lanes: 1,
                bkw_lanes: 1,
                width: 12.0,
                median: RoadMedian::from_tool(2, 5.0).unwrap(),
                physical_length: end.length(),
                speed_limit: 50.0 / 3.6,
                geometry: points.clone(),
                physical_geometry: points,
                ..Default::default()
            });
        }
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core.precompute_road_mesh_data();
        let before = core.transit_network.road_surface.compiled_sections()[&0].clone();
        assert!(core.region_graph.median_continues_at_node(0, center));
        core.set_crosswalk_override_internal(center, 0, true);
        assert!(!core.region_graph.median_continues_at_node(0, center));
        assert!(
            core.transit_network
                .road_surface
                .published_generation_matches_source()
        );
        assert_ne!(
            core.transit_network.road_surface.compiled_sections()[&0],
            before
        );
        assert!((0..core.transit_network.lane_system.lanes.len()).any(|id| {
            core.transit_network.lane_system.lanes[id].node_id == center as usize
                && core.transit_network.lane_system.crosswalk(id).is_some()
        }));
        assert!(core.undo_action_internal());
        assert!(core.region_graph.median_continues_at_node(0, center));
        assert_eq!(
            core.transit_network.road_surface.compiled_sections()[&0],
            before
        );
    }

    #[test]
    fn in_place_cross_section_keeps_edge_and_agents_valid_and_undo_restores_profile() {
        let mut core = SimCore::new(WorldConfig {
            width_m: 512.0,
            height_m: 512.0,
            terrain_cell_m: 4.0,
            terrain_chunk_m: 128.0,
            ..WorldConfig::default()
        });
        let start = core
            .region_graph
            .add_node(Vector3::new(-80.0, 0.0, 0.0), NodeType::Junction);
        let end = core
            .region_graph
            .add_node(Vector3::new(80.0, 0.0, 0.0), NodeType::Junction);
        let points = vec![
            core.region_graph.node(start).pos,
            core.region_graph.node(end).pos,
        ];
        let id = core.region_graph.add_edge(Edge {
            start_node: start,
            end_node: end,
            primary_type: TransitType::Road,
            allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
            fwd_lanes: 2,
            bkw_lanes: 2,
            width: 14.0,
            physical_length: 160.0,
            speed_limit: 50.0 / 3.6,
            geometry: points.clone(),
            physical_geometry: points.clone(),
            ..Default::default()
        });
        core.region_graph.rebuild_intersection_clips();
        core.transit_network
            .lane_system
            .rebuild(&mut core.region_graph);
        core.precompute_road_mesh_data();
        let lane = core.transit_network.lane_system.edge_lanes[&id]
            .iter()
            .copied()
            .find(|&lid| {
                let lane = &core.transit_network.lane_system.lanes[lid];
                lane.lane_type == LaneType::Vehicle && lane.is_fwd && lane.lane_idx == 1
            })
            .unwrap();
        let agent = core
            .agents
            .spawn_border_arrival_agent(usize::MAX, start, -20.0, 5.25);
        core.agents.agents.transit[agent] = TRANSIT_NETWORK;
        core.agents.agents.transit_mode[agent] = MODE_CAR;
        core.agents.agents.current_edge[agent] = id;
        core.agents.agents.current_lane_id[agent] = lane;
        core.agents.agents.lane_distance[agent] = 60.0;
        let median = Some(RoadMedian {
            style: MedianStyle::KerbedGrass,
            width_m: 5.0,
            start_setback_m: 0.0,
            end_setback_m: 0.0,
        });
        assert!(
            core.change_road_cross_section(id, 1, 1, median),
            "{:?}",
            core.last_road_rejection
        );
        assert_eq!(core.region_graph.edge_count(), 1);
        assert_eq!(core.region_graph.edge(id).geometry, points);
        assert_eq!(core.region_graph.edge(id).median, median);
        let current = core.agents.agents.current_lane_id[agent];
        assert!(core.transit_network.lane_system.edge_lanes[&id].contains(&current));
        assert!(core.transit_network.lane_system.lanes[current].is_fwd);
        assert!(core.undo_action_internal());
        assert_eq!(core.region_graph.edge(id).median, None);
        assert_eq!(core.region_graph.edge(id).width, 14.0);
        assert_eq!(core.region_graph.edge(id).fwd_lanes, 2);
        let current = core.agents.agents.current_lane_id[agent];
        assert!(core.transit_network.lane_system.edge_lanes[&id].contains(&current));
        let undo_count = core.undo_stack.len();
        assert!(!core.change_road_cross_section(id, 1, 0, median));
        assert_eq!(
            core.last_road_rejection.as_ref().unwrap().reason,
            "median_requires_both_directions"
        );
        assert_eq!(core.undo_stack.len(), undo_count);
        assert_eq!(core.region_graph.edge(id).median, None);
    }
}
