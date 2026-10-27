// SPDX-License-Identifier: GPL-2.0-only

//! Authored zebra authority, independent of legal unmarked pedestrian crossings.

use super::RegionGraph;
use crate::simulation::network::types::{NodeType, TransitFlags, TransitType};

impl RegionGraph {
    /// Whether this arm has authored zebra paint. Importers must explicitly set marked OSM
    /// crossings through `set_crosswalk_override`; absent tags never create a zebra.
    /// O(1), allocation-free; queried only while compiling roads and lanes.
    pub fn arm_has_zebra(&self, node: u32, edge: usize) -> bool {
        self.crosswalk_arm_eligible(node, edge)
            && self.node(node).crosswalk_overrides.get(&edge) == Some(&true)
    }

    pub(crate) fn crosswalk_arm_eligible(&self, node: u32, edge: usize) -> bool {
        (node as usize) < self.node_count()
            && !matches!(
                self.node(node).node_type,
                NodeType::LaneTransition | NodeType::Ramp
            )
            && self.get_edge(edge).is_some_and(|road| {
                !road.deleted
                    && road.primary_type == TransitType::Road
                    && road.allowed_types & TransitFlags::FOOT != 0
                    && road.physical_geometry.len() >= 2
                    && (road.start_node == node || road.end_node == node)
            })
    }

    /// Freezes pre-v95 procedural markings as explicit authoring on load. Old files contain
    /// no import provenance, so all their roads retain their existing zebras. Integration
    /// must clear both ends of imported roads before applying the actual OSM crossing tags.
    pub(crate) fn preserve_legacy_zebras(&mut self) {
        for node in 0..self.node_count() as u32 {
            let mut arms: Vec<_> = self
                .node_adjacency(node)
                .iter()
                .copied()
                .filter(|&id| self.crosswalk_arm_eligible(node, id))
                .collect();
            let angle = |id| {
                let e = self.edge(id);
                let p = if e.start_node == node {
                    e.physical_geometry[1]
                } else {
                    e.physical_geometry[e.physical_geometry.len() - 2]
                };
                let d = p - self.node(node).pos;
                d.z.atan2(d.x)
            };
            arms.sort_by(|&a, &b| angle(a).total_cmp(&angle(b)).then(a.cmp(&b)));
            let degree = arms.len();
            let mut selected = Vec::new();
            for id in arms {
                if degree == 2 && !selected.is_empty() {
                    break;
                }
                if self.edge(id).median.is_some() && self.median_continues_at_node(id, node) {
                    continue;
                }
                selected.push(id);
            }
            for id in selected {
                self.set_crosswalk_override(node, id, true);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::graph::Edge;
    use godot::prelude::*;

    #[test]
    fn new_arms_are_unmarked_and_legacy_migration_is_deterministic() {
        let mut graph = RegionGraph::new();
        let node = graph.add_node(Vector3::ZERO, NodeType::Junction);
        for point in [Vector3::new(80.0, 0.0, 0.0), Vector3::new(-80.0, 0.0, 0.0)] {
            let end = graph.add_node(point, NodeType::Junction);
            graph.add_edge(Edge {
                start_node: node,
                end_node: end,
                primary_type: TransitType::Road,
                allowed_types: TransitFlags::FOOT | TransitFlags::CAR,
                geometry: vec![Vector3::ZERO, point],
                physical_geometry: vec![Vector3::ZERO, point],
                fwd_lanes: 1,
                bkw_lanes: 1,
                width: 7.0,
                physical_length: 80.0,
                ..Default::default()
            });
        }
        assert!(!graph.arm_has_zebra(node, 0));
        assert!(!graph.arm_has_zebra(node, 1));
        let mut lanes = crate::simulation::network::lanes::LaneSystem::new();
        lanes.rebuild(&mut graph);
        assert!((0..lanes.lanes.len()).all(|id| lanes.crosswalk_marking(id).is_none()));
        assert!(
            lanes.node_lanes[&(node as usize)].iter().any(|&id| {
                let lane = &lanes.lanes[id];
                lane.lane_type == crate::simulation::network::lanes::LaneType::Foot
                    && lane.length == 0.0
                    && lane.geometry.len() == 2
                    && lane
                        .next_lanes
                        .iter()
                        .any(|&next| lanes.lanes[next].edge_id == 1)
            }),
            "unmarked straight sidewalks keep stationary continuations"
        );
        let mut repeated = graph.clone();
        graph.preserve_legacy_zebras();
        repeated.preserve_legacy_zebras();
        assert_eq!(
            graph.node(node).crosswalk_overrides,
            repeated.node(node).crosswalk_overrides
        );
        assert_eq!(graph.node(node).crosswalk_overrides.len(), 1);
        graph.set_crosswalk_override(node, 0, true);
        graph.set_crosswalk_override(node, 1, false);
        assert!(graph.arm_has_zebra(node, 0));
        assert!(!graph.arm_has_zebra(node, 1));
        graph.nodes[node as usize].node_type = NodeType::Ramp;
        lanes.rebuild(&mut graph);
        assert!(
            lanes.node_lanes[&(node as usize)]
                .iter()
                .all(|&id| lanes.crosswalk(id).is_none()),
            "ramp barriers do not become unmarked crossing opportunities"
        );
    }
}
