// SPDX-License-Identifier: GPL-2.0-only

//! Authored lane access, independent of the physical carriageway and junction movement policy.

use super::{Edge, RegionGraph};
use crate::simulation::network::types::{NodeType, TransitType};

/// Vehicle class permission for one physical lane ribbon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaneUse {
    /// Cars and future public transport share the lane.
    General,
    /// Reserved for public transport; no longitudinal car travel or frontage access.
    BusOnly,
}

impl Edge {
    /// Returns the class permission of a centre-outward lane in one stored direction.
    /// Future bus routing can retain all ribbons while car routing filters `BusOnly`.
    pub fn lane_use(&self, is_fwd: bool, rank: u8) -> LaneUse {
        if rank < 8 && self.bus_lanes[usize::from(!is_fwd)] & (1 << rank) != 0 {
            LaneUse::BusOnly
        } else {
            LaneUse::General
        }
    }

    /// Number of lanes available to cars in this direction, without allocation or scanning.
    pub fn car_lane_count(&self, is_fwd: bool) -> u8 {
        let count = if is_fwd {
            self.fwd_lanes
        } else {
            self.bkw_lanes
        };
        let bus = self.bus_lanes[usize::from(!is_fwd)];
        if bus == 0 {
            return count;
        }
        let mask = ((1_u16 << count.min(8)) - 1) as u8;
        count - (bus & mask).count_ones() as u8
    }

    pub(crate) fn signed_lane_is_bus(&self, lane: i8) -> bool {
        let rank = if lane >= 0 {
            i16::from(lane)
        } else {
            -i16::from(lane) - 1
        };
        self.lane_use(lane >= 0, rank as u8) == LaneUse::BusOnly
    }

    pub(crate) fn car_lane_rank(&self, is_fwd: bool, rank: usize) -> usize {
        let bus = self.bus_lanes[usize::from(!is_fwd)];
        if bus == 0 {
            return rank;
        }
        let before = ((1_u16 << rank.min(8)) - 1) as u8;
        rank - (bus & before).count_ones() as usize
    }

    pub(crate) fn validate_bus_lanes(&self, masks: [u8; 2]) -> Result<(), &'static str> {
        if self.deleted || self.primary_type != TransitType::Road {
            return Err("bus_lane_requires_road");
        }
        for (mask, count) in masks.into_iter().zip([self.fwd_lanes, self.bkw_lanes]) {
            if u16::from(mask) >= 1_u16 << count.min(8) {
                return Err("bus_lane_out_of_range");
            }
        }
        // Pocket rank shifts need a combined authoring operation; never silently move a use.
        if masks != [0; 2] && self.lane_tapers.is_some() {
            return Err("bus_lane_overlaps_taper");
        }
        Ok(())
    }
}

impl RegionGraph {
    /// Validates saved permissions and the two endpoint types in O(1), without allocation.
    /// A taper and its narrow base need combined rank authoring before either can reserve lanes.
    pub(crate) fn validate_bus_lane_edit(
        &self,
        id: usize,
        masks: [u8; 2],
    ) -> Result<(), &'static str> {
        let edge = self.get_edge(id).ok_or("invalid_road")?;
        edge.validate_bus_lanes(masks)?;
        if masks != [0; 2]
            && [edge.start_node, edge.end_node]
                .into_iter()
                .any(|node| self.node(node).node_type == NodeType::LaneTransition)
        {
            return Err("bus_lane_overlaps_taper");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bus_lane_edit_rejects_the_narrow_transition_base_and_allows_clearing() {
        let mut graph = RegionGraph::new();
        let start = graph.add_node(godot::prelude::Vector3::ZERO, NodeType::Junction);
        let end = graph.add_node(
            godot::prelude::Vector3::RIGHT * 100.0,
            NodeType::LaneTransition,
        );
        let id = graph.add_edge(Edge {
            start_node: start,
            end_node: end,
            primary_type: TransitType::Road,
            fwd_lanes: 2,
            bkw_lanes: 2,
            width: 14.0,
            physical_length: 100.0,
            geometry: vec![graph.node(start).pos, graph.node(end).pos],
            physical_geometry: vec![graph.node(start).pos, graph.node(end).pos],
            ..Default::default()
        });
        assert_eq!(
            graph.validate_bus_lane_edit(id, [2, 0]),
            Err("bus_lane_overlaps_taper")
        );
        assert!(graph.validate_bus_lane_edit(id, [0; 2]).is_ok());
    }

    #[test]
    fn bus_lane_masks_preserve_physical_rank_and_filter_car_access() {
        let edge = Edge {
            primary_type: TransitType::Road,
            fwd_lanes: 3,
            bkw_lanes: 2,
            bus_lanes: [0b010, 0b11],
            ..Default::default()
        };
        assert_eq!(edge.lane_use(true, 1), LaneUse::BusOnly);
        assert_eq!(edge.lane_use(true, 2), LaneUse::General);
        assert_eq!(edge.car_lane_count(true), 2);
        assert_eq!(edge.car_lane_rank(true, 2), 1);
        assert_eq!(edge.car_lane_count(false), 0);
        assert_eq!(
            edge.validate_bus_lanes([8, 0]),
            Err("bus_lane_out_of_range")
        );
        assert!(edge.validate_bus_lanes([7, 3]).is_ok());
    }
}
