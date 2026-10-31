// SPDX-License-Identifier: GPL-2.0-only

//! Saved edge-side permission, separate from roadbed geometry and widths.

use crate::simulation::network::graph::{Edge, RegionGraph};
use crate::simulation::network::types::{EdgeClass, NodeType, TransitType};

/// A permission on the existing carriageway, never a road profile band.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(crate) enum SideRule {
    /// This side offers no general parking.
    #[default]
    None,
    /// Kerbside parking without paint, the residential default.
    Unmarked,
    /// Player-selected marked bays; paint/sign content belongs to PARK-04.
    Marked,
}

/// Edge data stored alongside the edge records to avoid changing roadbed-owned constructors.
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct ParkingRule {
    pub sides: [SideRule; 2],
    pub automatic: bool,
    // Station offset preserves the six-metre lattice when an edge splits.
    pub origin_m: f32,
    pub lineage: u64,
}

impl ParkingRule {
    /// O(1) eligibility. The existing preset has no road hierarchy: higher speed 1+1 roads
    /// are excluded; a low-speed main road can explicitly set both sides to None in the tool.
    pub(crate) fn default_for(edge: &Edge) -> Self {
        let eligible = edge.primary_type == TransitType::Road
            && edge.class == EdgeClass::Standard
            && edge.fwd_lanes == 1
            && edge.bkw_lanes == 1
            && edge.speed_limit <= crate::config::DEFAULT_URBAN_ROAD_SPEED_MS;
        Self {
            sides: if eligible {
                [SideRule::None, SideRule::Unmarked]
            } else {
                [SideRule::None; 2]
            },
            automatic: true,
            origin_m: 0.0,
            lineage: 0,
        }
    }

    /// O(1), fewer mouths wins; ties use the forward edge's left side (+1).
    pub(crate) fn choose_side(&mut self, mouths: [usize; 2]) {
        if self.automatic && self.sides != [SideRule::None; 2] {
            self.sides = if mouths[0] < mouths[1] {
                [SideRule::Unmarked, SideRule::None]
            } else {
                [SideRule::None, SideRule::Unmarked]
            };
        }
    }
}

impl RegionGraph {
    /// Saved per-edge-side value in O(1); border connections never acquire default supply.
    pub(crate) fn parking_rule(&self, edge: usize) -> ParkingRule {
        let Some(record) = self.get_edge(edge) else {
            return ParkingRule::default();
        };
        let mut rule = self
            .parking_rules
            .get(&edge)
            .copied()
            .unwrap_or_else(|| ParkingRule::default_for(record));
        if rule.automatic {
            let defaults = ParkingRule::default_for(record).sides;
            if defaults == [SideRule::None; 2] || rule.sides == [SideRule::None; 2] {
                rule.sides = defaults;
            }
        }
        if rule.lineage == 0 {
            rule.lineage = edge as u64 + 1;
        }
        if rule.automatic
            && [record.start_node, record.end_node]
                .into_iter()
                .any(|n| self.node(self.get_valid_node(n)).node_type == NodeType::Border)
        {
            rule.sides = [SideRule::None; 2];
        }
        rule
    }

    /// O(1) authoritative property mutation; the caller rebuilds only this source's bays.
    pub(crate) fn set_parking_rule(&mut self, edge: usize, rule: ParkingRule) {
        if edge < self.edge_count() {
            self.parking_rules.insert(edge, rule);
        }
    }
}
