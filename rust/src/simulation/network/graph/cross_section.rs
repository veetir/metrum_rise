// SPDX-License-Identifier: GPL-2.0-only

//! Authored central separators and their validation; geometry remains roadbed-owned.

use super::Edge;
use crate::simulation::network::types::{TransitFlags, TransitType, VehicleFrontageAccess};

/// Surface treatment of a central separator, independent of its longitudinal extent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MedianStyle {
    /// Flush asphalt with white diagonal hatching; frontage turns may cross it.
    Painted = 1,
    /// A raised grass strip bounded by kerbs.
    KerbedGrass = 2,
    /// A raised paved strip bounded by kerbs.
    KerbedPaving = 3,
}

/// One median in metres. End setbacks are additional clearance beyond the automatic mouth
/// and zebra clearance, leaving additional authored clearance at a junction mouth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoadMedian {
    /// Total separator width including its two kerbs, when raised.
    pub width_m: f32,
    /// Surface and access treatment.
    pub style: MedianStyle,
    /// Additional clearance at the edge's stored start.
    pub start_setback_m: f32,
    /// Additional clearance at the edge's stored end.
    pub end_setback_m: f32,
}

impl RoadMedian {
    /// Decodes tool/save style codes, rejecting invalid widths instead of silently clamping.
    pub fn from_tool(style: i32, width_m: f32) -> Result<Option<Self>, &'static str> {
        let style = match style {
            0 => return Ok(None),
            1 => MedianStyle::Painted,
            2 => MedianStyle::KerbedGrass,
            3 => MedianStyle::KerbedPaving,
            _ => return Err("unknown_median_style"),
        };
        let median = Self {
            width_m,
            style,
            start_setback_m: 0.0,
            end_setback_m: 0.0,
        };
        median.validate()?;
        Ok(Some(median))
    }

    /// Checks finite, buildable dimensions. Wider grass strips share the same geometry cost.
    pub fn validate(self) -> Result<(), &'static str> {
        let (min, max) = match self.style {
            MedianStyle::Painted => (0.5, 2.0),
            MedianStyle::KerbedGrass => (2.0, 12.0),
            MedianStyle::KerbedPaving => (1.0, 3.0),
        };
        if !self.width_m.is_finite() || !(min..=max).contains(&self.width_m) {
            return Err("median_width_out_of_range");
        }
        if !self.start_setback_m.is_finite()
            || !self.end_setback_m.is_finite()
            || self.start_setback_m < 0.0
            || self.end_setback_m < 0.0
        {
            return Err("invalid_median_setback");
        }
        Ok(())
    }

    /// Whether vehicle access must remain on the building's own carriageway.
    pub fn is_raised(self) -> bool {
        self.style != MedianStyle::Painted
    }
}

impl Edge {
    /// Positions unequal directional lane groups inside a centreline-centred road footprint.
    pub fn median_center_offset(&self) -> f32 {
        let side = if crate::config::DRIVE_ON_LEFT {
            -1.0
        } else {
            1.0
        };
        self.pocket_surface_shift()
            + (f32::from(self.bkw_lanes) - f32::from(self.fwd_lanes))
                * crate::config::LANE_WIDTH
                * 0.5
                * side
    }
    /// Median displacement of each directional lane group; O(1), allocation-free.
    pub fn median_half_width(&self) -> f32 {
        self.median.map_or(0.0, |median| {
            (median.width_m
                - self.pocket_taper().map_or(0.0, |(_, t)| {
                    if t.left_pocket {
                        crate::config::LANE_WIDTH
                    } else {
                        0.0
                    }
                }))
                * 0.5
        })
    }

    /// The effective access policy, preserving the player's choice underneath a raised median.
    pub fn effective_vehicle_frontage_access(&self) -> VehicleFrontageAccess {
        if self.median.is_some_and(RoadMedian::is_raised) {
            VehicleFrontageAccess::SameSideOnly
        } else {
            self.vehicle_frontage_access
        }
    }

    /// Validates a cross-section without mutating topology or any derived cache.
    pub fn validate_cross_section(
        &self,
        fwd: u8,
        bkw: u8,
        median: Option<RoadMedian>,
    ) -> Result<(), &'static str> {
        if self.roundabout && (fwd != 1 || bkw != 0 || median.is_some()) {
            return Err("roundabout_requires_single_circulating_lane");
        }
        if let Some(median) = median {
            median.validate()?;
            if self.primary_type != TransitType::Road || self.allowed_types & TransitFlags::CAR == 0
            {
                return Err("median_requires_vehicle_road");
            }
            if fwd == 0 || bkw == 0 {
                return Err("median_requires_both_directions");
            }
        }
        Ok(())
    }
}

impl super::RegionGraph {
    /// Changes one road's authored cross-section, keeping the centreline and edge ID.
    /// Callers must transactionally rebuild incident profiles, lanes, access and surface caches.
    pub(crate) fn set_edge_cross_section(
        &mut self,
        id: usize,
        fwd: u8,
        bkw: u8,
        median: Option<RoadMedian>,
    ) -> Result<(), &'static str> {
        let edge = self
            .get_edge(id)
            .filter(|edge| !edge.deleted)
            .ok_or("invalid_road")?;
        edge.validate_cross_section(fwd, bkw, median)?;
        if fwd > 4 || bkw > 4 {
            return Err("too_many_lanes");
        }
        if (edge.primary_type == TransitType::Foot) != (fwd == 0 && bkw == 0) {
            return Err("cross_section_cannot_change_transit_mode");
        }
        self.remove_from_spatial_index(id);
        let edge = self.edge_mut(id);
        edge.fwd_lanes = fwd;
        edge.bkw_lanes = bkw;
        edge.median = median;
        edge.width = (f32::from(fwd) + f32::from(bkw)) * crate::config::LANE_WIDTH
            + median.map_or(0.0, |median| median.width_m);
        edge.width = edge.width.max(2.0);
        self.add_to_spatial_index(id);
        Ok(())
    }
}

impl super::RegionGraph {
    /// Tests matching directional groups, style and local setbacks in O(local degree).
    pub(crate) fn median_continues_at_node(&self, id: usize, node: u32) -> bool {
        let node = self.get_valid_node(node);
        if self.node(node).node_type == crate::simulation::network::types::NodeType::LaneTransition
        {
            return self.node_adjacency(node).len() == 2
                && self
                    .node_adjacency(node)
                    .iter()
                    .all(|&id| self.edge(id).median.is_some());
        }
        if self
            .node(node)
            .crosswalk_overrides
            .values()
            .any(|&enabled| enabled)
        {
            // A player-requested crossing opens both ends before its zebra marking.
            return false;
        }
        if self
            .node(node)
            .lane_connections
            .keys()
            .any(|&(edge, lane)| self.is_vehicle_turn_rule(edge, lane))
        {
            // Explicit junction movements retain their existing rules and need an open mouth.
            return false;
        }
        let mut incident = self
            .node_adjacency(node)
            .iter()
            .copied()
            .filter(|&id| !self.edge(id).deleted);
        let Some(a) = incident.next() else {
            return false;
        };
        let Some(b) = incident.next() else {
            return false;
        };
        if incident.next().is_some() {
            return false;
        }
        let edge = self.edge(id);
        let other = self.edge(if a == id { b } else { a });
        let outward_lanes = |edge: &Edge| {
            if self.get_valid_node(edge.start_node) == node {
                (edge.fwd_lanes, edge.bkw_lanes)
            } else {
                (edge.bkw_lanes, edge.fwd_lanes)
            }
        };
        let a = outward_lanes(edge);
        let b = outward_lanes(other);
        edge.median.zip(other.median).is_some_and(|(a, b)| {
            let own_setback = if self.get_valid_node(edge.start_node) == node {
                a.start_setback_m
            } else {
                a.end_setback_m
            };
            let other_setback = if self.get_valid_node(other.start_node) == node {
                b.start_setback_m
            } else {
                b.end_setback_m
            };
            a.style == b.style
                && a.width_m == b.width_m
                && own_setback == 0.0
                && other_setback == 0.0
        }) && edge.width == other.width
            && a == (b.1, b.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn median_validation_refuses_one_way_footpaths_and_invalid_dimensions() {
        let edge = Edge {
            primary_type: TransitType::Road,
            allowed_types: TransitFlags::CAR,
            ..Default::default()
        };
        let median = RoadMedian::from_tool(2, 5.0).unwrap();
        assert_eq!(
            edge.validate_cross_section(1, 0, median),
            Err("median_requires_both_directions")
        );
        assert_eq!(
            edge.validate_cross_section(0, 1, median),
            Err("median_requires_both_directions")
        );
        assert!(edge.validate_cross_section(1, 1, median).is_ok());
        let foot = Edge {
            primary_type: TransitType::Foot,
            allowed_types: TransitFlags::FOOT,
            ..Default::default()
        };
        assert_eq!(
            foot.validate_cross_section(0, 0, median),
            Err("median_requires_vehicle_road")
        );
        assert!(RoadMedian::from_tool(1, f32::NAN).is_err());
        assert!(RoadMedian::from_tool(3, 0.5).is_err());
    }
}
