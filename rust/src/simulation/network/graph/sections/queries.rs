// SPDX-License-Identifier: GPL-2.0-only

//! Named lateral queries; bounded authoring evaluation, never an agent-tick dependency.

use super::{RegionGraph, RoadSideExtents, StripRole};
use crate::simulation::network::types::{TransitFlags, TransitType};

impl RegionGraph {
    /// Motor envelope at an authored station, signed in stored start-to-end orientation.
    /// Existing two-end taper evaluation is O(1); outer strips never move the road axis.
    pub fn carriageway_bounds(&self, id: usize, station: f32) -> [f32; 2] {
        let edge = self.edge(id);
        if self.road_layout(id).path_width.get() > 0.0 {
            return [0.0; 2];
        }
        if edge.lane_tapers.is_some() {
            let section = edge.lane_section_at(station);
            [section.left, section.right]
        } else {
            let half = self.road_layout(id).core_width.get().max(3.5) * 0.5;
            [-half, half]
        }
    }

    /// Signed motor-lane axis, rank zero innermost; absent ranks return None.
    /// At most four prefix widths are visited, with no heap allocation.
    pub fn lane_center(&self, id: usize, forward: bool, rank: u8, station: f32) -> Option<f32> {
        let layout = self.road_layout(id);
        let direction = usize::from(!forward);
        if rank >= layout.lane_counts[direction] {
            return None;
        }
        let edge = self.edge(id);
        let sign = if forward != crate::config::DRIVE_ON_LEFT {
            1.0
        } else {
            -1.0
        };
        let prefix: f32 = layout.lane_widths[direction][..usize::from(rank)]
            .iter()
            .map(|w| w.get())
            .sum();
        let center = edge.median_center_offset()
            + (edge.median_half_width()
                + prefix
                + layout.lane_widths[direction][usize::from(rank)].get() * 0.5)
                * sign;
        let section = edge.lane_section_at(station);
        let full = edge.full_lane_section();
        Some(if edge.lane_tapers.is_some() {
            if sign > 0.0 {
                section.median_right + (center - full.median_right)
            } else {
                section.median_left + (center - full.median_left)
            }
        } else {
            center
        })
    }

    /// Explicit walking axis on a side (0 negative, 1 positive); independent paths use zero.
    pub fn walk_axis(&self, id: usize, side: usize, station: f32) -> Option<f32> {
        if side > 1 {
            return None;
        }
        let edge = self.edge(id);
        if edge.allowed_types & TransitFlags::FOOT == 0 {
            return None;
        }
        if edge.primary_type == TransitType::Foot {
            return Some(0.0);
        }
        let layout = self.road_layout(id);
        if layout.walk_axes[side].get() == 0.0 {
            return None;
        }
        let axis = layout.walk_axes[side].get();
        let bounds = self.carriageway_bounds(id, station);
        let full = edge.full_lane_section();
        Some(if edge.lane_tapers.is_some() {
            axis + bounds[side] - if side == 0 { full.left } else { full.right }
        } else {
            axis
        })
    }

    /// Outside paved boundaries, excluding nonpaved verge and ditch strips.
    pub fn paved_bounds(&self, id: usize, station: f32) -> [f32; 2] {
        self.bounds_with_strips(id, station, false)
    }

    /// Full road-owned plan envelope, including support shoulders and drainage strips.
    pub fn owned_ground_bounds(&self, id: usize, station: f32) -> [f32; 2] {
        self.bounds_with_strips(id, station, true)
    }

    /// Retained frontage origin; old zoning uses the 1.5 m assembly even without FOOT.
    pub fn frontage_boundary(&self, id: usize, side: usize, station: f32) -> Option<f32> {
        if side > 1 {
            return None;
        }
        let edge = self.edge(id);
        let sign = if side == 0 { -1.0 } else { 1.0 };
        let bounds = self.carriageway_bounds(id, station);
        Some(if self.road_layout(id).path_width.get() > 0.0 {
            sign * (edge.width * 0.5 + 1.5)
        } else {
            bounds[side] + sign * 1.5
        })
    }

    /// Physical corridor used by clearance checks; furniture policy remains strip-owned.
    pub fn clearance_envelope(&self, id: usize, station: f32) -> [f32; 2] {
        self.owned_ground_bounds(id, station)
    }

    fn bounds_with_strips(&self, id: usize, station: f32, all: bool) -> [f32; 2] {
        let cached = self.road_side_extents(id);
        let mut bounds = if all { cached.owned } else { cached.paved };
        let edge = self.edge(id);
        if edge.lane_tapers.is_some() && self.road_layout(id).path_width.get() == 0.0 {
            let full = edge.full_lane_section();
            let section = edge.lane_section_at(station);
            bounds[0] += section.left - full.left;
            bounds[1] += section.right - full.right;
        }
        bounds
    }

    pub(in crate::simulation::network) fn section_extents(&self, id: usize) -> RoadSideExtents {
        let layout = self.road_layout(id);
        if layout.path_width.get() > 0.0 {
            let half = layout.path_width.get() * 0.5;
            return RoadSideExtents {
                paved: [-half, half],
                owned: [-half, half],
            };
        }
        let edge = self.edge(id);
        let half = layout.core_width.get().max(3.5) * 0.5;
        let core = if edge.lane_tapers.is_some() {
            let full = edge.full_lane_section();
            [full.left, full.right]
        } else {
            [-half, half]
        };
        let mut extents = RoadSideExtents {
            paved: core,
            owned: core,
        };
        for (side, &core_boundary) in core.iter().enumerate() {
            let sign = if side == 0 { -1.0 } else { 1.0 };
            // Add outward in compiler order; summing widths first changes raw f32 bits.
            let mut boundary = core_boundary * sign;
            for strip in &layout.strips[side][..usize::from(layout.strip_counts[side])] {
                boundary += strip.width.get();
                if !matches!(strip.role, StripRole::Verge | StripRole::Ditch) && strip.surface != 3
                {
                    extents.paved[side] = boundary * sign;
                }
            }
            extents.owned[side] = boundary * sign;
        }
        extents
    }
}
