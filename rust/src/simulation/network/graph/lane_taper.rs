// SPDX-License-Identifier: GPL-2.0-only

//! Authored degree-two lane transitions and shared lateral profile evaluation.

use super::Edge;
use crate::config::{DRIVE_ON_LEFT, LANE_WIDTH};

/// A wider edge's transition from its neighbor's section at one stored end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoadLaneTaper {
    /// Longitudinal distance to reach the wider edge's full section.
    pub length_m: f32,
    /// Forward count at the narrow mouth, expressed in this edge's stored direction.
    pub from_fwd: u8,
    /// Backward count at the narrow mouth, expressed in this edge's stored direction.
    pub from_bkw: u8,
    /// Median width at the narrow mouth; zero for an undivided road.
    pub from_median_m: f32,
    /// The new innermost approach lane consumes the original median.
    pub left_pocket: bool,
    /// The new outermost approach lane widens only its own kerb side.
    pub right_pocket: bool,
}

/// Independent lateral boundaries in an edge's stored tangent frame.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LaneSection {
    pub(crate) left: f32,
    pub(crate) right: f32,
    pub(crate) median_left: f32,
    pub(crate) median_right: f32,
}

fn side() -> f32 {
    if DRIVE_ON_LEFT { -1.0 } else { 1.0 }
}

impl Edge {
    pub(crate) fn pocket_taper(&self) -> Option<(usize, RoadLaneTaper)> {
        self.lane_tapers
            .as_ref()?
            .iter()
            .enumerate()
            .find_map(|(end, taper)| {
                taper
                    .filter(|t| t.left_pocket || t.right_pocket)
                    .map(|t| (end, t))
            })
    }

    pub(crate) fn pocket_surface_shift(&self) -> f32 {
        self.pocket_taper().map_or(0.0, |(end, taper)| {
            if taper.right_pocket {
                LANE_WIDTH * 0.5 * side() * if end == 0 { 1.0 } else { -1.0 }
            } else {
                0.0
            }
        })
    }

    pub(crate) fn full_lane_section(&self) -> LaneSection {
        let shift = self.pocket_surface_shift();
        let center = self.median_center_offset();
        let median_half = self.median_half_width();
        LaneSection {
            left: shift - self.width * 0.5,
            right: shift + self.width * 0.5,
            median_left: center - median_half,
            median_right: center + median_half,
        }
    }

    pub(crate) fn taper_weight(&self, end: usize, s: f32) -> f32 {
        let Some(taper) = self.lane_tapers.as_ref().and_then(|ends| ends[end]) else {
            return 1.0;
        };
        let distance = if end == 0 {
            s
        } else {
            self.physical_length - s
        };
        let t = (distance / taper.length_m).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    /// Evaluates the shared authored lane/roadbed profile in constant time, without allocation.
    pub(crate) fn lane_section_at(&self, s: f32) -> LaneSection {
        let mut section = self.full_lane_section();
        if let Some(ends) = &self.lane_tapers {
            for (end, taper) in ends.iter().enumerate() {
                let Some(taper) = taper else {
                    continue;
                };
                let weight = self.taper_weight(end, s);
                if weight == 1.0 {
                    continue;
                }
                let width =
                    f32::from(taper.from_fwd + taper.from_bkw) * LANE_WIDTH + taper.from_median_m;
                let center = (f32::from(taper.from_bkw) - f32::from(taper.from_fwd))
                    * LANE_WIDTH
                    * 0.5
                    * side();
                let narrow = LaneSection {
                    left: -width * 0.5,
                    right: width * 0.5,
                    median_left: center - taper.from_median_m * 0.5,
                    median_right: center + taper.from_median_m * 0.5,
                };
                section = LaneSection {
                    left: narrow.left + (section.left - narrow.left) * weight,
                    right: narrow.right + (section.right - narrow.right) * weight,
                    median_left: narrow.median_left
                        + (section.median_left - narrow.median_left) * weight,
                    median_right: narrow.median_right
                        + (section.median_right - narrow.median_right) * weight,
                };
            }
        }
        section
    }

    pub(crate) fn transition_rank(&self, end: usize, is_fwd: bool, index: usize) -> usize {
        let Some(taper) = self.lane_tapers.as_ref().and_then(|ends| ends[end]) else {
            return index;
        };
        let count = if is_fwd {
            taper.from_fwd
        } else {
            taper.from_bkw
        };
        if count == 0 {
            return index;
        }
        if is_fwd == (end == 0) && taper.left_pocket {
            index.saturating_sub(1).min(usize::from(count - 1))
        } else if taper.left_pocket || taper.right_pocket {
            index.min(usize::from(count - 1))
        } else {
            let wide_count = if is_fwd {
                self.fwd_lanes
            } else {
                self.bkw_lanes
            };
            (index * usize::from(count) / usize::from(wide_count)).min(usize::from(count - 1))
        }
    }

    pub(crate) fn tapered_lane_offset(&self, is_fwd: bool, index: usize, s: f32, full: f32) -> f32 {
        let mut offset = full;
        if let Some(ends) = &self.lane_tapers {
            for (end, taper) in ends.iter().enumerate() {
                let Some(taper) = taper else {
                    continue;
                };
                let weight = self.taper_weight(end, s);
                if weight == 1.0 {
                    continue;
                }
                let count = if is_fwd {
                    taper.from_fwd
                } else {
                    taper.from_bkw
                };
                if count == 0 {
                    continue;
                }
                let rank = self.transition_rank(end, is_fwd, index);
                let center = (f32::from(taper.from_bkw) - f32::from(taper.from_fwd))
                    * LANE_WIDTH
                    * 0.5
                    * side();
                let narrow = center
                    + if is_fwd { side() } else { -side() }
                        * (taper.from_median_m * 0.5 + (rank as f32 + 0.5) * LANE_WIDTH);
                offset = narrow + (offset - narrow) * weight;
            }
        }
        offset
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simulation::network::graph::RoadMedian;

    #[test]
    fn pocket_keeps_opposite_kerb_and_two_through_lanes_aligned() {
        let edge = Edge {
            fwd_lanes: 4,
            bkw_lanes: 2,
            width: 22.5,
            physical_length: 150.0,
            median: RoadMedian::from_tool(2, 5.0).unwrap(),
            lane_tapers: Some(Box::new([
                Some(RoadLaneTaper {
                    length_m: 35.0,
                    from_fwd: 2,
                    from_bkw: 2,
                    from_median_m: 5.0,
                    left_pocket: true,
                    right_pocket: true,
                }),
                None,
            ])),
            ..Default::default()
        };
        let start = edge.lane_section_at(0.0);
        let end = edge.lane_section_at(35.0);
        assert_eq!((start.left, start.right), (-9.5, 9.5));
        assert_eq!((end.left, end.right), (-9.5, 13.0));
        assert_eq!((end.median_left, end.median_right), (-2.5, -1.0));
        for s in [0.0, 10.0, 35.0, 150.0] {
            assert_eq!(edge.tapered_lane_offset(true, 1, s, 4.25), 4.25);
            assert_eq!(edge.tapered_lane_offset(true, 2, s, 7.75), 7.75);
        }
    }
}
