// SPDX-License-Identifier: GPL-2.0-only

//! Local median continuity, setback stations and deterministic tapered noses.

use crate::simulation::network::graph::RegionGraph;

// Four metre noses are sampled explicitly, independently of terrain/section cadence.
const NOSE_LENGTH_M: f32 = 4.0;
// Clear asphalt behind the zebra's far edge.
const ZEBRA_CLEARANCE_M: f32 = 1.0;

#[derive(Clone, Copy)]
pub(super) struct MedianEnds {
    start: Option<f32>,
    end: Option<f32>,
}

impl MedianEnds {
    pub(super) fn for_edge(
        graph: &RegionGraph,
        id: usize,
        length: f32,
        mouth: Option<(f32, f32)>,
    ) -> Self {
        let edge = graph.edge(id);
        let Some(median) = edge.median else {
            return Self {
                start: None,
                end: None,
            };
        };
        let (start, end) = mouth.unwrap_or((edge.start_clip, length - edge.end_clip));
        let tip = |at_start: bool, boundary: f32, extra: f32| {
            let node = if at_start {
                edge.start_node
            } else {
                edge.end_node
            };
            if extra == 0.0 && graph.median_continues_at_node(id, node) {
                return None;
            }
            let clip = if at_start {
                edge.start_clip
            } else {
                edge.end_clip
            };
            let zebra_end = clip
                + crate::config::CROSSWALK_INSET
                + crate::config::CROSSWALK_LENGTH_M * 0.5
                + ZEBRA_CLEARANCE_M;
            Some(boundary.max(zebra_end) + extra)
        };
        Self {
            start: tip(true, start, median.start_setback_m),
            end: tip(false, length - end, median.end_setback_m).map(|distance| length - distance),
        }
    }

    pub(super) fn scale(self, s: f32) -> f32 {
        let start = self
            .start
            .map_or(1.0, |tip| ((s - tip) / NOSE_LENGTH_M).clamp(0.0, 1.0));
        let end = self
            .end
            .map_or(1.0, |tip| ((tip - s) / NOSE_LENGTH_M).clamp(0.0, 1.0));
        let t = start.min(end);
        t * t * (3.0 - 2.0 * t)
    }

    pub(super) fn add_samples(self, samples: &mut Vec<f32>, length: f32) {
        // A nose has its own one-metre station lattice. Mixing in the terrain/profile lattice
        // can split its tip into sub-centimetre kerb triangles which the surface quality
        // contract rejects, leaving holes in an otherwise complete ordered band partition.
        samples.retain(|&s| {
            !self
                .start
                .is_some_and(|tip| s > tip && s < tip + NOSE_LENGTH_M)
                && !self
                    .end
                    .is_some_and(|tip| s < tip && s > tip - NOSE_LENGTH_M)
        });
        for (tip, sign) in [(self.start, 1.0), (self.end, -1.0)] {
            if let Some(tip) = tip {
                for step in 0..=4 {
                    let s = tip + sign * step as f32;
                    if s > 0.0 && s < length {
                        samples.push(s);
                    }
                }
            }
        }
        samples.sort_by(f32::total_cmp);
        samples.dedup_by(|a, b| (*a - *b).abs() < 0.001);
    }
}
