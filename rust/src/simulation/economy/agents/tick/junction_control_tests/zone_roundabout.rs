// SPDX-License-Identifier: GPL-2.0-only

//! Saturated finite roundabout entries must drain with zone exclusion.

use super::*;
use crate::simulation::network::roundabouts::{RoundaboutLayout, RoundaboutRequest};

#[test]
fn roundabout_entries_drain_safely_and_match_worker_counts() {
    let run = |workers| {
        let mut graph = RegionGraph::new();
        RoundaboutLayout::prepare(
            &graph,
            RoundaboutRequest {
                center: Vector3::ZERO,
                diameter: 50.0,
                angles: (0..4)
                    .map(|i| i as f32 * std::f32::consts::FRAC_PI_2)
                    .collect(),
                replace: None,
            },
        )
        .unwrap()
        .apply(&mut graph);
        super::zone_support::routed_fleet(
            graph,
            &[
                (0, false, 2, true),
                (1, false, 3, true),
                (2, false, 0, true),
                (3, false, 1, true),
            ],
            workers,
        )
    };
    assert_eq!(run(1), run(8));
}
