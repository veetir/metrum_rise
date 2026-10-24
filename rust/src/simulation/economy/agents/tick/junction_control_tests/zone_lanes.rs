// SPDX-License-Identifier: GPL-2.0-only

//! Turn-pocket and bus-marked approaches retain safe draining connector paths.

use super::*;
use crate::simulation::network::graph::RoadLaneTaper;

#[test]
fn pockets_and_bus_lane_fleets_drain_safely_and_match_workers() {
    for bus in [false, true] {
        let run = |workers| {
            let (_, mut graph, _) = fixture(false, false);
            for id in 0..4 {
                let edge = graph.edge_mut(id);
                edge.fwd_lanes = 2;
                edge.width = if bus { 14.0 } else { 10.5 };
                if bus {
                    edge.bkw_lanes = 2;
                    edge.bus_lanes = [2, 2];
                } else {
                    edge.lane_tapers = Some(Box::new([
                        None,
                        Some(RoadLaneTaper {
                            length_m: 30.0,
                            from_fwd: 1,
                            from_bkw: 1,
                            from_median_m: 0.0,
                            left_pocket: true,
                            right_pocket: false,
                        }),
                    ]));
                }
            }
            super::zone_support::routed_fleet(
                graph,
                &[
                    (0, true, 2, false),
                    (1, true, 3, false),
                    (2, true, 1, false),
                    (3, true, 0, false),
                ],
                workers,
            )
        };
        assert_eq!(run(1), run(8));
    }
}
