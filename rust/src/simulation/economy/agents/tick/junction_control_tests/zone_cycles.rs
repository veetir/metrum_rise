// SPDX-License-Identifier: GPL-2.0-only

//! Zone exclusion, all-yield cycles and opposing left turns under one/eight workers.

use super::*;

#[test]
fn all_yield_and_opposing_left_fleets_are_safe_drain_and_match_workers() {
    for left in [false, true] {
        let run = |workers| {
            let (network, graph, lanes) = fixture(false, false);
            let mut agents = AgentSystem::new();
            let mut targets = Vec::new();
            for wave in 0..6 {
                for from in 0..4 {
                    let to = if left {
                        [2, 3, 1, 0][from]
                    } else {
                        [1, 0, 3, 2][from]
                    };
                    let i = car(&mut agents, &network, &lanes, from, to);
                    agents.lane_distance[i] -=
                        wave as f32 * (crate::config::CAR_LENGTH + crate::config::IDM_S_MIN + 0.5);
                    targets.push(to);
                }
            }
            super::zone_support::drain(agents, network, graph, &targets, workers)
        };
        assert_eq!(run(1), run(8));
    }
}
