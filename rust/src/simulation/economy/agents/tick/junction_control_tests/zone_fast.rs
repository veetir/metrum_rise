// SPDX-License-Identifier: GPL-2.0-only

//! Maximum UI-speed steps must not jump over an unclaimed overlap interval.

use super::*;

#[test]
fn maximum_speed_steps_keep_zone_exclusion_and_drain_equal_road_cycles() {
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
            super::zone_support::drain_step(agents, network, graph, &targets, workers, 32.0 / 60.0)
        };
        assert_eq!(run(1), run(8));
    }
}
