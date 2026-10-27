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

#[test]
#[ignore = "manual unprofiled junction demand/throughput measurement for TRAFFIC-09"]
#[allow(clippy::assertions_on_constants)]
fn queued_yield_junction_throughput() {
    use crate::simulation::economy::agents::{TRANSIT_IN_BUILDING, TRANSIT_INTERSECTION};
    assert!(!cfg!(debug_assertions), "run in release mode");
    for left in [false, true] {
        let (mut network, mut graph, lanes) = fixture(false, false);
        let mut agents = AgentSystem::new();
        let allocator = crate::simulation::buildings::allocator::BuildingAllocator::new();
        let time = crate::simulation::core::time::TimeSystem::new();
        let dt = 32.0 / 60.0;
        let mut offered = 0;
        let mut completed = 0;
        let mut entered = 0;
        let mut next_offer = 0.0;
        let mut targets = Vec::new();
        let mut last = Vec::new();
        // Fixed source: one arrival per arm every two movement seconds, with normal
        // border admission. Clear output sinks isolate junction capacity from road spillback.
        // This fixture has no buildings, demand passes, scheduling or economic side effects.
        for step in 0..2250 {
            let seconds = step as f32 * dt;
            if seconds < 120.0 && seconds >= next_offer {
                next_offer += 2.0;
                for from in 0..4 {
                    let to = if left {
                        [2, 3, 1, 0][from]
                    } else {
                        [1, 0, 3, 2][from]
                    };
                    let i = car(&mut agents, &network, &lanes, from, to);
                    assert_eq!(i, targets.len());
                    agents.current_lane_id[i] = usize::MAX;
                    agents.current_edge[i] = usize::MAX;
                    agents.lane_distance[i] = 0.0;
                    targets.push(to);
                    last.push(usize::MAX);
                    offered += 1;
                }
            }
            for (i, lane) in last.iter_mut().enumerate() {
                *lane = agents.current_lane_id[i];
            }
            agents.tick(&allocator, &mut network, &mut graph, dt, &time);
            for (i, &target) in targets.iter().enumerate() {
                if agents.transit[i] == TRANSIT_INTERSECTION && agents.current_lane_id[i] != last[i]
                {
                    entered += 1;
                }
                if agents.transit[i] == TRANSIT_NETWORK
                    && agents.current_edge[i] == target
                    && agents.lane_distance[i] > crate::config::CAR_LENGTH + 0.1
                {
                    // Absorbing boundary beyond the fully cleared junction; never remove
                    // a waiting trip to manufacture flow or free conflicting-zone storage.
                    agents.transit[i] = TRANSIT_IN_BUILDING;
                    agents.current_lane_id[i] = usize::MAX;
                    agents.tail_connector_lane_id[i] = u32::MAX;
                    agents.speed[i] = 0.0;
                    completed += 1;
                    agents.invalidate_lane_bucket_snapshot();
                }
            }
            if step == 224 {
                eprintln!(
                    "TRAFFIC09_CAPACITY {}",
                    serde_json::json!({
                        "turn":if left {"left"} else {"straight"},
                        "movement_seconds":120.0,"offered":offered,"entered":entered,
                        "completed":completed,"pending":offered-completed,
                        "offered_per_movement_hour":offered as f64*30.0,
                        "completed_per_movement_hour":completed as f64*30.0,
                        "delta":dt,
                    })
                );
            }
            if seconds >= 120.0 && completed == offered {
                eprintln!(
                    "TRAFFIC09_DRAIN {}",
                    serde_json::json!({
                        "turn":if left {"left"} else {"straight"},
                        "completed":completed,"movement_seconds":seconds+dt,
                    })
                );
                break;
            }
        }
        assert_eq!(offered, 240);
        assert_eq!(
            completed, offered,
            "finite excess demand must drain after the source stops"
        );
    }
}
