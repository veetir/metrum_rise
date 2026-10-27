// SPDX-License-Identifier: GPL-2.0-only

//! First-lane entry uses one ordered claim and a physical gap, preserving held trip state.

use super::*;
use crate::config::{CAR_LENGTH, IDM_S_MIN};

#[test]
fn border_origin_cars_wait_outside_a_full_lane_and_do_not_stack_in_one_tick() {
    let (mut network, mut graph, lanes) = fixture(false, false);
    let mut agents = AgentSystem::new();
    let ids: Vec<_> = (0..2)
        .map(|_| {
            let id = car(&mut agents, &network, &lanes, 0, 2);
            agents.current_lane_id[id] = usize::MAX;
            agents.current_edge[id] = usize::MAX;
            agents.lane_distance[id] = 0.0;
            id
        })
        .collect();
    let path = agents.current_path[ids[1]].clone();
    let seed = agents.schedule_seed[ids[1]];
    let target = agents.target_building[ids[1]];
    let allocator = crate::simulation::buildings::allocator::BuildingAllocator::new();
    let time = crate::simulation::core::time::TimeSystem::new();
    agents.tick(&allocator, &mut network, &mut graph, 0.1, &time);
    assert_eq!(
        ids.iter()
            .filter(|&&i| agents.current_lane_id[i] == lanes[0])
            .count(),
        1
    );
    assert_eq!(agents.current_lane_id[ids[1]], usize::MAX);
    assert_eq!(agents.current_path[ids[1]], path);
    assert_eq!(agents.schedule_seed[ids[1]], seed);
    assert_eq!(agents.target_building[ids[1]], target);
    agents.tick(&allocator, &mut network, &mut graph, 0.1, &time);
    assert_eq!(agents.current_lane_id[ids[1]], usize::MAX);
    assert_eq!(agents.speed[ids[1]], 0.0);
    let mut entered = false;
    for _ in 0..120 {
        agents.tick(&allocator, &mut network, &mut graph, 0.1, &time);
        entered |= agents.current_lane_id[ids[1]] == lanes[0];
        if ids.iter().all(|&i| agents.current_lane_id[i] == lanes[0]) {
            assert!(
                (agents.lane_distance[ids[0]] - agents.lane_distance[ids[1]]).abs() + 0.0001
                    >= CAR_LENGTH + IDM_S_MIN
            );
        }
    }
    assert!(
        entered,
        "the held route must resume once entry storage clears"
    );
}
