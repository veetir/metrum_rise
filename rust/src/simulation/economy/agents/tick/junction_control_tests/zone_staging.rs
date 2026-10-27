// SPDX-License-Identifier: GPL-2.0-only

//! A priority car can stage before a busy interval without occupying it or blocking its exit.

use super::*;
use crate::simulation::economy::agents::TRANSIT_INTERSECTION;

#[test]
fn staged_entry_waits_before_a_busy_zone_then_drains_without_overlap() {
    let run = |workers| {
        let (mut network, mut graph, lanes) = fixture(true, false);
        let mut agents = AgentSystem::new();
        let a = car(&mut agents, &network, &lanes, 0, 1);
        let b = car(&mut agents, &network, &lanes, 3, 2);
        let ac = connector(&network, lanes[0], 1);
        let bc = connector(&network, lanes[3], 2);
        let z = *network
            .lane_system
            .reservation_movement(ac)
            .unwrap()
            .zones
            .iter()
            .find(|z| z.other == bc)
            .unwrap();
        agents
            .agents
            .enter_lane(b, &network.lane_system, bc, z.other_enter + 0.2, 0);
        agents.transit[b] = TRANSIT_INTERSECTION;
        agents.current_path_index[b] = 2;
        prepare(&mut agents, &network, &graph);
        assert!(
            !agents.junction_admission.allows(a, ac, &network),
            "the occupied crossing cannot be traversed"
        );
        assert!(
            agents.junction_admission.entry_allows(a, ac, &network),
            "its clear prefix and booked exit can be used"
        );
        agents.tick(
            &crate::simulation::buildings::allocator::BuildingAllocator::new(),
            &mut network,
            &mut graph,
            0.05,
            &crate::simulation::core::time::TimeSystem::new(),
        );
        assert_eq!(agents.current_lane_id[a], ac);
        assert!(agents.lane_distance[a] < z.enter);
        assert!(agents.lane_distance[b] - crate::config::CAR_LENGTH < z.other_exit);
        super::zone_support::drain(agents, network, graph, &[1, 2], workers)
    };
    assert_eq!(run(1), run(8));
}
