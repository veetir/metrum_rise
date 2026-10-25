// SPDX-License-Identifier: GPL-2.0-only

//! Future step leases cannot be confused with physical occupation of the whole crossing.

use super::*;

#[test]
fn a_future_zone_lease_allows_safe_approach_before_the_occupied_interval() {
    let (network, graph, lanes) = fixture(true, false);
    let mut agents = AgentSystem::new();
    let a = car(&mut agents, &network, &lanes, 0, 1);
    let b = car(&mut agents, &network, &lanes, 3, 2);
    let ac = connector(&network, lanes[0], 1);
    let bc = connector(&network, lanes[3], 2);
    let z = network
        .lane_system
        .reservation_movement(ac)
        .unwrap()
        .zones
        .iter()
        .find(|z| z.other == bc)
        .unwrap();
    assert!(z.enter > 0.6 && z.other_enter > 0.1);
    agents
        .agents
        .enter_lane(b, &network.lane_system, bc, z.other_enter - 0.1, 0);
    agents.transit[b] = crate::simulation::economy::agents::TRANSIT_INTERSECTION;
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(a, ac, &network));
    // The active connector's step lease still blocks actual movement through the interval.
    assert!(
        agents
            .junction_admission
            .connector_travel_limit(a, ac, z.enter - 0.05, &network)
            <= 0.05
    );
}
