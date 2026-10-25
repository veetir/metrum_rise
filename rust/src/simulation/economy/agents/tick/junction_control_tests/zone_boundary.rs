// SPDX-License-Identifier: GPL-2.0-only

//! A staged car touching a zone boundary cannot steal the physical occupant's lease.

use super::*;
use crate::config::{CAR_LENGTH, IDM_S_MIN};
use crate::simulation::economy::agents::TRANSIT_INTERSECTION;

#[test]
fn a_waiter_at_the_exact_zone_entrance_cannot_overwrite_an_occupied_zone() {
    let (network, graph, lanes) = fixture(true, false);
    let (a, z) = network
        .lane_system
        .lanes
        .iter()
        .enumerate()
        .find_map(|(a, _)| {
            network
                .lane_system
                .reservation_movement(a)?
                .zones
                .iter()
                .find(|z| {
                    a < z.other
                        && z.enter == 0.0
                        && z.other_enter == 0.0
                        && z.other_exit > CAR_LENGTH + IDM_S_MIN + 0.2
                })
                .copied()
                .map(|z| (a, z))
        })
        .unwrap();
    let source = |c| {
        network.lane_system.lanes[network
            .lane_system
            .reservation_movement(c)
            .unwrap()
            .incoming]
            .edge_id
    };
    let target =
        |c: usize| network.lane_system.lanes[network.lane_system.lanes[c].next_lanes[0]].edge_id;
    let mut agents = AgentSystem::new();
    let waiting = car(&mut agents, &network, &lanes, source(a), target(a));
    let passing = car(
        &mut agents,
        &network,
        &lanes,
        source(z.other),
        target(z.other),
    );
    for (agent, connector, distance) in [(waiting, a, 0.0), (passing, z.other, z.other_exit - 0.1)]
    {
        agents
            .agents
            .enter_lane(agent, &network.lane_system, connector, distance, 0);
        agents.transit[agent] = TRANSIT_INTERSECTION;
        agents.current_path_index[agent] = 2;
    }
    prepare(&mut agents, &network, &graph);
    assert_eq!(
        agents
            .junction_admission
            .connector_travel_limit(waiting, a, 0.0, &network),
        0.0
    );
    let targets = [target(a), target(z.other)];
    super::zone_support::drain(agents, network, graph, &targets, 8);
}
