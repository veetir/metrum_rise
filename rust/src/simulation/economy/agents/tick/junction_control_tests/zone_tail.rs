// SPDX-License-Identifier: GPL-2.0-only

//! A passed overlap releases before the rest of its connector, including body length.

use super::*;
use crate::config::CAR_LENGTH;

#[test]
fn a_passed_zone_releases_after_the_tail_before_the_connector_ends() {
    let (network, graph, lanes) = fixture(true, false);
    let (a, z) = network
        .lane_system
        .lanes
        .iter()
        .enumerate()
        .find_map(|(a, lane)| {
            network
                .lane_system
                .controlled_movement(a)?
                .zones
                .iter()
                .find(|z| {
                    z.exit + CAR_LENGTH + 0.2 < lane.length
                        && network
                            .lane_system
                            .controlled_movement(a)
                            .unwrap()
                            .zones
                            .iter()
                            .filter(|other| other.other == z.other)
                            .count()
                            == 1
                })
                .copied()
                .map(|z| (a, z))
        })
        .expect("fixture has a local fork overlap shorter than a traversal");
    let b = z.other;
    let source = |id| {
        network.lane_system.lanes[network
            .lane_system
            .controlled_movement(id)
            .unwrap()
            .incoming]
            .edge_id
    };
    let target =
        |id: usize| network.lane_system.lanes[network.lane_system.lanes[id].next_lanes[0]].edge_id;
    let mut agents = AgentSystem::new();
    let first = car(&mut agents, &network, &lanes, source(a), target(a));
    agents
        .agents
        .enter_lane(first, &network.lane_system, a, z.exit + CAR_LENGTH - 0.1, 0);
    agents.transit[first] = crate::simulation::economy::agents::TRANSIT_INTERSECTION;
    agents.speed[first] = 0.0;
    let waiting = car(&mut agents, &network, &lanes, source(b), target(b));
    prepare(&mut agents, &network, &graph);
    assert!(!agents.junction_admission.allows(waiting, b, &network));
    agents.lane_distance[first] = z.exit + CAR_LENGTH + 0.1;
    prepare(&mut agents, &network, &graph);
    assert!(agents.junction_admission.allows(waiting, b, &network));
    assert_eq!(agents.current_lane_id[first], a);
}
