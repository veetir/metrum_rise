// SPDX-License-Identifier: GPL-2.0-only

//! Downstream storage must hold a complete body and cannot be stolen by frontage entry.

use super::*;
use crate::config::{CAR_LENGTH, IDM_S_MIN};

#[test]
fn one_car_short_link_reserves_tail_clearance_and_excludes_external_entry() {
    let (mut network, graph, lanes) = fixture(false, false);
    let connection = connector(&network, lanes[0], 1);
    let out = network.lane_system.lanes[connection].next_lanes[0];
    // Isolate the storage contract: an empty link can fit one complete car even
    // when it has no room for an additional following gap at its far stop line.
    network.lane_system.lanes[out].length = CAR_LENGTH + 0.2;
    let mut agents = AgentSystem::new();
    let first = car(&mut agents, &network, &lanes, 0, 1);
    prepare(&mut agents, &network, &graph);
    assert!(
        agents
            .junction_admission
            .allows(first, connection, &network)
    );
    agents
        .junction_admission
        .reserve_path(first, connection, &network);
    assert!(
        !agents
            .junction_admission
            .storage_and_zones_clear(connection, &network)
    );
    assert!(!agents.junction_admission.external_entry_clear(out, 0.0));
    agents
        .agents
        .enter_lane(first, &network.lane_system, connection, 0.0, 0);
    agents.transit[first] = crate::simulation::economy::agents::TRANSIT_INTERSECTION;
    prepare(&mut agents, &network, &graph);
    assert!(
        !agents
            .junction_admission
            .storage_and_zones_clear(connection, &network)
    );
    let prefix = CAR_LENGTH + (CAR_LENGTH + IDM_S_MIN);
    assert!(
        !agents
            .junction_admission
            .external_entry_clear(out, prefix - 0.01)
    );
    assert!(
        agents
            .junction_admission
            .external_entry_clear(out, prefix + 0.01)
    );
    agents
        .junction_admission
        .release_exit(first, connection, out, &network);
    assert!(agents.junction_admission.external_entry_clear(out, 0.0));
}
