// SPDX-License-Identifier: GPL-2.0-only

//! Mid-exit body identity survives compacted lane IDs; pre-v109 snapshots have an explicit unknown tail.

use super::*;
use crate::simulation::economy::agents::MODE_CAR;

#[test]
fn outgoing_tail_source_round_trips_through_compaction_and_v108_initialises_unknown() {
    let (mut graph, mut network) = fixture();
    let center = graph.node(2).pos;
    let p = center + Vector3::RIGHT * 100.0;
    let end = graph.add_node(p, NodeType::Junction);
    graph.add_edge(Edge {
        start_node: 2,
        end_node: end,
        geometry: vec![center, p],
        physical_geometry: vec![center, p],
        physical_length: 100.0,
        fwd_lanes: 1,
        bkw_lanes: 1,
        width: 7.0,
        speed_limit: 10.0,
        allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
        ..Default::default()
    });
    graph.rebuild_all_indices();
    network.lane_system.rebuild(&mut graph);
    network
        .lane_system
        .rebuild_edges_incremental(&mut graph, &HashSet::from([1]));
    let connection = network.lane_system.node_lanes[&2]
        .iter()
        .copied()
        .find(|&id| network.lane_system.lanes[id].lane_type == LaneType::Vehicle)
        .unwrap();
    let out = network.lane_system.lanes[connection].next_lanes[0];
    let mut agents = AgentSystem::new();
    let i = agents.spawn_housed_agent(usize::MAX, 0.0, 0.0);
    agents.transit_mode[i] = MODE_CAR;
    agents.agents.enter_lane(
        i,
        &network.lane_system,
        connection,
        network.lane_system.lanes[connection].length - 0.1,
        2,
    );
    agents
        .agents
        .enter_lane(i, &network.lane_system, out, 0.1, 2);
    assert_eq!(agents.tail_connector_lane_id[i], connection as u32);
    let allocator = BuildingAllocator::new();
    let maps = crate::simulation::save::build_snapshot_maps(&graph, &allocator, &agents).unwrap();
    let mut conn = Connection::open_in_memory().unwrap();
    conn.execute_batch(crate::simulation::save::schema::SCHEMA)
        .unwrap();
    let tx = conn.transaction().unwrap();
    crate::simulation::save::network::save_network(&tx, &graph, &maps).unwrap();
    save_agents(&tx, &agents, &graph, &network, &maps).unwrap();
    tx.commit().unwrap();
    let format = crate::simulation::save::format::SaveFormat {
        version: 109,
        road: false,
    };
    let mut restored = load_agents(&conn, 0.0, format).unwrap();
    let mut loaded_graph = crate::simulation::save::network::load_graph(&conn).unwrap();
    let mut loaded = TransitNetwork::new();
    loaded.lane_system.rebuild(&mut loaded_graph);
    restore_lane_references(&conn, &mut restored, &loaded_graph, &mut loaded).unwrap();
    let tail = restored.tail_connector_lane_id[i] as usize;
    assert_ne!(tail, connection, "fixture must exercise lane compaction");
    assert_eq!(
        loaded.lane_system.lanes[tail].geometry,
        network.lane_system.lanes[connection].geometry
    );
    assert_eq!(
        loaded.lane_system.lanes[tail].next_lanes[0],
        restored.current_lane_id[i]
    );
    conn.execute_batch("ALTER TABLE agents DROP COLUMN tail_connector_lane_id")
        .unwrap();
    assert!(
        load_agents(&conn, 0.0, format).is_err(),
        "v109 missing authoritative state must refuse"
    );
    let older = load_agents(
        &conn,
        0.0,
        crate::simulation::save::format::SaveFormat {
            version: 108,
            road: false,
        },
    )
    .unwrap();
    assert_eq!(older.tail_connector_lane_id[i], u32::MAX);
}
