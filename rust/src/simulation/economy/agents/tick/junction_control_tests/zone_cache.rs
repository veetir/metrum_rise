// SPDX-License-Identifier: GPL-2.0-only

//! Cached overlap stations follow terrain lifting and incremental ownership without remote churn.

use super::*;

#[test]
fn zone_stations_follow_heights_and_incremental_rebuild_keeps_remote_products() {
    let (_, mut graph, _) = fixture(false, false);
    let remote = graph.add_node(Vector3::new(1000.0, 0.0, 0.0), NodeType::Junction);
    for offset in [
        Vector3::LEFT,
        Vector3::RIGHT,
        Vector3::FORWARD,
        Vector3::BACK,
    ] {
        let p = graph.node(remote).pos + offset * 100.0;
        let end = graph.add_node(p, NodeType::Junction);
        graph.add_edge(Edge {
            start_node: end,
            end_node: remote,
            geometry: vec![p, graph.node(remote).pos],
            physical_geometry: vec![p, graph.node(remote).pos],
            physical_length: 100.0,
            width: 7.0,
            fwd_lanes: 1,
            bkw_lanes: 1,
            speed_limit: 30.0 / 3.6,
            allowed_types: TransitFlags::CAR | TransitFlags::FOOT,
            ..Default::default()
        });
    }
    graph.rebuild_adjacency_list();
    graph.rebuild_intersection_clips();
    let mut network = TransitNetwork::new();
    network.lane_system.rebuild(&mut graph);
    let remote_before = network.lane_system.junction_zones(remote as usize).to_vec();
    let before = network.lane_system.junction_zones(0).to_vec();
    for &id in &network.lane_system.node_lanes[&0].clone() {
        let lane = &mut network.lane_system.lanes[id];
        for p in &mut lane.geometry {
            p.y = p.x * 0.3;
        }
        lane.cum_dist = crate::simulation::network::lanes::geometry::build_cum_dist(&lane.geometry);
        lane.length = *lane.cum_dist.last().unwrap();
    }
    network.lane_system.refresh_zone_stations(0);
    assert!(
        before
            .iter()
            .zip(network.lane_system.junction_zones(0))
            .any(
                |(a, b)| a.movement.enter != b.movement.enter || a.movement.exit != b.movement.exit
            )
    );
    for zone in network.lane_system.junction_zones(0) {
        assert!(zone.movement.exit <= network.lane_system.lanes[zone.a].length);
        let opposite = network
            .lane_system
            .reservation_movement(zone.b)
            .unwrap()
            .zones
            .iter()
            .find(|z| z.id == zone.movement.id)
            .unwrap();
        assert_eq!(opposite.other_enter, zone.movement.enter);
        assert_eq!(opposite.enter, zone.movement.other_enter);
    }
    assert_eq!(
        remote_before,
        network.lane_system.junction_zones(remote as usize)
    );
    network
        .lane_system
        .rebuild_edges_incremental(&mut graph, &[0].into_iter().collect());
    assert_eq!(
        remote_before,
        network.lane_system.junction_zones(remote as usize)
    );
    assert!(
        network
            .lane_system
            .junction_zones(0)
            .iter()
            .all(|z| !before.iter().any(|old| old.movement.id == z.movement.id))
    );
}
