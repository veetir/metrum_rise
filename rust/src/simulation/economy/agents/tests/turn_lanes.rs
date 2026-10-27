// SPDX-License-Identifier: GPL-2.0-only

//! Advance turn-lane choice and physical missed-turn recovery through generated connectors.

use super::support::*;
use super::*;

#[test]
fn turn_lane_choice_reaches_left_straight_and_right_before_the_junction() {
    for count in 2..=4 {
        for (destination, expected_slot) in [
            (3, 0),
            (2, usize::from(count > 2)),
            (4, usize::from(count - 1)),
        ] {
            let (mut network, mut graph, arms) = build_4way_junction(count, count);
            let initial = if destination == 2 && count == 2 {
                0
            } else if expected_slot == 0 {
                usize::from(count - 1)
            } else {
                0
            };
            let mut agents = AgentSystem::new();
            let lane = arms[0][initial];
            let i = place_on_lane(&mut agents, 0, lane, 1.0, 5.0);
            agents.current_node[i] = 1;
            agents.current_path[i] = vec![1, 0, destination];
            agents.overtake_cooldown_s[i] = f32::MAX;
            let mut reached = false;
            for _ in 0..250 {
                agents.tick(
                    &BuildingAllocator::new(),
                    &mut network,
                    &mut graph,
                    0.1,
                    &test_clock(0, 0),
                );
                let lane_id = agents.current_lane_id[i];
                if lane_id == arms[0][expected_slot] {
                    reached = true;
                    assert!(
                        agents.lane_distance[i] < network.lane_system.lanes[lane_id].length - 12.0
                    );
                    break;
                }
                assert_eq!(
                    agents.current_edge[i], 0,
                    "missed preparation: count={count}, destination={destination}"
                );
            }
            assert!(reached, "count={count}, destination={destination}");
        }
    }
}

#[test]
fn missed_turn_follows_a_real_connector_then_replans_and_reaches_the_door() {
    let (mut network, mut graph, arms) = build_4way_junction(2, 2);
    let mut allocator = BuildingAllocator::new();
    allocator.buildings.push(create_test_building(2, -1));
    let destination = bkw_vehicle_lanes(&network, 2)[1];
    allocator.entrances.push(BuildingEntrance {
        edge_idx: 2,
        side: -1,
        entrance_s_m: 70.0,
        door_pos: Vector2::new(5.0, -70.0),
        curb_pos: Vector2::new(4.0, -70.0),
        car_lane_bkw: destination,
        ..BuildingEntrance::default()
    });
    let mut agents = AgentSystem::new();
    let source = arms[0][1]; // straight/right: cannot take the requested left turn.
    let end = network.lane_system.lanes[source].length;
    let i = place_on_lane(&mut agents, 0, source, end - 2.0, 5.0);
    agents.current_node[i] = 1;
    agents.current_path[i] = vec![1, 0, 3];
    agents.target_building[i] = 0;
    agents.access_flags[i] = ACCESS_PLAN_VALID;
    agents.planned_detach_node[i] = 0;
    agents.planned_detach_lane_id[i] = destination as u32;
    agents.planned_detach_lane_d[i] = 70.0;
    agents.overtake_cooldown_s[i] = f32::MAX;
    // A blocked adjacent lane ensures no last-instant lateral acquisition is possible.
    let blocker = place_on_lane(&mut agents, 0, arms[0][0], end - 2.0, 0.0);
    agents.current_node[blocker] = 1;
    agents.current_path[blocker] = vec![1, 0, 3];
    let mut previous = source;
    let mut saw_onward = false;
    let mut arrived = false;
    for tick in 0..3000 {
        agents.tick(&allocator, &mut network, &mut graph, 0.1, &test_clock(0, 0));
        let now = agents.current_lane_id[i];
        if now != previous && now != usize::MAX {
            let before = &network.lane_system.lanes[previous];
            let after = &network.lane_system.lanes[now];
            if before.edge_id != after.edge_id {
                let entered_then_changed = agents.lane_change_from_lane_id[i] != u32::MAX
                    && before
                        .next_lanes
                        .contains(&(agents.lane_change_from_lane_id[i] as usize));
                assert!(
                    before.next_lanes.contains(&now) || entered_then_changed,
                    "tick {tick}: nonphysical handoff {previous}->{now}"
                );
            }
            if after.edge_id == usize::MAX && previous == source {
                let outgoing = &network.lane_system.lanes[after.next_lanes[0]];
                assert_ne!(outgoing.edge_id, 2, "cannot make a forbidden left turn");
                saw_onward = true;
            }
        }
        if agents.transit[i] == TRANSIT_ACCESS_INGRESS || agents.transit[i] == TRANSIT_IN_BUILDING {
            arrived = true;
            break;
        }
        assert_ne!(now, usize::MAX, "tick {tick}: car lost its physical lane");
        previous = now;
    }
    assert!(saw_onward);
    assert!(
        arrived,
        "car deadlocked at lane {} distance {} with path {:?}",
        agents.current_lane_id[i], agents.lane_distance[i], agents.current_path[i]
    );
}

#[test]
fn freight_missed_turn_replans_on_its_outgoing_road_and_reaches_the_border() {
    let (mut network, mut graph, arms) = build_4way_junction(2, 2);
    let mut agents = AgentSystem::new();
    let source = arms[0][1];
    let end = network.lane_system.lanes[source].length;
    let i = place_on_lane(&mut agents, 0, source, end - 2.0, 5.0);
    agents.current_node[i] = 1;
    agents.current_path[i] = vec![1, 0, 3];
    agents.access_flags[i] = ACCESS_PLAN_VALID | ACCESS_FREIGHT_BORDER_DESTINATION;
    agents.freight_target_border_node[i] = 3;
    let mut previous = source;
    let mut arrived = false;
    for _ in 0..2000 {
        agents.tick(
            &BuildingAllocator::new(),
            &mut network,
            &mut graph,
            0.1,
            &test_clock(0, 0),
        );
        let now = agents.current_lane_id[i];
        if now == usize::MAX {
            assert_eq!(agents.current_node[i], 3);
            arrived = true;
            break;
        }
        if now != previous {
            let from = &network.lane_system.lanes[previous];
            let to = &network.lane_system.lanes[now];
            if from.edge_id != to.edge_id {
                let changing_from = agents.lane_change_from_lane_id[i] as usize;
                assert!(from.next_lanes.contains(&now) || from.next_lanes.contains(&changing_from));
            }
        }
        previous = now;
    }
    assert!(arrived);
}
