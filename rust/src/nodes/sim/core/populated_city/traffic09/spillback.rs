// SPDX-License-Identifier: GPL-2.0-only

//! Manual frame-by-frame queue accounting; instrumentation never runs in gameplay.

use super::*;
use std::collections::BTreeMap;

#[derive(Default, serde::Serialize)]
struct LaneCounts {
    initial: usize,
    arrivals: usize,
    departures: usize,
    connector_entries: usize,
    connector_exits: usize,
    final_count: usize,
}

#[test]
#[ignore = "manual TRAFFIC-09 spillback audit; requires task-owned output path"]
fn audit_saved_spillback() {
    super::super::contract::timing_mode();
    let output = std::path::PathBuf::from(std::env::var("METRUM_TRAFFIC09_OUTPUT").unwrap());
    let hours: usize = std::env::var("METRUM_TRAFFIC09_HOURS")
        .unwrap_or_else(|_| "48".into())
        .parse()
        .unwrap();
    std::fs::create_dir_all(&output).unwrap();
    let mut city = if let Ok(input) = std::env::var("METRUM_TRAFFIC09_INPUT") {
        let mut core = SimCore {
            benchmark_mode: true,
            ..SimCore::new(WorldConfig::new(2048.0, 2048.0, 40.0, 10.0))
        };
        install_bootstrap_assets(&mut core);
        core.load_game_internal(&input).unwrap();
        PopulatedCity {
            core,
            departures: 0,
        }
    } else {
        PopulatedCity::build(10_000)
    };
    city.core.time.speed_multiplier = SIMULATION_SPEED_STEPS[SIMULATION_SPEED_STEPS.len() - 1];
    city.core
        .save_game_internal(output.join("initial.sqlite").to_str().unwrap(), None)
        .unwrap();
    write_lane_authority(&city, &output);
    let mut counts: BTreeMap<usize, LaneCounts> = BTreeMap::new();
    let initial = traffic(&city);
    let start = city.core.time.operational_time_seconds();
    for i in 0..city.core.agents.transit.len() {
        let a = &city.core.agents;
        if a.transit_mode[i] == MODE_CAR
            && matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
        {
            counts.entry(a.current_lane_id[i]).or_default().initial += 1;
        }
    }
    // Stable handles survive departures and dense-store swap-removes between frames.
    // This O(A log A) observer and its allocations are test-only, outside acceptance timings.
    let mut before = Vec::new();
    let mut before_handles = BTreeMap::new();
    let mut stalled_cases = Vec::new();
    let mut stationary = BTreeMap::<_, f64>::new();
    let mut longest = BTreeMap::<usize, f64>::new();
    let mut ready_wait = BTreeMap::new();
    let mut longest_ready_wait = BTreeMap::<usize, f64>::new();
    let mut hourly = Vec::new();
    let mut elapsed_hours = 0;
    while elapsed_hours < hours {
        before.clear();
        before_handles.clear();
        for i in 0..city.core.agents.transit.len() {
            let a = &mut city.core.agents;
            if a.transit_mode[i] == MODE_CAR
                && matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
            {
                let handle = a.agent_handle(i);
                before_handles.insert(i, handle);
                before.push((handle, a.current_lane_id[i], a.lane_distance[i]));
            }
        }
        let dt = FRAME_DT_S * f64::from(city.core.time.speed_multiplier);
        let frame = city.core.step_frame(FRAME_DT_S);
        let lanes = &city.core.transit_network.lane_system;
        for &(handle, old, old_d) in &before {
            let a = &city.core.agents;
            let new = a.resolve_agent(handle).filter(|&i| {
                a.transit_mode[i] == MODE_CAR
                    && matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
            });
            let new_lane = new.map_or(usize::MAX, |i| a.current_lane_id[i]);
            if old != new_lane || new.is_none() {
                counts.entry(old).or_default().departures += 1;
                stationary.remove(&handle);
                if let Some(lane) = lanes.lanes.get(old)
                    && lane.edge_id == usize::MAX
                    && lane.next_lanes.first().copied() == Some(new_lane)
                {
                    counts.entry(old).or_default().connector_exits += 1;
                }
            } else if let Some(i) = new {
                if a.lane_distance[i] <= old_d + 0.0001 {
                    let seconds = stationary.entry(handle).or_default();
                    *seconds += dt;
                    let maximum = longest.entry(old).or_default();
                    *maximum = maximum.max(*seconds);
                    if *seconds >= 30.0
                        && *seconds - dt < 30.0
                        && lanes.reservation_movement(old).is_some()
                    {
                        let out = lanes.lanes[old].next_lanes[0];
                        let prefix = (0..a.transit.len())
                            .filter(|&j| {
                                a.current_lane_id[j] == out
                                    && a.transit_mode[j] == MODE_CAR
                                    && matches!(
                                        a.transit[j],
                                        TRANSIT_NETWORK | TRANSIT_INTERSECTION
                                    )
                            })
                            .map(|j| a.lane_distance[j])
                            .min_by(f32::total_cmp);
                        stalled_cases.push(serde_json::json!({
                            "seconds":city.core.time.operational_time_seconds()-start,
                            "connector":old,"agent_handle":handle.to_bits(),"front":a.lane_distance[i],
                            "out_prefix":prefix,"travel_limit":a.junction_admission.connector_travel_limit(i,old,a.lane_distance[i],&city.core.transit_network),
                            "reservation":a.junction_admission.zone_diagnostic(old,&city.core.transit_network),
                            "junction":a.junction_admission.diagnostic(&city.core.transit_network),
                        }));
                    }
                } else {
                    stationary.remove(&handle);
                }
            }
        }
        // Granted heads with clear crossings and still-unbooked exit room must progress.
        let mut waiting_now = BTreeMap::new();
        for (connector, incoming, i) in city
            .core
            .agents
            .junction_admission
            .audit_ready_heads(&city.core.transit_network)
        {
            let a = &mut city.core.agents;
            if i < a.transit.len()
                && a.current_lane_id[i] == incoming
                && a.lane_distance[i] + 0.05 >= lanes.lanes[incoming].length
                && a.speed[i] <= 0.1
            {
                let handle = a.agent_handle(i);
                if before_handles.get(&i).copied() != Some(handle) {
                    continue;
                }
                let key = (handle, connector);
                let seconds = ready_wait.get(&key).copied().unwrap_or(0.0) + dt;
                waiting_now.insert(key, seconds);
                let maximum = longest_ready_wait.entry(connector).or_default();
                *maximum = maximum.max(seconds);
            }
        }
        ready_wait = waiting_now;
        // Include newly appended cars and former building/access occupants as arrivals.
        let previous: BTreeMap<_, _> = before.iter().map(|&(h, l, _)| (h, l)).collect();
        for i in 0..city.core.agents.transit.len() {
            let a = &mut city.core.agents;
            if a.transit_mode[i] != MODE_CAR
                || !matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
            {
                continue;
            }
            let handle = a.agent_handle(i);
            let new = a.current_lane_id[i];
            if previous.get(&handle).copied() != Some(new) {
                counts.entry(new).or_default().arrivals += 1;
                if city
                    .core
                    .transit_network
                    .lane_system
                    .reservation_movement(new)
                    .is_some()
                {
                    counts.entry(new).or_default().connector_entries += 1;
                }
            }
        }
        if frame.hourly_ticks > 0 {
            elapsed_hours += 1;
            let t = traffic(&city);
            hourly.push(
                serde_json::json!({"hour":elapsed_hours,"day":city.core.time.day_index,
                "physical_cars":t.physical,"held_cars":t.held,
                "physical_speed_ms":t.physical_speed/t.physical.max(1) as f64, "lanes":counts}),
            );
            if elapsed_hours % 6 == 0 {
                report(&city, elapsed_hours as u32, 0.0);
            }
        }
    }
    for i in 0..city.core.agents.transit.len() {
        let a = &city.core.agents;
        if a.transit_mode[i] == MODE_CAR
            && matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
        {
            counts.entry(a.current_lane_id[i]).or_default().final_count += 1;
        }
    }
    // An exact conservation assertion prevents a swapped/new agent being counted as a transfer.
    for row in counts.values() {
        assert_eq!(row.initial + row.arrivals, row.departures + row.final_count);
    }
    let diagnostic = city
        .core
        .agents
        .junction_admission
        .diagnostic(&city.core.transit_network);
    let t = traffic(&city);
    let lanes = &city.core.transit_network.lane_system;
    let occupancy: Vec<_> = counts
        .keys()
        .filter_map(|&id| {
            let lane = lanes.lanes.get(id)?;
            let mut cars: Vec<_> = (0..city.core.agents.transit.len())
                .filter(|&i| {
                    let a = &city.core.agents;
                    a.current_lane_id[i] == id
                        && a.transit_mode[i] == MODE_CAR
                        && matches!(a.transit[i], TRANSIT_NETWORK | TRANSIT_INTERSECTION)
                })
                .map(|i| {
                    serde_json::json!({"agent":i,"d":city.core.agents.lane_distance[i],
            "speed":city.core.agents.speed[i],"path":city.core.agents.current_path[i],
            "path_index":city.core.agents.current_path_index[i]})
                })
                .collect();
            cars.sort_by(|a, b| {
                a["d"]
                    .as_f64()
                    .unwrap()
                    .total_cmp(&b["d"].as_f64().unwrap())
            });
            Some(serde_json::json!({"lane":id,"length":lane.length,"cars":cars}))
        })
        .collect();
    std::fs::write(
        output.join("audit.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "movement_seconds":city.core.time.operational_time_seconds()-start,
            "hours":hours,"initial_physical":initial.physical,"initial_holds":initial.held,
            "final_physical":t.physical,"final_holds":t.held,"hourly":hourly,
            "lanes":counts,"longest_stationary_seconds":longest,
            "junctions":diagnostic,"occupancy":occupancy,
        "longest_clear_granted_head_wait_seconds":longest_ready_wait,
        "connector_stalls_over_30_seconds":stalled_cases,
        }))
        .unwrap(),
    )
    .unwrap();
    city.core
        .save_game_internal(output.join("final.sqlite").to_str().unwrap(), None)
        .unwrap();
}
