// SPDX-License-Identifier: GPL-2.0-only

//! Identical-initial-world flow regression and manual diagnostics for TRAFFIC-09.

use super::*;
use crate::simulation::economy::agents::{MODE_CAR, TRANSIT_INTERSECTION, TRANSIT_NETWORK};
use std::path::Path;

fn write_lane_authority(city: &PopulatedCity, output: &Path) {
    let lanes = &city.core.transit_network.lane_system;
    let authority: Vec<_> = lanes
        .lanes
        .iter()
        .enumerate()
        .map(|(id, lane)| {
            let control = lanes.controlled_movement(id);
            serde_json::json!({
                "id": id, "edge": lane.edge_id, "node": lane.node_id,
                "type": format!("{:?}", lane.lane_type), "length": lane.length,
                "next": lane.next_lanes.iter().copied().collect::<Vec<_>>(),
                "geometry": lane.geometry.iter().map(|p| [p.x,p.y,p.z]).collect::<Vec<_>>(),
                "turn_preparation": lanes.needs_turn_preparation(id),
                "incoming": control.map(|c| c.incoming),
                "conflicts": control.map(|c| &c.conflicts),
                "yields_to": control.map(|c| &c.yields_to),
                "stop": control.is_some_and(|c| c.stop),
            })
        })
        .collect();
    std::fs::write(
        output.join("lanes.json"),
        serde_json::to_vec(&authority).unwrap(),
    )
    .unwrap();
}

#[derive(Default)]
struct Traffic {
    network: usize,
    cars: usize,
    speed: f64,
    physical: usize,
    physical_speed: f64,
    held: usize,
    intersection: usize,
}

fn traffic(city: &PopulatedCity) -> Traffic {
    let mut counts = Traffic::default();
    let a = &city.core.agents;
    for i in 0..a.transit.len() {
        if a.transit[i] == TRANSIT_NETWORK {
            counts.network += 1;
            if a.transit_mode[i] == MODE_CAR {
                counts.cars += 1;
                counts.speed += f64::from(a.speed[i]);
                if a.current_lane_id[i] < city.core.transit_network.lane_system.lanes.len() {
                    counts.physical += 1;
                    counts.physical_speed += f64::from(a.speed[i]);
                } else {
                    counts.held += 1;
                }
            }
        }
        if a.transit[i] == TRANSIT_INTERSECTION {
            counts.intersection += 1;
        }
    }
    counts
}

fn report(city: &PopulatedCity, hour: u32, mean_ms: f64) {
    let record = city.record();
    let t = traffic(city);
    eprintln!(
        "TRAFFIC09 {}",
        serde_json::json!({"hour":hour,"day":record.day,"residents":record.residents,"employed":record.employed,"buildings":record.buildings,"network":t.network,"intersection":t.intersection,"car_network":t.cars,"car_speed_ms":t.speed / t.cars.max(1) as f64,"physical_cars":t.physical,"physical_car_speed_ms":t.physical_speed/t.physical.max(1) as f64,"held_cars":t.held,"hour_mean_ms":mean_ms})
    );
}

#[test]
#[ignore = "matched-world release regression; requires METRUM_TRAFFIC09_OUTPUT"]
fn dense_city_warmup() {
    super::contract::timing_mode();
    let output = std::path::PathBuf::from(std::env::var("METRUM_TRAFFIC09_OUTPUT").unwrap());
    std::fs::create_dir_all(&output).unwrap();
    let mut city = if let Ok(source) = std::env::var("METRUM_TRAFFIC09_INPUT") {
        let mut core = SimCore {
            benchmark_mode: true,
            ..SimCore::new(WorldConfig::new(2048.0, 2048.0, 40.0, 10.0))
        };
        install_bootstrap_assets(&mut core);
        core.load_game_internal(&source).unwrap();
        PopulatedCity {
            core,
            departures: 0,
        }
    } else {
        let (mut core, row_z, half_x) = street_grid(side_m_for(10000));
        paint_districts(&mut core, &row_z, half_x);
        fill_lots(&mut core);
        core.time.speed_multiplier = SIMULATION_SPEED_STEPS[SIMULATION_SPEED_STEPS.len() - 1];
        let mut city = PopulatedCity {
            core,
            departures: 0,
        };
        city.admit_households(10000);
        city
    };
    city.core.time.speed_multiplier = SIMULATION_SPEED_STEPS[SIMULATION_SPEED_STEPS.len() - 1];
    write_lane_authority(&city, &output);
    city.core
        .save_game_internal(output.join("initial.sqlite").to_str().unwrap(), None)
        .unwrap();
    let initial = traffic(&city);
    let employed = city.record().employed;
    report(&city, 0, 0.0);
    let mut hourly_ms = 0.0;
    let mut hours = 0u32;
    for hour in 1..=48 {
        let timing = city.advance_hour();
        if timing.demand_pass {
            hourly_ms += timing.hourly.as_secs_f64() * 1000.0;
            hours += 1;
        }
        if hour % 6 == 0 {
            report(&city, hour, hourly_ms / f64::from(hours.max(1)));
        }
    }
    std::fs::write(
        output.join("junctions.json"),
        serde_json::to_vec(
            &city
                .core
                .agents
                .junction_admission
                .diagnostic(&city.core.transit_network),
        )
        .unwrap(),
    )
    .unwrap();
    city.core
        .save_game_internal(output.join("final.sqlite").to_str().unwrap(), None)
        .unwrap();
    let final_traffic = traffic(&city);
    // Initial snapshots can contain TRAFFIC-07's zero-distance border stack.
    // Count outside holds explicitly above, never relabel them as completed trips.
    // After 48 unchanged hours, physical traffic must drain below its initial load
    // and sustain at least the game's minimum free junction traversal speed.
    assert!(
        final_traffic.physical <= initial.cars,
        "physical network traffic did not drain"
    );
    assert!(
        final_traffic.physical_speed / final_traffic.physical.max(1) as f64
            >= f64::from(crate::config::CAR_JUNCTION_MIN_SPEED_MS),
        "physical network traffic remains below junction crawl speed"
    );
    assert!(
        city.record().employed >= employed,
        "traffic starvation shed the initial employed workload"
    );
}
