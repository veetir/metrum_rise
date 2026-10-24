// SPDX-License-Identifier: GPL-2.0-only

//! Manual, identical-initial-world dense traffic diagnosis for TRAFFIC-09.

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

fn report(city: &PopulatedCity, hour: u32, mean_ms: f64) {
    let record = city.record();
    let a = &city.core.agents;
    let mut network = 0usize;
    let mut cars = 0usize;
    let mut speed = 0.0f64;
    let mut intersection = 0usize;
    for i in 0..a.transit.len() {
        if a.transit[i] == TRANSIT_NETWORK {
            network += 1;
            if a.transit_mode[i] == MODE_CAR {
                cars += 1;
                speed += f64::from(a.speed[i]);
            }
        }
        if a.transit[i] == TRANSIT_INTERSECTION {
            intersection += 1;
        }
    }
    eprintln!(
        "TRAFFIC09 {}",
        serde_json::json!({"hour":hour,"day":record.day,"residents":record.residents,"employed":record.employed,"buildings":record.buildings,"network":network,"intersection":intersection,"car_network":cars,"car_speed_ms":speed / cars.max(1) as f64,"hour_mean_ms":mean_ms})
    );
}

#[test]
#[ignore = "manual matched-world traffic diagnosis; requires METRUM_TRAFFIC09_OUTPUT"]
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
    city.core
        .save_game_internal(output.join("final.sqlite").to_str().unwrap(), None)
        .unwrap();
}
