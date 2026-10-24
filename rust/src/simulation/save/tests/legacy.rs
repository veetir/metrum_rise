// SPDX-License-Identifier: GPL-2.0-only

//! Load the populated round-trip fixture through the exact committed historical SQL layouts.
//! The JSON records source commits and deduplicates identical table definitions; it is not
//! inferred from current field names or a version-number range.

use super::*;

pub(super) fn check_histories(current: &Path, registry: &crate::assets::AssetRegistry) {
    let history: serde_json::Value =
        serde_json::from_str(include_str!("data/save_history_schemas.json")).unwrap();
    let baseline = load_from_sqlite(current, registry).unwrap();
    for entry in history["versions"].as_array().unwrap() {
        let version = entry["version"].as_i64().unwrap();
        let road = entry["history"] == "road" && version >= 88;
        let format = super::super::format::SaveFormat { version, road };
        let path = temp_path(&format!(
            "legacy_{}_v{version}",
            entry["history"].as_str().unwrap()
        ));
        let conn = Connection::open(&path).unwrap();
        conn.execute("ATTACH DATABASE ?1 AS current", [current.to_str().unwrap()])
            .unwrap();
        for (table, key) in entry["tables"].as_object().unwrap() {
            conn.execute_batch(history["tables"][key.as_str().unwrap()].as_str().unwrap())
                .unwrap();
            let columns = conn
                .prepare(&format!("PRAGMA main.table_info(\"{table}\")"))
                .unwrap()
                .query_map([], |r| r.get::<_, String>(1))
                .unwrap()
                .collect::<Result<Vec<_>, _>>()
                .unwrap();
            let current_columns = conn
                .prepare(&format!("PRAGMA current.table_info(\"{table}\")"))
                .unwrap()
                .query_map([], |r| r.get::<_, String>(1))
                .unwrap()
                .collect::<Result<Vec<_>, _>>()
                .unwrap();
            if current_columns.is_empty() {
                match table.as_str() {
                    // Already retired by the old v86 reader: lanes rebuild walking steps;
                    // outside export quotas were deliberately removed in the v71 economy.
                    "agent_ped_steps" => {
                        conn.execute_batch(
                            "INSERT INTO main.agent_ped_steps VALUES (0, 0, 0, 1, 1)",
                        )
                        .unwrap();
                        continue;
                    }
                    "owa_export_saturation" => {
                        conn.execute_batch(
                            "INSERT INTO main.owa_export_saturation VALUES (0, 123)",
                        )
                        .unwrap();
                        continue;
                    }
                    _ => panic!("historical table needs an explicit fixture conversion: {table}"),
                }
            }
            let expressions = columns.iter().map(|column| {
                if version == 57 && table == "agents" && column == "current_lane_id" {
                    return "CASE WHEN current_edge != -1 THEN COALESCE((SELECT lane_idx FROM current.saved_lanes WHERE lane_id = agents.current_lane_id), -1) ELSE current_lane_id END".into();
                }
                if !format.since(70) && matches!(table.as_str(), "building_inventories" | "shipments") && column == "amount" {
                    return "amount / CASE resource_runtime_id WHEN 2 THEN 1.25 WHEN 3 THEN 1.25 WHEN 8 THEN 2.5 WHEN 9 THEN 2.5 ELSE 1.0 END".into();
                }
                if current_columns.contains(column) { return format!("\"{column}\""); }
                match (table.as_str(), column.as_str()) {
                    ("network_edges", "current_congestion") | ("agents", "money") | ("agents", "pedestrian_side") | ("city_budget_history", "coal_bought") => "0".into(),
                    ("demand_state", _) => "0".into(),
                    ("agents", "job_lock_days") => "MAX(0, job_lock_until_day - (SELECT agent_settlement_days FROM current.scheduler_state))".into(),
                    ("buildings", "construction_remaining_hours") => "CASE WHEN construction_complete_slot = 0 THEN 0 ELSE MAX(1, (construction_complete_slot - (SELECT next_slot - 1 FROM current.scheduler_state) + 3599) / 3600) END".into(),
                    ("buildings", "shipment_cooldown_hours") => "MAX(0, (shipment_cooldown_until_slot - (SELECT next_slot - 1 FROM current.scheduler_state) + 3599) / 3600)".into(),
                    ("buildings", "shipment_cooldown_until_tick") => "(SELECT logistics_hour_ticks FROM current.scheduler_state) + MAX(0, (shipment_cooldown_until_slot - (SELECT next_slot - 1 FROM current.scheduler_state) + 3599) / 3600)".into(),
                    ("households", "shopping_agent_id") => "COALESCE((SELECT dense FROM current.entity_handle_slots WHERE kind = 2 AND slot = shopping_agent_slot AND generation = shopping_agent_generation), -1)".into(),
                    ("households", "shopping_agent_schedule_seed") => "COALESCE((SELECT schedule_seed FROM current.agents WHERE agent_id = (SELECT dense FROM current.entity_handle_slots WHERE kind = 2 AND slot = shopping_agent_slot AND generation = shopping_agent_generation)), 0)".into(),
                    ("shipments", "carrier_agent_id") => "COALESCE((SELECT dense FROM current.entity_handle_slots WHERE kind = 2 AND slot = carrier_slot AND generation = carrier_generation), -1)".into(),
                    ("shipments", "queued_hours") => "0".into(),
                    _ => panic!("historical field needs an explicit fixture conversion: {table}.{column}"),
                }
            }).collect::<Vec<_>>();
            let filter = if table == "timed_events" && !format.since(91) {
                if format.since(90) {
                    " WHERE kind < 2"
                } else {
                    " WHERE kind = 0"
                }
            } else {
                ""
            };
            conn.execute_batch(&format!(
                "INSERT INTO main.\"{table}\" SELECT {} FROM current.\"{table}\"{filter};",
                expressions
                    .join(", ")
                    .replace(
                        "3599",
                        &(crate::simulation::core::scheduler::SLOTS_PER_HOUR - 1).to_string()
                    )
                    .replace(
                        "3600",
                        &crate::simulation::core::scheduler::SLOTS_PER_HOUR.to_string()
                    )
            ))
            .unwrap();
        }
        conn.execute("UPDATE main.save_meta SET version = ?1", [version])
            .unwrap();
        if format.since(73) && !format.since(93) {
            let removed: String = conn
                .query_row(
                    "SELECT removed_buildings FROM main.electricity_day_state",
                    [],
                    |r| r.get(0),
                )
                .unwrap();
            let mut removed: serde_json::Value = serde_json::from_str(&removed).unwrap();
            removed["settled"] = false.into();
            removed["city_input_cost"] = 10.0.into();
            removed["power_input_cost"] = 5.0.into();
            removed["coal_consumed_kg"] = 1.0.into();
            conn.execute(
                "UPDATE main.electricity_day_state SET removed_buildings = ?1",
                [removed.to_string()],
            )
            .unwrap();
        }
        if format.since(86) && !format.since(93) {
            let blob: Vec<u8> = conn
                .query_row(
                    "SELECT quote_blob FROM main.workplace_route_cache",
                    [],
                    |r| r.get(0),
                )
                .unwrap();
            let mut old = b"WRC1".to_vec();
            old.extend_from_slice(&blob[4..12]);
            for entry in blob[16..].as_chunks::<24>().0 {
                old.extend_from_slice(&entry[..20]);
            }
            conn.execute(
                "UPDATE main.workplace_route_cache SET quote_blob = ?1",
                [old],
            )
            .unwrap();
        }
        if format.since(97) && !format.since(102) {
            let mut saved = conn
                .query_row(
                    "SELECT front, back, changes, deferred FROM main.census_inputs",
                    [],
                    |r| {
                        Ok(crate::simulation::economy::hiring::SavedCensusInputs {
                            front: r.get(0)?,
                            back: r.get(1)?,
                            changes: r.get(2)?,
                            deferred: r.get(3)?,
                            published: None,
                        })
                    },
                )
                .unwrap();
            let clock = &baseline.scheduler;
            crate::simulation::economy::hiring::CensusInputs::migrate_saved_rows(
                &mut saved,
                108,
                version,
                clock.logistics_hour_ticks,
                clock.next_slot - 1,
            )
            .unwrap();
            conn.execute(
                "UPDATE main.census_inputs SET front = ?1, back = ?2",
                rusqlite::params![saved.front, saved.back],
            )
            .unwrap();
        }
        drop(conn);
        let bytes = fs::read(&path).unwrap();
        let loaded = load_from_sqlite(&path, registry)
            .unwrap_or_else(|error| panic!("{} v{version}: {error}", entry["history"]));
        assert_eq!(fs::read(&path).unwrap(), bytes);
        assert_eq!(
            loaded.allocator.buildings.len(),
            baseline.allocator.buildings.len()
        );
        assert_eq!(
            loaded.households.households.len(),
            baseline.households.households.len()
        );
        assert_eq!(loaded.agents.len(), baseline.agents.len());
        assert_eq!(
            loaded.logistics.shipments.len(),
            baseline.logistics.shipments.len()
        );
        assert_eq!(loaded.graph.edge_count(), baseline.graph.edge_count());
        assert_eq!(loaded.agents.work_building, baseline.agents.work_building);
        if format.since(75) {
            assert_eq!(loaded.agents.speed, baseline.agents.speed);
        } else {
            let defaults = loaded
                .agents
                .transit_mode
                .iter()
                .map(|&mode| {
                    if mode == MODE_CAR {
                        DEFAULT_URBAN_ROAD_SPEED_MS
                    } else {
                        4.0
                    }
                })
                .collect::<Vec<_>>();
            assert_eq!(
                loaded.agents.speed, defaults,
                "{} v{version} must retain the original pre-v75 loader's mode defaults",
                entry["history"]
            );
        }
        assert_eq!(
            loaded.agents.current_lane_id,
            baseline.agents.current_lane_id
        );
        assert_eq!(
            loaded.allocator.buildings[0].daily_city_funded_input_cost,
            if format.since(73) {
                baseline.allocator.buildings[0].daily_city_funded_input_cost
            } else {
                0.0
            }
        );
        assert_eq!(loaded.scheduler.next_slot, baseline.scheduler.next_slot);
        for edge in 0..baseline.graph.edge_count() {
            let (loaded, saved) = (loaded.graph.edge(edge), baseline.graph.edge(edge));
            macro_rules! same {
                ($($field:ident),+ $(,)?) => { $(assert_eq!(loaded.$field, saved.$field, "{}", stringify!($field));)+ };
            }
            same!(
                start_node,
                end_node,
                width,
                fwd_lanes,
                bkw_lanes,
                median,
                lane_tapers,
                bus_lanes,
                roundabout,
                speed_limit,
                physical_length,
                start_clip,
                end_clip,
                geometry,
                physical_geometry,
                crossing_level,
                no_building_spawn,
                vehicle_frontage_access
            );
        }
        if format.since(73) && !format.since(93) {
            let input: f64 = baseline
                .allocator
                .buildings
                .iter()
                .map(|b| f64::from(b.daily_city_funded_input_cost))
                .sum();
            assert_eq!(loaded.allocator.city_input_day.total, 10.0 + input);
            assert!(loaded.households.day_totals.coal_consumed_kg >= 1.0);
        }
        assert_eq!(
            loaded.weather,
            if format.weather() {
                baseline.weather.clone()
            } else {
                WeatherState::default()
            }
        );
        if version == 59 {
            // The real tracked v59 fixture carries the later wage pair. Test populated rows
            // under that explicit backport as well as the committed v59 layout above.
            let conn = Connection::open(&path).unwrap();
            conn.execute_batch("ALTER TABLE households ADD gross_daily_wages REAL NOT NULL DEFAULT 12.5; ALTER TABLE households ADD employed_adults INTEGER NOT NULL DEFAULT 1;").unwrap();
            drop(conn);
            let backport = load_from_sqlite(&path, registry).unwrap();
            assert_eq!(backport.households.households[0].gross_daily_wages, 12.5);
            assert_eq!(backport.households.households[0].employed_adults, 1);
        }
        fs::remove_file(path).unwrap();
    }
}
