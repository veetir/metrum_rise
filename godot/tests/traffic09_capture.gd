# SPDX-License-Identifier: GPL-2.0-only

## Read-only TRAFFIC-09 traffic inspection through the production scene and Rust entry points.
## Writes captures only; the imported city and populated fixture are never saved over.
extends "res://tests/road_junction_geometry_capture.gd"

func run() -> void:
	preload("res://scripts/core/user_data_bootstrap.gd").run()
	await setup_capture_scene()
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	output = OS.get_environment("METRUM_TRAFFIC09_CAPTURE_OUTPUT")
	DirAccess.make_dir_recursive_absolute(output)
	var imported := OS.get_environment("METRUM_JUNCTION_SAVE")
	expect(simulation.load_game(imported), "Load read-only Kuopio copy")
	main.get_node("InputManager")._refresh_after_world_load()
	simulation.set_simulation_speed(0)
	var kys := Vector3(-73.19017, 98.29294, 922.70374)
	await native_capture("kys_before_inflow", kys, 110, -1.05, -1.94)
	var routes := [[41, 1119], [1119, 41], [152, 41], [41, 152], [1119, 152]]
	var cars: Array[int] = []
	simulation.set_simulation_speed(4)
	for wave in 6:
		for route in routes:
			var id: int = simulation.spawn_debug_car(simulation.get_node_pos(route[0]), simulation.get_node_pos(route[1]))
			expect(id >= 0, "Spawn routed KYS inflow")
			cars.append(id)
			# Space debug spawns on repeated source lanes by several physical seconds.
			await create_timer(1.0).timeout
	for i in 120:
		await process_frame
	await native_capture("kys_busy_overhead", kys, 110, -1.05, -1.94)
	await edge_street_capture("kys_busy_street", 47)
	var sampled: Array[Dictionary] = []
	var selected: int = simulation.pick_debug_agent(kys, 150.0)
	if selected >= 0:
		for sample in 2:
			var detail: Dictionary = simulation.get_agent_inspection(selected)
			if detail.get("active", false):
				sampled.append({"id": selected, "position": str(detail.get("position")), "speed_mps": detail.get("speed_mps", -1)})
			await create_timer(0.5).timeout
	FileAccess.open(output.path_join("kys_traffic.json"), FileAccess.WRITE).store_string(JSON.stringify({"spawned": cars.size(), "samples": sampled}, "\t"))
	simulation.set_simulation_speed(0)
	var fixture := OS.get_environment("METRUM_TRAFFIC09_CITY_SAVE")
	expect(simulation.load_game(fixture), "Load read-only matched 10k traffic fixture")
	main.get_node("InputManager")._refresh_after_world_load()
	_waited_for_full_load = false
	var pivot := Vector3(-200, 0, -600)
	var configured := OS.get_environment("METRUM_TRAFFIC09_CITY_PIVOT")
	if not configured.is_empty():
		var values := configured.split(",")
		pivot = Vector3(values[0].to_float(), values[1].to_float(), values[2].to_float())
	simulation.set_simulation_speed(1)
	await native_capture("city_10k_overhead", pivot, 180, -1.15, -0.6)
	var city_agent: int = simulation.pick_debug_agent(pivot, 150.0)
	var city_samples: Array[Dictionary] = []
	if city_agent >= 0:
		city_samples.append(_traffic_sample(city_agent))
	await native_capture("city_10k_street", pivot, 27, -0.12, PI)
	# A second frame group records motion, rather than only a paused layout.
	await create_timer(3.0).timeout
	await native_capture("city_10k_street_later", pivot, 27, -0.12, PI)
	if city_agent >= 0:
		city_samples.append(_traffic_sample(city_agent))
	FileAccess.open(output.path_join("city_traffic.json"), FileAccess.WRITE).store_string(JSON.stringify(city_samples, "\t"))
	main.queue_free()
	await process_frame
	quit(failures)

func _traffic_sample(id: int) -> Dictionary:
	var detail: Dictionary = simulation.get_agent_inspection(id)
	return {"id": id, "active": detail.get("active", false), "position": str(detail.get("position")), "speed_mps": detail.get("speed_mps", -1), "time_ms": Time.get_ticks_msec()}
