# SPDX-License-Identifier: GPL-2.0-only

## Automatic junction-control bridge regression and windowed stage-one captures.
## METRUM_JC_CAPTURE=1 writes only captures/junction_control; no player saves are loaded.
extends "res://tests/road_turn_lanes_test.gd"

func run() -> void:
	if OS.get_environment("METRUM_JC_CAPTURE") == "1":
		await capture_stage()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		expect(simulation.create_blank_world(1024.0, 1024.0, 4.0, 128.0, 0.0), "Junction-control blank world")
		simulation.set_simulation_speed(0.0)
		await junction()
		await check_right_lane()
		var data: Dictionary = simulation.get_road_mesh_data(true)
		var priority := 0
		var yield_signs := 0
		for chunk: Dictionary in data.get("chunks", []):
			var signs: PackedFloat32Array = chunk.get("road_signs", PackedFloat32Array())
			for index in range(4, signs.size(), 5):
				priority += int(int(signs[index]) == 24)
				yield_signs += int(int(signs[index]) == 25)
		expect(priority >= 2 and yield_signs >= 2, "Priority and yield poses cross the renderer bridge")
		simulation.queue_free()
		await process_frame
	quit(failures)

func junction() -> void:
	await build(PackedVector3Array([ground(-230, 0), ground(230, 0)]), 2, 2, 2, 5.0)
	if main != null:
		await settle()
	await build(PackedVector3Array([ground(0, -150), ground(0, 150)]), 1, 1, 0, 0.0)
	var edge: int = simulation.get_hovered_edge(-60.0, 0.0)
	var result := await finish_request(simulation.change_road_pocket(edge, false, true, true, 80.0, 35.0))
	expect(result.get("committed", false), "Pocket approach for automatic priority")
	if main != null:
		await settle()

func check_right_lane() -> void:
	var id: int = simulation.spawn_debug_car(ground(-230, 0), ground(0, 150))
	expect(id >= 0, "Controlled pocket route exists")
	simulation.set_simulation_speed(4.0)
	var pick_deadline := Time.get_ticks_msec() + 10000
	var picked := false
	while not picked and Time.get_ticks_msec() < pick_deadline:
		await process_frame
		picked = simulation.pick_debug_agent(ground(-230, 0), 35.0) == id
	expect(picked, "Inspect the test car after its first occupancy snapshot")
	var deadline := Time.get_ticks_msec() + 45000
	var turned := false
	while not turned and Time.get_ticks_msec() < deadline:
		await process_frame
		var details: Dictionary = simulation.get_agent_inspection(id)
		if details.get("active", false) and (details.position as Vector3).z > 35.0:
			turned = true
	expect(turned, "Car must clear controlled junction from its pocket")
	simulation.set_simulation_speed(0.0)

func capture_control_sign(label: String, kind: int) -> void:
	var data: Dictionary = simulation.get_road_mesh_data(true)
	var span: float = data.chunk_span_m
	for chunk: Dictionary in data.get("chunks", []):
		var origin := Vector3(data.chunk_origin_x_m + chunk.chunk_x * span, 0.0, data.chunk_origin_z_m + chunk.chunk_z * span)
		var signs: PackedFloat32Array = chunk.get("road_signs", PackedFloat32Array())
		for index in range(0, signs.size(), 5):
			if int(signs[index + 4]) == kind:
				var pole := origin + Vector3(signs[index], signs[index + 1], signs[index + 2])
				# Printed +Z faces the camera; keep this inspection tied to Rust's actual pose.
				await capture(label, pole, 8.0, -0.1, signs[index + 3])
				return
	expect(false, "Missing control sign for capture " + label)

func capture_stage() -> void:
	output = ProjectSettings.globalize_path("res://../captures/junction_control")
	DirAccess.make_dir_recursive_absolute(output)
	main = load("res://scenes/Main.tscn").instantiate()
	main.set_script(null)
	root.add_child(main)
	current_scene = main
	await process_frame
	main.get_node("MainUI").visible = false
	var manager: Node = main.get_node("InputManager")
	manager.set_process(false)
	manager.set_process_input(false)
	manager.set_process_unhandled_input(false)
	simulation = main.get_node("SimulationNode")
	camera = main.get_node("CameraNode")
	camera.set_process(false)
	camera.set_process_input(false)
	camera.set_process_unhandled_input(false)
	root.set_disable_input(true)
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	Engine.max_fps = 60
	main.get_node("SceneLighting").pin_hour_of_day(12.0)
	expect(simulation.create_blank_world(1024.0, 1024.0, 4.0, 128.0, 0.0), "Fresh priority capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0.0)
	camera.set_view_state(PackedFloat32Array([0, ground(0, 0).y, 0, -0.65, -1.1, 180, 0]))
	await junction()
	await check_right_lane()
	await capture("automatic_priority_above", ground(0, 0), 150.0, -1.15)
	await capture_control_sign("priority_pole_street", 24)
	await capture_control_sign("yield_pole_street", 25)
	simulation.set_simulation_speed(1.0)
	for index in 48:
		var source := ground(-230, 0) if index % 2 == 0 else ground(0, 150)
		var target := ground(230, 0) if index % 2 == 0 else ground(0, -150)
		expect(simulation.spawn_debug_car(source, target) >= 0, "Spawn conflicting inflow")
		await create_timer(0.6).timeout
	simulation.set_simulation_speed(0.0)
	await capture("priority_traffic_above", ground(0, 0), 125.0, -1.2)
	await capture("priority_traffic_street", ground(0, 40), 24.0, -0.1, 0.0)
	main.queue_free()
	await process_frame
