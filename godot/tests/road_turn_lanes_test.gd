# SPDX-License-Identifier: GPL-2.0-only

## Slice B lane-choice bridge regression and opt-in windowed inspection.
## METRUM_ROADS_B_CAPTURE=1 runs the real scene and writes captures/roads_b.
## Four full-length approach lanes stand in for the still-pending pocket geometry.
extends "res://tests/road_cross_section_test.gd"

func run() -> void:
	if OS.get_environment("METRUM_ROADS_B_CAPTURE") == "1":
		await capture_world()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		expect(simulation.create_blank_world(1024.0, 1024.0, 4.0, 128.0, 0.0), "Blank turn-lane world")
		simulation.set_simulation_speed(0.0)
		await junction()
		await check_right_lane()
		simulation.queue_free()
		await process_frame
	quit(failures)

func junction() -> void:
	await build(PackedVector3Array([ground(-230, 0), ground(230, 0)]), 4, 4, 2, 5.0)
	if main != null:
		await settle()
	await build(PackedVector3Array([ground(0, -150), ground(0, 150)]), 1, 1, 0, 0.0)
	if main != null:
		await settle()

func check_right_lane() -> void:
	var start := ground(-230, 0)
	var id: int = simulation.spawn_debug_car(start, ground(0, 150))
	expect(id >= 0, "Route through the junction must exist")
	simulation.set_simulation_speed(4.0)
	# Spawning while paused precedes the first occupancy snapshot. Wait for the Rust
	# picker to see that snapshot before inspecting the car through its render ID.
	var pick_deadline := Time.get_ticks_msec() + 10000
	var picked := false
	while not picked and Time.get_ticks_msec() < pick_deadline:
		await process_frame
		picked = simulation.pick_debug_agent(start, 35.0) == id
	expect(picked, "Inspect the test car after its first occupancy snapshot")
	var deadline := Time.get_ticks_msec() + 30000
	var prepared := false
	var turned := false
	while Time.get_ticks_msec() < deadline and not turned:
		await process_frame
		var details: Dictionary = simulation.get_agent_inspection(id)
		if not details.get("active", false):
			continue
		var at: Vector3 = details.position
		# RHT: median half width + 3.5 m * (rightmost rank + half a lane).
		if at.x < -25.0 and at.z > 13.5:
			prepared = true
		if at.z > 35.0:
			turned = true
			expect(details.get("last_failure", "").is_empty(), "The prepared car must not lose its turn")
	expect(prepared, "Car must acquire the rightmost approach lane before the junction")
	expect(turned, "Car must drive the right turn onto the cross road")
	simulation.set_simulation_speed(0.0)

func capture_world() -> void:
	output = ProjectSettings.globalize_path("res://../captures/roads_b")
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
	expect(simulation.create_blank_world(1024.0, 1024.0, 4.0, 128.0, 0.0), "Fresh turn-lane capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0.0)
	camera.set_view_state(PackedFloat32Array([0.0, ground(0, 0).y, 0.0, -0.65, -1.1, 180.0, 0.0]))
	await junction()
	await check_right_lane()
	await capture("prisma_approach_above", ground(0, 0), 180.0, -1.1)
	await capture("prisma_approach_street", ground(-70, 8), 25.0, -0.1, -1.57)
	var console: Node = root.get_node_or_null("DebugConsole")
	if console != null:
		console.set_visual_mode("junction_connectors", "lanes")
	await capture("prisma_connectors_above", ground(0, 0), 150.0, -1.2)
	if console != null:
		console.set_visual_mode("junction_connectors", "off")
	# Sustain routed inflow while turn speeds and car following form queues.
	# Interleaved destinations exercise left, straight and right approach selection.
	# Capture before the tail of the inflow clears the junction.
	simulation.set_simulation_speed(1.0)
	for index in 84:
		var destination := ground(0, 150) if index % 3 == 0 else (ground(230, 0) if index % 3 == 1 else ground(0, -150))
		expect(simulation.spawn_debug_car(ground(-230, 0), destination) >= 0, "Spawn routed traffic")
		await create_timer(0.7).timeout
	simulation.set_simulation_speed(0.0)
	await capture("prisma_traffic_above", ground(-28, 0), 95.0, -1.3)
	await capture("prisma_traffic_street", ground(-65, 8), 24.0, -0.1, -1.57)
	await capture("prisma_turn_queue_street", ground(-30, 17), 35.0, -0.1, -1.57)
	main.queue_free()
	await process_frame
