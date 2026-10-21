# SPDX-License-Identifier: GPL-2.0-only

## Roundabout bridge regression and opt-in real-scene captures. Only fresh blank worlds are used.
## METRUM_RB_CAPTURE=1 writes overhead, street and entry-sign PNGs to captures/roundabouts.
extends "res://tests/road_junction_control_test.gd"

const ANGLES := [0.0, 1.4, 3.0, 4.7]

func run() -> void:
	if OS.get_environment("METRUM_RB_CAPTURE") == "1":
		await capture_roundabouts()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		expect(simulation.create_blank_world(1024, 1024, 4, 128, 0), "Roundabout blank world")
		simulation.set_simulation_speed(0)
		await junction()
		await check_right_lane()
		await check_presets()
		simulation.queue_free()
		await process_frame
	quit(failures)

func junction() -> void:
	var angles := PackedFloat32Array(ANGLES)
	var preview: Dictionary = simulation.preview_roundabout(ground(0, 0), 50.0, angles, -1)
	expect(preview.get("is_valid", false), "Roundabout preview: %s" % preview.get("invalid_reason", ""))
	expect(not preview.get("terrain_preview", {}).get("patches", []).is_empty(), "Roundabout terrain cutouts are previewed")
	var result := await finish_request(simulation.place_roundabout(ground(0, 0), 50.0, angles, -1))
	expect(result.get("committed", false), "Roundabout transaction: %s" % result)
	if main != null:
		await settle()
	var data: Dictionary = simulation.get_road_mesh_data(true)
	var entry_signs := 0
	for chunk: Dictionary in data.get("chunks", []):
		var signs: PackedFloat32Array = chunk.get("road_signs", PackedFloat32Array())
		for index in range(4, signs.size(), 5):
			entry_signs += int(int(signs[index]) == 27)
	expect(entry_signs == 4, "Every arm exports its B5/D2 sign (%d)" % entry_signs)

func check_right_lane() -> void:
	var goals := {}
	var exited := {}
	simulation.set_camera_aabb(-400, 400, -400, 400)
	simulation.set_simulation_speed(4.0)
	# Debug spawning starts at the nearest node, not the requested point along a lane.
	# Feed spaced waves at the outer nodes instead of superimposing a paused queue.
	for wave in 6:
		for arm in 4:
			var source := Vector2(cos(ANGLES[arm]), sin(ANGLES[arm])) * 78.0
			var target := Vector2(cos(ANGLES[(arm + 1) % 4]), sin(ANGLES[(arm + 1) % 4])) * 78.0
			var id: int = simulation.spawn_debug_car(ground(source.x, source.y), ground(target.x, target.y))
			expect(id >= 0, "Roundabout route from arm %d" % arm)
			goals[id] = ground(target.x, target.y)
		await create_timer(0.8).timeout
	if main != null:
		simulation.set_simulation_speed(0.0)
		await capture("traffic_loaded_above", ground(0, 0), 100.0, -1.35)
		await capture("traffic_loaded_street", ground(40, 0), 22.0, -0.12, 1.57)
		simulation.set_simulation_speed(4.0)
	var deadline := Time.get_ticks_msec() + 60000
	while Time.get_ticks_msec() < deadline and exited.size() < goals.size():
		await process_frame
		var buckets: Dictionary = simulation.get_car_render_data()
		for bucket: Dictionary in buckets.values():
			var ids: PackedInt64Array = bucket.ids
			var poses: PackedFloat32Array = bucket.transforms
			for index in ids.size():
				if goals.has(ids[index]):
					var at := Vector3(poses[index * 12 + 3], poses[index * 12 + 7], poses[index * 12 + 11])
					if at.distance_to(goals[ids[index]]) < 42.0:
						exited[ids[index]] = true
	expect(exited.size() == goals.size(), "All 24 cars must leave the loaded ring (%d/%d)" % [exited.size(), goals.size()])
	simulation.set_simulation_speed(0.0)

func check_presets() -> void:
	for count in range(3, 7):
		expect(simulation.create_blank_world(1024, 1024, 4, 128, 0), "Preset matrix blank world")
		simulation.set_simulation_speed(0)
		var angles := PackedFloat32Array()
		for arm in count:
			angles.append(TAU * float(arm) / float(count) + 0.12)
		var preview: Dictionary = simulation.preview_roundabout(ground(0, 0), 60.0, angles, -1)
		expect(preview.get("is_valid", false), "%d-arm preview: %s" % [count, preview])
		var result := await finish_request(simulation.place_roundabout(ground(0, 0), 60.0, angles, -1))
		expect(result.get("committed", false), "%d-arm preset: %s" % [count, result])

func capture_roundabouts() -> void:
	output = ProjectSettings.globalize_path("res://../captures/roundabouts")
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
	expect(simulation.create_blank_world(1024, 1024, 4, 128, 0), "Fresh roundabout capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0.0)
	camera.set_view_state(PackedFloat32Array([0, 0, 0, -0.65, -1.1, 120, 0]))
	await junction()
	await capture("four_uneven_arms_above", ground(0, 0), 100.0, -1.35)
	await capture("central_island_street", ground(28, 0), 22.0, -0.12, 1.57)
	await capture_control_sign("entry_b5_d2", 27)
	var tool: Node3D = main.get_node("RoundaboutTool")
	tool.diameter = 60.0
	tool.arms = 5
	tool.set_scripted_pointer(true, ground(210, 0))
	tool.active = true
	camera.set_view_state(PackedFloat32Array([210, 0, 0, -0.65, -1.3, 110, 0]))
	await settle()
	var deadline := Time.get_ticks_msec() + 15000
	while not tool._valid and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(tool._valid, "Real tool installs paired five-arm road/terrain preview")
	await capture("tool_five_arm_preview_above", ground(210, 0), 110.0, -1.3)
	tool.active = false
	tool.cancel()
	await check_right_lane()
	await capture("traffic_drained_above", ground(0, 0), 100.0, -1.35)
	main.queue_free()
	await process_frame
