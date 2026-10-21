# SPDX-License-Identifier: GPL-2.0-only

## ROAD-68 automatic over/under authoring through the player's Rust tool entry points.
## Optional captures use the production Main terrain, road and lighting renderers.
extends "res://tests/road_grade_separation_test.gd"

func run() -> void:
	if OS.get_environment("METRUM_ROADS_C2_CAPTURE") == "1":
		await capture_filled_world()
		quit(failures)
		return
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	for over in [true, false]:
		for foot in [false, true]:
			expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Automatic crossing world")
			simulation.set_simulation_speed(0)
			expect(simulation.set_road_tool_crossing_level(0).is_empty(), "Ground layer")
			await build(PackedVector3Array([Vector3(-160, 0, 0), Vector3(160, 0, 0)]), 0 if over and foot else 2, 0 if over and foot else 2, 0, 0)
			var lanes := 0 if not over and foot else 1
			var raw := PackedVector3Array([Vector3(0, 0, -140), Vector3(0, 0, 140)])
			var profile: Dictionary = simulation.prepare_road_crossing(raw, lanes, lanes, over)
			expect(profile.get("is_valid", false), "Solve crossing: %s" % profile)
			if not profile.get("is_valid", false):
				continue
			expect(profile.points[0] == raw[0] and profile.points[-1] == raw[-1], "Grounded endpoints stay authored")
			expect(profile.crossing_level == (1 if over else -1), "Explicit relative level")
			simulation.set_road_tool_crossing_level(profile.crossing_level)
			var request: int = simulation.request_preview_road_surface_with_options(profile.points, lanes, lanes, true, true)
			var preview: Variant = null
			var deadline := Time.get_ticks_msec() + 60000
			while preview == null and Time.get_ticks_msec() < deadline:
				preview = simulation.get_preview_road_surface_result(request, 0, PackedInt64Array())
				await process_frame
			expect(preview is Dictionary and preview.get("is_valid", false), "Exact crossing preview: %s" % preview.get("invalid_reason", "no result"))
			if not await build(profile.points, lanes, lanes, 0, 0):
				continue
			if preview is Dictionary:
				expect(simulation.get_edge_geometry_3d(1) == preview.get("prepared_points", PackedVector3Array()), "Exact preview profile equals commit")
			expect(simulation.get_closest_node(Vector3.ZERO, 1.0) == -1, "No crossing junction")
			if not over:
				expect(simulation.get_no_building_spawn(1), "Deep cut reuses the existing frontage exclusion")
			var before := roadbed_digest()
			var path := "/tmp/roads-c2-roundtrip.sqlite"
			expect(simulation.save_game(path), "Save crossing")
			expect(simulation.undo_action(), "One undo removes the whole solved crossing")
			deadline = Time.get_ticks_msec() + 10000
			while not simulation.get_road_cross_section(1).is_empty() and Time.get_ticks_msec() < deadline:
				await process_frame
			expect(simulation.get_road_cross_section(1).is_empty(), "Placement undo removes its edge")
			expect(not simulation.get_road_cross_section(0).is_empty(), "Placement undo retains the crossed road")
			expect(simulation.load_game(path), "Reload crossing")
			expect(roadbed_digest() == before, "Structures and terrain ownership survive reload exactly")
			var concrete := 0
			for chunk: Dictionary in simulation.get_road_mesh_data(true).get("chunks", []):
				concrete += chunk.get("concrete_vertices", PackedVector3Array()).size()
			expect(concrete > 30, "Opening, retaining walls and parapets are emitted")
			var refusal: Dictionary = simulation.prepare_road_crossing(PackedVector3Array([Vector3(-10, 0, -10), Vector3(-10, 0, 10)]), 1, 1, over)
			expect(not refusal.get("is_valid", true), "Insufficient approaches are refused")
			if not foot:
				var car: int = simulation.spawn_debug_car(raw[0] + Vector3(0, 0, 10), raw[-1] - Vector3(0, 0, 10))
				expect(car >= 0, "Solved grade has a vehicle route")
				var changed := await finish_request(simulation.change_road_cross_section(0, 1, 1))
				expect(changed.get("committed", false), "Width edit with a car on the separated road: %s" % changed)
				expect(simulation.undo_action(), "Undo lower-owner width edit")
				deadline = Time.get_ticks_msec() + 10000
				while simulation.get_road_cross_section(0).get("fwd_lanes", 0) != 2 and Time.get_ticks_msec() < deadline:
					await process_frame
				expect(simulation.get_road_cross_section(0).get("fwd_lanes", 0) == 2, "Undo restores lower-owner width")
	# Relative level zero still owns an authored cut beneath a level-one road.
	expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Relative-zero cut world")
	simulation.set_simulation_speed(0)
	simulation.set_road_tool_crossing_level(1)
	await build(PackedVector3Array([Vector3(-160, 0, 0), Vector3(160, 0, 0)]), 1, 1, 0, 0)
	var zero: Dictionary = simulation.prepare_road_crossing(PackedVector3Array([Vector3(0, 0, -140), Vector3(0, 0, 140)]), 1, 1, false)
	expect(zero.get("is_valid", false) and zero.get("crossing_level", -2) == 0, "Under may return relative level zero")
	if zero.get("is_valid", false):
		simulation.set_road_tool_crossing_level(0)
		await build(zero.points, 1, 1, 0, 0)
		expect(simulation.get_no_building_spawn(1), "Relative-zero cut excludes ordinary frontage")
	simulation.queue_free()
	await process_frame
	quit(failures)

func capture_filled_world() -> void:
	await setup_capture_scene()
	var manager: Node = main.get_node("InputManager")
	expect(simulation.create_blank_world(1600, 1600, 4, 128, 0), "Production terrain capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0)
	await capture_blank_crossings()
	main.queue_free()
	await process_frame

func setup_capture_scene() -> void:
	output = ProjectSettings.globalize_path("res://../captures/roads_c2")
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
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	Engine.max_fps = 60
	main.get_node("SceneLighting").pin_hour_of_day(12.0)

func capture_blank_crossings() -> void:
	for over in [true, false]:
		var z := -250.0 if over else 250.0
		simulation.set_road_tool_crossing_level(0)
		await build(PackedVector3Array([Vector3(-180, 0, z), Vector3(180, 0, z)]), 2, 2, 0, 0)
		var lanes := 1 if over else 0
		var raw := PackedVector3Array([Vector3(0, 0, z - 160), Vector3(0, 0, z + 160)])
		var profile: Dictionary = simulation.prepare_road_crossing(raw, lanes, lanes, over)
		expect(profile.get("is_valid", false), "Capture profile")
		if not profile.get("is_valid", false):
			continue
		var name := "vehicle_fill" if over else "pedestrian_cut"
		var tool: Node = main.get_node("RoadTool")
		tool.active = true
		tool.fwd_lanes = lanes
		tool.bkw_lanes = lanes
		tool.set_median(0, 0)
		tool.set_crossing_mode(1 if over else 2)
		tool.set_road_preview_mode(1)
		tool._begin_stroke(raw[0], 0, Vector3.ZERO)
		tool.set_scripted_pointer(true, raw[-1])
		tool._queue_preview_update()
		var deadline := Time.get_ticks_msec() + 60000
		while (tool._preview_drawn_request_id == 0 or tool._preview_result_pending) and Time.get_ticks_msec() < deadline:
			await process_frame
		expect(tool._preview_cache_surface.get("is_valid", false), "Player crossing preview: " + str(tool._preview_cache_surface.get("invalid_reason", "")))
		await native_capture(name + "_preview", Vector3(0, 0, z), 145.0, -0.9, -0.65)
		expect(tool._commit_segment(raw[-1]), "Player crossing commit")
		deadline = Time.get_ticks_msec() + 60000
		while tool._commit_request_id > 0 and Time.get_ticks_msec() < deadline:
			await process_frame
		tool.cancel_road()
		tool.set_scripted_pointer(false)
		tool.active = false
		expect(simulation.get_road_cross_section(simulation.get_hovered_edge(0, z + 30)).get("crossing_level", 0) == profile.crossing_level, "Player crossing layer is committed")
		tool.update_main_mesh()
		var position := Vector3(-170, 0, z)
		var car: int = simulation.spawn_debug_car(position, Vector3(170, 0, z))
		expect(car >= 0, "Captured lower/main road accepts traffic")
		simulation.set_simulation_speed(4.0)
		var crossed := false
		deadline = Time.get_ticks_msec() + 15000
		while car >= 0 and not crossed and Time.get_ticks_msec() < deadline:
			await process_frame
			if simulation.pick_debug_agent(position, 35.0) == car:
				var details: Dictionary = simulation.get_agent_inspection(car)
				if details.get("active", false):
					position = details.position
					crossed = absf(position.x) < 20.0
		simulation.set_simulation_speed(0.0)
		expect(crossed, "Captured traffic reaches the opening on its own road")
		await native_capture(name + "_overhead", Vector3(0, 0, z), 145.0, -0.9, -0.65)
		var floor_y := 0.0
		if not over:
			for point: Vector3 in profile.points:
				floor_y = minf(floor_y, point.y)
		await native_capture(name + "_street", Vector3(-3 if over else 0, floor_y, z), 32.0, -0.08, -PI * 0.5 if over else PI)

func native_capture(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float) -> void:
	var offset := Vector3(sin(yaw) * cos(pitch), -sin(pitch), cos(yaw) * cos(pitch)) * distance
	camera.global_position = pivot + offset
	if pitch > -0.2:
		var eye := camera.global_position
		eye.y = simulation.get_world_surface_height(Vector2(eye.x, eye.z)) + 1.7
		camera.global_position = eye
		pivot.y += 1.3
	camera.look_at(pivot, Vector3.UP)
	var stable := 0
	var deadline := Time.get_ticks_msec() + 60000
	while stable < 24 and Time.get_ticks_msec() < deadline:
		await process_frame
		# Certify the crossing neighborhood; the university's 18 km world also requests
		# coarse background residency outside this capture, which is a separate concern.
		if capture_terrain_settled(pivot, maxf(distance, 180.0)) and not simulation.is_network_dirty():
			stable += 1
		else:
			stable = 0
	expect(stable >= 24, "Production terrain settled for " + label)
	if stable < 24:
		var terrain: Node = main.get_node("Terrain")
		var diagnostics := {"blocked_patches": terrain.get_blocked_dirty_patch_failures()}
		for field in ["patch_payload_requested", "pending_terrain_ack_states", "patch_lod_refresh_queue", "water_texture_sync_queue", "_terrain_residency_pending_mutations"]:
			diagnostics[field] = terrain.get(field)
		var file := FileAccess.open(output.path_join(label + "_pending.json"), FileAccess.WRITE)
		file.store_string(JSON.stringify(diagnostics))
	await RenderingServer.frame_post_draw
	expect(root.get_texture().get_image().save_png(output.path_join(label + ".png")) == OK, "Capture " + label)

func capture_terrain_settled(pivot: Vector3, radius: float) -> bool:
	var terrain: Node = main.get_node("Terrain")
	var half: Vector2 = terrain.terrain_world_size * 0.5
	var span: float = terrain.patch_span_m
	var begin := Vector2i(maxi(0, floori((pivot.x - radius + half.x) / span)), maxi(0, floori((pivot.z - radius + half.y) / span)))
	var end := Vector2i(mini(terrain.patch_cols - 1, floori((pivot.x + radius + half.x) / span)), mini(terrain.patch_rows - 1, floori((pivot.z + radius + half.y) / span)))
	var within := func(key: Vector2i) -> bool: return key.x >= begin.x and key.x <= end.x and key.y >= begin.y and key.y <= end.y
	for z in range(begin.y, end.y + 1):
		for x in range(begin.x, end.x + 1):
			var key := Vector2i(x, z)
			if not terrain.resident_patch_lookup.has(key) or terrain.patch_payload_requested.has(key) or terrain.get_patch_surface_generation(key) < 0:
				return false
	for field in ["patch_lod_refresh_queue", "water_texture_sync_queue"]:
		for key: Vector2i in terrain.get(field):
			if within.call(key):
				return false
	var ack: PackedInt64Array = terrain.pending_terrain_ack_states
	for i in range(0, ack.size(), 3):
		if within.call(Vector2i(ack[i], ack[i + 1])):
			return false
	for failure: Dictionary in terrain.get_blocked_dirty_patch_failures():
		if within.call(Vector2i(failure.patch_x, failure.patch_z)):
			return false
	return true
