# SPDX-License-Identifier: GPL-2.0-only

## ROAD-68 topology-stage bridge regression and optional single-batch visual audit.
## METRUM_ROADS_C_CAPTURE=1 enables captures/roads_c; user saves are never opened here.
extends "res://tests/road_pockets_legacy_test.gd"

func layered_build(points: PackedVector3Array, lanes: int, level: int) -> bool:
	expect(simulation.set_road_tool_crossing_level(level).is_empty(), "Select crossing layer")
	return await build(points, lanes, lanes, 0, 0.0)

func run() -> void:
	if not OS.get_environment("METRUM_ROADS_C_LEGACY_DIR").is_empty():
		await legacy_saved_geometry()
		quit(failures)
		return
	if OS.get_environment("METRUM_ROADS_C_CAPTURE") == "1":
		await capture_crossings()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		for foot in [false, true]:
			expect(simulation.create_blank_world(512.0, 512.0, 4.0, 128.0, 0.0), "Blank layered road world")
			simulation.set_simulation_speed(0)
			await layered_build(PackedVector3Array([Vector3(-80, 0, 0), Vector3(80, 0, 0)]), 0 if foot else 1, 0)
			expect(simulation.set_road_tool_crossing_level(1).is_empty(), "Select upper layer")
			var insufficient := 3.0 if foot else 5.0
			var bad: Dictionary = simulation.validate_road_candidate_with_snap(PackedVector3Array([Vector3(0, insufficient, -80), Vector3(0, insufficient, 80)]), 1, 1, true)
			expect(not bad.get("is_valid", true) and bad.get("invalid_reason", "") == "grade_separation_clearance", "Clearance refusal: %s" % bad)
			var height := 3.8 if foot else 6.0
			var points := PackedVector3Array([Vector3(0, height, -80), Vector3(0, height, 80)])
			var request: int = simulation.request_preview_road_surface_with_options(points, 1, 1, true, false)
			var preview: Variant = null
			var deadline := Time.get_ticks_msec() + 30000
			while preview == null and Time.get_ticks_msec() < deadline:
				preview = simulation.get_preview_road_surface_result(request, 0, PackedInt64Array())
				await process_frame
			expect(preview is Dictionary and preview.get("is_valid", false), "Layered preview: %s" % preview)
			if not await layered_build(points, 1, 1):
				continue
			expect(simulation.get_closest_node(Vector3.ZERO, 1.0) == -1, "Crossing must not create a node")
			expect(simulation.get_road_cross_section(0).get("crossing_level", -1) == 0, "Lower authoritative layer")
			expect(simulation.get_road_cross_section(1).get("crossing_level", -1) == 1, "Upper authoritative layer")
			if preview is Dictionary:
				expect(simulation.get_edge_geometry_3d(1) == preview.get("prepared_points", PackedVector3Array()), "Preview profile equals commit")
			var save := "/tmp/roads-c-bridge-roundtrip.sqlite"
			expect(simulation.save_game(save), "Save layered crossing")
			expect(simulation.load_game(save), "Reload layered crossing")
			expect(simulation.get_road_cross_section(1).get("crossing_level", -1) == 1, "Layer survives save/load")
			if not foot:
				await check_crossing_cars()
		simulation.queue_free()
		await process_frame
	quit(failures)

func check_crossing_cars() -> void:
	var starts := [Vector3(-65, 0, 0), Vector3(0, 6, -65)]
	var goals := [Vector3(65, 0, 0), Vector3(0, 6, 65)]
	var ids: Array[int] = []
	for index in 2:
		ids.append(simulation.spawn_debug_car(starts[index], goals[index]))
		expect(ids[index] >= 0, "Each road must route its own car")
	var positions: Array = await drive_crossing(ids, starts)
	var edited := await finish_request(simulation.change_road_cross_section(0, 2, 2))
	expect(edited.get("committed", false), "Edit with cars on both roads: %s" % edited)
	expect(simulation.undo_action(), "Undo the cross-section edit with both cars present")
	var deadline := Time.get_ticks_msec() + 10000
	while simulation.get_road_cross_section(0).get("fwd_lanes", 0) != 1 and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(simulation.get_road_cross_section(0).get("fwd_lanes", 0) == 1, "Undo restores the lower road's lanes")
	var node: int = simulation.get_closest_node(Vector3(0, 6, -80), 1.0)
	expect(node >= 0, "Pick upper endpoint for an in-place grade edit")
	expect(simulation.move_network_node(node, Vector3(0, 8, -80)), "Raise the upper approach with cars present")
	expect(simulation.undo_action(), "Undo the grade edit with both cars present")
	deadline = Time.get_ticks_msec() + 10000
	while absf(simulation.get_node_pos(node).y - 6.0) > 0.001 and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(absf(simulation.get_node_pos(node).y - 6.0) < 0.001, "Undo restores the upper road profile")
	for index in 2:
		expect(simulation.pick_debug_agent(positions[index], 35.0) == ids[index], "Select each edited car separately")
		var details: Dictionary = simulation.get_agent_inspection(ids[index])
		expect(details.get("active", false) and details.get("lane_edge", -1) == index, "Edit/undo retains each car's own lane: %s" % details)
		if details.get("active", false):
			expect(absf(details.position.y - (0.0 if index == 0 else 6.0)) < 0.05, "Restored car height belongs to its own road")
	# Edits deliberately clear paths so real destination-bound agents can replan. Debug cars
	# have no destination building; use new routes to verify driving after the restored edit.
	ids.clear()
	for index in 2:
		ids.append(simulation.spawn_debug_car(goals[index], starts[index]))
		expect(ids[index] >= 0, "Restored roads accept fresh routes")
	await drive_crossing(ids, goals)

func drive_crossing(ids: Array[int], starts: Array) -> Array:
	simulation.set_simulation_speed(4.0)
	var positions: Array = starts.duplicate()
	var crossed := [false, false]
	var observations: Array = [{}, {}]
	var deadline := Time.get_ticks_msec() + 10000
	while not crossed.all(func(value): return value) and Time.get_ticks_msec() < deadline:
		await process_frame
		for index in 2:
			if crossed[index] or simulation.pick_debug_agent(positions[index], 35.0) != ids[index]:
				continue
			# The inspector is a single selection; select immediately before each read.
			var details: Dictionary = simulation.get_agent_inspection(ids[index])
			observations[index] = details
			if not details.get("active", false):
				continue
			var at: Vector3 = details.position
			positions[index] = at
			if absf(at.x if index == 0 else at.z) < 12.0:
				crossed[index] = true
				expect(absf(at.y - (0.0 if index == 0 else 6.0)) < 0.05, "Car stays on its own physical layer at the crossing: %s" % at)
				expect(details.get("last_failure", "").is_empty(), "Crossing route stays valid")
	expect(crossed.all(func(value): return value), "Both cars drive through their own crossing road: crossed=%s, observations=%s" % [crossed, observations])
	simulation.set_simulation_speed(0.0)
	return positions

func capture_crossings() -> void:
	# Isolate road meshes on the exact flat source surface. Full Main captures currently
	# stall on unrelated empty-water payload residency; this fixture has no water renderer.
	output = ProjectSettings.globalize_path("res://../captures/roads_c")
	DirAccess.make_dir_recursive_absolute(output)
	main = Node3D.new()
	root.add_child(main)
	current_scene = main
	simulation = ClassDB.instantiate("SimulationNode")
	simulation.name = "SimulationNode"
	main.add_child(simulation)
	var terrain := MeshInstance3D.new()
	terrain.name = "Terrain"
	var plane := PlaneMesh.new()
	plane.size = Vector2(1600, 1600)
	terrain.mesh = plane
	# Same pavement clearance as the structural earthwork stamper, below the road's y=0.
	terrain.position.y = -0.04
	terrain.material_override = preload("res://scripts/renderers/world_materials.gd").flat_terrain_material()
	main.add_child(terrain)
	var tool := NetworkTool.new()
	tool.name = "RoadTool"
	main.add_child(tool)
	tool.set_process(false)
	camera = Camera3D.new()
	camera.far = 2000.0
	main.add_child(camera)
	camera.make_current()
	var sunlight := DirectionalLight3D.new()
	sunlight.rotation_degrees = Vector3(-55, -25, 0)
	sunlight.light_energy = 1.25
	sunlight.shadow_enabled = true
	main.add_child(sunlight)
	var sky := WorldEnvironment.new()
	sky.environment = Environment.new()
	sky.environment.background_mode = Environment.BG_COLOR
	sky.environment.background_color = Color(0.55, 0.72, 0.86)
	sky.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	sky.environment.ambient_light_color = Color(0.75, 0.8, 0.9)
	sky.environment.ambient_light_energy = 0.5
	main.add_child(sky)
	root.set_disable_input(true)
	DisplayServer.window_set_size(Vector2i(1600, 1000))
	Engine.max_fps = 60
	expect(simulation.create_blank_world(1600.0, 1600.0, 4.0, 128.0, 0.0), "Fresh topology capture world")
	simulation.set_simulation_speed(0.0)
	for foot in [false, true]:
		var z := 160.0 if foot else -160.0
		await layered_build(PackedVector3Array([Vector3(-120, 0, z), Vector3(120, 0, z)]), 0 if foot else 1, 0)
		var height := 3.8 if foot else 6.0
		await layered_build(PackedVector3Array([Vector3(0, height, z - 100), Vector3(0, height, z + 100)]), 1, 1)
		expect(tool.update_main_mesh(simulation.get_network_render_generation()) >= 0, "Upload crossing road chunks")
		var label := "pedestrian" if foot else "vehicle"
		await capture(label + "_overhead", Vector3(0, 0, z), 145.0, -0.95)
		await capture(label + "_street", Vector3(-8, 0, z), 28.0, -0.1, -1.57)
	expect(simulation.set_road_tool_crossing_level(0).is_empty(), "Restore junction layer")
	await build(PackedVector3Array([Vector3(-300, 0, 420), Vector3(300, 0, 420)]), 2, 2, 2, 5.0)
	await build(PackedVector3Array([Vector3(0, 0, 300), Vector3(0, 0, 650)]), 2, 2, 0, 0.0)
	var edge: int = simulation.get_hovered_edge(-60.0, 420.0)
	var result := await finish_request(simulation.change_road_pocket(edge, false, false, true, 80.0, 35.0))
	expect(result.get("committed", false), "Build right pocket capture: %s" % result)
	expect(tool.update_main_mesh(simulation.get_network_render_generation()) >= 0, "Upload right pocket chunks")
	await capture("pocket_overhead", Vector3(-85, 0, 420), 120.0, -1.1, -1.57)
	await capture("pocket_street", Vector3(-70, 0, 425), 35.0, -0.1, -1.57)
	main.queue_free()
	await process_frame

func capture(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float = -0.65) -> void:
	var offset := Vector3(sin(yaw) * cos(pitch), -sin(pitch), cos(yaw) * cos(pitch)) * distance
	camera.global_position = pivot + offset
	if pitch > -0.2:
		camera.global_position.y = 1.7
		pivot.y = 1.3
	camera.look_at(pivot, Vector3.UP)
	for frame in 24:
		await process_frame
	await RenderingServer.frame_post_draw
	expect(root.get_texture().get_image().save_png(output.path_join(label + ".png")) == OK, "Save capture " + label)

func legacy_saved_geometry() -> void:
	var directory := OS.get_environment("METRUM_ROADS_C_LEGACY_DIR")
	var record := OS.get_environment("METRUM_ROADS_C_LEGACY_RECORD") == "1"
	DirAccess.make_dir_recursive_absolute(directory)
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	var expected: Array = []
	if not record:
		expected = JSON.parse_string(FileAccess.get_file_as_string(directory.path_join("legacy-hashes.json")))
	var hashes: Array[String] = []
	for index in 6:
		var path := directory.path_join("roads-c-v90-%d.sqlite" % index)
		if record:
			expect(simulation.create_blank_world(512.0, 512.0, 4.0, 128.0, 0.0), "Legacy blank world")
			simulation.set_simulation_speed(0.0)
			if index < 4:
				var style := 2 if index >= 2 else 0
				await build(PackedVector3Array([Vector3(-160, 0, 0), Vector3(160, 0, 0)]), 2, 2, style, 5.0 if style == 2 else 0.0)
				var skew := index % 2 == 1
				await build(PackedVector3Array([Vector3(-30 if skew else 0, 0, -120), Vector3(30 if skew else 0, 0, 120)]), 1, 1, 0, 0.0)
			else:
				await build(PackedVector3Array([Vector3(-100, 6, 0), Vector3(100, 6, 0)]), 1, 1, 2 if index == 5 else 0, 5.0 if index == 5 else 0.0)
			hashes.append(roadbed_digest())
			expect(simulation.save_game(path), "Record version-90 fixture")
		expect(simulation.load_game(path), "Read copied version-90 fixture")
		expect(roadbed_digest() == (hashes[index] if record else expected[index]), "Version-90 fixture %d must retain its exact roadbed" % index)
	if record:
		var file := FileAccess.open(directory.path_join("legacy-hashes.json"), FileAccess.WRITE)
		file.store_string(JSON.stringify(hashes))
	simulation.queue_free()
	await process_frame
