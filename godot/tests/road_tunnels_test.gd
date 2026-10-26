# SPDX-License-Identifier: GPL-2.0-only

## ROAD-61 authoring, quick/exact parity, save/undo and traffic through a closed terrain hill.
## METRUM_ROAD61_CAPTURE=1 uses Main's production renderers and writes captures/road61.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	var capturing := OS.get_environment("METRUM_ROAD61_CAPTURE") == "1"
	if capturing:
		await setup_capture_scene()
		output = ProjectSettings.globalize_path("res://../captures/road61")
		DirAccess.make_dir_recursive_absolute(output)
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
	await diagnosed_failures()
	if OS.get_environment("METRUM_ROAD61_DIAGNOSE_ONLY") != "1":
		await hill_tunnel(capturing)
	if main != null:
		main.queue_free()
	else:
		simulation.queue_free()
	await process_frame
	quit(failures)

func exact_preview(points: PackedVector3Array, snap: bool = false) -> Dictionary:
	var request: int = simulation.request_preview_road_surface_with_options(points, 1, 1, snap, true)
	var deadline := Time.get_ticks_msec() + 60000
	while Time.get_ticks_msec() < deadline:
		var result: Variant = simulation.get_preview_road_surface_result(request, 0, PackedInt64Array())
		if result is Dictionary and not result.get("is_pending", false):
			return result
		await process_frame
	expect(false, "Exact tunnel preview remained pending")
	return {}

func quick_verdict(points: PackedVector3Array) -> Dictionary:
	var deadline := Time.get_ticks_msec() + 60000
	while Time.get_ticks_msec() < deadline:
		var result: Dictionary = simulation.validate_road_candidate_with_snap(points, 1, 1, false)
		if not result.get("is_pending", false):
			return result
		await process_frame
	expect(false, "Synchronous candidate remained pending")
	return {}

func diagnosed_failures() -> void:
	for grounded in [true, false]:
		expect(simulation.create_blank_world(512, 512, 4, 128, 0), "Diagnostic blank world")
		simulation.set_simulation_speed(0)
		simulation.set_road_tool_crossing_level(0)
		# This comparison mode also runs against the preserved pre-ROAD-61 library.
		if simulation.has_method("set_road_tool_tunnel"):
			simulation.set_road_tool_tunnel(grounded)
		var points := PackedVector3Array([Vector3(-100, 0, 0), Vector3(-50, -6, 0), Vector3(50, -6, 0), Vector3(100, 0, 0)]) if grounded else PackedVector3Array([Vector3(-50, -6, 0), Vector3(50, -6, 0)])
		var quick: Dictionary = await quick_verdict(points)
		var exact := await exact_preview(points)
		expect(quick.get("is_valid", false) and exact.get("is_valid", false), "Quick/exact tunnel acceptance: %s / %s" % [quick.get("invalid_reason"), exact.get("invalid_reason")])
		var result := await finish_request(simulation.add_road_with_snap(points, 1, 1, false))
		expect(result.get("committed", false), "Diagnosed stroke commits: %s" % result)
		if not result.get("committed", false):
			continue
		expect(simulation.get_edge_class(0) == 2, "Buried intent is saved as Tunnel")
		var geometry: PackedVector3Array = simulation.get_edge_geometry_3d(0)
		expect(geometry == exact.get("prepared_points", PackedVector3Array()), "Exact published profile agrees")
		expect(geometry.has(Vector3(-50, -6, 0)), "The buried profile is retained")
		var digest := roadbed_digest()
		var path := "/private/tmp/road61-%s.sqlite" % ("grounded" if grounded else "buried")
		expect(simulation.save_game(path), "Save new tunnel fixture")
		expect(simulation.undo_action(), "One tunnel placement has one undo")
		var deadline := Time.get_ticks_msec() + 15000
		while not simulation.get_road_cross_section(0).is_empty() and Time.get_ticks_msec() < deadline:
			await process_frame
		expect(simulation.get_road_cross_section(0).is_empty(), "Undo removes tunnel and shell")
		expect(simulation.load_game(path), "Reload saved tunnel")
		expect(roadbed_digest() == digest, "Profile, concrete and terrain ownership round-trip")

func hill_tunnel(capturing: bool) -> void:
	expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Hill world")
	simulation.set_simulation_speed(0)
	simulation.sculpt_terrain(Vector2.ZERO, 100, 1.4)
	var roof_before: float = simulation.get_world_surface_height(Vector2.ZERO)
	expect(roof_before > 25.0, "Terrain editor creates a real hill")
	var points := PackedVector3Array([Vector3(-150, 0, 0), Vector3(150, 0, 0)])
	if capturing:
		var manager: Node = main.get_node("InputManager")
		manager._refresh_after_world_load()
		manager.set_simulation_speed(0)
		var tool: Node = main.get_node("RoadTool")
		tool.active = true
		tool.fwd_lanes = 1
		tool.bkw_lanes = 1
		tool.set_median(0, 0)
		tool.set_crossing_mode(3)
		tool.set_road_preview_mode(1)
		tool._begin_stroke(points[0], 0, Vector3.ZERO)
		tool.set_scripted_pointer(true, points[-1])
		tool._queue_preview_update()
		var deadline := Time.get_ticks_msec() + 60000
		while (tool._preview_drawn_request_id == 0 or tool._preview_result_pending) and Time.get_ticks_msec() < deadline:
			await process_frame
		expect(tool._preview_cache_surface.get("is_valid", false), "Player Tunnel preview is valid")
		expect(tool._commit_segment(points[-1]), "Player Tunnel action submits")
		deadline = Time.get_ticks_msec() + 60000
		while tool._commit_request_id > 0 and Time.get_ticks_msec() < deadline:
			await process_frame
		tool.cancel_road()
		tool.set_scripted_pointer(false)
		tool.active = false
		tool.update_main_mesh()
	else:
		simulation.set_road_tool_tunnel(true)
		var quick: Dictionary = await quick_verdict(points)
		var exact := await exact_preview(points)
		expect(quick.get("is_valid", false) == exact.get("is_valid", false), "Hill quick/exact parity")
		var result := await finish_request(simulation.add_road_with_snap(points, 1, 1, false))
		expect(result.get("committed", false), "Hill tunnel commits: %s" % result)
	expect(simulation.get_edge_class(0) == 2, "Hill road is a tunnel")
	expect(absf(simulation.get_world_surface_height(Vector2.ZERO) - roof_before) < 0.01, "Terrain stays closed above the bore")
	var concrete := 0
	for chunk: Dictionary in simulation.get_road_mesh_data(true).get("chunks", []):
		concrete += chunk.get("concrete_vertices", PackedVector3Array()).size()
	expect(concrete > 100, "Both portals and the interior lining have concrete")
	await traffic_probe()
	if capturing:
		await tunnel_capture("west_portal", Vector3(-94, 1.7, 1.75), Vector3(-50, 2.0, 0))
		await tunnel_capture("east_portal", Vector3(94, 1.7, -1.75), Vector3(50, 2.0, 0))
		await tunnel_capture("inside_street", Vector3(0, 1.7, 1.75), Vector3(90, 1.7, 1.75))
		await tunnel_capture("overhead", Vector3(-160, 160, 180), Vector3(0, 5, 0))

func traffic_probe() -> void:
	var position := Vector3(-132, 0, 1.75)
	var car: int = simulation.spawn_debug_car(position, Vector3(132, 0, 1.75))
	expect(car >= 0, "A vehicle route crosses the tunnel")
	var entered := false
	var exited := false
	var observations: Array = []
	simulation.set_simulation_speed(4.0)
	var deadline := Time.get_ticks_msec() + 15000
	while not exited and car >= 0 and Time.get_ticks_msec() < deadline:
		await process_frame
		if simulation.pick_debug_agent(position, 40) != car:
			continue
		var details: Dictionary = simulation.get_agent_inspection(car)
		if not details.get("active", false):
			continue
		position = details.position
		if absf(position.x) < 25:
			entered = true
			expect(absf(position.y) < 0.1, "Traffic stays on buried pavement")
		if entered and position.x > 100:
			exited = true
		if observations.is_empty() or position.x - observations[-1][0] > 20:
			observations.append([position.x, position.y, position.z])
	simulation.set_simulation_speed(0)
	observations.append([position.x, position.y, position.z])
	expect(entered and exited, "Traffic enters and exits both portals: %s" % [observations])
	if main != null:
		var file := FileAccess.open(output.path_join("traffic_probe.json"), FileAccess.WRITE)
		file.store_string(JSON.stringify({"entered": entered, "exited": exited, "positions": observations}))

func tunnel_capture(label: String, eye: Vector3, target: Vector3) -> void:
	camera.global_position = eye
	camera.look_at(target, Vector3.UP)
	var stable := 0
	var deadline := Time.get_ticks_msec() + 60000
	while stable < 24 and Time.get_ticks_msec() < deadline:
		await process_frame
		stable = stable + 1 if capture_terrain_settled(Vector3.ZERO, 220) and not simulation.is_network_dirty() else 0
	expect(stable >= 24, "Production terrain settled: " + label)
	await RenderingServer.frame_post_draw
	expect(root.get_texture().get_image().save_png(output.path_join(label + ".png")) == OK, "Capture " + label)
