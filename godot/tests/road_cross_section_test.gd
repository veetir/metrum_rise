# SPDX-License-Identifier: GPL-2.0-only

## Median bridge regression and opt-in windowed road slice A capture.
## METRUM_ROADS_A_CAPTURE=1 godot --path godot --script res://tests/road_cross_section_test.gd
## Captures use a fresh world and write only captures/roads_a; no player save is overwritten.
extends SceneTree

var simulation: Node
var main: Node
var failures: int = 0
var output: String
var camera: Camera3D

func _initialize() -> void:
	call_deferred("run")

func expect(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error(message)

func finish_request(request: int) -> Dictionary:
	var deadline := Time.get_ticks_msec() + 60000
	while Time.get_ticks_msec() < deadline:
		var result: Variant = simulation.get_road_commit_result(request)
		if result is Dictionary:
			return result
		await process_frame
	expect(false, "Road command timed out")
	return {}

func build(points: PackedVector3Array, fwd: int, bkw: int, style: int, width: float) -> bool:
	expect(simulation.set_road_tool_median(style, width).is_empty(), "Median selection must be accepted")
	var result := await finish_request(simulation.add_road_with_snap(points, fwd, bkw, true))
	var accepted := bool(result.get("committed", false))
	expect(accepted, "Road build failed: %s %s" % [result.get("reason", ""), result.get("detail", "")])
	return accepted

func run() -> void:
	if OS.get_environment("METRUM_ROADS_A_CAPTURE") == "1":
		await capture_world()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		await bridge_tests()
		simulation.queue_free()
		await process_frame
	quit(failures)

func bridge_tests() -> void:
	for style in [1, 2, 3]:
		expect(simulation.create_blank_world(512.0, 512.0, 4.0, 128.0, 0.0), "Blank road test world")
		simulation.set_simulation_speed(0.0)
		var width: float = [0.0, 1.0, 5.0, 2.0][style]
		var points := PackedVector3Array([Vector3(-100, 0, 0), Vector3(100, 0, 0)])
		expect(simulation.set_road_tool_median(style, width).is_empty(), "Select median")
		var refused: Dictionary = simulation.validate_road_candidate_with_snap(points, 1, 0, true)
		expect(not refused.get("is_valid", true) and refused.get("invalid_reason", "") == "median_requires_both_directions", "One-way median must explain its refusal")
		var request: int = simulation.request_preview_road_surface_with_options(points, 1, 1, true, false)
		var preview: Variant = null
		var deadline := Time.get_ticks_msec() + 30000
		while preview == null and Time.get_ticks_msec() < deadline:
			preview = simulation.get_preview_road_surface_result(request, 0, PackedInt64Array())
			await process_frame
		expect(preview is Dictionary and preview.get("is_valid", false), "Median preview must compile")
		if not await build(points, 1, 1, style, width):
			continue
		var edge: int = simulation.get_hovered_edge(0.0, 0.0)
		var section: Dictionary = simulation.get_road_cross_section(edge)
		expect(section.get("median_style", -1) == style and section.get("median_width", 0.0) == width, "Committed median selection")
		var result := await finish_request(simulation.change_road_cross_section(edge, 2, 2))
		expect(result.get("committed", false), "In-place edit: %s" % result)
		expect(simulation.get_road_cross_section(edge).get("fwd_lanes", 0) == 2, "Edit must keep the original edge ID")
		if style == 2:
			var data: Dictionary = simulation.get_road_mesh_data(true)
			expect(str(data).contains("median_grass_vertices"), "Grass layer must cross the bridge")
		# A divided arm crossing an undivided road must retain its median on both split halves.
		await build(PackedVector3Array([Vector3(0, 0, -90), Vector3(0, 0, 90)]), 1, 1, 0, 0.0)
		expect(simulation.get_road_cross_section(simulation.get_hovered_edge(-50.0, 0.0)).get("median_style", -1) == style, "Split must preserve median")
		expect(simulation.get_road_cross_section(simulation.get_hovered_edge(50.0, 0.0)).get("median_style", -1) == style, "Both split halves retain median")

func ground(x: float, z: float) -> Vector3:
	return Vector3(x, simulation.get_world_surface_height(Vector2(x, z)), z)

func stroke(origin: Vector2, offsets: Array) -> PackedVector3Array:
	var points := PackedVector3Array()
	for offset: Vector2 in offsets:
		points.append(ground(origin.x + offset.x, origin.y + offset.y))
	return points

func settle() -> bool:
	var stable := 0
	var deadline := Time.get_ticks_msec() + 120000
	while stable < 24 and Time.get_ticks_msec() < deadline:
		await process_frame
		if not main.get_node("Terrain").has_pending_render_work(false) and not main.get_node("Water").has_pending_render_work(false) and not main.get_node("Vegetation").has_pending_work() and not simulation.is_network_dirty():
			stable += 1
		else:
			stable = 0
	expect(stable >= 24, "Rendered world must settle: terrain=%s water=%s vegetation=%s network=%s" % [main.get_node("Terrain").get_pending_render_work_counts(), main.get_node("Water").get_pending_render_work_counts(), main.get_node("Vegetation").has_pending_work(), simulation.is_network_dirty()])
	return stable >= 24

func capture(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float = -0.65) -> void:
	camera.set_view_state(PackedFloat32Array([pivot.x, pivot.y, pivot.z, yaw, pitch, distance, 0.0]))
	if pitch > -0.2:
		# The normal orbit clamps its pitch for city views. Inspect at a driver's eye height.
		var forward := Vector3(-sin(yaw), 0.0, -cos(yaw))
		var eye := pivot - forward * distance
		eye.y = simulation.get_world_surface_height(Vector2(eye.x, eye.z)) + 1.7
		var target := pivot + forward * 60.0
		target.y = simulation.get_world_surface_height(Vector2(target.x, target.z)) + 1.3
		camera.global_position = eye
		camera.look_at(target, Vector3.UP)
	if not await settle():
		return
	for frame in 24:
		await process_frame
	await RenderingServer.frame_post_draw
	expect(root.get_texture().get_image().save_png(output.path_join(label + ".png")) == OK, "Save capture " + label)

func capture_world() -> void:
	output = ProjectSettings.globalize_path("res://../captures/roads_a")
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
	expect(simulation.create_blank_world(1600.0, 1600.0, 4.0, 128.0, 0.0), "Fresh capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0.0)
	var neula := Vector2(-350.0, -300.0)
	await build(stroke(neula, [Vector2(-130, 0), Vector2(-60, 0), Vector2(0, 0), Vector2(65, 4), Vector2(130, 16)]), 1, 1, 2, 5.0)
	await capture("neulamaentie_above", ground(neula.x, neula.y), 155.0, -0.85)
	await capture("neulamaentie_street", ground(neula.x - 40.0, neula.y + 4.25) + Vector3.UP * 1.0, 24.0, -0.10, -1.57)
	var prisma := Vector2(300.0, -300.0)
	await build(stroke(prisma, [Vector2(-150, 0), Vector2(150, 0)]), 2, 2, 2, 3.0)
	await build(stroke(prisma, [Vector2(0, -110), Vector2(0, 110)]), 2, 2, 0, 0.0)
	await capture("prisma_above", ground(prisma.x, prisma.y), 165.0, -0.95)
	await capture("prisma_street", ground(prisma.x - 55.0, prisma.y + 3.25) + Vector3.UP * 1.0, 30.0, -0.12, -1.57)
	for item in [{"name": "painted", "style": 1, "width": 1.5, "origin": Vector2(-400, 250)}, {"name": "paving", "style": 3, "width": 2.0, "origin": Vector2(0, 250)}, {"name": "grass_curve", "style": 2, "width": 5.0, "origin": Vector2(400, 250)}]:
		var origin: Vector2 = item.origin
		await build(stroke(origin, [Vector2(-120, -10), Vector2(-60, -4), Vector2(0, 0), Vector2(60, -4), Vector2(120, -10)]), 1, 1, item.style, item.width)
		await capture(item.name + "_above", ground(origin.x, origin.y), 145.0, -0.9)
		await capture(item.name + "_street", ground(origin.x - 30.0, origin.y + item.width * 0.5 + 1.75) + Vector3.UP, 24.0, -0.12, -1.57)
	await build(stroke(Vector2(0, -50), [Vector2(-110, 0), Vector2(0, 0)]), 1, 1, 2, 5.0)
	await build(stroke(Vector2(0, -50), [Vector2(0, 0), Vector2(110, 0)]), 1, 1, 0, 0.0)
	await build(stroke(Vector2(0, -50), [Vector2(0, 0), Vector2(0, 90)]), 1, 1, 0, 0.0)
	await capture("divided_to_undivided_t", ground(0, -50), 125.0, -0.95)
	await build(stroke(Vector2(-550, -40), [Vector2(-130, -40), Vector2(0, 0)]), 1, 1, 2, 5.0)
	await build(stroke(Vector2(-550, -40), [Vector2(0, 0), Vector2(90, 80)]), 1, 1, 2, 5.0)
	await capture("continuous_divided_bend", ground(-550, -40), 145.0, -0.95)
	# A player-requested crossing opens the otherwise continuous island before the zebra.
	var bend_node: int = simulation.get_closest_node(ground(-550, -40), 5.0)
	var bend_edge: int = simulation.get_hovered_edge(-610.0, -58.0)
	expect(bend_node >= 0 and bend_edge >= 0, "Pick divided bend for explicit crossing")
	expect(not simulation.has_crosswalk(bend_node, bend_edge), "Continuous median has no default zebra")
	simulation.set_crosswalk_override(bend_node, bend_edge, true)
	expect(simulation.has_crosswalk(bend_node, bend_edge), "Requested crossing opens the median")
	main.get_node("RoadTool").update_main_mesh()
	await capture("divided_bend_requested_crossing", ground(-550, -40), 145.0, -0.95)
	# Exercise the actual tool controller's edit action, retaining the same edge.
	var tool: Node = main.get_node("RoadTool")
	tool.active = true
	tool.fwd_lanes = 2
	tool.bkw_lanes = 2
	tool.set_median(2, 5.0)
	tool.toggle_cross_section_edit()
	var edited: int = simulation.get_hovered_edge(neula.x, neula.y)
	tool.apply_cross_section_to_edge(edited)
	var deadline := Time.get_ticks_msec() + 60000
	while tool._edit_request_id > 0 and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(simulation.get_road_cross_section(edited).get("fwd_lanes", 0) == 2, "Tool in-place edit")
	await capture("tool_edited_neulamaentie", ground(neula.x, neula.y), 155.0, -0.85)
	tool.toggle_cross_section_edit()
	tool.fwd_lanes = 1
	tool.bkw_lanes = 1
	tool.set_median(3, 2.0)
	tool.set_road_preview_mode(1)
	var draw_start := ground(-200, 520)
	var draw_end := ground(0, 520)
	tool._begin_stroke(draw_start, 0, Vector3.ZERO)
	tool.set_scripted_pointer(true, draw_end)
	tool._queue_preview_update()
	deadline = Time.get_ticks_msec() + 60000
	while (tool._preview_drawn_request_id == 0 or tool._preview_result_pending) and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(tool._preview_cache_surface.get("is_valid", false), "Drawing tool median preview")
	await capture("tool_paving_preview", ground(-100, 520), 145.0, -0.95)
	expect(tool._commit_segment(draw_end), "Drawing tool submits median stroke")
	deadline = Time.get_ticks_msec() + 60000
	while tool._commit_request_id > 0 and Time.get_ticks_msec() < deadline:
		await process_frame
	tool.cancel_road()
	tool.set_scripted_pointer(false)
	expect(simulation.get_road_cross_section(simulation.get_hovered_edge(-100, 520)).get("median_style", -1) == 3, "Drawing tool committed paving")
	await capture("tool_paving_committed", ground(-100, 520), 145.0, -0.95)
	tool.active = false
	# A separate fresh real-Kuopio world: deterministically find a dry sloping land corridor.
	var world := ProjectSettings.globalize_path("res://bootstrap/worlds/kuopio_324km2_10m.sqlite")
	expect(manager.menu_load_world_definition(world), "Load Kuopio terrain")
	manager.set_simulation_speed(0.0)
	var slope_origin := Vector2.ZERO
	var found := false
	for z in range(-1800, 1801, 200):
		for x in range(-1800, 1801, 200):
			var a := ground(x - 100, z)
			var b := ground(x + 100, z)
			var grade := absf(b.y - a.y) / 200.0
			if a.y > 100.0 and b.y > 100.0 and grade >= 0.02 and grade <= 0.06:
				slope_origin = Vector2(x, z)
				found = true
				break
		if found: break
	expect(found, "Find a dry Kuopio slope")
	await build(stroke(slope_origin, [Vector2(-100, 0), Vector2(-50, 0), Vector2(0, 0), Vector2(50, 0), Vector2(100, 0)]), 1, 1, 2, 5.0)
	await capture("kuopio_sloped_above", ground(slope_origin.x, slope_origin.y), 150.0, -0.9)
	await capture("kuopio_sloped_street", ground(slope_origin.x - 30, slope_origin.y + 4.25) + Vector3.UP, 25.0, -0.12, -1.57)
	main.queue_free()
	await process_frame
