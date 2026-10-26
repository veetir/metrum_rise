# SPDX-License-Identifier: GPL-2.0-only

## Bounded ROAD-82 player-controller diagnostic, with optional single-window captures.
## METRUM_PLAYER_CAPTURE=1 enables paired views; artifacts stay in captures/player_prepr.
## Refused placements are reported as findings, so the remaining matrix still runs.
extends "res://tests/road_pockets_legacy_test.gd"

var road: Node
var manager: Node
var findings: Array = []
var checks: Array = []
var case_name := ""
var captures := false

func note(ok: bool, label: String, detail: Variant = "") -> bool:
	var entry := {"case": case_name, "check": label, "ok": ok, "detail": str(detail)}
	checks.append(entry)
	if not ok:
		findings.append(entry)
	var report := FileAccess.open(output.path_join("report.json"), FileAccess.WRITE)
	report.store_string(JSON.stringify({"checks": checks, "findings": findings}, "\t"))
	return ok

func finish_request(request: int) -> Dictionary:
	# The replay consumes completion once; suspend the controllers' normal pollers.
	var controllers: Array = [road, main.get_node("RoundaboutTool"), main.get_node("RampTool")]
	for controller: Node in controllers:
		controller.set_process(false)
	var result := await super.finish_request(request)
	for controller: Node in controllers:
		controller.set_process(true)
	return result

func run() -> void:
	output = ProjectSettings.globalize_path("res://../captures/player_prepr")
	DirAccess.make_dir_recursive_absolute(output)
	captures = OS.get_environment("METRUM_PLAYER_CAPTURE") == "1"
	main = load("res://scenes/Main.tscn").instantiate()
	main.set_script(null)
	root.add_child(main)
	current_scene = main
	await process_frame
	main.get_node("MainUI").visible = false
	manager = main.get_node("InputManager")
	manager.set_process(false)
	manager.set_process_input(false)
	manager.set_process_unhandled_input(false)
	simulation = main.get_node("SimulationNode")
	road = main.get_node("RoadTool")
	camera = main.get_node("CameraNode")
	camera.set_process(false)
	camera.set_process_input(false)
	camera.set_process_unhandled_input(false)
	root.set_disable_input(true)
	Engine.max_fps = 60
	if captures:
		DisplayServer.window_set_size(Vector2i(1200, 800))
	main.get_node("SceneLighting").pin_hour_of_day(12)
	for slope in [false, true]:
		for shape in ["t90", "x90", "t45", "x45", "bus", "ring3", "ring4", "over", "under", "ramp"]:
			case_name = ("slope_" if slope else "flat_") + shape
			await fresh(slope)
			if shape.begins_with("ring"):
				await ring(int(shape.right(1)))
			elif shape == "ramp":
				await ramp()
			elif shape in ["over", "under"]:
				await crossing(shape == "over")
			else:
				await junction_case(shape)
			expect(simulation.save_game(output.path_join("roads-player-" + case_name + ".sqlite")), "Save task-owned audit fixture")
			await traffic_smoke(shape)
			await pair("final")
			await bulldoze_and_undo()
	main.queue_free()
	await process_frame
	quit(failures)

func fresh(slope: bool) -> void:
	road.cancel_road()
	road.active = false
	for name in ["RoundaboutTool", "RampTool"]:
		main.get_node(name).active = false
		main.get_node(name).cancel()
	expect(simulation.create_blank_world(1600, 1600, 4, 128, 2.5), "New blank player map")
	simulation.set_simulation_speed(0)
	if slope:
		# The normal terrain editor creates one exact plane, away from brush falloff.
		simulation.slope_terrain(Vector2.ZERO, 1600, Vector2(-800, 0), 34, Vector2(800, 0), 66, 1000)
		note(absf((ground(400, 0).y - ground(-400, 0).y) / 800.0 - 0.02) < 0.0001, "2 percent terrain slope")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0)
	road.draw_mode = 0
	road.edit_cross_section = false
	road.edit_pocket = false
	road.set_road_preview_mode(1)
	road.set_crossing_mode(0)
	camera.set_view_state(PackedFloat32Array([0, 50, 0, -0.65, -1.1, 260, 0]))
	await ready_scene()

func ready_scene() -> void:
	var deadline := Time.get_ticks_msec() + 20000
	var stable := 0
	while stable < 8 and Time.get_ticks_msec() < deadline:
		await process_frame
		if not simulation.is_network_dirty() and not main.get_node("Terrain").has_pending_render_work(false) and not main.get_node("Water").has_pending_render_work(false):
			stable += 1
		else:
			stable = 0
	if stable < 8:
		note(false, "scene settling timeout")

func pair(label: String, distance: float = 220.0) -> void:
	if not captures or label.contains("restore"):
		return
	await view(label + "_overhead", ground(0, 0), distance, -1.25)
	await view(label + "_street", ground(-65, 5), 24, -0.10, -1.57)

func view(label: String, pivot: Vector3, distance: float, pitch: float, yaw: float = -0.65) -> void:
	camera.set_view_state(PackedFloat32Array([pivot.x, pivot.y, pivot.z, yaw, pitch, distance, 0]))
	if pitch > -0.2:
		camera.global_position = pivot + Vector3(sin(yaw), 0, cos(yaw)) * distance
		camera.global_position.y = simulation.get_world_surface_height(Vector2(camera.global_position.x, camera.global_position.z)) + 1.7
		var target := pivot + Vector3(-sin(yaw), 0, -cos(yaw)) * 60
		target.y = pivot.y + 1.3
		camera.look_at(target)
	await ready_scene()
	for frame in 8:
		await process_frame
	await RenderingServer.frame_post_draw
	expect(root.get_texture().get_image().save_png(output.path_join(case_name + "_" + label + ".png")) == OK, "Capture " + label)

func undo_to(digest: String, label: String) -> bool:
	road.cancel_road()
	road.active = false
	manager._handle_undo()
	var deadline := Time.get_ticks_msec() + 15000
	while roadbed_digest() != digest and Time.get_ticks_msec() < deadline:
		await process_frame
	return note(roadbed_digest() == digest, "undo " + label + " restores roadbed")

func draw(a: Vector3, b: Vector3, lanes: int, median: bool, label: String, check_undo: bool = true) -> bool:
	road.fwd_lanes = lanes
	road.bkw_lanes = lanes
	road.set_median(2 if median else 0, 5 if median else 0)
	var before := roadbed_digest()
	road.set_scripted_pointer(true, b)
	road._begin_stroke(a, 0, Vector3.ZERO)
	road._queue_preview_update()
	var deadline := Time.get_ticks_msec() + 15000
	while (road._preview_drawn_request_id == 0 or road._preview_result_pending) and Time.get_ticks_msec() < deadline:
		await process_frame
	var preview: Dictionary = road._preview_cache_surface
	if not note(preview.get("is_valid", false), label + " preview", preview.get("invalid_reason", "no exact preview")):
		road.cancel_road()
		road.active = false
		return false
	await pair(label + "_preview")
	var points: PackedVector3Array = preview.get("prepared_points", PackedVector3Array())
	if not note(road._commit_segment(b), label + " submit"):
		return false
	var result := await finish_request(road._commit_request_id)
	road._commit_request_id = 0
	road.cancel_road()
	road.active = false
	if not note(result.get("committed", false), label + " commit", result):
		return false
	await ready_scene()
	if road.crossing_mode != 0:
		var edge: int = simulation.get_hovered_edge(0, -90)
		note(simulation.get_edge_geometry_3d(edge) == points, label + " preview profile equals built")
	await pair(label + "_built")
	if check_undo and await undo_to(before, label):
		return await draw(a, b, lanes, median, label + "_restore", false)
	return true

func junction_case(shape: String) -> void:
	var lanes := 2 if shape in ["x90", "x45", "bus"] else 1
	if not await draw(ground(-260, 0), ground(260, 0), lanes, true, "main"):
		return
	if shape == "bus":
		var controls: Node = load("res://scripts/ui/road_lane_use_controls.gd").new()
		root.add_child(controls)
		controls.setup(simulation, null)
		var edge: int = simulation.get_hovered_edge(-100, 0)
		var before := roadbed_digest()
		controls.show_edge(edge)
		controls.get_child(1).button_pressed = true
		while controls.busy:
			await process_frame
		note(simulation.get_road_cross_section(edge).get("bus_forward_mask", 0) == 2, "Road Properties bus checkbox")
		manager._handle_undo()
		var deadline := Time.get_ticks_msec() + 15000
		while simulation.get_road_cross_section(edge).get("bus_forward_mask", 0) != 0 and Time.get_ticks_msec() < deadline:
			await process_frame
		note(simulation.get_road_cross_section(edge).get("bus_forward_mask", 0) == 0 and roadbed_digest() == before, "undo bus permission")
		controls.show_edge(edge)
		controls.get_child(1).button_pressed = true
		while controls.busy:
			await process_frame
		controls.queue_free()
		return
	var skew := shape.ends_with("45")
	var is_t := shape.begins_with("t")
	if not await draw(ground(-150 if skew else 0, -150), ground(0 if is_t else (150 if skew else 0), 0 if is_t else 150), 1, false, "branch"):
		return
	var edge: int = simulation.get_hovered_edge(-60, 0)
	var before := roadbed_digest()
	road.active = true
	road.pocket_left = true
	road.pocket_right = false
	road.pocket_length = 80
	road.pocket_taper = 35
	var preview: Dictionary = simulation.preview_road_pocket(edge, false, true, false, 80, 35)
	if note(preview.get("is_valid", false), "pocket preview", preview.get("invalid_reason", "")):
		road.show_local_edit_preview(preview)
		await pair("pocket_preview")
		var result := await finish_request(road.apply_pocket_to_end(edge, false))
		road._edit_request_id = 0
		road.active = false
		if note(result.get("committed", false), "pocket commit", result):
			await pair("pocket_built")
			if await undo_to(before, "pocket"):
				result = await finish_request(road.apply_pocket_to_end(edge, false))
				road._edit_request_id = 0
				note(result.get("committed", false), "restore pocket", result)
	# Selection's exact zebra preview and authoritative per-arm setter.
	var node: int = simulation.get_closest_node(ground(0, 0), 3)
	edge = simulation.get_hovered_edge(60, 0)
	preview = simulation.preview_crosswalk(node, edge, true)
	if note(preview.get("is_valid", false), "zebra preview", preview.get("invalid_reason", "")):
		road.show_local_edit_preview(preview)
		await pair("zebra_preview")
		road._clear_preview_visual()
		simulation.set_crosswalk_override(node, edge, true)
		note(simulation.has_crosswalk(node, edge), "Selection zebra setter")
		await pair("zebra_built")
		manager._handle_undo()
		var deadline := Time.get_ticks_msec() + 15000
		while simulation.has_crosswalk(node, edge) and Time.get_ticks_msec() < deadline:
			await process_frame
		note(not simulation.has_crosswalk(node, edge), "undo zebra")
		simulation.set_crosswalk_override(node, edge, true)

func click(tool: Node) -> void:
	var event := InputEventMouseButton.new()
	event.button_index = MOUSE_BUTTON_LEFT
	event.pressed = true
	tool._unhandled_input(event)

func wait_tool_preview(tool: Node) -> void:
	var deadline := Time.get_ticks_msec() + 15000
	while not tool._valid and Time.get_ticks_msec() < deadline:
		tool._update_preview()
		await process_frame

func ring(arms: int) -> void:
	var before := roadbed_digest()
	var tool: Node = main.get_node("RoundaboutTool")
	tool.arms = arms
	tool.diameter = 50
	tool.set_scripted_pointer(true, ground(0, 0))
	tool.active = true
	var deadline := Time.get_ticks_msec() + 15000
	while not tool._valid and Time.get_ticks_msec() < deadline:
		await process_frame
	if not note(tool._valid, "roundabout preview", tool._label.text):
		return
	await pair("preview", 115)
	click(tool)
	var result := await finish_request(tool._request)
	tool._request = 0
	tool.active = false
	tool.cancel()
	if not note(result.get("committed", false), "roundabout click", result):
		return
	await pair("built", 115)
	if await undo_to(before, "roundabout"):
		# Use the same controller input for the second placement.
		tool.active = true
		await wait_tool_preview(tool)
		click(tool)
		result = await finish_request(tool._request)
		tool._request = 0
		tool.active = false
		tool.cancel()
		note(result.get("committed", false), "restore roundabout", result)

func crossing(over: bool) -> void:
	if not await draw(ground(-220, 0), ground(220, 0), 2, false, "lower"):
		return
	road.set_crossing_mode(1 if over else 2)
	await draw(ground(0, -180), ground(0, 180), 1, false, "over" if over else "under")
	road.set_crossing_mode(0)

func ramp() -> void:
	if not await draw(ground(-600, 0), ground(600, 0), 2, true, "main"):
		return
	var before := roadbed_digest()
	var tool: Node = main.get_node("RampTool")
	tool.set_scripted_pointer(true, ground(0, 0))
	tool.active = true
	# Scripted pointer positions enter the same immutable picker in both display modes.
	click(tool)
	if not note(not tool._anchor.is_empty(), "ramp anchor click", tool._anchor):
		return
	tool.set_scripted_pointer(true, ground(200, 100))
	var deadline := Time.get_ticks_msec() + 15000
	while not tool._valid and Time.get_ticks_msec() < deadline:
		await process_frame
	if not note(tool._valid, "ramp preview", tool._label.text):
		return
	await pair("preview", 260)
	click(tool)
	var result := await finish_request(tool._request)
	tool._request = 0
	tool.active = false
	tool.cancel()
	if note(result.get("committed", false), "ramp endpoint click", result):
		await pair("built", 260)
		if await undo_to(before, "ramp"):
			tool._anchor = simulation.get_ramp_anchor(ground(0, 0))
			tool.active = true
			await wait_tool_preview(tool)
			click(tool)
			result = await finish_request(tool._request)
			tool._request = 0
			tool.active = false
			tool.cancel()
			note(result.get("committed", false), "restore ramp", result)

func traffic_smoke(shape: String) -> void:
	var source := ground(-260, 0)
	var goal := ground(260, 0)
	if shape.begins_with("ring"):
		source = ground(78, 0)
		var angle := TAU / float(int(shape.right(1)))
		goal = ground(cos(angle) * 78, sin(angle) * 78)
	elif shape in ["over", "under"]:
		source = ground(0, -180)
		goal = ground(0, 180)
	elif shape == "ramp":
		source = ground(-600, 0)
		goal = ground(200, 100)
	var id: int = simulation.spawn_debug_car(source, goal)
	if not note(id >= 0, "traffic route exists"):
		return
	main.get_node("Agents").set_process(false)
	simulation.set_camera_aabb(-800, 800, -800, 800)
	simulation.set_simulation_speed(4)
	var moved := false
	var deadline := Time.get_ticks_msec() + 4000
	while not moved and Time.get_ticks_msec() < deadline:
		await process_frame
		for bucket: Dictionary in simulation.get_car_render_data().values():
			var ids: PackedInt64Array = bucket.ids
			var transforms: PackedFloat32Array = bucket.transforms
			for index in ids.size():
				if ids[index] == id:
					var at := Vector3(transforms[index * 12 + 3], transforms[index * 12 + 7], transforms[index * 12 + 11])
					moved = at.distance_to(source) > 8
	simulation.set_simulation_speed(0)
	main.get_node("Agents").set_process(true)
	note(moved, "traffic advances (smoke only)")

func bulldoze_and_undo() -> void:
	var pos := Vector2(-100, 0)
	if case_name.contains("ring"):
		pos = Vector2(60, 0)
	var target: Dictionary = {}
	var pick_deadline := Time.get_ticks_msec() + 5000
	while not target.get("valid", false) and Time.get_ticks_msec() < pick_deadline:
		target = simulation.get_bulldoze_target_at(pos.x, pos.y)
		await process_frame
	if not note(target.get("valid", false), "bulldoze target", target.get("kind", "")):
		return
	var before := roadbed_digest()
	var tool: Node = main.get_node("BulldozeTool")
	tool._target_world_pos = pos
	tool._target_valid = true
	tool._delete_hovered_target()
	var deadline := Time.get_ticks_msec() + 15000
	while roadbed_digest() == before and Time.get_ticks_msec() < deadline:
		await process_frame
	if not note(roadbed_digest() != before, "bulldoze click changes roadbed"):
		await pair("bulldoze_ignored")
		# Record the lost click above, then retry to cover deletion/undo independently.
		deadline = Time.get_ticks_msec() + 5000
		while roadbed_digest() == before and Time.get_ticks_msec() < deadline:
			tool._target_valid = true
			tool._delete_hovered_target()
			await create_timer(0.1).timeout
	if roadbed_digest() != before:
		await pair("bulldozed")
		await undo_to(before, "bulldoze")
