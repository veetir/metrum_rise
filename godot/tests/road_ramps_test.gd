# SPDX-License-Identifier: GPL-2.0-only

## ROAD-69 ramp-stub bridge regression and opt-in real-scene captures.
## METRUM_RP_CAPTURE=1 writes only captures/ramps and roads-rp-prefixed local saves.
extends "res://tests/road_pockets_legacy_test.gd"

class BusySurfaceTool extends "res://scripts/tools/ramp_tool.gd":
	var legacy_picks := 0

	func get_terrain_interaction():
		legacy_picks += 1
		return null

func run() -> void:
	if OS.get_environment("METRUM_RP_POINTER_ONLY") == "1":
		await check_player_pointer()
		quit(failures)
		return
	if OS.get_environment("METRUM_RP_LEGACY_ONLY") == "1":
		await legacy_tapers()
		quit(failures)
		return
	if OS.get_environment("METRUM_RP_CAPTURE") == "1":
		await capture_ramps()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		for exit_ramp in [true, false]:
			for forward in [true, false]:
				expect(simulation.create_blank_world(1600, 1024, 4, 128, 0), "Fresh ramp test world")
				simulation.set_simulation_speed(0)
				await ramp_fixture(exit_ramp, forward)
				await drive_ramp(exit_ramp, forward)
		simulation.queue_free()
		await process_frame
		await check_player_pointer()
	quit(failures)

func endpoint(exit_ramp: bool, forward: bool) -> Vector3:
	var side := 1.0 if forward else -1.0
	return Vector3(side * (200.0 if exit_ramp else -200.0), 0, side * 100.0)

func ramp_fixture(exit_ramp: bool, forward: bool) -> void:
	await build(PackedVector3Array([Vector3(-600, 0, 0), Vector3(600, 0, 0)]), 2, 2, 2, 5)
	var anchor: Dictionary = simulation.get_ramp_anchor(Vector3.ZERO)
	expect(not anchor.is_empty() and absf(float(anchor.get("station", 0)) - 600) < 0.01, "Indexed ramp anchor")
	var points := PackedVector3Array([Vector3.ZERO, endpoint(exit_ramp, forward)])
	var preview: Dictionary = simulation.preview_ramp(0, 600, forward, exit_ramp, points, 80, 35)
	expect(preview.get("is_valid", false), "Ramp preview: %s" % preview.get("invalid_reason", ""))
	expect(not preview.get("terrain_preview", {}).get("patches", []).is_empty(), "Ramp previews terrain cutout replacement")
	if main != null:
		var tool: Node3D = main.get_node("RampTool")
		tool.forward = forward
		tool.exit_ramp = exit_ramp
		tool._anchor = anchor
		tool.set_scripted_pointer(true, endpoint(exit_ramp, forward))
		tool.active = true
		await settle()
		expect(tool._valid, "Real ramp tool installs paired road/terrain preview")
		await capture(("exit" if exit_ramp else "entry") + "_tool_preview", Vector3.ZERO, 240, -1.3)
		tool.active = false
		tool.cancel()
	var result := await finish_request(simulation.place_ramp(0, 600, forward, exit_ramp, points, 80, 35))
	expect(result.get("committed", false), "Ramp transaction: %s" % result)
	var save := "/tmp/roads-rp-bridge-%s-%s.sqlite" % [exit_ramp, forward]
	expect(simulation.save_game(save), "Save ramp role and lane restrictions")
	expect(simulation.load_game(save), "Reload ramp role and lane restrictions")
	DirAccess.remove_absolute(save)
	if main != null:
		main.get_node("InputManager")._refresh_after_world_load()
	var ramp: Dictionary = simulation.get_road_cross_section(3)
	expect(ramp.get("fwd_lanes", 0) == 1 and ramp.get("bkw_lanes", -1) == 0, "Ramp is one way")

func drive_ramp(exit_ramp: bool, forward: bool) -> void:
	if main != null:
		# Observe both distant goals independently of the capture camera's frustum.
		# Keep the normal renderer uploads below; only its AABB writer is bypassed.
		main.get_node("Agents").set_process(false)
	var side := 1.0 if forward else -1.0
	var source := Vector3(-600 * side, 0, 0)
	var target := Vector3(600 * side, 0, 0)
	var ramp_end := endpoint(exit_ramp, forward)
	var routes := [[source if exit_ramp else ramp_end, ramp_end if exit_ramp else target], [source, target], [target, source]]
	var goals := {}
	var reached := {}
	simulation.set_camera_aabb(-800, 800, -500, 500)
	simulation.set_simulation_speed(4)
	for route: Array in routes:
		var id: int = simulation.spawn_debug_car(route[0], route[1])
		expect(id >= 0, "Ramp/through route exists")
		goals[id] = route[1]
		await create_timer(0.8).timeout
	if main != null:
		await create_timer(5.0).timeout
		simulation.set_simulation_speed(0)
		main.get_node("Agents").update_swarm()
		await capture(("exit" if exit_ramp else "entry") + "_traffic", Vector3.ZERO, 240, -1.3)
		simulation.set_camera_aabb(-800, 800, -500, 500)
		simulation.set_simulation_speed(4)
	var deadline := Time.get_ticks_msec() + 45000
	while Time.get_ticks_msec() < deadline and reached.size() < goals.size():
		await process_frame
		if main != null:
			main.get_node("Agents").update_swarm(main.get_process_delta_time())
		var buckets: Dictionary = simulation.get_car_render_data()
		for bucket: Dictionary in buckets.values():
			var ids: PackedInt64Array = bucket.ids
			var poses: PackedFloat32Array = bucket.transforms
			for index in ids.size():
				if goals.has(ids[index]):
					var at := Vector3(poses[index * 12 + 3], poses[index * 12 + 7], poses[index * 12 + 11])
					if at.distance_to(goals[ids[index]]) < 30:
						reached[ids[index]] = true
	expect(reached.size() == goals.size(), "Ramp and both main carriageways drain (%d/%d), exit=%s forward=%s" % [reached.size(), goals.size(), exit_ramp, forward])
	simulation.set_simulation_speed(0)
	if main != null:
		main.get_node("Agents").set_process(true)

func capture_ramps() -> void:
	output = ProjectSettings.globalize_path("res://../captures/ramps")
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
	main.get_node("SceneLighting").pin_hour_of_day(12)
	for exit_ramp in [true, false]:
		expect(simulation.create_blank_world(1600, 1024, 4, 128, 0), "Fresh ramp capture world")
		manager._refresh_after_world_load()
		manager.set_simulation_speed(0)
		camera.set_view_state(PackedFloat32Array([0, 0, 0, -0.65, -1.1, 200, 0]))
		await ramp_fixture(exit_ramp, true)
		var label := "exit" if exit_ramp else "entry"
		await capture(label + "_overhead", Vector3.ZERO, 240, -1.3)
		await capture(label + "_street", Vector3(-35, 0, 7.5), 35, -0.12, -1.57)
		await capture(label + "_taper", Vector3(-145 if exit_ramp else 145, 0, 0), 100, -1.2)
		await drive_ramp(exit_ramp, true)
	main.queue_free()
	await process_frame

func legacy_tapers() -> void:
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	var hashes: Array[String] = []
	for style in [0, 2]:
		for at_start in [false, true]:
			for left in ([false] if style == 0 else [false, true]):
				expect(simulation.create_blank_world(1200, 1024, 4, 128, 0), "Legacy taper blank world")
				simulation.set_simulation_speed(0)
				await build(PackedVector3Array([Vector3(-450, 0, 0), Vector3(450, 0, 0)]), 2, 2, style, 5 if style == 2 else 0)
				await build(PackedVector3Array([Vector3(0, 0, -200), Vector3(0, 0, 200)]), 1, 1, 0, 0)
				var edge: int = simulation.get_hovered_edge(100 if at_start else -100, 0)
				var result := await finish_request(simulation.change_road_pocket(edge, at_start, left, true, 80, 35))
				expect(result.get("committed", false), "Legacy right/combined pocket")
				var digest := roadbed_digest()
				var saves := OS.get_environment("METRUM_RP_LEGACY_SAVES")
				if not saves.is_empty():
					var path := saves.path_join("roads-rp-legacy-%d.sqlite" % hashes.size())
					if OS.get_environment("METRUM_RP_LOAD_LEGACY") == "1":
						expect(simulation.load_game(path), "Load previous-version pocket save")
						expect(roadbed_digest() == digest, "Old save retains the complete legacy roadbed")
					else:
						expect(simulation.save_game(path), "Record previous-version pocket save")
				hashes.append(digest)
	var file := FileAccess.open(OS.get_environment("METRUM_RP_LEGACY_OUTPUT"), FileAccess.WRITE)
	expect(file != null, "Write local taper product hashes")
	if file != null:
		file.store_string(JSON.stringify(hashes))
	simulation.queue_free()
	await process_frame

func check_player_pointer() -> void:
	# Real viewport rays on a 2% slope, while the legacy mutex raycast is contended.
	var viewport := SubViewport.new()
	viewport.size = Vector2i(1440, 900)
	root.add_child(viewport)
	main = load("res://scenes/Main.tscn").instantiate()
	main.set_script(null)
	viewport.add_child(main)
	await process_frame
	await process_frame
	var manager: Node = main.get_node("InputManager")
	var original: Node = manager.ramp_tool
	main.remove_child(original)
	original.queue_free()
	var tool := BusySurfaceTool.new()
	tool.name = "RampTool"
	main.add_child(tool)
	manager.ramp_tool = tool
	simulation = main.get_node("SimulationNode")
	camera = main.get_node("CameraNode")
	expect(simulation.create_blank_world(1600, 1024, 4, 128, 2.5), "Ramp player-pointer blank world")
	simulation.slope_terrain(Vector2.ZERO, 1600, Vector2(-800, 0), 34, Vector2(800, 0), 66, 1000)
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0)
	await build(PackedVector3Array([ground(-600, 0), ground(600, 0)]), 2, 2, 2, 5)
	var target := ground(180, 0)
	camera.set_view_state(PackedFloat32Array([target.x, target.y, target.z, 0, -1.2, 180, 0]))
	var motion := InputEventMouseMotion.new()
	motion.position = Vector2(viewport.size) * 0.5
	viewport.push_input(motion, true)
	tool.active = true
	await settle()
	var click := InputEventMouseButton.new()
	click.button_index = MOUSE_BUTTON_LEFT
	click.pressed = true
	tool._unhandled_input(click)
	expect(not tool._scripted_pointer_enabled, "Ramp pointer regression never supplies a scripted world position")
	expect(not tool._anchor.is_empty() and tool._anchor.position.distance_to(target) < 1.0, "Busy first click retains its actual slope anchor instead of the world origin: %s" % tool._anchor)
	expect(tool.legacy_picks == 0, "Ramp picking uses the existing immutable road query")
	# Camera-only pointer motion must also update the outer endpoint preview.
	target = ground(380, 100)
	camera.set_view_state(PackedFloat32Array([target.x, target.y, target.z, 0, -1.2, 180, 0]))
	var deadline := Time.get_ticks_msec() + 15000
	while not tool._valid and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(tool._valid, "Actual player pointer installs the sloping ramp preview: %s" % tool._label.text)
	if tool._valid:
		expect(tool._points[-1].distance_to(target) < 1.0, "Ramp endpoint stays at the camera's real world pointer")
		tool.set_process(false)
		tool._unhandled_input(click)
		var result := await finish_request(tool._request)
		expect(result.get("committed", false), "Commit the real-pointer slope ramp: %s" % result)
	viewport.queue_free()
	await process_frame
	main = null
