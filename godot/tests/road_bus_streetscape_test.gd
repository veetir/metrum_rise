# SPDX-License-Identifier: GPL-2.0-only

## Bus-only lane bridge, save/undo and car turn regression, with real-scene captures.
## METRUM_BS_CAPTURE=1 writes only captures/bus_streetscape and roads-bs-prefixed saves.
extends "res://tests/road_pockets_legacy_test.gd"

func run() -> void:
	if OS.get_environment("METRUM_BS_CAPTURE") == "1":
		await capture_bus_lanes()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		expect(simulation.create_blank_world(1024, 1024, 4, 128, 0), "Fresh bus lane world")
		simulation.set_simulation_speed(0)
		await bus_fixture()
		await check_lane_controls()
		await check_bus_traffic()
		simulation.queue_free()
		await process_frame
	quit(failures)

func bus_fixture() -> void:
	await build(PackedVector3Array([ground(-230, 0), ground(230, 0)]), 2, 2, 2, 5)
	var original: int = simulation.get_hovered_edge(-100, 0)
	var reserved := await finish_request(simulation.change_road_bus_lanes(original, 2, 2))
	expect(reserved.get("committed", false), "Reserve lanes before topology splitting")
	await build(PackedVector3Array([ground(0, -150), ground(0, 150)]), 1, 1, 0, 0)
	var before := roadbed_digest()
	for x in [-100.0, 100.0]:
		var edge: int = simulation.get_hovered_edge(x, 0)
		var split: Dictionary = simulation.get_road_cross_section(edge)
		expect(split.get("bus_forward_mask", 0) == 2 and split.get("bus_backward_mask", 0) == 2, "Both split halves preserve bus permissions")
		var refused := await finish_request(simulation.change_road_bus_lanes(edge, 4, 0))
		expect(not refused.get("committed", true), "Reject nonexistent bus lane")
		var result := await finish_request(simulation.change_road_bus_lanes(edge, 2, 2))
		expect(result.get("committed", false), "Reserve both outer lanes: %s" % result)
		var section: Dictionary = simulation.get_road_cross_section(edge)
		expect(section.get("bus_forward_mask", 0) == 2 and section.get("bus_backward_mask", 0) == 2, "Masks cross the bridge")
	expect(roadbed_digest() == before, "Lane use preserves the solved roadbed")
	var save := "/private/tmp/roads-bs-lane-bridge.sqlite"
	expect(simulation.save_game(save), "Save bus lane permissions")
	expect(simulation.load_game(save), "Load bus lane permissions")
	DirAccess.remove_absolute(save)
	var section: Dictionary = simulation.get_road_cross_section(simulation.get_hovered_edge(-100, 0))
	expect(section.get("bus_forward_mask", 0) == 2, "Bus permissions survive load")
	expect(roadbed_digest() == before, "Bus save keeps identical roadbed")
	if main != null:
		main.get_node("InputManager")._refresh_after_world_load()

func check_lane_controls() -> void:
	var controls: Node = load("res://scripts/ui/road_lane_use_controls.gd").new()
	root.add_child(controls)
	controls.setup(simulation, null)
	var edge: int = simulation.get_hovered_edge(-100, 0)
	controls.show_edge(edge)
	for enabled in [true, false]:
		var check: CheckBox = controls.get_child(0)
		check.button_pressed = enabled
		var deadline := Time.get_ticks_msec() + 60000
		while controls.busy and Time.get_ticks_msec() < deadline:
			await process_frame
		expect(not controls.busy, "Lane checkbox completes its road transaction")
		var section: Dictionary = simulation.get_road_cross_section(edge)
		expect(section.get("bus_forward_mask", -1) == (3 if enabled else 2), "Checkbox changes saved lane permission")
	controls.show_edge(-1)
	controls.queue_free()
	await process_frame

func check_bus_traffic() -> void:
	var goals := {}
	var reached := {}
	var saw_car := false
	simulation.set_camera_aabb(-500, 500, -500, 500)
	simulation.set_simulation_speed(4)
	for destination in [ground(230, 0), ground(0, 150), ground(0, -150)]:
		var id: int = simulation.spawn_debug_car(ground(-230, 0), destination)
		expect(id >= 0, "Left, through and right car routes remain legal")
		goals[id] = destination
		await create_timer(0.8).timeout
	var deadline := Time.get_ticks_msec() + 45000
	while reached.size() < goals.size() and Time.get_ticks_msec() < deadline:
		await process_frame
		var buckets: Dictionary = simulation.get_car_render_data()
		for bucket: Dictionary in buckets.values():
			var ids: PackedInt64Array = bucket.ids
			var poses: PackedFloat32Array = bucket.transforms
			for i in ids.size():
				if not goals.has(ids[i]):
					continue
				var at := Vector3(poses[i * 12 + 3], poses[i * 12 + 7], poses[i * 12 + 11])
				if at.x > -190 and at.x < -40:
					saw_car = true
					expect(at.z < 6.0, "Cars stay out of the outer bus ribbon: %s" % at)
				if at.distance_to(goals[ids[i]]) < 22:
					reached[ids[i]] = true
	expect(saw_car, "Observe a car in the retained general lane")
	expect(reached.size() == goals.size(), "Every turn reaches its goal: %s" % reached)
	simulation.set_simulation_speed(0)

func capture_bus_lanes() -> void:
	output = ProjectSettings.globalize_path("res://../captures/bus_streetscape")
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
	expect(simulation.create_blank_world(1024, 1024, 4, 128, 0), "Fresh bus capture world")
	manager._refresh_after_world_load()
	manager.set_simulation_speed(0)
	camera.set_view_state(PackedFloat32Array([0, 0, 0, -0.65, -1.1, 180, 0]))
	await bus_fixture()
	await check_bus_traffic()
	await capture("prisma_bus_lanes_overhead", ground(0, 0), 180, -1.2)
	await capture("prisma_bus_lane_street", ground(-95, 7.75), 35, -0.12, -1.57)
	await capture("bus_lettering", ground(-100, 7.75), 32, -1.25, -1.57)
	expect(simulation.save_game(output.path_join("roads-bs-lanes.sqlite")), "Save the inspected bus fixture")
	main.queue_free()
	await process_frame
