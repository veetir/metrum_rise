# SPDX-License-Identifier: GPL-2.0-only

## Deterministic PARK-01 visual fixture, launched only by tools/capture_parking.py in isolation.
## Copied residential meshes are explicitly placed scenery; no resident travel is simulated.
extends SceneTree

var main: Node
var sim: SimulationNode
var _failed := false

func _initialize() -> void:
	call_deferred("_run")

func _road(points: PackedVector3Array) -> void:
	var before := sim.get_network_render_generation()
	sim.add_road_with_snap(points, 1, 1, true)
	var deadline := Time.get_ticks_msec() + 20000
	while sim.get_network_render_generation() <= before and Time.get_ticks_msec() < deadline:
		await process_frame
	if sim.get_network_render_generation() <= before:
		push_error("Capture road failed")
		_failed = true

func _run() -> void:
	main = load("res://scenes/Main.tscn").instantiate()
	main.set_script(null)
	root.add_child(main)
	current_scene = main
	await process_frame
	main.get_node("MainUI").hide()
	var input: Node = main.get_node("InputManager")
	input.set_process(false)
	input.set_process_input(false)
	input.set_process_unhandled_input(false)
	sim = main.get_node("SimulationNode")
	if not sim.create_blank_world(768.0, 512.0, 2.0, 128.0, 0.0):
		quit(1)
		return
	input._refresh_after_world_load()
	sim.set_simulation_speed(0.0)
	sim.set_weather_automatic(false)
	sim.set_weather_scenario(0, 0.0)
	main.get_node("Buildings").reload_asset_packs()
	await _road(PackedVector3Array([Vector3(-240, 0, 0), Vector3(240, 0, 0)]))
	await _road(PackedVector3Array([Vector3(75, 0, 0), Vector3(75, 0, 125)]))
	# The roadbed is unchanged; only authored building anchors supply parking.
	var placements := [
		["neukku:building.residential.lamelli_8", -160.0, -24.0],
		["neukku:building.residential.pistetalo_12", -85.0, -24.0],
		["neukku:building.residential.lamelli_8", -10.0, -24.0],
		["kuopio:building.residential.house_1", -165.0, 24.0],
		["kuopio:building.residential.house_2", -125.0, 24.0],
		["kuopio:building.residential.house_3", -85.0, 24.0],
		["kuopio:building.residential.house_4", -45.0, 24.0],
		["parking:building.parking.small_lot", 145.0, -30.0],
	]
	var results := []
	for placement: Array in placements:
		var error: String = sim.place_service_building(placement[0], placement[1], placement[2])
		results.append({"asset": placement[0], "error": error})
		if not error.is_empty():
			push_error("Capture placement %s: %s" % [placement[0], error])
			_failed = true
	var count := sim.debug_fill_parking()
	if count <= 0:
		push_error("Capture fleet has no places")
		_failed = true
	var info: Dictionary = sim.get_parking_info()
	var output := OS.get_environment("METRUM_PARKING_CAPTURE_DIR")
	DirAccess.make_dir_recursive_absolute(output)
	var report := FileAccess.open(output.path_join("scene.json"), FileAccess.WRITE)
	report.store_string(JSON.stringify({"placements": results, "parking": info}, "  "))
	report.close()
	if _failed:
		quit(1)
		return
	if DisplayServer.get_name() == "headless":
		main.queue_free()
		await process_frame
		quit(0)
		return
	var camera: Camera3D = main.get_node("CameraNode")
	camera.set_process(false)
	camera.set_process_input(false)
	camera.set_process_unhandled_input(false)
	camera.far = 1800.0
	camera.fov = 62.0
	DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	DisplayServer.window_set_size(Vector2i(1600, 900))
	Engine.max_fps = 30
	var views := {
		"lot": [Vector3(155, 31, 40), Vector3(145, 0, -20)],
	}
	for hour in [9.0]:
		main.get_node("SceneLighting").pin_hour_of_day(hour)
		for view: String in views:
			camera.global_position = views[view][0]
			camera.look_at(views[view][1], Vector3.UP)
			var deadline := Time.get_ticks_msec() + 30000
			var stable := 0
			while stable < 20 and Time.get_ticks_msec() < deadline:
				await process_frame
				if main.get_node("Terrain").is_render_settled():
					stable += 1
				else:
					stable = 0
			if stable < 20:
				push_error("Capture terrain did not settle")
				quit(1)
				return
			await RenderingServer.frame_post_draw
			root.get_texture().get_image().save_png(output.path_join("%s-%02d00.png" % [view, int(hour)]))
	main.queue_free()
	await process_frame
	quit(0)
