# SPDX-License-Identifier: GPL-2.0-only

## Read-only ROAD-81 import diagnostics using production terrain, road and light renderers.
## METRUM_IMPORT_CAPTURE_SAVE is never written; captures remain in the ignored task directory.
## METRUM_IMPORT_CAPTURE_VIEW_FILE optionally supplies exact cameras extracted from read-only saves.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	var source := OS.get_environment("METRUM_IMPORT_CAPTURE_SAVE")
	if source.is_empty():
		expect(false, "Provide an explicit importer-owned save for inspection")
		quit(failures)
		return
	await setup_capture_scene()
	output = OS.get_environment("METRUM_IMPORT_CAPTURE_OUTPUT")
	if output.is_empty():
		output = ProjectSettings.globalize_path("res://../captures/road_76b/import")
	DirAccess.make_dir_recursive_absolute(output)
	if not simulation.load_game(source):
		expect(false, "Read imported checkpoint")
		quit(failures)
		return
	simulation.set_simulation_speed(0)
	main.get_node("InputManager")._refresh_after_world_load()
	var views: Array = [
		{"name": "kys_user_view", "x": -73.19017, "y": 98.29294, "z": 922.70374, "distance": 110.0, "pitch": -1.05, "yaw": -1.94},
		{"name": "kys_overhead", "x": -73.19017, "z": 922.70374, "distance": 152.47514, "pitch": -0.596, "yaw": -1.94},
		{"name": "kys_street", "x": -73.19017, "z": 922.70374, "distance": 40.0, "pitch": -0.08, "yaw": -1.94},
		{"name": "bioteknia_overhead", "x": -778.56964, "z": 1471.5917, "distance": 105.0, "pitch": -1.1, "yaw": -2.933},
		{"name": "bioteknia_street", "x": -778.56964, "z": 1471.5917, "distance": 27.74849, "pitch": -0.23, "yaw": -2.933},
		{"name": "prisma_overhead", "x": -999.179, "z": 1829.554, "distance": 175.0, "pitch": -1.25, "yaw": -0.65},
		{"name": "prisma_street", "x": -999.179, "z": 1829.554, "distance": 35.0, "pitch": -0.12, "yaw": -3.446},
		{"name": "niirala_overhead", "x": 176.207, "y": 120.997, "z": 1548.486, "distance": 200.0, "pitch": -1.4, "yaw": -0.65},
		{"name": "niirala_street", "x": 176.207, "y": 120.997, "z": 1548.486, "distance": 35.0, "pitch": -0.1, "yaw": 0.65},
		{"name": "neulamaki_overhead", "x": -1239.476, "z": 1851.525, "distance": 170.0, "pitch": -1.2, "yaw": -0.65},
		{"name": "university_overhead", "x": -535.0, "z": 1245.0, "distance": 145.0, "pitch": -0.9, "yaw": -0.65},
		{"name": "university_underpass", "x": -548.65, "z": 1260.38, "distance": 27.0, "pitch": -0.08, "yaw": -2.3},
		{"name": "university_parking", "x": -502.0, "z": 1206.7, "distance": 38.0, "pitch": -0.3, "yaw": 0.8},
		{"name": "luola_roundabout", "x": -1342.151, "z": 1709.442, "distance": 100.0, "pitch": -1.15, "yaw": -0.65},
		{"name": "roundabout_west", "x": 51.128, "z": 973.101, "distance": 90.0, "pitch": -1.2, "yaw": -0.65},
		{"name": "roundabout_east", "x": 745.712, "z": 1079.876, "distance": 85.0, "pitch": -1.2, "yaw": -0.65}]
	var views_file := OS.get_environment("METRUM_IMPORT_CAPTURE_VIEW_FILE")
	if not views_file.is_empty():
		var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(views_file))
		if not parsed is Array:
			expect(false, "Camera manifest must contain an array")
			main.queue_free()
			await process_frame
			quit(failures)
			return
		views = parsed
	for view: Dictionary in views:
		var selected := OS.get_environment("METRUM_IMPORT_CAPTURE_VIEWS").split(",", false)
		if not selected.is_empty() and not String(view.name) in selected:
			continue
		var pivot := Vector3(view.x, view.get("y", simulation.get_height_at(Vector2(view.x, view.z))), view.z)
		await native_capture(view.name, pivot, view.distance, view.pitch, view.yaw)
		var water: Node = main.get_node("Water")
		var water_deadline := Time.get_ticks_msec() + 30000
		while water.has_pending_render_work(false) and Time.get_ticks_msec() < water_deadline:
			await process_frame
		var terrain: Node = main.get_node("Terrain")
		var half: Vector2 = terrain.terrain_world_size * 0.5
		var keys := PackedInt32Array()
		for z in range(floori((pivot.z-view.distance+half.y)/terrain.patch_span_m), ceili((pivot.z+view.distance+half.y)/terrain.patch_span_m)):
			for x in range(floori((pivot.x-view.distance+half.x)/terrain.patch_span_m), ceili((pivot.x+view.distance+half.x)/terrain.patch_span_m)):
				keys.append(x)
				keys.append(z)
		var diagnostics := FileAccess.open(output.path_join(view.name + "_water.json"), FileAccess.WRITE)
		diagnostics.store_string(JSON.stringify({"pending": water.get_pending_render_work_counts(),
			"patches": water.road_geometry_debug_patch_lines(keys)}, "\t"))
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(output.path_join(view.name + ".png"))
	var manifest := FileAccess.open(output.path_join("source.json"), FileAccess.WRITE)
	manifest.store_string(JSON.stringify({"save": source.get_file(), "save_sha256": FileAccess.get_sha256(source),
		"origin": OS.get_environment("METRUM_IMPORT_ORIGIN"),
		"native_library_sha256": FileAccess.get_sha256("res://bin/libmetrum_rise.dylib" if OS.has_feature("macos") else "res://bin/libmetrum_rise.so"),
		"camera_manifest_sha256": FileAccess.get_sha256(views_file) if not views_file.is_empty() else "built-in"}, "\t") + "\n")
	main.queue_free()
	await process_frame
	quit(failures)
