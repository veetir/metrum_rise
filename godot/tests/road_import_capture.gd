# SPDX-License-Identifier: GPL-2.0-only

## Read-only ROAD-76 import diagnostics using production terrain, road and light renderers.
## METRUM_IMPORT_CAPTURE_SAVE is never written; captures remain in the ignored task directory.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	var source := OS.get_environment("METRUM_IMPORT_CAPTURE_SAVE")
	if source.is_empty():
		expect(false, "Provide an explicit importer-owned save for inspection")
		quit(failures)
		return
	await setup_capture_scene()
	output = ProjectSettings.globalize_path("res://../captures/road_76/import_diagnostic")
	DirAccess.make_dir_recursive_absolute(output)
	if not simulation.load_game(source):
		expect(false, "Read imported checkpoint")
		quit(failures)
		return
	simulation.set_simulation_speed(0)
	main.get_node("InputManager")._refresh_after_world_load()
	for view: Dictionary in [
		{"name": "kys_overhead", "x": -73.19017, "z": 922.70374, "distance": 152.47514, "pitch": -0.596, "yaw": -1.94},
		{"name": "kys_street", "x": -73.19017, "z": 922.70374, "distance": 40.0, "pitch": -0.08, "yaw": -1.94},
		{"name": "bioteknia_overhead", "x": -778.56964, "z": 1471.5917, "distance": 105.0, "pitch": -1.1, "yaw": -2.933},
		{"name": "bioteknia_street", "x": -778.56964, "z": 1471.5917, "distance": 27.74849, "pitch": -0.23, "yaw": -2.933},
		{"name": "prisma_overhead", "x": -999.179, "z": 1829.554, "distance": 175.0, "pitch": -1.25, "yaw": -0.65},
		{"name": "niirala_overhead", "x": -50.0, "z": 1820.0, "distance": 200.0, "pitch": -1.4, "yaw": -0.65},
		{"name": "university_overhead", "x": -535.0, "z": 1245.0, "distance": 145.0, "pitch": -0.9, "yaw": -0.65},
		{"name": "roundabout_west", "x": 51.128, "z": 973.101, "distance": 90.0, "pitch": -1.2, "yaw": -0.65},
		{"name": "roundabout_east", "x": 745.712, "z": 1079.876, "distance": 85.0, "pitch": -1.2, "yaw": -0.65}]:
		var pivot := Vector3(view.x, simulation.get_height_at(Vector2(view.x, view.z)), view.z)
		await native_capture(view.name, pivot, view.distance, view.pitch, view.yaw)
	main.queue_free()
	await process_frame
	quit(failures)
