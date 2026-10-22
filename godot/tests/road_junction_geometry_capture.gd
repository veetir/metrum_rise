# SPDX-License-Identifier: GPL-2.0-only

## Read-only ROAD-73 imported junction captures using the production renderers.
## METRUM_JUNCTION_SAVE is never written; output is confined to ignored captures/.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	await setup_capture_scene()
	var phase := OS.get_environment("METRUM_JUNCTION_PHASE")
	output = ProjectSettings.globalize_path("res://../captures/road_junction_geometry/" + phase)
	DirAccess.make_dir_recursive_absolute(output)
	var source := OS.get_environment("METRUM_JUNCTION_SAVE")
	if not simulation.load_game(source):
		expect(false, "Read imported junction save")
		quit(failures)
		return
	simulation.set_simulation_speed(0)
	main.get_node("InputManager")._refresh_after_world_load()
	if source.get_file().contains("legacy"):
		var car: int = simulation.spawn_debug_car(Vector3(-190,0,0),Vector3(190,0,0))
		expect(car >= 0, "Legacy comparison accepts traffic")
		simulation.set_simulation_speed(4)
		var deadline := Time.get_ticks_msec() + 30000
		var crossed := false
		while car >= 0 and not crossed and Time.get_ticks_msec() < deadline:
			await process_frame
			simulation.pick_debug_agent(Vector3.ZERO,400)
			var agent: Dictionary = simulation.get_agent_inspection(car)
			if agent.get("active",false):
				crossed = (agent.position as Vector3).x > 25
		expect(crossed, "Legacy traffic clears the junction")
		simulation.set_simulation_speed(0)
		await native_capture("legacy_overhead", Vector3.ZERO,100,-1.0,-0.65)
		await native_capture("legacy_street", Vector3.ZERO,30,-0.12,-0.65)
	elif source.get_file().contains("kys-slip"):
		await native_capture("kys_slip_overhead", Vector3(-41.53546,97.88994,920.85071),45,-0.85,-1.94)
		await native_capture("kys_slip_street", Vector3(-41.53546,97.88994,920.85071),22,-0.12,-1.94)
	elif source.get_file().contains("bioteknia"):
		await native_capture("bioteknia_original", Vector3(-778.56964,98.94254,1471.5917),27.74849,-0.23,-2.933)
		await native_capture("bioteknia_overhead", Vector3(-778.56964,98.94254,1471.5917),105,-1.1,-2.933)
	elif source.get_file().contains("curve"):
		var pivot := Vector3(1205.44,85.82,3415.43)
		var label := "south_curve"
		if source.get_file().contains("west"):
			pivot = Vector3(-1546.87,138.54,2281.71)
			label = "west_curve"
		elif source.get_file().contains("east"):
			pivot = Vector3(803.42,108.63,1060.13)
			label = "east_curve"
		await native_capture(label + "_overhead", pivot,70,-0.95,-0.65)
		await road_street_capture(label + "_street", pivot)
	else:
		await native_capture("kys_original", Vector3(-73.19017,98.29294,922.70374),152.47514,-0.596,-1.94)
		await native_capture("kys_street", Vector3(-73.19017,98.29294,922.70374),40,-0.08,-1.94)
	main.queue_free()
	await process_frame
	quit(failures)

func road_street_capture(label: String, probe: Vector3) -> void:
	var edge: int = simulation.get_hovered_edge(probe.x, probe.z)
	var points: PackedVector3Array = simulation.get_edge_geometry_3d(edge)
	expect(points.size() >= 2, "Street capture has a source road")
	if points.size() < 2:
		return
	var nearest := 0
	for i in points.size():
		if points[i].distance_squared_to(probe) < points[nearest].distance_squared_to(probe):
			nearest = i
	var eye_index := maxi(0, nearest - 4)
	var target_index := mini(points.size() - 1, nearest + 3)
	var target := points[target_index]
	var offset := points[eye_index] - target
	var horizontal := Vector2(offset.x, offset.z).length()
	expect(horizontal > 1.0, "Street camera has a viewing baseline")
	if horizontal > 1.0:
		# Keep the eye on the source road, rather than in the adjoining tall grass.
		await native_capture(label, target, horizontal / cos(0.12), -0.12, atan2(offset.x, offset.z))
