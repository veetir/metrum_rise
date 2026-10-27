# SPDX-License-Identifier: GPL-2.0-only

## Read-only WATER-05 runtime check of original and repaired importer city copies.
## Pass both paths after --; roads are validated in memory and no saves are written.
extends SceneTree

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() != 2:
		push_error("Expected original and repaired city paths")
		quit(2)
		return
	var simulation := SimulationNode.new()
	root.add_child(simulation)
	var failures := 0
	var dry_height := 0.0
	for index in 2:
		if not simulation.load_game(ProjectSettings.globalize_path(args[index])):
			push_error("Cannot load WATER-05 city")
			quit(2)
			return
		simulation.set_simulation_speed(0.0)
		# Both sides of the old z=261.63 road-crop edge share the same lake plane.
		for point in [Vector2(-2000, 100), Vector2(-2000, 400)]:
			var level: float = simulation.get_camera_water_surface_at(point)
			if not is_finite(level) or absf(level-82.46) > 0.001:
				push_error("Water surface moved from the measured 82.41 m level")
				failures += 1
		var ground: float = simulation.get_height_at(Vector2(-2000, 100))
		if index == 1 and ground > 81.0:
			push_error("Uncropped lake bed must have real depth")
			failures += 1
		var current_dry: float = simulation.get_height_at(Vector2(-1000, 0))
		if index == 0:
			dry_height = current_dry
		elif current_dry != dry_height:
			push_error("Dry terrain changed")
			failures += 1
		var points := PackedVector3Array([Vector3(-2050, ground, 100), Vector3(-1950, ground, 100)])
		var result: Dictionary = simulation.validate_road_candidate_with_snap(points, 1, 1, true)
		if result.get("invalid_reason", "") != "water_requires_bridge":
			push_error("The lake must refuse an ordinary grounded road: %s" % result)
			failures += 1
		points = PackedVector3Array([Vector3(-1000, current_dry, 0), Vector3(-950, simulation.get_height_at(Vector2(-950, 0)), 0)])
		result = simulation.validate_road_candidate_with_snap(points, 1, 1, true)
		if result.get("invalid_reason", "") == "water_requires_bridge":
			push_error("Dry land must not be refused as water")
			failures += 1
	simulation.queue_free()
	await process_frame
	quit(failures)
