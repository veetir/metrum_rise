# SPDX-License-Identifier: GPL-2.0-only

## ROAD-91 crossing feedback and save-copy captures through the production road tool.
## METRUM_STRUCTURE_SAVE_DIR names task-owned copies; METRUM_STRUCTURE_CAPTURE enables PNGs.
extends "res://tests/road_filled_underpass_test.gd"

var road: Node
var report: Array = []
var capturing := false

func run() -> void:
	await setup_capture_scene()
	road = main.get_node("RoadTool")
	capturing = OS.get_environment("METRUM_STRUCTURE_CAPTURE") == "1"
	output = ProjectSettings.globalize_path("res://../captures/road88_92").path_join(OS.get_environment("METRUM_STRUCTURE_PHASE"))
	DirAccess.make_dir_recursive_absolute(output)
	var copies := OS.get_environment("METRUM_STRUCTURE_SAVE_DIR")
	if not copies.is_empty():
		expect(simulation.load_game(copies.path_join("tunnel-over.sqlite")), "Load task-owned tunnel-over copy")
		main.get_node("InputManager")._refresh_after_world_load()
		simulation.set_simulation_speed(0)
		var view: PackedFloat32Array = camera.get_view_state()
		var pivot := Vector3(view[0], view[1], view[2])
		report.append({"saved_view": Array(view)})
		if capturing:
			await native_capture("tunnel-over_saved", pivot, view[5], view[4], view[3])
			await native_capture("tunnel-over_overhead", pivot, maxf(view[5], 150), -1.15, view[3])
			await native_capture("tunnel-over_street", pivot, 40, -0.08, view[3])
		# Repeat across the saved neighborhood: a refusal must never suppress feedback.
		for offset in [Vector3.ZERO, Vector3(100, 0, 100), Vector3(-100, 0, -100)]:
			for mode in [1, 2, 3]:
				var start: Vector3 = pivot + offset + Vector3(-20, 0, 0)
				var end: Vector3 = pivot + offset + Vector3(20, 0, 0)
				start.y = simulation.get_world_surface_height(Vector2(start.x, start.z))
				end.y = simulation.get_world_surface_height(Vector2(end.x, end.z))
				await feedback_stroke(mode, start, end, "saved_%d_%d" % [int(offset.x), mode])
	road.cancel_road()
	expect(simulation.create_blank_world(512, 512, 4, 128, 0), "Empty feedback world")
	main.get_node("InputManager")._refresh_after_world_load()
	simulation.set_simulation_speed(0)
	for mode in [0, 1, 2, 3]:
		await feedback_stroke(mode, Vector3(-40, 0, 0), Vector3(40, 0, 0), "empty_%d" % mode)
		if mode in [1, 2]:
			expect(not road.is_valid and road._crossing_refusal.get("invalid_reason") == "crossing_not_found", "No crossable road gives a refusal")
			expect(int(road._road_preview_material.get_shader_parameter("placement_state")) == 2, "Refused crossing is red")
			expect(road._info_label.text.contains("needs a road to cross"), "Refusal names the missing crossing")
		if capturing:
			await native_capture("empty_%d" % mode, Vector3.ZERO, 110, -0.8, -0.65)
	for reason in ["too_short", "invalid_cross_section", "crossing_not_found", "crossing_level_out_of_range", "crossing_approach_too_short", "grade_separation_clearance", "grade_separation_grade", "tunnel_clearance"]:
		expect(road._preview_invalid_text({"invalid_reason": reason}) != "Road cannot be placed", "Player-facing crossing refusal: " + reason)
	road.cancel_road()
	road.active = false
	var file := FileAccess.open(output.path_join("feedback.json"), FileAccess.WRITE)
	file.store_string(JSON.stringify(report, "\t"))
	main.queue_free()
	await process_frame
	quit(failures)

func feedback_stroke(mode: int, start: Vector3, end: Vector3, label: String) -> void:
	road.cancel_road()
	road.fwd_lanes = 1
	road.bkw_lanes = 1
	road.set_median(0, 0)
	road.set_crossing_mode(mode)
	road._begin_stroke(start, 0, Vector3.ZERO)
	road.set_scripted_pointer(true, end)
	road._queue_preview_update()
	var deadline := Time.get_ticks_msec() + 15000
	while Time.get_ticks_msec() < deadline:
		await process_frame
		if not road._preview_result_pending and not road._preview_update_pending:
			break
		# Missing ribbon generations used to retry forever, hiding both mesh and HUD.
		if not road._crossing_refusal.is_empty():
			break
	var visible: bool = road.blueprint_mesh.mesh != null or road._junction_preview.generation >= 0
	expect(visible, label + ": road preview is visible")
	expect(road._info_label.visible and not road._info_label.text.is_empty(), label + ": feedback label is visible")
	if not road.is_valid:
		expect(road._info_label.text != "Road cannot be placed", label + ": refusal has a player-facing reason")
	report.append({"case": label, "mode": mode, "valid": road.is_valid, "preview": visible,
		"message": road._info_label.text, "label_visible": road._info_label.visible, "refusal": road._crossing_refusal})
