# SPDX-License-Identifier: GPL-2.0-only

## ROAD-92 presets clear incompatible medians and stale edit refusals; N is inert on walkways.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	await setup_capture_scene()
	var manager: Node = main.get_node("InputManager")
	var road: Node = main.get_node("RoadTool")
	expect(simulation.create_blank_world(512, 512, 4, 128, 0), "Walkway preset world")
	manager._refresh_after_world_load()
	simulation.set_simulation_speed(0)
	await build(PackedVector3Array([Vector3(-100, 0, -60), Vector3(100, 0, -60)]), 0, 0, 0, 0)
	manager._toggle_tool(manager.Tool.ROAD)
	road.set_median(2, 5)
	# Produce the actual edit refusal before switching presets.
	road.fwd_lanes = 0
	road.bkw_lanes = 0
	road.apply_cross_section_to_edge(0)
	var deadline := Time.get_ticks_msec() + 15000
	while road._edit_request_id > 0 and Time.get_ticks_msec() < deadline:
		await process_frame
	expect(road._edit_message == "A median needs vehicle lanes in both directions", "Reproduce the stale median refusal")
	manager._toggle_tool(manager.Tool.WALKWAY)
	expect(road.median_style == 0 and road._edit_message.is_empty(), "Walkway selection clears median and stale refusal")
	var n := InputEventKey.new()
	n.keycode = KEY_N
	n.pressed = true
	manager._unhandled_input(n)
	road.cycle_median()
	expect(road.median_style == 0 and road._edit_message.is_empty(), "N and direct median cycling do nothing without vehicle lanes")
	road.set_crossing_mode(0)
	road._begin_stroke(Vector3(-80, 0, 40), 0, Vector3.ZERO)
	road.set_scripted_pointer(true, Vector3(80, 0, 40))
	road._queue_preview_update()
	deadline = Time.get_ticks_msec() + 15000
	while Time.get_ticks_msec() < deadline:
		await process_frame
		if not road._preview_result_pending and not road._preview_update_pending:
			break
	expect(road.is_valid, "Walkway preview accepts the cleared native median")
	var capturing := OS.get_environment("METRUM_STRUCTURE_CAPTURE") == "1"
	if capturing:
		output = ProjectSettings.globalize_path("res://../captures/road88_92").path_join(OS.get_environment("METRUM_STRUCTURE_PHASE"))
		DirAccess.make_dir_recursive_absolute(output)
		await native_capture("walkway_preset", Vector3(0, 0, 40), 120, -0.8, -0.65)
	road.cancel_road()
	road.set_scripted_pointer(false)
	road.active = false
	manager._toggle_tool(manager.Tool.ROAD)
	expect(road._edit_message.is_empty(), "Returning to a vehicle preset keeps feedback clear")
	main.queue_free()
	await process_frame
	quit(failures)
