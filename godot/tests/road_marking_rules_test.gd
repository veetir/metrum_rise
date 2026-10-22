# SPDX-License-Identifier: GPL-2.0-only

## ROAD-74/75 per-arm zebra persistence/undo and opt-in read-only imported-map captures.
## Prepare with tools/road_marking_capture_fixtures.py; METRUM_MARKINGS_CAPTURE=before|after
## writes only captures/road_marking_rules, with no writes to the source user saves.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	var phase := OS.get_environment("METRUM_MARKINGS_CAPTURE")
	if not phase.is_empty():
		await setup_capture_scene()
		output = ProjectSettings.globalize_path("res://../captures/road_marking_rules/" + phase)
		DirAccess.make_dir_recursive_absolute(output)
		var fixtures := ProjectSettings.globalize_path("res://../captures/road_marking_rules")
		var prisma := fixtures.path_join("roads-markings-prisma-" + phase + ".sqlite")
		if not simulation.load_game(prisma):
			expect(false, "Read Prisma local snapshot")
			quit(failures)
			return
		simulation.set_simulation_speed(0)
		main.get_node("InputManager")._refresh_after_world_load()
		await native_capture("prisma_overhead", Vector3(-1002.87, 100.44, 1817.89), 115.0, -1.44, -3.446)
		await native_capture("prisma_street", Vector3(-1002.87, 100.44, 1817.89), 45.0, -0.08, -3.446)
		expect(simulation.load_game(fixtures.path_join("roads-markings-niirala-" + phase + ".sqlite")), "Read Niirala snapshot")
		simulation.set_simulation_speed(0)
		main.get_node("InputManager")._refresh_after_world_load()
		await native_capture("niirala_overhead", Vector3(-77.09, 97.96, 926.68), 150.0, -1.44, 0.0)
		await native_capture("niirala_street", Vector3(-77.09, 97.96, 926.68), 40.0, -0.08, -0.93)
		if phase == "after":
			await capture_authoring()
		main.queue_free()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
		expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Marking rule world")
		simulation.set_simulation_speed(0)
		await build(PackedVector3Array([Vector3(-180, 0, 0), Vector3(180, 0, 0)]), 1, 1, 0, 0)
		await build(PackedVector3Array([Vector3(0, 0, -180), Vector3(0, 0, 180)]), 1, 1, 0, 0)
		var node: int = simulation.get_closest_node(Vector3.ZERO, 3.0)
		var edge: int = simulation.get_hovered_edge(-30, 0)
		expect(not simulation.has_crosswalk(node, edge), "New roads start without zebra")
		var mesh: Dictionary = simulation.get_road_mesh_data(true)
		for chunk: Dictionary in mesh.get("chunks", []):
			expect(chunk.get("marking_vertices", PackedVector3Array()).is_empty(), "Plain 1+1 roads have no arrows")
		var preview: Dictionary = simulation.preview_crosswalk(node, edge, true)
		expect(preview.get("is_valid", false), "Arm zebra preview compiles")
		expect(not simulation.has_crosswalk(node, edge), "Preview is immutable")
		simulation.set_crosswalk_override(node, edge, true)
		expect(simulation.has_crosswalk(node, edge), "Arm zebra added")
		simulation.set_crosswalk_override(node, edge, false)
		expect(not simulation.has_crosswalk(node, edge), "Arm zebra removed")
		expect(simulation.undo_action(), "Undo zebra removal")
		var undo_deadline := Time.get_ticks_msec() + 10000
		while not simulation.has_crosswalk(node, edge) and Time.get_ticks_msec() < undo_deadline:
			await process_frame
		expect(simulation.has_crosswalk(node, edge), "Undo restores zebra")
		var path := ProjectSettings.globalize_path("res://../captures/road_marking_rules/roads-markings-roundtrip.sqlite")
		expect(simulation.save_game(path), "Save own zebra fixture")
		expect(simulation.load_game(path), "Load own zebra fixture")
		node = simulation.get_closest_node(Vector3.ZERO, 3.0)
		edge = simulation.get_hovered_edge(-30, 0)
		expect(simulation.has_crosswalk(node, edge), "Saved zebra restored")
		simulation.queue_free()
	await process_frame
	quit(failures)

func capture_authoring() -> void:
	expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Authoring capture world")
	simulation.set_simulation_speed(0)
	main.get_node("InputManager")._refresh_after_world_load()
	await build(PackedVector3Array([Vector3(-180, 0, 0), Vector3(180, 0, 0)]), 1, 1, 2, 5)
	await build(PackedVector3Array([Vector3(0, 0, -150), Vector3(0, 0, 150)]), 1, 1, 0, 0)
	var node: int = simulation.get_closest_node(Vector3.ZERO, 3)
	var edge: int = simulation.get_hovered_edge(-30, 0)
	await native_capture("authored_unmarked", Vector3.ZERO, 85, -1.1, -0.8)
	var preview: Dictionary = simulation.preview_crosswalk(node, edge, true)
	expect(main.get_node("RoadTool").show_local_edit_preview(preview), "Install actual zebra/median/terrain preview")
	await native_capture("authored_preview", Vector3.ZERO, 85, -1.1, -0.8)
	expect(main.get_node("RoadTool").has_local_edit_preview(), "Zebra preview persists while Selection owns the edit")
	expect(not simulation.has_crosswalk(node, edge), "Rendered preview has no authoritative mutation")
	var road_tool: Node = main.get_node("RoadTool")
	var pending_patch := Vector2i(999, 999)
	road_tool._pocket_pending_patches[pending_patch] = true
	road_tool._pocket_patch_will_change(pending_patch)
	expect(not road_tool.has_local_edit_preview(), "Streaming invalidates Selection-owned zebra preview")
	expect(road_tool.show_local_edit_preview(preview), "Zebra preview can reinstall after streaming")
	road_tool._clear_preview_visual()
	simulation.set_crosswalk_override(node, edge, true)
	var selection: Node = main.get_node("SelectTool")
	selection._build_node_visuals(node)
	await native_capture("authored_added", Vector3.ZERO, 85, -1.1, -0.8)
	selection._clear_node_visuals()
	var car: int = simulation.spawn_debug_car(Vector3(-180, 0, 0), Vector3(180, 0, 0))
	expect(car >= 0, "Spawn traffic through marked junction")
	simulation.set_simulation_speed(4)
	var deadline := Time.get_ticks_msec() + 45000
	var crossed := false
	while Time.get_ticks_msec() < deadline and not crossed:
		await process_frame
		simulation.pick_debug_agent(Vector3(-180, 0, 0), 400)
		var agent: Dictionary = simulation.get_agent_inspection(car)
		if agent.get("active", false):
			crossed = (agent.position as Vector3).x > 30
	expect(crossed, "Car clears authored zebra junction")
	simulation.set_simulation_speed(0)
	await native_capture("authored_traffic_street", Vector3(0, 0, 0), 35, -0.08, -1.57)
