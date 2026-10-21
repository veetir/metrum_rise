# SPDX-License-Identifier: GPL-2.0-only

## Reconstructs the university crossing from a task-prefixed copy; never opens the player source.
## SOURCE_SAVE and OUTPUT_SAVE must both have roads-c- names. CAPTURE=1 uses production renderers.
extends "res://tests/road_filled_underpass_test.gd"

func run() -> void:
	var source := OS.get_environment("METRUM_ROADS_C2_SOURCE_SAVE")
	var destination := OS.get_environment("METRUM_ROADS_C2_OUTPUT_SAVE")
	if not source.get_file().begins_with("roads-c-") or not destination.get_file().begins_with("roads-c-"):
		expect(false, "Use a roads-c- copy and a new roads-c- result")
		quit(failures)
		return
	var capture_enabled := OS.get_environment("METRUM_ROADS_C2_CAPTURE") == "1"
	if capture_enabled:
		await setup_capture_scene()
	else:
		simulation = ClassDB.instantiate("SimulationNode")
		root.add_child(simulation)
		await process_frame
	expect(simulation.load_game(source), "Read isolated university copy")
	if failures > 0:
		quit(failures)
		return
	simulation.set_simulation_speed(0)
	if main != null:
		main.get_node("InputManager")._refresh_after_world_load()
	# The upstream source Bridge also needs rebuilding at grade: otherwise extending
	# its terminal deliberately invokes the legacy long bridge-transition profile.
	# Keep the distant Standard junction and use the normal Rust ground solver.
	var approach: PackedVector3Array = simulation.get_edge_geometry_3d(59)
	var points: PackedVector3Array = simulation.get_edge_geometry_3d(806)
	var bridge: PackedVector3Array = simulation.get_edge_geometry_3d(36)
	var continuation: PackedVector3Array = simulation.get_edge_geometry_3d(35)
	var path: PackedVector3Array = simulation.get_edge_geometry_3d(389)
	var removals := []
	for id in [806, 36, 35, 389, 31, 30, 32, 33, 34, 805, 59]:
		var geometry: PackedVector3Array = simulation.get_edge_geometry_3d(id)
		expect(geometry.size() > 2, "Original university owner %d" % id)
		if geometry.size() < 3:
			quit(failures)
			return
		removals.append({"edge": id, "point": geometry[geometry.size() / 2]})
	for i in range(1, bridge.size()):
		points.append(bridge[i])
	for i in range(1, continuation.size()):
		points.append(continuation[i])
	for i in points.size():
		points[i].y = simulation.get_height_at(Vector2(points[i].x, points[i].z)) + 0.15
	points[-1] = continuation[-1]
	for i in range(1, approach.size()):
		approach[i].y = simulation.get_height_at(Vector2(approach[i].x, approach[i].z)) + 0.15
	for item: Dictionary in removals:
		var p: Vector3 = item.point
		var deadline := Time.get_ticks_msec() + 60000
		var target: Dictionary = simulation.get_bulldoze_target_at(p.x, p.z)
		while not target.get("valid", false) and Time.get_ticks_msec() < deadline:
			await process_frame
			target = simulation.get_bulldoze_target_at(p.x, p.z)
		expect(target.get("id", -1) == item.edge, "Pick original owner %d" % item.edge)
		if target.get("id", -1) != item.edge:
			quit(failures)
			return
		var removal: Dictionary = simulation.bulldoze_at(p.x, p.z)
		while not removal.get("queued", false) and removal.get("reason", "") == "simulation busy" and Time.get_ticks_msec() < deadline:
			await process_frame
			removal = simulation.bulldoze_at(p.x, p.z)
		expect(removal.get("queued", false), "Remove original owner: %s" % removal)
		if not removal.get("queued", false):
			quit(failures)
			return
		while not simulation.get_road_cross_section(item.edge).is_empty() and Time.get_ticks_msec() < deadline:
			await process_frame
		expect(simulation.get_road_cross_section(item.edge).is_empty(), "Removal completes")
	simulation.set_road_tool_crossing_level(0)
	if not await build(approach, 3, 3, 0, 0):
		expect(false, "Grounded upstream approach")
		quit(failures)
		return
	points[0] = simulation.get_node_pos(simulation.get_closest_node(approach[-1], 3.0))
	# Solve the ground alignment before adding the separate footpath. Over then raises
	# only the short clearance envelope; the remainder stays on its ground profile.
	var grounded: Dictionary = simulation.validate_road_candidate_with_snap(points, 1, 1, false)
	var joint_start := points[0]
	points = grounded.get("prepared_points", points)
	# The ground sampler can differ from a retained junction by millimetres. Keep
	# the exact physical joints so Over does not try to clear the roads it joins.
	points[0] = joint_start
	points[-1] = continuation[-1]
	if not await build(path, 0, 0, 0, 0):
		quit(failures)
		return
	var over: Dictionary = simulation.prepare_road_crossing(points, 1, 1, true)
	expect(over.get("is_valid", false), "University Over profile: " + str(over.get("invalid_reason", "")))
	if not over.get("is_valid", false):
		quit(failures)
		return
	points = over.points
	simulation.set_road_tool_crossing_level(over.crossing_level)
	if not await build(points, 1, 1, 0, 0):
		expect(false, "Ground-following main road with short box opening")
		quit(failures)
		return
	# The existing parking endpoint reaches the ground-level road through an ordinary
	# T junction. A 2 m paved refuge and requested zebra reuse the existing median tools.
	var access_end := points[0]
	for p: Vector3 in points:
		if Vector2(p.x + 502.0, p.z - 1206.7).length_squared() < Vector2(access_end.x + 502.0, access_end.z - 1206.7).length_squared():
			access_end = p
	var access := PackedVector3Array([Vector3(-534.482, 85.3693, 1186.639), access_end])
	# Median end setbacks bound the island within this short access arm.
	if not await build(access, 1, 1, 3, 2.0):
		expect(false, "At-grade parking access reconstruction")
		quit(failures)
		return
	var access_node: int = simulation.get_closest_node(access[-1], 8.0)
	var access_edge: int = simulation.get_hovered_edge(-511.0, 1203.0)
	expect(access_node >= 0 and access_edge >= 0, "Parking access joins the main road at grade")
	if access_node >= 0 and access_edge >= 0:
		simulation.set_crosswalk_override(access_node, access_edge, true)
		expect(simulation.has_crosswalk(access_node, access_edge), "Parking access zebra and refuge")
	if failures == 0:
		var position := access[0]
		var car: int = simulation.spawn_debug_car(position, continuation[-1])
		expect(car >= 0, "Parking access routes onto Yliopistonranta")
		var joined := false
		var drive_deadline := Time.get_ticks_msec() + 15000
		simulation.set_simulation_speed(4.0)
		while car >= 0 and not joined and Time.get_ticks_msec() < drive_deadline:
			await process_frame
			if simulation.pick_debug_agent(position, 35.0) == car:
				var details: Dictionary = simulation.get_agent_inspection(car)
				if details.get("active", false):
					position = details.position
					joined = Vector2(position.x - access_end.x, position.z - access_end.z).length() < 12.0
					if joined:
						expect(absf(position.y - access_end.y) < 1.0, "Parking car reaches the junction at main-road height")
		simulation.set_simulation_speed(0.0)
		expect(joined, "Parking traffic drives through the at-grade junction")
	if capture_enabled and failures == 0:
		main.get_node("RoadTool").update_main_mesh()
		await native_capture("university_overhead", Vector3(-535, 84, 1245), 145, -0.9, -0.65)
		await native_capture("university_street", Vector3(-548.65, 83.74, 1260.38), 27, -0.08, -2.3)
		await native_capture("university_level_junction", Vector3(-502.0, 87.1, 1206.7), 38, -0.3, 0.8)
		camera.set_view_state(PackedFloat32Array([-548.65, 84.0, 1260.38, -1.76, -0.4, 65.0, 0.0]))
	if failures == 0:
		var main_edge: int = simulation.get_hovered_edge(-600, 1312)
		expect(simulation.get_edge_class(main_edge) == 0, "Yliopistonranta uses a grounded mixed cut/fill corridor")
		expect(simulation.get_edge_class(access_edge) == 0, "Parking access is an ordinary surface road")
		expect(simulation.get_road_cross_section(main_edge).get("crossing_level", -2) == 1, "Ground-following main-road level")
		expect(simulation.get_road_cross_section(access_edge).get("crossing_level", -2) == 1, "Parking access shares the main-road level")
		var digest := university_contract_digest()
		expect(simulation.save_game(destination), "Write new university reconstruction")
		expect(simulation.load_game(destination), "Reload new university reconstruction")
		expect(university_contract_digest() == digest, "University profiles, cross sections and support meshes survive reload exactly")
	if main != null:
		main.queue_free()
	else:
		simulation.queue_free()
	await process_frame
	quit(failures)

func university_contract_digest() -> String:
	# Save compaction changes numeric owner IDs. Compare authoritative road records as a
	# sorted multiset, and the complete concrete/grass buffers, independently of those IDs.
	var profiles := []
	for id in simulation.get_road_benchmark_state().edge_slots:
		var section: Dictionary = simulation.get_road_cross_section(id)
		if section.is_empty():
			continue
		profiles.append(digest_value([simulation.get_edge_geometry_3d(id), section, simulation.get_edge_class(id), simulation.get_no_building_spawn(id)]))
	profiles.sort()
	var chunks: Array = simulation.get_road_mesh_data(true).get("chunks", [])
	chunks.sort_custom(func(a: Dictionary, b: Dictionary) -> bool: return Vector2i(a.chunk_x, a.chunk_z) < Vector2i(b.chunk_x, b.chunk_z))
	var support := []
	for chunk: Dictionary in chunks:
		support.append([chunk.chunk_x, chunk.chunk_z])
		var keys: Array = chunk.keys()
		keys.sort()
		for key: String in keys:
			if key.begins_with("concrete_") or key.begins_with("median_grass_"):
				support.append([key, chunk[key]])
	return digest_value([profiles, support])

func digest_value(value: Variant) -> String:
	var context := HashingContext.new()
	context.start(HashingContext.HASH_SHA256)
	context.update(var_to_bytes(value))
	return context.finish().hex_encode()
