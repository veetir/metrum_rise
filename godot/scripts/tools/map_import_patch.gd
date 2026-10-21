# SPDX-License-Identifier: GPL-2.0-only

## Executes offline-authored importer repairs and lane edits through acknowledged player APIs.
## Failed approach replacements undo the deletion before continuing; save outputs stay local.
extends "res://scripts/tools/map_import_apply.gd"

func _run() -> void:
	var args := _arguments()
	_output = _resolve_path(String(args["output"]))
	var local_maps := ProjectSettings.globalize_path("res://../maps/processed/").simplify_path()+"/"
	if not (_output+"/").begins_with(local_maps):
		_fail("Patch output must stay in maps/processed")
		return
	DirAccess.make_dir_recursive_absolute(_output)
	var simulation := SimulationNode.new()
	root.add_child(simulation)
	simulation.load_all_asset_packs(_resolve_path(String(args.get("packs", "res://bootstrap/mods"))))
	if not simulation.load_game(_resolve_path(args["base"])):
		_fail("Patch base failed to load")
		return
	simulation.set_simulation_speed(0.0)
	Engine.max_fps = 120
	_results = {"actions": [], "saved": false}
	var actions: Array = []
	if args.has("actions"):
		var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(_resolve_path(args["actions"])))
		actions = data.get("actions", [])
	for action in actions:
		var result := {"action": action}
		var kind: String = action["kind"]
		if kind == "replace":
			result.merge(await _replace(simulation, action))
		elif kind == "pocket":
			var request: int = simulation.change_road_pocket(int(action["edge"]), action["at_start"], action["left"], action["right"], action["pocket_m"], action["taper_m"])
			result.merge(await _finish_request(simulation, request))
		elif kind == "section":
			simulation.set_road_tool_median(int(action["median_style"]), action["median_width_m"])
			result.merge(await _finish_request(simulation, simulation.change_road_cross_section(int(action["edge"]), int(action["fwd_lanes"]), int(action["bkw_lanes"]))))
		elif kind == "road":
			var record := {"attempts": []}
			var stroke: Dictionary = action.duplicate(true)
			stroke["id"] = "source_patch"
			for variant in action.get("variants", [action]):
				stroke.merge(variant, true)
				result["reason"] = await _attempt(simulation, stroke, variant["points_xz"], record, "source_patch")
				if result["reason"].is_empty():
					break
			result["attempts"] = record["attempts"]
		elif kind == "clear_lane_rules":
			simulation.clear_lane_connections(int(action["node"]))
			result["committed"] = true
		elif kind == "lane_rule":
			var sources := {}
			for rule in simulation.get_lane_connections_array(int(action["node"])):
				sources[Vector2i(rule["from_edge"], rule["from_lane"])] = true
			var ordered: Array = sources.keys()
			ordered.sort_custom(func(a, b): return a.x < b.x if a.x != b.x else a.y < b.y)
			for source in ordered:
				simulation.clear_lane_source(int(action["node"]), source.x, source.y)
			for rule in action["rules"]:
				simulation.set_lane_connection(int(action["node"]), int(rule[0]), int(rule[1]), int(rule[2]), int(rule[3]))
			result["committed"] = true
		_results["actions"].append(result)
		_write_results()
		if _results["actions"].size() % 10 == 0:
			simulation.save_game(_output.path_join("checkpoint.sqlite"))
	_results["snapshot"] = JSON.parse_string(simulation.get_road_terrain_benchmark_snapshot())
	_results["node_lanes"] = {}
	for edge in _results["snapshot"]["edges"]:
		for key in ["start_node", "end_node"]:
			var node := int(edge[key])
			if not _results["node_lanes"].has(node):
				_results["node_lanes"][node] = simulation.get_node_lanes(node)
	_results["saved"] = simulation.save_game(_output.path_join("city.sqlite"))
	_write_results()
	quit(0 if _results["saved"] else 2)

func _finish_request(simulation: Node, request: int) -> Dictionary:
	var deadline := Time.get_ticks_msec()+120000
	while Time.get_ticks_msec() < deadline:
		var result: Variant = simulation.get_road_commit_result(request)
		if result is Dictionary:
			return result
		await process_frame
	_fail("Patch acknowledgement timed out")
	return {"reason": "timeout", "committed": false}

func _replace(simulation: Node, action: Dictionary) -> Dictionary:
	var edge := int(action["edge"])
	var node := int(action["node"])
	if simulation.get_road_cross_section(edge).is_empty() or (action.get("require_dead_end", true) and simulation.get_node_connection_count(node) != 1):
		return {"reason": "endpoint_changed"}
	var current: Vector3 = simulation.get_node_pos(node)
	var expected: Array = action["point_xz"]
	if Vector2(current.x, current.z).distance_to(Vector2(expected[0], expected[1])) > 0.1:
		return {"reason": "endpoint_identity_changed"}
	var deleted := []
	var reason := ""
	for id in action.get("edges", [edge]):
		var old: PackedVector3Array = simulation.get_edge_geometry_3d(int(id))
		if old.size() < 2 or simulation.get_road_cross_section(int(id)).is_empty():
			reason = "edge_absent"
			break
		var probe := old[old.size()/2].lerp(old[old.size()/2-1], 0.5)
		var target: Dictionary = simulation.get_bulldoze_target_at(probe.x, probe.z)
		if int(target.get("id", -1)) != int(id):
			reason = "bulldoze_target_differs"
			break
		var deletion: Dictionary = simulation.bulldoze_at(probe.x, probe.z)
		if not deletion.get("queued", false):
			reason = "deletion_not_queued"
			break
		while not simulation.get_road_cross_section(int(id)).is_empty():
			await process_frame
		deleted.append(int(id))
	var record := {"attempts": []}
	var committed_roads := 0
	if reason.is_empty():
		for road in action.get("roads", [action]):
			var stroke: Dictionary = road.duplicate(true)
			stroke["id"] = "replacement/%d" % edge
			reason = await _attempt(simulation, stroke, road["points_xz"], record, "whole_approach")
			if not reason.is_empty():
				break
			committed_roads += 1
	if not reason.is_empty():
		# Undo commands share the simulation FIFO; additions precede the original deletions.
		for _index in committed_roads:
			simulation.undo_action()
		deleted.reverse()
		for id in deleted:
			simulation.undo_action()
			while simulation.get_road_cross_section(id).is_empty():
				await process_frame
	return {"reason": reason, "attempts": record["attempts"]}
