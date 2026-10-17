# SPDX-License-Identifier: GPL-2.0-only

## Applies an offline map plan using the player's road and zoning APIs, then saves a city.
## Standalone SceneTree script, run only through maps/processed/run_apply.sh. See
## tools/map_import/cli.py for command arguments. RoadCommit acknowledgements are awaited;
## unknown completion stops the entire apply so a late road cannot race a subsequent stroke.
## Source speeds and building footprints remain in neutral.json. Final save is paused.
extends SceneTree

const Network := preload("res://scripts/tools/map_import_network.gd")
var _output := ""
var _results: Dictionary = {}
var _fatal := false
var _deferred: Array = []
var _car_started := false
var _junction_slides: Array = [-12.0, -8.0, 8.0, 12.0]
var _junction_clearance := 12.0

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	var args := _arguments()
	_output = _resolve_path(String(args.get("output", "../maps/processed/niirala")))
	var plan_path := _resolve_path(String(args.get("plan", "../maps/processed/niirala/plan.json")))
	var world_path := _resolve_path(String(args.get("world", "bootstrap/worlds/kuopio_324km2_10m.sqlite")))
	# Restrict derived data and saves to the repository's local map output directory.
	var local_maps := ProjectSettings.globalize_path("res://../maps/processed/").simplify_path() + "/"
	_output = _output.simplify_path()
	if not (_output + "/").begins_with(local_maps):
		_fail("Output must be inside maps/processed")
		return
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(plan_path))
	if not parsed is Dictionary or int(parsed.get("schema_version", 0)) != 1:
		_fail("Invalid map plan")
		return
	var plan: Dictionary = parsed
	_junction_slides = plan["parameters"].get("junction_slide_m", _junction_slides)
	_junction_clearance = float(plan["parameters"].get("junction_clearance_m", _junction_clearance))
	if FileAccess.get_sha256(world_path) != String(plan["world_sha256"]):
		_fail("World definition hash differs from prepared plan")
		return
	_results = {"schema_version": 1, "plan_sha256": FileAccess.get_sha256(plan_path),
		"complete": false, "saved": false, "save_path": "", "roads": [], "zoning": [], "parcels": []}
	var simulation := SimulationNode.new()
	simulation.name = "SimulationNode"
	root.add_child(simulation)
	simulation.set_simulation_speed(0.0)
	var warnings: String = simulation.load_all_asset_packs(_resolve_path(String(args.get("packs", "res://bootstrap/mods"))))
	_results["asset_warnings"] = warnings
	if not simulation.load_world_definition(world_path):
		_fail("World definition failed to load")
		return
	simulation.set_simulation_speed(0.0)
	var world_size: Vector2 = simulation.get_terrain_world_size()
	var transform: Dictionary = plan["world_transform"]
	if absf(world_size.x - float(transform["max_e"]) + float(transform["min_e"])) > 0.01 or absf(world_size.y - float(transform["max_n"]) + float(transform["min_n"])) > 0.01:
		_fail("Loaded world dimensions differ from plan")
		return
	_results["forest"] = _clear_forest(simulation, plan["target_box_xz"])
	var accepted := {}
	var roads_repaired := false
	for stroke in plan["strokes"]:
		if stroke["kind"] == "footpath" and not roads_repaired:
			await _retry_deferred(simulation)
			await _repair_network(simulation)
			roads_repaired = true
		var record := {"id": stroke["id"], "attempts": [], "pieces": [], "failed_pieces": []}
		_results["roads"].append(record)
		await _place(simulation, stroke, stroke["points_xz"], record, 0, "original")
		if _fatal:
			return
		var accepted_m := 0.0
		for piece in record["pieces"]:
			accepted_m += _length(piece)
		record["accepted_length_m"] = accepted_m
		record["status"] = "refused" if record["pieces"].is_empty() else ("partial" if not record["failed_pieces"].is_empty() else "accepted")
		record["reason"] = String(record["attempts"][-1]["reason"]) if record["status"] == "refused" else ""
		accepted[stroke["id"]] = accepted_m > 0.0
		if _results["roads"].size() % 10 == 0:
			_write_results()
	if not roads_repaired:
		await _retry_deferred(simulation)
		await _repair_network(simulation)
	for record in _results["roads"]:
		_finish_record(record)
		accepted[record["id"]] = record["accepted_length_m"] > 0.0
	_results["gateways"] = _create_gateway(simulation, plan["target_box_xz"])
	_results["network"] = Network.inspect_network(simulation)
	_results["car_components"] = _results["network"]["components"]
	var profiles: Array = simulation.get_zone_profiles()
	profiles.sort_custom(func(a, b): return String(a["id"]) < String(b["id"]))
	var profile_ids := {}
	var profile_types := {}
	for profile in profiles:
		var key := String(profile["zone_type"]) + "/" + String(profile["density"])
		if not profile_ids.has(key):
			profile_ids[key] = int(profile["runtime_id"])
		profile_types[int(profile["runtime_id"])] = String(profile["zone_type"])
	for zone in plan["zoning"]:
		_results["zoning"].append(_apply_zone(simulation, zone, accepted, profile_ids))
	# One final scan of the exported parcels, never one resident scan per intent.
	for parcel in simulation.get_zoning_parcels_overlay():
		var corners: PackedVector3Array = parcel["corners"]
		var twice_area := 0.0
		for i in corners.size():
			var a := corners[i]
			var b := corners[(i + 1) % corners.size()]
			twice_area += a.x * b.z - a.z * b.x
		_results["parcels"].append({"id": int(parcel["id"]),
			"zone_type": profile_types.get(int(parcel["profile_runtime_id"]), "unknown"), "area_m2": absf(twice_area) / 2.0})
	_results["parcels"].sort_custom(func(a, b): return int(a["id"]) < int(b["id"]))
	_results["growth"] = await _grow(simulation, args)
	if _fatal:
		return
	if DirAccess.make_dir_recursive_absolute(_output) != OK:
		_fail("Cannot create save directory")
		return
	var save_path := _output.path_join("city.sqlite")
	_results["saved"] = simulation.save_game(save_path)
	_results["save_path"] = save_path if _results["saved"] else ""
	_results["complete"] = true
	if not _write_results():
		quit(2)
		return
	quit(0 if _results["saved"] else 2)

func _create_gateway(simulation: Node, box: Array) -> Array:
	var snapshot: Dictionary = JSON.parse_string(simulation.get_road_terrain_benchmark_snapshot())
	var degree := {}
	for edge in snapshot["edges"]:
		if int(edge["fwd_lanes"]) + int(edge["bkw_lanes"]) == 0:
			continue
		for field in ["start_node", "end_node"]:
			var node := int(edge[field])
			degree[node] = int(degree.get(node, 0)) + 1
	var candidates := []
	for node in degree:
		if int(degree[node]) != 1:
			continue
		var p: Vector3 = simulation.get_node_pos(node)
		var margin := minf(minf(absf(p.x-box[0]), absf(p.x-box[2])), minf(absf(p.z-box[1]), absf(p.z-box[3])))
		if margin <= 15.0:
			candidates.append({"node": node, "point_xz": [p.x, p.z], "margin_m": margin})
	candidates.sort_custom(func(a, b): return float(a["margin_m"]) < float(b["margin_m"]) if not is_equal_approx(a["margin_m"], b["margin_m"]) else int(a["node"]) < int(b["node"]))
	if candidates.is_empty():
		return []
	# The crop boundary represents the unmodelled city beyond this import. Use
	# the existing outside-connection mutator, including its normal 10 m extension.
	var gateway: Dictionary = candidates[0]
	simulation.set_border_connection(int(gateway["node"]))
	return [gateway]

func _grow(simulation: Node, args: Dictionary) -> Dictionary:
	var days := clampi(int(args.get("grow-days", "0")), 0, 30)
	var limit_ms := clampi(int(args.get("grow-max-seconds", "180")), 1, 1800) * 1000
	var frame_limit := clampi(int(args.get("grow-max-frames", "120000")), 1, 1000000)
	var start_minutes: int = simulation.get_current_day()*1440 + simulation.get_current_minute_of_day()
	var begin := Time.get_ticks_msec()
	var frames := 0
	var speed := 0.0
	if days > 0:
		var steps: PackedFloat32Array = SimulationNode.get_simulation_speed_steps()
		speed = steps[-1]
		if not simulation.set_simulation_speed(speed):
			_fail("Cannot set growth speed")
			return {}
		while frames < frame_limit and Time.get_ticks_msec()-begin < limit_ms:
			var minutes: int = simulation.get_current_day()*1440 + simulation.get_current_minute_of_day()
			if minutes-start_minutes >= days*1440:
				break
			frames += 1
			await process_frame
	simulation.set_simulation_speed(0.0)
	# The setter is queued. Confirm pause on the core before saving it.
	var pause_deadline := Time.get_ticks_msec() + 5000
	while float(simulation.get_road_benchmark_state()["simulation_speed"]) != 0.0:
		if Time.get_ticks_msec() > pause_deadline:
			_fail("Timed out waiting for final pause")
			return {}
		await process_frame
	var finish_minutes: int = simulation.get_current_day()*1440 + simulation.get_current_minute_of_day()
	var state: Dictionary = simulation.get_road_benchmark_state()
	return {"requested_days": days, "elapsed_minutes": finish_minutes-start_minutes, "frames": frames,
		"wall_seconds": (Time.get_ticks_msec()-begin)/1000.0, "speed": speed,
		"target_reached": finish_minutes-start_minutes >= days*1440,
		"agents": state["agents"], "buildings": state["buildings"], "paused": state["simulation_speed"] == 0.0}

func _repair_network(simulation: Node) -> void:
	_results["repairs"] = []
	var tried := {}
	for sweep in 6:
		var network := Network.inspect_network(simulation)
		if int(network["components"]) <= 1:
			break
		var joined := {}
		var successes := 0
		for candidate in network["candidates"]:
			var key := JSON.stringify(candidate["points_xz"])
			if tried.has(key) or joined.has(candidate["from_component"]) or joined.has(candidate["to_component"]):
				continue
			tried[key] = true
			var record := {"id": "repair/%d/%d" % [sweep, _results["repairs"].size()], "attempts": [], "pieces": [], "failed_pieces": [], "candidate": candidate}
			var stroke := {"fwd_lanes": 1, "bkw_lanes": 1, "id": record["id"]}
			var reason := await _attempt(simulation, stroke, candidate["points_xz"], record, "near_miss_connection")
			record["reason"] = reason
			record["status"] = "accepted" if reason.is_empty() else "refused"
			_results["repairs"].append(record)
			if _fatal:
				return
			if reason.is_empty():
				joined[candidate["from_component"]] = true
				joined[candidate["to_component"]] = true
				successes += 1
		if successes == 0:
			break

func _finish_record(record: Dictionary) -> void:
	var total := 0.0
	for piece in record["pieces"]:
		total += _length(piece)
	record["accepted_length_m"] = total
	record["status"] = "refused" if record["pieces"].is_empty() else ("partial" if not record["failed_pieces"].is_empty() else "accepted")
	record["reason"] = String(record["failed_pieces"][-1]["reason"]) if record["status"] == "refused" and not record["failed_pieces"].is_empty() else ""

func _retry_deferred(simulation: Node) -> void:
	for sweep in 8:
		var pending: Array = _deferred
		_deferred = []
		var before := 0
		var after := 0
		for entry in pending:
			var record: Dictionary = entry["record"]
			before += record["pieces"].size()
			record["failed_pieces"].erase(entry["failure"])
			await _place(simulation, entry["stroke"], entry["failure"]["points_xz"], record, 4, "deferred/%d" % sweep)
			after += record["pieces"].size()
			_finish_record(record)
			if _fatal:
				return
		_write_results()
		if after == before:
			break

func _clear_forest(simulation: Node, box: Array) -> Dictionary:
	var removed := 0
	var stamps := 0
	# Overlapping brush discs cover every point in the target square. Boundary
	# discs extend at most 42 m past the square; no global vegetation switch.
	var x := float(box[0]) + 100.0
	while x < float(box[2]):
		var z := float(box[1]) + 100.0
		while z < float(box[3]):
			removed += simulation.remove_vegetation_at(Vector2(x, z), 142.0, 0)
			stamps += 1
			z += 200.0
		x += 200.0
	return {"removed": removed, "stamps": stamps, "radius_m": 142.0, "target_box_xz": box}

func _length(points: Array) -> float:
	var total := 0.0
	for i in range(1, points.size()):
		total += Vector2(points[i][0] - points[i-1][0], points[i][1] - points[i-1][1]).length()
	return total

func _attempt(simulation: Node, stroke: Dictionary, points: Array, record: Dictionary, strategy: String) -> String:
	var xyz := []
	var placed := []
	var touches := false
	for i in points.size():
		var p: Array = points[i]
		var pos := Vector2(float(p[0]), float(p[1]))
		var world := Vector3(pos.x, simulation.get_world_surface_height(pos), pos.y)
		var snapped: Variant = simulation.get_closest_network_point(world, 8.0)
		touches = touches or snapped is Vector3
		if i == 0 or i == points.size()-1:
			# The commit's snap flag only enables 2 m graph matching. The player's
			# cursor also projects the clicked position onto a node/edge beforehand.
			if snapped is Vector3 and strategy != "water_unsnapped" and not strategy.begins_with("junction_slide/"):
				world = snapped
		xyz.append([world.x, world.y, world.z])
		placed.append([world.x, world.z])
	var compact := []
	var compact_xz := []
	for i in xyz.size():
		var p := Vector3(xyz[i][0], xyz[i][1], xyz[i][2])
		if not compact.is_empty():
			var last: Array = compact[-1]
			if p.distance_to(Vector3(last[0], last[1], last[2])) < 0.05:
				continue
		compact.append(xyz[i])
		compact_xz.append(placed[i])
	xyz = compact
	placed = compact_xz
	var degenerate := xyz.size() < 2 or _length(placed) < 2.0
	if not degenerate:
		degenerate = Vector2(placed[0][0], placed[0][1]).distance_to(Vector2(placed[-1][0], placed[-1][1])) < 0.05
	for i in range(1, placed.size()-1):
		var u := Vector2(placed[i][0]-placed[i-1][0], placed[i][1]-placed[i-1][1])
		var v := Vector2(placed[i+1][0]-placed[i][0], placed[i+1][1]-placed[i][1])
		if u.length_squared() > 0.0 and v.length_squared() > 0.0 and u.normalized().dot(v.normalized()) < -0.99999:
			degenerate = true
	var attempt: Dictionary = stroke.duplicate(true)
	attempt["points"] = xyz
	var outcome: Dictionary
	if degenerate:
		outcome = {"by": "importer", "reason": "degenerate_after_snap"}
	elif _car_started and int(stroke["fwd_lanes"]) + int(stroke["bkw_lanes"]) > 0 and not touches:
		outcome = {"by": "importer", "reason": "disconnected_candidate_deferred"}
	else:
		outcome = await _replay(simulation, attempt)
	var reason := String(outcome.get("reason", ""))
	record["attempts"].append({"strategy": strategy, "points_xz": placed, "reason": reason, "by": outcome["by"]})
	if reason == "replay_timeout" or reason == "validation_unavailable":
		_fatal = true
		_fail("Unknown road completion for %s: %s" % [stroke["id"], reason])
	return reason

func _replay(simulation: Node, attempt: Dictionary) -> Dictionary:
	var points := PackedVector3Array()
	for p in attempt["points"]:
		points.append(Vector3(p[0], p[1], p[2]))
	var deadline := Time.get_ticks_msec() + 120000
	var validation: Dictionary = {}
	while Time.get_ticks_msec() < deadline:
		validation = simulation.validate_road_candidate_for_commit_with_snap(points, int(attempt["fwd_lanes"]), int(attempt["bkw_lanes"]), true)
		if not validation.get("is_pending", false):
			break
		await process_frame
	if not validation.get("is_valid", false):
		return {"by": "tool", "reason": String(validation.get("invalid_reason", "validation_unavailable"))}
	var request: int = simulation.add_road_with_snap(points, int(attempt["fwd_lanes"]), int(attempt["bkw_lanes"]), true)
	# Wall-clock bound: headless frames can run much faster than the background
	# compiler, so a frame-only timeout loses live request acknowledgements.
	deadline = Time.get_ticks_msec() + 120000
	while Time.get_ticks_msec() < deadline:
		var result: Variant = simulation.get_road_commit_result(request)
		if result is Dictionary:
			if result.get("committed", false) and int(attempt["fwd_lanes"]) + int(attempt["bkw_lanes"]) > 0:
				var network := Network.inspect_network(simulation, 0.0)
				if int(network["components"]) > 1:
					# Undo this acknowledged edit before any other mutation. Detached
					# streets are deferred until their connecting street exists.
					if not simulation.undo_action():
						return {"by": "importer", "reason": "validation_unavailable"}
					return {"by": "importer", "reason": "disconnected_candidate_deferred"}
				_car_started = true
			return {"by": "commit", "reason": "" if result.get("committed", false) else String(result.get("reason", "commit_failed"))}
		await process_frame
	return {"by": "commit", "reason": "replay_timeout"}

func _split(points: Array, junctions: Array) -> Array:
	var pieces := []
	var start := 0
	for i in range(1, points.size()-1):
		if points[i] in junctions:
			pieces.append(points.slice(start, i+1))
			start = i
	if not pieces.is_empty():
		pieces.append(points.slice(start))
		return pieces
	var half := _length(points) / 2.0
	var station := 0.0
	for i in range(1, points.size()):
		var run := _length([points[i-1], points[i]])
		if station + run >= half and run > 0.0:
			var t := (half - station) / run
			var middle := [lerpf(points[i-1][0], points[i][0], t), lerpf(points[i-1][1], points[i][1], t)]
			var first: Array = points.slice(0, i)
			first.append(middle)
			var second: Array = [middle]
			second.append_array(points.slice(i))
			return [first, second]
		station += run
	return []

func _slide_candidates(simulation: Node, points: Array) -> Array:
	var candidates := []
	for endpoint in [0, points.size()-1]:
		var old_tip := Vector2(points[endpoint][0], points[endpoint][1])
		var hit: Variant = simulation.get_closest_network_point(Vector3(old_tip.x, 0.0, old_tip.y), 8.0)
		if not hit is Vector3:
			continue
		var snap: Vector3 = hit
		var edge_id: int = simulation.get_hovered_edge(snap.x, snap.z)
		if edge_id < 0:
			continue
		var geometry: PackedVector3Array = simulation.get_edge_geometry_3d(edge_id)
		var tangent := Vector2.ZERO
		var nearest := INF
		for i in range(1, geometry.size()):
			var a := Vector2(geometry[i-1].x, geometry[i-1].z)
			var b := Vector2(geometry[i].x, geometry[i].z)
			if a.distance_to(b) < 0.05:
				continue
			var q := Geometry2D.get_closest_point_to_segment(Vector2(snap.x, snap.z), a, b)
			var gap := q.distance_to(Vector2(snap.x, snap.z))
			if gap < nearest:
				nearest = gap
				tangent = (b-a).normalized()
		if tangent == Vector2.ZERO:
			continue
		for shift in _junction_slides:
			var probe := Vector2(snap.x, snap.z) + tangent * float(shift)
			var target: Variant = simulation.get_closest_network_point(Vector3(probe.x, 0.0, probe.y), 1.0)
			if not target is Vector3:
				continue
			var moved: Vector3 = target
			if Vector2(moved.x, moved.z).distance_to(old_tip) < 4.0:
				continue
			var adjusted: Array = points.duplicate(true)
			# Remove the old tip's short control segments, which would otherwise
			# turn an on-road junction slide into a sharp return to the old node.
			while adjusted.size() > 2:
				var neighbor := 1 if endpoint == 0 else adjusted.size()-2
				if old_tip.distance_to(Vector2(adjusted[neighbor][0], adjusted[neighbor][1])) >= _junction_clearance:
					break
				adjusted.remove_at(neighbor)
			var tip := 0 if endpoint == 0 else adjusted.size()-1
			adjusted[tip] = [moved.x, moved.z]
			candidates.append({"points_xz": adjusted, "strategy": "junction_slide/%d/%.1fm" % [endpoint, float(shift)]})
	return candidates

func _place(simulation: Node, stroke: Dictionary, points: Array, record: Dictionary, depth: int, strategy: String) -> void:
	if _fatal:
		return
	var reason := await _attempt(simulation, stroke, points, record, strategy)
	var original_reason := reason
	if _fatal:
		return
	if reason.is_empty():
		record["pieces"].append(record["attempts"][-1]["points_xz"])
		return
	if reason == "disconnected_candidate_deferred":
		var failure := {"points_xz": points, "reason": reason}
		record["failed_pieces"].append(failure)
		_deferred.append({"stroke": stroke, "record": record, "failure": failure})
		return
	if reason == "water_requires_bridge":
		reason = await _attempt(simulation, stroke, points, record, "water_unsnapped")
		if reason.is_empty():
			record["pieces"].append(record["attempts"][-1]["points_xz"])
			return
	# Split at known source junctions first; then bounded bisection isolates the
	# bad part without sacrificing an entire street. Maximum depth is four.
	if depth < 4 and _length(points) >= 35.0:
		for part in _split(points, stroke.get("junctions_xz", [])):
			await _place(simulation, stroke, part, record, depth+1, "split/%d" % (depth+1))
		return
	# Bounded endpoint retries: exact nearby network node first, then a short
	# endpoint trim. Every attempted geometry and validator result is retained.
	for adjustment in ["node", "trim3", "trim6"]:
		var adjusted: Array = points.duplicate(true)
		for endpoint in [0, adjusted.size()-1]:
			var p := Vector3(adjusted[endpoint][0], 0.0, adjusted[endpoint][1])
			if adjustment == "node":
				var node_id: int = simulation.get_closest_node(p, 18.0)
				if node_id >= 0:
					var node_pos: Vector3 = simulation.get_node_pos(node_id)
					adjusted[endpoint] = [node_pos.x, node_pos.z]
			else:
				var neighbor := 1 if endpoint == 0 else adjusted.size()-2
				var q := Vector2(adjusted[neighbor][0], adjusted[neighbor][1])
				var tip := Vector2(p.x, p.z)
				var moved := tip.move_toward(q, minf(3.0 if adjustment == "trim3" else 6.0, tip.distance_to(q)*0.4))
				adjusted[endpoint] = [moved.x, moved.y]
		if adjusted == points or _length(adjusted) < 5.0:
			continue
		reason = await _attempt(simulation, stroke, adjusted, record, adjustment)
		if _fatal:
			return
		if reason.is_empty():
			record["pieces"].append(record["attempts"][-1]["points_xz"])
			return
	if stroke.get("kind", "road") == "road" and original_reason in ["surface_geometry_invalid", "render_geometry_invalid", "junction_arm_angle"]:
		for candidate in _slide_candidates(simulation, points):
			reason = await _attempt(simulation, stroke, candidate["points_xz"], record, candidate["strategy"])
			if _fatal:
				return
			if reason.is_empty():
				record["pieces"].append(record["attempts"][-1]["points_xz"])
				return
	var failure := {"points_xz": points, "reason": reason}
	record["failed_pieces"].append(failure)
	if stroke.get("kind", "road") == "road":
		_deferred.append({"stroke": stroke, "record": record, "failure": failure})

func _apply_zone(simulation: Node, zone: Dictionary, accepted: Dictionary, profiles: Dictionary) -> Dictionary:
	var result := {"id": zone["id"], "status": "skipped", "reason": "frontage_stroke_refused"}
	if not accepted.get(zone["stroke_id"], false):
		return result
	var key := String(zone["zone_type"]) + "/" + String(zone["density"])
	if not profiles.has(key):
		result["reason"] = "zone_profile_unavailable"
		return result
	var p: Array = zone["point_xz"]
	var runtime_id := int(profiles[key])
	var width := int(zone["frontage_cells"])
	var depth := int(zone["depth_cells"])
	var preview: Dictionary = simulation.get_zoning_parcel_preview(float(p[0]), float(p[1]), runtime_id, width, depth)
	if not preview.get("valid", false):
		result["status"] = "refused"
		# Empty preview has no exposed Rust reason; do not invent a game refusal.
		result["reason"] = String(preview.get("reason", "zoning_preview_unavailable"))
		return result
	var committed: bool = simulation.apply_zoning_parcel_at(float(p[0]), float(p[1]), runtime_id, width, depth)
	result["status"] = "accepted" if committed else "refused"
	result["reason"] = "" if committed else "zoning_commit_false_no_exposed_reason"
	return result

func _write_results() -> bool:
	if DirAccess.make_dir_recursive_absolute(_output) != OK:
		push_error("Cannot create map output directory")
		return false
	var file := FileAccess.open(_output.path_join("apply_results.json"), FileAccess.WRITE)
	if file == null:
		push_error("Cannot write map apply results")
		return false
	file.store_string(JSON.stringify(_results, "\t", true) + "\n")
	file.close()
	return true

func _fail(message: String) -> void:
	_fatal = true
	push_error(message)
	if not _results.is_empty():
		_results["error"] = message
		_write_results()
	quit(2)

func _arguments() -> Dictionary:
	var result := {}
	var args := OS.get_cmdline_user_args()
	var index := 0
	while index < args.size():
		if not args[index].begins_with("--") or index + 1 >= args.size():
			return {}
		result[args[index].substr(2)] = args[index + 1]
		index += 2
	return result

func _resolve_path(path: String) -> String:
	var anchored := path if path.is_absolute_path() else "res://".path_join(path)
	return ProjectSettings.globalize_path(anchored).simplify_path()
