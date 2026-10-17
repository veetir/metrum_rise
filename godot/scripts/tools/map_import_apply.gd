# SPDX-License-Identifier: GPL-2.0-only

## Applies an offline map plan using the player's road and zoning APIs, then saves a city.
## Standalone SceneTree script; deliberately unrun during importer development. See
## tools/map_import/cli.py for command arguments. RoadCommit acknowledgements are awaited;
## unknown completion stops the entire apply so a late road cannot race a subsequent stroke.
## Simulation stays paused; source speeds and building footprints remain in neutral.json.
extends SceneTree

const Replay := preload("res://scripts/ui/debug_console/road_attempt_replay.gd")
var _output := ""
var _results: Dictionary = {}

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
	var accepted := {}
	for stroke in plan["strokes"]:
		var points := []
		for p in stroke["points_xz"]:
			var pos := Vector2(float(p[0]), float(p[1]))
			points.append([pos.x, simulation.get_height_at(pos), pos.y])
		var attempt: Dictionary = stroke.duplicate(true)
		attempt["points"] = points
		var outcome: Dictionary = await Replay.replay_attempt(simulation, self, attempt)
		var reason := String(outcome.get("reason", ""))
		if reason == "replay_timeout" or reason == "validation_unavailable":
			_fail("Unknown road completion for %s: %s" % [stroke["id"], reason])
			return
		_results["roads"].append({"id": stroke["id"], "status": "accepted" if reason.is_empty() else "refused",
			"reason": reason, "by": outcome["by"]})
		accepted[stroke["id"]] = reason.is_empty()
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
