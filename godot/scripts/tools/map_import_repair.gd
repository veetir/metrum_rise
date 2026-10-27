# SPDX-License-Identifier: GPL-2.0-only

## Reopens only an importer-owned save and retries refused pieces after the full network exists.
## Explicit legacy gap repair runner; fresh application never invents these connections.
extends "res://scripts/tools/map_import_apply.gd"

func _run() -> void:
	var args := _arguments()
	Engine.max_fps = 60
	_output = _resolve_path(String(args.get("output", "../maps/processed/kuopio_sw/repair")))
	var base := _resolve_path(String(args.get("base", "../maps/processed/kuopio_sw/run1")))
	var plan_path := _resolve_path(String(args.get("plan", "../maps/processed/kuopio_sw/plan.json")))
	var local_maps := ProjectSettings.globalize_path("res://../maps/processed/").simplify_path()+"/"
	if not (_output+"/").begins_with(local_maps) or not (base+"/").begins_with(local_maps) or _output == base:
		_fail("Repair needs distinct importer-owned base and output folders")
		return
	var plan: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(plan_path))
	_results = JSON.parse_string(FileAccess.get_file_as_string(base.path_join("apply_results.json")))
	if not _results.get("saved", false) or _results["plan_sha256"] != FileAccess.get_sha256(plan_path):
		_fail("Repair base must be saved and match this exact plan")
		return
	_results["complete"] = false
	_results["saved"] = false
	_results["repair_base"] = base
	_results["repair_policy"] = {"gap_radius_m": 40.0, "targets_per_endpoint": 4}
	var simulation := SimulationNode.new()
	root.add_child(simulation)
	simulation.load_all_asset_packs(_resolve_path(String(args.get("packs", "res://bootstrap/mods"))))
	if not simulation.load_game(base.path_join("city.sqlite")):
		_fail("Cannot load repair base")
		return
	simulation.set_simulation_speed(0.0)
	var strokes := {}
	for stroke in plan["strokes"]:
		strokes[stroke["id"]] = stroke
	for record in _results["roads"]:
		var failures: Array = record["failed_pieces"].duplicate(true)
		record["failed_pieces"] = []
		for failure in failures:
			await _place(simulation, strokes[record["id"]], failure["points_xz"], record, 4, "full_network_retry")
			if _fatal:
				return
		_finish_record(record)
		_write_results()
		_checkpoint(simulation)
	await _repair_network(simulation)
	_results["network"] = Network.inspect_network(simulation)
	_results["car_components"] = _results["network"]["components"]
	_results["snapshot"] = JSON.parse_string(simulation.get_road_terrain_benchmark_snapshot())
	_results["saved"] = simulation.save_game(_output.path_join("city.sqlite"))
	_results["save_path"] = _output.path_join("city.sqlite")
	_results["complete"] = true
	_write_results()
	quit(0 if _results["saved"] else 2)
