# SPDX-License-Identifier: GPL-2.0-only

## ROAD-76 source-sized roundabout application through production preview/commit/save APIs.
extends "res://tests/road_cross_section_test.gd"

const ImportFeatures := preload("res://scripts/tools/map_import_features.gd")

func run() -> void:
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Importer feature world")
	simulation.set_simulation_speed(0)
	await build(PackedVector3Array([Vector3(-180, 0, 0), Vector3(180, 0, 0)]), 2, 2, 3, 2.0)
	await build(PackedVector3Array([Vector3(0, 0, 0), Vector3(0, 0, 180)]), 1, 1, 0, 0)
	var result: Array = await ImportFeatures.roundabouts(simulation, self, [{
		"centre_xz": [0, 0], "diameter_m": 28.675, "source_ids": ["osm/test/ring"],
		"source_lanes": 1, "remaining": ""}])
	expect(result.size() == 1 and result[0].get("committed", false), "Imported ring commits at measured source size: " + str(result))
	expect(not simulation.get_road_mesh_data(false).get("full_replace", true), "Offline consumer acknowledges the exact completed publication")
	expect(not simulation.get_road_mesh_data(true).get("chunks", []).is_empty(), "Acknowledgement retains validated CPU meshes for later hydration")
	var ring_count := 0
	var snapshot: Dictionary = JSON.parse_string(simulation.get_road_terrain_benchmark_snapshot())
	for edge: Dictionary in snapshot.edges:
		var section: Dictionary = simulation.get_road_cross_section(int(edge.edge_id))
		if section.get("fwd_lanes", 0) == 1 and section.get("bkw_lanes", 0) == 0:
			ring_count += 1
			expect(section.fwd_lanes == 1 and section.bkw_lanes == 0, "Ring has a single circulating lane")
	expect(ring_count == 3, "Source junction becomes three ROAD-62 circulating edges")
	var path := ProjectSettings.globalize_path("res://../maps/processed/kuopio_sw/road-76/roundabout-test.sqlite")
	expect(simulation.save_game(path), "Save imported ring")
	expect(simulation.load_game(path), "Reload imported ring")
	for x in [-80.0, 80.0]:
		var section: Dictionary = simulation.get_road_cross_section(simulation.get_hovered_edge(x, 0))
		expect(section.get("fwd_lanes", 0) == 2 and section.get("bkw_lanes", 0) == 2, "Retain divided approach lane counts")
		expect(section.get("median_width", 0.0) == 2.0, "Retain approach median through save/reload")
	var missing: Array = await ImportFeatures.roundabouts(simulation, self, [{
		"centre_xz": [250, 250], "diameter_m": 30.0, "source_ids": ["osm/test/missing"],
		"source_lanes": 2, "remaining": "two_lane_entry_selection_and_spiral_exits"}])
	expect(missing[0].reason == "roundabout_source_junction_missing", "Missing approaches are reported without inventing roads")
	simulation.queue_free()
	await process_frame
	quit(failures)
