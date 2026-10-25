# SPDX-License-Identifier: GPL-2.0-only

## ROAD-67 roadbed contract, excluding intentionally changed paint.
## ROAD-73 recomputes the two nonplanar median junction grades. Their XZ triangles and
## counts match 3f4abd8d exactly; the largest asphalt height adjustment is 17.6 mm.
## ROAD-81 deliberately shortens divided-road visual mouths to their facing-rail conflicts.
## The two undivided new-road hashes stay identical; divided products now retain longer medians.
## An optional local record run writes hashes and roads-b-prefixed saves for load comparison.
extends "res://tests/road_cross_section_test.gd"

const EXPECTED: Array[String] = ["56579143fcfd507b8634b309e5818d40b6f101cdbc6630f4936ad7a2e1fb93ac", "b2c44f4bc925367153472c21a5b416ca4b930b187963a1e7a12e10294b24a595", "063a051fb7deb7718016932860eb858294d1961ebc0ce4b3b3b9c7d315dff33a", "ccd15777f3d3b0ebfad559f6970bf6659f84118f1f2cda15b1d221fe4006fb54"]

# New v95 roads deliberately omit zebra partitions and median zebra clearance.
const UNMARKED_EXPECTED: Array[String] = ["8068b4901c45e21b9b65eb4fc63a640c3e0f61a5711f10de72fc48bbd3cd369e", "2a246b05934b10437929ccf5eac549c099ebf7c494ec862df33e595be1710f53", "125d14bf21037e869d49b98071ffcdba6ffc41f152f23772ac63d1b3d8fdb001", "f791465cef2ffe425e82c71efaf909b348af60de8d7621beddfed207cc6d9f10"]

func roadbed_digest() -> String:
	var payload: Dictionary = simulation.get_road_mesh_data(true)
	var chunks: Array = payload.get("chunks", [])
	chunks.sort_custom(func(a: Dictionary, b: Dictionary) -> bool:
		return int(a.chunk_x) < int(b.chunk_x) or (int(a.chunk_x) == int(b.chunk_x) and int(a.chunk_z) < int(b.chunk_z)))
	var geometry: Array = []
	for chunk: Dictionary in chunks:
		geometry.append([chunk.chunk_x, chunk.chunk_z])
		var keys: Array = chunk.keys()
		keys.sort()
		for key: String in keys:
			if key.begins_with("marking_"):
				continue
			if key.ends_with("_vertices") or key.ends_with("_normals") or key.ends_with("_uvs") or key.ends_with("_colors"):
				geometry.append([key, chunk[key]])
	var context := HashingContext.new()
	context.start(HashingContext.HASH_SHA256)
	context.update(var_to_bytes(geometry))
	return context.finish().hex_encode()

func run() -> void:
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	var hashes: Array[String] = []
	var artifact_dir := OS.get_environment("METRUM_ROADS_B_CONTRACT_DIR")
	var load_legacy := not artifact_dir.is_empty() and OS.get_environment("METRUM_ROADS_B_RECORD_LEGACY") != "1"
	for style in [0, 2]:
		for skew in [false, true]:
			if load_legacy:
				var path := artifact_dir.path_join("roads-b-legacy-%d.sqlite" % hashes.size())
				expect(simulation.load_game(path), "Read pre-v95 fixture")
				hashes.append(roadbed_digest())
				continue
			expect(simulation.create_blank_world(640, 640, 4, 128, 0), "Legacy blank world")
			simulation.set_simulation_speed(0)
			await build(PackedVector3Array([Vector3(-210, 0, 0), Vector3(210, 0, 0)]), 2, 2, style, 5.0 if style == 2 else 0.0)
			await build(PackedVector3Array([Vector3(-45 if skew else 0, 0, -160), Vector3(45 if skew else 0, 0, 160)]), 1, 1, 0, 0)
			var digest := roadbed_digest()
			hashes.append(digest)
			if not artifact_dir.is_empty():
				var path := artifact_dir.path_join("roads-b-legacy-%d.sqlite" % (hashes.size() - 1))
				if OS.get_environment("METRUM_ROADS_B_RECORD_LEGACY") == "1":
					expect(simulation.save_game(path), "Record old-format road fixture")
				expect(simulation.load_game(path), "Load old-format road fixture")
				expect(roadbed_digest() == digest, "Existing save must retain roadbed geometry")
	if OS.get_environment("METRUM_ROADS_B_RECORD_LEGACY") == "1":
		var file := FileAccess.open(artifact_dir.path_join("legacy-roadbed-hashes.json"), FileAccess.WRITE)
		file.store_string(JSON.stringify(hashes))
	else:
		expect(hashes == (EXPECTED if load_legacy else UNMARKED_EXPECTED), "Roadbed geometry must match legacy or new unmarked contract: %s" % [hashes])
	simulation.queue_free()
	await process_frame
	quit(failures)
