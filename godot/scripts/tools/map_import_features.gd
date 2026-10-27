# SPDX-License-Identifier: GPL-2.0-only

## Applies source-authored road features through the same acknowledged Rust APIs as the player.
## Preparation owns source interpretation; this bridge records every unsupported or refused product.
extends RefCounted

static func acknowledge_headless_render(simulation: Node) -> void:
	# No renderer consumes this offline bridge's published CPU products. Clear only
	# the exact publication backlog; validation and cached meshes remain intact.
	# Leaving full_replace pending rebuilds the whole city after every source road.
	simulation.acknowledge_network_render(simulation.get_network_render_generation())

static func finish(simulation: Node, tree: SceneTree, request: int) -> Dictionary:
	var deadline := Time.get_ticks_msec() + 120000
	while Time.get_ticks_msec() < deadline:
		var result: Variant = simulation.get_road_commit_result(request)
		if result is Dictionary:
			acknowledge_headless_render(simulation)
			return result
		await tree.process_frame
	return {"committed": false, "reason": "feature_acknowledgement_timeout", "unknown_completion": true}

static func roundabouts(simulation: Node, tree: SceneTree, products: Array) -> Array:
	var results := []
	for product: Dictionary in products:
		var p: Array = product.centre_xz
		var centre := Vector3(p[0], simulation.get_height_at(Vector2(p[0], p[1])), p[1])
		var node: int = simulation.get_closest_node(centre, 3.0)
		var record := {"source_ids": product.source_ids, "diameter_m": product.diameter_m,
			"source_lanes": product.source_lanes, "imported_lanes": 1,
			"remaining": product.remaining, "committed": false}
		if node < 0:
			record["reason"] = "roundabout_source_junction_missing"
		else:
			centre = simulation.get_node_pos(node)
			var preview: Dictionary = simulation.preview_roundabout(centre, product.diameter_m, PackedFloat32Array(), node)
			if not preview.get("is_valid", false):
				record["reason"] = preview.get("invalid_reason", "roundabout_preview_failed")
			else:
				record.merge(await finish(simulation, tree, simulation.place_roundabout(centre, product.diameter_m, PackedFloat32Array(), node)), true)
		results.append(record)
		if record.get("unknown_completion", false):
			return results
	return results
