# SPDX-License-Identifier: GPL-2.0-only

## Offline importer diagnostics and bounded gap candidates from the existing road snapshot API.
## A 32 m segment grid limits searches to local roads; no per-tick or all-pairs work.
extends RefCounted

static func _root(parents: Dictionary, node: int) -> int:
	while int(parents[node]) != node:
		parents[node] = parents[parents[node]]
		node = int(parents[node])
	return node

static func inspect_network(simulation: Node, radius: float = 40.0) -> Dictionary:
	var snapshot: Dictionary = JSON.parse_string(simulation.get_road_terrain_benchmark_snapshot())
	var parents := {}
	var positions := {}
	var canonical := {}
	var degree := {}
	var edges := []
	for edge in snapshot["edges"]:
		if int(edge["fwd_lanes"]) + int(edge["bkw_lanes"]) == 0:
			continue
		var nodes := []
		for field in ["start_node", "end_node"]:
			var original := int(edge[field])
			if not canonical.has(original):
				var p: Vector3 = simulation.get_node_pos(original)
				var node: int = simulation.get_closest_node(p, 0.05)
				canonical[original] = node if node >= 0 else original
				positions[int(canonical[original])] = p
			var node := int(canonical[original])
			if not parents.has(node):
				parents[node] = node
			degree[node] = int(degree.get(node, 0)) + 1
			nodes.append(node)
		var a := _root(parents, nodes[0])
		var b := _root(parents, nodes[1])
		parents[maxi(a, b)] = mini(a, b)
		edges.append({"a": nodes[0], "b": nodes[1], "points": edge["points"]})
	var components := {}
	for node in parents:
		var r := _root(parents, node)
		components[r] = int(components.get(r, 0)) + 1
	if radius <= 0.0:
		return {"components": components.size(), "car_edges": edges.size(), "car_nodes": parents.size()}
	var grid := {}
	var segments := []
	for edge in edges:
		var points: Array = edge["points"]
		for i in range(1, points.size()):
			var a := Vector2(points[i-1][0], points[i-1][2])
			var b := Vector2(points[i][0], points[i][2])
			if a.distance_to(b) < 0.01:
				continue
			var index := segments.size()
			segments.append({"a": a, "b": b, "root": _root(parents, edge["a"])})
			for x in range(floori(minf(a.x, b.x)/32.0), floori(maxf(a.x, b.x)/32.0)+1):
				for z in range(floori(minf(a.y, b.y)/32.0), floori(maxf(a.y, b.y)/32.0)+1):
					var cell := Vector2i(x, z)
					if not grid.has(cell):
						grid[cell] = []
					grid[cell].append(index)
	var candidates := []
	var nodes: Array = parents.keys()
	nodes.sort()
	for node in nodes:
		if int(degree[node]) != 1:
			continue
		var pos: Vector3 = positions[node]
		var p := Vector2(pos.x, pos.z)
		var own := _root(parents, node)
		var nearby := {}
		for x in range(floori((p.x-radius)/32.0), floori((p.x+radius)/32.0)+1):
			for z in range(floori((p.y-radius)/32.0), floori((p.y+radius)/32.0)+1):
				for i in grid.get(Vector2i(x, z), []):
					nearby[i] = true
		var by_component := {}
		var indices: Array = nearby.keys()
		indices.sort()
		for i in indices:
			var s: Dictionary = segments[i]
			if int(s["root"]) == own:
				continue
			var a: Vector2 = s["a"]
			var b: Vector2 = s["b"]
			var t := clampf((p-a).dot(b-a)/(b-a).length_squared(), 0.0, 1.0)
			var q := a.lerp(b, t)
			var gap := p.distance_to(q)
			var best: Dictionary = by_component.get(s["root"], {})
			if gap < 2.1 or gap > radius or (not best.is_empty() and gap >= float(best["gap_m"])):
				continue
			by_component[s["root"]] = {"node": node, "from_component": own, "to_component": s["root"],
				"gap_m": gap, "points_xz": [[p.x, p.y], [q.x, q.y]]}
		var choices: Array = by_component.values()
		choices.sort_custom(func(a, b): return float(a["gap_m"]) < float(b["gap_m"]) if not is_equal_approx(a["gap_m"], b["gap_m"]) else int(a["to_component"]) < int(b["to_component"]))
		candidates.append_array(choices.slice(0, mini(4, choices.size())))
	candidates.sort_custom(func(a, b): return float(a["gap_m"]) < float(b["gap_m"]) if not is_equal_approx(a["gap_m"], b["gap_m"]) else (int(a["node"]) < int(b["node"]) if int(a["node"]) != int(b["node"]) else int(a["to_component"]) < int(b["to_component"])))
	return {"components": components.size(), "component_nodes": components, "candidates": candidates,
		"car_edges": edges.size(), "car_nodes": parents.size()}
