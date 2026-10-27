# SPDX-License-Identifier: GPL-2.0-only

## Synthetic regression: three detached branches can join one main component in one sweep.
extends "res://scripts/tools/map_import_apply.gd"

class Fixture extends Node:
	var positions := [Vector3(0,0,0), Vector3(2,0,0), Vector3(12,0,10), Vector3(22,0,10), Vector3(-12,0,10), Vector3(-22,0,10), Vector3(0,0,-12), Vector3(0,0,-22)]
	var links := [[0,1], [2,3], [4,5], [6,7]]

	func get_height_at(_pos: Vector2) -> float:
		return 7.0

	func get_world_surface_height(_pos: Vector2) -> float:
		return 13.0

	func get_node_pos(id: int) -> Vector3:
		return positions[id]

	func get_closest_node(p: Vector3, radius: float) -> int:
		var result := -1
		var best := radius*radius
		# Eight fixed synthetic vertices; never used by the real importer.
		for i in positions.size():
			var d: float = positions[i].distance_squared_to(p)
			if d <= best:
				result = i
				best = d
		return result

	func get_road_terrain_benchmark_snapshot() -> String:
		var edges := []
		for pair in links:
			var points := []
			for id in pair:
				var p: Vector3 = positions[id]
				points.append([p.x, p.y, p.z, p.y])
			edges.append({"start_node": pair[0], "end_node": pair[1], "fwd_lanes": 1, "bkw_lanes": 1, "points": points})
		return JSON.stringify({"edges": edges})

	func connect_points(points: Array) -> void:
		var a := get_closest_node(Vector3(points[0][0],0,points[0][1]), 0.1)
		var b := get_closest_node(Vector3(points[-1][0],0,points[-1][1]), 0.1)
		assert(a >= 0 and b >= 0)
		links.append([a,b])

func _run() -> void:
	_results = {"repairs": []}
	var fixture := Fixture.new()
	root.add_child(fixture)
	assert(_source_point(fixture, {}, Vector2.ZERO).y == 7.0)
	assert(_source_point(fixture, {"bridge": true}, Vector2.ZERO).y == 7.0)
	assert(_source_point(fixture, {"tunnel": true}, Vector2.ZERO).y == 7.0)
	await _repair_network(fixture)
	assert(Network.inspect_network(fixture, 0.0)["components"] == 1)
	assert(_results["repairs"].size() == 3)
	for record in _results["repairs"]:
		assert(String(record["id"]).begins_with("repair/0/"))
	quit(0)

func _attempt(simulation: Node, _stroke: Dictionary, points: Array, _record: Dictionary, _strategy: String) -> String:
	simulation.connect_points(points)
	return ""

func _checkpoint(_simulation: Node) -> void:
	pass

func _write_results() -> bool:
	return true
