# SPDX-License-Identifier: GPL-2.0-only

## Dense importer targets use the standard Rust profile solve before layered placement.
## Structural endpoint heights survive conditioning beside a different-level owner.
extends "res://tests/road_cross_section_test.gd"

func run() -> void:
	simulation = ClassDB.instantiate("SimulationNode")
	root.add_child(simulation)
	await process_frame
	expect(simulation.create_blank_world(512, 512, 4, 128, 0), "Source profile world")
	simulation.set_simulation_speed(0)
	simulation.set_road_tool_crossing_level(1)
	await build(PackedVector3Array([Vector3(-100, 0, -20), Vector3(100, 0, -20)]), 1, 1, 0, 0)
	var raw := PackedVector3Array()
	for i in range(41):
		raw.append(Vector3(-80 + i * 4, 0.2 if i % 2 == 0 else -0.2, 0))
	raw[0].y = 1.0
	raw[-1].y = 1.0
	var prepared: PackedVector3Array = simulation.prepare_source_road_profile(raw)
	expect(prepared[0] == raw[0] and prepared[-1] == raw[-1], "Preserve exact structural endpoint pins")
	var original_variation := 0.0
	var prepared_variation := 0.0
	for i in range(1, raw.size()):
		original_variation += absf(raw[i].y - raw[i-1].y)
	for i in range(1, prepared.size()):
		prepared_variation += absf(prepared[i].y - prepared[i-1].y)
	expect(prepared_variation < original_variation * 0.75, "Suppress dense DEM noise before the layered path")
	var plateau := PackedVector3Array([Vector3(-40, 8, 40), Vector3(0, 8, 40), Vector3(40, 8, 40)])
	for point: Vector3 in simulation.prepare_source_road_profile(plateau):
		expect(point.y == 8.0, "Engineered source targets survive conditioning")
	simulation.set_road_tool_crossing_level(0)
	await build(prepared, 1, 1, 0, 0)
	simulation.queue_free()
	await process_frame
	quit(failures)
