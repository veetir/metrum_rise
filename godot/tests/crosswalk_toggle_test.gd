# SPDX-License-Identifier: GPL-3.0-or-later

## The Zebra tool through the native bridge: a click adds and removes a mid-road crossing,
## and an arm click updates the published junction paint. Undo publishes the same products.
extends SceneTree

const ZebraTool := preload("res://scripts/tools/zebra_tool.gd")
const RoadToolScript := preload("res://scripts/tools/road_tool.gd")
const RoadToolHudScript := preload("res://scripts/ui/road_tool_hud.gd")
var _failures := 0
var simulation: SimulationNode

class TerrainStub extends Node3D:
	pass

func _initialize() -> void:
	call_deferred("_run")

func _expect(condition: bool, message: String) -> void:
	if not condition:
		_failures += 1
		push_error(message)

func _ground_point(x: float, z: float) -> Vector3:
	return Vector3(x, simulation.get_world_surface_height(Vector2(x, z)), z)

# Waits for the render generation to pass `generation`; false after 20 s.
func _await_generation(generation: int) -> bool:
	var deadline := Time.get_ticks_msec() + 20000
	while Time.get_ticks_msec() < deadline:
		if simulation.get_network_render_generation() > generation:
			return true
		await process_frame
	return false

func _commit(points: PackedVector3Array) -> bool:
	var generation := simulation.get_network_render_generation()
	simulation.add_road_with_snap(points, 1, 1, true)
	var done := await _await_generation(generation)
	_expect(done, "fixture road must commit")
	return done

# Vertices of the installed road chunks' road layers that carry crossing paint (CUSTOM0.z > 0, as
# the road shader reads it).
func _painted_vertices(road_tool: Node3D) -> int:
	var painted := 0
	for instance: MeshInstance3D in road_tool.chunks.chunks.values():
		var mesh: ArrayMesh = instance.mesh
		if mesh == null:
			continue
		for surface in mesh.get_surface_count():
			var crossings = mesh.surface_get_arrays(surface)[Mesh.ARRAY_CUSTOM0]
			if crossings == null:
				continue
			for index in range(2, crossings.size(), 4):
				if crossings[index] > 0.0:
					painted += 1
	return painted

# The road tool installs the current render generation; returns whether it did.
func _install(road_tool: Node3D) -> bool:
	var generation := simulation.get_network_render_generation()
	return road_tool.chunks.update(generation) == generation

func _click(camera: Camera3D, world: Vector3) -> void:
	for pressed in [true, false]:
		var event := InputEventMouseButton.new()
		event.button_index = MOUSE_BUTTON_LEFT
		event.pressed = pressed
		event.position = camera.unproject_position(world)
		root.push_input(event, true)

func _run() -> void:
	var scene := Node3D.new()
	root.add_child(scene)
	simulation = SimulationNode.new()
	simulation.name = "SimulationNode"
	scene.add_child(simulation)
	_expect(simulation.create_blank_world(512.0, 512.0, 8.0, 128.0, 0.0), "fixture world must load")
	simulation.set_simulation_speed(0.0)
	if not await _commit(PackedVector3Array([_ground_point(-60.0, 0.0), _ground_point(60.0, 0.0)])) \
			or not await _commit(PackedVector3Array([_ground_point(0.0, -60.0), _ground_point(0.0, 60.0)])):
		_finish(scene)
		return
	var terrain := TerrainStub.new()
	terrain.name = "Terrain"
	scene.add_child(terrain)
	# Use the same chunk installation as the game.
	var road_tool := RoadToolScript.new()
	road_tool.name = "RoadTool"
	road_tool.simulation_node = simulation
	road_tool.terrain_node = terrain
	road_tool.make_chunks()
	road_tool.blueprint_mesh = MeshInstance3D.new()
	road_tool.add_child(road_tool.blueprint_mesh)
	road_tool.attach_hud(RoadToolHudScript.new())
	_expect(_install(road_tool), "the cross's chunks must install")
	var painted_before := _painted_vertices(road_tool)
	_expect(painted_before > 0, "the cross is drawn with zebras")

	var tool := ZebraTool.new()
	scene.add_child(tool)
	tool.set_tool_active(true)
	var camera := Camera3D.new()
	scene.add_child(camera)
	camera.position = Vector3(0.0, 24.0, 0.01)
	camera.look_at(Vector3.ZERO)
	camera.current = true
	await process_frame

	var mid := _ground_point(-37.37, 0.0)
	var count := simulation.get_network_nodes().size()
	tool.set_scripted_pointer(true, mid)
	await process_frame
	_expect(tool.blueprint_mesh.mesh != null, "the straight road shows a ghost")
	_click(camera, mid)
	_expect(simulation.get_network_nodes().size() == count + 1, "the mid-road click adds one node")
	_expect(simulation.preview_zebra(mid).get("removing", false), "the crossing is now the remove target")
	_click(camera, mid)
	_expect(not simulation.preview_zebra(mid).get("removing", false), "the second click removes the crossing")
	var generation := simulation.get_network_render_generation()
	_expect(simulation.undo_action(), "removal undo is queued")
	_expect(await _await_generation(generation), "removal undo publishes")
	_expect(simulation.preview_zebra(mid).get("removing", false), "undo brings back the mid-road zebra")
	generation = simulation.get_network_render_generation()
	_expect(simulation.undo_action(), "addition undo is queued")
	_expect(await _await_generation(generation), "addition undo publishes")
	_expect(simulation.get_network_nodes().size() == count, "addition undo restores the original road")

	var node: int = simulation.get_closest_node(Vector3.ZERO, 8.0)
	_expect(node >= 0, "the cross has a junction node")
	var arm_hit := Vector3.INF
	for step in range(1, 21):
		var point := _ground_point(float(step), 0.0)
		if simulation.preview_zebra(point).get("removing", false):
			arm_hit = point
			break
	_expect(arm_hit.is_finite(), "a painted junction arm is targetable")
	if arm_hit.is_finite():
		tool.set_scripted_pointer(true, arm_hit)
		generation = simulation.get_network_render_generation()
		_click(camera, arm_hit)
		_expect(not simulation.preview_zebra(arm_hit).get("removing", false), "the click removes the arm zebra")
		_expect(simulation.get_network_render_generation() > generation, "the toggle publishes a new render generation")
		_expect(_install(road_tool), "the toggle's chunks install")
		_expect(_painted_vertices(road_tool) < painted_before, "the arm's zebra paint is gone")
		generation = simulation.get_network_render_generation()
		_expect(simulation.undo_action(), "arm undo is queued")
		_expect(await _await_generation(generation), "arm undo publishes")
		_expect(simulation.preview_zebra(arm_hit).get("removing", false), "undo restores the arm zebra")
		_expect(_install(road_tool), "the undo's chunks install")
		_expect(_painted_vertices(road_tool) == painted_before, "undo restores the zebra paint")
	tool.set_tool_active(false)
	road_tool.free()
	_finish(scene)

func _finish(scene: Node) -> void:
	scene.free()
	if _failures == 0:
		print("crosswalk_toggle_test: PASS")
	quit(_failures)
