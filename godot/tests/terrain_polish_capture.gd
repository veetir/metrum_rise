# SPDX-License-Identifier: GPL-3.0-or-later

## Reproducible terrain-brush stills on a fresh isolated world; never reads a player save.
## METRUM_TERRAIN_CAPTURE_PHASE selects before/after under the terrain-polish screenshot folder.
extends SceneTree

var _scene: Node3D
var _sim: SimulationNode
var _terrain: Node
var _camera: Camera3D
var _tool: Node
var _output: String
var _guide_focus: Variant = null

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("user://mods"))
	_output = ProjectSettings.globalize_path("res://../screenshots/2026-11-24-terrain-polish/")
	_output = _output.path_join(OS.get_environment("METRUM_TERRAIN_CAPTURE_PHASE"))
	DirAccess.make_dir_recursive_absolute(_output)
	_scene = load("res://scenes/Main.tscn").instantiate()
	_scene.set_script(null)
	root.add_child(_scene)
	current_scene = _scene
	await process_frame
	_sim = _scene.get_node("SimulationNode")
	assert(_sim.create_blank_world(1280.0, 1280.0, 10.0, 320.0, 0.0))
	_sim.set_simulation_speed(0.0)
	var manager: Node = _scene.get_node("InputManager")
	manager._refresh_after_world_load()
	manager.set_process(false)
	manager.set_process_input(false)
	manager.set_process_unhandled_input(false)
	_scene.get_node("CameraNode").set_process(false)
	_scene.get_node("CameraNode").set_process_input(false)
	_camera = Camera3D.new()
	_scene.add_child(_camera)
	_camera.far = 2000.0
	_camera.current = true
	for child in _scene.get_children():
		if child is CanvasLayer:
			child.visible = false
	_scene.get_node("SceneLighting").pin_hour_of_day(12.0)
	_scene.get_node("Vegetation").hide()
	_scene.get_node("Grass").hide()
	root.set_disable_input(true)
	root.size = Vector2i(1440, 900)
	_terrain = _scene.get_node("Terrain")
	_tool = manager.terrain_brush
	_tool.set_process(false)
	_stroke(0, Vector2(-50, 0), 80.0, 2.38125)
	_tool.select_mode(4)
	_tool.diameter_m = 200.0
	_tool._anchor_a = Vector3(-50, _sim.get_world_surface_height(Vector2(-50, 0)), 0)
	_tool._anchor_b = Vector3(100, -0.00001, 0)
	_tool._anchor_count = 2
	var focus := Vector3(-7, 27.2, 0)
	for distance_m in [60.0, 300.0]:
		_camera.position = focus + Vector3(0.2, 0.65, 0.733).normalized() * distance_m
		_camera.look_at(focus)
		_guide_focus = focus
		await _capture("guide_%dm" % distance_m)
	_guide_focus = null
	_tool.interrupt()
	# An oblique stroke exposes lattice steps on its sides instead of following a grid row.
	var start := Vector2(-50, -20)
	var end := Vector2(100, 55)
	var a := Vector3(start.x, _sim.get_world_surface_height(start), start.y)
	var b := Vector3(end.x, _sim.get_world_surface_height(end), end.y)
	_sim.begin_game_terrain_stroke(4, start, 30.0, 1.0, a, b)
	for step in range(1, 61):
		_sim.game_terrain_stroke_step(start.lerp(end, step / 60.0), 1.0 / 12.0)
	_terrain.apply_game_terrain_heights(_sim.refresh_game_terrain_stroke_visuals())
	_sim.end_game_terrain_stroke()
	_terrain.finish_game_terrain_heights()
	_camera.position = Vector3(88, 83, 130)
	_camera.look_at(Vector3(5, 14, 0))
	await _capture("ramp_sides")
	_stroke(0, Vector2(-220, -200), 40.0, 3.0)
	_camera.position = Vector3(-153, 55, -112)
	_camera.look_at(Vector3(-220, 8, -200))
	await _capture("raised_rim")
	_stroke(1, Vector2(220, -200), 40.0, 3.0)
	_camera.position = Vector3(287, 55, -112)
	_camera.look_at(Vector3(220, -8, -200))
	await _capture("lowered_rim")
	_scene.queue_free()
	await process_frame
	quit()

func _stroke(mode: int, at: Vector2, radius: float, seconds: float) -> void:
	var a: Vector3 = _tool._anchor_a
	var b: Vector3 = _tool._anchor_b
	_sim.begin_game_terrain_stroke(mode, at, radius, 1.0, a, b)
	_sim.game_terrain_stroke_step(at, seconds)
	_terrain.apply_game_terrain_heights(_sim.refresh_game_terrain_stroke_visuals())
	_sim.end_game_terrain_stroke()
	_terrain.finish_game_terrain_heights()

func _capture(label: String) -> void:
	var deadline := Time.get_ticks_msec() + 30000
	while Time.get_ticks_msec() < deadline:
		await process_frame
		if _guide_focus != null:
			_tool._show_preview(_guide_focus, Vector2(1000, 580))
		if not _sim.is_terrain_dirty() and _terrain.is_render_settled():
			break
	for frame in 30:
		await process_frame
		if _guide_focus != null:
			_tool._show_preview(_guide_focus, Vector2(1000, 580))
	await RenderingServer.frame_post_draw
	assert(root.get_texture().get_image().save_png(_output.path_join(label + ".png")) == OK)
