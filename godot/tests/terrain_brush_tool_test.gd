# SPDX-License-Identifier: GPL-3.0-or-later

## Exercises the gameplay brush through the input manager in an isolated Godot test profile.
extends SceneTree

const TerrainBrushTool := preload("res://scripts/tools/terrain_brush_tool.gd")

var _failures := 0

func _initialize() -> void:
	call_deferred("_run")

func _expect(condition: bool, message: String) -> void:
	if not condition:
		_failures += 1
		push_error(message)

func _key(code: Key) -> InputEventKey:
	var event := InputEventKey.new()
	event.keycode = code
	event.pressed = true
	return event

func _mouse(button: MouseButton, pressed: bool, position: Vector2) -> InputEventMouseButton:
	var event := InputEventMouseButton.new()
	event.button_index = button
	event.pressed = pressed
	event.position = position
	return event

func _expect_height(sim: SimulationNode, expected: float, message: String) -> void:
	var deadline := Time.get_ticks_msec() + 2000
	while not is_equal_approx(sim.get_height_at(Vector2.ZERO), expected) and Time.get_ticks_msec() < deadline:
		await process_frame
	_expect(is_equal_approx(sim.get_height_at(Vector2.ZERO), expected), message)

# Global parameters are write-only in a running game. A rendered probe reads the values
# the shader actually receives without calling the editor-only getter.
func _disc_probe() -> SubViewport:
	var viewport := SubViewport.new()
	viewport.size = Vector2i(4, 4)
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(viewport)
	var card := ColorRect.new()
	card.size = Vector2(4, 4)
	var shader := Shader.new()
	shader.code = """
shader_type canvas_item;
global uniform bool terrain_brush_enabled;
global uniform vec4 terrain_brush_cursor;
void fragment() {
	COLOR = vec4(float(terrain_brush_enabled), float(terrain_brush_cursor.z > 0.0), 0.0, 1.0);
}
"""
	var material := ShaderMaterial.new()
	material.shader = shader
	card.material = material
	viewport.add_child(card)
	return viewport

func _run() -> void:
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("user://mods"))
	var scene: Node = load("res://scenes/Main.tscn").instantiate()
	scene.set_script(null)
	root.add_child(scene)
	current_scene = scene
	var manager: Node = scene.get_node("InputManager")
	var camera: CameraNode = scene.get_node("CameraNode")
	var sim: SimulationNode = scene.get_node("SimulationNode")
	manager.set_simulation_speed(0.0)
	camera.focus_on(Vector3.ZERO, 100.0)
	await process_frame
	manager._unhandled_input(_key(KEY_Y))
	var tool: Node = manager.terrain_brush
	var disc_probe := _disc_probe() if DisplayServer.get_name() != "headless" else null
	_expect(tool.active, "Y should activate the gameplay brush")
	tool.set_process(false)

	var cursor := camera.unproject_position(Vector3(0, sim.get_height_at(Vector2.ZERO), 0))
	var wheel := _mouse(MOUSE_BUTTON_WHEEL_UP, true, cursor)
	wheel.shift_pressed = true
	for next in [tool.Mode.LOWER, tool.Mode.FLATTEN, tool.Mode.SMOOTH, tool.Mode.RAMP, tool.Mode.RAISE]:
		manager._input(wheel)
		_expect(tool.mode == next, "Shift-wheel should cycle and wrap the terrain mode")
	tool.select_mode(tool.Mode.SMOOTH)
	manager._input(_mouse(MOUSE_BUTTON_RIGHT, true, cursor))
	manager._unhandled_input(_mouse(MOUSE_BUTTON_RIGHT, true, cursor))
	_expect(tool.mode == tool.Mode.SMOOTH and tool.stroke_button == MOUSE_BUTTON_NONE,
		"Right-click should do nothing in Smooth")
	manager._unhandled_input(_key(KEY_ESCAPE))
	_expect(tool.active and tool.mode == tool.Mode.RAISE, "Escape should back Smooth out to Raise")
	tool.select_mode(tool.Mode.RAMP)
	tool._anchor_count = 1
	manager._unhandled_input(_key(KEY_ESCAPE))
	_expect(tool.mode == tool.Mode.RAMP and tool._anchor_count == 0,
		"Escape should clear placed ramp anchors before leaving Ramp")
	manager._unhandled_input(_key(KEY_ESCAPE))
	_expect(tool.active and tool.mode == tool.Mode.RAISE, "A second Escape should leave Ramp for Raise")
	manager._unhandled_input(_key(KEY_ESCAPE))
	_expect(not tool.active, "Escape in Raise should close the brush")
	manager._unhandled_input(_key(KEY_Y))
	_expect(tool.active, "Y should reopen the brush")
	tool.set_process(false)
	wheel.shift_pressed = false
	wheel.ctrl_pressed = true
	manager._input(wheel)
	_expect(tool.diameter_m == 70.0, "Ctrl-wheel should increase diameter by 10 m")
	var strength_key := _key(KEY_G)
	manager._unhandled_input(strength_key)
	_expect(tool.strength_index == 2, "G should step to Strong")
	manager._unhandled_input(strength_key)
	_expect(tool.strength_index == 0, "G should wrap to Gentle")

	for at in [Vector2.ZERO, Vector2(60.0, -20.0)]:
		var point := Vector3(at.x, sim.get_height_at(at), at.y)
		var mouse := camera.unproject_position(point)
		var hit: Variant = tool._probe(mouse, true)
		_expect(hit is Vector3, "The cursor ray should hit the source terrain")
		if hit is Vector3:
			tool._show_preview(hit, mouse)
			var disc: Vector4 = tool._cursor
			_expect(Vector2(disc.x, disc.y).distance_to(at) < 0.25 and disc.z == 35.0,
				"The disc centre should follow the terrain hit at the selected radius")
			_expect(tool._panel.visible and tool._label.text.begins_with("Raise"), "Preview should name the active mode")
	if disc_probe != null:
		await RenderingServer.frame_post_draw
		var enabled := disc_probe.get_texture().get_image().get_pixel(1, 1)
		_expect(enabled.r > 0.9 and enabled.g > 0.9, "The shader should receive an enabled footprint")

	tool.set_roads_follow(true)
	tool._show_preview(Vector3.ZERO, cursor)
	_expect(tool.roads_follow and tool._label.text.contains("Roads follow"), "The label should show Roads follow")
	tool.set_roads_follow(false)
	tool._show_preview(Vector3.ZERO, cursor)
	_expect(not tool._label.text.contains("Roads follow"), "Roads follow should switch off")
	tool.select_mode(tool.Mode.RAMP)
	tool._anchor_a = Vector3(-20, -0.00001, 0)
	tool._anchor_b = Vector3(20, -0.00002, 0)
	tool._anchor_count = 2
	tool._show_preview(Vector3.ZERO, cursor)
	_expect(not tool._label.text.contains("-0.0") and not tool._label.text.contains("·"),
		"Ramp heights and grade must round away negative zero without dot separators")
	_expect(tool._anchor_labels[0].visible and tool._anchor_labels[0].text == "A"
		and tool._anchor_labels[1].visible and tool._anchor_labels[1].text == "B",
		"Both road-style handles should have matching endpoint letters")
	tool.interrupt()
	_expect(not tool._handles[0].visible and not tool._anchor_labels[1].visible,
		"Interrupt should hide both endpoint handles and letters")
	tool.select_mode(tool.Mode.FLATTEN)
	tool._flatten_height = -0.00001
	tool._update_label(Vector3.ZERO)
	_expect(not tool._label.text.contains("-0.0"), "Flatten should round away negative zero too")
	tool.select_mode(tool.Mode.RAISE)
	var initial_height := sim.get_height_at(Vector2.ZERO)
	manager._unhandled_input(_mouse(MOUSE_BUTTON_LEFT, true, cursor))
	_expect(tool.stroke_button == MOUSE_BUTTON_LEFT, "World press should start a stroke")
	for step in 4:
		tool._advance(Vector3.ZERO, 1.0 / 60.0)
		_expect(sim.refresh_game_terrain_stroke_visuals().is_empty(),
			"The tool must consume each frame's changed patches immediately")
	tool._show_preview(Vector3.ZERO, cursor)
	_expect(tool._cursor.w == 2.0 and tool._preview_enabled, "The disc should show the active stroke")
	manager._input(_mouse(MOUSE_BUTTON_LEFT, false, cursor))
	_expect(tool.stroke_button == MOUSE_BUTTON_NONE, "Release should end the stroke")
	var raised := sim.get_height_at(Vector2.ZERO)
	_expect(raised > initial_height, "The released stroke should raise the ground")
	var history := _key(KEY_Z)
	history.ctrl_pressed = true
	manager._unhandled_input(history)
	await _expect_height(sim, initial_height, "Ctrl+Z should undo the entire stroke")
	history.shift_pressed = true
	manager._unhandled_input(history)
	await _expect_height(sim, raised, "Ctrl+Shift+Z should redo the entire stroke")

	manager._unhandled_input(_mouse(MOUSE_BUTTON_LEFT, true, cursor))
	_expect(tool.stroke_button == MOUSE_BUTTON_LEFT, "A second press should start a new stroke")
	manager._unhandled_input(_key(KEY_R))
	_expect(tool.stroke_button == MOUSE_BUTTON_NONE and not tool.active, "Tool switch should end the stroke")
	_expect(tool._cursor == Vector4.ZERO and not tool._preview_enabled and not tool._panel.visible,
		"Tool switch should clear the disc globals")
	if disc_probe != null:
		await RenderingServer.frame_post_draw
		var disabled := disc_probe.get_texture().get_image().get_pixel(1, 1)
		_expect(disabled.r < 0.1 and disabled.g < 0.1, "The shader should receive a cleared footprint")
	manager._unhandled_input(_key(KEY_Y))
	_expect(tool.diameter_m == 70.0 and tool.strength_index == 0, "Brush settings should survive tool switches")
	manager._unhandled_input(_mouse(MOUSE_BUTTON_RIGHT, true, cursor))
	_expect(tool.stroke_button == MOUSE_BUTTON_RIGHT and tool._stroke_mode == tool.Mode.LOWER,
		"Right mouse should invert Raise")
	manager._on_camera_focus_exited()
	_expect(tool.stroke_button == MOUSE_BUTTON_NONE, "Focus loss should close the stroke")
	tool.select_mode(tool.Mode.SMOOTH)
	scene.queue_free()
	await process_frame
	var reopened := TerrainBrushTool.new()
	_expect(reopened.mode == TerrainBrushTool.Mode.SMOOTH and reopened.diameter_m == 70.0
		and reopened.strength_index == 0, "Brush settings should survive rebuilding the game scene")
	reopened.free()
	if _failures == 0:
		print("terrain_brush_tool_test: PASS")
	quit(int(_failures != 0))
