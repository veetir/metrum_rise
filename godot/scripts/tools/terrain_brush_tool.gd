# SPDX-License-Identifier: GPL-3.0-or-later

## Gameplay-only terrain brush input, cursor preview and stroke lifecycle.
## Rust owns path sampling, terrain kernels, protection and history.
extends Node

signal mode_changed

enum Mode { RAISE, LOWER, FLATTEN, SMOOTH, RAMP }

const UIStyle := preload("res://scripts/ui/ui_style.gd")
const MODE_NAMES := ["Raise", "Lower", "Flatten", "Smooth", "Ramp"]
const STRENGTH_NAMES := ["Gentle", "Normal", "Strong"]
const ALL_PROTECTED := 64
const WATER := 32
const ENABLED := &"terrain_brush_enabled"
const GRADE := &"terrain_brush_grade"
const MASK := &"terrain_brush_protection"
const MASK_GRID := &"terrain_brush_protection_grid"
const PREVIEW := &"terrain_brush_cursor"
const RAMP := &"terrain_brush_ramp"
const RAMP_COUNT := &"terrain_brush_ramp_count"

static var _session_mode := Mode.RAISE
static var _session_diameter_m := 60.0
static var _session_strength_index := 1

var mode := _session_mode
var diameter_m := _session_diameter_m
var strength_index := _session_strength_index
var stroke_button := MOUSE_BUTTON_NONE
var active := false

var _manager: Node
var _sim: SimulationNode
var _panel: PanelContainer
var _label: Label
var _cursor := Vector4.ZERO
var _status := 0
var _protection_text := ""
var _grade := Vector4.ZERO
var _stroke_mode := Mode.RAISE
var _flatten_height := 0.0
var _anchor_a := Vector3.ZERO
var _anchor_b := Vector3.ZERO
var _anchor_count := 0
var _last_sample_us := 0
var _preview_enabled := false
var _mask_image := Image.new()
var _mask_texture: ImageTexture
var _mask_revision := -1
var _mask_texture_revision := -1
var _mask_grid := Vector4.ZERO
var _cell_m := 0.0
var _notice := ""
var _notice_left := 0.0
var _gesture_steps := 0.0

func _ready() -> void:
	# After camera motion, before the terrain renderer syncs its graded materials.
	process_priority = -100
	set_process(false)

func setup(manager: Node) -> void:
	_manager = manager
	_sim = manager.simulation_node
	var layer := CanvasLayer.new()
	layer.layer = 5
	add_child(layer)
	_panel = PanelContainer.new()
	_panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var style := UIStyle.panel_style(UIStyle.BG_DARK, UIStyle.CORNER_PANEL)
	style.content_margin_left = 10
	style.content_margin_right = 10
	style.content_margin_top = 5
	style.content_margin_bottom = 5
	_panel.add_theme_stylebox_override("panel", style)
	layer.add_child(_panel)
	_label = Label.new()
	_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_label.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	UIStyle.set_font_size(_label, 13)
	_panel.add_child(_label)
	_panel.hide()
	get_window().mouse_exited.connect(interrupt)

func _exit_tree() -> void:
	_clear_preview()

func set_active(enabled: bool) -> void:
	active = enabled
	set_process(enabled)
	if not enabled:
		interrupt()
		reset_ramp()
		_notice_left = 0.0

func select_mode(next: int) -> void:
	if next == mode:
		return
	end_stroke()
	reset_ramp()
	mode = next as Mode
	_session_mode = mode
	mode_changed.emit()

## Whether Escape has a level to back out of before it closes the tool.
func can_step_back() -> bool:
	return mode > Mode.LOWER

# Escape outside Raise/Lower backs out one level: placed ramp anchors first, then the mode
# itself, returning to the plain Raise brush.
func step_back() -> void:
	end_stroke()
	if mode == Mode.RAMP and _anchor_count > 0:
		reset_ramp()
	else:
		select_mode(Mode.RAISE)

func reset_ramp() -> void:
	_anchor_count = 0
	RenderingServer.global_shader_parameter_set(RAMP_COUNT, 0)

func cycle_strength() -> void:
	end_stroke()
	strength_index = (strength_index + 1) % STRENGTH_NAMES.size()
	_session_strength_index = strength_index

func apply_gesture(event: InputEvent) -> bool:
	if not (event is InputEventWithModifiers and (event.ctrl_pressed or event.shift_pressed)):
		return false
	var direction := 0
	if event is InputEventMouseButton and event.pressed:
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_LEFT: direction = 1
			MOUSE_BUTTON_WHEEL_DOWN, MOUSE_BUTTON_WHEEL_RIGHT: direction = -1
	elif event is InputEventPanGesture:
		var motion: Vector2 = event.delta
		_gesture_steps -= motion.y if absf(motion.y) >= absf(motion.x) else motion.x
		direction = int(_gesture_steps)
		_gesture_steps -= direction
		if direction == 0:
			return true
	if direction == 0:
		return false
	if event.ctrl_pressed:
		end_stroke()
		diameter_m = clampf(diameter_m + 10.0 * direction, 20.0, 200.0)
		_session_diameter_m = diameter_m
	else:
		select_mode(posmod(mode + direction, MODE_NAMES.size()))
	return true

func press(event: InputEventMouseButton) -> bool:
	if event.alt_pressed or (event.button_index != MOUSE_BUTTON_LEFT and event.button_index != MOUSE_BUTTON_RIGHT):
		return false
	if stroke_button != MOUSE_BUTTON_NONE or get_viewport().gui_get_hovered_control() != null:
		return true
	if event.button_index == MOUSE_BUTTON_RIGHT and mode > Mode.LOWER:
		# Right mouse only inverts Raise and Lower; elsewhere it does nothing, so a stray
		# click never changes the mode or the ground.
		return true
	var hit: Variant = _probe(event.position, true)
	if hit is Vector3:
		_press(hit, event.button_index)
	return true

func _press(hit: Vector3, button: MouseButton) -> void:
	if mode == Mode.RAMP and _anchor_count < 2:
		if _anchor_count == 0:
			_anchor_a = hit
		elif Vector2(hit.x - _anchor_a.x, hit.z - _anchor_a.z).length_squared() < 1.0:
			_show_notice("Ramp anchors must be at least 1 m apart")
			return
		else:
			_anchor_b = hit
		_anchor_count += 1
		return
	_stroke_mode = mode
	if button == MOUSE_BUTTON_RIGHT:
		_stroke_mode = Mode.LOWER if mode == Mode.RAISE else Mode.RAISE
	_flatten_height = _sim.game_terrain_flatten_height(Vector2(hit.x, hit.z))
	_sim.begin_game_terrain_stroke(_stroke_mode, Vector2(hit.x, hit.z), diameter_m * 0.5,
		strength_index, _anchor_a, _anchor_b)
	stroke_button = button
	_last_sample_us = Time.get_ticks_usec()

func release(event: InputEventMouseButton) -> void:
	if _manager.terrain_brush_world_input() and get_viewport().gui_get_hovered_control() == null:
		var hit: Variant = _probe(event.position, true)
		if hit is Vector3:
			_advance_pointer(hit)
	end_stroke()

func end_stroke() -> void:
	if stroke_button == MOUSE_BUTTON_NONE:
		return
	stroke_button = MOUSE_BUTTON_NONE
	_sim.end_game_terrain_stroke()
	_manager.terrain_node.finish_game_terrain_heights()

func interrupt() -> void:
	end_stroke()
	_clear_preview()

func _process(delta: float) -> void:
	var viewport := get_viewport()
	var mouse := viewport.get_mouse_position()
	if (not _manager.terrain_brush_world_input() or not viewport.get_visible_rect().has_point(mouse)
			or viewport.gui_get_hovered_control() != null):
		interrupt()
		return
	_notice_left = maxf(0.0, _notice_left - delta)
	var hit: Variant = _probe(mouse, stroke_button != MOUSE_BUTTON_NONE)
	if not hit is Vector3:
		interrupt()
		return
	if stroke_button != MOUSE_BUTTON_NONE:
		if Input.is_mouse_button_pressed(stroke_button):
			_advance_pointer(hit)
			# Movement of the ground can move the ray's intersection under a stationary mouse.
			var live_hit: Variant = _probe(mouse, true)
			if live_hit is Vector3:
				hit = live_hit
		else:
			end_stroke()
	_show_preview(hit, mouse)

func _probe(mouse: Vector2, wait: bool) -> Variant:
	var camera := get_viewport().get_camera_3d()
	if not camera:
		return null
	var origin := camera.project_ray_origin(mouse)
	var direction := camera.project_ray_normal(mouse)
	return _sim.game_terrain_brush_hit(origin, direction, wait)

func _advance_pointer(hit: Vector3) -> void:
	var now := Time.get_ticks_usec()
	var elapsed := (now - _last_sample_us) * 0.000001
	_last_sample_us = now
	_advance(hit, elapsed)

func _advance(hit: Vector3, delta: float) -> void:
	_sim.game_terrain_stroke_step(Vector2(hit.x, hit.z), delta)
	_manager.terrain_node.apply_game_terrain_heights(_sim.refresh_game_terrain_stroke_visuals())

func _show_notice(message: String) -> void:
	_notice = message
	_notice_left = 2.5

func _show_preview(hit: Vector3, mouse: Vector2) -> void:
	var radius := diameter_m * 0.5
	var pos := Vector2(hit.x, hit.z)
	var mask := _sim.game_terrain_brush_preview(pos, radius, _mask_revision, _mask_image)
	if int(mask.z) != _status:
		_status = int(mask.z)
		_protection_text = _reason(_status)
	_mask_revision = int(mask.w)
	var mask_grid := Vector4.ZERO
	if _status & 15 and not _mask_image.is_empty():
		var side := _mask_image.get_width()
		if _mask_texture_revision != _mask_revision:
			_mask_texture_revision = _mask_revision
			_cell_m = float(_sim.get_terrain_patch_layout()["terrain_cell_m"])
			if _mask_texture == null or _mask_texture.get_width() != side:
				_mask_texture = ImageTexture.create_from_image(_mask_image)
				RenderingServer.global_shader_parameter_set(MASK, _mask_texture)
			else:
				_mask_texture.update(_mask_image)
		mask_grid = Vector4(mask.x, mask.y, _cell_m, side)
	if mask_grid != _mask_grid:
		if _mask_grid == Vector4.ZERO:
			RenderingServer.global_shader_parameter_set(MASK, _mask_texture)
		_mask_grid = mask_grid
		RenderingServer.global_shader_parameter_set(MASK_GRID, _mask_grid)
	var state := 1.0 if stroke_button == MOUSE_BUTTON_NONE else 2.0
	var cursor := Vector4(pos.x, pos.y, radius, state)
	if cursor != _cursor:
		_cursor = cursor
		RenderingServer.global_shader_parameter_set(PREVIEW, _cursor)
	var shown_mode := mode if stroke_button == MOUSE_BUTTON_NONE else _stroke_mode
	var target := 0.0
	if shown_mode == Mode.FLATTEN:
		if stroke_button == MOUSE_BUTTON_NONE:
			# On a building's pad or paved yard the target is its floor height.
			_flatten_height = _sim.game_terrain_flatten_height(pos)
		target = _flatten_height
	var grade := Vector4(shown_mode, target, _anchor_a.y, _anchor_b.y)
	if grade != _grade:
		_grade = grade
		RenderingServer.global_shader_parameter_set(GRADE, _grade)
	if mode == Mode.RAMP:
		RenderingServer.global_shader_parameter_set(RAMP,
			Vector4(_anchor_a.x, _anchor_a.z, _anchor_b.x, _anchor_b.z))
		RenderingServer.global_shader_parameter_set(RAMP_COUNT, _anchor_count)
	if not _preview_enabled:
		_preview_enabled = true
		RenderingServer.global_shader_parameter_set(ENABLED, true)
	_update_label(hit)
	var limit := get_viewport().get_visible_rect().end - _panel.size - Vector2(6, 6)
	_panel.position = (mouse + Vector2(18, 24)).clamp(Vector2(6, 6), limit.max(Vector2(6, 6)))
	_panel.show()

func _update_label(hit: Vector3) -> void:
	var shown_mode := mode if stroke_button == MOUSE_BUTTON_NONE else _stroke_mode
	var heading: String = MODE_NAMES[shown_mode]
	if shown_mode == Mode.FLATTEN:
		heading += "  %.1f m" % _flatten_height
	elif shown_mode == Mode.RAMP:
		match _anchor_count:
			0: heading += "   Set A"
			1: heading += "   A %.1f m   Set B" % _anchor_a.y
			2:
				var start := Vector2(_anchor_a.x, _anchor_a.z)
				var segment := Vector2(_anchor_b.x, _anchor_b.z) - start
				var fraction := clampf((Vector2(hit.x, hit.z) - start).dot(segment) / segment.length_squared(), 0.0, 1.0)
				var grade := 100.0 * (_anchor_b.y - _anchor_a.y) / segment.length()
				heading += "  %.1f m   %+.1f%%\nA %.1f m   B %.1f m" % [
					lerpf(_anchor_a.y, _anchor_b.y, fraction), grade, _anchor_a.y, _anchor_b.y]
	var content := "%s\n%.0f m   %s" % [heading, diameter_m, STRENGTH_NAMES[strength_index]]
	if not _protection_text.is_empty():
		content += "\n" + _protection_text
	if _notice_left > 0.0:
		content += "\n" + _notice
	if _label.text != content:
		_label.text = content
		_panel.reset_size()

func _reason(status: int) -> String:
	if status & 16: return "Outside the world"
	if status & WATER: return "Water bed"
	if (status & 15) == 0: return ""
	var kinds := PackedStringArray()
	if status & 1: kinds.append("road")
	if status & 2: kinds.append("building")
	if status & 8: kinds.append("work area")
	var names := " and ".join(kinds)
	if names.is_empty():
		return ""
	names = names.left(1).to_upper() + names.substr(1)
	return names + (" protected" if status & ALL_PROTECTED else " kept")

func _clear_preview() -> void:
	if not _preview_enabled:
		return
	_preview_enabled = false
	_cursor = Vector4.ZERO
	RenderingServer.global_shader_parameter_set(ENABLED, false)
	RenderingServer.global_shader_parameter_set(PREVIEW, _cursor)
	_grade = Vector4.ZERO
	RenderingServer.global_shader_parameter_set(GRADE, _grade)
	RenderingServer.global_shader_parameter_set(RAMP_COUNT, 0)
	RenderingServer.global_shader_parameter_set(RAMP, Vector4.ZERO)
	RenderingServer.global_shader_parameter_set(MASK, null)
	_mask_grid = Vector4.ZERO
	RenderingServer.global_shader_parameter_set(MASK_GRID, _mask_grid)
	_mask_revision = -1
	_mask_texture_revision = -1
	_panel.hide()
