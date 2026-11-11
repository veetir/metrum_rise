# SPDX-License-Identifier: GPL-2.0-only

## A speed limit picker drawn as Finnish speed limit signs (a yellow disc in a red ring, black
## numerals; the sign alone, no pole): a strip of `full_signs` whole signs (1 or 3) with the
## chosen limit in the middle, larger and lit, and one more sign on each side cut in half by
## the edge, so the strip reads as continuing. The mouse wheel steps one limit (down is
## faster), a click on a sign left or right of the middle picks it.
##
## It reads as a scrollable control: a recessed track whose ends, half signs included, fade
## into the panel (a canvas mask on the strip), ‹ › over a faded end while more limits lie that
## way, and a row of dots under it (one per limit) whose lit pill slides with the value.
## A step slides the strip with a slight overshoot and pops the new sign; scrolling past the
## first or last limit nudges the strip and springs it back. A side sign under the cursor lights.
##
## Keeps SpinBox's surface for its callers: `min_value`, `max_value`, `step`, `value` (setting it
## emits `value_changed`, as SpinBox does) and `set_value_no_signal`. The limits come from Rust's
## speed grid (`get_road_types().speeds`); the picker does not decide which are allowed.
extends Control

const UIStyle = preload("res://scripts/ui/ui_style.gd")

const RING := Color(0.80, 0.10, 0.14)
const DISC := UIStyle.SIGN_YELLOW
const INK := Color(0.06, 0.06, 0.07)
## Base height, the dots row included; the width follows from `full_signs`.
const BASE_HEIGHT := 62.0
# The dots row under the signs, unscaled.
const DOTS_H := 8.0
const SLIDE_S := 0.26
const POP_S := 0.30
const NUDGE_S := 0.32
# How far (in slots) the strip leans when scrolled past an end.
const NUDGE_SLOTS := 0.22
# How much larger the new sign is at the start of its pop.
const POP_SCALE := 0.09
# The neighbours' size and opacity next to the chosen sign; a hovered one lights to HOVER_ALPHA.
const SIDE_SCALE := 0.78
const SIDE_ALPHA := 0.55
const HOVER_ALPHA := 0.85
# The faded band at each end, in sign spacings.
const FADE_SPACINGS := 0.45
# Fades the strip's ends to transparent; `fade_px` is the band's width.
const FADE_SHADER := """
shader_type canvas_item;
uniform float width_px = 100.0;
uniform float fade_px = 20.0;
varying float x;
void vertex() { x = VERTEX.x; }
void fragment() {
	COLOR.a *= smoothstep(0.0, fade_px, x) * smoothstep(0.0, fade_px, width_px - x);
}
"""

static var _fade_shader: Shader

## Emitted with the new limit (km/h) when the player picks one or `value` is set.
signal value_changed(value: float)

## Whole signs in the strip: 1 (the chosen one) or 3; a half sign shows at each edge too.
var full_signs := 1:
	set(v):
		full_signs = 3 if v >= 3 else 1
		_resize()

var min_value := 20.0:
	set(v):
		min_value = v
		_snap()
var max_value := 120.0:
	set(v):
		max_value = v
		_snap()
var step := 10.0:
	set(v):
		step = maxf(v, 1.0)
		_snap()
var value := 50.0:
	set(v):
		var snapped_value := _clamped(v)
		var changed := not is_equal_approx(snapped_value, value)
		value = snapped_value
		if changed:
			_animate_to(value)
			value_changed.emit(value)

# Animated draw state, each redrawing on change: the value the strip is drawn at (it eases to
# `value`, so the signs slide), the new sign's pop (1 → 0) and the end-of-range lean in slots.
var _shown := 50.0:
	set(v):
		_shown = v
		_redraw()
var _pop := 0.0:
	set(v):
		_pop = v
		_redraw()
var _nudge := 0.0:
	set(v):
		_nudge = v
		_redraw()
var _slide_tween: Tween
var _pop_tween: Tween
var _nudge_tween: Tween
# The cursor's x over the strip, or NAN when it is elsewhere.
var _hover_x := NAN
var _font: Font
var _track := StyleBoxFlat.new()
# ‹ › drawn above the mask, so they stay sharp over the faded ends.
var _ends := Control.new()

func _init() -> void:
	focus_mode = Control.FOCUS_NONE
	mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	clip_contents = true
	_resize()
	var bold := FontVariation.new()
	bold.base_font = ThemeDB.fallback_font
	bold.variation_embolden = 0.9
	# Road sign numerals are tall and narrow.
	bold.variation_transform = Transform2D(Vector2(0.86, 0), Vector2(0, 1), Vector2.ZERO)
	_font = bold
	_track.border_width_left = 1
	_track.border_width_top = 1
	_track.border_width_right = 1
	_track.border_width_bottom = 1
	_track.anti_aliasing = true
	if _fade_shader == null:
		_fade_shader = Shader.new()
		_fade_shader.code = FADE_SHADER
	material = ShaderMaterial.new()
	(material as ShaderMaterial).shader = _fade_shader
	_ends.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_ends.set_anchors_preset(Control.PRESET_FULL_RECT)
	_ends.draw.connect(_draw_ends)
	add_child(_ends)
	resized.connect(_update_mask)
	mouse_exited.connect(func():
		_hover_x = NAN
		_redraw())

## Sets the limit without emitting `value_changed`, and without the slide.
func set_value_no_signal(v: float) -> void:
	_set_silently(_clamped(v))

func _set_silently(v: float) -> void:
	set_block_signals(true)
	value = v
	set_block_signals(false)
	for tween in [_slide_tween, _pop_tween, _nudge_tween]:
		if is_instance_valid(tween):
			tween.kill()
	_shown = v
	_pop = 0.0
	_nudge = 0.0

func _get_tooltip(_at_position: Vector2) -> String:
	return tooltip_text if not tooltip_text.is_empty() else "%d km/h · scroll or click a sign to change" % int(value)

func _gui_input(event: InputEvent) -> void:
	var motion := event as InputEventMouseMotion
	if motion:
		_hover_x = motion.position.x
		_redraw()
		return
	var button := event as InputEventMouseButton
	if button == null or not button.pressed:
		return
	match button.button_index:
		MOUSE_BUTTON_WHEEL_DOWN, MOUSE_BUTTON_WHEEL_RIGHT:
			_step_by(1)
			accept_event()
		MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_LEFT:
			_step_by(-1)
			accept_event()
		MOUSE_BUTTON_LEFT:
			# Each sign is one slot wide; a click picks the slot's limit.
			var slots := _slot_at(button.position.x)
			if slots != 0:
				_step_by(slots)
			accept_event()

# Steps `slots` limits; at an end of the range the strip leans that way and springs back.
func _step_by(slots: int) -> void:
	var before := value
	value = value + float(slots) * step
	if is_equal_approx(before, value):
		_lean(signf(slots))

func _draw() -> void:
	var dots_h := UIStyle.scaled_px(DOTS_H)
	var signs := Rect2(Vector2.ZERO, Vector2(size.x, size.y - dots_h))
	var hovered := not is_nan(_hover_x)
	_draw_track(signs, hovered)
	var centre := signs.get_center()
	var spacing := _spacing()
	var radius := _radius()
	# Slide: while easing, the row is drawn `offset` slots from its rest.
	var offset := (_shown - value) / step - _nudge
	var hover_slot := _slot_at(_hover_x) if hovered else 0
	var reach := (full_signs + 1) / 2 + 1
	for i in range(-reach, reach + 1):
		var kmh := value + float(i) * step
		if kmh < min_value - 0.5 or kmh > max_value + 0.5:
			continue
		var at := float(i) + offset
		var near := clampf(1.0 - absf(at), 0.0, 1.0)
		var scale := lerpf(SIDE_SCALE, 1.0, near) * (1.0 + POP_SCALE * _pop * near)
		var alpha := lerpf(SIDE_ALPHA, 1.0, near)
		if i != 0 and i == hover_slot:
			alpha = maxf(alpha, HOVER_ALPHA)
		_draw_sign(Vector2(centre.x + at * spacing, centre.y), radius * scale, int(kmh), alpha)
	_draw_dots(Rect2(0, signs.end.y, size.x, dots_h))

# The recessed pill the signs sit in; a touch lighter under the cursor.
func _draw_track(rect: Rect2, hovered: bool) -> void:
	_track.bg_color = Color(1, 1, 1, 0.07 if hovered else 0.04)
	_track.border_color = Color(1, 1, 1, 0.16 if hovered else 0.09)
	_track.set_corner_radius_all(int(rect.size.y * 0.5))
	draw_style_box(_track, rect.grow_individual(0, -1, 0, -1))

func _draw_sign(at: Vector2, radius: float, kmh: int, alpha: float) -> void:
	draw_circle(at + Vector2(0, radius * 0.06), radius, Color(0, 0, 0, 0.35 * alpha))
	draw_circle(at, radius, Color(RING, alpha))
	draw_circle(at, radius * 0.80, Color(DISC, alpha))
	var text := str(kmh)
	var font_size := int(radius * (0.95 if text.length() <= 2 else 0.78))
	var width := _font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
	var baseline := at.y + (_font.get_ascent(font_size) - _font.get_descent(font_size)) * 0.5
	draw_string(_font, Vector2(at.x - width * 0.5, baseline), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size,
		Color(INK, alpha))

# ‹ and › over the faded ends, each only while more limits lie that way (on `_ends`).
func _draw_ends() -> void:
	var rect := Rect2(Vector2.ZERO, Vector2(size.x, size.y - UIStyle.scaled_px(DOTS_H)))
	var hovered := not is_nan(_hover_x)
	var h := rect.size.y * 0.16
	var w := h * 0.6
	var inset := UIStyle.scaled_px(9.0)
	var color := Color(1, 1, 1, 0.95 if hovered else 0.7)
	var y := rect.get_center().y
	var shadow := Color(0, 0, 0, 0.55)
	for side in [-1.0, 1.0]:
		var more := value > min_value + 0.5 if side < 0.0 else value < max_value - 0.5
		if not more:
			continue
		var tip_x := rect.position.x + inset if side < 0.0 else rect.end.x - inset
		var points := PackedVector2Array([Vector2(tip_x - side * w, y - h), Vector2(tip_x, y), Vector2(tip_x - side * w, y + h)])
		_ends.draw_polyline(points, shadow, UIStyle.scaled_px(4.0), true)
		_ends.draw_polyline(points, color, UIStyle.scaled_px(2.0), true)

# One dot per limit; a lit pill at the shown value slides along them.
func _draw_dots(rect: Rect2) -> void:
	var count := int(roundf((max_value - min_value) / step)) + 1
	if count < 2:
		return
	var gap := UIStyle.scaled_px(5.0)
	var y := rect.get_center().y
	var x0 := rect.get_center().x - gap * float(count - 1) * 0.5
	var dot := UIStyle.scaled_px(1.3)
	for i in range(count):
		draw_circle(Vector2(x0 + gap * float(i), y), dot, Color(1, 1, 1, 0.22), true, -1.0, true)
	var at := x0 + gap * ((_shown - min_value) / step + _nudge * 0.5)
	var half := UIStyle.scaled_px(3.2)
	draw_line(Vector2(at - half, y), Vector2(at + half, y), UIStyle.HUD_ACCENT, dot * 2.0, true)

func _redraw() -> void:
	queue_redraw()
	_ends.queue_redraw()

func _update_mask() -> void:
	var mask := material as ShaderMaterial
	mask.set_shader_parameter("width_px", size.x)
	mask.set_shader_parameter("fade_px", _spacing() * FADE_SPACINGS)

# The slot (signs from the middle) under x.
func _slot_at(x: float) -> int:
	return roundi((x - size.x * 0.5) / _spacing())

# Centre to centre of neighbouring signs.
func _spacing() -> float:
	return _radius() * 2.15

# Wide enough for the whole signs and half of one more at each edge.
func _resize() -> void:
	var height := UIStyle.scaled_px(BASE_HEIGHT)
	var radius := (height - UIStyle.scaled_px(DOTS_H)) * 0.46
	custom_minimum_size = Vector2(radius * 2.15 * float(full_signs + 1), height)
	_redraw()

func _radius() -> float:
	return (size.y - UIStyle.scaled_px(DOTS_H)) * 0.46

func _clamped(v: float) -> float:
	if step <= 0.0:
		return clampf(v, min_value, max_value)
	return clampf(min_value + roundf((v - min_value) / step) * step, min_value, max_value)

func _snap() -> void:
	_set_silently(_clamped(value))

# The slide to `target` with a slight overshoot, and the new sign's pop.
func _animate_to(target: float) -> void:
	if not is_inside_tree():
		_shown = target
		return
	if is_instance_valid(_slide_tween):
		_slide_tween.kill()
	_slide_tween = create_tween().set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
	_slide_tween.tween_property(self, "_shown", target, SLIDE_S)
	if is_instance_valid(_pop_tween):
		_pop_tween.kill()
	_pop = 1.0
	_pop_tween = create_tween().set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
	_pop_tween.tween_property(self, "_pop", 0.0, POP_S)

# Scrolled past an end: lean `direction` slots-ward and spring back.
func _lean(direction: float) -> void:
	if not is_inside_tree():
		return
	if is_instance_valid(_nudge_tween):
		_nudge_tween.kill()
	_nudge_tween = create_tween()
	_nudge_tween.tween_property(self, "_nudge", direction * NUDGE_SLOTS, NUDGE_S * 0.3) \
		.set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
	_nudge_tween.tween_property(self, "_nudge", 0.0, NUDGE_S * 0.7) \
		.set_trans(Tween.TRANS_ELASTIC).set_ease(Tween.EASE_OUT)
