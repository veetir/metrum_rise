# SPDX-License-Identifier: GPL-2.0-only

## A speed limit picker drawn as Finnish speed limit signs (a yellow disc in a red ring, black
## numerals; the sign alone, no pole): a strip of `full_signs` whole signs (1 or 3) with the
## chosen limit in the middle, larger and lit, and one more sign on each side cut in half by
## the edge, so the strip reads as continuing. The mouse wheel steps one limit (down is
## faster), a click on a sign left or right of the middle picks it, and the strip slides.
##
## Keeps SpinBox's surface for its callers: `min_value`, `max_value`, `step`, `value` (setting it
## emits `value_changed`, as SpinBox does) and `set_value_no_signal`. The limits come from Rust's
## speed grid (`get_road_types().speeds`); the picker does not decide which are allowed.
extends Control

const UIStyle = preload("res://scripts/ui/ui_style.gd")

const RING := Color(0.80, 0.10, 0.14)
const DISC := UIStyle.SIGN_YELLOW
const INK := Color(0.06, 0.06, 0.07)
## Base height; the width follows from `full_signs`.
const BASE_HEIGHT := 56.0
const SLIDE_S := 0.14
# The neighbours' size and opacity next to the chosen sign.
const SIDE_SCALE := 0.78
const SIDE_ALPHA := 0.55

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
		_slide_to(value)
		if changed:
			value_changed.emit(value)

# The value the row is drawn at; it eases to `value`, so the signs slide.
var _shown := 50.0:
	set(v):
		_shown = v
		queue_redraw()
var _tween: Tween
var _font: Font

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

## Sets the limit without emitting `value_changed`, and without the slide.
func set_value_no_signal(v: float) -> void:
	_set_silently(_clamped(v))

func _set_silently(v: float) -> void:
	set_block_signals(true)
	value = v
	set_block_signals(false)
	if is_instance_valid(_tween):
		_tween.kill()
	_shown = v

func _get_tooltip(_at_position: Vector2) -> String:
	return tooltip_text if not tooltip_text.is_empty() else "%d km/h · scroll or click a sign to change" % int(value)

func _gui_input(event: InputEvent) -> void:
	var button := event as InputEventMouseButton
	if button == null or not button.pressed:
		return
	match button.button_index:
		MOUSE_BUTTON_WHEEL_DOWN, MOUSE_BUTTON_WHEEL_RIGHT:
			value = value + step
			accept_event()
		MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_LEFT:
			value = value - step
			accept_event()
		MOUSE_BUTTON_LEFT:
			# Each sign is one slot wide; a click picks the slot's limit.
			var slots := roundi((button.position.x - size.x * 0.5) / _spacing())
			value = value + float(slots) * step
			accept_event()

func _draw() -> void:
	var centre := size * 0.5
	var spacing := _spacing()
	# Slide: while easing, the row is drawn `offset` steps from its rest.
	var offset := (_shown - value) / step if step > 0.0 else 0.0
	var reach := (full_signs + 1) / 2 + 1
	for i in range(-reach, reach + 1):
		var kmh := value + float(i) * step
		if kmh < min_value - 0.5 or kmh > max_value + 0.5:
			continue
		var at := float(i) + offset
		var near := clampf(1.0 - absf(at), 0.0, 1.0)
		var scale := lerpf(SIDE_SCALE, 1.0, near)
		var alpha := lerpf(SIDE_ALPHA, 1.0, near)
		_draw_sign(Vector2(centre.x + at * spacing, centre.y), _radius() * scale, int(kmh), alpha)

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

# Centre to centre of neighbouring signs.
func _spacing() -> float:
	return _radius() * 2.15

# Wide enough for the whole signs and half of one more at each edge.
func _resize() -> void:
	var height := UIStyle.scaled_px(BASE_HEIGHT)
	custom_minimum_size = Vector2(height * 0.46 * 2.15 * float(full_signs + 1), height)
	queue_redraw()

func _radius() -> float:
	return size.y * 0.46

func _clamped(v: float) -> float:
	if step <= 0.0:
		return clampf(v, min_value, max_value)
	return clampf(min_value + roundf((v - min_value) / step) * step, min_value, max_value)

func _snap() -> void:
	_set_silently(_clamped(value))

func _slide_to(target: float) -> void:
	if not is_inside_tree():
		_shown = target
		return
	if is_instance_valid(_tween):
		_tween.kill()
	_tween = create_tween().set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_OUT)
	_tween.tween_property(self, "_shown", target, SLIDE_S)
