# SPDX-License-Identifier: GPL-3.0-or-later

## Displays one sidewalk's width with the same stepper layout as the median control.
## The host supplies Rust's width grid and applies the changes Rust accepts.
extends HBoxContainer

signal changed(width_m: float)

const UIStyle = preload("res://scripts/ui/ui_style.gd")

var grid: Dictionary = {}
var _width_m := 0.0
var _minus: Button
var _plus: Button
var _value: Label

func _init() -> void:
	add_theme_constant_override("separation", 2)
	_minus = _button("-", -1)
	_value = Label.new()
	_value.custom_minimum_size.x = 52.0
	_value.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	UIStyle.set_font_size(_value, 13)
	add_child(_value)
	_plus = _button("+", 1)

func _button(text: String, direction: int) -> Button:
	var button := Button.new()
	button.text = text
	button.focus_mode = Control.FOCUS_NONE
	button.custom_minimum_size.x = 28.0
	UIStyle.set_font_size(button, 13)
	button.pressed.connect(_step.bind(direction))
	add_child(button)
	return button

## Displays the width in metres. Zero shows an absent sidewalk.
func set_width_no_signal(width_m: float) -> void:
	_width_m = width_m
	_value.text = "%.1f m" % _width_m if _width_m > 0.0 else "None"
	_minus.disabled = _width_m <= float(grid.get("min_m", 0.0))
	_plus.disabled = _width_m <= 0.0 or _width_m >= float(grid.get("max_m", 0.0))

func _step(direction: int) -> void:
	var width_m := clampf(_width_m + direction * float(grid.step_m), grid.min_m, grid.max_m)
	if width_m != _width_m:
		changed.emit(width_m)
