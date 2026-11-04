# SPDX-License-Identifier: GPL-2.0-only

## Slim status row below the shared menu. Speed values come directly from Rust's supported steps.
extends MarginContainer

const UIStyle = preload("res://scripts/ui/ui_style.gd")
const HudIcons = preload("res://scripts/ui/hud_icons.gd")
const TopMenu = preload("res://scripts/ui/top_menu.gd")
var clock_label: Label
var city_status_panel: Control
var demand_meter: Control
var _speed_buttons: Array[Button] = []
var _speed_steps: PackedFloat32Array

func build(input_manager: Node) -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	set_anchors_and_offsets_preset(Control.PRESET_TOP_LEFT)
	offset_left = UIStyle.HUD_LEFT_MARGIN
	offset_top = TopMenu.BAR_HEIGHT + 6
	_bind_menu.call_deferred()
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	var panel := HudIcons.shell(row)
	panel.add_to_group("debug_console_keep_clear")
	add_child(panel)
	clock_label = Label.new()
	clock_label.text = "Day 1 00:00"
	UIStyle.set_font_size(clock_label, 13)
	row.add_child(clock_label)
	var speeds := HBoxContainer.new()
	speeds.add_theme_constant_override("separation", 0)
	row.add_child(speeds)
	_speed_steps = SimulationNode.get_simulation_speed_steps()
	for index in range(_speed_steps.size()):
		var speed := float(_speed_steps[index])
		var title := "Paused" if speed == 0.0 else "%gx speed" % speed
		var icon: String = ["pause", "play", "fast", "faster"][mini(index, 3)]
		var button := HudIcons.icon_button(icon, title, "Space — pause / resume" if speed == 0.0 else title, 32)
		button.toggle_mode = true
		button.pressed.connect(func(): input_manager.set_simulation_speed(speed))
		var step := VBoxContainer.new()
		step.add_theme_constant_override("separation", 0)
		speeds.add_child(step)
		step.add_child(button)
		var value := Label.new()
		value.text = "%gx" % speed
		value.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		value.mouse_filter = Control.MOUSE_FILTER_IGNORE
		UIStyle.set_font_size(value, 10)
		value.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
		step.add_child(value)
		_speed_buttons.append(button)
	row.add_child(VSeparator.new())
	city_status_panel = preload("res://scripts/ui/city_status_panel.gd").new()
	row.add_child(city_status_panel)
	row.add_child(VSeparator.new())
	demand_meter = preload("res://scripts/ui/demand_meter.gd").new()
	demand_meter.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	row.add_child(demand_meter)

func mark_speed(speed: float) -> void:
	for i in range(_speed_buttons.size()):
		HudIcons.mark(_speed_buttons[i], is_equal_approx(speed, _speed_steps[i]))

# MainScene attaches TopMenu after child _ready calls; bind once after that construction.
func _bind_menu() -> void:
	for control in get_tree().get_nodes_in_group("debug_console_keep_clear"):
		if control.get_parent().get_parent() is TopMenu:
			control.resized.connect(func(): offset_top = control.get_global_rect().end.y + 6)
			offset_top = control.get_global_rect().end.y + 6
			return
