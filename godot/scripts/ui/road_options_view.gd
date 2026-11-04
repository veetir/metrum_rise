# SPDX-License-Identifier: GPL-2.0-only

## Compact road-option view. The host owns tool state and connects the existing handlers.
extends VBoxContainer

const HudIcons = preload("res://scripts/ui/hud_icons.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")

var straight_btn: Button
var spline_btn: Button
var snap_btn: Button
var tunnel_btn: Button

func _init() -> void:
	add_theme_constant_override("separation", 5)
	_heading(self, "Tool mode")
	var modes := HBoxContainer.new()
	modes.alignment = BoxContainer.ALIGNMENT_CENTER
	add_child(modes)
	straight_btn = HudIcons.icon_button(
		"straight", "Straight", "Draw a straight road between its ends.", 38)
	spline_btn = HudIcons.icon_button(
		"spline", "Spline", "Draw a curved road using control points.", 38)
	for button in [straight_btn, spline_btn]:
		button.toggle_mode = true
		modes.add_child(button)
	straight_btn.set_pressed_no_signal(true)
	var options := HBoxContainer.new()
	options.add_theme_constant_override("separation", 10)
	add_child(options)
	snap_btn = HudIcons.icon_button("snap_to_grid", "Snap to zoning grid",
		"Snap near parallel or perpendicular cell lines; draw other angles freely. "
		+ "Accounts for road widths. Existing road connections take priority. Straight mode only.", 38)
	tunnel_btn = HudIcons.icon_button("tunnel", "Tunnel",
		"T — while drawing a road\nKeep the drawn height through the ground. "
		+ "Place both ends outside the hill; the stretch under 5 m of ground becomes a tunnel.", 38)
	for entry in [["Snapping", snap_btn], ["Structure", tunnel_btn]]:
		var column := VBoxContainer.new()
		options.add_child(column)
		_heading(column, entry[0])
		var button: Button = entry[1]
		button.toggle_mode = true
		column.add_child(button)

func _heading(parent: Control, text: String) -> void:
	var label := Label.new()
	label.text = text
	label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	UIStyle.set_font_size(label, 11)
	label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	parent.add_child(label)
