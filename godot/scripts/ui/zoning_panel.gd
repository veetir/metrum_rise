# SPDX-License-Identifier: GPL-2.0-only

## Zoning palette structure and tiles; MainUI retains the profile selection handlers.
extends VBoxContainer

const HudIcons = preload("res://scripts/ui/hud_icons.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")
var profile_panel: PanelContainer
var profile_menu: HBoxContainer
var mode_menu: HBoxContainer
var type_menu: HBoxContainer
var options_button: Button
var cell_controls: HBoxContainer

func build() -> void:
	size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	add_theme_constant_override("separation", 8)
	profile_menu = HBoxContainer.new()
	profile_menu.alignment = BoxContainer.ALIGNMENT_CENTER
	profile_panel = HudIcons.shell(profile_menu)
	add_child(profile_panel)
	profile_panel.hide()
	cell_controls = preload("res://scripts/ui/zoning_cell_controls.gd").new()
	# Added by the controller once the tool and workflow listener are attached.
	var row := HBoxContainer.new()
	row.alignment = BoxContainer.ALIGNMENT_CENTER
	add_child(row)
	mode_menu = HBoxContainer.new()
	row.add_child(mode_menu)
	options_button = HudIcons.icon_button("gear", "Parcel options", "Parcel width, depth and gap")
	mode_menu.add_child(options_button)
	options_button.hide()
	type_menu = HBoxContainer.new()
	type_menu.add_theme_constant_override("separation", 6)
	row.add_child(HudIcons.shell(type_menu))

static func tile(zone: String, title: String, detail: String, profile_color: String = "") -> Button:
	var icon := zone if zone in ["residential", "commercial", "industrial"] else "zoning"
	var button := HudIcons.icon_button(icon, title, detail, 64)
	button.toggle_mode = true
	var color: Color = {
		"residential": UIStyle.ZONE_RESIDENTIAL,
		"commercial": UIStyle.ZONE_COMMERCIAL,
		"industrial": UIStyle.ZONE_INDUSTRIAL,
	}.get(zone, UIStyle.TEXT_PRIMARY)
	if not profile_color.is_empty():
		color = Color.from_string(profile_color, color)
	HudIcons.tint(button, color.lightened(0.25))
	return button
