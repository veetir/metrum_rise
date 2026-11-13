# SPDX-License-Identifier: GPL-2.0-only

## Persistent, non-exclusive feedback for optional road-type pockets refused by Rust.
## Uses the inspector card and the shared road reason translations (`ROAD-114`).
extends Window

const Card = preload("res://scripts/ui/inspector_card.gd")
const HudIcons = preload("res://scripts/ui/hud_icons.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")
const RoadReasonText = preload("res://scripts/ui/road_reason_text.gd")
const DIRECTIONS := ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

## Shows every refused approach, identified by its junction position and incoming direction.
func show_refusals(refusals: Array) -> void:
	size = Vector2i(560, 300)
	exclusive = false
	close_requested.connect(queue_free)
	window_input.connect(func(event: InputEvent):
		if event is InputEventKey and event.pressed and event.keycode == KEY_ESCAPE:
			queue_free())
	var parts := Card.build(self)
	HudIcons.set_icon(parts.icon, "road")
	parts.eyebrow.text = "Turn pockets"
	parts.title.text = "Road built"
	parts.subtitle.text = "These approaches could not receive their pockets"
	var scroll := ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.custom_minimum_size.y = 180
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	parts.content.add_child(scroll)
	var rows := VBoxContainer.new()
	rows.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	rows.add_theme_constant_override("separation", 12)
	scroll.add_child(rows)
	for refusal: Dictionary in refusals:
		var at: Vector3 = refusal.get("position", Vector3.ZERO)
		var direction: String = DIRECTIONS[posmod(roundi(float(refusal.get("bearing_deg", 0.0)) / 45.0), 8)]
		var label := Label.new()
		label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		label.text = "From %s at (%.0f, %.0f)\n%s" % [direction, at.x, at.z, RoadReasonText.text(refusal)]
		UIStyle.set_font_size(label, 13)
		label.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
		rows.add_child(label)
	UIStyle.place_window_near_cursor(self, Vector2.ZERO, get_parent().get_viewport())
	popup()
