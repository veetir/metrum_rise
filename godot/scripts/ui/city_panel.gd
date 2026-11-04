# SPDX-License-Identifier: GPL-2.0-only

## Tab "status check" panel sliding in from the left: treasury, agents and each zone's growth
## with Rust's own explanation lines (`get_growth_overview`). A read-only view: MainUI feeds it
## the values it already polls for the status notch, only while it is open.
extends Control

const UIStyle = preload("res://scripts/ui/ui_style.gd")
const CityStatusPanel = preload("res://scripts/ui/city_status_panel.gd")
const TopMenu = preload("res://scripts/ui/top_menu.gd")
const ZONES := ["residential", "commercial", "industrial"]
const ZONE_COLORS := {
	"residential": UIStyle.ZONE_RESIDENTIAL,
	"commercial": UIStyle.ZONE_COMMERCIAL,
	"industrial": UIStyle.ZONE_INDUSTRIAL,
}
const WIDTH := 340.0
const SLIDE_S := 0.16

var _panel: PanelContainer
var _treasury: Label
var _agents: Label
var _zone_rows: Dictionary = {}
var _tween: Tween
var _open := false

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	# Content height, below the menu bar; only the panel itself takes the mouse.
	set_anchors_and_offsets_preset(Control.PRESET_TOP_LEFT)
	offset_top = TopMenu.BAR_HEIGHT + 12
	_panel = PanelContainer.new()
	var style := UIStyle.panel_style(UIStyle.BG_DARK, UIStyle.CORNER_SUB, UIStyle.HUD_ACCENT.darkened(0.55))
	style.corner_radius_top_left = 0
	style.corner_radius_bottom_left = 0
	style.border_width_left = 0
	style.content_margin_left = 18
	style.content_margin_right = 18
	style.content_margin_top = 14
	style.content_margin_bottom = 14
	_panel.add_theme_stylebox_override("panel", style)
	_panel.custom_minimum_size.x = WIDTH
	_panel.add_to_group("debug_console_keep_clear")
	add_child(_panel)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 12)
	_panel.add_child(column)

	var header := HBoxContainer.new()
	column.add_child(header)
	var title := _label("City", 18, UIStyle.TEXT_PRIMARY)
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(title)
	header.add_child(_label("Tab", 11, UIStyle.TEXT_DIM))

	var stats := HBoxContainer.new()
	stats.add_theme_constant_override("separation", 8)
	column.add_child(stats)
	stats.add_child(_icon("money", Color.WHITE))
	_treasury = _label("0", 15, UIStyle.TEXT_PRIMARY)
	_treasury.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	stats.add_child(_treasury)
	stats.add_child(_icon("agents", UIStyle.TEXT_DIM))
	_agents = _label("0", 15, UIStyle.TEXT_PRIMARY)
	stats.add_child(_agents)

	column.add_child(_section("Growth"))
	var zones := VBoxContainer.new()
	zones.add_theme_constant_override("separation", 14)
	column.add_child(zones)
	for zone in ZONES:
		zones.add_child(_zone_card(zone))
	_panel.position.x = -WIDTH - 20
	hide()

## Whether MainUI should feed it values.
func is_open() -> bool:
	return _open

## Slides the panel in or out.
func toggle() -> void:
	_open = not _open
	if is_instance_valid(_tween):
		_tween.kill()
	if _open:
		show()
	_tween = create_tween().set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
	_tween.tween_property(_panel, "position:x", 0.0 if _open else -_panel.size.x - 20, SLIDE_S)
	if not _open:
		_tween.tween_callback(hide)

## Treasury and agent count, formatted like the status notch.
func set_stats(treasury: float, agents: int) -> void:
	_treasury.text = CityStatusPanel.format_currency(treasury)
	_agents.text = CityStatusPanel.format_count(agents)

## One entry per zone: `projects` and Rust's multi-line `detail`, as `get_growth_overview` gives.
func set_growth(growth: Dictionary) -> void:
	var scale := 1
	for zone in ZONES:
		scale = maxi(scale, int(Dictionary(growth.get(zone, {})).get("projects", 0)))
	for zone in ZONES:
		var entry: Dictionary = growth.get(zone, {})
		var row: Dictionary = _zone_rows[zone]
		var projects := maxi(0, int(entry.get("projects", 0)))
		row.count.text = "%d project%s" % [projects, "" if projects == 1 else "s"]
		row.fill.anchor_right = float(projects) / float(scale)
		var lines := str(entry.get("detail", "")).split("\n", false)
		var body: VBoxContainer = row.body
		for child in body.get_children():
			body.remove_child(child)
			child.queue_free()
		for line in lines:
			# Rust's first line is the "Latest hourly assessment" heading; the counts follow.
			if line.begins_with("Latest hourly"):
				continue
			var label := _label(line, 12, UIStyle.TEXT_DIM)
			label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			body.add_child(label)

func _zone_card(zone: String) -> Control:
	var color := Color(ZONE_COLORS[zone], 1.0).lightened(0.3)
	var card := VBoxContainer.new()
	card.add_theme_constant_override("separation", 4)
	var head := HBoxContainer.new()
	head.add_theme_constant_override("separation", 8)
	card.add_child(head)
	head.add_child(_icon(zone, color))
	var title := _label(zone.capitalize(), 14, UIStyle.TEXT_PRIMARY)
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(title)
	var count := _label("0 projects", 12, color)
	head.add_child(count)
	var track := ColorRect.new()
	track.color = Color(color, 0.18)
	track.custom_minimum_size.y = 4
	card.add_child(track)
	var fill := ColorRect.new()
	fill.color = color
	fill.anchor_bottom = 1.0
	fill.anchor_right = 0.0
	track.add_child(fill)
	var body := VBoxContainer.new()
	body.add_theme_constant_override("separation", 2)
	card.add_child(body)
	_zone_rows[zone] = {"count": count, "fill": fill, "body": body}
	return card

func _section(text: String) -> Label:
	return _label(text.to_upper(), 11, UIStyle.TEXT_SECTION)

func _icon(icon_name: String, tint: Color) -> TextureRect:
	var icon := TextureRect.new()
	icon.texture = load("res://assets/ui/icons/%s.svg" % icon_name)
	icon.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	icon.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	icon.custom_minimum_size = Vector2(18, 18)
	icon.modulate = tint
	return icon

func _label(text: String, font_size: int, color: Color) -> Label:
	var label := Label.new()
	label.text = text
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	UIStyle.set_font_size(label, font_size)
	label.add_theme_color_override("font_color", color)
	return label
