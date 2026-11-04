# SPDX-License-Identifier: GPL-2.0-only

## Event-driven HUD tooltip. Reads the owner's text and tooltip_text when shown, so library
## edits and selection changes need no second copy of the content. Never takes mouse focus.
extends Node

const UIStyle = preload("res://scripts/ui/ui_style.gd")
const DELAY := 0.15
## Control meta holding the shortcut key drawn as a keycap; see HudIcons.set_shortcut.
const SHORTCUT_META := &"hud_shortcut"

var _button: Control
var _title := ""
var _timer: Timer
var _layer: CanvasLayer
var _card: PanelContainer

## Installs one delayed tooltip on a HUD button; the button suppresses its native tooltip.
func attach(button: Control, title: String = "") -> void:
	_button = button
	_title = title
	button.add_child(self)
	_timer = Timer.new()
	_timer.one_shot = true
	_timer.wait_time = DELAY
	add_child(_timer)
	_timer.timeout.connect(_show)
	button.mouse_entered.connect(func(): _timer.start())
	button.mouse_exited.connect(_hide)
	if button is BaseButton:
		button.button_down.connect(_hide)
	button.visibility_changed.connect(_hide)
	button.resized.connect(_hide)

func _show() -> void:
	if not _button.is_visible_in_tree() or not _button.get_global_rect().has_point(_button.get_global_mouse_position()):
		return
	_layer = CanvasLayer.new()
	_layer.layer = 100
	add_child(_layer)
	var viewport_size := _button.get_viewport_rect().size
	var width := minf(UIStyle.scaled_px(310), viewport_size.x - 60)
	var title: String = _button.text if _button is Button else _title
	_card = view(title, _button.tooltip_text, width, str(_button.get_meta(SHORTCUT_META, "")))
	_card.visible = false
	_card.resized.connect(_place, CONNECT_DEFERRED)
	_layer.add_child(_card)
	_place.call_deferred()

func _place() -> void:
	if not is_instance_valid(_card):
		return
	_card.reset_size()
	var bounds := _button.get_viewport_rect()
	var anchor := _button.get_global_rect()
	var at := Vector2(
		anchor.get_center().x - _card.size.x * 0.5, anchor.position.y - _card.size.y - 12)
	if at.y < 12:
		at.y = anchor.end.y + 12
	at.x = clampf(at.x, 12, maxf(12, bounds.size.x - _card.size.x - 12))
	at.y = clampf(at.y, 12, maxf(12, bounds.size.y - _card.size.y - 12))
	_card.position = at.round()
	_card.show()

func _hide() -> void:
	_timer.stop()
	if is_instance_valid(_layer):
		_layer.hide()
		_layer.queue_free()
	_layer = null
	_card = null

## Builds a wrapped, scale-aware card; road cost/upkeep (the last line) receives emphasis.
static func view(title: String, body: String, width: float, key: String = "") -> PanelContainer:
	var panel := PanelContainer.new()
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var style := UIStyle.panel_style(
		UIStyle.BG_DARK, UIStyle.CORNER_SUB, UIStyle.HUD_ACCENT.darkened(0.45))
	style.content_margin_left = 14
	style.content_margin_right = 14
	style.content_margin_top = 12
	style.content_margin_bottom = 12
	style.shadow_color = Color(0, 0, 0, 0.25)
	style.shadow_size = 6
	panel.add_theme_stylebox_override("panel", style)
	var column := VBoxContainer.new()
	column.mouse_filter = Control.MOUSE_FILTER_IGNORE
	column.add_theme_constant_override("separation", 8)
	column.custom_minimum_size.x = width
	panel.add_child(column)
	var heading := HBoxContainer.new()
	heading.mouse_filter = Control.MOUSE_FILTER_IGNORE
	heading.add_theme_constant_override("separation", 10)
	column.add_child(heading)
	_label(heading, title, 16, UIStyle.TEXT_PRIMARY).size_flags_horizontal = Control.SIZE_EXPAND_FILL
	if not key.is_empty():
		heading.add_child(_keycap(key))
	for line in body.split("\n", false):
		_label(column, line, 13, UIStyle.HUD_ACCENT if line.contains("upkeep") else UIStyle.TEXT_DIM)
	return panel

static func _label(parent: Control, text: String, font_size: int, color: Color) -> Label:
	var label := Label.new()
	label.text = text
	label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	UIStyle.set_font_size(label, font_size)
	label.add_theme_color_override("font_color", color)
	parent.add_child(label)
	return label

# A small raised key: light rim, darker face, a thicker bottom edge like a physical key.
static func _keycap(key: String) -> PanelContainer:
	var cap := PanelContainer.new()
	cap.mouse_filter = Control.MOUSE_FILTER_IGNORE
	cap.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	var style := UIStyle.panel_style(Color(1, 1, 1, 0.10), 5, Color(1, 1, 1, 0.45))
	style.border_width_bottom = 3
	style.content_margin_left = 7
	style.content_margin_right = 7
	style.content_margin_top = 1
	style.content_margin_bottom = 1
	cap.add_theme_stylebox_override("panel", style)
	_label(cap, key, 12, UIStyle.TEXT_PRIMARY).autowrap_mode = TextServer.AUTOWRAP_OFF
	return cap
