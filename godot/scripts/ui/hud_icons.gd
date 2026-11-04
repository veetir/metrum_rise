# SPDX-License-Identifier: GPL-2.0-only

## Shared icon/tile factories and interruptible hover feedback for the bottom HUD. Native
## Button text remains available to callers; only the styled tooltip paints the name.
extends RefCounted

const UIStyle = preload("res://scripts/ui/ui_style.gd")
const Tooltip = preload("res://scripts/ui/hud_tooltip.gd")
const RoadThumbnail = preload("res://scripts/ui/hud_road_thumbnail.gd")
const ICON_DIR := "res://assets/ui/icons/"
const TILE_SIZE := 72.0
const FEEDBACK := &"HUDFeedback"

class IconButton extends Button:
	func _get_tooltip(_at_position: Vector2) -> String:
		return ""

class MenuTile extends MenuButton:
	func _get_tooltip(_at_position: Vector2) -> String:
		return ""

## A square, tintable icon button. Detail includes the existing shortcut, when one exists.
static func icon_button(
	icon_name: String, title: String, detail: String, side: float = 44.0
) -> Button:
	var button := IconButton.new()
	button.text = title
	button.tooltip_text = detail
	decorate(button, icon_name, side)
	return button

## A square thumbnail tile; content uses Rust's parts and per-part widths.
static func road_tile(type: Dictionary, detail: String) -> Button:
	var button := icon_button("road", String(type.get("name", "Road")), detail, TILE_SIZE)
	set_road(button, type)
	button.toggle_mode = true
	return button

## A bounded menu tile retaining MenuButton's popup API and item signals.
static func menu_tile(title: String, detail: String) -> MenuButton:
	var button := MenuTile.new()
	button.text = title
	button.tooltip_text = detail
	decorate(button, "custom_roads", TILE_SIZE)
	return button

## Attaches one hover helper and tooltip. Icons are authored at 48 px for a 24 px design grid.
static func decorate(button: Button, icon_name: String, side: float) -> void:
	button.custom_minimum_size = Vector2.ONE * side
	button.size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	button.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	button.focus_mode = Control.FOCUS_NONE
	button.clip_text = true
	UIStyle.set_font_size(button, 12)
	for state in ["normal", "hover", "pressed", "hover_pressed", "focus", "disabled"]:
		button.add_theme_stylebox_override(state, StyleBoxEmpty.new())
	for state in [
		"font_color", "font_hover_color", "font_pressed_color", "font_hover_pressed_color",
		"font_focus_color", "font_disabled_color",
	]:
		button.add_theme_color_override(state, Color.TRANSPARENT)
	var feedback := HoverFeedback.new()
	feedback.name = FEEDBACK
	feedback.button = button
	feedback.texture = load(ICON_DIR + icon_name + ".svg") as Texture2D
	button.add_child(feedback)
	feedback.attach()
	Tooltip.new().attach(button)

## Swaps the artwork, e.g. pause/resume.
static func set_icon(button: Button, icon_name: String) -> void:
	var feedback: HoverFeedback = button.get_node(NodePath(FEEDBACK))
	feedback.texture = load(ICON_DIR + icon_name + ".svg") as Texture2D
	button.queue_redraw()

## The key the tooltip shows as a keycap beside the title. Buttons without a binding show none.
static func set_shortcut(button: Control, key: String) -> void:
	button.set_meta(Tooltip.SHORTCUT_META, key)

## A small digit at the tile's bottom-right edge naming its number key; empty removes it.
static func set_number(button: Button, digit: String) -> void:
	var feedback: HoverFeedback = button.get_node(NodePath(FEEDBACK))
	feedback.number = digit
	button.queue_redraw()

## Acts as a click would: flips a toggle (a grouped one only on), then emits `pressed`.
static func activate(button: Button) -> void:
	if button.disabled or not button.is_visible_in_tree():
		return
	if button.toggle_mode and not (button.button_group and button.button_pressed):
		button.button_pressed = not button.button_pressed
	button.pressed.emit()

## Tints artwork without tinting the shared shell or selected accent underline.
static func tint(button: Button, color: Color) -> void:
	var feedback: HoverFeedback = button.get_node(NodePath(FEEDBACK))
	feedback.ink = Color(color, 1.0)
	feedback.tinted = true
	button.queue_redraw()

## A padded, compact palette shell shared by zoning, assets and the status bar.
static func shell(content: Control) -> PanelContainer:
	var panel := PanelContainer.new()
	var style := UIStyle.submenu_style()
	style.content_margin_left = 10
	style.content_margin_right = 10
	style.content_margin_top = 6
	style.content_margin_bottom = 6
	panel.add_theme_stylebox_override("panel", style)
	panel.add_child(content)
	return panel

## Replaces the custom tile's preview after a selection or library edit; empty resets its icon.
static func set_road(button: Button, type: Dictionary) -> void:
	var feedback: HoverFeedback = button.get_node(NodePath(FEEDBACK))
	feedback.road = null
	if not type.is_empty():
		feedback.road = RoadThumbnail.new()
		feedback.road.setup(type)
	button.queue_redraw()

## Sets a persistent mark without triggering selection handlers (also works for a MenuButton).
static func mark(button: Button, selected: bool) -> void:
	if button is MenuButton:
		var feedback: HoverFeedback = button.get_node(NodePath(FEEDBACK))
		feedback.menu_selected = selected
	else:
		button.set_pressed_no_signal(selected)
	button.queue_redraw()

## One helper per control. Drawing transforms the artwork, keeping the container's hit box
## stable. No process callback: redraws follow native button state and the 0.12 s event tween.
class HoverFeedback extends Node:
	var button: Button
	var texture: Texture2D
	var ink := UIStyle.TEXT_PRIMARY
	var tinted := false
	var road: RoadThumbnail
	var menu_selected := false
	var number := ""
	var _hovered := false
	var _down := false
	var _tween: Tween
	var _style := UIStyle.panel_style(UIStyle.BG_HUD_GROUP, UIStyle.CORNER_SUB, Color.TRANSPARENT, 0)
	var _amount := 0.0:
		set(value):
			_amount = value
			button.queue_redraw()
	var _zoom := 1.0:
		set(value):
			_zoom = value
			button.queue_redraw()

	func attach() -> void:
		button.draw.connect(_draw)
		button.mouse_entered.connect(func():
			_hovered = true
			_animate())
		button.mouse_exited.connect(func():
			_hovered = false
			_animate())
		button.button_down.connect(func():
			_down = true
			_animate())
		button.button_up.connect(func():
			_down = false
			_animate())
		button.visibility_changed.connect(_reset)

	func _reset() -> void:
		if is_instance_valid(_tween):
			_tween.kill()
		_hovered = false
		_down = false
		_amount = 0.0
		_zoom = 1.0

	func _animate() -> void:
		if is_instance_valid(_tween):
			_tween.kill()
		_tween = create_tween().set_parallel(true).set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
		_tween.tween_property(self, "_amount", 1.0 if _hovered and not button.disabled else 0.0, 0.12)
		var zoom := 0.97 if _down else (1.08 if _hovered else 1.0)
		_tween.tween_property(self, "_zoom", 1.0 if button.disabled else zoom, 0.12)

	func _draw() -> void:
		var selected := menu_selected or (button.toggle_mode and button.button_pressed)
		var amount := 0.0 if button.disabled else _amount
		var rect := Rect2(Vector2.ZERO, button.size).grow(-3.0)
		var zoom := 1.0 if button.disabled else _zoom
		button.draw_set_transform(button.size * 0.5 * (1.0 - zoom), 0.0, Vector2.ONE * zoom)
		_style.bg_color = UIStyle.BG_HUD_GROUP.lerp(UIStyle.BG_SUBMENU.lightened(0.16), amount)
		if selected:
			_style.bg_color = _style.bg_color.lerp(UIStyle.HUD_ACCENT.darkened(0.65), 0.65)
		button.draw_style_box(_style, rect)
		var paint := ink
		if selected:
			paint = ink.lightened(0.18) if tinted else UIStyle.HUD_ACCENT
		if button.disabled:
			paint = Color(UIStyle.TEXT_DIM, 0.4)
		if road != null:
			road.paint(button, rect.grow(-8.0))
		else:
			# The scale-aware font size is already refreshed by Accessibility; no settings I/O.
			# Small buttons (the status row in the menu bar) keep the icon at 3/4 of their face.
			var side := minf(2.0 * button.get_theme_font_size("font_size"), rect.size.x * 0.75)
			var box := Rect2(rect.get_center() - Vector2.ONE * side * 0.5, Vector2.ONE * side)
			button.draw_texture_rect(texture, box, false, paint)
		var accent := UIStyle.HUD_ACCENT
		accent.a = 1.0 if selected else amount * 0.75
		if button.disabled:
			accent.a *= 0.35
		button.draw_line(
			Vector2(rect.position.x + 12, rect.end.y - 3),
			Vector2(rect.end.x - 12, rect.end.y - 3), accent, 2.0, true)
		button.draw_set_transform(Vector2.ZERO)
		if not number.is_empty():
			_draw_number(rect, selected)

	# Unscaled by the hover zoom, right of the accent underline, outlined to read over thumbnails.
	func _draw_number(rect: Rect2, selected: bool) -> void:
		var font := button.get_theme_font("font")
		var font_size := maxi(8, roundi(button.get_theme_font_size("font_size") * 0.8))
		var width := font.get_string_size(number, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
		var at := Vector2(rect.end.x - width - 4.0, rect.end.y - 3.0)
		var color := UIStyle.HUD_ACCENT if selected else UIStyle.TEXT_DIM
		if button.disabled:
			color = Color(UIStyle.TEXT_DIM, 0.4)
		button.draw_string_outline(font, at, number, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, 3, Color(0, 0, 0, 0.7))
		button.draw_string(font, at, number, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)
