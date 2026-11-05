# SPDX-License-Identifier: GPL-2.0-only

## In-game pause menu: Esc with nothing left to cancel opens it over a dimmed screen. A view:
## each button emits `action_requested` with a TopMenu file action id (or RESUME), and TopMenu
## runs it. Leaving the game (main menu, quit) asks first, as nothing tracks unsaved changes.
extends CanvasLayer

const UIStyle = preload("res://scripts/ui/ui_style.gd")
## Closes the menu; not a TopMenu action.
const RESUME := -1

## The player picked `id`: RESUME or a TopMenu.ActionId file action.
signal action_requested(id: int)

var _confirm: ConfirmationDialog
var _pending_id := RESUME

## Builds the menu, hidden. `items` lists [label, id, confirm title] in order; a non-empty
## confirm title asks before emitting (leaving the game).
func build(items: Array) -> void:
	# Above the HUD and the menu bar; dialogs are subwindows and still draw over it.
	layer = 110
	visible = false
	var backdrop := ColorRect.new()
	backdrop.color = Color(0, 0, 0, 0.55)
	backdrop.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	backdrop.mouse_filter = Control.MOUSE_FILTER_STOP
	add_child(backdrop)
	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(center)
	var card := PanelContainer.new()
	var style := UIStyle.window_body_style()
	style.content_margin_left = 28
	style.content_margin_right = 28
	style.content_margin_top = 22
	style.content_margin_bottom = 24
	card.add_theme_stylebox_override("panel", style)
	center.add_child(card)
	var column := VBoxContainer.new()
	column.add_theme_constant_override("separation", 10)
	card.add_child(column)
	var title := Label.new()
	title.text = "Paused"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	UIStyle.set_font_size(title, 22)
	title.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	column.add_child(title)
	for item in items:
		var button := Button.new()
		button.text = str(item[0])
		button.custom_minimum_size = Vector2(260, 44)
		button.focus_mode = Control.FOCUS_NONE
		UIStyle.set_font_size(button, 15)
		var id := int(item[1])
		var ask := str(item[2])
		button.pressed.connect(func():
			if ask.is_empty():
				action_requested.emit(id)
			else:
				_ask(id, ask))
		column.add_child(button)
	_confirm = ConfirmationDialog.new()
	_confirm.dialog_text = "Unsaved progress will be lost."
	_confirm.confirmed.connect(func(): action_requested.emit(_pending_id))
	add_child(_confirm)

func _ask(id: int, title: String) -> void:
	_pending_id = id
	_confirm.title = title
	_confirm.ok_button_text = title
	_confirm.popup_centered()
