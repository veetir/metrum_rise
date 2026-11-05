# SPDX-License-Identifier: GPL-2.0-only

## Options, drawn in the game's own UI over a dimmed screen (main menu and gameplay): a Display
## tab (graphics and interface scale) and a Mods tab (installed packs). The panels own their
## pending values and persistence; this shell owns the tabs and the Reset / Cancel / Apply
## footer. Esc and Cancel discard pending edits; it takes the keyboard while open.
extends CanvasLayer

const AccessibilityOptions = preload("res://scripts/ui/accessibility_options.gd")
const GameSettings = preload("res://scripts/core/game_settings.gd")
const GraphicsOptions = preload("res://scripts/ui/graphics_options.gd")
const PackManager = preload("res://scripts/ui/pack_manager.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")

## Opened from the main menu.
const CONTEXT_MAIN_MENU := "main_menu"
## Opened in game (File menu or the pause menu).
const CONTEXT_GAMEPLAY := "gameplay"
const TAB_DISPLAY := "display"
const TAB_MODS := "mods"
# The card's design size; it shrinks to 92 % of a smaller screen.
const CARD_SIZE := Vector2(860, 580)

## The menu closed (Cancel, Esc or the close button).
signal closed

var context: String = CONTEXT_GAMEPLAY

# Tab id -> its button, its page and the option panels on it.
var _tab_buttons: Dictionary = {}
var _pages: Dictionary = {}
var _panels: Dictionary = {}
var _active_tab := TAB_DISPLAY
var _card: PanelContainer
var _apply_btn: Button
var _reset_btn: Button
var _status_label: Label

func _ready() -> void:
	# Above the pause menu; option drop-downs are subwindows and still draw over it.
	layer = 120
	visible = false
	_build_ui()
	var saved := str(GameSettings.load_options_window_state().get("active_category", TAB_DISPLAY))
	_select_tab(TAB_MODS if saved == TAB_MODS else TAB_DISPLAY)

## Shows the menu with every panel re-read from the saved settings.
func popup_options(next_context: String) -> void:
	context = next_context
	_discard_pending_changes()
	var screen := get_viewport().get_visible_rect().size
	_card.custom_minimum_size = Vector2(
		minf(UIStyle.scaled_px(CARD_SIZE.x), screen.x * 0.92),
		minf(UIStyle.scaled_px(CARD_SIZE.y), screen.y * 0.92))
	show()

## Whether the menu is up.
func is_open() -> bool:
	return visible

func _input(event: InputEvent) -> void:
	if not visible or not (event is InputEventKey) or not event.pressed:
		return
	if event.keycode == KEY_ESCAPE and not event.echo:
		_on_cancel_pressed()
	# Nothing behind the menu reacts to keys while it is open.
	get_viewport().set_input_as_handled()

func _build_ui() -> void:
	var backdrop := ColorRect.new()
	backdrop.color = Color(0, 0, 0, 0.6)
	backdrop.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	backdrop.mouse_filter = Control.MOUSE_FILTER_STOP
	add_child(backdrop)
	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(center)
	_card = PanelContainer.new()
	var style := UIStyle.panel_style(UIStyle.BG_DARK, UIStyle.CORNER_PANEL, UIStyle.HUD_ACCENT.darkened(0.55))
	style.content_margin_left = 24
	style.content_margin_right = 24
	style.content_margin_top = 18
	style.content_margin_bottom = 18
	_card.add_theme_stylebox_override("panel", style)
	center.add_child(_card)
	var layout := VBoxContainer.new()
	layout.add_theme_constant_override("separation", 14)
	_card.add_child(layout)

	# Header: title, the tabs beside it, close at the right.
	var header := HBoxContainer.new()
	header.add_theme_constant_override("separation", 18)
	layout.add_child(header)
	var title := Label.new()
	title.text = "Options"
	UIStyle.set_font_size(title, 20)
	title.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	header.add_child(title)
	var tabs := HBoxContainer.new()
	tabs.add_theme_constant_override("separation", 4)
	tabs.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_child(tabs)
	var close := Button.new()
	close.text = "×"
	close.flat = true
	close.focus_mode = Control.FOCUS_NONE
	close.tooltip_text = "Close (Esc)"
	UIStyle.set_font_size(close, 16)
	close.pressed.connect(_on_cancel_pressed)
	header.add_child(close)
	layout.add_child(HSeparator.new())

	var pages := MarginContainer.new()
	pages.size_flags_vertical = Control.SIZE_EXPAND_FILL
	layout.add_child(pages)
	_add_tab(tabs, pages, TAB_DISPLAY, "Display", [GraphicsOptions.new(), AccessibilityOptions.new()])
	_add_tab(tabs, pages, TAB_MODS, "Mods", [PackManager.new()])

	var footer := HBoxContainer.new()
	footer.add_theme_constant_override("separation", 8)
	layout.add_child(footer)
	_reset_btn = _footer_button(footer, "Reset Defaults", _on_reset_defaults_pressed)
	_status_label = Label.new()
	_status_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_status_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	UIStyle.set_font_size(_status_label, 12)
	_status_label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	footer.add_child(_status_label)
	_footer_button(footer, "Cancel", _on_cancel_pressed)
	_apply_btn = _footer_button(footer, "Apply", _on_apply_pressed)

# One tab: its button and a scrolling page holding `panels` one under another.
func _add_tab(tabs: HBoxContainer, pages: Control, id: String, label: String, panels: Array) -> void:
	var button := Button.new()
	button.text = label
	button.toggle_mode = true
	button.focus_mode = Control.FOCUS_NONE
	button.custom_minimum_size = Vector2(UIStyle.scaled_px(96), 0)
	UIStyle.set_font_size(button, 14)
	var idle := StyleBoxEmpty.new()
	var lit := UIStyle.panel_style(UIStyle.HUD_ACCENT.darkened(0.7), UIStyle.CORNER_SUB, Color.TRANSPARENT, 0)
	lit.content_margin_top = 6
	lit.content_margin_bottom = 6
	for state in ["normal", "hover", "focus"]:
		button.add_theme_stylebox_override(state, idle)
	for state in ["pressed", "hover_pressed"]:
		button.add_theme_stylebox_override(state, lit)
	button.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	button.add_theme_color_override("font_hover_color", UIStyle.TEXT_PRIMARY)
	button.add_theme_color_override("font_pressed_color", UIStyle.HUD_ACCENT.lightened(0.3))
	button.add_theme_color_override("font_hover_pressed_color", UIStyle.HUD_ACCENT.lightened(0.3))
	button.pressed.connect(func(): _select_tab(id))
	tabs.add_child(button)
	_tab_buttons[id] = button
	var scroll := ScrollContainer.new()
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	pages.add_child(scroll)
	var column := VBoxContainer.new()
	column.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	column.add_theme_constant_override("separation", 26)
	scroll.add_child(column)
	for panel: Control in panels:
		panel.size_flags_vertical = Control.SIZE_SHRINK_BEGIN
		panel.dirty_changed.connect(func(_dirty: bool): _sync_footer_state())
		column.add_child(panel)
	_pages[id] = scroll
	_panels[id] = panels

func _footer_button(footer: HBoxContainer, label: String, action: Callable) -> Button:
	var button := Button.new()
	button.text = label
	button.focus_mode = Control.FOCUS_NONE
	button.custom_minimum_size.x = UIStyle.scaled_px(88)
	UIStyle.set_font_size(button, 14)
	button.pressed.connect(action)
	footer.add_child(button)
	return button

func _select_tab(id: String) -> void:
	_active_tab = id
	for key in _pages:
		(_pages[key] as Control).visible = key == id
		(_tab_buttons[key] as Button).set_pressed_no_signal(key == id)
	GameSettings.save_options_category(id)
	_sync_footer_state()

func _all_panels() -> Array:
	var all := []
	for key in _panels:
		all.append_array(_panels[key])
	return all

func _on_reset_defaults_pressed() -> void:
	for panel in _panels[_active_tab]:
		if panel.has_method("reset_defaults"):
			panel.reset_defaults()
	_sync_footer_state()

func _on_cancel_pressed() -> void:
	_discard_pending_changes()
	hide()
	closed.emit()

func _on_apply_pressed() -> void:
	var restart_required := false
	for panel in _all_panels():
		if not panel.has_pending_changes():
			continue
		if panel.apply_changes() != OK:
			_status_label.text = "Could not apply settings."
			_sync_footer_state()
			return
		if panel is PackManager:
			restart_required = true
	_status_label.text = "Saved. Restart to load the pack changes." if restart_required else "Saved."
	_sync_footer_state()

func _discard_pending_changes() -> void:
	for panel in _all_panels():
		panel.refresh()
	if _status_label:
		_status_label.text = ""
	_sync_footer_state()

func _sync_footer_state() -> void:
	if not _apply_btn:
		return
	var dirty := false
	for panel in _all_panels():
		dirty = dirty or panel.has_pending_changes()
	_apply_btn.disabled = not dirty
	var resettable := false
	for panel in _panels.get(_active_tab, []):
		resettable = resettable or panel.has_method("reset_defaults")
	_reset_btn.disabled = not resettable
	if dirty:
		_status_label.text = "Unsaved changes."
	elif _status_label.text == "Unsaved changes.":
		_status_label.text = ""
