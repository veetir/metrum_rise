# SPDX-License-Identifier: GPL-2.0-only

## Save, Load and New Game file lists, drawn in the game's own UI like Options: one folder
## (`user://saves` or `user://worlds`), its `.sqlite` files only, by name without the extension
## with the time saved and size, newest first. Load and New Game pick a row; Save takes a name
## (a row fills it in) and says before it replaces a save. A view: the caller's callback gets
## the full path. Takes the keyboard while open; Esc cancels, Enter confirms, Up/Down move.
extends CanvasLayer

const OptionsRows = preload("res://scripts/ui/options_rows.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")

## Pick a save to load.
const MODE_LOAD := 0
## Name a save to write.
const MODE_SAVE := 1
## Pick a world for a new game.
const MODE_WORLD := 2
## The only file type listed; the extension is never shown.
const EXTENSION := "sqlite"
const COLUMN_WIDTH := 760.0
const NODE_NAME := "SaveBrowser"
# Mode -> title, confirm label, text for an empty folder.
const TEXT := {
	MODE_LOAD: ["Load Game", "Load", "No saved games yet."],
	MODE_SAVE: ["Save Game", "Save", "No saved games yet."],
	MODE_WORLD: ["New Game", "Continue", "No worlds in the worlds folder."],
}

## The browser closed, chosen or not.
signal closed

var _mode := MODE_LOAD
var _dir := ""
# Called with the chosen path; replaced on every open.
var _on_chosen := Callable()
var _column: VBoxContainer
var _title: Label
var _scroll: ScrollContainer
var _list: VBoxContainer
var _empty: Label
var _name_row: Control
var _name_edit: LineEdit
var _status: Label
var _confirm: Button
var _group := ButtonGroup.new()
# Row buttons in list order, each with meta "file" (its file name).
var _rows: Array[Button] = []

## Opens the browser that lives under `parent`, making it on first use, and returns it.
## `on_chosen` gets the confirmed file's native path (in `dir`, with its extension).
static func open(parent: Node, mode: int, dir: String, on_chosen: Callable, default_name: String = ""):
	var browser = parent.get_node_or_null(NODE_NAME)
	if browser == null:
		browser = load("res://scripts/ui/save_browser.gd").new()
		browser.name = NODE_NAME
		parent.add_child(browser)
	browser.popup_browse(mode, dir, on_chosen, default_name)
	return browser

func _ready() -> void:
	# With Options: above the pause menu and the HUD.
	layer = 120
	visible = false
	_build_ui()

## Lists `dir` for `mode`; Save starts with `default_name` (extension optional) in the name field.
func popup_browse(mode: int, dir: String, on_chosen: Callable, default_name: String = "") -> void:
	_mode = mode
	_on_chosen = on_chosen
	_dir = dir
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(dir))
	var text: Array = TEXT[mode]
	_title.text = str(text[0]).to_upper()
	_confirm.text = str(text[1]).to_upper()
	_empty.text = str(text[2])
	_name_row.visible = mode == MODE_SAVE
	_name_edit.text = _display_name(default_name)
	_refresh_list()
	var screen := get_viewport().get_visible_rect().size
	_column.custom_minimum_size.x = minf(UIStyle.scaled_px(COLUMN_WIDTH), screen.x * 0.92)
	OptionsRows.fade_covered(get_tree(), 0.0)
	show()
	if mode == MODE_SAVE:
		_name_edit.grab_focus()
		_name_edit.select_all()
	_sync_state()

## Whether the browser is up.
func is_open() -> bool:
	return visible

func _input(event: InputEvent) -> void:
	if not visible or not (event is InputEventKey) or not event.pressed:
		return
	match event.keycode:
		KEY_ESCAPE:
			_close()
		KEY_ENTER, KEY_KP_ENTER:
			_on_confirm()
		KEY_UP:
			_move_selection(-1)
		KEY_DOWN:
			_move_selection(1)
		_:
			return
	get_viewport().set_input_as_handled()

# Keys the name field does not take stop here, so typing never steers the city behind.
func _unhandled_input(event: InputEvent) -> void:
	if visible and event is InputEventKey:
		get_viewport().set_input_as_handled()

func _build_ui() -> void:
	var backdrop := ColorRect.new()
	backdrop.color = Color(0.02, 0.03, 0.04, 0.86)
	backdrop.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	backdrop.mouse_filter = Control.MOUSE_FILTER_STOP
	add_child(backdrop)
	var frame := MarginContainer.new()
	frame.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	frame.mouse_filter = Control.MOUSE_FILTER_IGNORE
	frame.add_theme_constant_override("margin_top", int(UIStyle.scaled_px(72)))
	frame.add_theme_constant_override("margin_bottom", int(UIStyle.scaled_px(48)))
	add_child(frame)
	_column = VBoxContainer.new()
	_column.size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	_column.add_theme_constant_override("separation", 0)
	frame.add_child(_column)

	_title = Label.new()
	UIStyle.set_font_size(_title, 30)
	_title.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	_title.custom_minimum_size.y = UIStyle.scaled_px(56)
	_column.add_child(_title)
	_column.add_child(OptionsRows.hairline())

	_scroll = ScrollContainer.new()
	_scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	_scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_column.add_child(_scroll)
	var page := VBoxContainer.new()
	page.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	page.add_theme_constant_override("separation", 0)
	_scroll.add_child(page)
	_list = VBoxContainer.new()
	_list.add_theme_constant_override("separation", 0)
	page.add_child(_list)
	_empty = Label.new()
	_empty.custom_minimum_size.y = UIStyle.scaled_px(56)
	_empty.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	UIStyle.set_font_size(_empty, 14)
	_empty.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	page.add_child(_empty)

	_column.add_child(OptionsRows.hairline())
	_name_edit = LineEdit.new()
	_name_edit.placeholder_text = "Name"
	_name_edit.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	UIStyle.set_font_size(_name_edit, 15)
	var field := StyleBoxFlat.new()
	field.bg_color = Color(1, 1, 1, 0.06)
	field.border_width_bottom = 1
	field.border_color = Color(1, 1, 1, 0.25)
	field.content_margin_left = 10
	field.content_margin_right = 10
	field.content_margin_top = 6
	field.content_margin_bottom = 6
	var focused: StyleBoxFlat = field.duplicate()
	focused.border_color = UIStyle.HUD_ACCENT
	_name_edit.add_theme_stylebox_override("normal", field)
	_name_edit.add_theme_stylebox_override("focus", focused)
	_name_edit.text_changed.connect(func(_text: String): _on_name_edited())
	var name_line := HBoxContainer.new()
	name_line.add_theme_constant_override("separation", 16)
	var name_label := Label.new()
	name_label.text = "Save as"
	UIStyle.set_font_size(name_label, 15)
	name_line.add_child(name_label)
	name_line.add_child(_name_edit)
	_name_row = OptionsRows.frame(name_line)
	_name_row.custom_minimum_size.y = UIStyle.scaled_px(56)
	_column.add_child(_name_row)

	var footer := HBoxContainer.new()
	footer.add_theme_constant_override("separation", int(UIStyle.scaled_px(24)))
	_column.add_child(footer)
	var folder := _footer_button(footer, "Open folder", func(): OS.shell_open(ProjectSettings.globalize_path(_dir)))
	folder.tooltip_text = "Show the folder in the file manager."
	_status = Label.new()
	_status.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_status.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_status.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	UIStyle.set_font_size(_status, 13)
	_status.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	footer.add_child(_status)
	_footer_button(footer, "Cancel", _close)
	_confirm = _footer_button(footer, "Load", _on_confirm)
	_confirm.add_theme_color_override("font_color", UIStyle.HUD_ACCENT)
	_confirm.add_theme_color_override("font_hover_color", UIStyle.HUD_ACCENT.lightened(0.4))

func _footer_button(footer: HBoxContainer, label: String, action: Callable) -> Button:
	var button := UIStyle.menu_text_button(label, 14, false)
	button.custom_minimum_size.y = UIStyle.scaled_px(48)
	button.pressed.connect(action)
	footer.add_child(button)
	return button

func _refresh_list() -> void:
	for row in _rows:
		_list.remove_child(row)
		row.queue_free()
	_rows.clear()
	var entries := []
	var native := ProjectSettings.globalize_path(_dir)
	for file in DirAccess.get_files_at(native):
		if file.get_extension().to_lower() == EXTENSION and not file.begins_with("."):
			var path := native.path_join(file)
			entries.append([FileAccess.get_modified_time(path), file, _file_size(path)])
	# Newest first; equal times by name, so the order is stable.
	entries.sort_custom(func(a, b): return a[0] > b[0] or (a[0] == b[0] and a[1] < b[1]))
	var now := int(Time.get_unix_time_from_system())
	for entry in entries:
		var row := _row(str(entry[1]), _when(int(entry[0]), now), _size_text(int(entry[2])))
		_list.add_child(row)
		_rows.append(row)
	_empty.visible = _rows.is_empty()
	if _mode != MODE_SAVE and not _rows.is_empty():
		_rows[0].button_pressed = true

func _row(file: String, when: String, size: String) -> Button:
	var row := Button.new()
	row.toggle_mode = true
	row.button_group = _group
	row.focus_mode = Control.FOCUS_NONE
	row.set_meta("file", file)
	row.custom_minimum_size.y = UIStyle.scaled_px(OptionsRows.ROW_HEIGHT)
	var idle := StyleBoxFlat.new()
	idle.bg_color = Color.TRANSPARENT
	idle.border_width_bottom = 1
	idle.border_color = Color(1, 1, 1, 0.07)
	var hover: StyleBoxFlat = idle.duplicate()
	hover.bg_color = Color(1, 1, 1, 0.05)
	var picked: StyleBoxFlat = idle.duplicate()
	picked.bg_color = Color(UIStyle.HUD_ACCENT, 0.12)
	picked.border_width_left = 2
	picked.border_color = UIStyle.HUD_ACCENT
	for state in ["normal", "focus", "disabled"]:
		row.add_theme_stylebox_override(state, idle)
	row.add_theme_stylebox_override("hover", hover)
	row.add_theme_stylebox_override("pressed", picked)
	row.add_theme_stylebox_override("hover_pressed", picked)
	var line := HBoxContainer.new()
	line.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	line.offset_left = 12
	line.offset_right = -12
	line.add_theme_constant_override("separation", 24)
	line.mouse_filter = Control.MOUSE_FILTER_IGNORE
	row.add_child(line)
	var name_label := _cell(line, _display_name(file), 15, UIStyle.TEXT_PRIMARY)
	name_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	name_label.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	_cell(line, when, 13, UIStyle.TEXT_DIM)
	var size_label := _cell(line, size, 13, UIStyle.TEXT_DIM)
	size_label.custom_minimum_size.x = UIStyle.scaled_px(64)
	size_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	row.toggled.connect(func(on: bool):
		if on:
			_on_row_picked(row))
	row.gui_input.connect(func(event: InputEvent):
		if event is InputEventMouseButton and event.double_click and event.button_index == MOUSE_BUTTON_LEFT:
			_on_confirm())
	return row

func _cell(line: HBoxContainer, text: String, font_size: int, color: Color) -> Label:
	var label := Label.new()
	label.text = text
	label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	UIStyle.set_font_size(label, font_size)
	label.add_theme_color_override("font_color", color)
	line.add_child(label)
	return label

func _on_row_picked(row: Button) -> void:
	if _mode == MODE_SAVE:
		_name_edit.text = _display_name(str(row.get_meta("file")))
	_sync_state()

# Typing a name selects the save it matches, or none.
func _on_name_edited() -> void:
	var file := _typed_file()
	for row in _rows:
		row.set_pressed_no_signal(str(row.get_meta("file")) == file)
	_sync_state()

func _move_selection(step: int) -> void:
	if _rows.is_empty():
		return
	var index := _rows.find(_group.get_pressed_button())
	index = clampi(index + step, 0, _rows.size() - 1) if index >= 0 else 0
	_rows[index].button_pressed = true
	_scroll.ensure_control_visible(_rows[index])

func _on_confirm() -> void:
	if _confirm.disabled:
		return
	var file := _typed_file() if _mode == MODE_SAVE else str(_group.get_pressed_button().get_meta("file"))
	# A native path, as Rust opens the file itself.
	var path := ProjectSettings.globalize_path(_dir.path_join(file))
	_close()
	_on_chosen.call(path)

func _close() -> void:
	if not visible:
		return
	hide()
	OptionsRows.fade_covered(get_tree(), 1.0)
	closed.emit()

# The confirm button and the status line say why a choice cannot go ahead.
func _sync_state() -> void:
	var reason := ""
	if _mode == MODE_SAVE:
		var typed := _name_edit.text.strip_edges()
		if typed.is_empty():
			reason = "Type a name for the save."
		elif not typed.is_valid_filename():
			reason = "A name cannot contain : / \\ ? * \" | % < >."
		elif FileAccess.file_exists(_dir.path_join(_typed_file())):
			_status.text = "Replaces the existing save."
			_confirm.disabled = false
			_confirm.text = "REPLACE"
			return
		_confirm.text = "SAVE"
	elif _group.get_pressed_button() == null:
		reason = "" if _rows.is_empty() else "Pick one from the list."
	_confirm.disabled = not reason.is_empty() or (_mode != MODE_SAVE and _rows.is_empty())
	_status.text = reason

func _typed_file() -> String:
	return "%s.%s" % [_name_edit.text.strip_edges(), EXTENSION]

static func _display_name(file: String) -> String:
	return file.get_basename() if file.get_extension().to_lower() == EXTENSION else file

static func _file_size(path: String) -> int:
	var file := FileAccess.open(path, FileAccess.READ)
	return file.get_length() if file else 0

## "Today 08:03", "Yesterday 21:10" or "3 Nov 2026 14:00", in local time.
static func _when(unix: int, now: int) -> String:
	var bias := int(Time.get_time_zone_from_system().get("bias", 0)) * 60
	var local := Time.get_datetime_dict_from_unix_time(unix + bias)
	var day := Time.get_date_string_from_unix_time(unix + bias)
	var clock := "%02d:%02d" % [local.hour, local.minute]
	if day == Time.get_date_string_from_unix_time(now + bias):
		return "Today " + clock
	if day == Time.get_date_string_from_unix_time(now + bias - 86400):
		return "Yesterday " + clock
	const MONTHS := ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
	return "%d %s %d %s" % [local.day, MONTHS[local.month - 1], local.year, clock]

static func _size_text(bytes: int) -> String:
	if bytes >= 1024 * 1024:
		return "%d MB" % roundi(bytes / 1048576.0)
	return "%d KB" % maxi(1, roundi(bytes / 1024.0))
