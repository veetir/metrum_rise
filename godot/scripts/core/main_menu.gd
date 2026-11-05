# SPDX-License-Identifier: GPL-2.0-only

## Main menu shell — default startup surface for normal gameplay launches.
## No map or SimulationNode is present here; the player must pick a world or a
## save before gameplay scene loading begins.
extends Control

const NewGameDialog = preload("res://scripts/ui/new_game_dialog.gd")
const OptionsMenu = preload("res://scripts/ui/options_menu.gd")
const UIStyle = preload("res://scripts/ui/ui_style.gd")
const GameSettings = preload("res://scripts/core/game_settings.gd")

const WORLDS_DIR := "user://worlds"
const SAVES_DIR := "user://saves"

var _options_menu = null
var _new_game_dialog: Window = null

func _enter_tree() -> void:
	GameSettings.set_menu_frame_cap(true)

func _exit_tree() -> void:
	GameSettings.set_menu_frame_cap(false)

func _ready() -> void:
	_build_ui()

# Plain text entries down the left side over a soft gradient: no card, no button boxes.
func _build_ui() -> void:
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)

	var gradient := Gradient.new()
	gradient.set_color(0, Color(0.09, 0.11, 0.14))
	gradient.set_color(1, Color(0.025, 0.03, 0.04))
	var fill := GradientTexture2D.new()
	fill.gradient = gradient
	fill.fill_from = Vector2(0.0, 0.0)
	fill.fill_to = Vector2(0.9, 1.0)
	var background := TextureRect.new()
	background.texture = fill
	background.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	background.stretch_mode = TextureRect.STRETCH_SCALE
	background.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	add_child(background)

	var frame := MarginContainer.new()
	frame.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	frame.add_theme_constant_override("margin_left", int(UIStyle.scaled_px(96)))
	add_child(frame)
	var content := VBoxContainer.new()
	content.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
	content.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	content.add_theme_constant_override("separation", int(UIStyle.scaled_px(4)))
	frame.add_child(content)

	# Inset by the entries' text margin, so the title lines up with their text.
	var heading := VBoxContainer.new()
	heading.add_theme_constant_override("separation", 6)
	var inset := MarginContainer.new()
	inset.add_theme_constant_override("margin_left", 14)
	inset.add_child(heading)
	content.add_child(inset)
	var title := Label.new()
	title.text = "METRUM RISE"
	UIStyle.set_font_size(title, 44)
	title.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	heading.add_child(title)
	var rule := ColorRect.new()
	rule.color = UIStyle.HUD_ACCENT
	rule.custom_minimum_size = Vector2(UIStyle.scaled_px(56), 3)
	rule.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
	heading.add_child(rule)
	var gap := Control.new()
	gap.custom_minimum_size.y = UIStyle.scaled_px(36)
	content.add_child(gap)

	content.add_child(_make_main_button("New Game", _on_new_game_pressed))
	content.add_child(_make_main_button("Load Game", _on_load_game_pressed))
	content.add_child(_make_main_button("World Editor", _on_world_editor_pressed))
	content.add_child(_make_main_button("Options", _on_options_pressed))
	content.add_child(_make_main_button("Quit", _on_quit_pressed))

func _make_main_button(label: String, callback: Callable) -> Button:
	var button := UIStyle.menu_text_button(label, 20)
	button.custom_minimum_size.y = UIStyle.scaled_px(44)
	button.pressed.connect(callback)
	return button

func _on_new_game_pressed() -> void:
	_ensure_dir(WORLDS_DIR)
	var dialog := FileDialog.new()
	dialog.access = FileDialog.ACCESS_FILESYSTEM
	dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	dialog.filters = PackedStringArray(["*.sqlite ; WorldDefinition Files"])
	dialog.current_dir = ProjectSettings.globalize_path(WORLDS_DIR)
	dialog.file_selected.connect(func(path: String): _on_new_game_selected(path, dialog))
	dialog.canceled.connect(dialog.queue_free)
	add_child(dialog)
	dialog.popup_centered(Vector2i(880, 620))

func _on_load_game_pressed() -> void:
	_ensure_dir(SAVES_DIR)
	var dialog := FileDialog.new()
	dialog.access = FileDialog.ACCESS_FILESYSTEM
	dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	dialog.filters = PackedStringArray(["*.sqlite ; Save Files"])
	dialog.current_dir = ProjectSettings.globalize_path(SAVES_DIR)
	dialog.file_selected.connect(func(path: String): _on_load_game_selected(path, dialog))
	dialog.canceled.connect(dialog.queue_free)
	add_child(dialog)
	dialog.popup_centered(Vector2i(880, 620))

func _on_world_editor_pressed() -> void:
	var pid := OS.create_instance(["--", "--world-editor"])
	if pid == -1:
		push_error("Failed to launch world editor.")

func _on_options_pressed() -> void:
	if not _options_menu:
		_options_menu = OptionsMenu.new()
		add_child(_options_menu)
	_options_menu.popup_options(OptionsMenu.CONTEXT_MAIN_MENU)

func _on_quit_pressed() -> void:
	get_tree().quit()

func _on_new_game_selected(path: String, dialog: FileDialog) -> void:
	dialog.hide()
	dialog.call_deferred("queue_free")
	# Deferred so the file dialog is gone before the options dialog takes exclusive focus.
	call_deferred("_open_new_game_options", path)

func _open_new_game_options(path: String) -> void:
	if _new_game_dialog == null:
		_new_game_dialog = NewGameDialog.new()
		_new_game_dialog.start_requested.connect(_finish_new_game_selection)
		add_child(_new_game_dialog)
	_new_game_dialog.popup_for_world(path)

func _on_load_game_selected(path: String, dialog: FileDialog) -> void:
	dialog.hide()
	dialog.call_deferred("queue_free")
	call_deferred("_finish_load_game_selection", path)

func _finish_new_game_selection(path: String, vegetation: Dictionary) -> void:
	LaunchState.queue_new_game(path, vegetation)
	get_tree().change_scene_to_file("res://scenes/Main.tscn")

func _finish_load_game_selection(path: String) -> void:
	LaunchState.queue_load_game(path)
	get_tree().change_scene_to_file("res://scenes/Main.tscn")

func _ensure_dir(path: String) -> void:
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(path))
