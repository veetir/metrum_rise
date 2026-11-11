# SPDX-License-Identifier: GPL-2.0-only

## Building inspector window manager backed by Godot's built-in Window chrome.
##
## Call try_inspect() with a world position to populate and show a per-building
## window. Only one inspector is open at a time: inspecting another building
## replaces it, and inspecting the same building again closes it. The open window
## refreshes only when the in-game hour changes, so the steady-state cost is O(1)
## per frame. The window never keeps the keyboard, so the camera can move while it is open.
##
## Laid out like Road Properties: the building's icon, name and a dim line (zone, level, business
## status) lead, alerts follow in red or yellow, and the details sit in tabs (Overview, Money,
## Site, Power, Stock, Parking). A tab with nothing to show is left out, the last tab picked stays
## picked for the next building, and Parking carries a warning icon when places are rejected. The
## window fits its height to the open tab; the tabs sit at the top, so only the bottom edge moves.
extends Node

const UIStyle = preload("res://scripts/ui/ui_style.gd")
const HudIcons = preload("res://scripts/ui/hud_icons.gd")

const SECTION_FONT_SIZE := 13
const ROW_FONT_SIZE := 14
const DEFAULT_SIZE := Vector2i(420, 360)
const MIN_SIZE := Vector2i(360, 140)
## Tab order; empty tabs are dropped per building.
const TABS: Array[String] = ["Overview", "Money", "Site", "Power", "Stock", "Parking"]

# The tab the player last picked, kept for the next building inspected this session.
static var _last_tab := "Overview"

@onready var simulation_node = $"../SimulationNode"

var _open_windows: Dictionary = {}
var _last_observed_hour: int = -1

func _process(_delta: float) -> void:
	if _open_windows.is_empty() or simulation_node == null:
		return

	var absolute_hour := _current_absolute_hour()
	if absolute_hour < 0 or absolute_hour == _last_observed_hour:
		return

	_last_observed_hour = absolute_hour
	_refresh_open_windows()

## Opens the inspector for the building at `world_pos`. Inspecting the open building again
## closes it, unless `toggle` is false (a link to the building), which keeps it open.
func try_inspect(world_pos: Vector3, screen_pos: Vector2 = Vector2.ZERO, toggle := true) -> bool:
	var info: Dictionary = simulation_node.get_building_info_at(world_pos.x, world_pos.z)
	if info.is_empty():
		close_window()
		return false

	var key := _building_key(info)
	var entry: Dictionary = _open_windows.get(key, {})
	if not entry.is_empty():
		if toggle:
			_close_entry(key)
		return true

	close_window()
	entry = _create_window_entry(key)
	_open_windows[key] = entry

	_populate(entry, info)
	_update_anchor(entry, info)

	var window: Window = entry["window"]
	if entry.has("body"):
		_fit_window(window, entry["body"])
	_place_window_near_cursor(window, screen_pos)
	window.show()
	_last_observed_hour = _current_absolute_hour()
	return true

## Whether the inspector currently owns a selection, used by the two-stage Escape action.
func is_inspecting() -> bool:
	return not _open_windows.is_empty()

func close_window() -> void:
	var keys := _open_windows.keys()
	for key_variant in keys:
		_close_entry(str(key_variant))

func _refresh_open_windows() -> void:
	var keys := _open_windows.keys()
	for key_variant in keys:
		var key := str(key_variant)
		var entry: Dictionary = _open_windows.get(key, {})
		if entry.is_empty():
			continue

		var anchor_pos: Vector3 = entry.get("anchor_pos", Vector3.ZERO)
		var info: Dictionary = simulation_node.get_building_info_at(anchor_pos.x, anchor_pos.z)
		if info.is_empty():
			_close_entry(key)
			continue

		_populate(entry, info)
		_update_anchor(entry, info)

func _current_absolute_hour() -> int:
	var hour_of_day := floori(float(simulation_node.get_current_minute_of_day()) / 60.0)
	return int(simulation_node.get_current_day()) * 24 + hour_of_day

func _create_window_entry(key: String) -> Dictionary:
	var window := Window.new()
	window.title = "Building Inspector"
	window.unresizable = true
	window.exclusive = false
	window.visible = false
	window.close_requested.connect(_close_entry.bind(key))
	# The camera keys, zoom and Esc keep working while the window is open.
	UIStyle.keep_keyboard_with_game(window)
	# Content-sized like Road Properties; the height follows the open tab.
	UIStyle.set_window_base_size(window, DEFAULT_SIZE, MIN_SIZE, get_viewport())

	var body := PanelContainer.new()
	body.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	body.add_theme_stylebox_override("panel", UIStyle.window_body_style())
	window.add_child(body)

	var margin := MarginContainer.new()
	margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	margin.add_theme_constant_override("margin_left", UIStyle.PAD_WINDOW)
	margin.add_theme_constant_override("margin_right", UIStyle.PAD_WINDOW)
	margin.add_theme_constant_override("margin_top", UIStyle.PAD_WINDOW)
	margin.add_theme_constant_override("margin_bottom", UIStyle.PAD_WINDOW)
	body.add_child(margin)

	var root_vbox := VBoxContainer.new()
	root_vbox.add_theme_constant_override("separation", 10)
	margin.add_child(root_vbox)

	var header := HBoxContainer.new()
	header.add_theme_constant_override("separation", 12)
	root_vbox.add_child(header)
	var icon := HudIcons.icon_button("services", "", "", 48)
	icon.mouse_filter = Control.MOUSE_FILTER_IGNORE
	header.add_child(icon)
	var names := VBoxContainer.new()
	names.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	names.alignment = BoxContainer.ALIGNMENT_CENTER
	header.add_child(names)
	var title_label := Label.new()
	title_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	UIStyle.set_font_size(title_label, 16)
	names.add_child(title_label)
	var subtitle_label := Label.new()
	subtitle_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	subtitle_label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	UIStyle.set_font_size(subtitle_label, 13)
	names.add_child(subtitle_label)

	var stats_body := VBoxContainer.new()
	stats_body.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	stats_body.add_theme_constant_override("separation", 6)
	root_vbox.add_child(stats_body)

	add_child(window)
	window.hide()

	return {
		"window": window,
		"body": body,
		"icon": icon,
		"title_label": title_label,
		"subtitle_label": subtitle_label,
		"stats_body": stats_body,
		"anchor_pos": Vector3.ZERO,
	}

# The height follows the content (the open tab), capped to the screen.
func _fit_window(window: Window, body: Control) -> void:
	if not is_instance_valid(window) or not is_instance_valid(body):
		return
	var cap := int(get_viewport().get_visible_rect().size.y * 0.9)
	window.size.y = mini(int(body.get_combined_minimum_size().y), cap)

func _close_entry(key: String) -> void:
	var entry: Dictionary = _open_windows.get(key, {})
	if entry.is_empty():
		return

	_open_windows.erase(key)
	var window: Window = entry["window"]
	if is_instance_valid(window):
		window.queue_free()

func _update_anchor(entry: Dictionary, info: Dictionary) -> void:
	entry["anchor_pos"] = Vector3(
		float(info.get("center_x", 0.0)),
		0.0,
		float(info.get("center_z", 0.0))
	)

func _building_key(info: Dictionary) -> String:
	return "%.3f:%.3f" % [
		float(info.get("center_x", 0.0)),
		float(info.get("center_z", 0.0)),
	]

func _display_name(info: Dictionary) -> String:
	var asset_display_name := str(info.get("asset_display_name", "")).strip_edges()
	if not asset_display_name.is_empty():
		return asset_display_name
	var asset_id: String = info.get("asset_id", "Building")
	var colon := asset_id.find(":")
	return asset_id.substr(colon + 1) if colon != -1 else asset_id

func _place_window_near_cursor(window: Window, screen_pos: Vector2) -> void:
	UIStyle.place_window_near_cursor(window, screen_pos, get_viewport())

func _populate(entry: Dictionary, info: Dictionary) -> void:
	entry["info"] = info
	var title_label: Label = entry["title_label"]
	var stats_body: VBoxContainer = entry["stats_body"]
	title_label.text = _display_name(info)
	var subtitle: Label = entry.get("subtitle_label")
	if subtitle:
		subtitle.text = _subtitle(info)
	var icon: Button = entry.get("icon")
	if icon:
		HudIcons.set_icon(icon, _icon_name(info))

	for child in stats_body.get_children():
		stats_body.remove_child(child)
		child.queue_free()

	_add_alerts(stats_body, info)

	var pages := {}
	for title in TABS:
		var page := VBoxContainer.new()
		page.name = title
		page.add_theme_constant_override("separation", 4)
		pages[title] = page
	_fill_overview(pages["Overview"], info)
	_fill_money(pages["Money"], info)
	if info.has("extractor_resource"):
		_add_extractor_reserve_section(pages["Site"], info)
	if info.has("field_resource"):
		_add_field_section(pages["Site"], info)
	_fill_power(pages["Power"], info)
	_fill_stock(pages["Stock"], info)
	_add_row(pages["Parking"], "Valid places", str(info.get("parking_valid_places", 0)))
	var parking_rejected: PackedStringArray = info.get("parking_rejected", PackedStringArray())
	for reason in parking_rejected:
		_add_alert(pages["Parking"], reason, false)

	var tabs := TabContainer.new()
	_style_tabs(tabs)
	stats_body.add_child(tabs)
	for title in TABS:
		var page: VBoxContainer = pages[title]
		if page.get_child_count() == 0:
			page.free()
			continue
		tabs.add_child(page)
		if title == "Parking" and not parking_rejected.is_empty():
			var index := tabs.get_tab_count() - 1
			tabs.set_tab_icon(index, load(HudIcons.ICON_DIR + "warning.svg") as Texture2D)
			tabs.set_tab_icon_max_width(index, int(UIStyle.scaled_px(14.0)))
	var picked := tabs.get_node_or_null(NodePath(_last_tab))
	tabs.current_tab = tabs.get_tab_idx_from_control(picked) if picked else 0
	tabs.tab_changed.connect(_on_tab_changed.bind(entry, tabs))
	# Once the new rows have their sizes; the hourly refresh can add or drop rows.
	if entry.has("body"):
		_fit_window.call_deferred(entry["window"], entry["body"])

func _on_tab_changed(index: int, entry: Dictionary, tabs: TabContainer) -> void:
	_last_tab = tabs.get_tab_title(index)
	if entry.has("body"):
		_fit_window.call_deferred(entry["window"], entry["body"])

# Zone, level and the business status, e.g. "Industrial · Level 1 · Running".
func _subtitle(info: Dictionary) -> String:
	var parts: PackedStringArray = [
		_zone_label(str(info.get("zone_type", ""))),
		"Level %d" % int(info.get("level", 0)),
	]
	if info.get("business_summary", false):
		parts.append(str(info.get("business_status", "-")))
	return " · ".join(parts)

# A utility's service icon, a farm's field, an extractor's pit, else the zone's.
func _icon_name(info: Dictionary) -> String:
	var candidates: Array[String] = [str(info.get("utility_service", ""))]
	if info.has("field_resource"):
		candidates.append("field")
	if info.has("extractor_resource"):
		candidates.append("extractor")
	candidates.append(str(info.get("zone_type", "")))
	for icon_name in candidates:
		if not icon_name.is_empty() and ResourceLoader.exists(HudIcons.ICON_DIR + icon_name + ".svg"):
			return icon_name
	return "services"

# Flat tabs with the HUD accent underline on the picked one, matching the toolbar tiles.
func _style_tabs(tabs: TabContainer) -> void:
	tabs.get_tab_bar().focus_mode = Control.FOCUS_NONE
	UIStyle.set_font_size(tabs, 13)
	var panel := StyleBoxEmpty.new()
	panel.content_margin_top = 8
	tabs.add_theme_stylebox_override("panel", panel)
	var bar := StyleBoxFlat.new()
	bar.bg_color = Color.TRANSPARENT
	bar.border_width_bottom = 1
	bar.border_color = UIStyle.BORDER_ACCENT
	tabs.add_theme_stylebox_override("tabbar_background", bar)
	var styles := {
		"tab_unselected": [Color.TRANSPARENT, Color.TRANSPARENT],
		"tab_hovered": [Color(1, 1, 1, 0.06), Color.TRANSPARENT],
		"tab_selected": [Color(UIStyle.HUD_ACCENT, 0.12), UIStyle.HUD_ACCENT],
	}
	for state: String in styles:
		var style := StyleBoxFlat.new()
		style.bg_color = styles[state][0]
		style.border_color = styles[state][1]
		style.border_width_bottom = 2
		style.corner_radius_top_left = 4
		style.corner_radius_top_right = 4
		style.content_margin_left = 10
		style.content_margin_right = 10
		style.content_margin_top = 5
		style.content_margin_bottom = 5
		tabs.add_theme_stylebox_override(state, style)
	tabs.add_theme_stylebox_override("tab_focus", StyleBoxEmpty.new())
	tabs.add_theme_color_override("font_selected_color", UIStyle.TEXT_PRIMARY)
	tabs.add_theme_color_override("font_hovered_color", UIStyle.TEXT_PRIMARY)
	tabs.add_theme_color_override("font_unselected_color", UIStyle.TEXT_DIM)

# Broken assets and economies in red; deserted, rezoning and construction in yellow.
func _add_alerts(stats_body: VBoxContainer, info: Dictionary) -> void:
	if info.get("broken", false):
		_add_alert(stats_body, "Asset broken", true)
	if info.get("economy_broken", false):
		_add_alert(stats_body, "Economy broken", true)
	if info.get("is_deserted", false):
		_add_alert(stats_body, "Deserted", false)
	if info.get("pending_redevelopment", false):
		_add_alert(stats_body, "Rezone (%d d grace)" % info.get("rezone_grace_days", 0), false)
	if info.get("under_construction", false):
		_add_alert(
			stats_body,
			"Under construction: %d h left (%.0f%%)" % [
				int(info.get("construction_remaining_hours", 0)),
				float(info.get("construction_progress", 0.0)) * 100.0,
			],
			false
		)

# Residents, staffing, output and the utility's state and funding.
func _fill_overview(page: VBoxContainer, info: Dictionary) -> void:
	var zone: String = info.get("zone_type", "")
	var is_farm := info.has("field_resource")
	if zone == "residential" or is_farm:
		if is_farm:
			_add_row(page, "Farm Household", "%d / %d" % [int(info.get("occupancy", 0)), int(info.get("household_capacity", 1))])
		else:
			_add_row(page, "Households", str(info.get("household_count", info.get("occupancy", 0))))
		_add_row(page, "Children", str(info.get("child_count", 0)))
		_add_row(page, "Adults", str(info.get("adult_count", 0)))
		_add_row(page, "Elders", str(info.get("elder_count", 0)))
	if zone == "residential":
		return
	if is_farm:
		_add_section(page, "Business")
	var is_power_utility := str(info.get("utility_service", "")) == "power"
	if info.get("business_summary", false):
		var workers := int(info.get("worker_count", 0))
		var active_capacity := int(info.get("business_active_worker_capacity", info.get("worker_capacity", 0)))
		var max_capacity := int(info.get("worker_capacity", 0))
		var worker_text := str(workers)
		if not is_power_utility:
			worker_text = "%d / %d" % [workers, max_capacity]
			if active_capacity != max_capacity:
				worker_text = "%d / %d active (%d max)" % [workers, active_capacity, max_capacity]
		_add_row(page, "Workers", worker_text)
		_add_row(page, "Production", "%.0f%%" % (float(info.get("business_production_ratio", 0.0)) * 100.0))
		if info.get("business_has_inventory_fill", false):
			_add_row(page, "Inventory", "%.0f%% full" % (float(info.get("business_inventory_fill_ratio", 0.0)) * 100.0))
		if info.has("utility_service"):
			var utility_name := str(info.get("utility_service", "")).capitalize()
			var utility_state := "active" if info.get("utility_service_available", false) else "inactive"
			_add_row(page, "Utility", "%s %s" % [utility_name, utility_state])
			if is_power_utility:
				_add_service_funding_slider(page, info)
	else:
		var fallback_worker_text := str(info.get("worker_count", 0))
		if not is_power_utility:
			fallback_worker_text = "%d / %d" % [info.get("worker_count", 0), info.get("worker_capacity", 0)]
		_add_row(page, "Workers", fallback_worker_text)
	if info.has("utility_service_available") and not info.has("utility_service"):
		_add_row(page, "Utility", "Yes" if info["utility_service_available"] else "No")

# The business's budget and profit, then the residents' money and supplies.
func _fill_money(page: VBoxContainer, info: Dictionary) -> void:
	var zone: String = info.get("zone_type", "")
	if zone != "residential":
		_add_row(page, "Budget", _money(float(info.get("operating_budget", 0.0))))
		if info.get("business_summary", false):
			_add_row(page, "Today", _signed_money(float(info.get("business_profit_today", 0.0))))
			_add_row(page, "Yesterday", _signed_money(float(info.get("business_profit_yesterday", 0.0))))
			if info.has("utility_service"):
				_add_row(page, "Utility Revenue", _money(float(info.get("utility_local_revenue", 0.0))))
				_add_row(page, "City Inputs Today", _money(float(info.get("city_input_cost_today", 0.0))))
	if zone != "residential" and not info.has("field_resource"):
		return
	if page.get_child_count() > 0:
		_add_section(page, "Households")
	_add_row(
		page,
		"Household Money",
		"$%.1f total / $%.1f avg" % [
			float(info.get("household_budget_total", 0.0)),
			float(info.get("household_budget_avg", 0.0)),
		]
	)
	_add_row(
		page,
		"Supplies",
		"%.1f d avg / %.1f d min" % [
			float(info.get("household_stock_days_avg", 0.0)),
			float(info.get("household_stock_days_min", 0.0)),
		]
	)
	_add_row(page, "Stored Supplies", "%.1f kg" % float(info.get("household_stock_total", 0.0)))
	_add_row(page, "Replenishment", str(info.get("household_replenishment_state", "-")))
	if int(info.get("household_replenishment_active", 0)) > 0:
		_add_row(page, "Active Replenishment", str(info.get("household_replenishment_active", 0)))

# A power plant's output first, then the building's own use over the last day.
func _fill_power(page: VBoxContainer, info: Dictionary) -> void:
	if info.get("business_summary", false) and str(info.get("utility_service", "")) == "power":
		var power_produced := float(info.get("utility_power_production_today", 0.0))
		var power_consumed := float(info.get("utility_power_consumed_today", 0.0))
		_add_row(page, "Energy Produced", _electricity_quantity(power_produced, "kWh", "MWh"))
		_add_power_consumption_bar(page, power_consumed, power_produced)
	if info.has("power_staffed_capacity_kw"):
		_add_row(page, "Staffed capacity", _electricity_quantity(float(info["power_staffed_capacity_kw"]), "kW", "MW"))
	_add_section(page, "Last day")
	_add_row(page, "Consumption", _electricity_quantity(float(info.get("electricity_consumption_kwh", 0.0)), "kWh", "MWh"))
	_add_row(page, "Local supply", _electricity_quantity(float(info.get("electricity_local_kwh", 0.0)), "kWh", "MWh"))
	_add_row(page, "Imports", _electricity_quantity(float(info.get("electricity_imported_kwh", 0.0)), "kWh", "MWh"))
	_add_row(page, "Cost", _money(float(info.get("electricity_cost", 0.0))))
	if float(info.get("household_power_arrears", 0.0)) > 0.0:
		_add_row(page, "Unpaid household bills", _money(float(info["household_power_arrears"])))

func _fill_stock(page: VBoxContainer, info: Dictionary) -> void:
	var inventory: Array = info.get("inventory", [])
	for item in inventory:
		var unit := str(item.get("unit", ""))
		var stock_text := "%.1f %s" % [float(item.get("amount", 0.0)), unit]
		if item.has("daily_input_units"):
			stock_text += " (%.1f %s/day at full operation)" % [float(item["daily_input_units"]), unit]
		_add_row(page, _resource_label(str(item.get("name", "?"))), stock_text)

func _zone_label(zone: String) -> String:
	match zone:
		"residential": return "Residential"
		"commercial": return "Commercial"
		"industrial": return "Industrial"
		"utility": return "Utility"
		_: return zone.capitalize()

func _electricity_quantity(amount: float, small_unit: String, large_unit: String) -> String:
	if absf(amount) >= 1000.0:
		return "%.1f %s" % [amount / 1000.0, large_unit]
	return "%.1f %s" % [amount, small_unit]

func _money(amount: float) -> String:
	return "$%.0f" % amount

func _signed_money(amount: float) -> String:
	var sign := "+" if amount >= 0.0 else "-"
	return "%s$%.0f" % [sign, absf(amount)]

func _resource_label(resource_id: String) -> String:
	return resource_id.replace("_", " ").capitalize()

func _resource_units(amount: float) -> String:
	if absf(amount) >= 1000.0:
		return "%.1f t" % (amount / 1000.0)
	return "%.1f kg" % amount

# A heading; the separator above it is left out at the top of a tab.
func _add_section(stats_body: VBoxContainer, title: String) -> void:
	if stats_body.get_child_count() > 0:
		stats_body.add_child(HSeparator.new())
	stats_body.add_child(UIStyle.inspector_heading(title, true))

func _add_row(stats_body: VBoxContainer, label_text: String, value_text: String) -> void:
	var hbox := HBoxContainer.new()
	stats_body.add_child(hbox)

	var label := Label.new()
	label.text = label_text
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	UIStyle.set_font_size(label, ROW_FONT_SIZE)
	hbox.add_child(label)

	var value := Label.new()
	value.text = value_text
	value.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	UIStyle.set_font_size(value, ROW_FONT_SIZE)
	hbox.add_child(value)

func _add_extractor_reserve_section(stats_body: VBoxContainer, info: Dictionary) -> void:
	_add_section(stats_body, "Extraction Pit")
	var resource_name: String = _resource_label(str(info.get("extractor_resource", "resource")))
	if not bool(info.get("extractor_has_site", false)):
		_add_row(stats_body, "Pit", "No polygon")
		return

	var available := maxf(float(info.get("extractor_available_reserve_units", 0.0)), 0.0)
	var consumed := maxf(float(info.get("extractor_consumed_reserve_units", 0.0)), 0.0)
	var total := maxf(float(info.get("extractor_total_reserve_units", 0.0)), 0.0)
	var consumed_ratio := clampf(float(info.get("extractor_consumed_reserve_ratio", 0.0)), 0.0, 1.0)
	var area_m2 := maxf(float(info.get("extractor_area_m2", 0.0)), 0.0)
	_add_row(stats_body, "%s Area" % resource_name, "%.0f m2" % area_m2)
	_add_row(stats_body, "%s Available" % resource_name, _resource_units(available))
	_add_row(stats_body, "%s Consumed" % resource_name, _resource_units(consumed))
	_add_row(stats_body, "%s Reserve" % resource_name, _resource_units(total))
	_add_reserve_consumption_bar(stats_body, consumed_ratio, consumed, total)

func _add_field_section(stats_body: VBoxContainer, info: Dictionary) -> void:
	_add_section(stats_body, "Field")
	var resource_name: String = _resource_label(str(info.get("field_resource", "resource")))
	if not bool(info.get("field_has_site", false)):
		_add_row(stats_body, "Field", "No polygon")
		return

	var area_m2 := maxf(float(info.get("field_area_m2", 0.0)), 0.0)
	_add_row(stats_body, "%s Area" % resource_name, "%.0f m2" % area_m2)
	var edit_button := Button.new()
	edit_button.text = "Edit Field"
	UIStyle.style_primary_button(edit_button)
	edit_button.pressed.connect(func():
		get_node("../InputManager").edit_farm_field(info)
	)
	stats_body.add_child(edit_button)

func apply_field_edit(building_id: int, details: Dictionary) -> void:
	# Merge the bounded edit response; avoid querying all occupants on every release.
	for entry: Dictionary in _open_windows.values():
		var info: Dictionary = entry.get("info", {})
		if int(info.get("building_id", -1)) != building_id:
			continue
		if info.get("center_x") != details.get("center_x") or info.get("center_z") != details.get("center_z"):
			continue
		info.merge(details, true)
		_populate(entry, info)

func _add_service_funding_slider(stats_body: VBoxContainer, info: Dictionary) -> void:
	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 8)
	stats_body.add_child(hbox)

	var label := Label.new()
	label.text = "Plant Funding"
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	UIStyle.set_font_size(label, ROW_FONT_SIZE)
	hbox.add_child(label)

	var slider := HSlider.new()
	slider.min_value = 0.0
	slider.max_value = 1.0
	slider.step = 0.01
	slider.value = clampf(float(info.get("service_funding_effective", 1.0)), 0.0, 1.0)
	slider.custom_minimum_size = UIStyle.scaled_vector2(Vector2(120.0, 0.0))
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hbox.add_child(slider)

	var value := Label.new()
	value.custom_minimum_size = UIStyle.scaled_vector2(Vector2(42.0, 0.0))
	value.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	value.text = "%.0f%%" % (slider.value * 100.0)
	value.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	UIStyle.set_font_size(value, ROW_FONT_SIZE)
	hbox.add_child(value)

	slider.value_changed.connect(
		_on_service_funding_slider_changed.bind(
			value,
			float(info.get("center_x", 0.0)),
			float(info.get("center_z", 0.0)),
			str(info.get("utility_service", ""))
		)
	)
	slider.drag_ended.connect(_on_service_funding_slider_drag_ended)

func _add_reserve_consumption_bar(
	stats_body: VBoxContainer,
	consumed_ratio: float,
	_consumed_units: float,
	_total_units: float
) -> void:
	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 8)
	stats_body.add_child(hbox)

	var label := Label.new()
	label.text = "Pit Used"
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	UIStyle.set_font_size(label, ROW_FONT_SIZE)
	hbox.add_child(label)

	var bar_holder := Control.new()
	bar_holder.custom_minimum_size = UIStyle.scaled_vector2(Vector2(0.0, 18.0))
	bar_holder.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hbox.add_child(bar_holder)

	var bar := ProgressBar.new()
	bar.min_value = 0.0
	bar.max_value = 1.0
	bar.value = consumed_ratio
	bar.show_percentage = false
	bar.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bar.add_theme_stylebox_override(
		"background",
		UIStyle.panel_style(Color(0.18, 0.18, 0.20, 0.90), 4, Color.TRANSPARENT, 0)
	)
	bar.add_theme_stylebox_override(
		"fill",
		UIStyle.panel_style(Color(0.53, 0.42, 0.30, 0.95), 4, Color.TRANSPARENT, 0)
	)
	bar_holder.add_child(bar)

	var value := Label.new()
	value.text = "%.0f%%" % (consumed_ratio * 100.0)
	value.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	value.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	value.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	UIStyle.set_font_size(value, SECTION_FONT_SIZE)
	value.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bar_holder.add_child(value)

func _on_service_funding_slider_changed(
	new_value: float,
	value_label: Label,
	center_x: float,
	center_z: float,
	service_id: String
) -> void:
	if is_instance_valid(value_label):
		value_label.text = "%.0f%%" % (new_value * 100.0)
	if simulation_node != null:
		simulation_node.set_building_service_funding_override_at(
			center_x,
			center_z,
			service_id,
			new_value
		)

func _on_service_funding_slider_drag_ended(_value_changed: bool) -> void:
	call_deferred("_refresh_open_windows")

func _add_power_consumption_bar(stats_body: VBoxContainer, consumed_units: float, produced_units: float) -> void:
	produced_units = maxf(produced_units, 0.0)
	consumed_units = clampf(consumed_units, 0.0, produced_units)
	var ratio := 0.0
	if produced_units > 0.0:
		ratio = clampf(consumed_units / produced_units, 0.0, 1.0)

	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 8)
	stats_body.add_child(hbox)

	var label := Label.new()
	label.text = "Power Use"
	label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	label.add_theme_color_override("font_color", UIStyle.TEXT_DIM)
	UIStyle.set_font_size(label, ROW_FONT_SIZE)
	hbox.add_child(label)

	var bar_holder := Control.new()
	bar_holder.custom_minimum_size = UIStyle.scaled_vector2(Vector2(0.0, 18.0))
	bar_holder.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hbox.add_child(bar_holder)

	var bar := ProgressBar.new()
	bar.min_value = 0.0
	bar.max_value = 1.0
	bar.value = ratio
	bar.show_percentage = false
	bar.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bar.add_theme_stylebox_override(
		"background",
		UIStyle.panel_style(Color(0.18, 0.18, 0.20, 0.90), 4, Color.TRANSPARENT, 0)
	)
	bar.add_theme_stylebox_override(
		"fill",
		UIStyle.panel_style(Color(0.20, 0.62, 0.72, 0.95), 4, Color.TRANSPARENT, 0)
	)
	bar_holder.add_child(bar)

	var value := Label.new()
	value.text = "%s / %s" % [_electricity_quantity(consumed_units, "kWh", "MWh"), _electricity_quantity(produced_units, "kWh", "MWh")]
	value.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	value.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	value.add_theme_color_override("font_color", UIStyle.TEXT_PRIMARY)
	UIStyle.set_font_size(value, SECTION_FONT_SIZE)
	value.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	bar_holder.add_child(value)

# Broken states read in red, the rest (deserted, rezoning, construction) as warnings in yellow.
func _add_alert(stats_body: VBoxContainer, text: String, broken: bool) -> void:
	var label := Label.new()
	label.text = "! " + text
	label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	label.add_theme_color_override("font_color", UIStyle.TEXT_ALERT if broken else UIStyle.SIGN_YELLOW)
	UIStyle.set_font_size(label, ROW_FONT_SIZE)
	stats_body.add_child(label)
