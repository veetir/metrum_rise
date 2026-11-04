# SPDX-License-Identifier: GPL-2.0-only

## Service/industry palette view. Packs keep arbitrary class IDs; unknown classes get a glyph.
extends VBoxContainer

const HudIcons = preload("res://scripts/ui/hud_icons.gd")
var asset_panel: PanelContainer
var asset_menu: HBoxContainer
var category_menu: HBoxContainer

func build(with_categories: bool) -> void:
	size_flags_horizontal = Control.SIZE_SHRINK_CENTER
	add_theme_constant_override("separation", 8)
	asset_menu = HBoxContainer.new()
	asset_menu.alignment = BoxContainer.ALIGNMENT_CENTER
	asset_menu.add_theme_constant_override("separation", 6)
	asset_panel = HudIcons.shell(asset_menu)
	add_child(asset_panel)
	category_menu = HBoxContainer.new()
	category_menu.alignment = BoxContainer.ALIGNMENT_CENTER
	category_menu.add_theme_constant_override("separation", 6)
	if with_categories:
		add_child(HudIcons.shell(category_menu))
		asset_panel.hide()
	else:
		category_menu.free()
		category_menu = null

static func class_icon(class_id: String) -> String:
	return class_id if class_id in ["parking", "power", "water", "waste", "sewage",
		"police", "fire", "healthcare", "education", "transit", "parks", "government", "field", "extractor"] else "asset"

static func class_tile(class_id: String, title: String) -> Button:
	var button := HudIcons.icon_button(class_icon(class_id), title, "Choose " + title.to_lower() + " assets", 64)
	button.toggle_mode = true
	return button

static func asset_tile(asset: Dictionary, class_id: String) -> Button:
	var title := str(asset.get("display_name", asset.get("asset_id", "Asset")))
	var details := PackedStringArray([str(asset.get("asset_id", ""))])
	if asset.has("resource_id"):
		details.append("Resource: " + str(asset.resource_id).capitalize())
	if asset.has("lot_width_cells") and asset.has("lot_depth_cells"):
		details.append("Lot: %d × %d cells" % [asset.lot_width_cells, asset.lot_depth_cells])
	if asset.has("worker_capacity"):
		details.append("Workers: %d" % int(asset.worker_capacity))
	# Only show economics supplied by the bridge; today's asset index has none.
	if asset.has("build_cost"):
		details.append("Cost: $%s" % str(asset.build_cost))
	if asset.has("upkeep"):
		details.append("upkeep: $%s" % str(asset.upkeep))
	var button := HudIcons.icon_button(class_icon(class_id), title, "\n".join(details), 72)
	button.toggle_mode = true
	return button
