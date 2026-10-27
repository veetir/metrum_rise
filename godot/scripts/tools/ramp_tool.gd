# SPDX-License-Identifier: GPL-2.0-only

## Two-click ramp stub tool; Rust owns snapping, auxiliary lanes, validation and publication.
## Select a divided road, then its outer endpoint. E switches exit/entry; F reverses carriageway.
extends "res://scripts/tools/network_tool.gd"

var forward := true
var exit_ramp := true
var auxiliary_length := 80.0
var taper_length := 35.0
var _anchor: Dictionary = {}
var _points := PackedVector3Array()
var _request := 0
var _key: Array = []
var _valid := false
var _props: Array[Vector2i] = []
var _scene = preload("res://scripts/renderers/road_junction_preview.gd").new()
var _terrain = preload("res://scripts/renderers/road_terrain_preview.gd").new()
var _label: Label

func _ready() -> void:
	super._ready()
	var canvas := CanvasLayer.new()
	add_child(canvas)
	_label = Label.new()
	_label.position = Vector2(20, 120)
	_label.add_theme_color_override("font_shadow_color", Color.BLACK)
	_label.add_theme_constant_override("shadow_outline_size", 4)
	canvas.add_child(_label)
	terrain_node.patch_render_will_change.connect(func(_patch: Vector2i):
		_key = []
		_clear())

func _process(delta: float) -> void:
	super._process(delta)
	_label.visible = active
	if not active:
		_clear()
		_key = []
		return
	if _request > 0:
		var result: Variant = simulation_node.get_road_commit_result(_request)
		if result is Dictionary:
			_request = 0
			_key = []
			_label.text = "Ramp placed" if result.get("committed", false) else _reason(str(result.get("reason", "")))
		return
	_update_preview()

func get_world_mouse_pos() -> Vector3:
	return get_immutable_world_mouse_pos()

func _unhandled_input(event: InputEvent) -> void:
	if not active or _request > 0:
		return
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_E: exit_ramp = not exit_ramp
			KEY_F: forward = not forward
			KEY_BRACKETLEFT: auxiliary_length = maxf(60.0, auxiliary_length - 10.0)
			KEY_BRACKETRIGHT: auxiliary_length = minf(300.0, auxiliary_length + 10.0)
			_: return
		_key = []
		get_viewport().set_input_as_handled()
	elif event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		if _anchor.is_empty():
			var point := get_world_mouse_pos()
			if is_valid:
				_anchor = simulation_node.get_ramp_anchor(point)
			_key = []
		else:
			_update_preview()
			if _valid:
				_request = simulation_node.place_ramp(int(_anchor.edge), float(_anchor.station), forward, exit_ramp, _points, auxiliary_length, taper_length)
				_clear()
				_anchor = {}
		get_viewport().set_input_as_handled()

func _update_preview() -> void:
	if _anchor.is_empty():
		_label.text = "Ramp · Click a straight divided road · E exit/entry · F carriageway"
		return
	var point := get_world_mouse_pos()
	if not is_valid:
		_clear()
		_key = []
		_label.text = "Ramp · Point at the map to preview."
		return
	point.x = snappedf(point.x, 2.0)
	point.z = snappedf(point.z, 2.0)
	var key: Array = [point, _anchor, forward, exit_ramp, auxiliary_length, taper_length, simulation_node.get_road_tool_surface_generation(), simulation_node.get_zoning_overlay_revision(), simulation_node.get_agriculture_field_overlay_revision()]
	if key == _key:
		return
	_key = key
	_points = PackedVector3Array([_anchor.position, point])
	_clear()
	var preview: Dictionary = simulation_node.preview_ramp(int(_anchor.edge), float(_anchor.station), forward, exit_ramp, _points, auxiliary_length, taper_length)
	_valid = bool(preview.get("is_valid", false))
	_label.text = "%s ramp · %s carriageway · %.0f m auxiliary lane · [ / ] length\nClick outer endpoint · E exit/entry · F carriageway" % ["Exit" if exit_ramp else "Entry", "Forward" if forward else "Reverse", auxiliary_length]
	if not _valid:
		_label.text += "\n" + _reason(str(preview.get("invalid_reason", "")))
		if preview.get("is_pending", false):
			_key = []
		return
	var road: Node3D = get_node("../RoadTool")
	var payloads: Array = preview.get("terrain_preview", {}).get("patches", [])
	payloads = payloads.filter(func(data: Dictionary) -> bool: return terrain_node.patches.has(Vector2i(data.patch_x, data.patch_z)))
	var staged: Array = _terrain.stage(terrain_node, payloads, int(preview.surface_generation))
	if not staged.is_empty() and _scene.show_preview(road, preview.junction_preview, int(preview.request_id)):
		_props = road.set_road_prop_preview(_scene.prop_chunks(preview.junction_preview))
		_terrain.commit(terrain_node, staged, int(preview.request_id), _invalidate)
	else:
		_terrain.discard(staged)
		_valid = false
		_key = []

func _invalidate() -> void:
	_key = []
	_clear()

func _clear() -> void:
	_valid = false
	_scene.clear()
	_terrain.clear()
	var road := get_node_or_null("../RoadTool")
	if road != null:
		road.clear_road_prop_preview(_props)
	_props = []

func cancel() -> void:
	_anchor = {}
	_clear()
	_key = []

func _reason(reason: String) -> String:
	match reason:
		"ramp_requires_plain_divided_road", "ramp_requires_straight_road": return "Choose a straight divided surface road without existing pockets."
		"ramp_requires_outer_branch": return "Place the endpoint outward and ahead for an exit, or behind for an entry."
		"ramp_road_too_short": return "Move the connection farther from the road ends, or shorten its auxiliary lane."
		"ramp_overlaps_road": return "Another road occupies this ramp corridor."
		"surface_geometry_invalid": return "These road surfaces cannot join. Try another location or angle."
		_: return reason.replace("_", " ").capitalize()
