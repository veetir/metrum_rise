# SPDX-License-Identifier: GPL-2.0-only

## Single-lane roundabout tool. Rust owns layout, validation, traffic rules and transactions.
## O activates; brackets size, N cycles 3–6 arms, comma/period rotate. Hover a junction to replace it.
extends "res://scripts/tools/network_tool.gd"

var diameter := 40.0
var arms := 4
var rotation_angle := 0.0
var _request := 0
var _key: Array = []
var _center := Vector3.ZERO
var _replace := -1
var _angles := PackedFloat32Array()
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
			_label.text = "Roundabout placed" if result.get("committed", false) else _reason(str(result.get("reason", "")))
		return
	_update_preview()

func _unhandled_input(event: InputEvent) -> void:
	if not active or _request > 0:
		return
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_BRACKETLEFT: diameter = maxf(12.0, diameter - 1.0)
			KEY_BRACKETRIGHT: diameter = minf(120.0, diameter + 1.0)
			KEY_N: arms = 3 + (arms - 2) % 4
			KEY_COMMA: rotation_angle -= PI / 24.0
			KEY_PERIOD: rotation_angle += PI / 24.0
			_: return
		get_viewport().set_input_as_handled()
		_key = []
	elif event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		# Re-resolve authority at click; never commit a different hover selection.
		_update_preview(true)
		if _valid:
			_request = simulation_node.place_roundabout(_center, diameter, _angles, _replace)
			_clear()
		get_viewport().set_input_as_handled()

# Use the same immutable query as RoadTool: the legacy world raycast can return null
# while the simulation owns its mutex, which the base picker turns into the world origin.
func get_world_mouse_pos() -> Vector3:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		is_valid = false
		return Vector3.ZERO
	var mouse := get_viewport().get_mouse_position()
	var cursor: Variant = simulation_node.get_road_tool_cursor_pos(
		camera.project_ray_origin(mouse), camera.project_ray_normal(mouse),
		0.0, true, 0, Vector3.ZERO, Vector3.ZERO, false, 1, 1, 0.0, {}, 0.0, Vector3.ZERO
	)
	is_valid = cursor != null
	return cursor.position if is_valid else Vector3.ZERO

func _update_preview(refresh_pointer: bool = false) -> void:
	if refresh_pointer and not _scripted_pointer_enabled:
		_last_world_mouse_pos = get_world_mouse_pos()
	if not _scripted_pointer_enabled and not is_valid:
		_clear()
		_key = []
		_label.text = "Roundabout · Point at the map to preview."
		return
	# NetworkTool already resolved the viewport pointer this frame. Only clicks repick.
	var point: Vector3 = _scripted_pointer_position if _scripted_pointer_enabled else _last_world_mouse_pos
	point.x = snappedf(point.x, 0.5)
	point.z = snappedf(point.z, 0.5)
	_replace = simulation_node.get_closest_node(point, 5.0)
	if _replace >= 0:
		point = simulation_node.get_node_pos(_replace)
	var key: Array = [point, _replace, diameter, arms, rotation_angle, simulation_node.get_road_tool_surface_generation(), simulation_node.get_zoning_overlay_revision(), simulation_node.get_agriculture_field_overlay_revision()]
	if key == _key:
		return
	_key = key
	_center = point
	_angles = PackedFloat32Array()
	for i in arms:
		_angles.append(rotation_angle + TAU * float(i) / float(arms))
	_clear()
	var preview: Dictionary = simulation_node.preview_roundabout(_center, diameter, _angles, _replace)
	_valid = bool(preview.get("is_valid", false))
	_label.text = "Roundabout %.0f m · %d arms · [ / ] size · N arms · , / . rotate\n%s" % [diameter, arms, "Click to replace junction" if _replace >= 0 else "Click to place"]
	if not _valid:
		_label.text += "\n" + _reason(str(preview.get("invalid_reason", "")))
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
	_clear()
	_key = []

func _reason(reason: String) -> String:
	match reason:
		"roundabout_arms_too_close": return "Arms need more room. Increase the diameter."
		"roundabout_overlaps_road": return "Another road occupies this site."
		"roundabout_requires_three_to_six_arms": return "Choose a junction with three to six arms."
		"roundabout_requires_simple_surface_arms": return "Replacement requires single-lane, two-way surface roads."
		"roundabout_has_lane_restrictions": return "Remove the junction's vehicle turn restrictions first."
		"roundabout_arm_too_short": return "An approach is too short for this diameter."
		"roundabout_replacement_too_large": return "Junction replacement supports diameters up to 60 m."
		"surface_geometry_invalid": return "The road surfaces cannot join here. Try another size or location."
		"water_requires_bridge": return "The roundabout needs dry ground."
		"parcel_overlap", "zoning_cell_overlap", "field_overlap": return "Clear the occupied land before placing the roundabout."
		_: return "Placement is not ready." if reason.is_empty() else reason.replace("_", " ").capitalize()
