# SPDX-License-Identifier: GPL-3.0-or-later

## Draws Rust's Zebra hover verdict and sends the click at the same world pointer.
extends "res://scripts/tools/network_tool.gd"

const RoadReasonText := preload("res://scripts/ui/road_reason_text.gd")
const REMOVE_COLOR := Color(1.0, 0.63, 0.20, 0.85)
const STRIPE_WIDTH := 0.5
const STRIPE_PITCH := 1.0
const STRIPE_HALF_LENGTH := 1.25
const PAINT_LIFT := 0.08

var _label: Label
var _key: Array = []

func _ready() -> void:
	super._ready()
	cursor_mesh.hide()
	var canvas := CanvasLayer.new()
	add_child(canvas)
	_label = Label.new()
	_label.position = Vector2(20, 120)
	_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_label.add_theme_color_override("font_shadow_color", Color.BLACK)
	_label.add_theme_constant_override("shadow_outline_size", 4)
	_label.hide()
	canvas.add_child(_label)
	set_process(false)
	set_process_unhandled_input(false)

## Enables the tool and clears the previous pointer's feedback.
func set_tool_active(enabled: bool) -> void:
	active = enabled
	set_process(enabled)
	set_process_unhandled_input(enabled)
	_key.clear()
	blueprint_mesh.mesh = null
	_label.visible = enabled

func _process(_delta: float) -> void:
	if not active:
		return
	var point: Variant = _pointer(get_viewport().get_mouse_position(), false)
	if point is bool:
		return
	var key: Array = [point, simulation_node.get_road_tool_surface_generation(),
		simulation_node.get_zoning_overlay_revision(), simulation_node.get_agriculture_field_overlay_revision()]
	if key == _key:
		return
	_key = key
	_show_preview(simulation_node.preview_zebra(point) if point != null else {})

func _unhandled_input(event: InputEvent) -> void:
	if active and event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		var point: Variant = _pointer(event.position, true)
		if point != null:
			var result: Dictionary = simulation_node.edit_zebra(point)
			if result.get("is_valid", false):
				_key.clear()
			else:
				var preview: Dictionary = simulation_node.preview_zebra(point)
				preview.merge(result, true)
				_show_preview(preview)
		else:
			_show_preview({})
		get_viewport().set_input_as_handled()

func _pointer(position: Vector2, click: bool) -> Variant:
	if _scripted_pointer_enabled:
		return _scripted_pointer_position
	if not click and get_viewport().gui_get_hovered_control() != null:
		return null
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return null
	var origin := camera.project_ray_origin(position)
	var direction := camera.project_ray_normal(position)
	if click:
		return simulation_node.intersect_world_surface(origin, direction, true)
	return simulation_node.hover_world_surface(origin, direction)

func _show_preview(preview: Dictionary) -> void:
	var valid := bool(preview.get("is_valid", false))
	var removing := bool(preview.get("removing", false))
	_label.text = ("Click to remove crossing" if removing else "Click to add crossing") if valid \
		else RoadReasonText.text(preview, "Point at a road")
	var colour: Color = (REMOVE_COLOR if removing else blue_color) if valid else red_color
	blueprint_mat.albedo_color = colour
	blueprint_mat.emission = Color(colour.r, colour.g, colour.b) * 0.45
	var kerbs: PackedVector3Array = preview.get("kerbs", PackedVector3Array())
	blueprint_mesh.mesh = _stripes(kerbs, float(preview.get("slope", 0.0))) if kerbs.size() == 2 else null

# Match the published zebra's pitch and symmetric margin, lifted above its asphalt.
func _stripes(kerbs: PackedVector3Array, slope: float) -> ImmediateMesh:
	var delta := kerbs[1] - kerbs[0]
	var width := Vector2(delta.x, delta.z).length()
	if width < STRIPE_WIDTH:
		return null
	var across := delta / width
	var along := Vector3(-across.z, slope, across.x) * STRIPE_HALF_LENGTH
	var count := int(floor((width + STRIPE_PITCH - STRIPE_WIDTH) / STRIPE_PITCH))
	var margin := (width - (count - 1) * STRIPE_PITCH - STRIPE_WIDTH) * 0.5
	var mesh := ImmediateMesh.new()
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	for stripe in count:
		var left := kerbs[0] + across * (margin + stripe * STRIPE_PITCH) + Vector3.UP * PAINT_LIFT
		var right := left + across * STRIPE_WIDTH
		for vertex: Vector3 in [left - along, left + along, right + along,
				left - along, right + along, right - along]:
			mesh.surface_add_vertex(vertex)
	mesh.surface_end()
	return mesh
