# SPDX-License-Identifier: GPL-2.0-only

## Player vegetation input and ground-ring preview; all placement decisions stay in Rust.
## Rust methods called: add_vegetation_at(), remove_vegetation_at(), paint_vegetation(),
## intersect_world_surface().
extends Node3D

enum Mode { PLANT, REMOVE }

# The radius is the only size control, and it also chooses between a point edit and a brush: at
# the minimum a plant lands exactly under the cursor, and above it the disc fills a 4 m lattice.
# The ground ring is therefore the mode readout as well as the footprint.
const MIN_RADIUS_M := 1.0
const MAX_RADIUS_M := 1024.0
# Matches the native paint bound; removal retains its 1 km radius.
const MAX_PAINT_RADIUS_M := 256.0
# Geometric, because a fixed step cannot serve both a one-tree cursor and a 1 km clear-cut.
const RADIUS_STEP := 1.4
# Frames the tool keeps ignoring a stamp after an embedded popup last held the input grab. The
# species dropdown takes that grab, and the click that dismisses it is delivered here as well, so
# one click used to close the menu and plant a tree. The tool cannot ask the popup directly
# without reaching into the UI layer, so it watches the viewport's own subwindow list instead.
# Two frames covers the dismissal and stays far under the interval a player can click again in.
const MENU_DISMISS_FRAMES := 2

const PLANT_RING_COLOR := Color(0.85, 0.95, 0.45)
const REMOVE_RING_COLOR := Color(0.95, 0.45, 0.35)

# Ring stroke, in pixels. A fixed world-space stroke is honest in metres and useless on screen: at
# the former 0.3 m it covered two pixels at 100 m and a quarter of one at 900 m, so the cursor
# faded out exactly where a large brush needs it most. The radius stays a true world measurement,
# because it is the footprint the stamp will cover; only the stroke follows the screen.
const RING_STROKE_PX := 3.0
# Bounds on what that resolves to in world metres. The lower bound is the former fixed stroke and
# keeps the torus from degenerating at the point-edit radius. The upper keeps a 256 m brush
# reading as a ring rather than filling in as a disc.
const RING_STROKE_MIN_M := 0.3
const RING_STROKE_MAX_RATIO := 0.06

@onready var simulation_node = $"../SimulationNode"
var active := false
var mode: Mode = Mode.PLANT:
	set(value):
		mode = value
		radius = minf(radius, MAX_PAINT_RADIUS_M if mode == Mode.PLANT else MAX_RADIUS_M)
var species := 0
var radius := MIN_RADIUS_M:
	set(value):
		radius = minf(value, MAX_PAINT_RADIUS_M if mode == Mode.PLANT else MAX_RADIUS_M)
var preview: MeshInstance3D
var _ring: TorusMesh
var _ring_material: StandardMaterial3D
var _preview_radius := -1.0
var _preview_stroke := -1.0
var _preview_mode := -1
var _painting := false
var _last_stamp := Vector2.INF
# Where the ring sits while the pointer is on a popup instead of on the ground.
var _last_hit := Vector3.INF
var _menu_grab_frames := 0

func _ready() -> void:
	_ring = TorusMesh.new()
	_ring.rings = 64
	_ring.ring_segments = 8
	preview = MeshInstance3D.new()
	preview.mesh = _ring
	preview.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	preview.top_level = true
	preview.visible = false
	_ring_material = StandardMaterial3D.new()
	_ring_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_ring_material.albedo_color = PLANT_RING_COLOR
	# This is a cursor footprint, so uneven ground must not hide half the ring.
	_ring_material.no_depth_test = true
	preview.material_override = _ring_material
	add_child(preview)

func _process(_delta: float) -> void:
	# The latch advances before any early return: the frames a popup holds the grab are exactly
	# the frames this function would otherwise leave through one of them.
	if not get_viewport().get_embedded_subwindows().is_empty():
		_menu_grab_frames = MENU_DISMISS_FRAMES
	elif _menu_grab_frames > 0:
		_menu_grab_frames -= 1
	preview.visible = false
	# Losing the tool mid-drag must not leave a stroke armed for the next activation.
	if not active:
		_painting = false
	if not active or get_viewport().gui_get_hovered_control() != null:
		return
	# With the species popup open the pointer is over the menu, so the ray lands wherever the
	# menu happens to sit rather than where the player was pointing. The ring holds its last
	# ground position instead, because resizing the brush from an open menu is blind otherwise
	# and being able to resize from there is the point.
	if _menu_grab_frames > 0:
		if _last_hit == Vector3.INF:
			return
	else:
		var probe = _mouse_world_pos()
		if probe == null:
			return
		_last_hit = probe
	var hit := _last_hit
	# TorusMesh rebuilds its surface on every property write, so the stroke is only re-applied
	# once it has drifted enough to see. Camera motion otherwise rebuilds 1024 triangles a frame.
	var stroke := _ring_stroke_m(hit)
	if radius != _preview_radius or absf(stroke - _preview_stroke) > _preview_stroke * 0.05:
		_preview_radius = radius
		_preview_stroke = stroke
		_ring.outer_radius = radius
		_ring.inner_radius = maxf(0.05, radius - stroke)
	# Colour carries the mode, because the panel has no space to and the cursor is where the
	# player is looking when it matters.
	if int(mode) != _preview_mode:
		_preview_mode = int(mode)
		_ring_material.albedo_color = (
			REMOVE_RING_COLOR if mode == Mode.REMOVE else PLANT_RING_COLOR
		)
	preview.global_position = hit + Vector3.UP * 0.15
	preview.visible = true

func _unhandled_input(event: InputEvent) -> void:
	if not active:
		return
	if event is InputEventMouseButton and event.pressed and event.ctrl_pressed:
		var direction := 0
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			direction = 1
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			direction = -1
		if direction != 0:
			step_radius(direction)
			get_viewport().set_input_as_handled()
			return
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			# This click is the one that dismisses an open popup. Dismissing is all it does.
			if _menu_grab_frames > 0:
				_menu_grab_frames = 0
				get_viewport().set_input_as_handled()
				return
			_painting = mode == Mode.REMOVE or radius > MIN_RADIUS_M
			_last_stamp = Vector2.INF
			if _stamp():
				get_viewport().set_input_as_handled()
		elif _painting:
			_painting = false
			get_viewport().set_input_as_handled()
	# Held-button motion continues the stroke, which is what makes a raised radius a brush rather
	# than a stamp. Restamping only after half a radius of travel keeps a slow drag from issuing
	# one native call per pixel; the discs still overlap, so the stroke has no gaps.
	elif event is InputEventMouseMotion and _painting:
		_stamp()

## Steps the radius one geometric notch, up for a positive direction. Public because the species
## dropdown forwards ctrl and the wheel here: an open popup holds the input grab, so those events
## never reach `_unhandled_input`.
func step_radius(direction: int) -> void:
	# Clamping at the minimum is what makes the point edit reachable again after any number of
	# steps up, since a geometric walk never lands back on it exactly.
	radius = clampf(
		radius * (RADIUS_STEP if direction > 0 else 1.0 / RADIUS_STEP),
		MIN_RADIUS_M,
		MAX_RADIUS_M
	)

# Applies one stamp at the cursor. Returns whether the surface was hit at all, since a click on
# sky must not be swallowed from the camera.
func _stamp() -> bool:
	var hit = _mouse_world_pos()
	if hit == null:
		return false
	var pos := Vector2(hit.x, hit.z)
	if _painting:
		if _last_stamp != Vector2.INF and pos.distance_to(_last_stamp) < radius * 0.5:
			return true
		_last_stamp = pos
	apply_at(pos)
	return true

## Applies one stamp of the active mode and returns how many plants Rust actually changed.
func apply_at(pos: Vector2) -> int:
	if mode == Mode.REMOVE:
		return simulation_node.remove_vegetation_at(pos, radius)
	if radius > MIN_RADIUS_M:
		return simulation_node.paint_vegetation(pos, radius, species)
	return int(simulation_node.add_vegetation_at(pos, species))

# World metres that cover RING_STROKE_PX at the cursor. Same projection term as the field edit
# tool's handle scaling, which is the existing idiom for this in the tool layer.
func _ring_stroke_m(hit: Vector3) -> float:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return RING_STROKE_MIN_M
	var height_px := maxf(float(get_viewport().get_visible_rect().size.y), 1.0)
	var metres_per_px := 2.0 * camera.global_position.distance_to(hit) * tan(
		deg_to_rad(camera.fov * 0.5)
	) / height_px
	# The upper bound never falls below the lower one, which it would at the point-edit radius.
	var upper := maxf(RING_STROKE_MIN_M, radius * RING_STROKE_MAX_RATIO)
	return clampf(RING_STROKE_PX * metres_per_px, RING_STROKE_MIN_M, upper)

func _mouse_world_pos() -> Variant:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return null
	var mouse := get_viewport().get_mouse_position()
	return simulation_node.intersect_world_surface(
		camera.project_ray_origin(mouse), camera.project_ray_normal(mouse)
	)
