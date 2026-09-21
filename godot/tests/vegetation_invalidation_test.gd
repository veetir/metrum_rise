# SPDX-License-Identifier: GPL-2.0-only

## Headless contract test for per-patch vegetation invalidation after a world edit.
## Trees are a derived product of the terrain surface, so a patch must regenerate when
## the terrain renderer commits a newer surface generation for that patch only.
##
## The vegetation grid is finer than the terrain grid, so "that patch only" now means every
## sub-patch the edited terrain patch owns and no sub-patch of any other. Both generations
## are still published per terrain patch, which is what makes the invalidation conservative.
extends SceneTree

const VegetationScript := preload("res://scripts/renderers/vegetation.gd")
const SPAN_M := 510.0
const SUBDIVISION := VegetationScript.PATCH_SUBDIVISION
const VEGETATION_SPAN_M := SPAN_M / SUBDIVISION
# Sub-patches one terrain patch owns, and so uploads one terrain patch costs.
const SUB_PATCHES := SUBDIVISION * SUBDIVISION
const WORLD_SIZE := Vector2(20400.0, 20400.0)

class MockTerrain:
	extends Node

	var resident: Array[Vector2i] = []
	var revision: int = 1
	var generations: Dictionary = {}

	func get_render_patch_span_m() -> float:
		return SPAN_M

	func get_resident_patch_keys() -> Array[Vector2i]:
		return resident

	func get_resident_patch_revision() -> int:
		return revision

	func get_patch_surface_generation(key: Vector2i) -> int:
		return int(generations.get(key, -1))

class MockSimulation:
	extends Node

	# One tree per patch is enough: the test asserts which patches regenerate, not placement.
	var patch_fetches: Array[Vector2] = []
	var vegetation_generations: Dictionary = {}
	var advance_during_fetch := false

	func get_vegetation_patch_generation(key: Vector2i) -> int:
		return int(vegetation_generations.get(key, 0))

	func get_terrain_world_size() -> Vector2:
		return WORLD_SIZE

	func get_decorative_tree_patch(
		origin: Vector2, _span: float, _understory: bool
	) -> PackedFloat32Array:
		patch_fetches.append(origin)
		if advance_during_fetch:
			var key := Vector2i(((origin + WORLD_SIZE * 0.5) / SPAN_M).floor())
			vegetation_generations[key] = get_vegetation_patch_generation(key) + 1
		return PackedFloat32Array([1.0, 0.0, 1.0, 0.0, 1.0, 0.0])

var _failures := 0

func _initialize() -> void:
	_run()

func _run() -> void:
	var host := Node3D.new()
	root.add_child(host)
	# The nodes must be inside the tree before global transforms or the viewport camera
	# are valid, and Vegetation resolves its siblings with @onready.
	await process_frame
	var camera := Camera3D.new()
	host.add_child(camera)
	camera.current = true

	var terrain := MockTerrain.new()
	terrain.name = "Terrain"
	host.add_child(terrain)
	var simulation := MockSimulation.new()
	simulation.name = "SimulationNode"
	host.add_child(simulation)

	var vegetation := VegetationScript.new()
	vegetation.name = "Vegetation"
	vegetation.set_process(false)
	host.add_child(vegetation)
	await process_frame

	var near := Vector2i(20, 20)
	var far := Vector2i(21, 20)
	var near_origins := _sub_origins(near)
	terrain.resident = [near, far]
	terrain.generations = {near: 4, far: 4}
	# Place the camera inside the near patch so both patches are within tree range.
	camera.global_position = Vector3(
		(near.x + 0.5) * SPAN_M - WORLD_SIZE.x * 0.5, 200.0,
		(near.y + 0.5) * SPAN_M - WORLD_SIZE.y * 0.5
	)
	vegetation.rebuild_from_simulation_state()

	# One patch per frame, so both terrain patches take their sub-patch count in frames.
	for i in range(SUB_PATCHES * 2 + 1):
		vegetation._process(0.016)
	_expect(
		vegetation.patches.size() == SUB_PATCHES * 2,
		"both resident patches must build every sub-patch, got %d" % vegetation.patches.size()
	)
	_expect(
		simulation.patch_fetches.size() == SUB_PATCHES * 2,
		"each sub-patch must fetch placements once, got %d" % simulation.patch_fetches.size()
	)

	# A road edit dirties one patch. The terrain renderer commits it at a newer generation.
	simulation.patch_fetches.clear()
	terrain.generations[near] = 9
	for i in range(SUB_PATCHES):
		vegetation._process(0.016)
	_expect(
		_sorted(simulation.patch_fetches) == _sorted(near_origins),
		"only the edited patch may regenerate, got %s" % [simulation.patch_fetches]
	)
	_expect(
		vegetation.patches.size() == SUB_PATCHES * 2,
		"the replacement must not orphan a patch key"
	)
	for key in _sub_keys(near):
		_expect(
			int(vegetation.patches[key].get_meta("surface_generation")) == 9,
			"the rebuilt patch must record the generation it was built against"
		)

	# A settled patch must not regenerate again, and an uncommitted patch must not churn.
	vegetation._process(0.016)
	vegetation._process(0.016)
	_expect(vegetation.queue.is_empty(), "a patch at the current generation must stay settled")
	terrain.generations[far] = -1
	vegetation._process(0.016)
	_expect(vegetation.queue.is_empty(), "a patch with no committed payload must not churn")

	# A vegetation edit changes no terrain generation, and only its own patch rebuilds.
	simulation.patch_fetches.clear()
	simulation.vegetation_generations[near] = 1
	for i in range(SUB_PATCHES):
		vegetation._process(0.016)
	_expect(
		_sorted(simulation.patch_fetches) == _sorted(near_origins),
		"vegetation edits must rebuild only their touched patch, got %s" % [simulation.patch_fetches]
	)
	for key in _sub_keys(near):
		_expect(int(vegetation.patches[key].get_meta("vegetation_generation")) == 1, "upload must stamp the independent vegetation revision")
	_expect(terrain.generations[near] == 9, "vegetation must not advance terrain generations")
	for key in _sub_keys(far):
		_expect(not vegetation._is_patch_stale(key), "vegetation edits must leave the neighboring patch settled")

	# A concurrent edit during fetch must remain detectable: revisions are read before fetch.
	var probe: Vector3i = _sub_keys(near)[0]
	simulation.advance_during_fetch = true
	vegetation._upload_patch(probe, VEGETATION_SPAN_M)
	_expect(vegetation._is_patch_stale(probe), "an edit during placement fetch must not be stamped as already rendered")
	simulation.advance_during_fetch = false
	vegetation._upload_patch(probe, VEGETATION_SPAN_M)
	_expect(not vegetation._is_patch_stale(probe), "a subsequent upload must settle the vegetation revision")

	host.free()
	quit(1 if _failures > 0 else 0)

## Every vegetation sub-patch key one terrain render patch owns.
func _sub_keys(terrain_key: Vector2i) -> Array[Vector3i]:
	var keys: Array[Vector3i] = []
	for column in range(SUBDIVISION):
		for row in range(SUBDIVISION):
			keys.append(Vector3i(terrain_key.x * SUBDIVISION + column,
				terrain_key.y * SUBDIVISION + row, SUBDIVISION))
	return keys

## World-space minimum corner of every sub-patch one terrain render patch owns.
func _sub_origins(terrain_key: Vector2i) -> Array[Vector2]:
	var origins: Array[Vector2] = []
	for key in _sub_keys(terrain_key):
		origins.append(Vector2(key.x, key.y) * VEGETATION_SPAN_M - WORLD_SIZE * 0.5)
	return origins

## Upload order follows the queue, which sorts by distance, so a set comparison is what the
## contract actually claims: these origins and no others.
func _sorted(origins: Array) -> Array:
	var copy := origins.duplicate()
	copy.sort_custom(func(a: Vector2, b: Vector2) -> bool:
		return a.x < b.x if a.x != b.x else a.y < b.y)
	return copy

func _expect(condition: bool, message: String) -> void:
	if condition:
		return
	_failures += 1
	push_error(message)
