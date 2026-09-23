# SPDX-License-Identifier: GPL-2.0-only

## Vegetation presentation reuses terrain residency; Rust owns the generated and edited state.
## Rust supplies deterministic patch-local placements for four species: conifer, broadleaf,
## bush and rock. Uploads at most one terrain patch of area per frame.
## A patch regenerates when the terrain renderer commits a newer surface generation for it,
## or Rust advances its independent vegetation revision. Later surface edits clear generated
## scatter; player placements remain authoritative.
extends Node3D

const TreeSpecies := preload("res://scripts/renderers/tree_species.gd")
# The shadow caster choice is bounded by the sun's own range; see _shadow_caster_wanted.
const SceneLightingConfig := preload("res://scripts/core/scene_lighting.gd")

# Per-species draw ranges. Canopy trees use near meshes and impostors for the silhouette
# of the landscape at every distance. Understory reads only close up, so it gets one level
# and a short range: it exists to stop near forest floor looking like mown lawn.
#
# Every range below is evaluated ONCE PER PATCH. Godot tests one distance for a whole
# MultiMeshInstance3D, so a plant gets the level its patch centre asks for and not the level
# its own distance asks for. Two rules follow, and both were broken before this was measured
# from a ground-level camera:
#   - A band must be wider than the patch diagonal, or one patch spans several bands and
#     the level it draws is wrong for most of what it contains. Only the near canopy is a
#     band: the two near levels share one instance and TREE_NEAR_DETAIL_M picks its mesh per
#     patch. The distant band draws one impostor per tree.
#   - A range must be longer than the patch half-diagonal, or the patch under the camera
#     can be culled while the plants at the camera's feet are still in view.
# Both rules are a tax on the patch size, and the patch used to be a 510 m terrain patch:
# 361 m from centre to corner, which put a 721 m floor under the near band on its own. That
# is why the near canopy reached 800 m, where a tree is 13 pixels tall. PATCH_SUBDIVISION
# below cuts the vegetation grid off the terrain grid so the band can follow the trees.
# Lane five of a packed placement carries the species ordinal in its low two bits and, above
# them, the renderer mesh variant the brush pinned, biased by one. Zero there leaves the
# variant to the appearance seed, which is every generated plant and every plant authored
# before the brush could name one tree. Rust packs the lane in vegetation_api.rs.
const SPECIES_BITS := 2
const SPECIES_MASK := 3

# The lathe needed 800 m: a smooth cone cannot replace a 53 px tree at 200 m.
# An impostor is a picture of the tree, allowing a 250 m handover: at 1080p / 75-degree
# FOV a 15 m tree is about 42 px there (15 / 250 * 703.7). The grid floor still applies.
const TREE_NEAR_FLOOR_M := 250.0
# Distance beyond which a patch draws its near canopy from the reduced level, which carries
# every foliage card and none of the interior wood: no child branch tubes and no solid tufts
# behind the cards. That was 41% to 50% of the triangles with a four-sided trunk, and it cost
# 26.0 ms of a close painted-stand frame to keep. The trunk keeps its five sides: a four-sided
# trunk reshaded every trunk in the patch under the camera, which was half of what the swap
# changed on screen. This is a per-patch mesh swap on the
# instance already uploaded, so it needs no band wider than a patch and costs no second draw.
# At 1080p and the default 75-degree vertical FOV the projection is 703.7 px/rad, so a 15 m
# tree covers 15 / d * 703.7 pixels: about 235 px here. Starting point for a rendered sweep.
const TREE_NEAR_DETAIL_M := 45.0
# Distance beyond which a patch casts from the lathe proxy instead of from the branched tree.
# The proxy is ten times cheaper but it is a solid volume standing exactly where the branched
# crown stands, so every foliage card inside it receives its shadow: up close that reads as
# hard dark bands across the crown, and the proxy's short trunk stub casts no stem shadow
# where the real trunk did. Both are only invisible once the tree is small enough that its
# self-shading is not resolvable. At 120 m a 15 m tree covers 88 pixels, and this lands just
# past the second of the four shadow cascades, so the two nearest cascades keep the correct
# caster and the two that cover almost all of the ground take the cheap one.
# Trees stop receiving cast shadows over the same distance, so a tree inside a proxy never sees it.
const SHADOW_PROXY_M := TreeSpecies.TREE_SHADOW_END_M
const TREE_FAR_M := 4500.0
const BUSH_RANGE_M := 420.0
const ROCK_RANGE_M := 420.0
# Half the diagonal of a square patch, per metre of span. A patch is one instance, so every
# conservative range test measures from the patch centre out to its farthest corner.
const PATCH_HALF_DIAGONAL := 0.7071067811865476

# How far inside its range one understory variant may stop, as a share of that range. Bush
# and rock carry a single level, so a variant that stops early drops nothing into a gap: the
# plant ends and the rest of the patch carries on. Twelve variants then end at twelve
# distances and the carpet dissolves over the last quarter of its range. Sharing one distance
# instead ends every plant of one patch on one line, and a camera above the canopy reads
# that line as a straight edge between forest floor and bare ground.
const UNDERSTORY_STAGGER_MIN := 0.75
# What share of the near band a patch may swap its canopy early by. The switch is a
# per-patch decision, so on one distance every patch switches on one circle and the boundary
# reads as a staircase of squares. Giving each patch its own
# share of this breaks the staircase up. The offset is negative only: a patch may switch
# early, never late, so the near band stays the conservative superset it already is, no patch
# builds near meshes it did not build before, and the near level draws over less ground.
const CANOPY_SWITCH_JITTER_FRACTION := 0.2

# How many vegetation patches one terrain render patch is cut into along each axis. The
# vegetation grid WAS the terrain grid, and that is what set every distance above: a patch is
# one instance, so no band can be narrower than the patch that has to fit inside it, and a
# 510 m terrain patch therefore forced a 721 m floor under the near band. The trees never
# needed that. Rust already takes an arbitrary origin and span in get_decorative_tree_patch,
# so the finer grid costs no simulation change: a sub-patch key divides back to its owning
# terrain key for both staleness questions, which leaves invalidation terrain-coarse and so
# conservative. Subdividing raises the distant instance count by its square, which is the
# cost this buys the near band's area reduction with.
const PATCH_SUBDIVISION := 4

## Which level a patch casts its shadows from. NEAR is correct and expensive, PROXY is cheap
## and wrong up close, NONE is for patches the sun's shadow range does not reach at all.
enum ShadowCaster { NONE, NEAR, PROXY }

var enabled := true
var density_fraction := 1.0
var cast_shadows := true
# Probe override for the canopy far range, in metres. Values at or below zero keep the
# authored TREE_FAR_M. Must exceed the near range to leave a distant band.
var far_range_override_m := 0.0
# Probe override for the canopy near range, in metres. Values at or below zero keep the
# range canopy_near_m() derives from the patch span.
var near_range_override_m := 0.0
# Probe override for PATCH_SUBDIVISION. Values below one keep the authored subdivision.
var patch_subdivision_override := 0
# Probe override for TREE_NEAR_DETAIL_M, in metres. Values below zero keep the authored
# distance. Zero is meaningful: it draws the whole near band from the reduced level.
var near_detail_override_m := -1.0
# Probe override for the per-frame upload budget, in sub-patch builds. Values below one keep
# the area budget the subdivision derives. Separates "how fine is the grid" from "how much
# work may one frame do", which the derived budget ties together.
var upload_budget_override := 0
# Vegetation patch span in metres, cached from the last residency pass so the range
# accessors can answer without a terrain call. Zero until the first patch is built.
var patch_span_m := 0.0
var patches: Dictionary = {}
# Patches that left terrain residency but are still inside the scatter radius. Terrain
# residency follows the camera frustum, so a rotation evicts patches that are about to be
# wanted again. These are hidden rather than freed, because rebuilding one costs a frame.
var cache: Dictionary = {}
var queue: Array[Vector3i] = []
# meshes[species][variant] is an Array[ArrayMesh], ordered near to far.
var meshes: Array = []
var last_revision := -1
# Span of one terrain render patch, in metres. A patch key carries the grid it belongs to,
# and this is what turns that grid back into a distance.
var terrain_span_m := 0.0
var last_camera_cell := Vector2i(2147483647, 2147483647)
var tree_count := 0
var generated_patches := 0
var generation_ms_max := 0.0
# Both staleness generations, keyed by owner patch, for the duration of one sweep. Every
# sub-patch of one terrain patch shares an owner, so the sweep asked the same two questions
# across the language boundary once per sub-patch: PATCH_SUBDIVISION squared times the
# answers it needed. Only the sweep reads it; _is_patch_stale and _upload_patch called from
# anywhere else still cross, because their answer is about the moment they are called. The
# dictionary is cleared, never replaced, to keep the sweep allocation-free.
var owner_generations: Dictionary = {}
var ready_for_world := false
@onready var terrain = $"../Terrain"
@onready var simulation = $"../SimulationNode"

func _ready() -> void:
	meshes = TreeSpecies.build_meshes()
	# Texture IO and array creation belong to catalogue setup, outside patch uploads.
	TreeSpecies.impostor_mesh()
	for species in [TreeSpecies.CONIFER, TreeSpecies.BROADLEAF]:
		TreeSpecies.impostor_material(species)

## Draw range for one species level as (begin_m, end_m). `near_band` says whether the patch
## also carries the near per-variant instances, and `switch_m` is the distance this patch
## swaps its canopy on. `variant` staggers the understory and is ignored by the canopy. A patch built far away carries none, and then
## the distant instance must begin at zero: it is the only thing in the patch, so a begin of
## the near range empties the patch the moment the camera reaches that distance. The rebuild that
## adds the near band is queued at one patch per frame, so a fast approach outruns it.
func lod_range(species: int, lod: int, variant: int, near_band: bool, switch_m: float) -> Vector2:
	if species == TreeSpecies.BUSH:
		return Vector2(0.0, BUSH_RANGE_M * _understory_stagger(variant))
	if species == TreeSpecies.ROCK:
		return Vector2(0.0, ROCK_RANGE_M * _understory_stagger(variant))
	# Both branched levels ride the same instance, which is built as level zero whichever
	# mesh it holds, so there is no second range here for the reduced one.
	if lod == 0:
		return Vector2(0.0, switch_m)
	# One impostor instance per species covers the whole distant band.
	return Vector2(switch_m if near_band else 0.0, canopy_far_m())

## Share of its range one understory variant keeps. A low-discrepancy sequence rather than a
## hash: twelve variants then spread evenly across the band, where twelve samples of a hash
## clump and leave the carpet ending on two or three lines instead of twelve.
func _understory_stagger(variant: int) -> float:
	return lerpf(UNDERSTORY_STAGGER_MIN, 1.0, fmod(float(variant) * 0.6180339887498949, 1.0))

## Distance at which one patch swaps its near canopy for the distant level. See
## CANOPY_SWITCH_JITTER_FRACTION. Keyed on the patch, so a rebuild reproduces the same distance and
## the switch does not move when a terrain edit regenerates the patch.
func canopy_switch_m(key: Vector3i) -> float:
	var near_m := canopy_near_m()
	return near_m - near_m * CANOPY_SWITCH_JITTER_FRACTION * _patch_hash01(key)

## Deterministic value in [0,1) for one patch key. The mix the vegetation shaders use for
## their cell hashes. GDScript integers are 64-bit and signed, so the multiplies wrap into
## negative values; the mask at the end is what brings the result back into range.
func _patch_hash01(key: Vector3i) -> float:
	var h := key.x * 374761393 + key.y * 668265263 + key.z * 2246822519
	h = (h ^ (h >> 13)) * 1274126177
	return float((h ^ (h >> 16)) & 0xFFFFFF) / 16777216.0

## Canopy far range in effect. One accessor so the draw range and the patch residency test
## can never disagree about how far the scatter reaches.
func canopy_far_m() -> float:
	return far_range_override_m if far_range_override_m > canopy_near_m() else TREE_FAR_M

## Distance the near canopy hands its cards over to the impostor on. Derived from the
## patch span rather than authored, because the binding constraint is the grid and not the
## tree: the band has to stay wider than the patch diagonal or one patch straddles the
## switch and draws the wrong level for most of what it holds. The floor is what the trees
## themselves ask for, and a patch small enough to reach it stops paying the grid's tax.
##
## The span is the fine grid's, from the terrain span, and never `patch_span_m`: an upload sets
## that to its own key's span, so it read 510 m after a coarse patch and 127.5 m after a fine
## one. While the floor sat above both diagonals the answer did not move. Below them, the band
## flipped between 721 m and 250 m with every upload, every patch near the switch disagreed
## with its own record on the next frame, and the forest rebuilt itself without end.
func canopy_near_m() -> float:
	if near_range_override_m > 0.0:
		return near_range_override_m
	var fine_span := terrain_span_m / float(patch_subdivision())
	return maxf(TREE_NEAR_FLOOR_M, fine_span * PATCH_HALF_DIAGONAL * 2.0)

## Distance the branched tree hands over to the reduced near level. One accessor so the two
## bands can never disagree about where they meet.
func near_detail_m() -> float:
	return near_detail_override_m if near_detail_override_m >= 0.0 else TREE_NEAR_DETAIL_M

## Vegetation patches per terrain render patch along one axis, with the probe override.
## Also the square root of the per-frame upload budget; see the budget in _process.
func patch_subdivision() -> int:
	return patch_subdivision_override if patch_subdivision_override >= 1 else PATCH_SUBDIVISION

## The terrain render patch that owns a vegetation sub-patch. Both staleness questions are
## answered on the terrain grid: the surface generation is published there, and routing the
## vegetation revision through the same key keeps one edit invalidating a whole terrain
## patch. That is coarser than it has to be and therefore never misses an edit.
func _owner_key(key: Vector3i) -> Vector2i:
	var divisor := maxi(key.z, 1)
	return Vector2i(floori(float(key.x) / divisor), floori(float(key.y) / divisor))

## Span of the patch this key names, in metres. The key carries its own grid divisor, so a
## coarse far patch and a fine near patch answer this differently and never collide in the
## patch dictionary even where their corners coincide.
func _key_span(key: Vector3i) -> float:
	return terrain_span_m / float(maxi(key.z, 1))

## Distance within which a terrain block has to be carried on the fine grid. Only a patch
## that can hold near canopy or understory needs the fine grid: those are the levels whose
## band is narrower than a terrain patch. Everything beyond draws one distant crown mesh,
## which is chosen per patch and needs no band at all, so subdividing it buys nothing and
## costs a patch, four instances and a sweep entry each.
func _fine_tier_radius() -> float:
	return maxf(canopy_near_m(), maxf(BUSH_RANGE_M, ROCK_RANGE_M))

func rebuild_from_simulation_state() -> void:
	for patch in patches.values():
		patch.queue_free()
	patches.clear()
	for patch in cache.values():
		patch.queue_free()
	cache.clear()
	queue.clear()
	tree_count = 0
	generated_patches = 0
	generation_ms_max = 0.0
	last_revision = -1
	last_camera_cell = Vector2i(2147483647, 2147483647)
	ready_for_world = true

func _process(_delta: float) -> void:
	visible = enabled
	if not enabled or not ready_for_world:
		return
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	var terrain_span: float = terrain.get_render_patch_span_m()
	if terrain_span <= 0.0:
		return
	terrain_span_m = terrain_span
	var divisor := patch_subdivision()
	var span := terrain_span / divisor
	patch_span_m = span
	owner_generations.clear()
	var camera_xz := Vector2(camera.global_position.x, camera.global_position.z)
	var camera_cell := Vector2i((camera_xz / span).floor())
	var revision: int = terrain.get_resident_patch_revision()
	# O(resident patches) only on residency/camera-cell changes, not O(total scatter).
	if revision != last_revision or camera_cell != last_camera_cell:
		last_revision = revision
		last_camera_cell = camera_cell
		var wanted: Dictionary = {}
		var world: Vector2 = simulation.get_terrain_world_size()
		# Terrain residency is the frustum test, and it is answered on the terrain grid. Each
		# resident terrain patch expands into the sub-patches it owns, so the frustum stays
		# terrain-coarse while every distance below is answered on the finer grid.
		var fine_radius := _fine_tier_radius()
		for terrain_key in terrain.get_resident_patch_keys():
			var block := Vector2i(terrain_key)
			var block_center := (Vector2(block) * terrain_span - world * 0.5
				+ Vector2.ONE * terrain_span * 0.5)
			var block_distance := block_center.distance_to(camera_xz)
			# Measured from the block's nearest corner, so a block with any part of it inside
			# the fine radius is carried whole on the fine grid.
			if block_distance - terrain_span * PATCH_HALF_DIAGONAL <= fine_radius:
				for column in range(divisor):
					for row in range(divisor):
						var key := Vector3i(block.x * divisor + column, block.y * divisor + row, divisor)
						var center := Vector2(key.x, key.y) * span - world * 0.5 + Vector2.ONE * span * 0.5
						if center.distance_to(camera_xz) <= canopy_far_m() + span:
							wanted[key] = center.distance_squared_to(camera_xz)
			elif block_distance <= canopy_far_m() + terrain_span:
				wanted[Vector3i(block.x, block.y, 1)] = block_center.distance_squared_to(camera_xz)
		for key in patches.keys():
			if not wanted.has(key):
				_retire_patch(key, world, camera_xz)
		# A cached patch outside the scatter radius will not be wanted again from here, so it
		# is freed. This is what bounds the cache: it holds at most the patches of one disk.
		for key in cache.keys():
			if _patch_distance(key, world, camera_xz) > canopy_far_m() + _key_span(key):
				cache[key].queue_free()
				cache.erase(key)
		queue.clear()
		for key in wanted:
			if patches.has(key):
				continue
			# A cached patch is already built. Showing it again costs no upload, so it does
			# not enter the queue and does not compete with a patch that has none.
			if cache.has(key):
				_restore_patch(key)
				continue
			queue.append(key)
		queue.sort_custom(func(a: Vector3i, b: Vector3i) -> bool: return wanted[a] > wanted[b])
	if queue.is_empty():
		# O(resident patches) per frame: one dictionary lookup and one distance. A road,
		# building or terrain edit advances the terrain surface generation of the patches it
		# covered, and only those patches regenerate. The two range checks rebuild a patch
		# when it crosses the distance where the understory or the near band starts or stops.
		var world: Vector2 = simulation.get_terrain_world_size()
		for key in patches:
			var patch: Node3D = patches[key]
			var key_span := _key_span(key)
			var distance := _patch_distance(key, world, camera_xz)
			_refresh_near_detail(patch, distance)
			if (
				_is_patch_stale(key, owner_generations)
				or _understory_wanted(distance, key_span) != bool(patch.get_meta("understory"))
				or _near_band_wanted(distance, key_span) != bool(patch.get_meta("near_band"))
				or _shadow_caster_wanted(distance, key_span) != int(patch.get_meta("shadow_caster"))
			):
				queue.append(key)
	# The budget is an area, not a count. One upload per frame was one terrain patch per
	# frame, and a sub-patch covers a square of that, so the same ground per frame is
	# PATCH_SUBDIVISION squared of them. Holding the count instead would make the time to
	# settle grow with the square of the subdivision, and in a dense forest that is minutes.
	var budget := upload_budget_override if upload_budget_override >= 1 else divisor * divisor
	while budget > 0 and not queue.is_empty():
		var next: Vector3i = queue.pop_back()
		_upload_patch(next, _key_span(next))
		budget -= 1

func _is_patch_stale(key: Vector3i, cache = null) -> bool:
	# -1 means the terrain patch has committed no payload yet. Keep the current scatter
	# rather than churning; the next commit advances the generation and triggers a rebuild.
	var generations := _owner_generations(_owner_key(key), cache)
	return (
		(generations.x >= 0 and generations.x != int(patches[key].get_meta("surface_generation")))
		or generations.y != int(patches[key].get_meta("vegetation_generation"))
	)

## The surface and vegetation generations of an owner patch, as (surface, vegetation). With a
## cache this crosses into Rust once per owner instead of once per caller; without one it
## always crosses, because the answer is only about the moment it is asked.
func _owner_generations(owner: Vector2i, cache) -> Vector2i:
	if cache != null and cache.has(owner):
		return cache[owner]
	var generations := Vector2i(
		terrain.get_patch_surface_generation(owner),
		simulation.get_vegetation_patch_generation(owner)
	)
	if cache != null:
		cache[owner] = generations
	return generations

## Camera position on the ground plane, or `INF` when there is no camera to measure from.
func _camera_xz() -> Vector2:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return Vector2.INF
	return Vector2(camera.global_position.x, camera.global_position.z)

## Distance from the camera to this patch centre, or a negative value with no camera. One
## distance feeds every range decision for the patch, so they cannot disagree.
func _patch_distance(key: Vector3i, world: Vector2, camera_xz: Vector2) -> float:
	if camera_xz == Vector2.INF:
		return -1.0
	var span := _key_span(key)
	return (Vector2(key.x, key.y) * span - world * 0.5
		+ Vector2.ONE * span * 0.5).distance_to(camera_xz)

## Whether this patch is close enough to be worth generating the dense understory layer.
## Conservative by the patch half-diagonal, like the near band: a patch centre outside the
## longest understory range can still have a near corner inside it. On the terrain grid that
## slack was 361 m on a 420 m range, so nearly half of every understory patch generated was
## never drawn.
func _understory_wanted(distance: float, span: float) -> bool:
	var longest := maxf(BUSH_RANGE_M, ROCK_RANGE_M) + span * PATCH_HALF_DIAGONAL
	return distance >= 0.0 and distance <= longest

## Whether any part of this patch can fall inside the near band, and so whether the near
## per-variant meshes are worth building at all. A patch is one instance, so the test is
## conservative by the patch half-diagonal rather than by the centre. Beyond this the patch
## builds one distant instance per canopy species and skips the variant split and tints.
## With no camera every level is built, which is what a headless caller wants.
func _near_band_wanted(distance: float, span: float) -> bool:
	return distance < 0.0 or distance <= canopy_near_m() + span * PATCH_HALF_DIAGONAL

## Which near mesh the patch's per-variant instances carry. The patch is one instance per
## variant, so this is a per-patch choice and needs no band wider than the patch.
func _near_detail_lod(distance: float) -> int:
	# With no camera every patch takes the detailed level, which is what a headless caller
	# wants: the catalogue test reads the mesh the instance holds.
	return 0 if distance < 0.0 or distance <= near_detail_m() else 1

## Moves resident near instances between the two branched levels. Both carry the same
## transforms and the same cards, so this is a mesh write on the buffer already uploaded.
func _refresh_near_detail(patch: Node3D, distance: float) -> void:
	var lod := _near_detail_lod(distance)
	if int(patch.get_meta("near_detail_lod")) == lod:
		return
	patch.set_meta("near_detail_lod", lod)
	for instance in patch.get_children():
		var species := int(instance.get_meta("species"))
		if species >= TreeSpecies.BUSH or int(instance.get_meta("lod")) != 0:
			continue
		instance.multimesh.mesh = meshes[species][int(instance.get_meta("variant"))][lod]

## Re-applies the shadow setting to resident patches. Avoids a full placement rebuild.
func set_cast_shadows(value: bool) -> void:
	cast_shadows = value
	for store in [patches, cache]:
		for patch in store.values():
			var caster: int = patch.get_meta("shadow_caster")
			for instance in patch.get_children():
				var is_proxy: bool = instance.get_meta("shadow_proxy")
				instance.cast_shadow = _shadow_setting(
					int(instance.get_meta("species")), int(instance.get_meta("lod")),
					is_proxy, caster
				)
				# OFF permits colour drawing, so a silent proxy must also be hidden.
				instance.visible = (not is_proxy
					or instance.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)

## Takes a patch out of the drawn set. A patch still inside the scatter radius is hidden and
## kept, because terrain residency is frustum-derived and will very likely ask for it again
## within a few frames. Only a patch that is genuinely out of range is freed.
func _retire_patch(key: Vector3i, world: Vector2, camera_xz: Vector2) -> void:
	var patch: Node3D = patches[key]
	tree_count -= int(patch.get_meta("tree_count"))
	patches.erase(key)
	if _patch_distance(key, world, camera_xz) <= canopy_far_m() + _key_span(key):
		patch.visible = false
		cache[key] = patch
		return
	patch.queue_free()

## Returns a hidden patch to the drawn set. The staleness and range tests in `_process` run
## against it on the following frame, so a patch that was edited while it was hidden still
## rebuilds; showing it first is what keeps the terrain from being bare in the meantime.
func _restore_patch(key: Vector3i) -> void:
	var patch: Node3D = cache[key]
	cache.erase(key)
	patch.visible = true
	patches[key] = patch
	tree_count += int(patch.get_meta("tree_count"))

func has_pending_work() -> bool:
	return enabled and (last_revision == -1 or not queue.is_empty())

func metrics() -> Dictionary:
	return {"enabled": enabled, "resident_patches": patches.size(), "resident_trees": tree_count,
		"pending_patches": queue.size(), "cached_patches": cache.size(),
		"generated_patches": generated_patches,
		"max_patch_generation_upload_ms": generation_ms_max, "density_fraction": density_fraction,
		"patch_span_m": patch_span_m, "patch_subdivision": patch_subdivision(),
		"fine_patches": _tier_count(patch_subdivision()), "coarse_patches": _tier_count(1),
		"fine_tier_radius_m": _fine_tier_radius(),
		"canopy_near_m": canopy_near_m(), "canopy_far_m": canopy_far_m()}

## How many resident patches sit on one grid. Reported so a probe can show what the two
## tiers actually cost against each other.
func _tier_count(divisor: int) -> int:
	var total := 0
	for key in patches:
		if key.z == divisor:
			total += 1
	return total

func _upload_patch(key: Vector3i, span: float) -> void:
	var start := Time.get_ticks_usec()
	# Read before the placement fetch. Stamping an older generation onto newer data only
	# causes one redundant rebuild; the reverse would leave a stale patch undetected.
	# Deliberately not the memoised read. The order matters here: this value is stamped onto
	# the patch, so it must be fetched after any edit that preceded this upload and before the
	# placement fetch below. A value memoised earlier in the frame can be newer than that and
	# would stamp an edit as already rendered.
	var owner := _owner_key(key)
	var generation: int = terrain.get_patch_surface_generation(owner)
	var vegetation_generation: int = simulation.get_vegetation_patch_generation(owner)
	var world: Vector2 = simulation.get_terrain_world_size()
	patch_span_m = span
	# A direct caller can name a span the key's own grid does not imply, so the key is made
	# to agree with it before any distance is measured from it.
	terrain_span_m = span * float(maxi(key.z, 1))
	var distance := _patch_distance(key, world, _camera_xz())
	var understory := _understory_wanted(distance, span)
	var near_band := _near_band_wanted(distance, span)
	var caster := _shadow_caster_wanted(distance, span)
	var near_detail_lod := _near_detail_lod(distance)
	var switch_m := canopy_switch_m(key)
	var origin := Vector2(key.x, key.y) * span - world * 0.5
	var data: PackedFloat32Array = simulation.get_decorative_tree_patch(origin, span, understory)
	var patch := Node3D.new()
	patch.position = Vector3(origin.x, 0.0, origin.y)
	add_child(patch)
	# One transform and tint array per variant, rebuilt per species. The near level draws
	# each array as its own MultiMesh and the distant level walks them in order, so the
	# placement loop needs no slot bookkeeping, index chains or scratch arrays. Appending
	# into a local typed array is what keeps this loop cheap: routing each placement through
	# a nested untyped Array instead cost 3.8 ms per patch on a 4096-placement fixture.
	# Out of the near band there is one transform bucket with a packed form layer per tree.
	# Work and storage remain O(placements) at upload; impostors add no per-frame CPU work.
	var count := 0
	for species in range(TreeSpecies.SPECIES_COUNT):
		var levels: Array = meshes[species][0]
		# Bush and rock have only a near level, so out of the near band they draw nothing.
		if not near_band and levels.size() == 1:
			continue
		# Two placement loops rather than one loop with two branches in it. The branches
		# would run once per placement, and per-placement interpreted work is what this
		# function is made of: the same reason the variant index reads seed bits instead of
		# hashing. The distant loop retains brush pins but needs no tint.
		var variant_transforms: Array = []
		var variant_tints: Array = []
		var species_count := 0
		var flat_layers := PackedFloat32Array()
		if near_band:
			var variant_count: int = TreeSpecies.VARIANT_COUNTS[species]
			variant_transforms.resize(variant_count)
			variant_tints.resize(variant_count)
			for variant in range(variant_count):
				var empty_transforms: Array[Transform3D] = []
				variant_transforms[variant] = empty_transforms
				var empty_tints: Array[Color] = []
				variant_tints[variant] = empty_tints
			for i in range(0, data.size(), 6):
				var packed := int(data[i + 5])
				if packed & SPECIES_MASK != species:
					continue
				# Stable subset for paired density experiments, independent of residency order.
				if fmod(absf(data[i] * 0.754877 + data[i + 2] * 0.56984), 1.0) >= density_fraction:
					continue
				var seed := _appearance_seed(data, i)
				var variant := _variant_index(species, seed, packed >> SPECIES_BITS)
				variant_transforms[variant].append(_instance_transform(data, i, species, seed))
				variant_tints[variant].append(_instance_tint(seed))
				species_count += 1
		else:
			var flat: Array[Transform3D] = []
			for i in range(0, data.size(), 6):
				var packed := int(data[i + 5])
				if packed & SPECIES_MASK != species:
					continue
				if fmod(absf(data[i] * 0.754877 + data[i + 2] * 0.56984), 1.0) >= density_fraction:
					continue
				var seed := _appearance_seed(data, i)
				var variant := _variant_index(species, seed, packed >> SPECIES_BITS)
				flat.append(_instance_transform(data, i, species, seed))
				flat_layers.append(TreeSpecies.impostor_layer(species, variant))
			species_count = flat.size()
			variant_transforms.append(flat)
		if species_count == 0:
			continue
		count += species_count
		if near_band:
			for variant in range(variant_transforms.size()):
				var near_transforms: Array[Transform3D] = variant_transforms[variant]
				if near_transforms.is_empty():
					continue
				var near_tints: Array[Color] = variant_tints[variant]
				var near_mm := MultiMesh.new()
				near_mm.transform_format = MultiMesh.TRANSFORM_3D
				near_mm.use_colors = true
				near_mm.mesh = meshes[species][variant][
					near_detail_lod if levels.size() > 1 else 0]
				near_mm.instance_count = near_transforms.size()
				for i in range(near_transforms.size()):
					near_mm.set_instance_transform(i, near_transforms[i])
					near_mm.set_instance_color(i, near_tints[i])
				_add_instance(patch, near_mm, species, 0, variant, near_band, switch_m, caster)
		# One buffer assignment per species, with transform rows followed by custom RGBA.
		# Near buckets already encode the variant; far-only placements supply their pin's layer.
		if levels.size() > 1:
			var far_mm := MultiMesh.new()
			far_mm.transform_format = MultiMesh.TRANSFORM_3D
			far_mm.use_custom_data = true
			far_mm.mesh = TreeSpecies.impostor_mesh()
			far_mm.instance_count = species_count
			var buffer := PackedFloat32Array()
			buffer.resize(species_count * 16)
			# The shader moves vertices outside the QuadMesh bounds. Include both baked
			# forms and the proxy before transforming, independent of the rendering backend.
			var object_bounds: AABB = levels[2].get_aabb()
			var material := TreeSpecies.impostor_material(species)
			var centres: PackedVector3Array = material.get_shader_parameter("bounds_centre")
			var sizes: PackedFloat32Array = material.get_shader_parameter("bounds_size")
			for layer in range(2):
				# A rotated square's corner reaches size / sqrt(2) from its centre.
				var radius := sizes[layer] * PATCH_HALF_DIAGONAL
				object_bounds = object_bounds.merge(AABB(centres[layer] - Vector3.ONE * radius,
					Vector3.ONE * radius * 2.0))
			# Per tree, only the origin span and the largest axis scale: transforming the box
			# for every tree was the dearest line of this loop. The box then grows by its
			# farthest corner at that scale, which contains every tree's box.
			var reach := 0.0
			for corner in range(8):
				reach = maxf(reach, object_bounds.get_endpoint(corner).length())
			var low := Vector3.INF
			var high := -Vector3.INF
			var scale_squared := 0.0
			var slot := 0
			for variant in range(variant_transforms.size()):
				var far_transforms: Array[Transform3D] = variant_transforms[variant]
				var layer := TreeSpecies.impostor_layer(species, variant)
				for i in range(far_transforms.size()):
					var placed := far_transforms[i]
					_write_impostor(buffer, slot * 16, placed, float(layer) if near_band else flat_layers[i])
					low = low.min(placed.origin)
					high = high.max(placed.origin)
					scale_squared = maxf(scale_squared,
						maxf(placed.basis.x.length_squared(), placed.basis.y.length_squared()))
					slot += 1
			far_mm.buffer = buffer
			var bounds := AABB(low, high - low).grow(reach * sqrt(scale_squared))
			far_mm.custom_aabb = bounds
			_add_instance(patch, far_mm, species, 2, 0, near_band, switch_m, caster)
			if caster == ShadowCaster.PROXY:
				# The same buffer, custom lane included, which the lathe shader never reads.
				# Packed arrays are copy-on-write, so this shares the data instead of copying it.
				var proxy := MultiMesh.new()
				proxy.transform_format = MultiMesh.TRANSFORM_3D
				proxy.use_custom_data = true
				proxy.mesh = levels[2]
				proxy.instance_count = species_count
				proxy.buffer = buffer
				proxy.custom_aabb = bounds
				_add_instance(patch, proxy, species, 2, 0, near_band, switch_m, caster, true)
	_share_patch_bounds(patch)
	patch.set_meta("tree_count", count)
	patch.set_meta("surface_generation", generation)
	patch.set_meta("vegetation_generation", vegetation_generation)
	patch.set_meta("understory", understory)
	patch.set_meta("near_band", near_band)
	patch.set_meta("shadow_caster", caster)
	patch.set_meta("near_detail_lod", near_detail_lod)
	# Replace in place. The previous node stays visible until this one is in the tree,
	# so an edit does not blank the surrounding forest for a frame.
	if patches.has(key):
		tree_count -= int(patches[key].get_meta("tree_count"))
		patches[key].queue_free()
	patches[key] = patch
	tree_count += count
	generated_patches += 1
	generation_ms_max = maxf(generation_ms_max, float(Time.get_ticks_usec() - start) / 1000.0)

# Godot's 3D MultiMesh buffer stores three ROWS, each ending with one origin component.
# Packed arrays are passed by reference; this writes the preallocated buffer without a copy.
func _write_impostor(buffer: PackedFloat32Array, offset: int, placed: Transform3D, layer: float) -> void:
	buffer[offset] = placed.basis.x.x
	buffer[offset + 1] = placed.basis.y.x
	buffer[offset + 2] = placed.basis.z.x
	buffer[offset + 3] = placed.origin.x
	buffer[offset + 4] = placed.basis.x.y
	buffer[offset + 5] = placed.basis.y.y
	buffer[offset + 6] = placed.basis.z.y
	buffer[offset + 7] = placed.origin.y
	buffer[offset + 8] = placed.basis.x.z
	buffer[offset + 9] = placed.basis.y.z
	buffer[offset + 10] = placed.basis.z.z
	buffer[offset + 11] = placed.origin.z
	buffer[offset + 12] = layer

## Gives every level in a patch one set of bounds, so they all change level on one distance.
## Godot measures a visibility range from the instance bounds, not from the node origin: a
## probe put two MultiMeshInstance3D at the camera's own position and culled the one whose
## single instance sat past the range. Each near level holds one variant and the distant level
## holds the whole species, so their bounds centres stand about 12 m apart on a random scatter
## and up to 67 m apart. The two ranges are complementary by construction and were not
## complementary in practice: over the metres between the two centres the distant level had
## already stopped and the near level had not yet started, and the trees of that variant were
## drawn by neither. Roughly half the variants sit on the losing side of that, which is why the
## canopy thins as the camera closes on the switch and fills back in a moment later.
##
## The pair is one species' near levels and that species' own distant level, so the merge is
## per species. A bush must not pull a 25 m canopy box into the box its own 420 m range is
## measured from, and a species with one level has no partner to fall between: it keeps its
## own bounds, which is also what leaves the understory stagger in `lod_range` an effect.
##
## One pass over the patch's own children, at upload, and nothing per frame. The dummy renderer
## reports empty automatic bounds; explicit impostor bounds also work headlessly.
func _share_patch_bounds(patch: Node3D) -> void:
	var bounds: Dictionary = {}
	var levels: Dictionary = {}
	for instance in patch.get_children():
		var species: int = instance.get_meta("species")
		var box: AABB = instance.multimesh.custom_aabb
		if box.size == Vector3.ZERO:
			box = instance.get_aabb()
		bounds[species] = box if not bounds.has(species) else (bounds[species] as AABB).merge(box)
		var seen: Dictionary = levels.get(species, {})
		seen[int(instance.get_meta("lod"))] = true
		levels[species] = seen
	for instance in patch.get_children():
		var species: int = instance.get_meta("species")
		var box: AABB = bounds[species]
		if box.size != Vector3.ZERO and ((levels[species] as Dictionary).size() > 1
			or instance.multimesh.use_custom_data):
			instance.custom_aabb = box

func _add_instance(
	patch: Node3D, mm: MultiMesh, species: int, lod: int, variant: int,
	near_band: bool, switch_m: float, caster: int, is_proxy: bool = false
) -> void:
	var instance := MultiMeshInstance3D.new()
	instance.multimesh = mm
	if lod == 2 and not is_proxy:
		instance.material_override = TreeSpecies.impostor_material(species)
	instance.set_meta("species", species)
	instance.set_meta("lod", lod)
	instance.set_meta("shadow_proxy", is_proxy)
	# Near instances record their bucket; distant instances carry their form in custom data.
	instance.set_meta("variant", variant)
	instance.cast_shadow = _shadow_setting(species, lod, is_proxy, caster)
	# A proxy is SHADOWS_ONLY when it casts and OFF when it does not, and OFF would let it
	# draw over the crown it stands inside, so a silent proxy has to be hidden outright.
	instance.visible = not is_proxy or instance.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var range_m := lod_range(species, lod, variant, near_band, switch_m)
	if is_proxy:
		# Range-culled instances cannot cast, so the proxy must also cover the near band.
		range_m = Vector2(0.0, canopy_far_m())
	# No fade mode. VISIBILITY_RANGE_FADE_SELF alpha-blends the whole instance, and
	# the instance is a whole patch: it made every plant in a patch translucent
	# whenever the patch centre sat in a band, however close the plant itself was,
	# and it moved the geometry out of the depth pre-pass to do it. The bands above
	# are now wider than a patch, so the switch is a step at a distance where one
	# patch changing silhouette is not readable.
	instance.visibility_range_begin = range_m.x
	instance.visibility_range_end = range_m.y
	patch.add_child(instance)

## Near mesh choice. A brush that plants one named tree passes a `pin` biased by one and that
## mesh is used; zero falls back to the seed's own bits, so the choice stays independent of
## proportions and lean. Bit extraction rather than another hash: the seed is already mixed,
## and one more hash call in the per-placement path is worth about a millisecond per patch on
## its own.
func _variant_index(species: int, appearance_seed: int, pin: int) -> int:
	if pin > 0:
		return pin - 1
	return ((appearance_seed >> 11) & 0xFFFF) % int(TreeSpecies.VARIANT_COUNTS[species])

## A small value spread and opposing red/blue shifts keep the foliage on its green axis.
## StandardMaterial3D multiplies instance and vertex colours, preserving per-face shade.
## Disjoint bit ranges of the already-mixed seed, for the reason given on _variant_index:
## the two hash calls this replaces cost 1.2 ms per patch on a 4096-placement fixture.
func _instance_tint(appearance_seed: int) -> Color:
	var value := lerpf(0.94, 1.06, float((appearance_seed >> 3) & 0xFFF) / 4096.0)
	var warmth := lerpf(-0.035, 0.035, float((appearance_seed >> 27) & 0xFFF) / 4096.0)
	return Color(value * (1.0 + warmth), value, value * (1.0 - warmth), 1.0)

## Default cosmetic seed from packed placement; never feeds simulation state.
func _appearance_seed(data: PackedFloat32Array, i: int) -> int:
	return hash(Vector2(data[i], data[i + 2]))

## Per-instance appearance. Yaw alone is nearly invisible on a near-symmetric lathe, so the
## variation that reads is the height/width ratio and a small lean off vertical.
## A caller can reroll those proportions with a different seed at the same placement.
func _instance_transform(
	data: PackedFloat32Array, i: int, species: int, appearance_seed: int
) -> Transform3D:
	var x := data[i]
	var z := data[i + 2]
	var scale := data[i + 4]
	# The four jitters below are the old _jitter() helper written out. Each of them ran once
	# per placement, and the call alone cost 2.6 ms per patch on a 4096-placement fixture --
	# more than the hashing it wrapped. The values are unchanged.
	var height := scale * (0.82 + float(hash(appearance_seed ^ 17) & 0xFFFFFF) / 16777216.0 * 0.42)
	var width := scale * (0.80 + float(hash(appearance_seed ^ 53) & 0xFFFFFF) / 16777216.0 * 0.48)
	# Rocks are squat and irregular; trees keep their proportions closer to upright.
	if species == TreeSpecies.ROCK:
		height *= 0.6
		width *= 1.1
	var lean := 0.0
	if species != TreeSpecies.ROCK:
		lean = 0.02 + float(hash(appearance_seed ^ 91) & 0xFFFFFF) / 16777216.0 * 0.07
	var lean_direction := float(hash(appearance_seed ^ 137) & 0xFFFFFF) / 16777216.0 * TAU
	var basis := (
		Basis(Vector3.UP, data[i + 3])
		* Basis(Vector3(cos(lean_direction), 0.0, sin(lean_direction)), lean)
		* Basis.from_scale(Vector3(width, height, width))
	)
	return Transform3D(basis, Vector3(x, data[i + 1], z))

## What one instance does in the shadow pass, from the patch's caster choice and whether this
## instance is the proxy. Understory is never a caster: it is the densest species and the key
## light runs four cascades, so each bush would be submitted up to five times while its own
## shadow sits under a canopy shadow that already darkens the same ground.
func _shadow_setting(species: int, lod: int, is_proxy: bool, caster: int) -> int:
	if not cast_shadows or species == TreeSpecies.ROCK or species == TreeSpecies.BUSH:
		return GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	if is_proxy:
		return (GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
			if caster == ShadowCaster.PROXY else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)
	if caster == ShadowCaster.NEAR and lod == 0:
		return GeometryInstance3D.SHADOW_CASTING_SETTING_ON
	return GeometryInstance3D.SHADOW_CASTING_SETTING_OFF

## Which level a patch casts from. Measured from the patch's nearest corner, so a patch with
## any part of it inside a boundary is treated as whole: one patch is one instance and cannot
## split. Beyond the sun's own range nothing it holds can reach a cascade, so it casts nothing.
func _shadow_caster_wanted(distance: float, span: float) -> int:
	# Deliberately independent of cast_shadows. This chooses which instances a patch holds, and
	# the runtime toggle must be able to turn casting back on without rebuilding a placement.
	var reach := distance - span * PATCH_HALF_DIAGONAL
	# With no camera every patch is a near caster, which is what a headless caller wants.
	if distance < 0.0 or reach <= SHADOW_PROXY_M:
		return ShadowCaster.NEAR
	if reach <= SceneLightingConfig.shadow_max_distance_m():
		return ShadowCaster.PROXY
	return ShadowCaster.NONE
