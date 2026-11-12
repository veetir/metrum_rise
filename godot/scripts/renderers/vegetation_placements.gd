# SPDX-License-Identifier: GPL-2.0-only

## Draws a handful of Rust placements (6 floats each: x, y, z, yaw, scale, species | biased
## variant << 2, as `yard_preview_plants` and `get_vegetation_preset_sample` pack them) with the
## game's own meshes, seed, proportions and tint, one MultiMeshInstance3D per mesh. For previews
## of a few dozen plants (the asset editor's yard, the terrain menu's thumbnails), not the world.
extends RefCounted

const TreeSpecies = preload("res://scripts/renderers/tree_species.gd")
const Vegetation = preload("res://scripts/renderers/vegetation.gd")

## A node holding `placements` drawn with `meshes` (`TreeSpecies.build_meshes()`), the nearest
## level of each. A hedge module keeps its packed yaw and no tint, as the world draws it.
static func instances(placements: PackedFloat32Array, meshes: Array) -> Node3D:
	var root := Node3D.new()
	# This helper never enters the tree, so it starts no world streaming or processing.
	var appearance := Vegetation.new()
	var buckets := {}
	for i in range(0, placements.size(), 6):
		var packed := int(placements[i + 5])
		var species := packed & 3
		var seed: int = appearance._appearance_seed(placements, i)
		var variant: int = appearance._variant_index(species, seed, packed >> 2)
		var key := Vector2i(species, variant)
		if not buckets.has(key):
			buckets[key] = []
		var hedge := TreeSpecies.is_hedge(species, variant)
		var placed: Transform3D = (Transform3D(Basis(Vector3.UP, placements[i + 3]),
			Vector3(placements[i], placements[i + 1], placements[i + 2])) if hedge
			else appearance._instance_transform(placements, i, seed, Vector2.ZERO))
		buckets[key].append([placed, Color.WHITE if hedge else appearance._instance_tint(seed)])
	for key: Vector2i in buckets:
		var entries: Array = buckets[key]
		var mm := MultiMesh.new()
		mm.transform_format = MultiMesh.TRANSFORM_3D
		mm.use_colors = true
		mm.mesh = meshes[key.x][key.y][0]
		mm.instance_count = entries.size()
		for i in entries.size():
			mm.set_instance_transform(i, entries[i][0])
			mm.set_instance_color(i, entries[i][1])
		var instance := MultiMeshInstance3D.new()
		instance.multimesh = mm
		root.add_child(instance)
	appearance.free()
	return root
