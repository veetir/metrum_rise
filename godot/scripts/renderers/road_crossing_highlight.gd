# SPDX-License-Identifier: GPL-2.0-only

## Unshaded selection ribbon for roads crossed by the current over/under preview.
extends RefCounted

var _mesh: MeshInstance3D

func show_roads(parent: Node3D, simulation: Node, edges: PackedInt64Array) -> void:
	if _mesh == null:
		_mesh = MeshInstance3D.new()
		var material := StandardMaterial3D.new()
		material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		material.cull_mode = BaseMaterial3D.CULL_DISABLED
		material.albedo_color = Color(1.0, 0.75, 0.1, 0.3)
		_mesh.material_override = material
		_mesh.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		parent.add_child(_mesh)
	var mesh := ImmediateMesh.new()
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	for edge in edges:
		var points: PackedVector3Array = simulation.get_edge_geometry_3d(edge)
		var width: float = simulation.get_edge_width(edge) * 0.5 + 1.5
		for i in range(points.size() - 1):
			var a := points[i] + Vector3.UP * 0.08
			var b := points[i + 1] + Vector3.UP * 0.08
			var tangent := (b - a).normalized()
			var lateral := Vector3(-tangent.z, 0.0, tangent.x) * width
			for point in [a - lateral, a + lateral, b + lateral, a - lateral, b + lateral, b - lateral]:
				mesh.surface_add_vertex(point)
	mesh.surface_end()
	_mesh.mesh = mesh

func clear() -> void:
	if is_instance_valid(_mesh):
		_mesh.mesh = null
