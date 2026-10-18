// SPDX-License-Identifier: GPL-2.0-only

shader_type spatial;
render_mode cull_back, specular_disabled;

// The same palette and world-space grass detail as terrain and authored lawns.
#include "res://scripts/shaders/ground_grass.gdshaderinc"
uniform float specular = 0.08;
#include "res://scripts/shaders/standard_light.gdshaderinc"
varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	float macro_mask = macro_variation(world_pos.xz);
	vec3 color = terrain_surface_color(world_pos.y, macro_mask, 0.0);
	color = apply_natural_land_variation(color, world_pos.xz, 0.0, 0.0, 0.0, 0.0, 0.0);
	ALBEDO = apply_grass_detail(color, world_pos, 1.0);
	ROUGHNESS = 0.96;
	standard_light_cloud_sun = standard_light_cloud(VERTEX, INV_VIEW_MATRIX);
}
