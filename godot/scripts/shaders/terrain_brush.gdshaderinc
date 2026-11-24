// SPDX-License-Identifier: GPL-3.0-or-later

global uniform bool terrain_brush_enabled;
// Centre XZ, radius, and stroke state. Grade holds mode, level and the two anchor heights.
global uniform vec4 terrain_brush_cursor;
global uniform vec4 terrain_brush_grade;
global uniform vec4 terrain_brush_ramp;
global uniform int terrain_brush_ramp_count;
global uniform sampler2D terrain_brush_protection : filter_nearest, repeat_disable;
// First sample XZ, sample spacing and square texture width.
global uniform vec4 terrain_brush_protection_grid;

void apply_terrain_brush(vec3 world, inout vec3 albedo, inout vec3 emission) {
	float radius = terrain_brush_cursor.z;
	float distance_m = distance(world.xz, terrain_brush_cursor.xy);
	float edge_distance = distance_m - radius;
	float pixel_m = max(fwidth(edge_distance), 0.0001);
	float aa = 0.5 * pixel_m;
	if (edge_distance < aa) {
		float disc = 1.0 - smoothstep(-aa, aa, edge_distance);
		float stroke = min(2.5 * pixel_m, radius * 0.4);
		float edge = smoothstep(-stroke - aa, -stroke + aa, edge_distance);
		float falloff = 0.5 + 0.5 * cos(3.14159265 * min(distance_m / radius, 1.0));
		vec3 ring_color = vec3(0.55, 0.92, 1.0);
		vec3 ink = terrain_brush_cursor.w > 1.5 ? vec3(1.0, 0.70, 0.19) : ring_color;
		float opacity = disc * max(0.24 * falloff, 0.85 * edge);
		bool ramp = terrain_brush_grade.x == 4.0 && terrain_brush_ramp_count == 2;
		bool flatten = terrain_brush_grade.x == 2.0;
		if (ramp || flatten) {
			float target = terrain_brush_grade.y;
			if (ramp) {
				vec2 a = terrain_brush_ramp.xy;
				vec2 ab = terrain_brush_ramp.zw - a;
				float t = clamp(dot(world.xz - a, ab) / max(dot(ab, ab), 1.0), 0.0, 1.0);
				target = mix(terrain_brush_grade.z, terrain_brush_grade.w, t);
			}
			float cut = world.y - target;
			vec3 grade = cut > 0.0 ? vec3(0.96, 0.40, 0.11) : vec3(0.30, 0.52, 1.0);
			float change = smoothstep(0.025, 0.65, abs(cut));
			ink = mix(ink, grade, change);
			// Cut/fill belongs to the disc; its boundary and ramp guide share one cyan ink.
			ink = mix(ink, ring_color, edge);
			opacity = disc * max(0.30 * falloff * change, 0.85 * edge);
		}
		if (terrain_brush_protection_grid.w > 0.0) {
			vec2 sample_pos = (world.xz - terrain_brush_protection_grid.xy) / terrain_brush_protection_grid.z;
			vec2 uv = (sample_pos + 0.5) / terrain_brush_protection_grid.w;
			if (texture(terrain_brush_protection, uv).r > 0.0) {
				ink = vec3(1.0, 0.12, 0.13);
				opacity = disc * max(0.57, opacity);
			}
		}
		albedo = mix(albedo, ink, opacity);
		emission = mix(emission, ink * 0.54, opacity);
	}
	if (terrain_brush_ramp_count > 0) {
		vec2 a = terrain_brush_ramp.xy;
		if (terrain_brush_ramp_count == 2) {
			vec2 b = terrain_brush_ramp.zw;
			vec2 ab = b - a;
			float t = clamp(dot(world.xz - a, ab) / max(dot(ab, ab), 1.0), 0.0, 1.0);
			float guide_m = distance(world.xz, mix(a, b, t));
			// Differentiate the signed distance: abs(distance) can cancel across a pixel quad
			// straddling the centreline, breaking the thin guide into dashes at far zoom.
			float signed_m = dot(world.xz - a, vec2(-ab.y, ab.x)) / max(length(ab), 0.001);
			float pixel_m = max(fwidth(signed_m), 0.0001);
			// The vegetation ring's 2.5 px stroke and the road line's one-pixel soft edge.
			float coverage = clamp(1.75 - guide_m / pixel_m, 0.0, 1.0);
			vec3 guide = vec3(0.55, 0.92, 1.0);
			albedo = mix(albedo, guide, 0.95 * coverage);
			emission = mix(emission, 0.6 * guide, 0.95 * coverage);
		}
	}
}
