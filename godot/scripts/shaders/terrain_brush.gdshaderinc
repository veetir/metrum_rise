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
		vec3 ink = terrain_brush_cursor.w > 1.5 ? vec3(1.0, 0.70, 0.19) : vec3(0.28, 0.88, 0.86);
		float opacity = disc * max(0.55 * falloff, 0.82 * edge);
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
			opacity = disc * max(0.67 * falloff * change, 0.66 * edge);
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
		float guide_m = distance(world.xz, a);
		float guide_pixel = max(length(fwidth(world.xz)), 0.001);
		if (terrain_brush_ramp_count == 2) {
			vec2 b = terrain_brush_ramp.zw;
			vec2 ab = b - a;
			float t = clamp(dot(world.xz - a, ab) / max(dot(ab, ab), 1.0), 0.0, 1.0);
			guide_m = min(guide_m, distance(world.xz, b));
			guide_m = min(guide_m, distance(world.xz, mix(a, b, t)) + max(1.4, 1.8 * guide_pixel));
		}
		float width = max(2.1, 3.5 * guide_pixel);
		float outer = 1.0 - smoothstep(width, width + guide_pixel, guide_m);
		float center = 1.0 - smoothstep(width * 0.62, width * 0.62 + guide_pixel, guide_m);
		vec3 guide = mix(vec3(0.04, 0.018, 0.09), vec3(0.63, 0.22, 0.93), center);
		albedo = mix(albedo, guide, outer);
		emission = mix(emission, 0.55 * guide, outer);
	}
}
