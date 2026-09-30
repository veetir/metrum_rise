shader_type spatial;
render_mode depth_prepass_alpha, shadows_disabled, cull_disabled;

uniform sampler2D heightmap : repeat_disable, filter_linear;
uniform sampler2D watermap : repeat_disable, filter_linear;
uniform sampler2D screen_texture : hint_screen_texture, repeat_disable, filter_linear_mipmap;
uniform sampler2D depth_texture : hint_depth_texture, repeat_disable, filter_nearest;
uniform float height_scale = 10.0;
uniform vec4 shallow_water_color : source_color = vec4(0.10, 0.28, 0.40, 0.62);
uniform vec4 deep_water_color : source_color = vec4(0.018, 0.09, 0.18, 0.90);
uniform vec4 foam_color : source_color = vec4(0.76, 0.91, 0.96, 0.82);
uniform vec4 sky_reflection_color : source_color = vec4(0.38, 0.56, 0.72, 1.0);
uniform float shore_softness_m = 0.26;
uniform float shore_foam_band_m = 0.18;
uniform float water_deep_color_depth_m = 3.5;
uniform float water_fresnel_strength = 0.46;
uniform float water_fresnel_power = 2.8;
uniform float water_wave_color_strength = 0.052;
uniform float water_wave_roughness_strength = 0.024;
uniform float water_wave_normal_strength = 0.42;
uniform float water_sun_glitter_strength = 0.38;
uniform float water_refraction_strength = 0.010;
uniform float water_refraction_mix = 0.13;
uniform float water_surface_smoothing = 0.94;
uniform int water_visual_debug_mode = 0;
global uniform vec3 scene_sun_direction;
global uniform vec4 scene_sun_color : source_color;
global uniform vec4 scene_sky_color : source_color;
global uniform float scene_ambient_strength;
// Water is lit normally, so it darkens with the environment on its own. This is used only to
// scale the sun glitter, whose strength is otherwise a fixed constant: without it the moon lays
// down a noon-bright specular path across the lake.
global uniform vec4 scene_ambient_light : source_color;
uniform float scene_shadow_max_distance_m = 1800.0;
uniform vec3 scene_shadow_split_distances_m = vec3(144.0, 396.0, 900.0);
uniform vec2 water_debug_patch_key = vec2(0.0);
uniform float water_debug_lod_step = 1.0;
uniform vec2 watermap_texture_size = vec2(1.0, 1.0);
uniform vec2 inner_sample_offset_texels = vec2(1.0, 1.0);
uniform vec2 inner_sample_size_texels = vec2(2.0, 2.0);

varying vec2 var_centered_uv;
varying vec3 var_world_pos;
varying vec3 var_sun_direction_view;
varying vec3 var_view_direction;
varying float var_view_distance_m;
// Water surface height in the patch's own space, which the terrain samples share.
varying float var_surface_m;

vec2 centered_uv(vec2 uv) {
	vec2 safe_inner_size = max(inner_sample_size_texels, vec2(1.0));
	return (inner_sample_offset_texels + uv * (safe_inner_size - 1.0) + 0.5)
		/ max(watermap_texture_size, vec2(1.0));
}

float sample_terrain_world_height(vec2 uv) {
	return texture(heightmap, uv).r * height_scale;
}

float sample_raw_water_depth(vec2 uv) {
	return texture(watermap, uv).r;
}

// Metres a vertex with no water near it sinks below the terrain, so the triangles that reach it
// from the shore slope away underground instead of climbing the bank.
const float WATER_BURY_M = 2.0;

// Water surface elevation for a vertex: the mean surface of the wet samples on a 5 x 5 block
// of texels, weighting the centre double and keeping part of a wet centre's own
// surface. Two spacings of reach keep a flat, marshy shore level until the terrain rises through
// it; with one, the surface was buried along a mesh edge and drew the shore as a straight line.
// A dry vertex beside a lake takes the lake's level, so the lake stays one flat plane out
// under its bank and the terrain cuts the shoreline where it rises through that plane. Taking
// the dry vertex's own terrain height instead tilted every shore triangle up the bank, which
// drew water as sloped planes climbing the land. `found` is false with no water within reach.
float water_display_surface(vec2 uv, out bool found) {
	float center_depth = sample_raw_water_depth(uv);
	float center_surface = sample_terrain_world_height(uv) + center_depth;
	// One texel, not one vertex spacing: the block has to stay inside the patch texture's border.
	vec2 texel = vec2(1.0) / max(watermap_texture_size, vec2(1.0, 1.0));
	float accum_surface = 0.0;
	float accum_weight = 0.0;
	for (int z = -2; z <= 2; z++) {
		for (int x = -2; x <= 2; x++) {
			vec2 at = uv + vec2(float(x), float(z)) * texel;
			float depth = sample_raw_water_depth(at);
			if (depth > 0.001) {
				float weight = (x == 0 && z == 0) ? 2.0 : 1.0;
				accum_surface += (sample_terrain_world_height(at) + depth) * weight;
				accum_weight += weight;
			}
		}
	}
	found = accum_weight > 0.0;
	if (!found) {
		return 0.0;
	}
	float average_surface = accum_surface / accum_weight;
	return center_depth > 0.001
		? mix(center_surface, average_surface, clamp(water_surface_smoothing, 0.0, 1.0))
		: average_surface;
}

void vertex() {
	var_centered_uv = centered_uv(UV);
	bool found;
	float surface = water_display_surface(var_centered_uv, found);
	VERTEX.y += found ? surface + 0.05 : sample_terrain_world_height(var_centered_uv) - WATER_BURY_M;
	var_surface_m = VERTEX.y - 0.05;
	var_world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	var_sun_direction_view = normalize((MODELVIEW_MATRIX * vec4(normalize(scene_sun_direction), 0.0)).xyz);
	vec3 view_position = (MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	var_view_direction = normalize(-view_position);
	var_view_distance_m = length(view_position);
}

float hash12(vec2 p) {
	return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

vec3 value_noise_field(vec2 p) {
	vec2 cell = floor(p);
	vec2 local = fract(p);
	vec2 smooth_local = local * local * (3.0 - 2.0 * local);
	vec2 smooth_derivative = 6.0 * local * (1.0 - local);
	float n00 = hash12(cell);
	float n10 = hash12(cell + vec2(1.0, 0.0));
	float n01 = hash12(cell + vec2(0.0, 1.0));
	float n11 = hash12(cell + vec2(1.0, 1.0));
	float row_0 = mix(n00, n10, smooth_local.x);
	float row_1 = mix(n01, n11, smooth_local.x);
	return vec3(
		mix(row_0, row_1, smooth_local.y),
		mix(n10 - n00, n11 - n01, smooth_local.y) * smooth_derivative.x,
		(row_1 - row_0) * smooth_derivative.y
	);
}

vec3 water_noise_octave(
	vec2 world_xz,
	float scale,
	vec2 axis,
	vec2 drift,
	vec2 offset
) {
	vec2 perpendicular = vec2(-axis.y, axis.x);
	vec2 rotated_world = vec2(dot(world_xz, axis), dot(world_xz, perpendicular));
	vec3 field = value_noise_field(rotated_world * scale + drift * TIME + offset);
	vec2 world_gradient = (axis * field.y + perpendicular * field.z) * scale;
	return vec3(field.x, world_gradient);
}

vec3 water_surface_field(vec2 world_xz) {
	// Rotated, independently drifting octaves avoid exposing either the source
	// value-noise cells or an infinite periodic wave train in high camera views.
	vec3 octave_1 = water_noise_octave(
		world_xz,
		0.0055,
		vec2(0.939693, 0.342020),
		vec2(0.026, -0.018),
		vec2(0.0)
	);
	vec3 octave_2 = water_noise_octave(
		world_xz,
		0.0117,
		vec2(0.573576, -0.819152),
		vec2(-0.021, 0.029),
		vec2(17.4, -9.1)
	);
	vec3 octave_3 = water_noise_octave(
		world_xz,
		0.0240,
		vec2(0.258819, 0.965926),
		vec2(0.017, 0.013),
		vec2(-31.7, 22.8)
	);
	vec3 combined = octave_1 * 0.58 + octave_2 * 0.29 + octave_3 * 0.13;
	return vec3(combined.x, combined.yz * 7.5);
}

vec3 perturb_water_normal(vec3 base_normal, vec2 wave_slope, float strength) {
	vec3 axis = abs(base_normal.y) < 0.98 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(axis, base_normal));
	vec3 bitangent = normalize(cross(base_normal, tangent));
	return normalize(base_normal + (tangent * wave_slope.x + bitangent * wave_slope.y) * strength);
}

vec3 water_world_normal(vec2 wave_slope) {
	return normalize(vec3(
		-wave_slope.x * water_wave_normal_strength * 2.6,
		1.0,
		-wave_slope.y * water_wave_normal_strength * 2.6
	));
}

float water_sun_glitter(vec3 view_normal, float breakup) {
	vec3 sun_dir = normalize(var_sun_direction_view);
	vec3 normal = normalize(view_normal);
	vec3 view_dir = normalize(var_view_direction);
	vec3 reflected_sun = reflect(-sun_dir, normal);
	float alignment = clamp(dot(reflected_sun, view_dir), 0.0, 1.0);
	float broad_path = pow(alignment, 30.0);
	float bright_core = pow(alignment, 72.0);
	float broken_surface = mix(0.72, 1.0, smoothstep(0.38, 0.78, breakup));
	return clamp(
		broad_path * broken_surface * 0.55
			+ bright_core * mix(0.55, 0.85, broken_surface),
		0.0,
		1.0
	);
}

// Screen-space reflection of the opaque scene in the water. Godot's own SSR skips transparent
// materials, so the water reflected only a flat sky colour: no shore, no trees, no buildings.
// The reflected view ray is marched through the opaque depth buffer in geometrically growing
// steps and refined by bisection on a hit. O(REFLECTION_STEPS + REFLECTION_REFINE) texture
// reads per water pixel. `weight` is zero when the ray finds neither scene nor sky on screen, and
// fades toward the screen edge, where what the ray would meet is not on screen.
const int REFLECTION_STEPS = 24;
const int REFLECTION_REFINE = 5;

vec3 water_view_at(vec2 uv, mat4 inv_projection) {
	float depth = texture(depth_texture, uv).r;
	vec4 view = inv_projection * vec4(uv * 2.0 - 1.0, depth, 1.0);
	return view.xyz / view.w;
}

vec2 water_screen_at(vec3 view, mat4 projection) {
	vec4 clip = projection * vec4(view, 1.0);
	return clip.xy / clip.w * 0.5 + 0.5;
}

// A ray that meets no scene reflects the sky at infinity along it: the screen point its direction
// projects to, where the sky is drawn. That is continuous from pixel to pixel, which the last sky
// sample along each march was not: neighbouring rays stopped on different steps and banded.
vec3 water_reflection(vec3 view_pos, vec3 view_normal, mat4 projection, mat4 inv_projection,
		out float weight) {
	weight = 0.0;
	vec3 direction = reflect(normalize(view_pos), view_normal);
	// A ray turning back toward the camera leaves the screen at once.
	if (direction.z > -0.02) {
		return vec3(0.0);
	}
	float near_t = 0.0;
	float far_t = max(length(view_pos) * 0.02, 0.5);
	for (int i = 0; i < REFLECTION_STEPS; i++) {
		vec3 sample_view = view_pos + direction * far_t;
		vec2 uv = water_screen_at(sample_view, projection);
		if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
			break;
		}
		float scene_depth = texture(depth_texture, uv).r;
		vec4 scene = inv_projection * vec4(uv * 2.0 - 1.0, scene_depth, 1.0);
		float scene_z = scene.z / scene.w;
		// Behind a drawn surface and not far behind it: the ray passed through that surface.
		if (scene_depth > 0.0 && scene_depth < 1.0 && sample_view.z < scene_z
				&& scene_z - sample_view.z < max(far_t * 0.35, 3.0)) {
			for (int r = 0; r < REFLECTION_REFINE; r++) {
				float mid_t = (near_t + far_t) * 0.5;
				vec3 mid_view = view_pos + direction * mid_t;
				if (mid_view.z < water_view_at(water_screen_at(mid_view, projection), inv_projection).z) {
					far_t = mid_t;
				} else {
					near_t = mid_t;
				}
			}
			uv = water_screen_at(view_pos + direction * far_t, projection);
			vec2 edge = min(uv, vec2(1.0) - uv);
			weight = smoothstep(0.0, 0.08, min(edge.x, edge.y));
			return textureLod(screen_texture, uv, 0.0).rgb;
		}
		near_t = far_t;
		far_t *= 1.38;
	}
	// Off screen the sky continues from the screen's own edge: a steep ray leaves through the top
	// and meets the top row, which is near the zenith that real water reflects there. One row
	// repeated draws its clouds as streaks, so the farther off screen, the blurrier the mip.
	vec4 infinity = projection * vec4(direction, 0.0);
	vec2 unclamped = infinity.xy / infinity.w * 0.5 + 0.5;
	vec2 sky_uv = clamp(unclamped, vec2(0.002), vec2(0.998));
	float sky_depth = texture(depth_texture, sky_uv).r;
	// Nothing drawn there but sky: the far plane in either depth convention.
	if (sky_depth <= 0.0 || sky_depth >= 1.0) {
		weight = 1.0;
		float off_screen = length(unclamped - sky_uv);
		return textureLod(screen_texture, sky_uv, smoothstep(0.0, 0.4, off_screen) * 6.0).rgb;
	}
	return vec3(0.0);
}

float water_debug_patch_border(vec2 patch_uv) {
	vec2 edge_distance = min(patch_uv, vec2(1.0) - patch_uv);
	float nearest_edge = min(edge_distance.x, edge_distance.y);
	return 1.0 - smoothstep(0.006, 0.018, nearest_edge);
}

vec3 water_debug_patch_color(vec2 key) {
	return vec3(
		0.25 + hash12(key + vec2(17.3, 3.9)) * 0.70,
		0.25 + hash12(key + vec2(61.1, 29.4)) * 0.70,
		0.25 + hash12(key + vec2(8.8, 83.7)) * 0.70
	);
}

vec3 water_debug_lod_color(float lod_step) {
	if (lod_step <= 1.5) {
		return vec3(0.10, 0.85, 0.22);
	}
	if (lod_step <= 2.5) {
		return vec3(0.05, 0.70, 1.00);
	}
	if (lod_step <= 4.5) {
		return vec3(1.00, 0.78, 0.08);
	}
	return vec3(1.00, 0.12, 0.08);
}

vec3 water_material_debug_color(
	vec2 patch_uv,
	vec3 depth_tint,
	float alpha_value,
	float fresnel_value,
	float foam_value,
	float normal_value
) {
	float column = floor(clamp(patch_uv.x, 0.0, 0.999) * 5.0);
	vec3 color = depth_tint;
	if (column < 1.0) {
		color = depth_tint;
	} else if (column < 2.0) {
		color = vec3(alpha_value);
	} else if (column < 3.0) {
		color = mix(vec3(0.03, 0.07, 0.12), sky_reflection_color.rgb, fresnel_value);
	} else if (column < 4.0) {
		color = mix(vec3(0.02, 0.05, 0.08), foam_color.rgb, foam_value);
	} else {
		color = vec3(normal_value, normal_value * 0.65, 1.0 - normal_value * 0.55);
	}
	float separator = step(fract(patch_uv.x * 5.0), 0.018);
	return mix(color, vec3(1.0), separator * 0.65);
}

vec3 water_lighting_debug_color(
	vec2 patch_uv,
	vec3 world_normal,
	float fresnel_value,
	float specular_value,
	float normal_value
) {
	float column = floor(clamp(patch_uv.x, 0.0, 0.999) * 5.0);
	vec3 sun_dir = normalize(scene_sun_direction);
	float sun_facing = clamp(dot(world_normal, sun_dir), 0.0, 1.0);
	float camera_distance_m = var_view_distance_m;
	vec3 cascade_color = vec3(1.00, 0.20, 0.14);
	if (camera_distance_m < scene_shadow_split_distances_m.x) {
		cascade_color = vec3(0.20, 0.82, 0.28);
	} else if (camera_distance_m < scene_shadow_split_distances_m.y) {
		cascade_color = vec3(0.12, 0.58, 1.00);
	} else if (camera_distance_m < scene_shadow_split_distances_m.z) {
		cascade_color = vec3(1.00, 0.78, 0.16);
	}
	vec3 color = mix(vec3(0.02, 0.04, 0.08), scene_sun_color.rgb, sun_facing);
	if (column < 1.0) {
		color = mix(vec3(0.02, 0.04, 0.08), scene_sun_color.rgb, sun_facing);
	} else if (column < 2.0) {
		color = mix(vec3(0.02, 0.03, 0.05), vec3(1.0, 0.94, 0.72), specular_value);
	} else if (column < 3.0) {
		color = mix(vec3(0.03, 0.07, 0.12), scene_sky_color.rgb, fresnel_value);
	} else if (column < 4.0) {
		color = cascade_color;
	} else {
		color = vec3(normal_value, normal_value * 0.65, 1.0 - normal_value * 0.55);
	}
	float separator = step(fract(patch_uv.x * 5.0), 0.018);
	return mix(color, vec3(1.0), separator * 0.65);
}

vec3 water_visual_debug_color(vec2 patch_uv, float depth_m, float shore_mask) {
	vec3 debug_color = vec3(0.0);
	if (water_visual_debug_mode == 1 || water_visual_debug_mode == 8) {
		debug_color = water_debug_patch_color(water_debug_patch_key);
	} else if (water_visual_debug_mode == 2 || water_visual_debug_mode == 7) {
		debug_color = water_debug_lod_color(water_debug_lod_step);
	} else if (water_visual_debug_mode == 3) {
		float terrain_height = sample_terrain_world_height(var_centered_uv);
		debug_color = vec3(clamp(terrain_height / max(height_scale, 0.001), 0.0, 1.0));
	} else if (water_visual_debug_mode == 5) {
		debug_color = vec3(shore_mask, depth_m > 0.001 ? 0.45 : 0.0, 1.0 - shore_mask);
	} else {
		float depth_visibility = smoothstep(0.0, 6.0, depth_m);
		debug_color = mix(vec3(0.02, 0.04, 0.08), vec3(0.02, 0.60, 1.00), depth_visibility);
	}
	float border = water_debug_patch_border(patch_uv);
	return mix(debug_color, vec3(1.0), border);
}

void fragment() {
	// The water column under this pixel: the level surface above the terrain. The shoreline
	// fades where the terrain rises through the surface, so it follows the terrain's own contour
	// rather than the straight segments of the filtered watermap between its 10 m samples.
	float contour_depth_m = max(var_surface_m - sample_terrain_world_height(var_centered_uv), 0.0);
	float depth_m = contour_depth_m;
	float shore_mask = smoothstep(0.0, max(shore_softness_m, 0.001), contour_depth_m);
	float depth_factor = smoothstep(0.12, max(water_deep_color_depth_m, 0.121), depth_m);
	vec3 surface_field = water_surface_field(var_world_pos.xz);
	float wave = surface_field.x;
	float wave_signed = wave * 2.0 - 1.0;
	vec2 wave_slope = surface_field.yz;
	vec3 world_normal = water_world_normal(wave_slope);
	float normal_debug_strength = clamp(length(wave_slope) * water_wave_normal_strength * 18.0, 0.0, 1.0);
	vec3 depth_tint = mix(shallow_water_color.rgb, deep_water_color.rgb, depth_factor);
	depth_tint *= 1.0 + wave_signed * water_wave_color_strength;
	float shore_foam = (1.0 - smoothstep(0.0, max(shore_foam_band_m, 0.001), contour_depth_m))
		* shore_mask;
	float foam_amount = clamp(shore_foam * 0.16, 0.0, 0.34);
	float final_alpha = mix(shallow_water_color.a, deep_water_color.a, depth_factor) * shore_mask;
	final_alpha = clamp(final_alpha, 0.0, 0.92);

	vec3 base_view_normal = normalize(NORMAL);
	NORMAL = perturb_water_normal(base_view_normal, wave_slope, water_wave_normal_strength);
	float fresnel = pow(
		1.0 - clamp(dot(base_view_normal, normalize(var_view_direction)), 0.0, 1.0),
		water_fresnel_power
	);
	float fresnel_mix = clamp(fresnel * water_fresnel_strength, 0.0, 1.0);
	float sun_specular = water_sun_glitter(NORMAL, wave)
		* shore_mask
		* (0.40 + depth_factor * 0.60);
	vec3 sky_reflection = mix(sky_reflection_color.rgb, scene_sky_color.rgb, 0.65);

	if (water_visual_debug_mode > 0) {
		if (water_visual_debug_mode == 9) {
			ALBEDO = water_material_debug_color(
				UV,
				depth_tint,
				final_alpha,
				fresnel_mix,
				foam_amount,
				normal_debug_strength
			);
		} else if (water_visual_debug_mode == 10) {
			ALBEDO = water_lighting_debug_color(
				UV,
				world_normal,
				fresnel_mix,
				sun_specular,
				normal_debug_strength
			);
		} else {
			ALBEDO = water_visual_debug_color(UV, depth_m, shore_mask);
		}
		ALPHA = 0.92;
		ROUGHNESS = 1.0;
		SPECULAR = 0.0;
	} else {
		// Terrain above the level surface ends the water. Clipping at the watermap's own zero
		// contour instead drew the grid lines through its dry samples as straight shore edges.
		if (contour_depth_m <= 0.001) {
			discard;
		}

		vec3 final_color = mix(depth_tint, foam_color.rgb, foam_amount);
		float shallow_refraction = (1.0 - depth_factor) * shore_mask;
		vec2 refraction_offset = wave_slope * water_refraction_strength * shallow_refraction;
		vec3 refracted_scene = texture(screen_texture, clamp(SCREEN_UV + refraction_offset, vec2(0.001), vec2(0.999))).rgb;
		final_color = mix(
			final_color,
			refracted_scene,
			clamp(shallow_refraction * water_refraction_mix, 0.0, 0.35)
		);
		// Reflected scene where the ray finds one, the sky tint where it does not. The scene is
		// already lit, so it is emitted rather than lit again as albedo. The flat view-space up
		// carries a small share of the wave slope, which ripples the reflection.
		vec3 view_up = normalize((VIEW_MATRIX * vec4(0.0, 1.0, 0.0, 0.0)).xyz);
		vec3 ripple_normal = perturb_water_normal(view_up, wave_slope, water_wave_normal_strength * 0.25);
		float reflection_weight;
		vec3 reflected_scene = water_reflection(VERTEX, ripple_normal, PROJECTION_MATRIX,
			INV_PROJECTION_MATRIX, reflection_weight);
		final_color = mix(final_color, sky_reflection, fresnel_mix * (1.0 - reflection_weight));
		final_color *= 1.0 - fresnel_mix * reflection_weight;
		EMISSION = reflected_scene * fresnel_mix * reflection_weight;
		final_color += scene_sun_color.rgb
			* scene_ambient_light.rgb
			* sun_specular
			* water_sun_glitter_strength;
		float topdown_ripple = smoothstep(0.48, 0.86, wave)
			* normal_debug_strength
			* (1.0 - fresnel_mix * 0.35)
			* 0.07;
		final_color = mix(final_color, sky_reflection, topdown_ripple);
		ALBEDO = final_color;
		// A reflection is light leaving the surface, not the lake bed seen through it.
		ALPHA = mix(final_alpha, 1.0, fresnel_mix * shore_mask);
		ROUGHNESS = clamp(0.08 + normal_debug_strength * water_wave_roughness_strength, 0.05, 0.20);
		// The engine's own specular reflects the sky; where the scene is reflected instead, it
		// would lay that sky over the reflected shore and wash it out.
		SPECULAR = 0.72 * (1.0 - reflection_weight);
	}
}
