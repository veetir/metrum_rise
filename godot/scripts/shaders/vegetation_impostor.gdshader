shader_type spatial;
render_mode cull_disabled;

#include "res://scripts/shaders/vegetation_wind.gdshaderinc"

uniform sampler2DArray albedo_atlas : source_color, filter_linear_mipmap, repeat_disable;
uniform sampler2DArray normal_atlas : filter_linear_mipmap, repeat_disable;
uniform vec3 bounds_centre[2];
uniform float bounds_size[2];
uniform float impostor_radiance_match = 1.0;
// A mip texel averages the normals under it, and the bake keeps that mean unnormalized: its
// shortfall from unit length measures how far those normals disagreed. Renormalizing it would
// light a whole distant crown by the one normal facing the camera, which is a closed shell,
// and a shell halves in brightness when the camera faces the sun. The near crown is seen
// through, so its response hardly depends on the view. Scattered texels therefore lift toward
// up and wrap their diffuse, which is the volume response the near crown has. Both gains were
// fitted over 36 camera and sun poses; see vegetation_level_match_test.gd. Roughness stays at
// the near crown's one, so the impostor reflects as much sky as the cards it replaces: a
// specular-free impostor lacked that blue and read as a warmer forest than the near band.
const float VOLUME_LIFT_GAIN = 3.0;
const float VOLUME_WRAP_GAIN = 1.5;

varying vec2 frame_uv0;
varying vec2 frame_uv1;
varying vec2 frame_uv2;
varying flat vec2 cell0;
varying flat vec2 cell1;
varying flat vec2 cell2;
varying flat vec3 weights;
varying flat float form_layer;
varying float canopy_light;
varying float shadow_keep;
varying float volume_wrap;

vec3 hemi_decode(vec2 uv) {
	vec2 p = vec2(uv.x + uv.y, uv.x - uv.y) * 0.5;
	return normalize(vec3(p.x, 1.0 - abs(p.x) - abs(p.y), p.y));
}

vec2 hemi_encode(vec3 d) {
	d.y = max(d.y, 0.0);
	vec3 a = d / (abs(d.x) + abs(d.y) + abs(d.z));
	return vec2(a.x + a.z, a.x - a.z);
}

vec2 project_frame(vec3 offset, vec2 cell) {
	vec3 d = hemi_decode((cell + 0.5) / 8.0 * 2.0 - 1.0);
	vec3 right = normalize(cross(vec3(0.0, 1.0, 0.0), d));
	vec3 up = cross(d, right);
	return vec2(dot(offset, right), -dot(offset, up)) + 0.5;
}

void vertex() {
	int layer = int(INSTANCE_CUSTOM.x);
	form_layer = float(layer);
	vec3 centre = bounds_centre[layer];
	vec3 camera_object = (inverse(MODEL_MATRIX) * vec4(CAMERA_POSITION_WORLD, 1.0)).xyz;
	vec3 view = normalize(camera_object - centre);
	vec2 f = (hemi_encode(view) * 0.5 + 0.5) * 8.0 - 0.5;
	vec2 base = floor(f);
	vec2 part = fract(f);
	if (part.x + part.y < 1.0) {
		cell0 = base;
		weights = vec3(1.0 - part.x - part.y, part.x, part.y);
	} else {
		cell0 = base + vec2(1.0);
		weights = vec3(part.x + part.y - 1.0, 1.0 - part.y, 1.0 - part.x);
	}
	cell0 = clamp(cell0, vec2(0.0), vec2(7.0));
	cell1 = clamp(base + vec2(1.0, 0.0), vec2(0.0), vec2(7.0));
	cell2 = clamp(base + vec2(0.0, 1.0), vec2(0.0), vec2(7.0));
	vec3 right = cross(vec3(0.0, 1.0, 0.0), view);
	// Exactly overhead has no preferred azimuth; keep a finite camera-facing basis.
	right = dot(right, right) > 0.000001 ? normalize(right) : vec3(1.0, 0.0, 0.0);
	vec3 up = cross(view, right);
	vec3 offset = right * (UV.x - 0.5) + up * (0.5 - UV.y);
	VERTEX = centre + offset * bounds_size[layer];
	NORMAL = view;
	frame_uv0 = project_frame(offset, cell0);
	frame_uv1 = project_frame(offset, cell1);
	frame_uv2 = project_frame(offset, cell2);
	vec3 world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	float view_distance_m = distance(world, CAMERA_POSITION_WORLD);
	canopy_light = vegetation_canopy_shade(MODEL_MATRIX[3].xz, view_distance_m);
	shadow_keep = vegetation_shadow_keep(view_distance_m);
}

// Limit sampling to the last independent frame mip. Its coarser trilinear tap
// determines the inset, so even a minified frame cannot sample an adjacent view.
float frame_lod(vec2 uv) {
	float footprint = max(length(dFdx(uv)), length(dFdy(uv))) * 128.0;
	return clamp(log2(max(footprint, 1.0)), 0.0, 7.0);
}

vec3 atlas_uv(vec2 uv, vec2 cell, float lod) {
	float inset = 0.5 * exp2(ceil(lod)) / 128.0;
	return vec3((cell + clamp(uv, vec2(inset), vec2(1.0 - inset))) / 8.0, form_layer);
}

void fragment() {
	float lod0 = frame_lod(frame_uv0);
	float lod1 = frame_lod(frame_uv1);
	float lod2 = frame_lod(frame_uv2);
	vec3 uv0 = atlas_uv(frame_uv0, cell0, lod0);
	vec3 uv1 = atlas_uv(frame_uv1, cell1, lod1);
	vec3 uv2 = atlas_uv(frame_uv2, cell2, lod2);
	vec4 albedo = textureLod(albedo_atlas, uv0, lod0) * weights.x
		+ textureLod(albedo_atlas, uv1, lod1) * weights.y
		+ textureLod(albedo_atlas, uv2, lod2) * weights.z;
	vec3 n = (textureLod(normal_atlas, uv0, lod0).xyz * weights.x
		+ textureLod(normal_atlas, uv1, lod1).xyz * weights.y
		+ textureLod(normal_atlas, uv2, lod2).xyz * weights.z) * 2.0 - 1.0;
	float spread = 1.0 - clamp(length(n), 0.0, 1.0);
	vec3 object_normal = normalize(mix(n, vec3(0.0, 1.0, 0.0), clamp(VOLUME_LIFT_GAIN * spread, 0.0, 1.0)));
	NORMAL = normalize(mat3(VIEW_MATRIX) * MODEL_NORMAL_MATRIX * object_normal);
	ALBEDO = albedo.rgb * canopy_light * impostor_radiance_match;
	BACKLIGHT = vegetation_backlight(albedo.rgb) * canopy_light * impostor_radiance_match;
	ROUGHNESS = 1.0;
	volume_wrap = clamp(VOLUME_WRAP_GAIN * spread, 0.0, 1.0);
	ALPHA = albedo.a;
	ALPHA_SCISSOR_THRESHOLD = 0.4;
}

void light() {
	float shadow = vegetation_shadow(ATTENUATION, LIGHT_IS_DIRECTIONAL, shadow_keep);
	// No sun highlight. A lifted normal faces the sky, and against a low sun ahead the grazing
	// Fresnel term lit the impostor 47% above the near crown, whose normals mostly face away.
	DIFFUSE_LIGHT += vegetation_diffuse_light(NORMAL, LIGHT, LIGHT_COLOR, shadow, BACKLIGHT,
		volume_wrap);
}
