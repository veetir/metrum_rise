shader_type spatial;
render_mode world_vertex_coords;

#include "res://scripts/shaders/vegetation_wind.gdshaderinc"

varying float canopy_light;
varying float shadow_keep;
// One for canopy trees, which have impostors to hand over to. Understory shares these materials
// and keeps its own ranges, so it stays at zero. Set per instance by vegetation.gd.
instance uniform float hands_over = 0.0;

void vertex() {
	// Measured before the displacement, which is bounded by half a metre and cannot change
	// which side of the gate a tree falls on. One distance now serves both terms.
	float view_distance_m = distance(VERTEX, CAMERA_POSITION_WORLD);
	VERTEX += vegetation_wind_offset(MODEL_MATRIX[3].xz, COLOR.a, TIME,
		vegetation_wind_visibility(view_distance_m, VIEWPORT_SIZE));
	canopy_light = vegetation_canopy_shade(MODEL_MATRIX[3].xz, view_distance_m);
	shadow_keep = vegetation_shadow_keep(view_distance_m);
	float crossfade = hands_over > 0.5
		? vegetation_crossfade(MODEL_MATRIX[3].xyz, MAIN_CAM_INV_VIEW_MATRIX[3].xyz) : 0.0;
	// Wood takes no dither: a discard would move every trunk off the opaque fast path, and a
	// trunk in the handover is about two pixels wide. It leaves at the middle of its tree's
	// handover, when the impostor has drawn half the pixels and its own baked trunk. Its
	// shadow goes to the proxy with the cards' shadow; see vegetation_casts_branched.
	bool shadow_pass = vegetation_shadow_pass(CAMERA_POSITION_WORLD, MAIN_CAM_INV_VIEW_MATRIX[3].xyz);
	if (shadow_pass
			? hands_over > 0.5 && !vegetation_casts_branched(MODEL_MATRIX[3].xyz,
				MAIN_CAM_INV_VIEW_MATRIX[3].xyz)
			: crossfade >= 0.5) {
		VERTEX = MODEL_MATRIX[3].xyz;
	}
}

void fragment() {
	// Godot folds MultiMesh instance tint into COLOR before vertex(). Alpha is weight only.
	ALBEDO = COLOR.rgb * canopy_light;
	ROUGHNESS = 1.0;
	BACKLIGHT = vegetation_backlight(COLOR.rgb) * canopy_light;
}

void light() {
	float shadow = vegetation_shadow(ATTENUATION, LIGHT_IS_DIRECTIONAL, shadow_keep);
	DIFFUSE_LIGHT += vegetation_diffuse_light(NORMAL, LIGHT, LIGHT_COLOR, shadow, BACKLIGHT, 0.0);
	SPECULAR_LIGHT += vegetation_specular_light(NORMAL, LIGHT, VIEW, LIGHT_COLOR, shadow,
		SPECULAR_AMOUNT);
}
