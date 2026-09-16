shader_type spatial;
render_mode world_vertex_coords;

#include "res://scripts/shaders/vegetation_wind.gdshaderinc"

varying float canopy_light;

void vertex() {
	VERTEX += vegetation_wind_offset(MODEL_MATRIX[3].xz, COLOR.a, TIME);
	canopy_light = vegetation_canopy_shade(
		MODEL_MATRIX[3].xz, distance(VERTEX, CAMERA_POSITION_WORLD)
	);
}

void fragment() {
	// Godot folds MultiMesh instance tint into COLOR before vertex(). Alpha is weight only.
	ALBEDO = COLOR.rgb * canopy_light;
	ROUGHNESS = 1.0;
	BACKLIGHT = vegetation_backlight(COLOR.rgb) * canopy_light;
}
