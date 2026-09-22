shader_type spatial;
render_mode diffuse_lambert_wrap;

#include "res://scripts/shaders/vegetation_wind.gdshaderinc"

// Filtered-mask retention measured against the unchanged near crowns at 16 px.
// Lower than the former .22 point mask; no extra surfaces or texture fetches.
uniform float crown_coverage : hint_range(0.0, 1.0) = 0.18;
// Rendered radiance, not albedo, is what has to agree across the level switch. The near crown
// is a cloud of cards whose normals point everywhere and which shadow each other and hide a dark
// interior; the distant crown is one convex lathe whose every visible face turns outward and
// catches the key light almost fully. Giving the lathe the near crown's mean albedo therefore
// draws it far brighter than the crowns it stands in for. This is the measured ratio between the
// two levels' rendered luminance; see DISTANT_RADIANCE_MATCH in tree_species.gd.
uniform float distant_radiance_match : hint_range(0.0, 2.0) = 1.0;
// The near crown is seen through: its gaps show cards on the far side of the crown, whose
// normals face every way, so its luminance hardly changes as the sun moves around the camera.
// The lathe is a closed shell that shows only the half facing the camera, so its luminance
// halved when the camera looked toward the sun. Tilting the shell normals toward world up
// removes the dependence on the sun's azimuth relative to the view. The wrap then flattens the
// response to sun elevation to match the near crown's, which is flatter for a conifer than for a
// broadleaf. See DISTANT_CROWN_WRAP in tree_species.gd.
const float CROWN_NORMAL_LIFT = 0.8;
uniform float crown_wrap : hint_range(0.0, 1.0) = 0.5;
varying vec3 crown_position;
varying float canopy_light;

void vertex() {
	crown_position = VERTEX;
	// No world_vertex_coords here: the hash below needs object space, so the world position
	// the shade ramp measures from is built rather than read.
	canopy_light = vegetation_canopy_shade(
		MODEL_MATRIX[3].xz,
		distance((MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz, CAMERA_POSITION_WORLD)
	);
}

// Power-of-two object-space cells follow the fragment footprint.
// Unsigned arithmetic is deterministic; neither TIME nor screen coordinates participates.
float crown_sample(vec3 position, float cell_size) {
	uvec3 cell = uvec3(ivec3(floor(position / cell_size)));
	uint h = cell.x * 374761393u + cell.y * 668265263u + cell.z * 2246822519u;
	h = (h ^ (h >> 13u)) * 1274126177u;
	return float((h ^ (h >> 16u)) & 16777215u) / 16777216.0;
}

// Convert a mixture of two uniform hashes back to a uniform sample. This avoids
// a loss of coverage halfway through an octave, without switching masks abruptly.
float filtered_sample(vec3 position, float footprint) {
	float octave = max(log2(footprint * 4.0), 0.0);
	float cell_size = exp2(floor(octave)) * 0.25;
	float t = fract(octave);
	float x = mix(crown_sample(position, cell_size), crown_sample(position, cell_size * 2.0), t);
	float a = max(t, 1.0 - t);
	float b = min(t, 1.0 - t);
	if (b < 0.00001) { return x; }
	if (x < b) { return x * x / (2.0 * a * b); }
	if (x > a) { return 1.0 - (1.0 - x) * (1.0 - x) / (2.0 * a * b); }
	return (x - 0.5 * b) / a;
}

void fragment() {
	float footprint = max(length(dFdx(crown_position)), length(dFdy(crown_position)));
	bool foliage = COLOR.g > COLOR.r;
	// Foliage keeps the near crown mean. Wood takes a canopy-shadow tone instead of
	// its own bark colour: a lit trunk at a kilometre is the brightest thing on the
	// mesh, and the real forest hides its trunks behind the canopy at that range.
	float radiance = canopy_light * distant_radiance_match;
	ALBEDO = (foliage ? COLOR.rgb : vec3(0.025, 0.030, 0.010)) * radiance;
	// diffuse_lambert_wrap takes its wrap from ROUGHNESS. Specular is off, so nothing else
	// reads it, and a specular lobe from a lifted normal would brighten a back-lit view.
	ROUGHNESS = crown_wrap;
	SPECULAR = 0.0;
	NORMAL = normalize(mix(NORMAL, (VIEW_MATRIX * vec4(0.0, 1.0, 0.0, 0.0)).xyz, CROWN_NORMAL_LIFT));
	BACKLIGHT = vegetation_backlight(COLOR.rgb) * radiance;
	// Unresolvable crowns gradually fill; wood stays sparse and in canopy shadow.
	float coverage = foliage ? mix(crown_coverage, 1.0, smoothstep(4.0, 8.0, footprint)) : 0.04;
	ALPHA = step(filtered_sample(crown_position, footprint), coverage);
	ALPHA_SCISSOR_THRESHOLD = 0.4;
}
