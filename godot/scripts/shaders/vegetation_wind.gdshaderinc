// World XZ direction; zero direction disables displacement without a normalization NaN.
uniform vec2 wind_direction = vec2(1.0, 0.35);
uniform float wind_strength : hint_range(0.0, 2.0) = 1.0;
// Radians per second; wavelength is in world metres.
uniform float wind_speed : hint_range(0.0, 4.0) = 0.8;
uniform float gust_wavelength : hint_range(1.0, 500.0) = 80.0;

// O(1) per near vertex: three sine terms, no textures or inverse transforms.
// Only instance origin and authored weight participate, so cores and cards stay together.
vec3 vegetation_wind_offset(vec2 instance_xz, float weight, float time) {
	vec2 direction = wind_direction / max(length(wind_direction), 0.0001);
	vec2 across = vec2(-direction.y, direction.x);
	float phase = time * wind_speed - dot(instance_xz, direction) * TAU / max(gust_wavelength, 1.0);
	float bend = 0.36 * sin(phase) + 0.08 * sin(phase * 0.5);
	float flutter = 0.06 * sin(phase * 4.0) * weight;
	// At weight 1 and default strength, the offset is bounded by 0.50 m.
	vec2 offset = (direction * bend + across * flutter) * weight * wind_strength;
	return vec3(offset.x, 0.0, offset.y);
}

// Transmission tint for light passing through a leaf, not reflecting off it.
uniform vec3 foliage_backlight : source_color = vec3(0.080, 0.150, 0.040);

// Foliage transmits light; bark does not. One material carries both, and splitting them
// would cost draw calls on a scatter that is already draw-call bound, so the authored
// palette's green bias is the discriminator: every foliage colour has g well above r,
// every bark and rock colour does not. No per-vertex tag and no extra attribute.
vec3 vegetation_backlight(vec3 albedo) {
	return foliage_backlight * smoothstep(0.0, 0.05, albedo.g - albedo.r);
}

// Canopy shading. The key light runs four cascades that end at the scene shadow range, well
// inside the vegetation draw range. Past them every crown is lit on every surface the sun can
// reach, so a stand loses the one thing that makes a forest darker than a single tree: its
// members shadow each other. That loss is why distant forest reads lighter than the same
// forest close up, and it begins at the cascade edge, not at the mesh change much further out.
//
// The replacement is one multiplier per instance that fades in exactly where the cascades fade
// out, so nothing inside shadow range changes. It is keyed on the instance origin, which the
// near and the distant mesh of one tree share, so a tree keeps its shade when its mesh changes.
uniform float canopy_shade_begin_m = 327.6;
uniform float canopy_shade_end_m = 420.0;
uniform float canopy_shade : hint_range(0.0, 2.0) = 1.0;

// Share of the key light a shadowed crown keeps. A crown in a neighbour's shadow still collects
// sky light and light scattered off the stand around it, so it is not black. This value leaves
// the multiplier with a mean of 0.70, which is what a canopy keeps when about two fifths of its
// visible surface sits in shadow at ambient level.
const float CANOPY_SHADE_FLOOR = 0.40;
// Metres per noise cell. Canopy shadow falls in clumps of crowns rather than tree by tree, and
// an uncorrelated value per instance reads as static at the distances this applies to: nothing
// here filters it over time. Default canopy density is 30.5 stems per hectare, so mean spacing
// is 18 m and one cell covers a group of roughly seven trees.
const float CANOPY_SHADE_CELL_M = 48.0;

// Deterministic value in [0,1) for one integer cell. Unsigned arithmetic only; neither TIME nor
// screen coordinates participates, so two runs of one frame shade the stand identically.
float canopy_shade_cell(vec2 cell) {
	uvec2 c = uvec2(ivec2(cell));
	uint h = c.x * 374761393u + c.y * 668265263u;
	h = (h ^ (h >> 13u)) * 1274126177u;
	return float((h ^ (h >> 16u)) & 16777215u) / 16777216.0;
}

// Smoothly interpolated value noise on the ground plane. Four hashes, and every vertex of one
// instance gets the same answer because the input is the instance origin rather than the vertex.
float canopy_shade_noise(vec2 instance_xz) {
	vec2 p = instance_xz / CANOPY_SHADE_CELL_M;
	vec2 cell = floor(p);
	vec2 f = p - cell;
	f = f * f * (3.0 - 2.0 * f);
	float a = canopy_shade_cell(cell);
	float b = canopy_shade_cell(cell + vec2(1.0, 0.0));
	float c = canopy_shade_cell(cell + vec2(0.0, 1.0));
	float d = canopy_shade_cell(cell + vec2(1.0, 1.0));
	return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

// Multiplier on reflected and transmitted foliage colour for one instance at one distance.
float vegetation_canopy_shade(vec2 instance_xz, float distance_m) {
	float shade = mix(CANOPY_SHADE_FLOOR, 1.0, canopy_shade_noise(instance_xz));
	float ramp = smoothstep(canopy_shade_begin_m, canopy_shade_end_m, distance_m) * canopy_shade;
	// The clamp is what lets `canopy_shade` run past one. A strength above one extrapolates
	// past the floor, which is how a probe asks for more darkening than the derived default.
	return clamp(mix(1.0, shade, ramp), 0.0, 1.0);
}
