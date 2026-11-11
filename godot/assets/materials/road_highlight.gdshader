// SPDX-License-Identifier: GPL-2.0-only
//
// Road selection and hover highlight (SelectTool). Two kinds of strip, told apart by the vertex
// colour's red channel:
// - the road ribbon (red 0): UV.x runs across it (0 at the left edge, 1 at the right, looking
//   along the road's drawn direction) and UV.y along it in metres. It draws a faint fill and a
//   crisp glowing outline at both edges; `side_glow` lights one half (1 left, 2 right), so the
//   Road Properties window's Left and Right read off the road.
// - one strip per vehicle lane (red 1): UV.x across the lane, UV.y in metres along the lane's
//   direction of travel. When chevron_alpha > 0 it draws chevrons drifting the way that lane's
//   traffic goes, so a two-way road shows both directions.
// Unshaded: it marks, it is not lit.
shader_type spatial;
render_mode unshaded, cull_disabled, depth_test_disabled, blend_mix, shadows_disabled, fog_disabled;

uniform vec4 tint : source_color = vec4(0.38, 0.80, 0.88, 1.0);
uniform float fill_alpha = 0.10;
uniform float glow_alpha = 0.30;
uniform float edge_alpha = 0.95;
// Outline width in metres.
uniform float edge_m = 0.35;
// The ribbon's width in metres, for the outline size.
uniform float width_m = 10.0;
// A lane strip's width in metres.
uniform float lane_m = 3.5;
uniform float chevron_alpha = 0.22;
uniform float chevron_spacing_m = 7.0;
uniform float chevron_speed_mps = 2.5;
// 0 none, 1 the left half, 2 the right half.
uniform int side_glow = 0;
uniform float side_glow_alpha = 0.16;

void fragment() {
	if (COLOR.r > 0.5) {
		// Chevrons: lines where (along + across offset) is constant, apex ahead in the lane's
		// direction of travel, fading out before the lane's sides.
		float across = abs(UV.x - 0.5) * lane_m;
		float phase = fract((UV.y + across * 0.7 - TIME * chevron_speed_mps) / chevron_spacing_m);
		float band = smoothstep(0.0, 0.05, phase) * (1.0 - smoothstep(0.12, 0.17, phase));
		ALBEDO = tint.rgb;
		ALPHA = band * chevron_alpha * (1.0 - smoothstep(lane_m * 0.36, lane_m * 0.48, across));
	} else {
		// Distance in metres to the nearer edge.
		float d = min(UV.x, 1.0 - UV.x) * width_m;
		float aa = fwidth(d) * 1.2 + 0.001;
		float edge = 1.0 - smoothstep(edge_m - aa, edge_m + aa, d);
		float glow = exp(-max(d - edge_m, 0.0) * 1.1) * glow_alpha;
		float side = 0.0;
		if (side_glow != 0) {
			float half_aa = fwidth(UV.x) * 1.5;
			float left = 1.0 - smoothstep(0.5 - half_aa, 0.5 + half_aa, UV.x);
			side = (side_glow == 1 ? left : 1.0 - left) * side_glow_alpha;
		}
		ALBEDO = mix(tint.rgb, vec3(1.0), edge * 0.35);
		ALPHA = clamp(fill_alpha + glow + side + edge * edge_alpha, 0.0, 1.0);
	}
}
