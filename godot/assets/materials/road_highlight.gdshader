// SPDX-License-Identifier: GPL-2.0-only
//
// Road selection and hover highlight (SelectTool): a ribbon over the road whose UV.x runs
// across it (0 at the left edge, 1 at the right) and UV.y along it in metres, in the road's
// drawn direction. It draws a faint fill, a crisp glowing outline at both edges and, when
// chevron_alpha > 0, chevrons that drift in the drawn direction, so "left" and "right" (the
// parking lane, the sidewalk sides) read off the highlight. Unshaded: it marks, it is not lit.
shader_type spatial;
render_mode unshaded, cull_disabled, depth_test_disabled, blend_mix, shadows_disabled, fog_disabled;

uniform vec4 tint : source_color = vec4(0.38, 0.80, 0.88, 1.0);
uniform float fill_alpha = 0.10;
uniform float glow_alpha = 0.30;
uniform float edge_alpha = 0.95;
// Outline width in metres.
uniform float edge_m = 0.35;
// The ribbon's width in metres, for the outline and chevron sizes.
uniform float width_m = 10.0;
uniform float chevron_alpha = 0.22;
uniform float chevron_spacing_m = 7.0;
uniform float chevron_speed_mps = 2.5;

void fragment() {
	// Distance in metres to the nearer edge.
	float d = min(UV.x, 1.0 - UV.x) * width_m;
	float aa = fwidth(d) * 1.2 + 0.001;
	float edge = 1.0 - smoothstep(edge_m - aa, edge_m + aa, d);
	float glow = exp(-max(d - edge_m, 0.0) * 1.1) * glow_alpha;
	// Chevrons: lines where (along + across offset) is constant, apex ahead in the drawn direction.
	float across = abs(UV.x - 0.5) * width_m;
	float phase = fract((UV.y + across * 0.7 - TIME * chevron_speed_mps) / chevron_spacing_m);
	float band = smoothstep(0.0, 0.05, phase) * (1.0 - smoothstep(0.12, 0.17, phase));
	float chevron = band * chevron_alpha * smoothstep(edge_m + 0.4, edge_m + 1.2, d);
	ALBEDO = mix(tint.rgb, vec3(1.0), edge * 0.35);
	ALPHA = clamp(fill_alpha + glow + chevron + edge * edge_alpha, 0.0, 1.0);
}
