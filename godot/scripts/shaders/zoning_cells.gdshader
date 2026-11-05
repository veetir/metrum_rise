// SPDX-License-Identifier: GPL-2.0-only

shader_type spatial;
render_mode unshaded, blend_mix, cull_disabled, depth_test_disabled, depth_draw_never, fog_disabled;

#include "res://scripts/shaders/zoning_depth.gdshaderinc"

void fragment() {
    // Fragment VERTEX is already in view space, including perspective interpolation.
    float distance_m = -VERTEX.z;
    float visibility = zoning_visibility(SCREEN_UV, distance_m, INV_PROJECTION_MATRIX);
    vec2 border = min(UV, vec2(1.0) - UV);
    vec2 aa = max(fwidth(UV), vec2(0.00001));
    vec2 edge_axes = vec2(1.0) - smoothstep(vec2(0.015) - aa, vec2(0.015) + aa, border);
    float edge = max(edge_axes.x, edge_axes.y) * (1.0 - smoothstep(180.0, 650.0, distance_m));
    // Packed alpha: occupied=0, unzoned=0.5, zoned=1. Both quiet styles are outline-only.
    float fill = 0.30 * smoothstep(0.5, 1.0, COLOR.a);
    float edge_alpha = mix(0.45, 0.75, step(0.25, COLOR.a));
    ALBEDO = COLOR.rgb;
    ALPHA = mix(fill, edge_alpha, edge) * visibility;
}
