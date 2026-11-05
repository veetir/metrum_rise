// SPDX-License-Identifier: GPL-2.0-only

shader_type spatial;
render_mode unshaded, blend_mix, cull_disabled, depth_test_disabled, depth_draw_never, fog_disabled;

#include "res://scripts/shaders/zoning_depth.gdshaderinc"

uniform vec4 line_color : source_color = vec4(1.0, 0.5, 0.0, 0.9);

void fragment() {
    ALBEDO = line_color.rgb;
    ALPHA = line_color.a * zoning_visibility(SCREEN_UV, -VERTEX.z, INV_PROJECTION_MATRIX);
}
