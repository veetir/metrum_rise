// SPDX-License-Identifier: GPL-2.0-only

// Forward+ copies opaque depth before the transparent pass. Never write overlay depth.
uniform sampler2D scene_depth : hint_depth_texture, repeat_disable, filter_nearest;

float zoning_visibility(vec2 screen_uv, float cell_distance, mat4 inverse_projection) {
    float depth = textureLod(scene_depth, screen_uv, 0.0).r;
    vec3 ndc = vec3(screen_uv * 2.0 - 1.0, depth);
#if CURRENT_RENDERER == RENDERER_COMPATIBILITY
    ndc.z = depth * 2.0 - 1.0;
#endif
    vec4 scene_view = inverse_projection * vec4(ndc, 1.0);
    float scene_distance = -scene_view.z / scene_view.w;
    // Metres along the view axis: tolerate the four-corner mesh's terrain penetration,
    // fading over 1.10–1.40 m rather than flickering at a single depth threshold.
    return 1.0 - smoothstep(1.10, 1.40, cell_distance - scene_distance);
}
