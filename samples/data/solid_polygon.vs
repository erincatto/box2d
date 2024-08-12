// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

uniform mat4 projectionMatrix;
uniform float pixelScale;

layout(location = 0) in vec2 v_localPosition;
layout(location = 1) in vec4 v_instanceTransform;
layout(location = 2) in vec4 v_instancePoints12;
layout(location = 3) in vec4 v_instancePoints34;
layout(location = 4) in vec4 v_instancePoints56;
layout(location = 5) in vec4 v_instancePoints78;
layout(location = 6) in int v_instanceCount;
layout(location = 7) in float v_instanceRadius;
layout(location = 8) in vec4 v_instanceColor;

out vec2 f_position;
out vec4 f_color;
out vec2 f_points[8];
flat out int f_count;
out float f_radius;
out float f_thickness;

void main()
{
    f_position = v_localPosition;
    f_color = v_instanceColor;

    f_radius = v_instanceRadius;
    f_count = v_instanceCount;

    f_points[0] = v_instancePoints12.xy;
    f_points[1] = v_instancePoints12.zw;
    f_points[2] = v_instancePoints34.xy;
    f_points[3] = v_instancePoints34.zw;
    f_points[4] = v_instancePoints56.xy;
    f_points[5] = v_instancePoints56.zw;
    f_points[6] = v_instancePoints78.xy;
    f_points[7] = v_instancePoints78.zw;

    // Compute polygon AABB
    vec2 lower = f_points[0];
    vec2 upper = f_points[0];
    for (int i = 1; i < v_instanceCount; ++i)
    {
        lower = min(lower, f_points[i]);
        upper = max(upper, f_points[i]);
    }

    vec2 center = 0.5 * (lower + upper);
    vec2 width = upper - lower;
    float maxWidth = max(width.x, width.y);

    float scale = f_radius + 0.5 * maxWidth;
    float invScale = 1.0 / scale;

    // Shift and scale polygon points so they fit in 2x2 quad
    for (int i = 0; i < f_count; ++i)
    {
        f_points[i] = invScale * (f_points[i] - center);
    }

    // Scale radius as well
    f_radius = invScale * f_radius;

    // resolution.y = pixelScale * scale
    f_thickness = 3.0f / (pixelScale * scale);

    // scale up and transform quad to fit polygon
    float x = v_instanceTransform.x;
    float y = v_instanceTransform.y;
    float c = v_instanceTransform.z;
    float s = v_instanceTransform.w;
    vec2 p = vec2(scale * v_localPosition.x, scale * v_localPosition.y) + center;
    p = vec2((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
    gl_Position = projectionMatrix * vec4(p, 0.0f, 1.0f);
}
