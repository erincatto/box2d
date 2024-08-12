// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

uniform mat4 projectionMatrix;
uniform float pixelScale;

layout(location = 0) in vec2 v_localPosition;
layout(location = 1) in vec4 v_instanceTransform;
layout(location = 2) in float v_instanceRadius;
layout(location = 3) in vec4 v_instanceColor;

out vec2 f_position;
out vec4 f_color;
out float f_thickness;

void main()
{
    f_position = v_localPosition;
    f_color = v_instanceColor;
    float radius = v_instanceRadius;

    // resolution.y = pixelScale * radius
    f_thickness = 3.0f / (pixelScale * radius);
    
    float x = v_instanceTransform.x;
    float y = v_instanceTransform.y;
    float c = v_instanceTransform.z;
    float s = v_instanceTransform.w;
    vec2 p = vec2(radius * v_localPosition.x, radius * v_localPosition.y);
    p = vec2((c * p.x - s * p.y) + x, (s * p.x + c * p.y) + y);
    gl_Position = projectionMatrix * vec4(p, 0.0f, 1.0f);
}
