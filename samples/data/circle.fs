// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

in vec2 f_position;
in vec4 f_color;
in float f_thickness;

out vec4 fragColor;

void main()
{
    // radius in unit quad
    float radius = 1.0;

    // distance to circle
    vec2 w = f_position;
    float dw = length(w);
    float d = abs(dw - radius);

    fragColor = vec4(f_color.rgb, smoothstep(f_thickness, 0.0, d));
}
