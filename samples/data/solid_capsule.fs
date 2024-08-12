// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

in vec2 f_position;
in vec4 f_color;
in float f_length;
in float f_thickness;

out vec4 color;

// Thanks to baz and kolyan3040 for help on this shader
// todo this can be optimized a bit, keeping some terms for clarity

// https://en.wikipedia.org/wiki/Alpha_compositing
vec4 blend_colors(vec4 front,vec4 back)
{
    vec3 cSrc = front.rgb;
    float alphaSrc = front.a;
    vec3 cDst = back.rgb;
    float alphaDst = back.a;
    
    vec3 cOut = cSrc * alphaSrc + cDst * alphaDst * (1.0 - alphaSrc);
    float alphaOut = alphaSrc + alphaDst * (1.0 - alphaSrc);
    
    // remove alpha from rgb
    cOut = cOut / alphaOut;
    
    return vec4(cOut, alphaOut);
}

void main()
{
    // radius in unit quad
    float radius = 0.5 * (2.0 - f_length);
    
    vec4 borderColor = f_color;
    vec4 fillColor = 0.6f * borderColor;
    
    vec2 v1 = vec2(-0.5 * f_length, 0);
    vec2 v2 = vec2(0.5 * f_length, 0);
    
    // distance to line segment
    vec2 e = v2 - v1;
    vec2 w = f_position - v1;
    float we = dot(w, e);
    vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
    float dw = length(b);
    
    // SDF union of capsule and line segment
    float d = min(dw, abs(dw - radius));
    
    // roll the fill alpha down at the border
    vec4 back = vec4(fillColor.rgb, fillColor.a * smoothstep(radius + f_thickness, radius, dw));

    // roll the border alpha down from 1 to 0 across the border thickness
    vec4 front = vec4(borderColor.rgb, smoothstep(f_thickness, 0.0f, d));
    
    color = blend_colors(front, back);
}
