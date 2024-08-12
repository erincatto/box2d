// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

in vec2 f_position;
in vec2 f_points[8];
flat in int f_count;
in float f_radius;
in vec4 f_color;
in float f_thickness;

out vec4 fragColor;

// https://en.wikipedia.org/wiki/Alpha_compositing
vec4 blend_colors(vec4 front, vec4 back)
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

float cross2d(in vec2 v1, in vec2 v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// Signed distance function for convex polygon
float sdConvexPolygon(in vec2 p, in vec2[8] v, in int count)
{
    // Initial squared distance
    float d = dot(p - v[0], p - v[0]);

    // Consider query point inside to start
    float side = -1.0;
    int j = count - 1;
    for (int i = 0; i < count; ++i)
    {
        // Distance to a polygon edge
        vec2 e = v[i] - v[j];
        vec2 w = p - v[j];
        float we = dot(w, e);
        vec2 b = w - e * clamp(we / dot(e, e), 0.0, 1.0);
        float bb = dot(b, b);

        // Track smallest distance
        if (bb < d)
        {
            d = bb;
        }

        // If the query point is outside any edge then it is outside the entire polygon.
        // This depends on the CCW winding order of points.
        float s = cross2d(w, e);
        if (s >= 0.0)
        {
            side = 1.0;
        }

        j = i;
    }

    return side * sqrt(d);
}

void main()
{
    vec4 borderColor = f_color;
    vec4 fillColor = 0.6f * borderColor;

    float dw = sdConvexPolygon(f_position, f_points, f_count);
    float d = abs(dw - f_radius);

    // roll the fill alpha down at the border
    vec4 back = vec4(fillColor.rgb, fillColor.a * smoothstep(f_radius + f_thickness, f_radius, dw));

    // roll the border alpha down from 1 to 0 across the border thickness
    vec4 front = vec4(borderColor.rgb, smoothstep(f_thickness, 0.0f, d));

    fragColor = blend_colors(front, back);

    // todo debugging
    // float resy = 3.0f / f_thickness;

    // if (resy < 539.9f)
    // {
    //     fragColor = vec4(1, 0, 0, 1);
    // }
    // else if (resy > 540.1f)
    // {
    //     fragColor = vec4(0, 1, 0, 1);
    // }
    // else
    // {
    //     fragColor = vec4(0, 0, 1, 1);
    // }
}
