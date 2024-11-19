// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#version 330

out vec4 FragColor;

uniform float time;
uniform vec2 resolution;
uniform vec3 baseColor;

// A simple pseudo-random function
float random(vec2 st)
{
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453123);
}

void main()
{
    vec2 uv = gl_FragCoord.xy / resolution.xy;
    
    // Create some noise
    float noise = random(uv + time * 0.1);
    
    // Adjust these values to control the intensity and color of the grain
    float grainIntensity = 0.01;
    
    // Mix the base color with the noise
    vec3 color = baseColor + vec3(noise * grainIntensity);
    
    FragColor = vec4(color, 1.0);
}

