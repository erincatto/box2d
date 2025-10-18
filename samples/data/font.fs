// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#version 330 core

in vec4 color;
in vec2 uv;

uniform sampler2D FontAtlas;

out vec4 fragColor;

void main()
{
	fragColor = vec4(color.rgb, color.a * texture(FontAtlas, uv).r);
}
