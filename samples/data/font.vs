// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#version 330 core

layout (location = 0) in vec2 aPosition;
layout (location = 1) in vec2 aUV;
layout (location = 2) in vec4 aColor;

out vec4 color;
out vec2 uv;

uniform mat4 ProjectionMatrix;

void main()
{
	gl_Position = ProjectionMatrix * vec4(aPosition, 0.0, 1.0);

	color = aColor;
	uv = aUV;
}
