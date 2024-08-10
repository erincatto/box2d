// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT
#version 330

layout(location = 0) in vec2 v_position;

void main(void)
{
    gl_Position = vec4(v_position, 0.0f, 1.0f);
}
