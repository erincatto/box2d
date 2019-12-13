/*
* Copyright (c) 2011 Erin Catto http://box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef ROPE_H
#define ROPE_H

#include "Box2D/Rope/b2Rope.h"

///
class Rope : public Test
{
public:
	Rope()
	{
		const int32 N = 20;
		const float L = 0.5f;
		b2Vec2 vertices[N];
		float32 masses[N];

		for (int32 i = 0; i < N; ++i)
		{
			vertices[i].Set(0.0f, L * (N - i));
			masses[i] = 1.0f;
		}
		masses[0] = 0.0f;
		masses[1] = 0.0f;

		m_tuning.bendHertz = 1.0f;
		m_tuning.bendDamping = 0.0f;
		m_tuning.bendStiffness = 0.5f;
		m_tuning.bendingModel = b2_xpbdAngleBendingModel;

		b2RopeDef def;
		def.vertices = vertices;
		def.count = N;
		def.gravity.Set(0.0f, -10.0f);
		def.masses = masses;
		def.tuning = m_tuning;

		m_rope.Initialize(&def);

		m_angle = 0.0f;
		m_rope.SetAngle(m_angle);
		m_iterations = 1;
		m_position.Set(0.0f, 0.0f);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_I:
			m_iterations += 1;
			break;

		case GLFW_KEY_K:
			m_iterations = b2Max(m_iterations - 1, 1);
			break;

		case GLFW_KEY_U:
			m_tuning.bendHertz = b2Min(m_tuning.bendHertz + 1.0f, 100.0f);
			m_tuning.bendStiffness = b2Min(m_tuning.bendStiffness + 0.1f, 1.0f);
			break;

		case GLFW_KEY_J:
			m_tuning.bendHertz = b2Max(m_tuning.bendHertz - 1.0f, 0.0f);
			m_tuning.bendStiffness = b2Max(m_tuning.bendStiffness - 0.1f, 0.0f);
			break;

		case GLFW_KEY_Y:
			m_tuning.bendDamping = b2Min(m_tuning.bendDamping + 0.1f, 10.0f);
			break;

		case GLFW_KEY_H:
			m_tuning.bendDamping = b2Max(m_tuning.bendDamping - 0.1f, 0.0f);
			break;
		}
	}

	void Step(Settings* settings)
	{
		float32 dt = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

		if (settings->pause == 1 && settings->singleStep == 0)
		{
			dt = 0.0f;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_COMMA) == GLFW_PRESS)
		{
			m_position.x -= 4.0f * dt;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_PERIOD) == GLFW_PRESS)
		{
			m_position.x += 4.0f * dt;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_Q) == GLFW_PRESS)
		{
			m_angle = b2Max(-b2_pi, m_angle - 0.2f * b2_pi / 180.0f);
			m_rope.SetAngle(m_angle);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_E) == GLFW_PRESS)
		{
			m_angle = b2Min(b2_pi, m_angle + 0.2f * b2_pi / 180.0f);
			m_rope.SetAngle(m_angle);
		}

		m_rope.SetTuning(m_tuning);
		m_rope.Step(dt, m_iterations, m_position);

		Test::Step(settings);

		m_rope.Draw(&g_debugDraw);

		g_debugDraw.DrawString(5, m_textLine, "Press (q,e) to adjust target angle, (i,k) to adjust iterations");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "angle = %g, iterations = %d", m_angle * 180.0f / b2_pi, m_iterations);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "bend: hertz = %g, damping = %g, stiffness = %g", m_tuning.bendHertz, m_tuning.bendDamping, m_tuning.bendStiffness);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Rope;
	}

	b2Rope m_rope;
	b2RopeTuning m_tuning;
	float32 m_angle;
	int32 m_iterations;
	b2Vec2 m_position;
};

#endif
