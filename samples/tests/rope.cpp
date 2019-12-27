// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "settings.h"
#include "test.h"
#include "box2d/b2_rope.h"

///
class Rope : public Test
{
public:
	Rope()
	{
		const int32 N = 20;
		const float L = 0.5f;
		b2Vec2 vertices[N];
		float masses[N];

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

	void Step(Settings& settings) override
	{
		float dt = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : 0.0f;

		if (settings.m_pause == 1 && settings.m_singleStep == 0)
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
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "angle = %g, iterations = %d", m_angle * 180.0f / b2_pi, m_iterations);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "bend: hertz = %g, damping = %g, stiffness = %g", m_tuning.bendHertz, m_tuning.bendDamping, m_tuning.bendStiffness);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Rope;
	}

	b2Rope m_rope;
	b2RopeTuning m_tuning;
	float m_angle;
	int32 m_iterations;
	b2Vec2 m_position;
};

static int testIndex = RegisterTest("Rope", "Bending", Rope::Create);
