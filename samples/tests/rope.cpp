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
#include "imgui/imgui.h"

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

		m_tuning1.bendHertz = 1.0f;
		m_tuning1.bendDamping = 0.0f;
		m_tuning1.bendStiffness = 0.5f;
		m_tuning1.bendingModel = b2_springAngleBendingModel;

		m_tuning2.bendHertz = 1.0f;
		m_tuning2.bendDamping = 0.0f;
		m_tuning2.bendStiffness = 0.5f;
		m_tuning2.bendingModel = b2_xpbdAngleBendingModel;

		b2RopeDef def;
		def.vertices = vertices;
		def.count = N;
		def.gravity.Set(0.0f, -10.0f);
		def.masses = masses;

		def.tuning = m_tuning1;
		m_rope1.Initialize(&def);

		def.tuning = m_tuning2;
		m_rope2.Initialize(&def);

		m_angle = 0.0f;
		m_rope1.SetAngle(m_angle);
		m_rope2.SetAngle(m_angle);
		m_iterations = 1;
		m_position1.Set(-2.0f, 4.0f);
		m_position2.Set(2.0f, 4.0f);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f * m_uiScale, 150.0f * m_uiScale));
		ImGui::SetNextWindowSize(ImVec2(200.0f * m_uiScale, 500.0f * m_uiScale));
		ImGui::Begin("Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::Separator();

		const ImGuiComboFlags comboFlags = 0;
		const char* items[] = { "Spring", "PBD", "XPBD" };

		ImGui::Text("Rope 1");
		static int modelA = m_tuning1.bendingModel;
		if (ImGui::BeginCombo("Model##1", items[modelA], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(items); ++i)
			{
				bool isSelected = (modelA == i);
				if (ImGui::Selectable(items[i], isSelected))
				{
					m_tuning1.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping", &m_tuning1.bendDamping, 0.0f, 2.0f, "%.1f");
		ImGui::SliderFloat("Hertz", &m_tuning1.bendHertz, 0.0f, 15.0f, "%.1f");
		ImGui::SliderFloat("Stiffness", &m_tuning1.bendStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::Separator();

		ImGui::Text("Rope 2");
		static int model2 = m_tuning2.bendingModel;
		if (ImGui::BeginCombo("Model##2", items[model2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(items); ++i)
			{
				bool isSelected = (model2 == i);
				if (ImGui::Selectable(items[i], isSelected))
				{
					m_tuning2.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping", &m_tuning2.bendDamping, 0.0f, 2.0f, "%.1f");
		ImGui::SliderFloat("Hertz", &m_tuning2.bendHertz, 0.0f, 15.0f, "%.1f");
		ImGui::SliderFloat("Stiffness", &m_tuning2.bendStiffness, 0.0f, 1.0f, "%.1f");
		ImGui::End();
	}

#if 0
	void Keyboard(int key) override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f * m_uiScale, 150.0f * m_uiScale));
		ImGui::SetNextWindowSize(ImVec2(200.0f * m_uiScale, 500.0f * m_uiScale));
		ImGui::Begin("Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::Separator();

		const ImGuiComboFlags comboFlags = 0;
		const char* items[] = { "Spring", "PBD", "XPBD" };

		ImGui::Text("Rope 1");
		static int model1 = m_tuning1.bendingModel;
		if (ImGui::BeginCombo("Model##1", items[model1], comboFlags))
		{
		case GLFW_KEY_I:
			m_iterations += 1;
			break;

		case GLFW_KEY_K:
			m_iterations = b2Max(m_iterations - 1, 1);
			break;

		case GLFW_KEY_U:
			m_tuning1.bendHertz = b2Min(m_tuning.bendHertz + 1.0f, 100.0f);
			m_tuning1.bendStiffness = b2Min(m_tuning.bendStiffness + 0.1f, 1.0f);
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

		ImGui::SliderInt("Iterations##1", &m_iterations1, 1, 100, "%d");
		ImGui::SliderFloat("Damping##1", &m_tuning1.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##1", &m_tuning1.bendHertz, 0.0f, 30.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##1", &m_tuning1.bendStiffness, 0.0f, 1.0f, "%.1f");
		ImGui::Checkbox("Isometric##1", &m_tuning1.isometric);
		ImGui::Checkbox("Fixed Mass##1", &m_tuning1.fixedEffectiveMass);

		ImGui::Separator();

		ImGui::Text("Rope 2");
		static int model2 = m_tuning2.bendingModel;
		if (ImGui::BeginCombo("Model##2", items[model2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(items); ++i)
			{
				bool isSelected = (model2 == i);
				if (ImGui::Selectable(items[i], isSelected))
				{
					model2 = i;
					m_tuning2.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}

		ImGui::SliderInt("Iterations##2", &m_iterations2, 1, 100, "%d");
		ImGui::SliderFloat("Damping##2", &m_tuning2.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##2", &m_tuning2.bendHertz, 0.0f, 30.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##2", &m_tuning2.bendStiffness, 0.0f, 1.0f, "%.1f");
		ImGui::Checkbox("Isometric##2", &m_tuning2.isometric);
		ImGui::Checkbox("Fixed Mass##2", &m_tuning2.fixedEffectiveMass);

		ImGui::Separator();

		if (ImGui::Button("Reset"))
		{
			m_position1.Set(-5.0f, 10.0f);
			m_position2.Set(5.0f, 10.0f);
			m_rope1.Reset(m_position1);
			m_rope2.Reset(m_position2);
		}

		ImGui::End();
	}
#endif

	void Step(Settings& settings) override
	{
		float dt = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : 0.0f;

		if (settings.m_pause == 1 && settings.m_singleStep == 0)
		{
			dt = 0.0f;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_COMMA) == GLFW_PRESS)
		{
			m_position1.x -= 4.0f * dt;
			m_position2.x -= 4.0f * dt;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_PERIOD) == GLFW_PRESS)
		{
			m_position1.x += 4.0f * dt;
			m_position2.x += 4.0f * dt;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_Q) == GLFW_PRESS)
		{
			m_angle = b2Max(-b2_pi, m_angle - 0.2f * b2_pi / 180.0f);
			m_rope1.SetAngle(m_angle);
			m_rope2.SetAngle(m_angle);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_E) == GLFW_PRESS)
		{
			m_angle = b2Min(b2_pi, m_angle + 0.2f * b2_pi / 180.0f);
			m_rope1.SetAngle(m_angle);
			m_rope2.SetAngle(m_angle);
		}

		m_rope1.SetTuning(m_tuning1);
		m_rope2.SetTuning(m_tuning2);
		m_rope1.Step(dt, m_iterations, m_position1);
		m_rope2.Step(dt, m_iterations, m_position2);

		Test::Step(settings);

		m_rope1.Draw(&g_debugDraw);
		m_rope2.Draw(&g_debugDraw);

		g_debugDraw.DrawString(5, m_textLine, "Press (q,e) to adjust target angle, (i,k) to adjust iterations");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "angle = %g, iterations = %d", m_angle * 180.0f / b2_pi, m_iterations);
		m_textLine += m_textIncrement;
		//g_debugDraw.DrawString(5, m_textLine, "bend: hertz = %g, damping = %g, stiffness = %g", m_tuning.bendHertz, m_tuning.bendDamping, m_tuning.bendStiffness);
		//m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Rope;
	}

	b2Rope m_rope1;
	b2Rope m_rope2;
	b2RopeTuning m_tuning1;
	b2RopeTuning m_tuning2;
	float m_angle;
	int32 m_iterations;
	b2Vec2 m_position1;
	b2Vec2 m_position2;
};

static int testIndex = RegisterTest("Rope", "Bending", Rope::Create);
