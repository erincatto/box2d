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

		m_tuning1.bendHertz = 30.0f;
		m_tuning1.bendDamping = 4.0f;
		m_tuning1.bendStiffness = 1.0f;
		m_tuning1.bendingModel = b2_pbdTriangleBendingModel;
		m_tuning1.isometric = true;

		m_tuning1.stretchHertz = 30.0f;
		m_tuning1.stretchDamping = 4.0f;
		m_tuning1.stretchStiffness = 1.0f;
		m_tuning1.stretchingModel = b2_pbdStretchingModel;

		m_tuning2.bendHertz = 30.0f;
		m_tuning2.bendDamping = 0.7f;
		m_tuning2.bendStiffness = 1.0f;
		m_tuning2.bendingModel = b2_pbdHeightBendingModel;
		m_tuning2.isometric = true;

		m_tuning2.stretchHertz = 30.0f;
		m_tuning2.stretchDamping = 1.0f;
		m_tuning2.stretchStiffness = 1.0f;
		m_tuning2.stretchingModel = b2_pbdStretchingModel;

		m_position1.Set(-5.0f, 15.0f);
		m_position2.Set(5.0f, 15.0f);

		b2RopeDef def;
		def.vertices = vertices;
		def.count = N;
		def.gravity.Set(0.0f, -10.0f);
		def.masses = masses;

		def.position = m_position1;
		def.tuning = m_tuning1;
		m_rope1.Create(def);

		def.position = m_position2;
		def.tuning = m_tuning2;
		m_rope2.Create(def);

		m_iterations1 = 8;
		m_iterations2 = 8;

		m_speed = 10.0f;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 700.0f));
		ImGui::Begin("Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::Separator();

        ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);
        
		const ImGuiComboFlags comboFlags = 0;
		const char* bendModels[] = { "Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height", "PBD Triangle" };
		const char* stretchModels[] = { "PBD", "XPBD" };

		ImGui::Text("Rope 1");
		static int bendModel1 = m_tuning1.bendingModel;
		if (ImGui::BeginCombo("Bend Model##1", bendModels[bendModel1], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(bendModels); ++i)
			{
				bool isSelected = (bendModel1 == i);
				if (ImGui::Selectable(bendModels[i], isSelected))
				{
					bendModel1 = i;
					m_tuning1.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##B1", &m_tuning1.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##B1", &m_tuning1.bendHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##B1", &m_tuning1.bendStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::Checkbox("Isometric##1", &m_tuning1.isometric);
		ImGui::Checkbox("Fixed Mass##1", &m_tuning1.fixedEffectiveMass);
		ImGui::Checkbox("Warm Start##1", &m_tuning1.warmStart);

		static int stretchModel1 = m_tuning1.stretchingModel;
		if (ImGui::BeginCombo("Stretch Model##1", stretchModels[stretchModel1], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(stretchModels); ++i)
			{
				bool isSelected = (stretchModel1 == i);
				if (ImGui::Selectable(stretchModels[i], isSelected))
				{
					stretchModel1 = i;
					m_tuning1.stretchingModel = b2StretchingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##S1", &m_tuning1.stretchDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##S1", &m_tuning1.stretchHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##S1", &m_tuning1.stretchStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::SliderInt("Iterations##1", &m_iterations1, 1, 100, "%d");

		ImGui::Separator();

		ImGui::Text("Rope 2");
		static int bendModel2 = m_tuning2.bendingModel;
		if (ImGui::BeginCombo("Bend Model##2", bendModels[bendModel2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(bendModels); ++i)
			{
				bool isSelected = (bendModel2 == i);
				if (ImGui::Selectable(bendModels[i], isSelected))
				{
					bendModel2 = i;
					m_tuning2.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##B2", &m_tuning2.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##B2", &m_tuning2.bendHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##B2", &m_tuning2.bendStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::Checkbox("Isometric##2", &m_tuning2.isometric);
		ImGui::Checkbox("Fixed Mass##2", &m_tuning2.fixedEffectiveMass);
		ImGui::Checkbox("Warm Start##2", &m_tuning2.warmStart);

		static int stretchModel2 = m_tuning2.stretchingModel;
		if (ImGui::BeginCombo("Stretch Model##2", stretchModels[stretchModel2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(stretchModels); ++i)
			{
				bool isSelected = (stretchModel2 == i);
				if (ImGui::Selectable(stretchModels[i], isSelected))
				{
					stretchModel2 = i;
					m_tuning2.stretchingModel = b2StretchingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##S2", &m_tuning2.stretchDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##S2", &m_tuning2.stretchHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##S2", &m_tuning2.stretchStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::SliderInt("Iterations##2", &m_iterations2, 1, 100, "%d");

		ImGui::Separator();

		ImGui::SliderFloat("Speed", &m_speed, 10.0f, 100.0f, "%.0f");

		if (ImGui::Button("Reset"))
		{
			m_position1.Set(-5.0f, 15.0f);
			m_position2.Set(5.0f, 15.0f);
			m_rope1.Reset(m_position1);
			m_rope2.Reset(m_position2);
		}

        ImGui::PopItemWidth();

		ImGui::End();
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
			m_position1.x -= m_speed * dt;
			m_position2.x -= m_speed * dt;
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_PERIOD) == GLFW_PRESS)
		{
			m_position1.x += m_speed * dt;
			m_position2.x += m_speed * dt;
		}

		m_rope1.SetTuning(m_tuning1);
		m_rope2.SetTuning(m_tuning2);
		m_rope1.Step(dt, m_iterations1, m_position1);
		m_rope2.Step(dt, m_iterations2, m_position2);

		Test::Step(settings);

		m_rope1.Draw(&g_debugDraw);
		m_rope2.Draw(&g_debugDraw);

		g_debugDraw.DrawString(5, m_textLine, "Press comma and period to move left and right");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Rope;
	}

	b2Rope m_rope1;
	b2Rope m_rope2;
	b2RopeTuning m_tuning1;
	b2RopeTuning m_tuning2;
	int32 m_iterations1;
	int32 m_iterations2;
	b2Vec2 m_position1;
	b2Vec2 m_position2;
	float m_speed;
};

static int testIndex = RegisterTest("Rope", "Bending", Rope::Create);
