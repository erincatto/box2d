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

#include "test.h"
#include "imgui/imgui.h"

class EdgeTest : public Test
{
public:

	EdgeTest()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2Vec2 v1(14.0f, -4.0f);
			b2Vec2 v2(14.0f, 0.0f);
			b2Vec2 v3(10.0f, 0.0f);
			b2Vec2 v4(7.0f, 2.0f);
			b2Vec2 v5(4.0f, 0.0f);
			b2Vec2 v6(0.0f, 0.0f);
			b2Vec2 v7(-4.0f, 0.0f);
			b2Vec2 v8(-7.0f, -2.0f);
			b2Vec2 v9(-10.0f, 0.0f);
			b2Vec2 v10(-10.0f, -4.0f);

			b2EdgeShape shape;

			shape.SetOneSided(v10, v1, v2, v3);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v1, v2, v3, v4);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v2, v3, v4, v5);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v3, v4, v5, v6);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v4, v5, v6, v7);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v5, v6, v7, v8);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v6, v7, v8, v9);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v7, v8, v9, v10);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v8, v9, v10, v1);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetOneSided(v9, v10, v1, v2);
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			float y = 8.0f;
			b2Vec2 v1(14.0f, -4.0f + y);
			b2Vec2 v2(14.0f, 0.0f + y);
			b2Vec2 v3(10.0f, 0.0f + y);
			b2Vec2 v4(7.0f, 2.0f + y);
			b2Vec2 v5(4.0f, 0.0f + y);
			b2Vec2 v6(0.0f, 0.0f + y);
			b2Vec2 v7(-4.0f, 0.0f + y);
			b2Vec2 v8(-7.0f, -2.0f + y);
			b2Vec2 v9(-10.0f, 0.0f + y);
			b2Vec2 v10(-10.0f, -4.0f + y);

			b2EdgeShape shape;

			shape.SetTwoSided(v1, v2);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v2, v3);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v3, v4);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v4, v5);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v5, v6);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v6, v7);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v7, v8);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v8, v9);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v9, v10);
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(v10, v1);
			ground->CreateFixture(&shape, 0.0f);
		}

		m_body1 = nullptr;
		m_body2 = nullptr;
		CreateBoxes();
		m_boxes = true;
	}

	void CreateBoxes()
	{
		if (m_body1)
		{
			m_world->DestroyBody(m_body1);
			m_body1 = nullptr;
		}

		if (m_body2)
		{
			m_world->DestroyBody(m_body2);
			m_body2 = nullptr;
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(12.0f, 2.6f);
			bd.allowSleep = false;
			m_body1 = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 1.0f);

			m_body1->CreateFixture(&shape, 1.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(12.0f, 2.6f + 8.0f);
			bd.allowSleep = false;
			m_body2 = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 1.0f);

			m_body2->CreateFixture(&shape, 1.0f);
		}
	}

	void CreateCircles()
	{
		if (m_body1)
		{
			m_world->DestroyBody(m_body1);
			m_body1 = nullptr;
		}

		if (m_body2)
		{
			m_world->DestroyBody(m_body2);
			m_body2 = nullptr;
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-0.5f, 0.6f);
			bd.allowSleep = false;
			m_body1 = m_world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.5f;

			m_body1->CreateFixture(&shape, 1.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-0.5f, 0.6f + 8.0f);
			bd.allowSleep = false;
			m_body2 = m_world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.5f;

			m_body2->CreateFixture(&shape, 1.0f);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Custom Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::RadioButton("Boxes", m_boxes == true))
		{
			CreateBoxes();
			m_boxes = true;
		}

		if (ImGui::RadioButton("Circles", m_boxes == false))
		{
			CreateCircles();
			m_boxes = false;
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		if (glfwGetKey(g_mainWindow, GLFW_KEY_A) == GLFW_PRESS)
		{
			m_body1->ApplyForceToCenter(b2Vec2(-10.0f, 0.0f), true);
			m_body2->ApplyForceToCenter(b2Vec2(-10.0f, 0.0f), true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_D) == GLFW_PRESS)
		{
			m_body1->ApplyForceToCenter(b2Vec2(10.0f, 0.0f), true);
			m_body2->ApplyForceToCenter(b2Vec2(10.0f, 0.0f), true);
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new EdgeTest;
	}

	b2Body* m_body1;
	b2Body* m_body2;
	bool m_boxes;
};

static int testIndex = RegisterTest("Geometry", "Edge Test", EdgeTest::Create);
