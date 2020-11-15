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

// This test shows how TOI is used to prevent ghost collisions that
// can happen with predicted collision.
class GhostTest : public Test
{
public:

	GhostTest()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 0.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2EdgeShape edge;

			edge.SetTwoSided(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f));
			body->CreateFixture(&edge, 0.0f);
		}

		m_x = 0.0f;
		m_body = nullptr;

		LaunchBox();
	}

	void LaunchBox()
	{
		if (m_body)
		{
			m_world->DestroyBody(m_body);
			m_body = nullptr;
		}

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(m_x, 10.0f);
		bd.gravityScale = 0.0f;

		b2PolygonShape box;
		box.SetAsBox(0.25f, 0.25f);

		m_body = m_world->CreateBody(&bd);
		m_body->CreateFixture(&box, 1.0f);

		m_body->SetTransform(b2Vec2(m_x, 10.0f), 0.0f);
		m_body->SetLinearVelocity(b2Vec2(50.0f, -50.0f));
		m_body->SetAngularVelocity(0.0f);
	}

	void LaunchCircle()
	{
		if (m_body)
		{
			m_world->DestroyBody(m_body);
			m_body = nullptr;
		}

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(m_x, 10.0f);
		bd.gravityScale = 0.0f;

		b2CircleShape circle;
		circle.m_radius = 0.25f;

		m_body = m_world->CreateBody(&bd);
		m_body->CreateFixture(&circle, 1.0f);

		m_body->SetTransform(b2Vec2(m_x, 10.0f), 0.0f);
		m_body->SetLinearVelocity(b2Vec2(50.0f, -50.0f));
		m_body->SetAngularVelocity(0.0f);
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(240.0f, 100.0f));
		ImGui::Begin("Ghost Test", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("x-coord", &m_x, -1.0f, 1.0f, "%.1f");

		if (ImGui::Button("Launch Box"))
		{
			LaunchBox();
		}

		if (ImGui::Button("Launch Circle"))
		{
			LaunchCircle();
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new GhostTest;
	}

	b2Body* m_body;
	float m_x;
};

static int testIndex = RegisterTest("Continuous", "Ghost Test", GhostTest::Create);
