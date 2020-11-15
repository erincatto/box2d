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

class Pyramid : public Test
{
public:
	enum
	{
		e_maxCount = 20
	};

	Pyramid()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		int32 n = e_maxCount * (e_maxCount + 1) / 2;
		for (int32 i = 0; i < n; ++i)
		{
			m_bodies[i] = nullptr;
		}

		m_count = 5;
		CreatePyramid();
	}

	void CreatePyramid()
	{
		int32 n = e_maxCount * (e_maxCount + 1) / 2;
		for (int32 i = 0; i < n; ++i)
		{
			if (m_bodies[i] != nullptr)
			{
				m_world->DestroyBody(m_bodies[i]);
				m_bodies[i] = nullptr;
			}
		}

		float a = 0.5f;
		b2PolygonShape shape;
		shape.SetAsBox(a, a);

		b2Vec2 x(-7.0f, 0.75f);
		b2Vec2 y;
		b2Vec2 deltaX(0.5625f, 1.25f);
		b2Vec2 deltaY(1.125f, 0.0f);

		int32 k = 0;

		for (int32 i = 0; i < m_count; ++i)
		{
			y = x;

			for (int32 j = i; j < m_count; ++j)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position = y;
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 5.0f);

				m_bodies[k] = body;
				++k;

				y += deltaY;
			}

			x += deltaX;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Pyramid", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		bool changed = ImGui::SliderInt("Base Count", &m_count, 1, e_maxCount);

		if (changed)
		{
			CreatePyramid();
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new Pyramid;
	}

	b2Body* m_bodies[e_maxCount * (e_maxCount + 1) / 2];
	int32 m_count;
};

static int testIndex = RegisterTest("Stacking", "Pyramid", Pyramid::Create);
