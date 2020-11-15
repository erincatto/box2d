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

// Note: even with a restitution of 1.0, there is some energy change
// due to position correction.
class Restitution : public Test
{
public:

	enum
	{
		e_count = 11,
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	Restitution()
	{
		const float threshold = 10.0f;
		b2BodyDef bd;
		b2Body* ground = m_world->CreateBody(&bd);

		b2EdgeShape shape;
		shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			
		b2FixtureDef fd;
		fd.shape = &shape;
		fd.restitutionThreshold = threshold;
		ground->CreateFixture(&fd);

		m_shapeType = e_circleShape;
		m_threshold = 1.0f;

		for (int32 i = 0; i < e_count; ++i)
		{
			m_bodies[i] = nullptr;
			m_restitutions[i] = 0.1f * i;
			m_xcoords[i] = -10.0f + 3.0f * i;
		}

		CreateShapes();
	}

	void CreateShapes()
	{
		for (int32 i = 0; i < e_count; ++i)
		{
			if (m_bodies[i] != nullptr)
			{
				m_world->DestroyBody(m_bodies[i]);
				m_bodies[i] = nullptr;
			}
		}

		b2CircleShape circle;
		circle.m_radius = 1.0f;

		b2PolygonShape box;
		box.SetAsBox(1.0f, 1.0f);

		b2FixtureDef fd;
		fd.density = 1.0f;

		if (m_shapeType == e_circleShape)
		{
			fd.shape = &circle;
		}
		else
		{
			fd.shape = &box;
		}

		for (int32 i = 0; i < e_count; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(m_xcoords[i], 20.0f);

			b2Body* body = m_world->CreateBody(&bd);

			fd.restitution = m_restitutions[i];
			fd.restitutionThreshold = m_threshold;
			body->CreateFixture(&fd);

			m_bodies[i] = body;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Restitution", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int(m_shapeType);
		changed = changed || ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_shapeType = ShapeType(shapeType);

		changed = changed || ImGui::SliderFloat("Threshold", &m_threshold, 0.0f, 20.0f);
		changed = changed || ImGui::Button("Respawn");

		if (changed)
		{
			CreateShapes();
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		for (int32 i = 0; i < e_count; ++i)
		{
			b2Vec2 p(m_xcoords[i], -1.0f);
			g_debugDraw.DrawString(p, "%g", m_restitutions[i]);
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Restitution;
	}

	b2Body* m_bodies[e_count];
	float m_restitutions[e_count];
	float m_xcoords[e_count];
	float m_threshold;
	ShapeType m_shapeType;
};

static int testIndex = RegisterTest("Forces", "Restitution", Restitution::Create);
