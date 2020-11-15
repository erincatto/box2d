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

extern B2_API bool g_blockSolve;

class VerticalStacks : public Test
{
public:

	enum
	{
		e_maxColumns = 5,
		e_maxRows = 15,
		e_maxBullets = 20
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	VerticalStacks()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(b2Vec2(20.0f, 0.0f), b2Vec2(20.0f, 20.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		for (int32 i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			m_bodies[i] = nullptr;
		}

		for (int32 i = 0; i < e_maxBullets; ++i)
		{
			m_bullets[i] = nullptr;
		}

		m_shapeType = e_boxShape;
		m_rowCount = 2;
		m_columnCount = 1;
		m_bulletCount = 1;
		m_bulletType = e_circleShape;

		CreateStacks();
	}

	void CreateStacks()
	{
		for (int32 i = 0; i < e_maxRows * e_maxColumns; ++i)
		{
			if (m_bodies[i] != nullptr)
			{
				m_world->DestroyBody(m_bodies[i]);
				m_bodies[i] = nullptr;
			}
		}

		float xs[5] = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

		b2CircleShape circle;
		circle.m_radius = 0.5f;

		b2PolygonShape box;
		box.SetAsBox(0.5f, 0.5f);

		b2FixtureDef fd;
		fd.density = 1.0f;
		fd.friction = 0.6f;

		float offset;

		if (m_shapeType == e_circleShape)
		{
			fd.shape = &circle;
			offset = 0.0f;
		}
		else
		{
			fd.shape = &box;
			offset = 0.01f;
		}

		for (int32 j = 0; j < m_columnCount; ++j)
		{
			for (int i = 0; i < m_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;

				int32 n = j * m_rowCount + i;

				float x = (i % 2 == 0 ? -offset : offset);
				bd.position.Set(xs[j] + x, 0.55f + 1.1f * i);
				b2Body* body = m_world->CreateBody(&bd);

				m_bodies[n] = body;

				body->CreateFixture(&fd);
			}
		}
	}

	void DestroyBullets()
	{
		for (int32 i = 0; i < e_maxBullets; ++i)
		{
			b2Body* bullet = m_bullets[i];

			if (bullet != nullptr)
			{
				m_world->DestroyBody(bullet);
				m_bullets[i] = nullptr;
			}
		}
	}

	void FireBullets()
	{
		b2CircleShape circle;
		circle.m_radius = 0.25f;

		b2PolygonShape box;
		box.SetAsBox(0.25f, 0.25f);

		b2FixtureDef fd;
		fd.density = 2.0f;
		fd.friction = 0.6f;

		if (m_bulletType == e_circleShape)
		{
			fd.shape = &circle;
		}
		else
		{
			fd.shape = &box;
		}

		for (int32 i = 0; i < m_bulletCount; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-25.0f - i, 5.0f);

			b2Body* bullet = m_world->CreateBody(&bd);
			bullet->CreateFixture(&fd);

			bullet->SetLinearVelocity(b2Vec2(400.0f, 0.0f));

			b2Assert(m_bullets[i] == nullptr);
			m_bullets[i] = bullet;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(240.0f, 230.0f));
		ImGui::Begin("Stacks", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int(m_shapeType);
		changed = changed || ImGui::Combo("Shape", &shapeType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_shapeType = ShapeType(shapeType);


		changed = changed || ImGui::SliderInt("Rows", &m_rowCount, 1, e_maxRows);
		changed = changed || ImGui::SliderInt("Columns", &m_columnCount, 1, e_maxColumns);

		ImGui::SliderInt("Bullets", &m_bulletCount, 1, e_maxBullets);

		int bulletType = int(m_bulletType);
		ImGui::Combo("Bullet Shape", &bulletType, shapeTypes, IM_ARRAYSIZE(shapeTypes));
		m_bulletType = ShapeType(bulletType);

		if (ImGui::Button("Fire Bullets"))
		{
			DestroyBullets();
			FireBullets();
		}

		ImGui::Checkbox("Block Solve", &g_blockSolve);

		changed = changed || ImGui::Button("Reset Stack");

		if (changed)
		{
			DestroyBullets();
			CreateStacks();
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new VerticalStacks;
	}

	b2Body* m_bullets[e_maxBullets];
	b2Body* m_bodies[e_maxRows * e_maxColumns];
	int32 m_columnCount;
	int32 m_rowCount;
	int32 m_bulletCount;
	ShapeType m_shapeType;
	ShapeType m_bulletType;
};

static int testIndex = RegisterTest("Stacking", "Vertical Stacks", VerticalStacks::Create);
