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

class Confined : public Test
{
public:

	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			// Floor
			shape.SetTwoSided(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			// Left wall
			shape.SetTwoSided(b2Vec2(-10.0f, 0.0f), b2Vec2(-10.0f, 20.0f));
			ground->CreateFixture(&shape, 0.0f);

			// Right wall
			shape.SetTwoSided(b2Vec2(10.0f, 0.0f), b2Vec2(10.0f, 20.0f));
			ground->CreateFixture(&shape, 0.0f);

			// Roof
			shape.SetTwoSided(b2Vec2(-10.0f, 20.0f), b2Vec2(10.0f, 20.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		float radius = 0.5f;
		b2CircleShape shape;
		shape.m_p.SetZero();
		shape.m_radius = radius;

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		fd.friction = 0.1f;

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			for (int i = 0; i < e_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&fd);
			}
		}

		m_world->SetGravity(b2Vec2(0.0f, 0.0f));
	}

	void CreateCircle()
	{
		float radius = 2.0f;
		b2CircleShape shape;
		shape.m_p.SetZero();
		shape.m_radius = radius;

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		fd.friction = 0.0f;

		b2Vec2 p(RandomFloat(), 3.0f + RandomFloat());
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position = p;
		//bd.allowSleep = false;
		b2Body* body = m_world->CreateBody(&bd);

		body->CreateFixture(&fd);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_C:
			CreateCircle();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		bool sleeping = true;
		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2_dynamicBody)
			{
				continue;
			}

			if (b->IsAwake())
			{
				sleeping = false;
			}
		}

		if (m_stepCount == 180)
		{
			m_stepCount += 0;
		}

		//if (sleeping)
		//{
		//	CreateCircle();
		//}

		Test::Step(settings);

		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2_dynamicBody)
			{
				continue;
			}

			b2Vec2 p = b->GetPosition();
			if (p.x <= -10.0f || 10.0f <= p.x || p.y <= 0.0f || 20.0f <= p.y)
			{
				p.x += 0.0f;
			}
		}

		g_debugDraw.DrawString(5, m_textLine, "Press 'c' to create a circle.");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Confined;
	}
};

static int testIndex = RegisterTest("Solver", "Confined", Confined::Create);
