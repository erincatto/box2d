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

extern B2_API bool g_blockSolve;

class BoxStack : public Test
{
public:

	enum
	{
		e_columnCount = 1,
		e_rowCount = 15
		//e_columnCount = 1,
		//e_rowCount = 1
	};

	BoxStack()
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

		float xs[5] = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			for (int i = 0; i < e_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;

				int32 n = j * e_rowCount + i;
				b2Assert(n < e_rowCount * e_columnCount);
				m_indices[n] = n;
				bd.userData.pointer = n;

				float x = 0.0f;
				//float x = RandomFloat(-0.02f, 0.02f);
				//float x = i % 2 == 0 ? -0.01f : 0.01f;
				bd.position.Set(xs[j] + x, 0.55f + 1.1f * i);
				b2Body* body = m_world->CreateBody(&bd);

				m_bodies[n] = body;

				body->CreateFixture(&fd);
			}
		}

		m_bullet = NULL;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_COMMA:
			if (m_bullet != NULL)
			{
				m_world->DestroyBody(m_bullet);
				m_bullet = NULL;
			}

			{
				b2CircleShape shape;
				shape.m_radius = 0.25f;

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 20.0f;
				fd.restitution = 0.05f;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.bullet = true;
				bd.position.Set(-31.0f, 5.0f);

				m_bullet = m_world->CreateBody(&bd);
				m_bullet->CreateFixture(&fd);

				m_bullet->SetLinearVelocity(b2Vec2(400.0f, 0.0f));
			}
			break;
                
        case GLFW_KEY_B:
            g_blockSolve = !g_blockSolve;
            break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press: (,) to launch a bullet.");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Blocksolve = %d", g_blockSolve);
		//if (m_stepCount == 300)
		//{
		//	if (m_bullet != NULL)
		//	{
		//		m_world->DestroyBody(m_bullet);
		//		m_bullet = NULL;
		//	}

		//	{
		//		b2CircleShape shape;
		//		shape.m_radius = 0.25f;

		//		b2FixtureDef fd;
		//		fd.shape = &shape;
		//		fd.density = 20.0f;
		//		fd.restitution = 0.05f;

		//		b2BodyDef bd;
		//		bd.type = b2_dynamicBody;
		//		bd.bullet = true;
		//		bd.position.Set(-31.0f, 5.0f);

		//		m_bullet = m_world->CreateBody(&bd);
		//		m_bullet->CreateFixture(&fd);

		//		m_bullet->SetLinearVelocity(b2Vec2(400.0f, 0.0f));
		//	}
		//}
	}

	static Test* Create()
	{
		return new BoxStack;
	}

	b2Body* m_bullet;
	b2Body* m_bodies[e_rowCount * e_columnCount];
	int32 m_indices[e_rowCount * e_columnCount];
};

static int testIndex = RegisterTest("Stacking", "Boxes", BoxStack::Create);
