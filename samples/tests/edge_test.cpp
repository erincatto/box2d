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

class EdgeTest : public Test
{
public:

	EdgeTest()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2Vec2 v1(-10.0f, 0.0f), v2(-7.0f, -2.0f), v3(-4.0f, 0.0f);
			b2Vec2 v4(0.0f, 0.0f), v5(4.0f, 0.0f), v6(7.0f, 2.0f), v7(10.0f, 0.0f);

			b2EdgeShape shape;

			shape.Set(v1, v2);
			shape.m_hasVertex3 = true;
			shape.m_vertex3 = v3;
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(v2, v3);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v1;
			shape.m_vertex3 = v4;
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(v3, v4);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v2;
			shape.m_vertex3 = v5;
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(v4, v5);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v3;
			shape.m_vertex3 = v6;
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(v5, v6);
			shape.m_hasVertex0 = true;
			shape.m_hasVertex3 = true;
			shape.m_vertex0 = v4;
			shape.m_vertex3 = v7;
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(v6, v7);
			shape.m_hasVertex0 = true;
			shape.m_vertex0 = v5;
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-0.5f, 0.6f);
			bd.allowSleep = false;
			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape shape;
			shape.m_radius = 0.5f;

			body->CreateFixture(&shape, 1.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(1.0f, 0.6f);
			bd.allowSleep = false;
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);

			body->CreateFixture(&shape, 1.0f);
		}
	}

	static Test* Create()
	{
		return new EdgeTest;
	}
};

static int testIndex = RegisterTest("Geometry", "Edge Test", EdgeTest::Create);
