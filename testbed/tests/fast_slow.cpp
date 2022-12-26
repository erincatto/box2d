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

class FastSlow : public Test
{
public:

	FastSlow()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-10.0f, 0.0f), b2Vec2(10.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.SetTwoSided(b2Vec2(-10.0f, 0.0f), b2Vec2(-10.0f, 5.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2CircleShape circle;
			circle.m_radius = 0.35f;

			b2FixtureDef fd;
			fd.density = 100.0f;
			fd.shape = &circle;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.gravityScale = 0.0f;		

			bd.position.Set(-9.0f, 2.5f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&circle, 100.0f);

			bd.position.Set(0.0f, 2.5f);
			bd.linearVelocity.Set(-400.0f, 0.0f);
			body = m_world->CreateBody(&bd);
			circle.m_radius = 0.25f;
			body->CreateFixture(&circle, 100.0f);
		}
	}

	static Test* Create()
	{
		return new FastSlow;
	}
};

static int testIndex = RegisterTest("Continuous", "Fast Slow", FastSlow::Create);
