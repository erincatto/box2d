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

class Platformer : public Test
{
public:

	enum State
	{
		e_unknown,
		e_above,
		e_below
	};

	Platformer()
	{
		// Ground
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		// Platform
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 10.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(3.0f, 0.5f);
			m_platform = body->CreateFixture(&shape, 0.0f);

			m_bottom = 10.0f - 0.5f;
			m_top = 10.0f + 0.5f;
		}

		// Actor
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 12.0f);
			b2Body* body = m_world->CreateBody(&bd);

			m_radius = 0.5f;
			b2CircleShape shape;
			shape.m_radius = m_radius;
			m_character = body->CreateFixture(&shape, 20.0f);

			body->SetLinearVelocity(b2Vec2(0.0f, -50.0f));

			m_state = e_unknown;
		}
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override
	{
		Test::PreSolve(contact, oldManifold);

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA != m_platform && fixtureA != m_character)
		{
			return;
		}

		if (fixtureB != m_platform && fixtureB != m_character)
		{
			return;
		}

#if 1
		b2Vec2 position = m_character->GetBody()->GetPosition();

		if (position.y < m_top + m_radius - 3.0f * b2_linearSlop)
		{
			contact->SetEnabled(false);
		}
#else
        b2Vec2 v = m_character->GetBody()->GetLinearVelocity();
        if (v.y > 0.0f)
		{
            contact->SetEnabled(false);
        }
#endif
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2Vec2 v = m_character->GetBody()->GetLinearVelocity();
        g_debugDraw.DrawString(5, m_textLine, "Character Linear Velocity: %f", v.y);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Platformer;
	}

	float m_radius, m_top, m_bottom;
	State m_state;
	b2Fixture* m_platform;
	b2Fixture* m_character;
};

static int testIndex = RegisterTest("Examples", "Platformer", Platformer::Create);
