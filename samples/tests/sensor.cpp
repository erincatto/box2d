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

// This shows how to use sensor shapes. Sensors don't have collision, but report overlap events.
class Sensors : public Test
{
public:

	enum
	{
		e_count = 7
	};

	Sensors()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			{
				b2EdgeShape shape;
				shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
				ground->CreateFixture(&shape, 0.0f);
			}

#if 0
			{
				b2FixtureDef sd;
				sd.SetAsBox(10.0f, 2.0f, b2Vec2(0.0f, 20.0f), 0.0f);
				sd.isSensor = true;
				m_sensor = ground->CreateFixture(&sd);
			}
#else
			{
				b2CircleShape shape;
				shape.m_radius = 5.0f;
				shape.m_p.Set(0.0f, 10.0f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.isSensor = true;
				m_sensor = ground->CreateFixture(&fd);
			}
#endif
		}

		{
			b2CircleShape shape;
			shape.m_radius = 1.0f;

			for (int32 i = 0; i < e_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-10.0f + 3.0f * i, 20.0f);
				bd.userData = m_touching + i;

				m_touching[i] = false;
				m_bodies[i] = m_world->CreateBody(&bd);

				m_bodies[i]->CreateFixture(&shape, 1.0f);
			}
		}
	}

	// Implement contact listener.
	void BeginContact(b2Contact* contact) override
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = true;
			}
		}
	}

	// Implement contact listener.
	void EndContact(b2Contact* contact) override
	{
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA == m_sensor)
		{
			void* userData = fixtureB->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}

		if (fixtureB == m_sensor)
		{
			void* userData = fixtureA->GetBody()->GetUserData();
			if (userData)
			{
				bool* touching = (bool*)userData;
				*touching = false;
			}
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int32 i = 0; i < e_count; ++i)
		{
			if (m_touching[i] == false)
			{
				continue;
			}

			b2Body* body = m_bodies[i];
			b2Body* ground = m_sensor->GetBody();

			b2CircleShape* circle = (b2CircleShape*)m_sensor->GetShape();
			b2Vec2 center = ground->GetWorldPoint(circle->m_p);

			b2Vec2 position = body->GetPosition();

			b2Vec2 d = center - position;
			if (d.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
			{
				continue;
			}

			d.Normalize();
			b2Vec2 F = 100.0f * d;
			body->ApplyForce(F, position, false);
		}
	}

	static Test* Create()
	{
		return new Sensors;
	}

	b2Fixture* m_sensor;
	b2Body* m_bodies[e_count];
	bool m_touching[e_count];
};

static int testIndex = RegisterTest("Collision", "Sensors", Sensors::Create);
