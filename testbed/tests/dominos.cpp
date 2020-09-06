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

class Dominos : public Test
{
public:

	Dominos()
	{
		b2Body* b1;
		{
			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));

			b2BodyDef bd;
			b1 = m_world->CreateBody(&bd);
			b1->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(6.0f, 0.25f);

			b2BodyDef bd;
			bd.position.Set(-1.5f, 10.0f);
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 1.0f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.1f;

			for (int i = 0; i < 10; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-6.0f + 1.0f * i, 11.25f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(7.0f, 0.25f, b2Vec2_zero, 0.3f);

			b2BodyDef bd;
			bd.position.Set(1.0f, 6.0f);
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0f);
		}

		b2Body* b2;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 1.5f);

			b2BodyDef bd;
			bd.position.Set(-7.0f, 4.0f);
			b2 = m_world->CreateBody(&bd);
			b2->CreateFixture(&shape, 0.0f);
		}

		b2Body* b3;
		{
			b2PolygonShape shape;
			shape.SetAsBox(6.0f, 0.125f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-0.9f, 1.0f);
			bd.angle = -0.15f;

			b3 = m_world->CreateBody(&bd);
			b3->CreateFixture(&shape, 10.0f);
		}

		b2RevoluteJointDef jd;
		b2Vec2 anchor;

		anchor.Set(-2.0f, 1.0f);
		jd.Initialize(b1, b3, anchor);
		jd.collideConnected = true;
		m_world->CreateJoint(&jd);

		b2Body* b4;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 0.25f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-10.0f, 15.0f);
			b4 = m_world->CreateBody(&bd);
			b4->CreateFixture(&shape, 10.0f);
		}

		anchor.Set(-7.0f, 15.0f);
		jd.Initialize(b2, b4, anchor);
		m_world->CreateJoint(&jd);

		b2Body* b5;
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(6.5f, 3.0f);
			b5 = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			b2FixtureDef fd;

			fd.shape = &shape;
			fd.density = 10.0f;
			fd.friction = 0.1f;

			shape.SetAsBox(1.0f, 0.1f, b2Vec2(0.0f, -0.9f), 0.0f);
			b5->CreateFixture(&fd);

			shape.SetAsBox(0.1f, 1.0f, b2Vec2(-0.9f, 0.0f), 0.0f);
			b5->CreateFixture(&fd);

			shape.SetAsBox(0.1f, 1.0f, b2Vec2(0.9f, 0.0f), 0.0f);
			b5->CreateFixture(&fd);
		}

		anchor.Set(6.0f, 2.0f);
		jd.Initialize(b1, b5, anchor);
		m_world->CreateJoint(&jd);

		b2Body* b6;
		{
			b2PolygonShape shape;
			shape.SetAsBox(1.0f, 0.1f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(6.5f, 4.1f);
			b6 = m_world->CreateBody(&bd);
			b6->CreateFixture(&shape, 30.0f);
		}

		anchor.Set(7.5f, 4.0f);
		jd.Initialize(b5, b6, anchor);
		m_world->CreateJoint(&jd);

		b2Body* b7;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 1.0f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(7.4f, 1.0f);

			b7 = m_world->CreateBody(&bd);
			b7->CreateFixture(&shape, 10.0f);
		}

		b2DistanceJointDef djd;
		djd.bodyA = b3;
		djd.bodyB = b7;
		djd.localAnchorA.Set(6.0f, 0.0f);
		djd.localAnchorB.Set(0.0f, -1.0f);
		b2Vec2 d = djd.bodyB->GetWorldPoint(djd.localAnchorB) - djd.bodyA->GetWorldPoint(djd.localAnchorA);
		djd.length = d.Length();

		b2LinearStiffness(djd.stiffness, djd.damping, 1.0f, 1.0f, djd.bodyA, djd.bodyB);
		m_world->CreateJoint(&djd);

		{
			float radius = 0.2f;

			b2CircleShape shape;
			shape.m_radius = radius;

			for (int32 i = 0; i < 4; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(5.9f + 2.0f * radius * i, 2.4f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 10.0f);
			}
		}
	}

	static Test* Create()
	{
		return new Dominos;
	}
};

static int testIndex = RegisterTest("Examples", "Dominos", Dominos::Create);
