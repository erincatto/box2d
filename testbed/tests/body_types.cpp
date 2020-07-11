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

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));

			b2FixtureDef fd;
			fd.shape = &shape;

			ground->CreateFixture(&fd);
		}

		// Define attachment
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 3.0f);
			m_attachment = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 2.0f);
			m_attachment->CreateFixture(&shape, 2.0f);
		}

		// Define platform
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-4.0f, 5.0f);
			m_platform = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 4.0f, b2Vec2(4.0f, 0.0f), 0.5f * b2_pi);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;
			m_platform->CreateFixture(&fd);

			b2RevoluteJointDef rjd;
			rjd.Initialize(m_attachment, m_platform, b2Vec2(0.0f, 5.0f));
			rjd.maxMotorTorque = 50.0f;
			rjd.enableMotor = true;
			m_world->CreateJoint(&rjd);

			b2PrismaticJointDef pjd;
			pjd.Initialize(ground, m_platform, b2Vec2(0.0f, 5.0f), b2Vec2(1.0f, 0.0f));

			pjd.maxMotorForce = 1000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0f;
			pjd.upperTranslation = 10.0f;
			pjd.enableLimit = true;

			m_world->CreateJoint(&pjd);

			m_speed = 3.0f;
		}

		// Create a payload
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 8.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.75f, 0.75f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;

			body->CreateFixture(&fd);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_D:
			m_platform->SetType(b2_dynamicBody);
			break;

		case GLFW_KEY_S:
			m_platform->SetType(b2_staticBody);
			break;

		case GLFW_KEY_K:
			m_platform->SetType(b2_kinematicBody);
			m_platform->SetLinearVelocity(b2Vec2(-m_speed, 0.0f));
			m_platform->SetAngularVelocity(0.0f);
			break;
		}
	}

	void Step(Settings& settings) override
	{
		// Drive the kinematic body.
		if (m_platform->GetType() == b2_kinematicBody)
		{
			b2Vec2 p = m_platform->GetTransform().p;
			b2Vec2 v = m_platform->GetLinearVelocity();

			if ((p.x < -10.0f && v.x < 0.0f) ||
				(p.x > 10.0f && v.x > 0.0f))
			{
				v.x = -v.x;
				m_platform->SetLinearVelocity(v);
			}
		}

		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new BodyTypes;
	}

	b2Body* m_attachment;
	b2Body* m_platform;
	float m_speed;
};

static int testIndex = RegisterTest("Examples", "Body Types", BodyTypes::Create);
