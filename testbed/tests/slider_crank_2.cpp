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

#include "settings.h"
#include "test.h"

// A motor driven slider crank with joint friction.

class SliderCrank2 : public Test
{
public:
	SliderCrank2()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2Body* prevBody = ground;

			// Define crank.
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 2.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 7.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0f, 5.0f));
				rjd.motorSpeed = 1.0f * b2_pi;
				rjd.maxMotorTorque = 10000.0f;
				rjd.enableMotor = true;
				m_joint1 = (b2RevoluteJoint*)m_world->CreateJoint(&rjd);

				prevBody = body;
			}

			// Define follower.
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5f, 4.0f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 13.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0f, 9.0f));
				rjd.enableMotor = false;
				m_world->CreateJoint(&rjd);

				prevBody = body;
			}

			// Define piston
			{
				b2PolygonShape shape;
				shape.SetAsBox(1.5f, 1.5f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.fixedRotation = true;
				bd.position.Set(0.0f, 17.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0f, 17.0f));
				m_world->CreateJoint(&rjd);

				b2PrismaticJointDef pjd;
				pjd.Initialize(ground, body, b2Vec2(0.0f, 17.0f), b2Vec2(0.0f, 1.0f));

				pjd.maxMotorForce = 1000.0f;
				pjd.enableMotor = true;

				m_joint2 = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
			}

			// Create a payload
			{
				b2PolygonShape shape;
				shape.SetAsBox(1.5f, 1.5f);

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.0f, 23.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);
			}
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_F:
			m_joint2->EnableMotor(!m_joint2->IsMotorEnabled());
			m_joint2->GetBodyB()->SetAwake(true);
			break;

		case GLFW_KEY_M:
			m_joint1->EnableMotor(!m_joint1->IsMotorEnabled());
			m_joint1->GetBodyB()->SetAwake(true);
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
		m_textLine += m_textIncrement;
		float torque = m_joint1->GetMotorTorque(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %5.0f", (float) torque);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new SliderCrank2;
	}

	b2RevoluteJoint* m_joint1;
	b2PrismaticJoint* m_joint2;
};

static int testIndex = RegisterTest("Examples", "Slider Crank 2", SliderCrank2::Create);
