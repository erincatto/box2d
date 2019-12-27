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

// The motor in this test gets smoother with higher velocity iterations.
class PrismaticJoint : public Test
{
public:
	PrismaticJoint()
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
			b2PolygonShape shape;
			shape.SetAsBox(2.0f, 0.5f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-10.0f, 10.0f);
			bd.angle = 0.5f * b2_pi;
			bd.allowSleep = false;
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0f);

			b2PrismaticJointDef pjd;

			// Bouncy limit
			b2Vec2 axis(2.0f, 1.0f);
			axis.Normalize();
			pjd.Initialize(ground, body, b2Vec2(0.0f, 0.0f), axis);

			// Non-bouncy limit
			//pjd.Initialize(ground, body, b2Vec2(-10.0f, 10.0f), b2Vec2(1.0f, 0.0f));

			pjd.motorSpeed = 10.0f;
			pjd.maxMotorForce = 10000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = 0.0f;
			pjd.upperTranslation = 20.0f;
			pjd.enableLimit = true;

			m_joint = (b2PrismaticJoint*)m_world->CreateJoint(&pjd);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_L:
			m_joint->EnableLimit(!m_joint->IsLimitEnabled());
			break;

		case GLFW_KEY_M:
			m_joint->EnableMotor(!m_joint->IsMotorEnabled());
			break;

		case GLFW_KEY_S:
			m_joint->SetMotorSpeed(-m_joint->GetMotorSpeed());
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motors, (s) speed");
		m_textLine += m_textIncrement;
		float force = m_joint->GetMotorForce(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Force = %4.0f", (float) force);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new PrismaticJoint;
	}

	b2PrismaticJoint* m_joint;
};

static int testIndex = RegisterTest("Joints", "Prismatic", PrismaticJoint::Create);
