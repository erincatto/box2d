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

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
class MotorJoint : public Test
{
public:
	MotorJoint()
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

		// Define motorized body
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 8.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(2.0f, 0.5f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;
			body->CreateFixture(&fd);

			b2MotorJointDef mjd;
			mjd.Initialize(ground, body);
			mjd.maxForce = 1000.0f;
			mjd.maxTorque = 1000.0f;
			m_joint = (b2MotorJoint*)m_world->CreateJoint(&mjd);
		}

		m_go = false;
		m_time = 0.0f;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_S:
			m_go = !m_go;
			break;
		}
	}

	void Step(Settings& settings) override
	{
		if (m_go && settings.m_hertz > 0.0f)
		{
			m_time += 1.0f / settings.m_hertz;
		}

		b2Vec2 linearOffset;
		linearOffset.x = 6.0f * sinf(2.0f * m_time);
		linearOffset.y = 8.0f + 4.0f * sinf(1.0f * m_time);
		
		float angularOffset = 4.0f * m_time;

		m_joint->SetLinearOffset(linearOffset);
		m_joint->SetAngularOffset(angularOffset);

		g_debugDraw.DrawPoint(linearOffset, 4.0f, b2Color(0.9f, 0.9f, 0.9f));

		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (s) pause");
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new MotorJoint;
	}

	b2MotorJoint* m_joint;
	float m_time;
	bool m_go;
};

static int testIndex = RegisterTest("Joints", "Motor Joint", MotorJoint::Create);
