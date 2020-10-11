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

class GearJoint : public Test
{
public:
	GearJoint()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2CircleShape circle1;
			circle1.m_radius = 1.0f;

			b2PolygonShape box;
			box.SetAsBox(0.5f, 5.0f);

			b2CircleShape circle2;
			circle2.m_radius = 2.0f;
			
			b2BodyDef bd1;
			bd1.type = b2_staticBody;
			bd1.position.Set(10.0f, 9.0f);
			b2Body* body1 = m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0f);

			b2BodyDef bd2;
			bd2.type = b2_dynamicBody;
			bd2.position.Set(10.0f, 8.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);
			body2->CreateFixture(&box, 5.0f);

			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;
			bd3.position.Set(10.0f, 6.0f);
			b2Body* body3 = m_world->CreateBody(&bd3);
			body3->CreateFixture(&circle2, 5.0f);

			b2RevoluteJointDef jd1;
			jd1.Initialize(body1, body2, bd1.position);
			b2Joint* joint1 = m_world->CreateJoint(&jd1);

			b2RevoluteJointDef jd2;
			jd2.Initialize(body2, body3, bd3.position);
			b2Joint* joint2 = m_world->CreateJoint(&jd2);

			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_world->CreateJoint(&jd4);
		}

		{
			b2CircleShape circle1;
			circle1.m_radius = 1.0f;

			b2CircleShape circle2;
			circle2.m_radius = 2.0f;
			
			b2PolygonShape box;
			box.SetAsBox(0.5f, 5.0f);

			b2BodyDef bd1;
			bd1.type = b2_dynamicBody;
			bd1.position.Set(-3.0f, 12.0f);
			b2Body* body1 = m_world->CreateBody(&bd1);
			body1->CreateFixture(&circle1, 5.0f);

			b2RevoluteJointDef jd1;
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA = ground->GetLocalPoint(bd1.position);
			jd1.localAnchorB = body1->GetLocalPoint(bd1.position);
			jd1.referenceAngle = body1->GetAngle() - ground->GetAngle();
			m_joint1 = (b2RevoluteJoint*)m_world->CreateJoint(&jd1);

			b2BodyDef bd2;
			bd2.type = b2_dynamicBody;
			bd2.position.Set(0.0f, 12.0f);
			b2Body* body2 = m_world->CreateBody(&bd2);
			body2->CreateFixture(&circle2, 5.0f);

			b2RevoluteJointDef jd2;
			jd2.Initialize(ground, body2, bd2.position);
			m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jd2);

			b2BodyDef bd3;
			bd3.type = b2_dynamicBody;
			bd3.position.Set(2.5f, 12.0f);
			b2Body* body3 = m_world->CreateBody(&bd3);
			body3->CreateFixture(&box, 5.0f);

			b2PrismaticJointDef jd3;
			jd3.Initialize(ground, body3, bd3.position, b2Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = (b2PrismaticJoint*)m_world->CreateJoint(&jd3);

			b2GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_joint4 = (b2GearJoint*)m_world->CreateJoint(&jd4);

			b2GearJointDef jd5;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.m_radius;
			m_joint5 = (b2GearJoint*)m_world->CreateJoint(&jd5);
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		float ratio, value;
		
		ratio = m_joint4->GetRatio();
		value = m_joint1->GetJointAngle() + ratio * m_joint2->GetJointAngle();
		g_debugDraw.DrawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", (float) ratio, (float) value);
		m_textLine += m_textIncrement;

		ratio = m_joint5->GetRatio();
		value = m_joint2->GetJointAngle() + ratio * m_joint3->GetJointTranslation();
		g_debugDraw.DrawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", (float) ratio, (float) value);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new GearJoint;
	}

	b2RevoluteJoint* m_joint1;
	b2RevoluteJoint* m_joint2;
	b2PrismaticJoint* m_joint3;
	b2GearJoint* m_joint4;
	b2GearJoint* m_joint5;
};

static int testIndex = RegisterTest("Joints", "Gear", GearJoint::Create);
