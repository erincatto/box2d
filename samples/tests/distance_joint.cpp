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

// This tests distance joints, body destruction, and joint destruction.
class DistanceJoint : public Test
{
public:
	DistanceJoint()
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
			shape.SetAsBox(0.5f, 0.5f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position.Set(-5.0f, 5.0f);
			m_bodies[0] = m_world->CreateBody(&bd);
			m_bodies[0]->CreateFixture(&shape, 5.0f);

			bd.position.Set(5.0f, 5.0f);
			m_bodies[1] = m_world->CreateBody(&bd);
			m_bodies[1]->CreateFixture(&shape, 5.0f);

			bd.position.Set(5.0f, 15.0f);
			m_bodies[2] = m_world->CreateBody(&bd);
			m_bodies[2]->CreateFixture(&shape, 5.0f);

			bd.position.Set(-5.0f, 15.0f);
			m_bodies[3] = m_world->CreateBody(&bd);
			m_bodies[3]->CreateFixture(&shape, 5.0f);

			b2DistanceJointDef jd;
			b2Vec2 p1, p2, d;

			jd.frequencyHz = 2.0f;
			jd.dampingRatio = 0.0f;

			jd.bodyA = ground;
			jd.bodyB = m_bodies[0];
			jd.localAnchorA.Set(-10.0f, 0.0f);
			jd.localAnchorB.Set(-0.5f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[0] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[1];
			jd.localAnchorA.Set(10.0f, 0.0f);
			jd.localAnchorB.Set(0.5f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[1] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[2];
			jd.localAnchorA.Set(10.0f, 20.0f);
			jd.localAnchorB.Set(0.5f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[2] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[3];
			jd.localAnchorA.Set(-10.0f, 20.0f);
			jd.localAnchorB.Set(-0.5f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[3] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[0];
			jd.bodyB = m_bodies[1];
			jd.localAnchorA.Set(0.5f, 0.0f);
			jd.localAnchorB.Set(-0.5f, 0.0f);;
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[4] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[1];
			jd.bodyB = m_bodies[2];
			jd.localAnchorA.Set(0.0f, 0.5f);
			jd.localAnchorB.Set(0.0f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[5] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[2];
			jd.bodyB = m_bodies[3];
			jd.localAnchorA.Set(-0.5f, 0.0f);
			jd.localAnchorB.Set(0.5f, 0.0f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[6] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[3];
			jd.bodyB = m_bodies[0];
			jd.localAnchorA.Set(0.0f, -0.5f);
			jd.localAnchorB.Set(0.0f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = d.Length();
			m_joints[7] = m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_B:
			for (int32 i = 0; i < 4; ++i)
			{
				if (m_bodies[i])
				{
					m_world->DestroyBody(m_bodies[i]);
					m_bodies[i] = NULL;
					break;
				}
			}
			break;

		case GLFW_KEY_J:
			for (int32 i = 0; i < 8; ++i)
			{
				if (m_joints[i])
				{
					m_world->DestroyJoint(m_joints[i]);
					m_joints[i] = NULL;
					break;
				}
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "This demonstrates a soft distance joint.");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Press: (b) to delete a body, (j) to delete a joint");
		m_textLine += m_textIncrement;
	}

	void JointDestroyed(b2Joint* joint) override
	{
		for (int32 i = 0; i < 8; ++i)
		{
			if (m_joints[i] == joint)
			{
				m_joints[i] = NULL;
				break;
			}
		}
	}

	static Test* Create()
	{
		return new DistanceJoint;
	}

	b2Body* m_bodies[4];
	b2Joint* m_joints[8];
};

static int testIndex = RegisterTest("Joints", "Distance Joint", DistanceJoint::Create);
