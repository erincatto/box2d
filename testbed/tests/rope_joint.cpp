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

/// This test shows how a rope joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the rope joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the rope joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class RopeJoint : public Test
{
public:
	RopeJoint()
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
			shape.SetAsBox(0.5f, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			b2RevoluteJointDef jd;
			jd.collideConnected = false;

			const int32 N = 10;
			const float y = 15.0f;
			m_ropeDef.localAnchorA.Set(0.0f, y);

			b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.5f + 1.0f * i, y);
				if (i == N - 1)
				{
					shape.SetAsBox(1.5f, 1.5f);
					fd.density = 100.0f;
					fd.filter.categoryBits = 0x0002;
					bd.position.Set(1.0f * i, y);
					bd.angularDamping = 0.4f;
				}

				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&fd);

				b2Vec2 anchor(float(i), y);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;
			}

			m_ropeDef.localAnchorB.SetZero();

			float extraLength = 0.01f;
			m_ropeDef.maxLength = N - 1.0f + extraLength;
			m_ropeDef.bodyB = prevBody;
		}

		{
			m_ropeDef.bodyA = ground;
			m_rope = m_world->CreateJoint(&m_ropeDef);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_J:
			if (m_rope)
			{
				m_world->DestroyJoint(m_rope);
				m_rope = NULL;
			}
			else
			{
				m_rope = m_world->CreateJoint(&m_ropeDef);
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press (j) to toggle the rope joint.");
		m_textLine += m_textIncrement;
		if (m_rope)
		{
			g_debugDraw.DrawString(5, m_textLine, "Rope ON");
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "Rope OFF");
		}
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new RopeJoint;
	}

	b2RopeJointDef m_ropeDef;
	b2Joint* m_rope;
};

static int testIndex = RegisterTest("Joints", "Rope", RopeJoint::Create);
