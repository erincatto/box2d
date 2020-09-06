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
#include "imgui/imgui.h"

/// This test shows how a distance joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the distance joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the distance joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class WreckingBall : public Test
{
public:
	WreckingBall()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
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
			m_distanceJointDef.localAnchorA.Set(0.0f, y);

			b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(0.5f + 1.0f * i, y);
				if (i == N - 1)
				{
					bd.position.Set(1.0f * i, y);
					bd.angularDamping = 0.4f;
				}

				b2Body* body = m_world->CreateBody(&bd);

				if (i == N - 1)
				{
					b2CircleShape circleShape;
					circleShape.m_radius = 1.5f;
					b2FixtureDef sfd;
					sfd.shape = &circleShape;
					sfd.density = 100.0f;
					sfd.filter.categoryBits = 0x0002;
					body->CreateFixture(&sfd);
				}
				else
				{
					body->CreateFixture(&fd);
				}

				b2Vec2 anchor(float(i), y);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;
			}

			m_distanceJointDef.localAnchorB.SetZero();

			float extraLength = 0.01f;
			m_distanceJointDef.minLength = 0.0f;
			m_distanceJointDef.maxLength = N - 1.0f + extraLength;
			m_distanceJointDef.bodyB = prevBody;
		}

		{
			m_distanceJointDef.bodyA = ground;
			m_distanceJoint = m_world->CreateJoint(&m_distanceJointDef);
			m_stabilize = true;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Wrecking Ball Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Stabilize", &m_stabilize))
		{
			if (m_stabilize == true && m_distanceJoint == nullptr)
			{
				m_distanceJoint = m_world->CreateJoint(&m_distanceJointDef);
			}
			else if (m_stabilize == false && m_distanceJoint != nullptr)
			{
				m_world->DestroyJoint(m_distanceJoint);
				m_distanceJoint = nullptr;
			}
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		if (m_distanceJoint)
		{
			g_debugDraw.DrawString(5, m_textLine, "Distance Joint ON");
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "Distance Joint OFF");
		}
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new WreckingBall;
	}

	b2DistanceJointDef m_distanceJointDef;
	b2Joint* m_distanceJoint;
	bool m_stabilize;
};

static int testIndex = RegisterTest("Examples", "Wrecking Ball", WreckingBall::Create);
