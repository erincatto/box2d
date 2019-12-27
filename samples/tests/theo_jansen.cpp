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

// Inspired by a contribution from roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

#include "test.h"

class TheoJansen : public Test
{
public:

	void CreateLeg(float s, const b2Vec2& wheelAnchor)
	{
		b2Vec2 p1(5.4f * s, -6.1f);
		b2Vec2 p2(7.2f * s, -1.2f);
		b2Vec2 p3(4.3f * s, -1.9f);
		b2Vec2 p4(3.1f * s, 0.8f);
		b2Vec2 p5(6.0f * s, 1.5f);
		b2Vec2 p6(2.5f * s, 3.7f);

		b2FixtureDef fd1, fd2;
		fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0f;
		fd2.density = 1.0f;

		b2PolygonShape poly1, poly2;

		if (s > 0.0f)
		{
			b2Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p2;
			vertices[2] = p3;
			poly1.Set(vertices, 3);

			vertices[0] = b2Vec2_zero;
			vertices[1] = p5 - p4;
			vertices[2] = p6 - p4;
			poly2.Set(vertices, 3);
		}
		else
		{
			b2Vec2 vertices[3];

			vertices[0] = p1;
			vertices[1] = p3;
			vertices[2] = p2;
			poly1.Set(vertices, 3);

			vertices[0] = b2Vec2_zero;
			vertices[1] = p6 - p4;
			vertices[2] = p5 - p4;
			poly2.Set(vertices, 3);
		}

		fd1.shape = &poly1;
		fd2.shape = &poly2;

		b2BodyDef bd1, bd2;
		bd1.type = b2_dynamicBody;
		bd2.type = b2_dynamicBody;
		bd1.position = m_offset;
		bd2.position = p4 + m_offset;

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

		b2Body* body1 = m_world->CreateBody(&bd1);
		b2Body* body2 = m_world->CreateBody(&bd2);

		body1->CreateFixture(&fd1);
		body2->CreateFixture(&fd2);

		b2DistanceJointDef djd;

		// Using a soft distance constraint can reduce some jitter.
		// It also makes the structure seem a bit more fluid by
		// acting like a suspension system.
		djd.dampingRatio = 0.5f;
		djd.frequencyHz = 10.0f;

		djd.Initialize(body1, body2, p2 + m_offset, p5 + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body1, body2, p3 + m_offset, p4 + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body1, m_wheel, p3 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&djd);

		djd.Initialize(body2, m_wheel, p6 + m_offset, wheelAnchor + m_offset);
		m_world->CreateJoint(&djd);

		b2RevoluteJointDef rjd;

		rjd.Initialize(body2, m_chassis, p4 + m_offset);
		m_world->CreateJoint(&rjd);
	}

	TheoJansen()
	{
		m_offset.Set(0.0f, 8.0f);
		m_motorSpeed = 2.0f;
		m_motorOn = true;
		b2Vec2 pivot(0.0f, 0.8f);

		// Ground
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-50.0f, 0.0f), b2Vec2(50.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(b2Vec2(-50.0f, 0.0f), b2Vec2(-50.0f, 10.0f));
			ground->CreateFixture(&shape, 0.0f);

			shape.Set(b2Vec2(50.0f, 0.0f), b2Vec2(50.0f, 10.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		// Balls
		for (int32 i = 0; i < 40; ++i)
		{
			b2CircleShape shape;
			shape.m_radius = 0.25f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-40.0f + 2.0f * i, 0.5f);

			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 1.0f);
		}

		// Chassis
		{
			b2PolygonShape shape;
			shape.SetAsBox(2.5f, 1.0f);

			b2FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = pivot + m_offset;
			m_chassis = m_world->CreateBody(&bd);
			m_chassis->CreateFixture(&sd);
		}

		{
			b2CircleShape shape;
			shape.m_radius = 1.6f;

			b2FixtureDef sd;
			sd.density = 1.0f;
			sd.shape = &shape;
			sd.filter.groupIndex = -1;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = pivot + m_offset;
			m_wheel = m_world->CreateBody(&bd);
			m_wheel->CreateFixture(&sd);
		}

		{
			b2RevoluteJointDef jd;
			jd.Initialize(m_wheel, m_chassis, pivot + m_offset);
			jd.collideConnected = false;
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 400.0f;
			jd.enableMotor = m_motorOn;
			m_motorJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		b2Vec2 wheelAnchor;
		
		wheelAnchor = pivot + b2Vec2(0.0f, -0.8f);

		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetPosition(), 120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		m_wheel->SetTransform(m_wheel->GetPosition(), -120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
		m_textLine += m_textIncrement;

		Test::Step(settings);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_motorJoint->SetMotorSpeed(-m_motorSpeed);
			break;

		case GLFW_KEY_S:
			m_motorJoint->SetMotorSpeed(0.0f);
			break;

		case GLFW_KEY_D:
			m_motorJoint->SetMotorSpeed(m_motorSpeed);
			break;

		case GLFW_KEY_M:
			m_motorJoint->EnableMotor(!m_motorJoint->IsMotorEnabled());
			break;
		}
	}

	static Test* Create()
	{
		return new TheoJansen;
	}

	b2Vec2 m_offset;
	b2Body* m_chassis;
	b2Body* m_wheel;
	b2RevoluteJoint* m_motorJoint;
	bool m_motorOn;
	float m_motorSpeed;
};

static int testIndex = RegisterTest("Examples", "Theo Jansen", TheoJansen::Create);
