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

// This test holds worlds dumped using b2World::Dump.
class DumpLoader : public Test
{
public:

	DumpLoader()
	{
		b2ChainShape chainShape;
		b2Vec2 vertices[] = {b2Vec2(-5,0), b2Vec2(5,0), b2Vec2(5,5), b2Vec2(4,1), b2Vec2(-4,1), b2Vec2(-5,5)};
		chainShape.CreateLoop(vertices, 6);

		b2FixtureDef groundFixtureDef;
		groundFixtureDef.density = 0;
		groundFixtureDef.shape = &chainShape;

		b2BodyDef groundBodyDef;
		groundBodyDef.type = b2_staticBody;

		b2Body *groundBody = m_world->CreateBody(&groundBodyDef);
		b2Fixture *groundBodyFixture = groundBody->CreateFixture(&groundFixtureDef);

		b2CircleShape ballShape;
		ballShape.m_radius = 1;

		b2FixtureDef ballFixtureDef;
		ballFixtureDef.restitution = 0.75f;
		ballFixtureDef.density = 1;
		ballFixtureDef.shape = &ballShape;

		b2BodyDef ballBodyDef;
		ballBodyDef.type = b2BodyType::b2_dynamicBody;
		ballBodyDef.position = b2Vec2(0, 10);
		// ballBodyDef.angularDamping = 0.2f;

		m_ball = m_world->CreateBody(&ballBodyDef);
		b2Fixture *ballFixture = m_ball->CreateFixture(&ballFixtureDef);
		m_ball->ApplyForceToCenter(b2Vec2(-1000, -400), true);
	}

	void Step(Settings& settings) override
	{
		b2Vec2 v = m_ball->GetLinearVelocity();
		float omega = m_ball->GetAngularVelocity();

		b2MassData massData = m_ball->GetMassData();

		float ke = 0.5f * massData.mass * b2Dot(v, v) + 0.5f * massData.I * omega * omega;

		g_debugDraw.DrawString(5, m_textLine, "kinetic energy = %.6f", ke);
		m_textLine += m_textIncrement;

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new DumpLoader;
	}

	b2Body* m_ball;
};

static int testIndex = RegisterTest("Bugs", "Dump Loader", DumpLoader::Create);
