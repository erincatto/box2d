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

#include "box2d/box2d.h"
#include "doctest.h"
#include <stdio.h>

DOCTEST_TEST_CASE("joint reactions")
{
	b2Vec2 gravity(0, -10.0f);
	b2World world = b2World(gravity);

	b2BodyDef bodyDef;
	b2Body* ground = world.CreateBody(&bodyDef);

	b2CircleShape circle;
	circle.m_radius = 1.0f;

	b2FixtureDef fixtureDef;

	// Disable collision
	fixtureDef.filter.maskBits = 0;
	fixtureDef.density = 1.0f;
	fixtureDef.shape = &circle;

	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(-2.0f, 3.0f);

	b2Body* bodyA = world.CreateBody(&bodyDef);
	b2Body* bodyB = world.CreateBody(&bodyDef);
	b2Body* bodyC = world.CreateBody(&bodyDef);

	b2MassData massData;
	circle.ComputeMass(&massData, fixtureDef.density);
	const float mg = massData.mass * gravity.y;

	bodyA->CreateFixture(&fixtureDef);
	bodyB->CreateFixture(&fixtureDef);
	bodyC->CreateFixture(&fixtureDef);

	b2DistanceJointDef distanceJointDef;
	distanceJointDef.Initialize(ground, bodyA, bodyDef.position + b2Vec2(0.0f, 4.0f), bodyDef.position);
	distanceJointDef.minLength = distanceJointDef.length;
	distanceJointDef.maxLength = distanceJointDef.length;

	b2PrismaticJointDef prismaticJointDef;
	prismaticJointDef.Initialize(ground, bodyB, bodyDef.position, b2Vec2(1.0f, 0.0f));

	b2RevoluteJointDef revoluteJointDef;
	revoluteJointDef.Initialize(ground, bodyC, bodyDef.position);

	b2DistanceJoint* distanceJoint = (b2DistanceJoint*)world.CreateJoint(&distanceJointDef);
	b2PrismaticJoint* prismaticJoint = (b2PrismaticJoint*)world.CreateJoint(&prismaticJointDef);
	b2RevoluteJoint* revoluteJoint = (b2RevoluteJoint*)world.CreateJoint(&revoluteJointDef);

	const float timeStep = 1.f / 60.f;
	const float invTimeStep = 60.0f;
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;

	world.Step(timeStep, velocityIterations, positionIterations);

	const float tol = 1e-5f;
	{
		b2Vec2 F = distanceJoint->GetReactionForce(invTimeStep);
		float T = distanceJoint->GetReactionTorque(invTimeStep);
		CHECK(F.x == 0.0f);
		CHECK(b2Abs(F.y + mg) < tol);
		CHECK(T == 0.0f);
	}

	{
		b2Vec2 F = prismaticJoint->GetReactionForce(invTimeStep);
		float T = prismaticJoint->GetReactionTorque(invTimeStep);
		CHECK(F.x == 0.0f);
		CHECK(b2Abs(F.y + mg) < tol);
		CHECK(T == 0.0f);
	}

	{
		b2Vec2 F = revoluteJoint->GetReactionForce(invTimeStep);
		float T = revoluteJoint->GetReactionTorque(invTimeStep);
		CHECK(F.x == 0.0f);
		CHECK(b2Abs(F.y + mg) < tol);
		CHECK(T == 0.0f);
	}
}
