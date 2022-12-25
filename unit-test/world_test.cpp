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

static bool begin_contact = false;

class MyContactListener : public b2ContactListener
{
public:
	void BeginContact(b2Contact* contact)
	{
		begin_contact = true;
	}
};

DOCTEST_TEST_CASE("begin contact")
{
	b2World world({ 0.0f, -10.0f });
	MyContactListener listener;
	world.SetContactListener(&listener);

	b2CircleShape circle;
	circle.m_radius = 5.f;

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	b2Body* bodyA = world.CreateBody(&bodyDef);
	b2Body* bodyB = world.CreateBody(&bodyDef);
	bodyA->CreateFixture(&circle, 0.0f);
	bodyB->CreateFixture(&circle, 0.0f);

	bodyA->SetTransform(b2Vec2(0.f, 0.f), 0.f);
	bodyB->SetTransform(b2Vec2(100.f, 0.f), 0.f);

	const float timeStep = 1.f / 60.f;
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;

	world.Step(timeStep, velocityIterations, positionIterations);

	CHECK(world.GetContactList() == nullptr);
	CHECK(begin_contact == false);
	
	bodyB->SetTransform(b2Vec2(1.f, 0.f), 0.f);

	world.Step(timeStep, velocityIterations, positionIterations);

	CHECK(world.GetContactList() != nullptr);
	CHECK(begin_contact == true);
}
