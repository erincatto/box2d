/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#include "gtest/gtest.h"
#include "Box2D/Box2D.h"
#include <stdio.h>
#include "BodyTracker.h"
#define BASELINE_TEST_EPSILON 0.0001f

class HelloWorldTests : public ::testing::Test {
		protected:
	virtual void SetUp();
	virtual void TearDown();

	b2World *m_world;
	b2Body *m_groundBody;
	b2Body *m_body;
};

void
HelloWorldTests::SetUp()
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	m_world = new b2World(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	m_groundBody = m_world->CreateBody(&groundBodyDef);

	// Define the ground box shape.
	b2PolygonShape groundBox;

	// The extents are the half-widths of the box.
	groundBox.SetAsBox(50.0f, 10.0f);

	// Add the ground fixture to the ground body.
	m_groundBody->CreateFixture(&groundBox, 0.0f);

	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(0.0f, 4.0f);
	m_body = m_world->CreateBody(&bodyDef);

	// Define another box shape for our dynamic body.
	b2PolygonShape dynamicBox;
	dynamicBox.SetAsBox(1.0f, 1.0f);

	// Define the dynamic body fixture.
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;

	// Set the box density to be non-zero, so it will be dynamic.
	fixtureDef.density = 1.0f;

	// Override the default friction.
	fixtureDef.friction = 0.3f;

	// Add the shape to the body.
	m_body->CreateFixture(&fixtureDef);
}
void
HelloWorldTests::TearDown()
{
	// Intentionally blank.
}

TEST_F(HelloWorldTests, Allocation) {
	EXPECT_TRUE(m_world != NULL) << "Failed allocation of b2World";
	EXPECT_TRUE(m_groundBody  != NULL) << "Failed allocation of static b2Body";
	EXPECT_TRUE(m_body !=  NULL) << "Failed allocation of dynamic b2Body";
}

TEST_F(HelloWorldTests, PositionAngleTest) {
	float32 timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 6;
	int32 positionIterations = 2;
	BodyTracker tracker("baselines/HelloWorldPositionAngle.txt",
			"testOutput/HelloWorldPositionAngle.txt",
			BodyTracker::TRACK_POSITION | BodyTracker::TRACK_ANGLE);

	tracker.TrackBody(m_body, "DynamicBody");

	// Check that the tracker engaged properly and call the test failed if it
	// doesn't.
	EXPECT_TRUE(tracker.BeginTracking()) << "Problems setting up BodyTracker";

	for (int32 i = 0; i < 60; ++i)
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		m_world->Step(timeStep, velocityIterations, positionIterations);

		tracker.TrackStep(m_body, static_cast<float32>(i));
	}
	tracker.EndTracking();
	bool matched = tracker.CompareToBaseline(m_body,
			BodyTracker::TRACK_POSITION | BodyTracker::TRACK_ANGLE,
			BASELINE_TEST_EPSILON);
	std::string errString;
	if (!matched) {
		const std::vector<std::string> &errors = tracker.GetErrors();
		for (size_t i = 0 ; i < errors.size() ; i++ )
			errString += "\t" + errors[i] + "\n";
	}
	EXPECT_TRUE(matched) << errString;
}

int
main(int argc, char **argv)
{
#if !(defined(ANDROID) || defined(__ANDROID__))
	BodyTracker::SetWorkingDirectory(argv[0]);
#endif  // !(defined(ANDROID) || defined(__ANDROID__))
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
