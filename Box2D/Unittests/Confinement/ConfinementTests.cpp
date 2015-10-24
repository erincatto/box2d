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
#include <complex>
#include "BodyTracker.h"
#define EPSILON 0.001f
#define DELTA_T 1.0f
#define NUMBER_OF_STEPS 210
#define WIDTH 0.1f
#define HEIGHT 0.1f

class ConfinementTests : public ::testing::Test {
		protected:
	virtual void SetUp();
	virtual void TearDown();
	int32 TestLeakCount();

	b2World *m_world;
	b2Body *m_groundBody;
	b2ParticleSystem *m_particleSystem;
	b2ParticleGroup *m_particleGroup;
};

void
ConfinementTests::SetUp()
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	m_world = new b2World(gravity);

	// Create the ground body
	const b2BodyDef groundBodyDef;
	m_groundBody = m_world->CreateBody(&groundBodyDef);

	// Create the particle system
	b2ParticleSystemDef particleSystemDef;
	particleSystemDef.radius = 0.01f;
	m_particleSystem = m_world->CreateParticleSystem(&particleSystemDef);

	// Create particles
	b2ParticleGroupDef particleDef;
	b2PolygonShape particleShape;
	particleShape.SetAsBox(WIDTH, HEIGHT);
	particleDef.shape = &particleShape;
	m_particleGroup = m_particleSystem->CreateParticleGroup(particleDef);
}

void
ConfinementTests::TearDown()
{
	// Intentionally blank.
}

int32
ConfinementTests::TestLeakCount()
{
	for (int32 t = 0; t < NUMBER_OF_STEPS; t++) {
		m_world->Step(DELTA_T, 1, 1);
	}
	int32 bufferIndex = m_particleGroup->GetBufferIndex();
	int32 particleCount = m_particleGroup->GetParticleCount();
	const b2Vec2 *positionBuffer = m_particleSystem->GetPositionBuffer();
	int32 leakCount = 0;
	for (int32 i = 0; i < particleCount; i++) {
		b2Vec2 p = positionBuffer[bufferIndex + i];
		if (std::abs(p.x) > WIDTH || std::abs(p.y) > HEIGHT) {
			leakCount++;
		}
	}
	return leakCount;
}

TEST_F(ConfinementTests, NoShapes) {
	ASSERT_EQ(TestLeakCount(), m_particleSystem->GetParticleCount());
}

TEST_F(ConfinementTests, PolygonShapes) {
	b2PolygonShape shape;
	shape.SetAsBox(WIDTH, HEIGHT, b2Vec2(2 * WIDTH, 0), 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.SetAsBox(WIDTH, HEIGHT, b2Vec2(-2 * WIDTH, 0), 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.SetAsBox(WIDTH, HEIGHT, b2Vec2(0, 2 * HEIGHT), 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.SetAsBox(WIDTH, HEIGHT, b2Vec2(0, -2 * HEIGHT), 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	ASSERT_EQ(TestLeakCount(), 0);
}

TEST_F(ConfinementTests, CircleShapes) {
	b2CircleShape shape;
	shape.m_radius = b2Sqrt(WIDTH * WIDTH + HEIGHT * HEIGHT);
	shape.m_p = b2Vec2(2 * WIDTH, 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(-2 * WIDTH, 0);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(0, 2 * HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(0, -2 * HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(-WIDTH, -HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(WIDTH, -HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(-WIDTH, HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.m_p = b2Vec2(WIDTH, HEIGHT);
	m_groundBody->CreateFixture(&shape, 0.0f);
	ASSERT_EQ(TestLeakCount(), 0);
}

TEST_F(ConfinementTests, EdgeShapes) {
	b2EdgeShape shape;
	shape.Set(b2Vec2(-WIDTH, -HEIGHT), b2Vec2(WIDTH, -HEIGHT));
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.Set(b2Vec2(WIDTH, -HEIGHT), b2Vec2(WIDTH, HEIGHT));
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.Set(b2Vec2(WIDTH, HEIGHT), b2Vec2(-WIDTH, HEIGHT));
	m_groundBody->CreateFixture(&shape, 0.0f);
	shape.Set(b2Vec2(-WIDTH, HEIGHT), b2Vec2(-WIDTH, -HEIGHT));
	m_groundBody->CreateFixture(&shape, 0.0f);
	ASSERT_EQ(TestLeakCount(), 0);
}

TEST_F(ConfinementTests, ChainShape) {
	b2ChainShape shape;
	const b2Vec2 vertices[4] = {
		b2Vec2(-WIDTH, -HEIGHT),
		b2Vec2(WIDTH, -HEIGHT),
		b2Vec2(WIDTH, HEIGHT),
		b2Vec2(-WIDTH, HEIGHT)};
	shape.CreateLoop(vertices, 4);
	m_groundBody->CreateFixture(&shape, 0.0f);
	ASSERT_EQ(TestLeakCount(), 0);
}

TEST_F(ConfinementTests, WallParticle) {
	b2ParticleGroupDef particleDef;
	particleDef.flags = b2_wallParticle;
	b2ChainShape shape;
	const b2Vec2 vertices[4] = {
		b2Vec2(-WIDTH, -HEIGHT),
		b2Vec2(WIDTH, -HEIGHT),
		b2Vec2(WIDTH, HEIGHT),
		b2Vec2(-WIDTH, HEIGHT)};
	shape.CreateLoop(vertices, 4);
	particleDef.shape = &shape;
	m_particleSystem->CreateParticleGroup(particleDef);
	ASSERT_GT(TestLeakCount(), 0);
}

TEST_F(ConfinementTests, BarrierWallParticle) {
	b2ParticleGroupDef particleDef;
	particleDef.flags = b2_barrierParticle | b2_wallParticle;
	b2ChainShape shape;
	const b2Vec2 vertices[4] = {
		b2Vec2(-WIDTH, -HEIGHT),
		b2Vec2(WIDTH, -HEIGHT),
		b2Vec2(WIDTH, HEIGHT),
		b2Vec2(-WIDTH, HEIGHT)};
	shape.CreateLoop(vertices, 4);
	particleDef.shape = &shape;
	m_particleSystem->CreateParticleGroup(particleDef);
	ASSERT_EQ(TestLeakCount(), 0);
}

int
main(int argc, char **argv)
{
		::testing::InitGoogleTest(&argc, argv);
		return RUN_ALL_TESTS();
}
