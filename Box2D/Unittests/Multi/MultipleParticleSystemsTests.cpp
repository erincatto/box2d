/*
* Copyright (c) 2014 Google, Inc.
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
#include "AndroidUtil/AndroidMainWrapper.h"

static const int kNumParticleSystems = 3;
static const int kNumParticlesPerSystem = 3;

class MultipleParticleSystemsTests : public ::testing::Test {
protected:
	virtual void SetUp();
	virtual void TearDown();
	void drop();

	b2World *m_world;
	b2Body *m_body;
	b2ParticleSystem *m_particleSystems[kNumParticleSystems];
};

void
MultipleParticleSystemsTests::SetUp()
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will simulate the contacts.
	m_world = new b2World(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, 0.0f);

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	m_body = m_world->CreateBody(&groundBodyDef);

	// Construct a valley and add to ground body.
	b2PolygonShape shape;
	const b2Vec2 vertices[3] = {
		b2Vec2(-10.0, 34.0),
		b2Vec2(-10.0, 35.0),
		b2Vec2(0.0, 15.0)};
	shape.Set(vertices, 3);
	m_body->CreateFixture(&shape, 0.0f);

	const b2Vec2 vertices2[3] = {
		b2Vec2(10.0, 34.0),
		b2Vec2(10.0, 35.0),
		b2Vec2(0.0, 15.0)};
	shape.Set(vertices2, 3);
	m_body->CreateFixture(&shape, 0.0f);

	// Construct several particle systems, all with the same definition.
	const b2ParticleSystemDef particleSystemDef;
	for (int i = 0; i < kNumParticleSystems; ++i) {
		m_particleSystems[i] =
			m_world->CreateParticleSystem(&particleSystemDef);
	}

	// Add particles to the particle systems.
	const float kParticlePositionsXStart = -10.0f;
	const float kParticlePositionsYStart = 32.0f;
	const float kParticlePositionsXEnd = 10.0f;
	const float kParticlePositionsYEnd = 33.0f;
	const float kParticlePositionsXStep =
		(kParticlePositionsXEnd - kParticlePositionsXStart)
		/ (kNumParticlesPerSystem - 1);
	const float kParticlePositionsYStep =
		(kParticlePositionsYEnd - kParticlePositionsYStart)
		/ (kNumParticlesPerSystem - 1);

	b2ParticleDef pd;
	pd.velocity.Set(0.0, -1.0);

	// Particles are spaced evenly between (X_START, Y_START) and
	// (X_END, Y_END).
	float x = kParticlePositionsXStart;
	float y = kParticlePositionsYStart;
	for (int j = 0; j < kNumParticlesPerSystem; ++j) {
		pd.position.Set(x, y);

		for (int i = 0; i < kNumParticleSystems; ++i) {
			m_particleSystems[i]->CreateParticle(pd);
		}

		x += kParticlePositionsXStep;
		y += kParticlePositionsYStep;
	}
}
void
MultipleParticleSystemsTests::TearDown()
{
	// Intentionally blank.
}

// Ensure that all particle systems give the same result, when they are
// initialized the same.
TEST_F(MultipleParticleSystemsTests, IdenticalSimulations) {

	// Count the number of contacts for each particle system
	int32 contacts[kNumParticleSystems];
	for (int j = 0; j < kNumParticleSystems; ++j) {
		contacts[j] = 0;
	}

	// Simulate the world for 10 seconds.
	// All of the particle systems should run the same.
	const float32 timeStep = 1.0f / 60.0f;
	const int32 timeout = (int32) (1.0f / timeStep) * 10; // 10 "seconds"
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;
	for (int i = 0; i < timeout; ++i) {
		m_world->Step(timeStep, velocityIterations, positionIterations);

		for (int j = 0; j < kNumParticleSystems; ++j) {
			contacts[j] += m_particleSystems[j]->GetBodyContactCount();
		}
	}

	// Verify that all the particle systems are in the same state.
	const b2Vec2* positions0 =
		m_particleSystems[0]->GetPositionBuffer();
	const b2Vec2* velocities0 =
		m_particleSystems[0]->GetVelocityBuffer();
	for (int j = 1; j < kNumParticleSystems; ++j) {
		// Check that the particle count matches.
		const int32 particleCount = m_particleSystems[0]->GetParticleCount();
		ASSERT_EQ(particleCount, m_particleSystems[j]->GetParticleCount());

		// Check that the particle positions and velocities match.
		const b2Vec2* positionsJ =
			m_particleSystems[j]->GetPositionBuffer();
		const b2Vec2* velocitiesJ =
			m_particleSystems[j]->GetVelocityBuffer();
		for (int k = 0; k < particleCount; ++k ) {
			EXPECT_EQ(positions0[k], positionsJ[k]) << "Positions differ";
			EXPECT_EQ(velocities0[k], velocitiesJ[k]) << "Velocities differ";
		}

		// Check that the contact reporting matches.
		EXPECT_EQ(contacts[0], contacts[j]) << "Contacts differ";
	}
}

int
main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
