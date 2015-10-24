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
#define EPSILON 0.001
#define DELTA_T 0.01f
#define NUMBER_OF_STEPS 200

/// Set of parameters that characterize a particle type.
struct ParticleType {
	const char* name;
	uint32 particleFlags;
	uint32 groupFlags;
};

::std::ostream& operator<<(::std::ostream& os, const ParticleType& param) {
	return os << param.name;
}

class ConservationTests : public ::testing::TestWithParam<ParticleType> {
		protected:
	virtual void SetUp();
	virtual void TearDown();

	b2World *m_world;
	b2ParticleSystem *m_particleSystem;
};

static const ParticleType particleTypes[] = {
	{"water", b2_waterParticle, 0},
	{"wall", b2_wallParticle, 0},
	{"spring", b2_springParticle, 0},
	{"elastic", b2_elasticParticle, 0},
	{"viscous", b2_viscousParticle, 0},
	{"powder", b2_powderParticle, 0},
	{"tensile", b2_tensileParticle, 0},
	{"colorMixing", b2_colorMixingParticle, 0},
	{"staticPressure", b2_staticPressureParticle, 0},
	{"repulsive", b2_repulsiveParticle, 0},
	{"rigid", 0, b2_rigidParticleGroup},
	{"solid rigid", 0, b2_solidParticleGroup | b2_rigidParticleGroup},
	{"solid spring", b2_springParticle, b2_solidParticleGroup},
	{"solid elastic", b2_elasticParticle, b2_solidParticleGroup},
	{"barrier rigid", b2_barrierParticle, b2_rigidParticleGroup},
	{"barrier spring", b2_barrierParticle | b2_springParticle, 0},
	{"barrier elastic", b2_barrierParticle | b2_elasticParticle, 0},
};

INSTANTIATE_TEST_CASE_P(
			ParticleType, ConservationTests, testing::ValuesIn(particleTypes));

void
ConservationTests::SetUp()
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, 0.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	m_world = new b2World(gravity);

	b2ParticleSystemDef particleSystemDef;
	particleSystemDef.radius = 0.01f;
	m_particleSystem = m_world->CreateParticleSystem(&particleSystemDef);

	// Create two particle groups colliding each other
	b2ParticleGroupDef pd;
	const ParticleType& param = GetParam();
	pd.flags = param.particleFlags;
	pd.groupFlags = param.groupFlags;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	pd.shape = &shape;

	pd.position = b2Vec2(-0.1f, 0);
	pd.linearVelocity = b2Vec2(0.01f, 0);
	m_particleSystem->CreateParticleGroup(pd);

	pd.position = b2Vec2(0.1f, 0);
	pd.linearVelocity = b2Vec2(-0.01f, 0);
	m_particleSystem->CreateParticleGroup(pd);
}

void
ConservationTests::TearDown()
{
	// Intentionally blank.
}

TEST_P(ConservationTests, GravityCenter) {
	// Test whether the gravity center is conserved.
	const ParticleType& param = GetParam();
	if (param.particleFlags & b2_barrierParticle)
	{
		// Barrier particles may move other particles to prevent penetration.
		// The gravity center is not preserved in that case.
		SUCCEED();
		return;
	}
	int32 particleCount = m_particleSystem->GetParticleCount();
	const b2Vec2 *positionBuffer = m_particleSystem->GetPositionBuffer();
	float64 beforeX = 0;
	float64 beforeY = 0;
	for (int32 i = 0; i < particleCount; i++) {
		beforeX += positionBuffer[i].x;
		beforeY += positionBuffer[i].y;
	}
	beforeX /= particleCount;
	beforeY /= particleCount;
	for (int32 t = 0; t < NUMBER_OF_STEPS; t++) {
		m_world->Step(DELTA_T, 1, 1);
	}
	float64 afterX = 0;
	float64 afterY = 0;
	for (int32 i = 0; i < particleCount; i++) {
		afterX += positionBuffer[i].x;
		afterY += positionBuffer[i].y;
	}
	afterX /= particleCount;
	afterY /= particleCount;
	EXPECT_NEAR(beforeX, afterX, EPSILON);
	EXPECT_NEAR(beforeY, afterY, EPSILON);
}

TEST_P(ConservationTests, LinearMomentum) {
	// Test whether the linear momentum is conserved.
	int32 particleCount = m_particleSystem->GetParticleCount();
	const b2Vec2 *velocityBuffer = m_particleSystem->GetVelocityBuffer();
	float64 beforeX = 0;
	float64 beforeY = 0;
	for (int32 i = 0; i < particleCount; i++) {
		beforeX += velocityBuffer[i].x;
		beforeY += velocityBuffer[i].y;
	}
	beforeX /= particleCount;
	beforeY /= particleCount;
	for (int32 t = 0; t < NUMBER_OF_STEPS; t++) {
		m_world->Step(DELTA_T, 1, 1);
	}
	float64 afterX = 0;
	float64 afterY = 0;
	for (int32 i = 0; i < particleCount; i++) {
		afterX += velocityBuffer[i].x;
		afterY += velocityBuffer[i].y;
	}
	afterX /= particleCount;
	afterY /= particleCount;
	EXPECT_NEAR(beforeX, afterX, EPSILON);
	EXPECT_NEAR(beforeY, afterY, EPSILON);
}

TEST_P(ConservationTests, AngularMomentum) {
	// Test whether the angular momentum is conserved.
	const ParticleType& param = GetParam();
	if (param.particleFlags &
				(b2_springParticle | b2_elasticParticle | b2_viscousParticle))
	{
		// Spring and elastic particles don't conserve the angular momentum
		// because of implicit method.
		// Viscous particles are designed to reduce the angular momentum.
		SUCCEED();
		return;
	}
	int32 particleCount = m_particleSystem->GetParticleCount();
	const b2Vec2 *positionBuffer = m_particleSystem->GetPositionBuffer();
	const b2Vec2 *velocityBuffer = m_particleSystem->GetVelocityBuffer();
	float64 before = 0;
	for (int32 i = 0; i < particleCount; i++) {
		before += b2Cross(positionBuffer[i], velocityBuffer[i]);
	}
	before /= particleCount;
	for (int32 t = 0; t < NUMBER_OF_STEPS; t++) {
		m_world->Step(DELTA_T, 1, 1);
	}
	float64 after = 0;
	for (int32 i = 0; i < particleCount; i++) {
		after += b2Cross(positionBuffer[i], velocityBuffer[i]);
	}
	after /= particleCount;
	EXPECT_NEAR(before, after, EPSILON);
}

TEST_P(ConservationTests, KineticEnergy) {
	// Test whether the kinetic energy decreases.
	const ParticleType& param = GetParam();
	if (param.particleFlags & b2_tensileParticle)
	{
		// Tensile particles have high potential energy at the initial placement
		// and may increase the kinetic energy.
		SUCCEED();
		return;
	}
	int32 particleCount = m_particleSystem->GetParticleCount();
	const b2Vec2 *velocityBuffer = m_particleSystem->GetVelocityBuffer();
	float64 before = 0;
	for (int32 i = 0; i < particleCount; i++) {
		b2Vec2 v = velocityBuffer[i];
		before += b2Dot(v, v) / 2;
	}
	for (int32 t = 0; t < NUMBER_OF_STEPS; t++) {
		m_world->Step(DELTA_T, 1, 1);
	}
	float64 after = 0;
	for (int32 i = 0; i < particleCount; i++) {
		b2Vec2 v = velocityBuffer[i];
		after += b2Dot(v, v) / 2;
	}
	ASSERT_GE(before, after);
}

int
main(int argc, char **argv)
{
		::testing::InitGoogleTest(&argc, argv);
		return RUN_ALL_TESTS();
}
