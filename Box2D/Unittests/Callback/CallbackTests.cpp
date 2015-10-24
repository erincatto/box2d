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
#include <vector>

#include "Box2D/Box2D.h"
#include "gtest/gtest.h"

#include "TestCommon.h"


using namespace std;

class CallbackTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
		// Define the gravity vector.
		b2Vec2 gravity(0.0f, -10.0f);

		// Construct a world object, which will hold and simulate the rigid
		// bodies.
		m_world = new b2World(gravity);

		b2ParticleSystemDef particleSystemDef;
		particleSystemDef.radius = 0.01f;
		m_particleSystem = m_world->CreateParticleSystem(&particleSystemDef);
	}

	virtual void TearDown()
	{
		// Clean up the world after each test.
		m_world->DestroyParticleSystem(m_particleSystem);
		delete m_world;
	}

	b2World *m_world;
	b2ParticleSystem *m_particleSystem;
};

// Query callback which counts the number of times ReportParticle() is called.
class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback()
	{
		m_count = 0;
		m_shouldQueryParticleSystem = true;
	}
	void ResetCount()
	{
		m_count = 0;
	}
	bool ReportFixture(b2Fixture* fixture)
	{
		B2_NOT_USED(fixture);
		return false;
	}
	bool ReportParticle(const b2ParticleSystem* particleSystem, int32 index)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
		m_count++;
		return true;
	}
	bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem)
	{
		B2_NOT_USED(particleSystem);
		return m_shouldQueryParticleSystem;
	}
	void SetShouldQueryParticleSystem(bool shouldQueryParticleSystem)
	{
		m_shouldQueryParticleSystem = shouldQueryParticleSystem;
	}

public:
	int32 m_count;
	bool m_shouldQueryParticleSystem;
};

// Verify that it's possible to determine the set of particles within an
// axis-aligned bounding box using the query callback.
TEST_F(CallbackTests, QueryCallback) {
	QueryCallback callback;
	b2AABB aabb;
	CreateBoxShapedParticleGroup(m_particleSystem);

	// This AABB query should miss all particles.
	aabb.lowerBound.Set(0.1f, -0.1f);
	aabb.upperBound.Set(0.2f, 0.1f);
	m_particleSystem->QueryAABB(&callback, aabb);
	EXPECT_EQ(callback.m_count, 0);

	// This AABB query should hit some particles.
	aabb.lowerBound.Set(-0.1f, -0.1f);
	aabb.upperBound.Set(0.1f, 0.1f);
	m_particleSystem->QueryAABB(&callback, aabb);
	EXPECT_NE(callback.m_count, 0);
}

// Verify that b2QueryCallback::SetShouldQueryParticleSystem is working,
// but only when QueryAABB is called through b2World.
TEST_F(CallbackTests, QueryCallback_ShouldQueryParticleSystem) {
	QueryCallback callback;
	b2AABB aabb;
	CreateBoxShapedParticleGroup(m_particleSystem);

	// This AABB query should still check the particle system,
	// because we're calling b2ParticleSystem::QueryAABB directly.
	aabb.lowerBound.Set(-0.1f, -0.1f);
	aabb.upperBound.Set(0.1f, 0.1f);
	callback.SetShouldQueryParticleSystem(false);
	m_particleSystem->QueryAABB(&callback, aabb);
	EXPECT_NE(callback.m_count, 0);

	// This AABB query shouldn't check the particle system at all,
	// because we're calling b2World::QueryAABB.
	callback.ResetCount();
	m_world->QueryAABB(&callback, aabb);
	EXPECT_EQ(callback.m_count, 0);
}

// Ray cast callback which counts the number of times ReportParticle() is
// called.
class RayCastCallback : public b2RayCastCallback
{
public:
	RayCastCallback()
	{
		m_count = 0;
		m_shouldQueryParticleSystem = true;
	}
	void ResetCount()
	{
		m_count = 0;
	}
	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
												const b2Vec2& normal, float32 fraction)
	{
		B2_NOT_USED(fixture);
		B2_NOT_USED(point);
		B2_NOT_USED(normal);
		B2_NOT_USED(fraction);
		return 0;
	}
	float32 ReportParticle(const b2ParticleSystem* particleSystem, int32 index,
							 const b2Vec2& point, const b2Vec2& normal,
							 float32 fraction)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
		B2_NOT_USED(point);
		B2_NOT_USED(normal);
		B2_NOT_USED(fraction);
		m_count++;
		return 0;
	}
	bool ShouldQueryParticleSystem(
		const b2ParticleSystem* particleSystem)
	{
		B2_NOT_USED(particleSystem);
		return m_shouldQueryParticleSystem;
	}
	void SetShouldQueryParticleSystem(bool shouldQueryParticleSystem)
	{
		m_shouldQueryParticleSystem = shouldQueryParticleSystem;
	}

public:
	int32 m_count;
	bool m_shouldQueryParticleSystem;
};

// Verify that it's possible to determine the set of particles intersected by
// a ray using a ray cast callback.
TEST_F(CallbackTests, RayCastCallback) {
	RayCastCallback callback;
	CreateBoxShapedParticleGroup(m_particleSystem);

	// This ray cast should miss all particles.
	m_world->RayCast(&callback, b2Vec2(0.21f, 0), b2Vec2(0, 0.21f));
	EXPECT_EQ(callback.m_count, 0);

	// This ray cast should hit a particle.
	m_world->RayCast(&callback, b2Vec2(-0.1f, -0.1f), b2Vec2(0.1f, 0.1f));
	EXPECT_NE(callback.m_count, 0);
}

// Verify that ShouldQueryParticleSystem disables ray casts against particle
// systems, but only when called through b2World::RayCast.
TEST_F(CallbackTests, RayCast_ShouldQueryParticleSystem) {
	RayCastCallback callback;
	CreateBoxShapedParticleGroup(m_particleSystem);

	// This ray cast should still hit a particle, since we're calling
	// b2ParticleSystem::RayCast directly.
	callback.SetShouldQueryParticleSystem(false);
	m_particleSystem->RayCast(
		&callback, b2Vec2(-0.1f, -0.1f), b2Vec2(0.1f, 0.1f));
	EXPECT_NE(callback.m_count, 0);

	// This ray cast shouldn't even cast against the particle system,
	// since we're calling b2World::RayCast.
	callback.ResetCount();
	m_world->RayCast(&callback, b2Vec2(-0.1f, -0.1f), b2Vec2(0.1f, 0.1f));
	EXPECT_EQ(callback.m_count, 0);
}

// Destruction listener which records references to particles or particle
// groups objects that have been destroyed.
class DestructionListener : public b2DestructionListener {
public:
	virtual ~DestructionListener() {}

	virtual void SayGoodbye(b2Joint* joint) { B2_NOT_USED(joint); }
	virtual void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }

	virtual void SayGoodbye(b2ParticleGroup* group) {
		m_destroyedParticleGroups.push_back(group);
	}

	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index) {
		B2_NOT_USED(particleSystem);
		m_destroyedParticles.push_back(index);
	}

public:
	vector<b2ParticleGroup*> m_destroyedParticleGroups;
	vector<int32> m_destroyedParticles;
};


// Ensure the destroy callback is not called when destroying a particle when
// the b2_destructionListener flag is not set.
TEST_F(CallbackTests, DestroyParticleWithNoCallback) {
	DestructionListener listener;
	m_world->SetDestructionListener(&listener);

	b2ParticleDef def;
	m_particleSystem->DestroyParticle(m_particleSystem->CreateParticle(def));
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 0U);

	CreateAndDestroyParticle(m_world, m_particleSystem, 0, false);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 0U);
}

// Test the particle destroy callback using the various methods to enable the
// functionality.
TEST_F(CallbackTests, DestroyParticleWithCallback) {
	int32 index;
	DestructionListener listenerNoFlags;
	m_world->SetDestructionListener(&listenerNoFlags);
	index = CreateAndDestroyParticle(m_world, m_particleSystem, 0, true);
	EXPECT_EQ(listenerNoFlags.m_destroyedParticles.size(), 1U);
	EXPECT_EQ(listenerNoFlags.m_destroyedParticles[0], index);

	DestructionListener listenerFlag;
	m_world->SetDestructionListener(&listenerFlag);
	index = CreateAndDestroyParticle(m_world, m_particleSystem,
																		 b2_destructionListenerParticle, false);
	EXPECT_EQ(listenerFlag.m_destroyedParticles.size(), 1U);
	EXPECT_EQ(listenerFlag.m_destroyedParticles[0], index);
}

// Test destroying particles in a shape without a callback on particle
// destruction.
TEST_F(CallbackTests, DestroyParticlesInShapeWithNoCallback) {
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	b2Transform xf;
	xf.SetIdentity();
	DestructionListener listener;
	m_world->SetDestructionListener(&listener);

	int32 destroyed;
	b2ParticleDef def;
	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 0U);

	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf, false);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 0U);
}

// Test destroying particles in a shape with a callback on particle
// destruction.
TEST_F(CallbackTests, DestroyParticlesInShapeWithCallback) {
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	b2Transform xf;
	xf.SetIdentity();
	DestructionListener listener;
	m_world->SetDestructionListener(&listener);

	int32 destroyed;
	b2ParticleDef def;
	int32 index = m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf, true);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 0U);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(listener.m_destroyedParticles.size(), 1U);
	EXPECT_EQ(listener.m_destroyedParticles[0], index);
}

// Verify the destruction callback is called when a particle group is
// destroyed.
TEST_F(CallbackTests, DestroyParticleGroupWithCallback) {
	DestructionListener listener;
	m_world->SetDestructionListener(&listener);
	b2ParticleGroup *group = CreateBoxShapedParticleGroup(m_particleSystem);
	group->DestroyParticles();
	EXPECT_EQ(listener.m_destroyedParticleGroups.size(), 0U);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(listener.m_destroyedParticleGroups.size(), 1U);
	EXPECT_EQ(listener.m_destroyedParticleGroups[0], group);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
