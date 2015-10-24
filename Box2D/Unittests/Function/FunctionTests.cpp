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
#include "Box2D/Box2D.h"
#include "gtest/gtest.h"

#include "TestCommon.h"
#include <map>
#include <vector>

class FunctionTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
		// Define the gravity vector.
		b2Vec2 gravity(0.0f, -10.0f);
		// Construct a world object, which will hold and simulate the rigid
		// bodies.
		m_world = new b2World(gravity);

		m_particleSystemDef.radius = 0.01f;
		m_particleSystemDef.gravityScale = 0.01f;
		m_particleSystem = m_world->CreateParticleSystem(&m_particleSystemDef);
	}
	virtual void TearDown()
	{
		// Clean up the world after each test.
		m_world->DestroyParticleSystem(m_particleSystem);
		delete m_world;
	}

protected:
	b2World *m_world;
	b2ParticleSystem *m_particleSystem;
	b2ParticleSystemDef m_particleSystemDef;
};

// Class which registers itself as a destruction listener on construction and
// unregisters itself on destruction.
class ScopedDestructionListener : public b2DestructionListener
{
public:
	// Set this instance as a destruction listener of world.
	ScopedDestructionListener(b2World* const world,
								b2ParticleSystem* const particleSystem) :
		m_world(world),
		m_particleSystem(particleSystem)
	{
		b2Assert(m_world);
		b2Assert(m_particleSystem);
		m_world->SetDestructionListener(this);
	}

	// Stop this instance acting as the world's destruction listener.
	virtual ~ScopedDestructionListener()
	{
		m_world->SetDestructionListener(NULL);
	}

	// Ignore destruction of joints.
	virtual void SayGoodbye(b2Joint* joint) { B2_NOT_USED(joint); }

	// Ignore destruction of fixtures.
	virtual void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }

	// Ignore destruction of particle groups.
	virtual void SayGoodbye(b2ParticleGroup* group) { B2_NOT_USED(group); }

	// Verify destruction of particles.
	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(index);
	}

protected:
	b2World *m_world;
	b2ParticleSystem *m_particleSystem;
};

// Class which tracks the user data associated with particles that are
// destroyed in a vector.
class UserDataDestructionTracker : public ScopedDestructionListener
{
public:
	// Set this instance as a destruction listener of world.
	UserDataDestructionTracker(b2World* const world,
								 b2ParticleSystem* const particleSystem) :
		ScopedDestructionListener(world, particleSystem)
	{

	}

	// Stop this instance acting as the world's destruction listener.
	virtual ~UserDataDestructionTracker() { }

	// Add the user data of the destroyed particle in a vector.
	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
	{
		if (particleSystem != m_particleSystem)
			return;

		m_userData.push_back(
			m_particleSystem->GetUserDataBuffer()[index]);
	}

	// Get the vector of destroyed particle user data.
	const std::vector<void*>& GetUserData() const {
		return m_userData;
	}

protected:
	std::vector<void*> m_userData;
};

// Class which verifies a set of particles are destroyed when they reach
// the end of their lifetime.
class LifetimeDestructionChecker : public ScopedDestructionListener
{
public:
	// Set this instance as a destruction listener of world.
	LifetimeDestructionChecker(
		float32 destructionTimeEpsilon, b2World* const world,
		b2ParticleSystem* const particleSystem) :
		ScopedDestructionListener(world, particleSystem),
		m_timeElapsed(0.0f),
		m_destructionTimeEpsilon(destructionTimeEpsilon)
	{
		b2Assert(m_world);
		b2Assert(m_particleSystem);
		m_world->SetDestructionListener(this);
	}

	// Stop this instance acting as the world's destruction listener.
	virtual ~LifetimeDestructionChecker() { }

	// Verify the destroyed particle was destroyed in time.
	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
	{
		if (particleSystem != m_particleSystem)
			return;

		void *handle =
			reinterpret_cast<void*>(
				m_particleSystem->GetUserDataBuffer()[index]);
		// Make sure this particle is being tracked.
		std::map<void*, float>::iterator it =
			m_particleLifetime.find(handle);
		ASSERT_TRUE(it != m_particleLifetime.end());

		// Make sure the particle was destroyed on time.
		const float32 lifetime = it->second;
		EXPECT_NEAR(lifetime, m_timeElapsed, m_destructionTimeEpsilon);
		m_particleLifetime.erase(it);
	}

	// Track particles with finite lifetimes.
	void TrackParticleLifetimes(const int32* const particleIndices,
								const int32 numberOfParticles)
	{
		b2Assert(particleIndices);
		for (int32 i = 0; i < numberOfParticles; ++i)
		{
			// Casting away from const is ok here since the pointer will
			// never be dereferenced.
			void* const handle = const_cast<int32*>(&particleIndices[i]);
			const int32 index = particleIndices[i];
			m_particleSystem->SetParticleFlags(
				index, m_particleSystem->GetParticleFlags(index) |
						 b2_destructionListenerParticle);
			m_particleSystem->GetUserDataBuffer()[index] = handle;
			m_particleLifetime[handle] =
				m_particleSystem->GetParticleLifetime(index);
		}
	}

	// Update the time elapsed.
	void Step(const float32 deltaT)
	{
		m_timeElapsed += deltaT;
	}

	// Get how much time has elapsed.
	float32 GetTimeElapsed() const { return m_timeElapsed; }

private:
	float32 m_timeElapsed;
	float32 m_destructionTimeEpsilon;
	// Map of psuedo handle to the particle stored in userdata and the
	// expected lifetime of the particle.
	std::map<void*, float32> m_particleLifetime;
};

// Class which verifies the oldest particle is always destroyed.
// For this class to check particles, they must be created with the
// b2_destructionListener flag set.
class OldestParticleDestroyedChecker : public ScopedDestructionListener
{
public:
	// Set this instance as a destruction listener of the world.
	OldestParticleDestroyedChecker(b2World* const world,
									 b2ParticleSystem* const particleSystem) :
		ScopedDestructionListener(world, particleSystem) { }

	// Stop this instance acting as the world's destruction listener.
	virtual ~OldestParticleDestroyedChecker() { }

	// Verify the destroyed particle is the oldest particle in the system.
	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
	{
		if (particleSystem != m_particleSystem)
			return;

		const int32 numberOfParticles =
			m_particleSystem->GetParticleCount();
		const int32* const expirationTimes =
			m_particleSystem->GetExpirationTimeBuffer();
		ASSERT_TRUE(expirationTimes != NULL);

		int32 oldestFiniteExpirationTimeIndex = b2_invalidParticleIndex;
		int32 oldestFiniteExpirationTime = (int32)1ULL << 31;
		int32 oldestInfiniteExpirationTimeIndex = b2_invalidParticleIndex;
		int32 oldestInfiniteExpirationTime = 0;
		for (int32 i = 0; i < numberOfParticles; ++i)
		{
			const int32 expirationTime = expirationTimes[i];
			if (expirationTime > 0)
			{
				if (expirationTime < oldestFiniteExpirationTime)
				{
					oldestFiniteExpirationTime = expirationTime;
					oldestFiniteExpirationTimeIndex = i;
				}
			}
			else
			{
				if (expirationTime > oldestInfiniteExpirationTime)
				{
					oldestInfiniteExpirationTime = expirationTime;
					oldestInfiniteExpirationTimeIndex = i;
				}
			}
		}
		if (oldestFiniteExpirationTimeIndex != b2_invalidParticleIndex)
		{
			EXPECT_EQ(oldestFiniteExpirationTimeIndex, index) <<
				"oldest finite expiration time " << oldestFiniteExpirationTime;
		}
		else if (oldestInfiniteExpirationTimeIndex != b2_invalidParticleIndex)
		{
			EXPECT_EQ(oldestInfiniteExpirationTimeIndex, index) <<
				"oldest infinite expiration time " <<
					oldestInfiniteExpirationTime;
		}
	}
};

TEST_F(FunctionTests, CreateParticle) {
	b2ParticleDef def;
	def.flags = b2_elasticParticle | b2_springParticle;
	def.position.Set(1, 2);
	def.velocity.Set(3, 4);
	def.color.Set(1, 2, 3, 4);
	def.userData = this;
	int index = m_particleSystem->CreateParticle(def);
	EXPECT_EQ(index, 0);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 1);
	EXPECT_EQ(m_particleSystem->GetFlagsBuffer()[index], def.flags);
	EXPECT_EQ(m_particleSystem->GetPositionBuffer()[index], def.position);
	EXPECT_EQ(m_particleSystem->GetVelocityBuffer()[index], def.velocity);
	EXPECT_EQ(m_particleSystem->GetColorBuffer()[index].r, def.color.r);
	EXPECT_EQ(m_particleSystem->GetColorBuffer()[index].g, def.color.g);
	EXPECT_EQ(m_particleSystem->GetColorBuffer()[index].b, def.color.b);
	EXPECT_EQ(m_particleSystem->GetColorBuffer()[index].a, def.color.a);
	EXPECT_EQ(m_particleSystem->GetUserDataBuffer()[index], def.userData);
}

TEST_F(FunctionTests, CreateParticleInExistingGroup) {
	b2ParticleGroupDef groupDef;
	b2ParticleGroup *groupA = m_particleSystem->CreateParticleGroup(groupDef);
	b2ParticleGroup *groupB = m_particleSystem->CreateParticleGroup(groupDef);
	b2ParticleDef def;
	for (int i = 0; i < 20; i++)
	{
		if (i % 2)
		{
			def.position.Set(1, 0);
			def.group = groupA;
		}
		else
		{
			def.position.Set(2, 0);
			def.group = groupB;
		}
		m_particleSystem->CreateParticle(def);
	}
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 20);
	EXPECT_EQ(groupA->GetParticleCount(), 10);
	EXPECT_EQ(groupB->GetParticleCount(), 10);
	b2Vec2 *positionBuffer = m_particleSystem->GetPositionBuffer();
	b2ParticleGroup *const *groupBuffer = m_particleSystem->GetGroupBuffer();
	for (int i = 0; i < 10; i++)
	{
		int a = groupA->GetBufferIndex() + i;
		int b = groupB->GetBufferIndex() + i;
		EXPECT_EQ((int)positionBuffer[a].x, 1);
		EXPECT_EQ((int)positionBuffer[b].x, 2);
		EXPECT_EQ(groupBuffer[a], groupA);
		EXPECT_EQ(groupBuffer[b], groupB);
	}
}

TEST_F(FunctionTests, ParticleRadius) {
	float r = 0.123f;
	m_particleSystem->SetRadius(r);
	EXPECT_EQ(m_particleSystem->GetRadius(), r);
}

TEST_F(FunctionTests, ParticleDensity) {
	float r = 12.3f;
	m_particleSystem->SetDensity(r);
	EXPECT_EQ(m_particleSystem->GetDensity(), r);
}

TEST_F(FunctionTests, ParticleGravityScale) {
	float g = 1.23f;
	m_particleSystem->SetGravityScale(g);
	EXPECT_EQ(m_particleSystem->GetGravityScale(), g);
}

TEST_F(FunctionTests, ParticleDamping) {
	float r = 0.123f;
	m_particleSystem->SetDamping(r);
	EXPECT_EQ(m_particleSystem->GetDamping(), r);
}

TEST_F(FunctionTests, ParticleStaticPressureItearations) {
	int n = 123;
	m_particleSystem->SetStaticPressureIterations(n);
	EXPECT_EQ(m_particleSystem->GetStaticPressureIterations(), n);
}

// Verify that it's possible to destroy a particle using
// DestroyParticle(int32) and DestroyParticle(int32, bool).
TEST_F(FunctionTests, DestroyParticle) {
	b2ParticleDef def;
	int index = m_particleSystem->CreateParticle(def);
	m_particleSystem->DestroyParticle(index);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);

	CreateAndDestroyParticle(m_world, m_particleSystem, 0, true);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);

	CreateAndDestroyParticle(m_world, m_particleSystem, 0, false);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);
}

// Attempt and fail to destroy a particle with a shape that is outside the
// perimeter of the shape.
TEST_F(FunctionTests, DestroyParticlesInShapeNoneInShape) {
	b2PolygonShape shape;
	shape.SetAsBox(0.02f, 0.02f);
	b2Transform xf;
	xf.SetIdentity();

	b2ParticleDef def;
	def.position.x = 3;
	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	int destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf);
	EXPECT_EQ(destroyed, 0);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 1);
}

// Destroy a particle within a shape using
// DestroyParticlesInShape(b2Shape&, b2Transform&) and
// DestroyParticlesInShape(b2Shape&, b2Transform&, bool).
TEST_F(FunctionTests, DestroyParticlesInShape) {
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	b2Transform xf;
	xf.SetIdentity();
	int32 destroyed;
	b2ParticleDef def;

	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 1);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);

	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf, true);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 1);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);

	m_particleSystem->CreateParticle(def);
	m_world->Step(0.001f, 1, 1);
	destroyed = m_particleSystem->DestroyParticlesInShape(shape, xf, false);
	EXPECT_EQ(destroyed, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 1);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);
}

TEST_F(FunctionTests, CreateParticleGroup) {
	b2ParticleGroup *group = CreateBoxShapedParticleGroup(m_particleSystem);
	EXPECT_NE(group, (b2ParticleGroup *)NULL);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 1);
	EXPECT_NE(m_particleSystem->GetParticleCount(), 0);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), group->GetParticleCount());
}

TEST_F(FunctionTests, CreateParticleGroupWithShapeList) {
	b2ParticleGroupDef def;
	static const int32 shapeCount = 10;
	b2CircleShape circleShapes[shapeCount];
	const b2Shape *shapes[shapeCount];
	for (int32 i = 0; i < shapeCount; i++)
	{
		circleShapes[i].m_p.Set(0.02f * (float32)i, 0.1f);
		circleShapes[i].m_radius = 0.01f;
		shapes[i] = &circleShapes[i];
	}
	def.shapes = shapes;
	def.shapeCount = 1;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	EXPECT_GT(group1->GetParticleCount(), 0);
	def.shapeCount = shapeCount;
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	EXPECT_GT(group2->GetParticleCount(), group1->GetParticleCount());
}

TEST_F(FunctionTests, CreateParticleGroupWithCustomStride) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	def.stride = 1 * m_particleSystem->GetRadius();
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	def.stride = 2 * m_particleSystem->GetRadius();
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	def.stride = 3 * m_particleSystem->GetRadius();
	b2ParticleGroup *group3 = m_particleSystem->CreateParticleGroup(def);
	EXPECT_GT(group1->GetParticleCount(), group2->GetParticleCount());
	EXPECT_GT(group2->GetParticleCount(), group3->GetParticleCount());
}

TEST_F(FunctionTests, CreateEmptyParticleGroupWithNoShape) {
	b2ParticleGroupDef def;
	def.groupFlags = b2_solidParticleGroup | b2_particleGroupCanBeEmpty;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);
	EXPECT_EQ(group->GetParticleCount(), 0);
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->JoinParticleGroups(group, m_particleSystem->CreateParticleGroup(def));
	EXPECT_GT(m_particleSystem->GetParticleCount(), 0);
	EXPECT_GT(group->GetParticleCount(), 0);
	m_world->Step(0.01f, 1, 1, 1);
	group->DestroyParticles(true);
	m_world->Step(0.01f, 1, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);
	EXPECT_EQ(group->GetParticleCount(), 0);
}

TEST_F(FunctionTests, CreateParticleGroupWithParticleCount) {
	static const int32 particleCount = 100;
	b2Vec2 positionData[particleCount];
	for (int32 i = 0; i < particleCount; i++)
	{
		positionData[i].Set(0.01f * (float32)i, 0.01f * (float32)i);
	}
	b2ParticleGroupDef def;
	def.particleCount = particleCount;
	def.positionData = positionData;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), particleCount);
	EXPECT_EQ(group->GetParticleCount(), particleCount);
	const b2Vec2 *positionBuffer = m_particleSystem->GetPositionBuffer();
	for (int32 i = 0; i < particleCount; i++)
	{
		ASSERT_EQ(positionBuffer[i].x, 0.01f * (float32)i);
		ASSERT_EQ(positionBuffer[i].y, 0.01f * (float32)i);
	}
}

TEST_F(FunctionTests, CreateParticleGroupInExistingGroup) {
	b2ParticleGroupDef groupDef;
	b2ParticleGroup *groupA = m_particleSystem->CreateParticleGroup(groupDef);
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	groupDef.shape = &shape;
	groupDef.group = groupA;
	b2ParticleGroup *groupB = m_particleSystem->CreateParticleGroup(groupDef);
	groupDef.shape = NULL;
	b2ParticleGroup *groupC = m_particleSystem->CreateParticleGroup(groupDef);
	EXPECT_EQ(groupA, groupB);
	EXPECT_EQ(groupA, groupC);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 1);
}

TEST_F(FunctionTests, DestroyParticleGroup) {
	b2ParticleGroup *group = CreateBoxShapedParticleGroup(m_particleSystem);
	group->DestroyParticles();
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 1);
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 0);
	EXPECT_EQ(m_particleSystem->GetParticleCount(), 0);
}

TEST_F(FunctionTests, GetParticleBuffer) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(group->GetBufferIndex(), 0);
	const b2ParticleSystem *constParticleSystem = m_particleSystem;
	EXPECT_EQ(m_particleSystem->GetFlagsBuffer(),
				constParticleSystem->GetFlagsBuffer());
	EXPECT_EQ(m_particleSystem->GetPositionBuffer(),
				constParticleSystem->GetPositionBuffer());
	EXPECT_EQ(m_particleSystem->GetVelocityBuffer(),
				constParticleSystem->GetVelocityBuffer());
	EXPECT_EQ(m_particleSystem->GetGroupBuffer(),
				constParticleSystem->GetGroupBuffer());
	EXPECT_EQ(m_particleSystem->GetColorBuffer(),
				constParticleSystem->GetColorBuffer());
	EXPECT_EQ(m_particleSystem->GetWeightBuffer(),
				constParticleSystem->GetWeightBuffer());
	EXPECT_EQ(m_particleSystem->GetUserDataBuffer(),
				constParticleSystem->GetUserDataBuffer());
	const b2ParticleGroup *constGroup = group;
	EXPECT_EQ(group->GetBufferIndex(), constGroup->GetBufferIndex());
}

TEST_F(FunctionTests, SetParticleBuffer) {
	static const int32 size = 256;
	uint32 flagsBuffer[size];
	b2Vec2 positionBuffer[size];
	b2Vec2 velocityBuffer[size];
	b2ParticleColor colorBuffer[size];
	void *userDataBuffer[size];
	m_particleSystem->SetFlagsBuffer(flagsBuffer, size);
	m_particleSystem->SetPositionBuffer(positionBuffer, size);
	m_particleSystem->SetVelocityBuffer(velocityBuffer, size);
	m_particleSystem->SetColorBuffer(colorBuffer, size);
	m_particleSystem->SetUserDataBuffer(userDataBuffer, size);
	EXPECT_EQ(m_particleSystem->GetFlagsBuffer(), flagsBuffer);
	EXPECT_EQ(m_particleSystem->GetPositionBuffer(), positionBuffer);
	EXPECT_EQ(m_particleSystem->GetVelocityBuffer(), velocityBuffer);
	EXPECT_EQ(m_particleSystem->GetColorBuffer(), colorBuffer);
	EXPECT_EQ(m_particleSystem->GetUserDataBuffer(), userDataBuffer);
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	EXPECT_LE(m_particleSystem->GetParticleCount(), size);
	EXPECT_EQ(m_particleSystem->GetFlagsBuffer(), flagsBuffer);
	EXPECT_EQ(m_particleSystem->GetPositionBuffer(), positionBuffer);
	EXPECT_EQ(m_particleSystem->GetVelocityBuffer(), velocityBuffer);
	EXPECT_EQ(m_particleSystem->GetColorBuffer(), colorBuffer);
	EXPECT_EQ(m_particleSystem->GetUserDataBuffer(), userDataBuffer);
	uint32 newFlagsBuffer[size];
	b2Vec2 newPositionBuffer[size];
	b2Vec2 newVelocityBuffer[size];
	b2ParticleColor newColorBuffer[size];
	void *newUserDataBuffer[size];
	m_particleSystem->SetFlagsBuffer(newFlagsBuffer, size);
	m_particleSystem->SetPositionBuffer(newPositionBuffer, size);
	m_particleSystem->SetVelocityBuffer(newVelocityBuffer, size);
	m_particleSystem->SetColorBuffer(newColorBuffer, size);
	m_particleSystem->SetUserDataBuffer(newUserDataBuffer, size);
	EXPECT_EQ(m_particleSystem->GetFlagsBuffer(), newFlagsBuffer);
	EXPECT_EQ(m_particleSystem->GetPositionBuffer(), newPositionBuffer);
	EXPECT_EQ(m_particleSystem->GetVelocityBuffer(), newVelocityBuffer);
	EXPECT_EQ(m_particleSystem->GetColorBuffer(), newColorBuffer);
	EXPECT_EQ(m_particleSystem->GetUserDataBuffer(), newUserDataBuffer);
}

TEST_F(FunctionTests, GroupData) {
	b2ParticleGroupDef def;
	def.flags = b2_elasticParticle | b2_springParticle;
	def.groupFlags = b2_particleGroupCanBeEmpty;
	def.position.Set(0.01f, 0.02f);
	def.angle = 3;
	def.linearVelocity.Set(0.04f, 0.05f);
	def.color.Set(1, 2, 3, 4);
	def.userData = this;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(group->GetAllParticleFlags(), def.flags);
	EXPECT_EQ(group->GetGroupFlags(), def.groupFlags);
	EXPECT_NE(group->GetParticleCount(), 0);
	EXPECT_EQ(group->GetPosition(), def.position);
	EXPECT_EQ(group->GetAngle(), def.angle);
	EXPECT_LE(b2Distance(group->GetLinearVelocity(), def.linearVelocity),
				EPSILON);
	EXPECT_EQ(group->GetUserData(), def.userData);
	group->SetUserData(NULL);
	EXPECT_EQ(group->GetUserData(), (void*)NULL);
}

TEST_F(FunctionTests, GroupList) {
	b2ParticleGroupDef def;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroup *group3 = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroup *list = m_particleSystem->GetParticleGroupList();
	EXPECT_EQ(list, group3);
	list = list->GetNext();
	EXPECT_EQ(list, group2);
	list = list->GetNext();
	EXPECT_EQ(list, group1);
	list = list->GetNext();
	EXPECT_EQ(list, (b2ParticleGroup *)0);
}

TEST_F(FunctionTests, ConstGroupList) {
	b2ParticleGroupDef def;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroup *group3 = m_particleSystem->CreateParticleGroup(def);
	const b2ParticleGroup *list =
		((const b2ParticleSystem *)m_particleSystem)->GetParticleGroupList();
	EXPECT_EQ(list, group3);
	list = list->GetNext();
	EXPECT_EQ(list, group2);
	list = list->GetNext();
	EXPECT_EQ(list, group1);
	list = list->GetNext();
	EXPECT_EQ(list, (const b2ParticleGroup *)NULL);
}

TEST_F(FunctionTests, JoinParticleGroups) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	shape.SetAsBox(0.1f, 0.2f);
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	shape.SetAsBox(0.1f, 0.3f);
	b2ParticleGroup *group3 = m_particleSystem->CreateParticleGroup(def);
	int32 count1 = group1->GetParticleCount();
	int32 count2 = group2->GetParticleCount();
	int32 count3 = group3->GetParticleCount();
	EXPECT_EQ(count1 + count2 + count3, m_particleSystem->GetParticleCount());
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 3);
	m_particleSystem->JoinParticleGroups(group1, group2);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 2);
	EXPECT_EQ(count1 + count2, group1->GetParticleCount());
}

TEST_F(FunctionTests, SplitParticleGroup) {
	b2ParticleGroupDef def;
	static const int32 shapeCount = 3;
	b2CircleShape circleShapes[shapeCount];
	const b2Shape *shapes[shapeCount];
	for (int32 i = 0; i < shapeCount; i++)
	{
		circleShapes[i].m_p.Set(0.2f * (float32)i, 0);
		circleShapes[i].m_radius = 0.08f;
		shapes[i] = &circleShapes[i];
	}
	def.shapes = shapes;
	def.shapeCount = shapeCount;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 1);
	m_particleSystem->SplitParticleGroup(group);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), shapeCount);
}

TEST_F(FunctionTests, SplitParticleGroupInterminglingWithOtherGroups) {
	b2ParticleGroupDef def;
	static const int32 shapeCount = 3;
	b2CircleShape circleShapes[shapeCount];
	const b2Shape *shapes[shapeCount];
	for (int32 i = 0; i < shapeCount; i++)
	{
		circleShapes[i].m_p.Set(0.2f * (float32)i, 0);
		circleShapes[i].m_radius = 0.08f;
		shapes[i] = &circleShapes[i];
	}
	def.shapes = shapes;
	def.shapeCount = shapeCount;
	b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(def);
	b2ParticleGroupDef anotherDef;
	b2PolygonShape polygonShape;
	polygonShape.SetAsBox(0.3f, 0.1f, b2Vec2(0.25f, 0), 0);
	anotherDef.shape = &polygonShape;
	m_particleSystem->CreateParticleGroup(anotherDef);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), 2);
	m_particleSystem->SplitParticleGroup(group);
	EXPECT_EQ(m_particleSystem->GetParticleGroupCount(), shapeCount + 1);
}

TEST_F(FunctionTests, GroupBuffer) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	shape.SetAsBox(0.1f, 0.2f);
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	shape.SetAsBox(0.1f, 0.3f);
	b2ParticleGroup *group3 = m_particleSystem->CreateParticleGroup(def);
	const b2ParticleGroup *const *groupBuffer =
		m_particleSystem->GetGroupBuffer();
	int32 offset1 = group1->GetBufferIndex();
	int32 count1 = group1->GetParticleCount();
	for (int32 i = 0; i < count1; i++) {
		ASSERT_EQ(group1, groupBuffer[offset1 + i]);
	}
	int32 offset2 = group2->GetBufferIndex();
	int32 count2 = group2->GetParticleCount();
	for (int32 i = 0; i < count2; i++) {
		ASSERT_EQ(group2, groupBuffer[offset2 + i]);
	}
	int32 offset3 = group3->GetBufferIndex();
	int32 count3 = group3->GetParticleCount();
	for (int32 i = 0; i < count3; i++) {
		ASSERT_EQ(group3, groupBuffer[offset3 + i]);
	}
	m_particleSystem->JoinParticleGroups(group1, group2);
	group3->DestroyParticles();
	m_world->Step(0.001f, 1, 1);
	groupBuffer = m_particleSystem->GetGroupBuffer();
	int32 count = m_particleSystem->GetParticleCount();
	for (int32 i = 0; i < count; i++) {
		ASSERT_EQ(group1, groupBuffer[i]);
	}
}

TEST_F(FunctionTests, AllFlags) {
	b2ParticleGroup* group = CreateBoxShapedParticleGroup(m_particleSystem);
	EXPECT_EQ(m_particleSystem->GetAllParticleFlags(), 0u);
	EXPECT_EQ(m_particleSystem->GetAllGroupFlags(), 0u);
	int32 particleCount = m_particleSystem->GetParticleCount();
	for (int32 i = 0; i < particleCount / 2; i++) {
		m_particleSystem->SetParticleFlags(i, b2_elasticParticle);
	}
	for (int32 i = particleCount / 2; i < particleCount; i++) {
		m_particleSystem->SetParticleFlags(i, b2_viscousParticle);
	}
	EXPECT_EQ(m_particleSystem->GetAllParticleFlags(),
									(uint32)(b2_elasticParticle | b2_viscousParticle));
	group->SetGroupFlags(b2_particleGroupCanBeEmpty);
	EXPECT_EQ(m_particleSystem->GetAllGroupFlags(), b2_particleGroupCanBeEmpty);
}

TEST_F(FunctionTests, GroupFlags) {
	b2ParticleGroup* group = CreateBoxShapedParticleGroup(m_particleSystem);
	group->SetGroupFlags(b2_rigidParticleGroup);
	EXPECT_EQ(group->GetGroupFlags(), b2_rigidParticleGroup);
	group->SetGroupFlags(b2_rigidParticleGroup | b2_solidParticleGroup);
	EXPECT_EQ(
		group->GetGroupFlags(),
								(uint32)(b2_rigidParticleGroup | b2_solidParticleGroup));
	group->SetGroupFlags(b2_particleGroupCanBeEmpty);
	EXPECT_EQ(group->GetGroupFlags(), b2_particleGroupCanBeEmpty);
	group->SetGroupFlags(0);
	EXPECT_EQ(group->GetGroupFlags(), 0u);
}

TEST_F(FunctionTests, GetParticleContact) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	EXPECT_NE(m_particleSystem->GetContactCount(), 0);
	EXPECT_NE(m_particleSystem->GetContacts(), (const b2ParticleContact *)NULL);
}

TEST_F(FunctionTests, GetParticleBodyContact) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	b2BodyDef bodyDef;
	b2Body *body = m_world->CreateBody(&bodyDef);
	body->CreateFixture(&shape, 1.0);
	m_world->Step(0.001f, 1, 1);
	EXPECT_NE(m_particleSystem->GetBodyContactCount(), 0);
	EXPECT_NE(m_particleSystem->GetBodyContacts(),
				(const b2ParticleBodyContact *)NULL);
}

TEST_F(FunctionTests, GetParticlePair) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->GetPairCount(), 0);
	def.flags = b2_springParticle;
	def.position.Set(0.2f, 0);
	b2ParticleGroup* group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_NE(m_particleSystem->GetPairCount(), 0);
	group->DestroyParticles();
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetPairCount(), 0);
	EXPECT_NE(m_particleSystem->GetPairs(), (const b2ParticlePair *)NULL);
}

TEST_F(FunctionTests, GetParticleTriad) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->GetTriadCount(), 0);
	def.flags = b2_elasticParticle;
	def.position.Set(0.2f, 0);
	b2ParticleGroup* group = m_particleSystem->CreateParticleGroup(def);
	EXPECT_NE(m_particleSystem->GetTriadCount(), 0);
	group->DestroyParticles();
	m_world->Step(0.001f, 1, 1);
	EXPECT_EQ(m_particleSystem->GetTriadCount(), 0);
	EXPECT_NE(m_particleSystem->GetTriads(), (const b2ParticleTriad *)NULL);
}

TEST_F(FunctionTests, ComputeCollisionEnergy) {
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	m_particleSystem->CreateParticleGroup(def);
	EXPECT_EQ(m_particleSystem->ComputeCollisionEnergy(), 0);

	def.position.Set(0.2f, 0);
	def.linearVelocity.Set(-0.01f, 0);
	m_particleSystem->CreateParticleGroup(def);
	for (int32 t = 0; t < 10; t++)
	{
		m_world->Step(0.1f, 1, 1, 1);
	}
	EXPECT_NE(m_particleSystem->ComputeCollisionEnergy(), 0);
}

TEST_F(FunctionTests, SetPaused) {
	EXPECT_FALSE(m_particleSystem->GetPaused());

	b2ParticleDef def;
	def.flags = b2_elasticParticle | b2_springParticle;
	def.position.Set(0.01f, 0.02f);
	def.velocity.Set(0.03f, 0.04f);
	def.color.Set(1, 2, 3, 4);
	def.userData = this;
	m_particleSystem->CreateParticle(def);

	// Step the simulation to ensure the particle is moving.
	const b2Vec2* positions = m_particleSystem->GetPositionBuffer();
	const b2Vec2 initialPosition = positions[0];
	m_world->Step(0.1f, 1, 1);
	const b2Vec2 steppedPosition = positions[0];
	EXPECT_NE(initialPosition, steppedPosition);

	// Pause the particle system, then step simulation again.
	// Ensure the particle hasn't moved.
	m_particleSystem->SetPaused(true);
	EXPECT_TRUE(m_particleSystem->GetPaused());
	m_world->Step(0.1f, 1, 1);
	EXPECT_EQ(steppedPosition, positions[0]);

	// Unpause the particle system. Ensure the particle is moving again.
	m_particleSystem->SetPaused(false);
	EXPECT_FALSE(m_particleSystem->GetPaused());
	m_world->Step(0.1f, 1, 1);
	EXPECT_NE(steppedPosition, positions[0]);
}

// Verify that it's possible to retrieve a handle from a particle index.
TEST_F(FunctionTests, GetParticleHandleFromIndex)
{
	b2ParticleSystem *system = m_world->GetParticleSystemList();
	const b2ParticleDef particleDef;
	const int32 particleIndex = system->CreateParticle(particleDef);
	ASSERT_NE(particleIndex, b2_invalidParticleIndex);
	const b2ParticleHandle* particleHandle =
		system->GetParticleHandleFromIndex(particleIndex);
	EXPECT_EQ(particleIndex, particleHandle->GetIndex());
}

// Ensure particle handles track particles correctly as the array of particles
// is compacted when particles are destroyed.
TEST_F(FunctionTests, ParticleHandleTrackCompactParticles)
{
	static const int32 kNumberOfHandles = 1024;
	b2TrackedBlockAllocator allocator;
	const b2ParticleHandle** const handles =
		(const b2ParticleHandle**)allocator.Allocate(
			kNumberOfHandles * sizeof(*handles));
	int32* const expectedUserData = (int32*)allocator.Allocate(
			kNumberOfHandles * sizeof(*expectedUserData));
	b2ParticleSystem *system = m_world->GetParticleSystemList();

	// Create particles and store a handle to each particle.  Also, associate
	// each particle with user data that contains the index into the array of
	// handles.
	b2ParticleDef particleDef;
	for (int32 i = 0; i < kNumberOfHandles; ++i)
	{
		handles[i] = system->GetParticleHandleFromIndex(
			system->CreateParticle(particleDef));
		EXPECT_TRUE(handles[i] != NULL);
		const int32 particleIndex = handles[i]->GetIndex();
		EXPECT_NE(particleIndex, b2_invalidParticleIndex);
		// NOTE: The user data buffer is retrieved each time since it's
		// possible for the particle system to reallocate it when particles are
		// created.
		system->GetUserDataBuffer()[particleIndex] =
			&expectedUserData[i];
		expectedUserData[i] = i;
	}

	// Update the particle system state.
	m_world->Step(0.001f, 1, 1);

	// Verify that it's possible to retrieve each particle's user data from a
	// handle.
	for (int32 i = 0; i < kNumberOfHandles; ++i)
	{
		const int32 particleIndex = handles[i]->GetIndex();
		EXPECT_EQ(*(((int32**)system->GetUserDataBuffer())[
						particleIndex]), i);
	}

	// Mark half of the particles for deletion and clear the user data
	// associated with each of the handles that is marked for deletion.
	for (int32 i = 0; i < kNumberOfHandles; i += 2)
	{
		const int32 particleIndex = handles[i]->GetIndex();
		system->DestroyParticle(particleIndex, false);
		expectedUserData[i] = -1;
	}

	// Delete the particles.
	// Since every other particle has been deleted this will result in
	// the compaction of particle buffers changing the index of each particle.
	m_world->Step(0.001f, 1, 1);

	// Verify user data referenced by remaining particles is correct.
	for (int32 i = 1; i < kNumberOfHandles; i += 2)
	{
		const int32 particleIndex = handles[i]->GetIndex();
		EXPECT_EQ(*(((int32**)system->GetUserDataBuffer())[
						particleIndex]), i);
	}
}

// Ensure particle handles track particles correctly as the array of particles
// is compacted when particles are destroyed.
TEST_F(FunctionTests, ParticleHandlesTrackGroups)
{
	b2TrackedBlockAllocator allocator;
	b2ParticleSystem *system = m_world->GetParticleSystemList();
	// Create particle groups, join them, check data after resorting.
	b2ParticleGroupDef groupDef;
	b2PolygonShape box;
	box.SetAsBox(0.1f, 0.1f);
	groupDef.shape = &box;
	b2ParticleGroup *groupA = system->CreateParticleGroup(groupDef);
	// Create a particle groupB next to groupA.
	groupDef.position = b2Vec2(0.1f, 0.0f);
	b2ParticleGroup *groupB = system->CreateParticleGroup(groupDef);

	const int32 groupAParticleCount = groupA->GetParticleCount();
	const int32 numberOfGroupParticles = groupAParticleCount +
		groupB->GetParticleCount();
	const b2ParticleHandle **particleHandles =
		(const b2ParticleHandle**)allocator.Allocate(
			numberOfGroupParticles * sizeof(*particleHandles));
	int32* const expectedGroupData = (int32*)allocator.Allocate(
			numberOfGroupParticles * sizeof(*expectedGroupData));

	// Get a handle for each particle in both groups and assign expected
	// user data to each particle.
	for (int32 i = 0; i < numberOfGroupParticles; ++i)
	{
		b2ParticleGroup* group;
		int32 particleIndex;
		if (i < groupAParticleCount)
		{
			group = groupA;
			particleIndex = i;
		}
		else
		{
			group = groupB;
			particleIndex = i - groupAParticleCount;
		}
		particleIndex += group->GetBufferIndex();

		particleHandles[i] = system->GetParticleHandleFromIndex(
			particleIndex);
		expectedGroupData[i] = i;
		system->GetUserDataBuffer()[particleIndex] =
			&expectedGroupData[i];
	}

	m_world->Step(0.001f, 1, 1);

	// Join the particle groups.
	system->JoinParticleGroups(groupA, groupB);
	groupB = NULL;

	m_world->Step(0.001f, 1, 1);

	// Verify the handles still point at valid data by checking the value
	// associated with each particle's user data.
	ASSERT_EQ(numberOfGroupParticles, groupA->GetParticleCount());
	for (int32 i = 0; i < numberOfGroupParticles; ++i)
	{
		EXPECT_EQ(expectedGroupData[i],
					*((int32*)system->GetUserDataBuffer()[
						particleHandles[i]->GetIndex()]));
	}
}

// Test the conversion of particle expiration times to lifetimes.
TEST_F(FunctionTests, ConvertExpirationTimeToLifetime)
{
	m_particleSystem->SetDestructionByAge(true);
	EXPECT_EQ(m_particleSystemDef.lifetimeGranularity,
				m_particleSystem->ExpirationTimeToLifetime(1));
}

// Get the particle expiration time buffers with no particles present.
TEST_F(FunctionTests, NoParticlesGetExpirationTimeBuffers)
{
	EXPECT_TRUE(NULL != m_particleSystem->GetExpirationTimeBuffer());
	EXPECT_TRUE(NULL !=
				m_particleSystem->GetIndexByExpirationTimeBuffer());
}

// Ensure the default lifetime of particles is infinite.
TEST_F(FunctionTests, GetParticleLifetime)
{
	b2ParticleDef def;
	const int32 index = m_particleSystem->CreateParticle(def);
	EXPECT_EQ(0.0f, m_particleSystem->GetParticleLifetime(index));
}

// Get particle expiration time buffers after creating some particles.
TEST_F(FunctionTests, GetExpirationTimeBuffers)
{
	const float32 expirationTimeEpislon =
		m_particleSystemDef.lifetimeGranularity;
	// Create some particles with increasing lifetimes (the first one has an
	// infinite lifetime).
	b2ParticleDef def;
	for (int32 i = 0; i < 10; ++i)
	{
		const int32 index = m_particleSystem->CreateParticle(def);
		m_particleSystem->SetParticleLifetime(index, (float32)i);
	}

	m_world->Step(m_particleSystemDef.lifetimeGranularity, 1, 1);

	const int32* const expirationTimes =
		m_particleSystem->GetExpirationTimeBuffer();
	const int32* const indexByLifetime =
		m_particleSystem->GetIndexByExpirationTimeBuffer();
	ASSERT_TRUE(expirationTimes != NULL);
	ASSERT_TRUE(indexByLifetime != NULL);

	// Verify the expiration of the created particles.
	// particle 0 is a special case since it has an infinite lifetime.
	EXPECT_LE(expirationTimes[indexByLifetime[0]], 0.0f);
	for (int32 i = 1; i < m_particleSystem->GetParticleCount(); ++i)
	{
		EXPECT_NEAR((float32)(10 - i),
					m_particleSystem->ExpirationTimeToLifetime(
						expirationTimes[indexByLifetime[i]]) +
						m_particleSystemDef.lifetimeGranularity,
					expirationTimeEpislon);
	}
}

// Set a particle lifetime.
TEST_F(FunctionTests, SetParticleLifetime)
{
	b2ParticleDef def;
	const int32 index = m_particleSystem->CreateParticle(def);
	m_particleSystem->SetParticleLifetime(index, 1.0f);
	EXPECT_NEAR(1.0f, m_particleSystem->GetParticleLifetime(index),
				m_particleSystemDef.lifetimeGranularity);
}

// Create a particle with an infinite lifetime and verify that it isn't
// destroyed after a relatively long time (30s).
TEST_F(FunctionTests, CreateParticleInfiniteLifetime)
{
	b2ParticleDef def;
	const int32 index = m_particleSystem->CreateParticle(def);
	m_particleSystem->SetParticleLifetime(index, 0.0f);
	m_world->Step(30.0f, 1, 1);
	EXPECT_EQ(1, m_particleSystem->GetParticleCount());
}

// Create a particle with a finite lifetime (1s) and verify that it's
// destroyed after the time has elasped.
TEST_F(FunctionTests, CreateParticleFiniteLifetime)
{
	static const float32 simulationPeriod = 0.1f;
	b2ParticleDef def;
	const int32 index = m_particleSystem->CreateParticle(def);
	m_particleSystem->SetParticleLifetime(index, 1.0f);
	float32 secondsElapsed = 0.0f;
	while (m_particleSystem->GetParticleCount())
	{
		 m_world->Step(simulationPeriod, 1, 1);
		 secondsElapsed += simulationPeriod;
	}
	EXPECT_FLOAT_EQ(1.0f, secondsElapsed);
}

// Create a particle with finite lifetime using a particle definition.
TEST_F(FunctionTests, CreateParticleWithFiniteLifetimeUsingDef)
{
	b2ParticleDef def;
	def.lifetime = 1.0f;
	const int32 index = m_particleSystem->CreateParticle(def);
	EXPECT_NEAR(1.0f, m_particleSystem->GetParticleLifetime(index),
				m_particleSystemDef.lifetimeGranularity);
}

// Create a particle group with finite lifetime.
TEST_F(FunctionTests, CreateParticleGroupWithFiniteLifetime)
{
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	b2ParticleGroupDef def;
	def.lifetime = 1.0f;
	def.shape = &shape;
	b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(def);
	const int32 particleCount = group->GetParticleCount();
	const int32 particleGroupIndex = group->GetBufferIndex();
	for (int32 i = 0; i < particleCount; ++i)
	{
		EXPECT_NEAR(1.0f, m_particleSystem->GetParticleLifetime(
						i + particleGroupIndex),
					m_particleSystemDef.lifetimeGranularity);
	}
}

// Create multiple particles with infinite lifetimes and verify the newest
// particle has the largest lifetime value and is referenced at the end of the
// lifetime buffer.
TEST_F(FunctionTests, CreateMultipleParticlesWithInfiniteLifetimes)
{
	static const float32 simulationPeriod =
		m_particleSystemDef.lifetimeGranularity;
	b2ParticleDef def;
	m_particleSystem->SetDestructionByAge(true);
	for (int32 i = 0; i < 10; ++i)
	{
		m_particleSystem->CreateParticle(def);
		m_world->Step(simulationPeriod, 1, 1);
	}
	const int32* const expirationTimes =
		m_particleSystem->GetExpirationTimeBuffer();
	const int32* const indexByExpirationTime =
		m_particleSystem->GetIndexByExpirationTimeBuffer();
	ASSERT_TRUE(expirationTimes != NULL);
	ASSERT_TRUE(indexByExpirationTime != NULL);

	int32 previousExpirationTime = expirationTimes[indexByExpirationTime[0]];
	for (int32 i = 1; i < m_particleSystem->GetParticleCount(); ++i)
	{
		const int32 currentExpirationTime =
			expirationTimes[indexByExpirationTime[i]];
		EXPECT_LT(currentExpirationTime, previousExpirationTime);
		previousExpirationTime = currentExpirationTime;
	}
}

// Create a set of particles each with a finite lifetime, setting unsorted
// lifetimes so that the particle system needs to resort the queue of
// particles that are scheduled to be destroyed.
TEST_F(FunctionTests, CreateParticlesWithUnsortedLifetimes)
{
	static const float32 simulationPeriod = 0.1f;
	static const float32 lifetimes[] = {0.9f, 0.2f, 0.3f, 0.0f, 2.0f};
	int32 particleIndices[B2_ARRAY_SIZE(lifetimes)];
	b2ParticleDef def;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(lifetimes); ++i)
	{
		particleIndices[i] = m_particleSystem->CreateParticle(def);
		m_particleSystem->SetParticleLifetime(i, lifetimes[i]);
	}

	LifetimeDestructionChecker checker(simulationPeriod,
										 m_world, m_particleSystem);
	checker.TrackParticleLifetimes(particleIndices,
									 B2_ARRAY_SIZE(particleIndices));

	// Simulate for 2.1 seconds since 2 seconds is the maximum lifetime of
	// the particles with a finite lifetime.
	while (checker.GetTimeElapsed() < 2.1f)
	{
		checker.Step(simulationPeriod);
		m_world->Step(simulationPeriod, 1, 1);
	}
	EXPECT_EQ(1, m_particleSystem->GetParticleCount());
}

// Create two particle groups containing particles with finite lifetimes
// joining the groups to force a resort of particle lifetimes.
TEST_F(FunctionTests, CreateParticleGroupsWithFiniteLifetimes)
{
	static const float32 simulationPeriod = 0.1f;
	static const float32 simulationTime = 10.0f;
	b2ParticleGroup *groups[2];

	// Create two particle groups.
	b2ParticleGroupDef def;
	b2PolygonShape shape;
	shape.SetAsBox(0.1f, 0.1f);
	def.shape = &shape;
	groups[0] = m_particleSystem->CreateParticleGroup(def);
	def.position = b2Vec2(0.05f, 0.0f);
	groups[1] = m_particleSystem->CreateParticleGroup(def);

	// Allocate an array to save indices of particles in the groups.
	const int32 totalParticleCount = groups[0]->GetParticleCount() +
		groups[1]->GetParticleCount();
	int32* particleIndices = new int32[totalParticleCount];
	int32 particleIndexOffset = 0;

	// Assign random lifetimes between 0.1f and simulationTime seconds to
	// particles in the groups.
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(groups); ++i)
	{
		const b2ParticleGroup * const group = groups[i];
		const int32 numberOfParticles = group->GetParticleCount();
		const int32 bufferIndex = group->GetBufferIndex();
		for (int32 j = 0; j < numberOfParticles; ++j)
		{
			const int32 particleIndex = bufferIndex + j;
			particleIndices[particleIndexOffset++] = particleIndex;
			m_particleSystem->SetParticleLifetime(
				particleIndex,
				((float32)(rand() % 99) / simulationTime) + 0.1f);
		}
	}

	// Track all particle lifetimes.
	LifetimeDestructionChecker checker(simulationPeriod,
										 m_world, m_particleSystem);
	checker.TrackParticleLifetimes(particleIndices, totalParticleCount);

	delete [] particleIndices;
	particleIndices = NULL;

	// Simulate 11 seconds (1 second longer than the longest particle lifetime.
	for (float32 time = 0.0f; time < simulationTime + 1.0f;
		 time += simulationPeriod)
	{
		// Join the particle groups half way through the simulation to
		// force a resort / compaction of the particle data.
		if (time >= simulationPeriod * 0.5f && groups[1])
		{
			m_particleSystem->JoinParticleGroups(groups[0], groups[1]);
			groups[1] = NULL;
		}

		checker.Step(simulationPeriod);
		m_world->Step(simulationPeriod, 1, 1);
	}
}

// Destroy the oldest particle in the world.
TEST_F(FunctionTests, DestroyOldestParticle)
{
	static const int32 numberOfParticles = 10;
	int32 age[numberOfParticles];
	UserDataDestructionTracker userDataTracker(m_world, m_particleSystem);
	b2ParticleDef def;
	m_particleSystem->SetDestructionByAge(true);
	for (int32 i = 0; i < numberOfParticles; ++i)
	{
		const int32 index = m_particleSystem->CreateParticle(def);
		age[i] = i;
		m_particleSystem->GetUserDataBuffer()[index] = &age[i];
		m_world->Step(0.1f, 1, 1);
	}
	// Destroy the particles, oldest first.
	for (int32 i = 0; i < numberOfParticles; ++i)
	{
		m_particleSystem->DestroyOldestParticle(0, true);
		m_world->Step(0.1f, 1, 1);
	}
	// Verify the particles were destroyed by age.
	const std::vector<void*> userData = userDataTracker.GetUserData();
	EXPECT_EQ((int32)userData.size(), numberOfParticles);
	for (int32 i = 0; i < numberOfParticles; ++i)
	{
		ASSERT_GE(userData.size(), 1U);
		// Check the age of the particle, the oldest particle age should be 0
		// with the youngest age numberOfParticles - 1.
		EXPECT_EQ(i, *((const int32*)userDataTracker.GetUserData()[i]));
	}
}

// Limit the number of particles in the world, continuously create particles
// destroying the oldest ones when the limit is reached.
TEST_F(FunctionTests, LimitParticleCountUsingLifetime)
{
	static const int32 particleLimit = 10;
	b2ParticleDef def;
	OldestParticleDestroyedChecker checker(m_world, m_particleSystem);
	m_particleSystem->SetMaxParticleCount(particleLimit);
	m_particleSystem->SetDestructionByAge(true);
	for (int32 i = 0; i < particleLimit; ++i)
	{
		const int32 index = m_particleSystem->CreateParticle(def);
		m_particleSystem->SetParticleFlags(
			index, m_particleSystem->GetParticleFlags(index) |
					 b2_destructionListenerParticle);
		// Create half of the particles with a finite lifetime.
		if (index & 1)
		{
			m_particleSystem->SetParticleLifetime(index, 1.0f);
		}
		m_world->Step(0.1f, 1, 1);
	}

	// The first (particleLimit / 2) finite lifetime particles will be
	// destroyed followed by (particleLimit / 2) infinite lifetime particles
	// as new particles are created.
	for (int32 i = 0; i < particleLimit; ++i)
	{
		const int32 index = m_particleSystem->CreateParticle(def);
		m_particleSystem->SetParticleFlags(
			index, m_particleSystem->GetParticleFlags(index) |
					 b2_destructionListenerParticle);
		m_world->Step(0.1f, 1, 1);
	}
}

TEST_F(FunctionTests, GetParticleMass) {
	const float mass = m_particleSystem->GetParticleMass();
	const float invMass = m_particleSystem->GetParticleInvMass();
	EXPECT_NEAR(mass * invMass, 1.0f, 0.000001f);
}

#if !BOX2D_EXTERNAL_LANGUAGE_API
#error The external language API should be defined for unit tests
#endif

static const b2Vec2 kParticlePositions[] = {
	b2Vec2(0.01f, 0.01f),
	b2Vec2(0.0f, 0.0f),
	b2Vec2(0.02f, -0.01f),
	b2Vec2(-0.041f, 0.035f),
	b2Vec2(-0.022f, -0.01f),
};

static const b2ParticleColor kParticleColors[] = {
	b2ParticleColor(100, 0, 0, 200),
	b2ParticleColor(0, 64, 0, 128),
	b2ParticleColor(0, 0, 32, 64),
	b2ParticleColor(0, 0, 0, 0),
	b2ParticleColor(255, 255, 255, 255),
};

TEST_F(FunctionTests, CircleShapeSetPosition) {
	b2CircleShape shape;
	b2CircleShape shape2;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		shape.SetPosition(kParticlePositions[i].x, kParticlePositions[i].y);
		shape2.m_p = kParticlePositions[i];
		EXPECT_EQ(shape2.m_p, shape.m_p);
	}
}

TEST_F(FunctionTests, CircleShapeGetPosition) {
	b2CircleShape shape;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		shape.m_p = kParticlePositions[i];
		EXPECT_EQ(shape.m_p.x, shape.GetPositionX());
		EXPECT_EQ(shape.m_p.y, shape.GetPositionY());
	}
}

TEST_F(FunctionTests, EdgeShapeSet) {
	b2EdgeShape shape;
	b2EdgeShape shape2;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions) - 1; ++i)
	{
		shape.Set(kParticlePositions[i].x, kParticlePositions[i].y,
					kParticlePositions[i + 1].x, kParticlePositions[i + 1].y);
		shape2.Set(kParticlePositions[i], kParticlePositions[i + 1]);
		EXPECT_EQ(shape2.m_vertex1, shape.m_vertex1);
		EXPECT_EQ(shape2.m_vertex2, shape.m_vertex2);
	}
}

TEST_F(FunctionTests, PolygonShapeSetCentroid) {
	b2PolygonShape shape;
	b2PolygonShape shape2;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		shape.SetCentroid(kParticlePositions[i].x, kParticlePositions[i].y);
		shape2.m_centroid = kParticlePositions[i];
		EXPECT_EQ(shape2.m_centroid, shape.m_centroid);
	}
}

TEST_F(FunctionTests, PolygonShapeSetAsBox) {
	b2PolygonShape shape;
	b2PolygonShape shape2;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		float halfWidth = 0.1f;
		float halfHeight = 0.05f;
		float angle = 3.0f;
		shape.SetAsBox(halfWidth, halfHeight, kParticlePositions[i].x,
									 kParticlePositions[i].y, angle);
		shape2.SetAsBox(halfWidth, halfHeight, kParticlePositions[i], angle);
		EXPECT_EQ(shape2.m_centroid, shape.m_centroid);
		EXPECT_EQ(shape2.m_count, shape.m_count);
		for (int32 j = 0; j < shape.m_count; ++j) {
			EXPECT_EQ(shape2.m_vertices[j], shape.m_vertices[j]);
			EXPECT_EQ(shape2.m_normals[j], shape.m_normals[j]);
		}
	}
}

TEST_F(FunctionTests, TransformGetPos) {
	b2Transform xf;
	float angle = 3.0f;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		xf.Set(kParticlePositions[i], angle);
		EXPECT_EQ(xf.p.x, xf.GetPositionX());
		EXPECT_EQ(xf.p.y, xf.GetPositionY());
		EXPECT_EQ(xf.q.s, xf.GetRotationSin());
		EXPECT_EQ(xf.q.c, xf.GetRotationCos());
	}
}

TEST_F(FunctionTests, BodyDefSetPosition) {
	b2BodyDef bd;
	b2BodyDef bd2;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		bd.SetPosition(kParticlePositions[i].x, kParticlePositions[i].y);
		bd2.position = kParticlePositions[i];
		EXPECT_EQ(bd2.position, bd.position);
	}
}

TEST_F(FunctionTests, BodyGetPosition) {
	b2BodyDef bodyDef;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		bodyDef.position = kParticlePositions[i];
		b2Body *body = m_world->CreateBody(&bodyDef);
		EXPECT_EQ(body->GetPosition().x, body->GetPositionX());
		EXPECT_EQ(body->GetPosition().y, body->GetPositionY());
	}
}

TEST_F(FunctionTests, BodySetTransform) {
	b2BodyDef bodyDef;
	b2Body *body = m_world->CreateBody(&bodyDef);
	b2Body *body2 = m_world->CreateBody(&bodyDef);
	float angle = 3.0f;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		body->SetTransform(kParticlePositions[i].x, kParticlePositions[i].y, angle);
		body2->SetTransform(kParticlePositions[i], angle);
		EXPECT_EQ(body2->GetTransform().p, body->GetTransform().p);
		EXPECT_EQ(body2->GetTransform().q.GetAngle(),
							body->GetTransform().q.GetAngle());
	}
}

TEST_F(FunctionTests, WorldConstruct) {
	b2Vec2 gravity = m_world->GetGravity();
	b2World* world = new b2World(gravity.x, gravity.y);
	EXPECT_EQ(m_world->GetGravity(), world->GetGravity());
	delete world;
}

TEST_F(FunctionTests, WorldSetGravity) {
	b2Vec2 gravity = m_world->GetGravity();
	b2World* world = new b2World(b2Vec2(0.0f, 0.0f));
	world->SetGravity(gravity.x, gravity.y);
	EXPECT_EQ(m_world->GetGravity(), world->GetGravity());
	delete world;
}

TEST_F(FunctionTests, ParticleDefSetPositionColor) {
	b2ParticleDef def;
	b2ParticleDef def2;
	def2.position.Set(0.01f, 0.02f);
	def2.color.Set(3, 4, 5, 6);
	def.SetPosition(def2.position.x, def2.position.y);
	def.SetColor(def2.color.r, def2.color.g, def2.color.b, def2.color.a);
	EXPECT_EQ(def2.position, def.position);
	EXPECT_EQ(def2.color, def.color);
}

TEST_F(FunctionTests, ParticleSystemSetParticleVelocity) {
	b2ParticleDef def;
	def.velocity.Set(0.0f, 0.0f);
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		int index = m_particleSystem->CreateParticle(def);
		m_particleSystem->SetParticleVelocity(index,
												kParticlePositions[i].x,
												kParticlePositions[i].y);
		EXPECT_EQ(kParticlePositions[i].x,
					m_particleSystem->GetVelocityBuffer()[index].x);
		EXPECT_EQ(kParticlePositions[i].y,
					m_particleSystem->GetVelocityBuffer()[index].y);
	}
}

TEST_F(FunctionTests, ParticleSystemGetParticlePosition) {
	b2ParticleDef def;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		def.position.Set(kParticlePositions[i].x, kParticlePositions[i].y);
		int index = m_particleSystem->CreateParticle(def);
		EXPECT_EQ(m_particleSystem->GetPositionBuffer()[index].x,
					m_particleSystem->GetParticlePositionX(index));
		EXPECT_EQ(m_particleSystem->GetPositionBuffer()[index].y,
					m_particleSystem->GetParticlePositionY(index));
	}
}

TEST_F(FunctionTests, CopyParticleBuffer) {
	b2ParticleDef def;
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		def.position = kParticlePositions[i];
		def.color = kParticleColors[i];
		m_particleSystem->CreateParticle(def);
	}

	// Test position buffer
	b2Vec2 positionsByValue[B2_ARRAY_SIZE(kParticlePositions)];
	int exceptionType = m_particleSystem->CopyPositionBuffer(
		0, B2_ARRAY_SIZE(kParticlePositions), positionsByValue,
		sizeof(positionsByValue));

	EXPECT_EQ(exceptionType, b2ParticleSystem::b2_noExceptions)
		<< "Position buffer not read";

	const b2Vec2* positionsByReference = m_particleSystem->GetPositionBuffer();
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		EXPECT_EQ(positionsByReference[i], positionsByValue[i])
			<< "Buffer positions different";
	}

	// Test color buffer
	b2ParticleColor colorsByValue[B2_ARRAY_SIZE(kParticlePositions)];
	exceptionType = m_particleSystem->CopyColorBuffer(
		0, B2_ARRAY_SIZE(kParticlePositions), colorsByValue,
		sizeof(colorsByValue));

	EXPECT_EQ(exceptionType, b2ParticleSystem::b2_noExceptions)
		<< "Color buffer not read";

	const b2ParticleColor* colorsByReference = m_particleSystem->GetColorBuffer();
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		EXPECT_EQ(colorsByReference[i], colorsByValue[i])
			<< "Buffer colors different";
	}

	m_world->Step(0.001f, 1, 1);

	// Test weight buffer after stepping
	float32 weightsByValue[B2_ARRAY_SIZE(kParticlePositions)];
	exceptionType = m_particleSystem->CopyWeightBuffer(
		0, B2_ARRAY_SIZE(kParticlePositions), weightsByValue,
		sizeof(weightsByValue));

	EXPECT_EQ(exceptionType, b2ParticleSystem::b2_noExceptions)
		<< "Weight buffer not read";

	const float32* weightsByReference = m_particleSystem->GetWeightBuffer();
	for (int32 i = 0; i < (int32)B2_ARRAY_SIZE(kParticlePositions); ++i)
	{
		EXPECT_EQ(weightsByReference[i], weightsByValue[i])
			<< "Buffer weights different";
	}
}

TEST_F(FunctionTests, CreateParticleGroupWithVertexList) {
	b2ParticleGroupDef def;
	const int shapeCount = B2_ARRAY_SIZE(kParticlePositions);
	def.SetCircleShapesFromVertexList((void*) kParticlePositions,
																		shapeCount, 1);
	def.shapeCount = 1;
	b2ParticleGroup *group1 = m_particleSystem->CreateParticleGroup(def);
	EXPECT_GT(group1->GetParticleCount(), 0);
	def.shapeCount = shapeCount;
	b2ParticleGroup *group2 = m_particleSystem->CreateParticleGroup(def);
	EXPECT_GT(group2->GetParticleCount(), group1->GetParticleCount());
}

TEST_F(FunctionTests, ParticleGroupDefSetPositionColor) {
	b2ParticleGroupDef def;
	b2ParticleGroupDef def2;
	def2.position.Set(0.01f, 0.02f);
	def2.color.Set(3, 4, 5, 6);
	def.SetPosition(def2.position.x, def2.position.y);
	def.SetColor(def2.color.r, def2.color.g, def2.color.b, def2.color.a);
	EXPECT_EQ(def2.position, def.position);
	EXPECT_EQ(def2.color, def.color);
}

TEST_F(FunctionTests, AreProxyBuffersTheSame) {
	b2BlockAllocator blockAllocator;
	b2GrowableBuffer<b2ParticleSystem::Proxy> a(blockAllocator);
	b2GrowableBuffer<b2ParticleSystem::Proxy> b(blockAllocator);

	// Compare proxies with same tags, but indices in different orders.
	static const int LEN_BUFFERS = 4;
	for (int i = 0; i < LEN_BUFFERS; ++i)
	{
		b2ParticleSystem::Proxy proxy;
		proxy.tag = 3;
		proxy.index = i;
		a.Append() = proxy;

		proxy.index = LEN_BUFFERS - i - 1;
		b.Append() = proxy;
	}
	EXPECT_TRUE(b2ParticleSystem::AreProxyBuffersTheSame(a, b));

	// Compare proxies with same tags, but different indices.
	b[LEN_BUFFERS / 2].index = LEN_BUFFERS - 1;
	EXPECT_FALSE(b2ParticleSystem::AreProxyBuffersTheSame(a, b));
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
