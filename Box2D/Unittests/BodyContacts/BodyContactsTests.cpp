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
#include <utility>
#include <vector>

class BodyContactTests : public ::testing::Test {
protected:
	virtual void SetUp();
	virtual void TearDown();

	// Create the world, ground body and the particle system.
	void CreateWorld();

	// Create a valley (similar to the Anti-Pointy test in the Testbed) and
	// attach to the ground body.
	void CreateValley();

	// Create a horizontal box fixture attached to the ground body.
	b2Fixture* CreateGroundBox();

	// Run simulation for a specified amount of time at the specified
	// frequency.
	void RunStep(const float32 frequencyHz, const float32 simulationTime);

	// Destroy all particles in the system.
	void DestroyAllParticles();

	// Destroy all particles in the system,
	// drop a particle onto the fixture and keep track of the number of
	// contacts with the fixture in addition to the stuck particle candiates.
	void DropParticle();

protected:
	b2World *m_world;
	b2Body *m_groundBody;
	b2ParticleSystem *m_particleSystem;
	float32 m_particleDiameter;

	// Total number of contacts per run of DropParticle().
	int32 m_contacts;
	// Total number of stuck particles per run of DropParticle().
	int32 m_stuck;
};

// Contact between a fixture and particle (index).
typedef std::pair<b2Fixture*, int32> FixtureParticleContact;
// Contact between two particles.
typedef std::pair<int32, int32> ParticleContact;

// Keeps track of particle / fixture, particle / particle contacts.
class ParticleContactTracker : public b2ContactFilter
{
public:
	typedef std::pair<int32, int32> ParticleContact;

public:
	ParticleContactTracker(b2ParticleSystem* const system) : m_system(system)
	{
	}

	virtual ~ParticleContactTracker() {}

	// Add the contact to the list of particle / fixture contacts.
	virtual bool ShouldCollide(b2Fixture* fixture,
								 b2ParticleSystem* particleSystem,
								 int32 particleIndex)
	{
		EXPECT_EQ(particleSystem, m_system);
		FixtureParticleContact contact(fixture, particleIndex);
		m_fixtureParticleContacts.push_back(contact);
		return true;
	}

	// Add the contact to the list of particle / particle contacts.
	virtual bool ShouldCollide(b2ParticleSystem* particleSystem,
								 int32 particleIndexA, int32 particleIndexB)
	{
		EXPECT_EQ(particleSystem, m_system);
		ParticleContact contact(particleIndexA, particleIndexB);
		m_particleContacts.push_back(contact);
		return true;
	}

public:
	std::vector<FixtureParticleContact> m_fixtureParticleContacts;
	std::vector<ParticleContact> m_particleContacts;

protected:
	b2ParticleSystem* m_system;
};

// Optionally disables particle / fixture and particle / particle contacts.
class ParticleContactDisabler : public b2ContactFilter
{
public:
	ParticleContactDisabler() :
		m_enableFixtureParticleCollisions(true),
		m_enableParticleParticleCollisions(true)
	{
	}

	virtual ~ParticleContactDisabler() { }

	// Blindly enable / disable collisions between fixtures and particles.
	virtual bool ShouldCollide(b2Fixture* fixture,
								 b2ParticleSystem* particleSystem,
								 int32 particleIndex)
	{
		B2_NOT_USED(fixture);
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(particleIndex);
		return m_enableFixtureParticleCollisions;
	}

	// Blindly enable / disable collisions between particles.
	virtual bool ShouldCollide(b2ParticleSystem* particleSystem,
								 int32 particleIndexA, int32 particleIndexB)
	{
		B2_NOT_USED(particleSystem);
		B2_NOT_USED(particleIndexA);
		B2_NOT_USED(particleIndexB);
		return m_enableParticleParticleCollisions;
	}

public:
	bool m_enableFixtureParticleCollisions;
	bool m_enableParticleParticleCollisions;
};

// Tracks contacts between particles and fixtures using a contact listener.
class ParticleContactListener : public b2ContactListener
{
public:
	ParticleContactListener(b2ParticleSystem* const system) :
		m_system(system)
	{
	}

	// Add the contact to m_beginFixtureContacts.
	virtual void BeginContact(b2ParticleSystem* particleSystem,
								b2ParticleBodyContact* particleBodyContact)
	{
		EXPECT_EQ(particleSystem, m_system);
		m_beginFixtureContacts.push_back(*particleBodyContact);
	}

	// Add the contact to m_endFixtureContacts.
	virtual void EndContact(b2Fixture* fixture,
							b2ParticleSystem* particleSystem, int32 index)
	{
		EXPECT_EQ(particleSystem, m_system);
		FixtureParticleContact contact(fixture, index);
		m_endFixtureContacts.push_back(contact);
	}

	// Add the contact to m_beginParticlContacts.
	virtual void BeginContact(b2ParticleSystem* particleSystem,
								b2ParticleContact* particleContact)
	{
		EXPECT_EQ(particleSystem, m_system);
		m_beginParticleContacts.push_back(*particleContact);
	}

	// Add the contact to m_endParticlContacts.
	virtual void EndContact(b2ParticleSystem* particleSystem,
							int32 indexA, int32 indexB)
	{
		EXPECT_EQ(particleSystem, m_system);
		ParticleContact contact(indexA, indexB);
		m_endParticleContacts.push_back(contact);
	}


public:
	std::vector<b2ParticleBodyContact> m_beginFixtureContacts;
	std::vector<FixtureParticleContact> m_endFixtureContacts;
	std::vector<b2ParticleContact> m_beginParticleContacts;
	std::vector<ParticleContact> m_endParticleContacts;

protected:
	b2ParticleSystem* m_system;
};

// Create the world, ground body and the particle system.
void BodyContactTests::SetUp()
{
	// Define the gravity vector.
	const b2Vec2 gravity(0.0f, -10.0f);
	// Construct a world object, which will simulate the contacts.
	m_world = new b2World(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, 0.0f);

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	m_groundBody = m_world->CreateBody(&groundBodyDef);

	// Create the particle system.
	b2ParticleSystemDef particleSystemDef;
	particleSystemDef.radius = 0.1f;
	m_particleSystem = m_world->CreateParticleSystem(&particleSystemDef);
	m_particleDiameter = m_particleSystem->GetRadius() * 2.0f;
}

// Clean up the world.
void BodyContactTests::TearDown()
{
	delete m_world;
	m_world = NULL;
	m_groundBody = NULL;
	m_particleSystem = NULL;
	m_contacts = 0;
	m_stuck = 0;
}

// Create a valley (similar to the Anti-Pointy test in the Testbed) and
// attach to the ground body.
void BodyContactTests::CreateValley()
{
	b2Assert(m_groundBody);
	float32 i;
	const float32 step = 0.1f;
	for (i = -1.0f; i < 1.0f; i+=step)
	{
		b2PolygonShape shape;
		const b2Vec2 vertices[] = {
			b2Vec2(i, -1.0f),
			b2Vec2(i+step, -1.0f),
			b2Vec2(0.0f, 1.5f)
		};
		shape.Set(vertices, B2_ARRAY_SIZE(vertices));
		m_groundBody->CreateFixture(&shape, 0.0f);
	}
	for (i = -1.0f; i < 3.5f; i+=step)
	{
		b2PolygonShape shape;
		const b2Vec2 vertices[] = {
			b2Vec2(-1.0f, i),
			b2Vec2(-1.0f, i+step),
			b2Vec2(0.0f, 1.5f)
		};
		shape.Set(vertices, B2_ARRAY_SIZE(vertices));
		m_groundBody->CreateFixture(&shape, 0.0f);

		const b2Vec2 vertices2[] = {
			b2Vec2(1.0f, i),
			b2Vec2(1.0f, i+step),
			b2Vec2(0.0f, 1.5f)
		};
		shape.Set(vertices2, B2_ARRAY_SIZE(vertices2));
		m_groundBody->CreateFixture(&shape, 0.0f);
	}
}

// Create a horizontal box fixture attached to the ground body.
b2Fixture* BodyContactTests::CreateGroundBox()
{
	b2Assert(m_groundBody);
	b2PolygonShape shape;
	shape.SetAsBox(10.0f * m_particleDiameter, m_particleDiameter);
	return m_groundBody->CreateFixture(&shape, 0.0f);
}

// Run simulation for a specified amount of time at the specified
// frequency.
void BodyContactTests::RunStep(const float32 frequencyHz,
								 const float32 simulationTime)
{
	const float32 timeStep = 1.0f / frequencyHz;
	const int32 frames = b2Max((int32)(frequencyHz * simulationTime), 1);
	for (int32 i = 0; i < frames; ++i)
	{
		m_world->Step(timeStep, 1, 1);
	}
}

// Destroy all particles in the system.
void BodyContactTests::DestroyAllParticles()
{
	const int32 count = m_particleSystem->GetParticleCount();
	for (int32 i = 0; i < count; ++i)
	{
		m_particleSystem->DestroyParticle(i);
	}
}

// Destroy all particles in the system, drop a particle onto the fixture and
// keep track of the number of contacts with the fixture in addition to the
// stuck particle candiates.
void BodyContactTests::DropParticle()
{
	// Reset counters.
	m_contacts = 0;
	m_stuck = 0;
	DestroyAllParticles();

	const float32 timeStep = 1.0f / 60.0f;
	const int32 timeout = (int32)(1.0f / timeStep) * 10; // 10 "seconds"
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;

	// Step once to eliminate particles.
	m_world->Step(timeStep, velocityIterations, positionIterations);

	b2ParticleDef pd;
	pd.position.Set(0.0f, 3.3f);
	pd.velocity.Set(0.0f, -0.1f);
	m_particleSystem->CreateParticle(pd);

	for (int32 i = 0; i < timeout; ++i)
	{
		m_world->Step(timeStep, velocityIterations, positionIterations);
		m_contacts += m_particleSystem->GetBodyContactCount();
		int32 stuck = m_particleSystem->GetStuckCandidateCount();
		if (stuck)
		{
			m_stuck += stuck;
			// Should always be particle 0.
			EXPECT_EQ(*(m_particleSystem->GetStuckCandidates()), 0);
		}
	}
}

TEST_F(BodyContactTests, ParticleDropStrictContactCheck)
{
	CreateValley();
	m_particleSystem->SetStrictContactCheck(false);

	DropParticle();
	EXPECT_GT(m_contacts, 0) << "No contacts within timeout";
	int32 contacts = m_contacts;
	m_particleSystem->SetStrictContactCheck(true);

	DropParticle();
	EXPECT_GT(m_contacts, 0) << "No strict contacts within timeout";
	EXPECT_LT(m_contacts, contacts) << "No contact pruning performed";
}

TEST_F(BodyContactTests, StuckParticleDetection)
{
	CreateValley();
	DropParticle();
	EXPECT_GT(m_contacts, 0) << "No contacts within timeout";
	EXPECT_EQ(m_stuck, 0) << "Stuck particle detected when detection disabled";
	int32 stuck = 0x7fffffff;

	for (int32 i = 1; i < 256; i *= 2)
	{
		m_particleSystem->SetStuckThreshold(i);
		DropParticle();
		EXPECT_GT(m_stuck, 0) << "No stuck particles detected";
		EXPECT_GT(stuck, m_stuck) << "Fewer stuck particle reports expected";
		stuck = m_stuck;
	}
}

// Verify that it's possible to detect particle / body collisions using
// a contact filter.
TEST_F(BodyContactTests, FixtureParticleContactFilter)
{
	// Create the ground.
	b2Fixture* const groundFixture = CreateGroundBox();
	{
		// Create a particle above the ground and slightly to the left of the
		// particle with the contact filter enabled.
		b2ParticleDef pd;
		pd.position.Set(2.0f * m_particleDiameter, 2.0f * m_particleDiameter);
		pd.position.Set(0.0f, -m_particleDiameter);
		m_particleSystem->CreateParticle(pd);
	}

	// Create a particle above the ground and drop it.
	int32 contactFilterParticleIndex;
	{
		b2ParticleDef pd;
		pd.flags = b2_fixtureContactFilterParticle;
		pd.position.Set(0.0f, 2.0f * m_particleDiameter);
		pd.position.Set(0.0f, -m_particleDiameter);
		// WARNING: This assumes that this particle index will not change
		// between simulation steps.
		contactFilterParticleIndex = m_particleSystem->CreateParticle(pd);
	}

	// Set the tracker as a contact listener.
	ParticleContactTracker tracker(m_particleSystem);
	m_world->SetContactFilter(&tracker);

	RunStep(60.0f, 1.0f);
	EXPECT_EQ(tracker.m_particleContacts.size(), 0U);
	EXPECT_GT(tracker.m_fixtureParticleContacts.size(), 0U);

	for (uint32 i = 0; i < tracker.m_fixtureParticleContacts.size(); ++i)
	{
		const FixtureParticleContact contact =
			tracker.m_fixtureParticleContacts[i];
		// Verify the contact is with the ground fixture.
		EXPECT_EQ(contact.first, groundFixture);
		// Since there is only one particle, the index will always be zero.
		EXPECT_EQ(contact.second, contactFilterParticleIndex);
	}
}

// Verify that it's possible to detect collisions between
TEST_F(BodyContactTests, ParticleParticleContactFilter)
{
	// Drop particle B on top of particle A.
	b2ParticleDef pd;
	pd.flags = b2_particleContactFilterParticle;
	pd.position.Set(0.0f, m_particleDiameter);
	pd.velocity.Set(0.0f, -m_particleDiameter);
	const int32 particleA = m_particleSystem->CreateParticle(pd);
	pd.position.Set(0.0f, m_particleDiameter * 3.0f);
	pd.velocity.Set(0.0f, m_particleDiameter * -2.0f);
	const int32 particleB = m_particleSystem->CreateParticle(pd);

	ParticleContactTracker tracker(m_particleSystem);
	m_world->SetContactFilter(&tracker);

	RunStep(60.0f, 2.0f);
	EXPECT_EQ(tracker.m_fixtureParticleContacts.size(), 0U);
	EXPECT_GT(tracker.m_particleContacts.size(), 0U);

	for (uint32 i = 0; i < tracker.m_particleContacts.size(); ++i)
	{
		const ParticleContactTracker::ParticleContact contact =
			tracker.m_particleContacts[i];
		EXPECT_TRUE(contact.first == particleA || contact.first == particleB);
		EXPECT_TRUE(contact.first == particleA ? contact.second == particleB :
						contact.second == particleA);
	}
}

// Verify that it's possible to enable / disable collisions between fixtures
// and particles.
TEST_F(BodyContactTests, EnableDisableFixtureParticleContactsWithContactFilter)
{
	// Create a fixture to drop particles on to.
	b2Fixture* const groundFixture = CreateGroundBox();

	// Create a particle above the ground and drop it.
	b2ParticleDef pd;
	pd.flags = b2_fixtureContactFilterParticle;
	pd.position.Set(0.0f, m_particleDiameter * 2.0f);
	pd.velocity.Set(0.0f, -m_particleDiameter);
	m_particleSystem->CreateParticle(pd);

	// Leave contacts enabled.
	ParticleContactDisabler contactDisabler;
	m_world->SetContactFilter(&contactDisabler);

	// Run the simulation and verify the particle is on top of the ground
	// fixture.
	RunStep(60.0f, 2.0f);
	EXPECT_GE(m_particleSystem->GetPositionBuffer()[0].y,
				groundFixture->GetAABB(0).upperBound.y);

	// Disable fixture / particle contacts.
	contactDisabler.m_enableFixtureParticleCollisions = false;

	// Run the simulation and verify the particle falls through the ground.
	RunStep(60.0f, 2.0f);
	EXPECT_LT(m_particleSystem->GetPositionBuffer()[0].y,
				groundFixture->GetAABB(0).upperBound.y);

}

// Verify that it's possible to enable / disable collisions between particles.
TEST_F(BodyContactTests, EnableDisableParticleContactsWithContactFilter)
{
	m_world->SetGravity(b2Vec2_zero);

	// Through two particles horizontally at each other
	// (A moving right, B moving left).
	b2ParticleDef pd;
	pd.flags = b2_particleContactFilterParticle;
	pd.position.Set(-m_particleDiameter * 2.0f, 0.0f);
	pd.velocity.Set(m_particleDiameter, 0.0f);
	int32 particleIndexA = m_particleSystem->CreateParticle(pd);

	pd.flags = b2_particleContactFilterParticle;
	pd.position.Set(m_particleDiameter * 2.0f, 0.0f);
	pd.velocity.Set(-m_particleDiameter, 0.0f);
	int32 particleIndexB = m_particleSystem->CreateParticle(pd);

	// Leave contacts enabled.
	ParticleContactDisabler contactDisabler;
	m_world->SetContactFilter(&contactDisabler);

	// Run the simulation and verify that particles don't pass each other.
	RunStep(60.0f, 3.0f);
	// WARNING: This assumes particle indicies have not been reallocated during
	// the simulation.
	b2Vec2* positions = m_particleSystem->GetPositionBuffer();
	b2Vec2* velocities = m_particleSystem->GetVelocityBuffer();
	EXPECT_LT(positions[particleIndexA].x, positions[particleIndexB].x);
	EXPECT_LT(b2Abs(velocities[particleIndexA].x), m_particleDiameter);
	EXPECT_LT(b2Abs(velocities[particleIndexB].x), m_particleDiameter);

	// Disable particle / particle contacts.
	contactDisabler.m_enableParticleParticleCollisions = false;

	// Reset the positions and velocities of the particles.
	positions[particleIndexA].Set(-m_particleDiameter * 2.0f, 0.0f);
	velocities[particleIndexA].Set(m_particleDiameter, 0.0f);
	positions[particleIndexB].Set(m_particleDiameter * 2.0f, 0.0f);
	velocities[particleIndexB].Set(-m_particleDiameter, 0.0f);

	// Run the simulation and verify that particles now pass each other (i.e
	// they no longer collide).
	RunStep(60.0f, 3.0f);
	positions = m_particleSystem->GetPositionBuffer();
	velocities = m_particleSystem->GetVelocityBuffer();
	EXPECT_GT(positions[particleIndexA].x, positions[particleIndexB].x);
	EXPECT_FLOAT_EQ(velocities[particleIndexA].x, m_particleDiameter);
	EXPECT_FLOAT_EQ(velocities[particleIndexB].x, -m_particleDiameter);
}

// Verify that BeginContact() and EndContact() are called for collisions
// between fixtures and particles.
TEST_F(BodyContactTests, ParticleFixtureContactListener)
{
	// Create a fixture to drop particles on to.
	b2Fixture* const groundFixture = CreateGroundBox();

	// Create a particle above the ground and drop it.
	b2ParticleDef pd;
	pd.flags = b2_fixtureContactListenerParticle;
	pd.position.Set(0.0f, m_particleDiameter * 2.0f);
	pd.velocity.Set(0.0f, -m_particleDiameter);
	const int32 particleIndex = m_particleSystem->CreateParticle(pd);

	// Listen for particle / fixture contacts.
	ParticleContactListener tracker(m_particleSystem);
	m_world->SetContactListener(&tracker);

	// Run the simulation and verify that the particle contacts with the
	// fixture.
	RunStep(60.0f, 3.0f);
	EXPECT_EQ(tracker.m_beginFixtureContacts.size(), 1U);
	EXPECT_EQ(tracker.m_endFixtureContacts.size(), 0U);
	for (uint32 i = 0; i < tracker.m_beginFixtureContacts.size(); ++i)
	{
		const b2ParticleBodyContact contact =
			tracker.m_beginFixtureContacts[i];
		EXPECT_EQ(contact.index, particleIndex);
		EXPECT_EQ(contact.fixture, groundFixture);
	}
	tracker.m_beginFixtureContacts.clear();

	// Throw the particle above the ground fixture and verify it's no longer
	// touching.
	m_particleSystem->GetVelocityBuffer()[particleIndex].y =
		m_particleDiameter * 20.0f;
	RunStep(60.0f, 0.5f);
	EXPECT_EQ(tracker.m_beginFixtureContacts.size(), 0U);
	EXPECT_EQ(tracker.m_endFixtureContacts.size(), 1U);

	for (uint32 i = 0; i < tracker.m_endFixtureContacts.size(); ++i)
	{
		const FixtureParticleContact contact =
			tracker.m_endFixtureContacts[i];
		EXPECT_EQ(contact.first, groundFixture);
		EXPECT_EQ(contact.second, particleIndex);
	}
}

// Verify that BeginContact() and EndContact() are called for collisions
// between particles.
TEST_F(BodyContactTests, ParticleContactListener)
{
	// Drop particle B on top of particle A.
	b2ParticleDef pd;
	pd.flags = b2_particleContactListenerParticle;
	pd.position.Set(0.0f, m_particleDiameter);
	pd.velocity.Set(0.0f, -m_particleDiameter);
	const int32 particleA = m_particleSystem->CreateParticle(pd);
	pd.position.Set(0.0f, m_particleDiameter * 3.0f);
	pd.velocity.Set(0.0f, m_particleDiameter * -2.0f);
	const int32 particleB = m_particleSystem->CreateParticle(pd);

	ParticleContactListener listener(m_particleSystem);
	m_world->SetContactListener(&listener);

	RunStep(60.0f, 2.0f);
	EXPECT_GT(listener.m_beginParticleContacts.size(), 0U);
	EXPECT_EQ(listener.m_endParticleContacts.size(), 0U);

	for (uint32 i = 0; i < listener.m_beginParticleContacts.size(); ++i)
	{
		b2ParticleContact contact = listener.m_beginParticleContacts[i];
		EXPECT_TRUE(contact.GetIndexA() == particleA ? contact.GetIndexB() == particleB :
					contact.GetIndexB() == particleA);
	}

	// Push particleB away from particleA.
	b2Vec2* velocities = m_particleSystem->GetVelocityBuffer();
	velocities[particleB].x = m_particleDiameter;
	velocities[particleB].y = 0.0f;
	listener.m_beginParticleContacts.clear();
	RunStep(60.0f, 0.5f);
	EXPECT_EQ(listener.m_beginParticleContacts.size(), 0U);
	EXPECT_GT(listener.m_endParticleContacts.size(), 0U);
}

int
main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
