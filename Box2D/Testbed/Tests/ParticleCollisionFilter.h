/*
* Copyright (c) 2015 Google, Inc.
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

#ifndef PARTICLE_COLLISION_FILTER_H
#define PARTICLE_COLLISION_FILTER_H

// Optionally disables particle / fixture and particle / particle contacts.
class ParticleContactDisabler : public b2ContactFilter {
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

class ParticleCollisionFilter : public Test {
public:

	ParticleCollisionFilter()
	{

		// must also set b2_particleContactFilterParticle and
		// b2_fixtureContactFilterParticle flags for particle group
		m_world->SetContactFilter(&m_contactDisabler);

		m_world->SetGravity(b2Vec2(0, 0));

		// Create the container.
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);
			b2ChainShape shape;
			const b2Vec2 vertices[4] = {
				b2Vec2(-kBoxSize, -kBoxSize + kOffset),
				b2Vec2(kBoxSize, -kBoxSize + kOffset),
				b2Vec2(kBoxSize, kBoxSize + kOffset),
				b2Vec2(-kBoxSize, kBoxSize + kOffset)};
			shape.CreateLoop(vertices, 4);
			b2FixtureDef def;
			def.shape = &shape;
			def.density = 0;
			def.restitution = 1.0;
			ground->CreateFixture(&def);
		}

		// create the particles
		m_particleSystem->SetRadius(0.5f);
		{
			b2PolygonShape shape;
			shape.SetAsBox(1.5f, 1.5f, b2Vec2(kBoxSizeHalf, kBoxSizeHalf + kOffset), 0.0f);
			b2ParticleGroupDef pd;
			pd.shape = &shape;
			pd.flags = b2_powderParticle
					| b2_particleContactFilterParticle
					| b2_fixtureContactFilterParticle;
			m_particleGroup =
				m_particleSystem->CreateParticleGroup(pd);

			b2Vec2* velocities =
				m_particleSystem->GetVelocityBuffer() +
				m_particleGroup->GetBufferIndex();
			for (int i = 0; i < m_particleGroup->GetParticleCount(); ++i) {
				b2Vec2& v = *(velocities + i);
				v.Set(RandomFloat(), RandomFloat());
				v.Normalize();
				v *= kSpeedup;
			}
		}

	}

	virtual void Step(Settings* settings)
	{
		Test::Step(settings);

		const int32 index = m_particleGroup->GetBufferIndex();
		b2Vec2* const velocities =
			m_particleSystem->GetVelocityBuffer() + index;
		for (int32 i = 0; i < m_particleGroup->GetParticleCount(); i++) {
			// Add energy to particles based upon the temperature.
			b2Vec2& v = velocities[i];
			v.Normalize();
			v *= kSpeedup;
		}

		// key help
		{
			static const char* k_keys[] = {
				"Keys: (a) toggle Fixture collisions",
				"   (s) toggle particle collisions"
			};
			for (uint32 i = 0; i < B2_ARRAY_SIZE(k_keys); ++i) {
				g_debugDraw.DrawString(5, m_textLine, k_keys[i]);
				m_textLine += DRAW_STRING_NEW_LINE;
			}
		}
	}

	virtual void Keyboard(unsigned char key)
	{
		switch (key) {
			case 'a':
				ToggleFixtureCollisions();
				break;
			case 's':
				ToggleParticleCollisions();
				break;
			default:
				Test::Keyboard(key);
				break;
		}
	}

	void ToggleFixtureCollisions()
	{
		m_contactDisabler.m_enableFixtureParticleCollisions = !m_contactDisabler.m_enableFixtureParticleCollisions;
	}

	void ToggleParticleCollisions()
	{
		m_contactDisabler.m_enableParticleParticleCollisions = !m_contactDisabler.m_enableParticleParticleCollisions;
	}

	static Test* Create()
	{
		return new ParticleCollisionFilter;
	}

private:
	ParticleContactDisabler m_contactDisabler;
	b2ParticleGroup * m_particleGroup;

	static const float32 kBoxSize;
	static const float32 kBoxSizeHalf;
	static const float32 kOffset;
	static const float32 kParticlesContainerSize;
	static const float32 kSpeedup;
};

const float32 ParticleCollisionFilter::kBoxSize = 10.0f;
const float32 ParticleCollisionFilter::kBoxSizeHalf = kBoxSize/2;
const float32 ParticleCollisionFilter::kOffset = 20.0f;
const float32 ParticleCollisionFilter::kParticlesContainerSize = kOffset+0.5f;
const float32 ParticleCollisionFilter::kSpeedup = 8.0f;


#endif