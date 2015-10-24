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

#ifndef PARTICLE_EMITTER_H
#define PARTICLE_EMITTER_H
#include <math.h>
#include <set>

// Callback used to notify the user of created particles.
class EmittedParticleCallback {
public:
	EmittedParticleCallback() {}
	virtual ~EmittedParticleCallback() {}

	// Called for each created particle.
	virtual void ParticleCreated(b2ParticleSystem * const system,
								 const int32 particleIndex) = 0;
};

// Emit particles from a circular region.
class RadialEmitter {
public:
	// Initialize a particle emitter.
	RadialEmitter() :
		m_particleSystem(NULL), m_callback(NULL), m_speed(0.0f),
		m_emitRate(1.0f), m_emitRemainder(0.0f), m_flags(b2_waterParticle),
		m_group(NULL)
	{
	}

	~RadialEmitter()
	{
		SetGroup(NULL);
	}

	// Set the center of the emitter.
	void SetPosition(const b2Vec2& origin)
	{
		m_origin = origin;
	}

	// Get the center of the emitter.
	const b2Vec2& GetPosition() const
	{
		return m_origin;
	}

	// Set the size of the circle which emits particles.
	void SetSize(const b2Vec2& size)
	{
		m_halfSize = size * 0.5f;
	}

	// Get the size of the circle which emits particles.
	b2Vec2 GetSize() const
	{
		return m_halfSize * 2.0f;
	}

	// Set the starting velocity of emitted particles.
	void SetVelocity(const b2Vec2& velocity)
	{
		m_startingVelocity = velocity;
	}

	// Get the starting velocity.
	const b2Vec2& GetVelocity() const
	{
		return m_startingVelocity;
	}

	// Set the speed of particles along the direction from the center of
	// the emitter.
	void SetSpeed(const float32 speed)
	{
		m_speed = speed;
	}

	// Get the speed of particles along the direction from the center of
	// the emitter.
	float32 GetSpeed() const
	{
		return m_speed;
	}

	// Set the flags for created particles.
	void SetParticleFlags(uint32 flags)
	{
		m_flags = flags;
	}

	// Get the flags for created particles.
	uint32 GetParticleFlags() const
	{
		return m_flags;
	}

	// Set the color of particles.
	void SetColor(const b2ParticleColor& color)
	{
		m_color = color;
	}

	// Get the color of particles emitter.
	const b2ParticleColor& GetColor() const
	{
		return m_color;
	}

	// Set the emit rate in particles per second.
	void SetEmitRate(const float32 emitRate)
	{
		m_emitRate = emitRate;
	}

	// Get the current emit rate.
	float32 GetEmitRate() const
	{
		return m_emitRate;
	}

	// Set the particle system this emitter is adding particles to.
	void SetParticleSystem(b2ParticleSystem * const particleSystem)
	{
		m_particleSystem = particleSystem;
	}

	// Get the particle system this emitter is adding particle to.
	b2ParticleSystem* GetParticleSystem() const
	{
		return m_particleSystem;
	}

	// Set the callback that is called on the creation of each particle.
	void SetCallback(EmittedParticleCallback* const callback)
	{
		m_callback = callback;
	}

	// Get the callback that is called on the creation of each particle.
	EmittedParticleCallback* GetCallback() const
	{
		return m_callback;
	}

	// This class sets the group flags to b2_particleGroupCanBeEmpty so that
	// it isn't destroyed and clears the b2_particleGroupCanBeEmpty on the
	// group when the emitter no longer references it so that the group
	// can potentially be cleaned up.
	void SetGroup(b2ParticleGroup * const group)
	{
		if (m_group)
		{
			m_group->SetGroupFlags(m_group->GetGroupFlags() &
								   ~b2_particleGroupCanBeEmpty);
		}
		m_group = group;
		if (m_group)
		{
			m_group->SetGroupFlags(m_group->GetGroupFlags() |
								   b2_particleGroupCanBeEmpty);
		}
	}

	// Get the group particles should be created within.
	b2ParticleGroup* GetGroup() const
	{
		return m_group;
	}

	// dt is seconds that have passed, particleIndices is an optional pointer
	// to an array which tracks which particles have been created and
	// particleIndicesCount is the size of the particleIndices array.
	// This function returns the number of particles created during this
	// simulation step.
	int32 Step(const float32 dt, int32* const particleIndices,
			   const int32 particleIndicesCount)
	{
		b2Assert(m_particleSystem);
		int32 numberOfParticlesCreated = 0;
		// How many (fractional) particles should we have emitted this frame?
		m_emitRemainder += m_emitRate * dt;

		b2ParticleDef pd;
		pd.color = m_color;
		pd.flags = m_flags;
		pd.group = m_group;

		// Keep emitting particles on this frame until we only have a
		// fractional particle left.
		while (m_emitRemainder > 1.0f) {
			m_emitRemainder -= 1.0f;

			// Randomly pick a position within the emitter's radius.
			const float32 angle = Random() * 2.0f * b2_pi;
			// Distance from the center of the circle.
			const float32 distance = Random();
			b2Vec2 positionOnUnitCircle(sin(angle), cos(angle));

			// Initial position.
			pd.position.Set(
				m_origin.x + positionOnUnitCircle.x * distance * m_halfSize.x,
				m_origin.y + positionOnUnitCircle.y * distance * m_halfSize.y);
			// Send it flying
			pd.velocity = m_startingVelocity;
			if (m_speed != 0.0f)
			{
				pd.velocity += positionOnUnitCircle * m_speed;
			}

			const int32 particleIndex = m_particleSystem->CreateParticle(pd);
			if (m_callback)
			{
				m_callback->ParticleCreated(m_particleSystem, particleIndex);
			}
			if (particleIndices &&
				numberOfParticlesCreated < particleIndicesCount)
			{
				particleIndices[numberOfParticlesCreated] = particleIndex;
			}
			++numberOfParticlesCreated;
		}
		return numberOfParticlesCreated;
	}

private:
	// Calculate a random number 0.0f..1.0f.
	static float32 Random()
	{
		return ((float32)rand() / (float32)RAND_MAX);
	}

private:
	// Pointer to global world
	b2ParticleSystem *m_particleSystem;
	// Called for each created particle.
	EmittedParticleCallback *m_callback;
	// Center of particle emitter
	b2Vec2 m_origin;
	// Launch direction.
	b2Vec2 m_startingVelocity;
	// Speed particles are emitted
	float32 m_speed;
	// Half width / height of particle emitter
	b2Vec2 m_halfSize;
	// Particles per second
	float32 m_emitRate;
	// Initial color of particle emitted.
	b2ParticleColor m_color;
	// Number particles to emit on the next frame
	float32 m_emitRemainder;
	// Flags for created particles, see b2ParticleFlag.
	uint32 m_flags;
	// Group to put newly created particles in.
	b2ParticleGroup* m_group;
};

#endif  // PARTICLE_EMITTER_H


