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
#ifndef MULTIPLE_PARTICLE_SYSTEMS_H
#define MULTIPLE_PARTICLE_SYSTEMS_H
#include "../Framework/ParticleEmitter.h"

// The "Multiple Systems" test uses two particle emitters to push a rigid body
// in opposing directions showing that particles from each system can interact
// with the same body and at the same time not interact with each other.
class MultipleParticleSystems : public Test
{
public:
	MultipleParticleSystems()
	{
		// Configure the default particle system's parameters.
		m_particleSystem->SetRadius(0.05f);
		m_particleSystem->SetMaxParticleCount(k_maxParticleCount);
		m_particleSystem->SetDestructionByAge(true);

		// Create a secondary particle system.
		b2ParticleSystemDef particleSystemDef;
		particleSystemDef.radius = m_particleSystem->GetRadius();
		particleSystemDef.destroyByAge = true;
		m_particleSystem2 = m_world->CreateParticleSystem(&particleSystemDef);
		m_particleSystem2->SetMaxParticleCount(k_maxParticleCount);

		// Don't restart the test when changing particle types.
		TestMain::SetRestartOnParticleParameterChange(false);

		// Create the ground.
		{
			b2BodyDef bd;
			b2Body* const ground = m_world->CreateBody(&bd);
			b2PolygonShape shape;
			shape.SetAsBox(5.0f, 0.1f);
			ground->CreateFixture(&shape, 0.0f);
		}

		// Create a dynamic body to push around.
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2PolygonShape shape;
			b2Vec2 center(0.0f, 1.2f);
			shape.SetAsBox(k_dynamicBoxSize.x, k_dynamicBoxSize.y, center,
						   0.0f);
			body->CreateFixture(&shape, 0.0f);
			b2MassData massData = {k_boxMass, center, 0.0f};
			body->SetMassData(&massData);
		}

		// Initialize the emitters.
		for (uint32 i = 0; i < B2_ARRAY_SIZE(m_emitters); ++i)
		{
			const float32 mirrorAlongY = i & 1 ? -1.0f : 1.0f;
			RadialEmitter& emitter = m_emitters[i];
			emitter.SetPosition(b2Vec2(k_emitterPosition.x * mirrorAlongY,
									   k_emitterPosition.y));
			emitter.SetSize(k_emitterSize);
			emitter.SetVelocity(b2Vec2(k_emitterVelocity.x * mirrorAlongY,
									   k_emitterVelocity.y));
			emitter.SetEmitRate(k_emitRate);
			emitter.SetColor(i & 1 ? k_rightEmitterColor : k_leftEmitterColor);
			emitter.SetParticleSystem(
				i & 1 ? m_particleSystem2 : m_particleSystem);
		}
	}

	// Run a simulation step.
	void Step(Settings* settings)
	{
		const float32 dt = 1.0f / settings->hz;
		Test::Step(settings);
		for (uint32 i = 0; i < B2_ARRAY_SIZE(m_emitters); ++i)
		{
			m_emitters[i].Step(dt, NULL, 0);
		}
	}

	float32 GetDefaultViewZoom() const
	{
		return 0.2f;
	}

	// Create the multiple particle systems test.
	static Test* Create()
	{
		return new MultipleParticleSystems;
	}

private:
	b2ParticleSystem* m_particleSystem2;
	RadialEmitter m_emitters[2];

	// Maximum number of particles per system.
	static const int32 k_maxParticleCount;
	// Size of the box which is pushed around by particles.
	static const b2Vec2 k_dynamicBoxSize;
	// Mass of the box.
	static const float32 k_boxMass;
	// Emit rate of the emitters in particles per second.
	static const float32 k_emitRate;
	// Location of the left emitter (the position of the right one is
	// mirrored along the y-axis).
	static const b2Vec2 k_emitterPosition;
	// Starting velocity of particles from the left emitter (the velocity
	// of particles from the right emitter are mirrored along the y-axis).
	static const b2Vec2 k_emitterVelocity;
	// Size of particle emitters.
	static const b2Vec2 k_emitterSize;
	// Color of the left emitter's particles.
	static const b2ParticleColor k_leftEmitterColor;
	// Color of the right emitter's particles.
	static const b2ParticleColor k_rightEmitterColor;
};

const int32 MultipleParticleSystems::k_maxParticleCount = 500;
const b2Vec2 MultipleParticleSystems::k_dynamicBoxSize(0.5f, 0.5f);
const float32 MultipleParticleSystems::k_boxMass = 1.0f;
const float32 MultipleParticleSystems::k_emitRate = 100.0f;
const b2Vec2 MultipleParticleSystems::k_emitterPosition(-5.0f, 4.0f);
const b2Vec2 MultipleParticleSystems::k_emitterVelocity(7.0f, -4.0f);
const b2Vec2 MultipleParticleSystems::k_emitterSize(1.0f, 1.0f);
const b2ParticleColor MultipleParticleSystems::k_leftEmitterColor(
	0x22, 0x33, 0xff, 0xff);
const b2ParticleColor MultipleParticleSystems::k_rightEmitterColor(
	0xff, 0x22, 0x11, 0xff);

#endif  // MULTIPLE_PARTICLE_SYSTEMS_H
