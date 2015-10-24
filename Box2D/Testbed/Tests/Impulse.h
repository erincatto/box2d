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
#ifndef IMPULSE_H
#define IMPULSE_H

// Call b2ParticleSystem::ApplyLinearImpulse or b2ParticleSystem::ApplyForce
// to all particles, in the direction indicated by the user.
class Impulse : public Test
{
	enum {
		kBoxLeft = -2,
		kBoxRight = 2,
		kBoxBottom = 0,
		kBoxTop = 4,
	};

public:
	Impulse()
	{
		m_useLinearImpulse = false;

		// Create the containing box.
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			const b2Vec2 box[] = {
				b2Vec2(kBoxLeft, kBoxBottom),
				b2Vec2(kBoxRight, kBoxBottom),
				b2Vec2(kBoxRight, kBoxTop),
				b2Vec2(kBoxLeft, kBoxTop)
			};
			b2ChainShape shape;
			shape.CreateLoop(box, sizeof(box) / sizeof(box[0]));
			ground->CreateFixture(&shape, 0.0f);
		}

		m_particleSystem->SetRadius(0.025f);
		m_particleSystem->SetDamping(0.2f);

		// Create the particles.
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.8f, 1.0f, b2Vec2(0.0f, 1.01f), 0);
			b2ParticleGroupDef pd;
			pd.flags = TestMain::GetParticleParameterValue();
			pd.shape = &shape;
			b2ParticleGroup* const group =
				m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}
	}

	void MouseUp(const b2Vec2& p)
	{
		Test::MouseUp(p);

		// Apply an impulse to the particles.
		const bool isInsideBox = kBoxLeft <= p.x && p.x <= kBoxRight &&
								 kBoxBottom <= p.y && p.y <= kBoxTop;
		if (isInsideBox) {
			const b2Vec2 kBoxCenter(0.5f * (kBoxLeft + kBoxRight),
									0.5f * (kBoxBottom + kBoxTop));
			b2Vec2 direction = p - kBoxCenter;
			direction.Normalize();
			ApplyImpulseOrForce(direction);
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'l':
			{
				m_useLinearImpulse = true;
			}
			break;

		case 'f':
			{
				m_useLinearImpulse = false;
			}
			break;
		}
	}

	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new Impulse;
	}

private:
	// Move the particles a bit more in 'direction' by apply a force or impulse
	// (depending on m_useLinearImpuse).
	void ApplyImpulseOrForce(const b2Vec2& direction)
	{
		b2ParticleSystem* particleSystem = m_world->GetParticleSystemList();
		b2ParticleGroup* particleGroup = particleSystem->GetParticleGroupList();
		const int32 numParticles = particleGroup->GetParticleCount();

		if (m_useLinearImpulse)
		{
			const float kImpulseMagnitude = 0.005f;
			const b2Vec2 impulse = kImpulseMagnitude * direction *
				(float32)numParticles;
			particleGroup->ApplyLinearImpulse(impulse);
		}
		else
		{
			const float kForceMagnitude = 1.0f;
			const b2Vec2 force = kForceMagnitude * direction *
				(float32)numParticles;
			particleGroup->ApplyForce(force);
		}
	}

	bool m_useLinearImpulse;
};
#endif
