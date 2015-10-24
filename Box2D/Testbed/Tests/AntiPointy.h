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
#ifndef ANTIPOINTY_H
#define ANTIPOINTY_H


// Test the behavior of particles falling onto a concave ambiguous Body
// contact fixture junction.
class AntiPointy : public Test
{
public:
	AntiPointy()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			// Construct a valley out of many polygons to ensure there's no
			// issue with particles falling directly on an ambiguous set of
			// fixture corners.

			float32 i;
			const float32 step = 1.0;

			for (i = -10.0; i < 10.0; i+=step)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[3] = {
					b2Vec2(i, -10.0),
					b2Vec2(i+step, -10.0),
					b2Vec2(0.0, 15.0)};
				shape.Set(vertices, 3);
				ground->CreateFixture(&shape, 0.0f);
			}
			for (i = -10.0; i < 35.0; i+=step)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[3] = {
					b2Vec2(-10.0, i),
					b2Vec2(-10.0, i+step),
					b2Vec2(0.0, 15.0)};
				shape.Set(vertices, 3);
				ground->CreateFixture(&shape, 0.0f);

				const b2Vec2 vertices2[3] = {
					b2Vec2(10.0, i),
					b2Vec2(10.0, i+step),
					b2Vec2(0.0, 15.0)};
				shape.Set(vertices2, 3);
				ground->CreateFixture(&shape, 0.0f);
			}
		}

		// Cap the number of generated particles or we'll fill forever
		m_particlesToCreate = 300;

		m_particleSystem->SetRadius(0.25f);
		const uint32 particleType = TestMain::GetParticleParameterValue();
		if (particleType == b2_waterParticle)
		{
			m_particleSystem->SetDamping(0.2f);
		}
	}

	virtual void Step(Settings *settings) // override from Test
	{
		Test::Step(settings);

		if (m_particlesToCreate <= 0)
		{
			return;
		}

		--m_particlesToCreate;

		int32 flags = TestMain::GetParticleParameterValue();
		b2ParticleDef pd;

		pd.position.Set(0.0, 40.0);
		pd.velocity .Set(0.0, -1.0);
		pd.flags = flags;

		if (flags & (b2_springParticle | b2_elasticParticle))
		{
			int32 count = m_particleSystem->GetParticleCount();
			pd.velocity.Set(count & 1 ? -1.0f : 1.0f, -5.0f);
			pd.flags |= b2_reactiveParticle;
		}

		m_particleSystem->CreateParticle(pd);
	}

	static Test* Create()
	{
		return new AntiPointy;
	}

private:
	int32 m_particlesToCreate;
};

#endif
