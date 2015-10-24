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
#ifndef POINTY_H
#define POINTY_H

// Test behavior when particles fall on a convex ambigious Body
// contact fixture junction.
class Pointy : public Test
{
public:
	Pointy()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			// Construct a triangle out of many polygons to ensure there's no
			// issue with particles falling directly on an ambiguous corner

			const float32 xstep = 1.0;
			for (float32 x = -10.0; x < 10.0; x += xstep)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[3] = {
					b2Vec2(x, -10.0),
					b2Vec2(x+xstep, -10.0),
					b2Vec2(0.0, 25.0)};
				shape.Set(vertices, 3);
				ground->CreateFixture(&shape, 0.0f);
			}
		}

		m_particleSystem->SetRadius(0.25f);
		const uint32 particleType = TestMain::GetParticleParameterValue();
		if (particleType == b2_waterParticle)
		{
			m_particleSystem->SetDamping(0.2f);
		}

		// Create killfield shape and transform
		m_killfieldShape = b2PolygonShape();
		m_killfieldShape.SetAsBox(50.0, 1.0);

		// Put this at the bottom of the world
		m_killfieldTransform = b2Transform();
		b2Vec2 loc = b2Vec2(-25, 1);
		m_killfieldTransform.Set(loc, 0);

	}

	virtual void Step(Settings *settings) // override from Test
	{
		Test::Step(settings);

		int32 flags = TestMain::GetParticleParameterValue();
		b2ParticleDef pd;

		pd.position.Set(0.0, 33.0);
		pd.velocity.Set(0.0, -1.0);
		pd.flags = flags;

		if (flags & (b2_springParticle | b2_elasticParticle))
		{
			int32 count = m_particleSystem->GetParticleCount();
			pd.velocity.Set(count & 1 ? -1.0f : 1.0f, -5.0f);
			pd.flags |= b2_reactiveParticle;
		}

		m_particleSystem->CreateParticle(pd);

		// kill every particle near the bottom of the screen
		m_particleSystem->DestroyParticlesInShape(m_killfieldShape, m_killfieldTransform);
	}

	static Test* Create()
	{
		return new Pointy;
	}

private:
	b2PolygonShape m_killfieldShape;
	b2Transform m_killfieldTransform;
};

#endif
