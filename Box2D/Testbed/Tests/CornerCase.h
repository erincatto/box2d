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
#ifndef TESTS_CORNERCASE_H
#define TESTS_CORNERCASE_H

class CornerCase : public Test
{
public:

	CornerCase()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			// Construct a pathological corner intersection out of many
			// polygons to ensure there's no issue with particle oscillation
			// from many fixture contact impulses at the corner

			// left edge
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[] = {
					b2Vec2(-20.0f, 30.0f),
					b2Vec2(-20.0f, 0.0f),
					b2Vec2(-25.0f, 0.0f),
					b2Vec2(-25.0f, 30.f)
				};
				shape.Set(vertices, B2_ARRAY_SIZE(vertices));
				ground->CreateFixture(&shape, 0.0f);
			}

			float32 x, y;
			const float32 yrange=30.0f, ystep = yrange/10.0f,
							xrange=20.0f, xstep=xrange/2.0f;

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[] = {
					b2Vec2(-25.0f, 0.0f),
					b2Vec2(20.0f, 15.0f),
					b2Vec2(25.0f, 0.0f)
				};
				shape.Set(vertices, B2_ARRAY_SIZE(vertices));
				ground->CreateFixture(&shape, 0.0f);
			}

			for (x = -xrange; x < xrange; x += xstep)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[] = {
					b2Vec2(-25.0f, 0.0f),
					b2Vec2(x, 15.0f),
					b2Vec2(x+xstep, 15.0f)
				};
				shape.Set(vertices, B2_ARRAY_SIZE(vertices));
				ground->CreateFixture(&shape, 0.0f);
			}

			for (y = 0.0; y < yrange; y += ystep)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[] = {
					b2Vec2(25.0f, y),
					b2Vec2(25.0f, y+ystep),
					b2Vec2(20.0f, 15.0f)
				};
				shape.Set(vertices, B2_ARRAY_SIZE(vertices));
				ground->CreateFixture(&shape, 0.0f);
			}

		}

		m_particleSystem->SetRadius(1.0f);
		const uint32 particleType = TestMain::GetParticleParameterValue();

		{
			b2CircleShape shape;
			shape.m_p.Set(0, 35);
			shape.m_radius = 12;
			b2ParticleGroupDef pd;
			pd.flags = particleType;
			pd.shape = &shape;
			b2ParticleGroup* const group =
				m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}
	}

	static Test* Create()
	{
		return new CornerCase;
	}
};

#endif
