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
#ifndef TESTS_RAMP_H
#define TESTS_RAMP_H

class Ramp : public Test
{
public:

	Ramp()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			// Construct a ramp out of many polygons to ensure there's no
			// issue with particles moving across vertices

			float32 x, y;
			const float32 xstep = 5.0, ystep = 5.0;

			for (y = 30.0; y > 0.0; y -= ystep)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[3] = {
					b2Vec2(-25.0, y),
					b2Vec2(-25.0, y-ystep),
					b2Vec2(0.0, 15.0)};
				shape.Set(vertices, 3);
				ground->CreateFixture(&shape, 0.0f);
			}

			for (x = -25.0; x < 25.0; x += xstep)
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[3] = {
					b2Vec2(x, 0.0),
					b2Vec2(x+xstep, 0.0),
					b2Vec2(0.0, 15.0)};
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

		{
			b2CircleShape shape;
			shape.m_p.Set(-20, 33);
			shape.m_radius = 3;
			b2ParticleGroupDef pd;
			pd.flags = particleType;
			pd.shape = &shape;
			b2ParticleGroup* const group = m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}
	}

	static Test* Create()
	{
		return new Ramp;
	}
};

#endif
