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
#ifndef PARTICLES_H
#define PARTICLES_H

class Particles : public Test
{
public:

	Particles()
	{

		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(-4, -1),
					b2Vec2(4, -1),
					b2Vec2(4, 0),
					b2Vec2(-4, 0)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(-4, -0.1f),
					b2Vec2(-2, -0.1f),
					b2Vec2(-2, 2),
					b2Vec2(-4, 3)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(2, -0.1f),
					b2Vec2(4, -0.1f),
					b2Vec2(4, 3),
					b2Vec2(2, 2)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}
		}

		m_particleSystem->SetRadius(0.035f);
		const uint32 particleType = TestMain::GetParticleParameterValue();
		m_particleSystem->SetDamping(0.2f);

		{
			b2CircleShape shape;
			shape.m_p.Set(0, 3);
			shape.m_radius = 2;
			b2ParticleGroupDef pd;
			pd.flags = particleType;
			pd.shape = &shape;
			b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(pd);
			if (pd.flags & b2_colorMixingParticle)
			{
				ColorParticleGroup(group, 0);
			}
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2CircleShape shape;
			shape.m_p.Set(0, 8);
			shape.m_radius = 0.5f;
			body->CreateFixture(&shape, 0.5f);
		}
	}

	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new Particles;
	}
};

#endif
