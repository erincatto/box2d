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
#ifndef PARTICLES_ELASTIC_H
#define PARTICLES_ELASTIC_H

class ElasticParticles : public Test
{
public:

	ElasticParticles()
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
					b2Vec2(-4, 2)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(2, -0.1f),
					b2Vec2(4, -0.1f),
					b2Vec2(4, 2),
					b2Vec2(2, 2)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}
		}

		m_particleSystem->SetRadius(0.035f);

		{
			b2CircleShape shape;
			shape.m_p.Set(0, 3);
			shape.m_radius = 0.5f;
			b2ParticleGroupDef pd;
			pd.flags = b2_springParticle;
			pd.groupFlags = b2_solidParticleGroup;
			pd.shape = &shape;
			pd.color.Set(255, 0, 0, 255);
			m_particleSystem->CreateParticleGroup(pd);
		}

		{
			b2CircleShape shape;
			shape.m_p.Set(-1, 3);
			shape.m_radius = 0.5f;
			b2ParticleGroupDef pd;
			pd.flags = b2_elasticParticle;
			pd.groupFlags = b2_solidParticleGroup;
			pd.shape = &shape;
			pd.color.Set(0, 255, 0, 255);
			m_particleSystem->CreateParticleGroup(pd);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(1, 0.5f);
			b2ParticleGroupDef pd;
			pd.flags = b2_elasticParticle;
			pd.groupFlags = b2_solidParticleGroup;
			pd.position.Set(1, 4);
			pd.angle = -0.5f;
			pd.angularVelocity = 2.0f;
			pd.shape = &shape;
			pd.color.Set(0, 0, 255, 255);
			m_particleSystem->CreateParticleGroup(pd);
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
		return new ElasticParticles;
	}
};

#endif
