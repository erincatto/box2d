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
#ifndef SOUP_H
#define SOUP_H

#include <string.h>
#include <memory.h>

class Soup : public Test
{
public:

	Soup()
	{
		// Disable the selection of wall and barrier particles for this test.
		InitializeParticleParameters(b2_wallParticle | b2_barrierParticle);

		{
			b2BodyDef bd;
			m_ground = m_world->CreateBody(&bd);

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(-4, -1),
					b2Vec2(4, -1),
					b2Vec2(4, 0),
					b2Vec2(-4, 0)};
				shape.Set(vertices, 4);
				m_ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(-4, -0.1f),
					b2Vec2(-2, -0.1f),
					b2Vec2(-2, 2),
					b2Vec2(-4, 3)};
				shape.Set(vertices, 4);
				m_ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(2, -0.1f),
					b2Vec2(4, -0.1f),
					b2Vec2(4, 3),
					b2Vec2(2, 2)};
				shape.Set(vertices, 4);
				m_ground->CreateFixture(&shape, 0.0f);
			}
		}

		m_particleSystem->SetRadius(0.035f);
		{
			b2PolygonShape shape;
			shape.SetAsBox(2, 1, b2Vec2(0, 1), 0);
			b2ParticleGroupDef pd;
			pd.shape = &shape;
			pd.flags = TestMain::GetParticleParameterValue();
			b2ParticleGroup * const group =
				m_particleSystem->CreateParticleGroup(pd);
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
			shape.m_p.Set(0, 0.5f);
			shape.m_radius = 0.1f;
			body->CreateFixture(&shape, 0.1f);
			m_particleSystem->DestroyParticlesInShape(shape,
													  body->GetTransform());
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 0.1f, b2Vec2(-1, 0.5f), 0);
			body->CreateFixture(&shape, 0.1f);
			m_particleSystem->DestroyParticlesInShape(shape,
													  body->GetTransform());
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2PolygonShape shape;
			shape.SetAsBox(0.1f, 0.1f, b2Vec2(1, 0.5f), 0.5f);
			body->CreateFixture(&shape, 0.1f);
			m_particleSystem->DestroyParticlesInShape(shape,
													  body->GetTransform());
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2EdgeShape shape;
			shape.Set(b2Vec2(0, 2), b2Vec2(0.1f, 2.1f));
			body->CreateFixture(&shape, 1);
			b2MassData massData =
				{0.1f, 0.5f * (shape.m_vertex1 + shape.m_vertex2), 0.0f};
			body->SetMassData(&massData);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2EdgeShape shape;
			shape.Set(b2Vec2(0.3f, 2.0f), b2Vec2(0.4f, 2.1f));
			body->CreateFixture(&shape, 1);
			b2MassData massData =
				{0.1f, 0.5f * (shape.m_vertex1 + shape.m_vertex2), 0.0f};
			body->SetMassData(&massData);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);
			b2EdgeShape shape;
			shape.Set(b2Vec2(-0.3f, 2.1f), b2Vec2(-0.2f, 2.0f));
			body->CreateFixture(&shape, 1);
			b2MassData massData =
				{0.1f, 0.5f * (shape.m_vertex1 + shape.m_vertex2), 0.0f};
			body->SetMassData(&massData);
		}
	}

	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

	static Test* Create()
	{
		return new Soup;
	}

protected:
	b2Body* m_ground;
};

#endif
