/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef POLY_SHAPES_H
#define POLY_SHAPES_H

const int32 k_maxBodies = 256;

class PolyShapes : public Test
{
public:
	PolyShapes()
	{
		// Ground body
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsEdge(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape);
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.5f, 0.0f);
			vertices[1].Set(0.5f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			polygons[0].Set(vertices, 3);
		}
		
		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.1f, 0.0f);
			vertices[1].Set(0.1f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			polygons[1].Set(vertices, 3);
		}

		{
			float32 w = 1.0f;
			float32 b = w / (2.0f + sqrtf(2.0f));
			float32 s = sqrtf(2.0f) * b;

			b2Vec2 vertices[8];
			vertices[0].Set(0.5f * s, 0.0f);
			vertices[1].Set(0.5f * w, b);
			vertices[2].Set(0.5f * w, b + s);
			vertices[3].Set(0.5f * s, w);
			vertices[4].Set(-0.5f * s, w);
			vertices[5].Set(-0.5f * w, b + s);
			vertices[6].Set(-0.5f * w, b);
			vertices[7].Set(-0.5f * s, 0.0f);

			polygons[2].Set(vertices, 8);
		}

		{
			polygons[3].SetAsBox(0.5f, 0.5f);
		}

		{
			circle.m_radius = 0.5f;
		}

		bodyIndex = 0;
		memset(bodies, 0, sizeof(bodies));
	}

	void Create(int32 index)
	{
		if (bodies[bodyIndex] != NULL)
		{
			m_world->DestroyBody(bodies[bodyIndex]);
			bodies[bodyIndex] = NULL;
		}

		b2BodyDef bd;

		float32 x = RandomFloat(-2.0f, 2.0f);
		bd.position.Set(x, 10.0f);
		bd.angle = RandomFloat(-b2_pi, b2_pi);

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		bodies[bodyIndex] = m_world->CreateBody(&bd);

		if (index < 4)
		{
			b2FixtureDef fd;
			fd.shape = polygons + index;
			fd.density = 1.0f;
			fd.friction = 0.3f;
			bodies[bodyIndex]->CreateFixture(&fd);
		}
		else
		{
			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			bodies[bodyIndex]->CreateFixture(&fd);
		}

		bodyIndex = (bodyIndex + 1) % k_maxBodies;
	}

	void DestroyBody()
	{
		for (int32 i = 0; i < k_maxBodies; ++i)
		{
			if (bodies[i] != NULL)
			{
				m_world->DestroyBody(bodies[i]);
				bodies[i] = NULL;
				return;
			}
		}
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
			Create(key - '1');
			break;

		case 'd':
			DestroyBody();
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		m_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += 15;
	}

	static Test* Create()
	{
		return new PolyShapes;
	}

	int32 bodyIndex;
	b2Body* bodies[k_maxBodies];
	b2PolygonShape polygons[4];
	b2CircleShape circle;
};

#endif
