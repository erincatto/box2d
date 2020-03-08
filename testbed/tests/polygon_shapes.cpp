// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "test.h"

/// This tests stacking. It also shows how to use b2World::Query
/// and b2TestOverlap.

/// This callback is called by b2World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
class PolygonShapesCallback : public b2QueryCallback
{
public:
	
	enum
	{
		e_maxCount = 4
	};

	PolygonShapesCallback()
	{
		m_count = 0;
	}

	/// Called for each fixture found in the query AABB.
	/// @return false to terminate the query.
	bool ReportFixture(b2Fixture* fixture) override
	{
		if (m_count == e_maxCount)
		{
			return false;
		}

		b2Body* body = fixture->GetBody();
		b2Shape* shape = fixture->GetShape();

		bool overlap = b2TestOverlap(shape, 0, &m_circle, 0, body->GetTransform(), m_transform);
			
		if (overlap)
		{
			b2Color color(0.95f, 0.95f, 0.6f);
			b2Vec2 center = body->GetWorldCenter();
			g_debugDraw->DrawPoint(center, 5.0f, color);
			++m_count;
		}

		return true;
	}

	b2CircleShape m_circle;
	b2Transform m_transform;
	b2Draw* g_debugDraw;
	int32 m_count;
};

class PolygonShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	PolygonShapes()
	{
		// Ground body
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.5f, 0.0f);
			vertices[1].Set(0.5f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			m_polygons[0].Set(vertices, 3);
		}
		
		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.1f, 0.0f);
			vertices[1].Set(0.1f, 0.0f);
			vertices[2].Set(0.0f, 1.5f);
			m_polygons[1].Set(vertices, 3);
		}

		{
			float w = 1.0f;
			float b = w / (2.0f + b2Sqrt(2.0f));
			float s = b2Sqrt(2.0f) * b;

			b2Vec2 vertices[8];
			vertices[0].Set(0.5f * s, 0.0f);
			vertices[1].Set(0.5f * w, b);
			vertices[2].Set(0.5f * w, b + s);
			vertices[3].Set(0.5f * s, w);
			vertices[4].Set(-0.5f * s, w);
			vertices[5].Set(-0.5f * w, b + s);
			vertices[6].Set(-0.5f * w, b);
			vertices[7].Set(-0.5f * s, 0.0f);

			m_polygons[2].Set(vertices, 8);
		}

		{
			m_polygons[3].SetAsBox(0.5f, 0.5f);
		}

		{
			m_circle.m_radius = 0.5f;
		}

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));
	}

	void Create(int32 index)
	{
		if (m_bodies[m_bodyIndex] != NULL)
		{
			m_world->DestroyBody(m_bodies[m_bodyIndex]);
			m_bodies[m_bodyIndex] = NULL;
		}

		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		float x = RandomFloat(-2.0f, 2.0f);
		bd.position.Set(x, 10.0f);
		bd.angle = RandomFloat(-b2_pi, b2_pi);

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = m_world->CreateBody(&bd);

		if (index < 4)
		{
			b2FixtureDef fd;
			fd.shape = m_polygons + index;
			fd.density = 1.0f;
			fd.friction = 0.3f;
			m_bodies[m_bodyIndex]->CreateFixture(&fd);
		}
		else
		{
			b2FixtureDef fd;
			fd.shape = &m_circle;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			m_bodies[m_bodyIndex]->CreateFixture(&fd);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
	}

	void DestroyBody()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (m_bodies[i] != NULL)
			{
				m_world->DestroyBody(m_bodies[i]);
				m_bodies[i] = NULL;
				return;
			}
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
		case GLFW_KEY_5:
			Create(key - GLFW_KEY_1);
			break;

		case GLFW_KEY_A:
			for (int32 i = 0; i < e_maxBodies; i += 2)
			{
				if (m_bodies[i])
				{
					bool enabled = m_bodies[i]->IsEnabled();
					m_bodies[i]->SetEnabled(!enabled);
				}
			}
			break;

		case GLFW_KEY_D:
			DestroyBody();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		PolygonShapesCallback callback;
		callback.m_circle.m_radius = 2.0f;
		callback.m_circle.m_p.Set(0.0f, 1.1f);
		callback.m_transform.SetIdentity();
		callback.g_debugDraw = &g_debugDraw;

		b2AABB aabb;
		callback.m_circle.ComputeAABB(&aabb, callback.m_transform, 0);

		m_world->QueryAABB(&callback, aabb);

		b2Color color(0.4f, 0.7f, 0.8f);
		g_debugDraw.DrawCircle(callback.m_circle.m_p, callback.m_circle.m_radius, color);

		g_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff, maximum of %d overlaps detected", PolygonShapesCallback::e_maxCount);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Press 'a' to enable/disable some bodies");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Press 'd' to destroy a body");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new PolygonShapes;
	}

	int32 m_bodyIndex;
	b2Body* m_bodies[e_maxBodies];
	b2PolygonShape m_polygons[4];
	b2CircleShape m_circle;
};

static int testIndex = RegisterTest("Geometry", "Polygon Shapes", PolygonShapes::Create);
