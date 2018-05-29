/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

class ShapeCast : public Test
{
public:
	ShapeCast()
	{
		m_shapeA.SetAsBox(0.5f, 0.5f);
		m_shapeB.SetAsBox(0.5f, 0.5f);
	}

	static Test* Create()
	{
		return new ShapeCast;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		b2Transform transformA;
		transformA.p = b2Vec2(2.0f, 0.0f);
		transformA.q.SetIdentity();

		b2Transform transformB;
		transformB.SetIdentity();

		b2ShapeCastInput input;
		input.proxyA.Set(&m_shapeA, 0);
		input.proxyB.Set(&m_shapeB, 0);
		input.transformA = transformA;
		input.transformB = transformB;
		input.translationB.Set(4.0f, 0.0f);

		b2ShapeCastOutput output;

		bool hit = b2ShapeCast(&output, &input);

		g_debugDraw.DrawString(5, m_textLine, "hit = %s, lambda = %g", hit ? "true" : "false", output.lambda);
		m_textLine += DRAW_STRING_NEW_LINE;

		b2Vec2 vertices[b2_maxPolygonVertices];

		for (int32 i = 0; i < m_shapeA.m_count; ++i)
		{
			vertices[i] = b2Mul(transformA, m_shapeA.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeA.m_count, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.5f, 0.9f, 0.5f));

		b2Transform transformBHit = transformB;
		transformBHit.p = transformB.p + output.lambda * input.translationB;
		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul(transformBHit, m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.5f, 0.7f, 0.9f));

		if (hit)
		{
			b2Vec2 p1 = output.point;
			g_debugDraw.DrawPoint(p1, 10.0f, b2Color(0.9f, 0.3f, 0.3f));
			b2Vec2 p2 = p1 + output.normal;
			g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.3f, 0.3f));
		}
	}

	b2PolygonShape m_shapeA;
	b2PolygonShape m_shapeB;
};
