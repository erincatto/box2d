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
#include "box2d/b2_distance.h"

class ShapeCast : public Test
{
public:
    enum
    {
        e_vertexCount = 8
    };

	ShapeCast()
	{
#if 1
        m_vAs[0].Set(-0.5f, 1.0f);
        m_vAs[1].Set(0.5f, 1.0f);
        m_vAs[2].Set(0.0f, 0.0f);
        m_countA = 3;
        m_radiusA = b2_polygonRadius;

        m_vBs[0].Set(-0.5f, -0.5f);
        m_vBs[1].Set(0.5f, -0.5f);
        m_vBs[2].Set(0.5f, 0.5f);
        m_vBs[3].Set(-0.5f, 0.5f);
        m_countB = 4;
        m_radiusB = b2_polygonRadius;
#else
        m_vAs[0].Set(0.0f, 0.0f);
        m_countA = 1;
        m_radiusA = 0.5f;

        m_vBs[0].Set(0.0f, 0.0f);
        m_countB = 1;
        m_radiusB = 0.5f;
#endif
	}

	static Test* Create()
	{
		return new ShapeCast;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2Transform transformA;
		transformA.p = b2Vec2(0.0f, 0.25f);
		transformA.q.SetIdentity();

		b2Transform transformB;
		transformB.SetIdentity();

		b2ShapeCastInput input;
		input.proxyA.Set(m_vAs, m_countA, m_radiusA);
		input.proxyB.Set(m_vBs, m_countB, m_radiusB);
		input.transformA = transformA;
		input.transformB = transformB;
		input.translationB.Set(8.0f, 0.0f);

		b2ShapeCastOutput output;

		bool hit = b2ShapeCast(&output, &input);

        b2Transform transformB2;
        transformB2.q = transformB.q;
        transformB2.p = transformB.p + output.lambda * input.translationB;
        
        b2DistanceInput distanceInput;
        distanceInput.proxyA.Set(m_vAs, m_countA, m_radiusA);
        distanceInput.proxyB.Set(m_vBs, m_countB, m_radiusB);
        distanceInput.transformA = transformA;
        distanceInput.transformB = transformB2;
        distanceInput.useRadii = false;
        b2SimplexCache simplexCache;
        simplexCache.count = 0;
        b2DistanceOutput distanceOutput;
        
        b2Distance(&distanceOutput, &simplexCache, &distanceInput);

		g_debugDraw.DrawString(5, m_textLine, "hit = %s, iters = %d, lambda = %g, distance = %g",
            hit ? "true" : "false", output.iterations, output.lambda, distanceOutput.distance);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		for (int32 i = 0; i < m_countA; ++i)
		{
			vertices[i] = b2Mul(transformA, m_vAs[i]);
		}
        //g_debugDraw.DrawCircle(vertices[0], m_radiusA, b2Color(0.9f, 0.9f, 0.9f));
		g_debugDraw.DrawPolygon(vertices, m_countA, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2Mul(transformB, m_vBs[i]);
		}
        //g_debugDraw.DrawCircle(vertices[0], m_radiusB, b2Color(0.5f, 0.9f, 0.5f));
        g_debugDraw.DrawPolygon(vertices, m_countB, b2Color(0.5f, 0.9f, 0.5f));

		for (int32 i = 0; i < m_countB; ++i)
		{
			vertices[i] = b2Mul(transformB2, m_vBs[i]);
		}
        //g_debugDraw.DrawCircle(vertices[0], m_radiusB, b2Color(0.5f, 0.7f, 0.9f));
        g_debugDraw.DrawPolygon(vertices, m_countB, b2Color(0.5f, 0.7f, 0.9f));

		if (hit)
		{
			b2Vec2 p1 = output.point;
			g_debugDraw.DrawPoint(p1, 10.0f, b2Color(0.9f, 0.3f, 0.3f));
			b2Vec2 p2 = p1 + output.normal;
			g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.3f, 0.3f));
		}
	}

    b2Vec2 m_vAs[b2_maxPolygonVertices];
    int32 m_countA;
    float m_radiusA;

    b2Vec2 m_vBs[b2_maxPolygonVertices];
    int32 m_countB;
    float m_radiusB;
};

static int testIndex = RegisterTest("Collision", "Shape Cast", ShapeCast::Create);
