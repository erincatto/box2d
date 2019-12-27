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
#include "box2d/b2_time_of_impact.h"

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		m_shapeA.SetAsBox(25.0f, 5.0f);
		m_shapeB.SetAsBox(2.5f, 2.5f);
	}

	static Test* Create()
	{
		return new TimeOfImpact;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2Sweep sweepA;
		sweepA.c0.Set(24.0f, -60.0f);
		sweepA.a0 = 2.95f;
		sweepA.c = sweepA.c0;
		sweepA.a = sweepA.a0;
		sweepA.localCenter.SetZero();

		b2Sweep sweepB;
		sweepB.c0.Set(53.474274f, -50.252514f);
		sweepB.a0 = 513.36676f; // - 162.0f * b2_pi;
		sweepB.c.Set(54.595478f, -51.083473f);
		sweepB.a = 513.62781f; //  - 162.0f * b2_pi;
		sweepB.localCenter.SetZero();

		//sweepB.a0 -= 300.0f * b2_pi;
		//sweepB.a -= 300.0f * b2_pi;

		b2TOIInput input;
		input.proxyA.Set(&m_shapeA, 0);
		input.proxyB.Set(&m_shapeB, 0);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0f;

		b2TOIOutput output;

		b2TimeOfImpact(&output, &input);

		g_debugDraw.DrawString(5, m_textLine, "toi = %g", output.t);
		m_textLine += m_textIncrement;

		extern int32 b2_toiMaxIters, b2_toiMaxRootIters;
		g_debugDraw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", b2_toiMaxIters, b2_toiMaxRootIters);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		b2Transform transformA;
		sweepA.GetTransform(&transformA, 0.0f);
		for (int32 i = 0; i < m_shapeA.m_count; ++i)
		{
			vertices[i] = b2Mul(transformA, m_shapeA.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeA.m_count, b2Color(0.9f, 0.9f, 0.9f));

		b2Transform transformB;
		sweepB.GetTransform(&transformB, 0.0f);
		
		//b2Vec2 localPoint(2.0f, -0.1f);

		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.5f, 0.9f, 0.5f));

		sweepB.GetTransform(&transformB, output.t);
		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.5f, 0.7f, 0.9f));

		sweepB.GetTransform(&transformB, 1.0f);
		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.9f, 0.5f, 0.5f));

#if 0
		for (float t = 0.0f; t < 1.0f; t += 0.1f)
		{
			sweepB.GetTransform(&transformB, t);
			for (int32 i = 0; i < m_shapeB.m_count; ++i)
			{
				vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, b2Color(0.9f, 0.5f, 0.5f));
		}
#endif
	}

	b2PolygonShape m_shapeA;
	b2PolygonShape m_shapeB;
};

static int testIndex = RegisterTest("Collision", "Time of Impact", TimeOfImpact::Create);
