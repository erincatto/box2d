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

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		{
			m_transformA.SetIdentity();
			m_transformA.p.Set(0.0f, -0.2f);
			m_polygonA.SetAsBox(10.0f, 0.2f);
		}

		{
			m_positionB.Set(12.017401f, 0.13678508f);
			m_angleB = -0.0109265f;
			m_transformB.Set(m_positionB, m_angleB);

			m_polygonB.SetAsBox(2.0f, 0.1f);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2DistanceInput input;
		input.proxyA.Set(&m_polygonA, 0);
		input.proxyB.Set(&m_polygonB, 0);
		input.transformA = m_transformA;
		input.transformB = m_transformB;
		input.useRadii = true;
		b2SimplexCache cache;
		cache.count = 0;
		b2DistanceOutput output;
		b2Distance(&output, &cache, &input);

		g_debugDraw.DrawString(5, m_textLine, "distance = %g", output.distance);
		m_textLine += m_textIncrement;

		g_debugDraw.DrawString(5, m_textLine, "iterations = %d", output.iterations);
		m_textLine += m_textIncrement;

		{
			b2Color color(0.9f, 0.9f, 0.9f);
			b2Vec2 v[b2_maxPolygonVertices];
			for (int32 i = 0; i < m_polygonA.m_count; ++i)
			{
				v[i] = b2Mul(m_transformA, m_polygonA.m_vertices[i]);
			}
			g_debugDraw.DrawPolygon(v, m_polygonA.m_count, color);

			for (int32 i = 0; i < m_polygonB.m_count; ++i)
			{
				v[i] = b2Mul(m_transformB, m_polygonB.m_vertices[i]);
			}
			g_debugDraw.DrawPolygon(v, m_polygonB.m_count, color);
		}

		b2Vec2 x1 = output.pointA;
		b2Vec2 x2 = output.pointB;

		b2Color c1(1.0f, 0.0f, 0.0f);
		g_debugDraw.DrawPoint(x1, 4.0f, c1);

		b2Color c2(1.0f, 1.0f, 0.0f);
		g_debugDraw.DrawPoint(x2, 4.0f, c2);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_positionB.x -= 0.1f;
			break;

		case GLFW_KEY_D:
			m_positionB.x += 0.1f;
			break;

		case GLFW_KEY_S:
			m_positionB.y -= 0.1f;
			break;

		case GLFW_KEY_W:
			m_positionB.y += 0.1f;
			break;

		case GLFW_KEY_Q:
			m_angleB += 0.1f * b2_pi;
			break;

		case GLFW_KEY_E:
			m_angleB -= 0.1f * b2_pi;
			break;
		}

		m_transformB.Set(m_positionB, m_angleB);
	}

	b2Vec2 m_positionB;
	float m_angleB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;
};

static int testIndex = RegisterTest("Geometry", "Distance Test", DistanceTest::Create);
