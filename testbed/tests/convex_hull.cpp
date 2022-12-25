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

class ConvexHull : public Test
{
public:
	enum
	{
		e_count = b2_maxPolygonVertices
	};

	ConvexHull()
	{
		m_generation = 0;
		m_auto = false;
		m_bulk = false;
		Generate();
	}

	void Generate()
	{
#if 0
		m_points[0] = { 5.65314484f, 0.204832315f };
		m_points[1] = {-5.65314484f, -0.204832315f };
		m_points[2] = {2.34463644f, 1.15731204f };
		m_points[3] = {0.0508846045f, 3.23230696f };
		m_points[4] = {-5.65314484f, -0.204832315f };
		m_points[5] = {-5.65314484f, -0.204832315f };
		m_points[6] = {3.73758054f, -1.11098099f };
		m_points[7] = {1.33504069f, -4.43795443f };

		m_count = e_count;
#else

		float angle = b2_pi * RandomFloat();
		b2Rot r(angle);

		b2Vec2 lowerBound(-4.0f, -4.0f);
		b2Vec2 upperBound(4.0f, 4.0f);

		for (int32 i = 0; i < e_count; ++i)
		{
			float x = 10.0f * RandomFloat();
			float y = 10.0f * RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			b2Vec2 v(x, y);
			v = b2Clamp(v, lowerBound, upperBound);
			m_points[i] = b2Mul(r, v);
		}

		m_count = e_count;
#endif

		m_generation += 1;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_auto = !m_auto;
			break;

		case GLFW_KEY_B:
			m_bulk = !m_bulk;
			break;

		case GLFW_KEY_G:
			Generate();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Options: generate(g), auto(a), bulk(b)");
		m_textLine += m_textIncrement;
		
		b2Hull hull;
		bool valid = false;
		float milliseconds = 0.0f;

		if (m_bulk)
		{
#if 1
			// defect hunting
			for (int32 i = 0; i < 10000; ++i)
			{
				Generate();
				hull = b2ComputeHull(m_points, m_count);
				if (hull.count == 0)
				{
					//m_bulk = false;
					//break;
					continue;
				}

				valid = b2ValidateHull(hull);
				if (valid == false || m_bulk == false)
				{
					m_bulk = false;
					break;
				}
			}
#else
			// performance
			Generate();
			b2Timer timer;
			for (int32 i = 0; i < 1000000; ++i)
			{
				hull = b2ComputeHull(m_points, m_count);
			}
			valid = hull.count > 0;
			milliseconds = timer.GetMilliseconds();
#endif
		}
		else
		{
			if (m_auto)
			{
				Generate();
			}

			hull = b2ComputeHull(m_points, m_count);
			if (hull.count > 0)
			{
				valid = b2ValidateHull(hull);
				if (valid == false)
				{
					m_auto = false;
				}
			}
		}

		if (valid == false)
		{
			g_debugDraw.DrawString(5, m_textLine, "generation = %d, FAILED", m_generation);
			m_textLine += m_textIncrement;
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "generation = %d, count = %d", m_generation, hull.count);
			m_textLine += m_textIncrement;
		}

		if (milliseconds > 0.0f)
		{
			g_debugDraw.DrawString(5, m_textLine, "milliseconds = %g", milliseconds);
			m_textLine += m_textIncrement;
		}

		m_textLine += m_textIncrement;

		g_debugDraw.DrawPolygon(hull.points, hull.count, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_count; ++i)
		{
			g_debugDraw.DrawPoint(m_points[i], 5.0f, b2Color(0.3f, 0.3f, 0.9f));
			g_debugDraw.DrawString(m_points[i] + b2Vec2(0.1f, 0.1f), "%d", i);
		}

		for (int32 i = 0; i < hull.count; ++i)
		{
			g_debugDraw.DrawPoint(hull.points[i], 6.0f, b2Color(0.3f, 0.7f, 0.3f));
		}
	}

	static Test* Create()
	{
		return new ConvexHull;
	}

	b2Vec2 m_points[b2_maxPolygonVertices];
	int32 m_count;
	int32 m_generation;
	bool m_auto;
	bool m_bulk;
};

static int testIndex = RegisterTest("Geometry", "Convex Hull", ConvexHull::Create);
