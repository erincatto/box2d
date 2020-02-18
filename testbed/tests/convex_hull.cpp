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
		Generate();
		m_auto = false;
	}

	void Generate()
	{
		b2Vec2 lowerBound(-8.0f, -8.0f);
		b2Vec2 upperBound(8.0f, 8.0f);

		for (int32 i = 0; i < e_count; ++i)
		{
			float x = 10.0f * RandomFloat();
			float y = 10.0f * RandomFloat();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			b2Vec2 v(x, y);
			v = b2Clamp(v, lowerBound, upperBound);
			m_points[i] = v;
		}

		m_count = e_count;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_auto = !m_auto;
			break;

		case GLFW_KEY_G:
			Generate();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2PolygonShape shape;
		shape.Set(m_points, m_count);

		g_debugDraw.DrawString(5, m_textLine, "Press g to generate a new random convex hull");
		m_textLine += m_textIncrement;

		g_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_count; ++i)
		{
			g_debugDraw.DrawPoint(m_points[i], 3.0f, b2Color(0.3f, 0.9f, 0.3f));
			g_debugDraw.DrawString(m_points[i] + b2Vec2(0.05f, 0.05f), "%d", i);
		}

		if (shape.Validate() == false)
		{
			m_textLine += 0;
		}

		if (m_auto)
		{
			Generate();
		}
	}

	static Test* Create()
	{
		return new ConvexHull;
	}

	b2Vec2 m_points[b2_maxPolygonVertices];
	int32 m_count;
	bool m_auto;
};

static int testIndex = RegisterTest("Geometry", "Convex Hull", ConvexHull::Create);
