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
		unsigned int vals[6][2] = {
			{0xC1078533,0x3F0BAB5E},
			{0xC0F5B2E8,0x3F28AF8C},
			{0xC0E2B14A,0x3F3E72AE},
			{0xC0EBC298, 0x402E521E},
			{0xC1158EA9, 0x401C2F82},
			{0xC1110602, 0x3EEBD078}};

		m_count = 6;
		for (int i = 0; i < m_count; ++i)
		{
			m_points[i].x = *(float*)(&(vals[i][0]));
			m_points[i].y = *(float*)(&(vals[i][1]));
		}

#elif 1

		m_points[0] = { -2.99307323f, -2.84361196f };
		m_points[1] = { -5.41493702f,  1.55601883f };
		m_points[2] = { 1.54026687f,  5.44312191f };
		m_points[3] = { -2.17538762f, -4.30669880f };
		m_points[4] = { -1.54026687f, -5.44312191f };
		m_points[5] = { -2.31474543f, -4.05734587f };
		m_points[6] = { 1.54026687f,  5.44312191f };
		m_points[7] = { -3.09342265f, -2.66405630f };

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

		g_debugDraw.DrawString(5, m_textLine, "Press g to generate a new random convex hull");
		m_textLine += m_textIncrement;
		
		bool success;
		b2PolygonShape shape;

		if (m_bulk)
		{
			for (int32 i = 0; i < 1000; ++i)
			{
				Generate();
				success = shape.Set(m_points, m_count);

				if (success && shape.Validate() == false)
				{
					m_bulk = false;
					break;
				}
			}
		}
		else
		{
			if (m_auto)
			{
				Generate();
			}

			success = shape.Set(m_points, m_count);
			if (success && shape.Validate() == false)
			{
				m_auto = false;
			}
		}

		if (success == false)
		{
			g_debugDraw.DrawString(5, m_textLine, "generation = %d, FAILED", m_generation);

			if (m_generation != 1570)
			{
				m_auto = false;
			}
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "generation = %d, count = %d", m_generation, shape.m_count);
		}

		m_textLine += m_textIncrement;

		g_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_count; ++i)
		{
			g_debugDraw.DrawPoint(m_points[i], 3.0f, b2Color(0.3f, 0.9f, 0.3f));
			g_debugDraw.DrawString(m_points[i] + b2Vec2(0.05f, 0.05f), "%d", i);
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
