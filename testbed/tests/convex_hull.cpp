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

		m_points[0] = { -4.03110123f, 1.21307147f };
		m_points[1] = { 0.991306305f, -5.56931877f };
		m_points[2] = { -2.30727673f, -3.26756334f };
		m_points[3] = { 0.991306305f, -5.56931877f };
		m_points[4] = { 5.20646906f, 1.24450326f };
		m_points[5] = { -0.459646702f, 3.91972303f };
		m_points[6] = { 5.20346594f, 1.24659896f };
		m_points[7] = { -3.36815786f, 2.16311741f };

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

		g_debugDraw.DrawString(5, m_textLine, "Press g to generate a new random convex hull");
		m_textLine += m_textIncrement;

		b2PolygonShape shape;
		bool success = shape.Set(m_points, m_count);
		if (success == false)
		{
			g_debugDraw.DrawString(5, m_textLine, "FAILED");
			shape.Set(m_points, m_count);
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "count = %d", shape.m_count);
		}

		m_textLine += m_textIncrement;

		g_debugDraw.DrawPolygon(shape.m_vertices, shape.m_count, b2Color(0.9f, 0.9f, 0.9f));

		for (int32 i = 0; i < m_count; ++i)
		{
			g_debugDraw.DrawPoint(m_points[i], 3.0f, b2Color(0.3f, 0.9f, 0.3f));
			g_debugDraw.DrawString(m_points[i] + b2Vec2(0.05f, 0.05f), "%d", i);
		}

		//if (success && shape.Validate() == false)
		//{
		//	shape.Set(m_points, m_count);
		//}

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
