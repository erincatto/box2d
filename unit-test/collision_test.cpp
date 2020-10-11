// MIT License

// Copyright (c) 2020 Erin Catto

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

#include "box2d/box2d.h"
#include "doctest.h"
#include <stdio.h>

// Unit tests for collision algorithms
DOCTEST_TEST_CASE("collision test")
{
	SUBCASE("polygon mass data")
	{
		const b2Vec2 center(100.0f, -50.0f);
		const float hx = 0.5f, hy = 1.5f;
		const float angle1 = 0.25f;

		// Data from issue #422. Not used because the data exceeds accuracy limits.
		//const b2Vec2 center(-15000.0f, -15000.0f);
		//const float hx = 0.72f, hy = 0.72f;
		//const float angle1 = 0.0f;

		b2PolygonShape polygon1;
		polygon1.SetAsBox(hx, hy, center, angle1);

		const float absTol = 2.0f * b2_epsilon;
		const float relTol = 2.0f * b2_epsilon;

		CHECK(b2Abs(polygon1.m_centroid.x - center.x) < absTol + relTol * b2Abs(center.x));
		CHECK(b2Abs(polygon1.m_centroid.y - center.y) < absTol + relTol * b2Abs(center.y));

		b2Vec2 vertices[4];
		vertices[0].Set(center.x - hx, center.y - hy);
		vertices[1].Set(center.x + hx, center.y - hy);
		vertices[2].Set(center.x - hx, center.y + hy);
		vertices[3].Set(center.x + hx, center.y + hy);

		b2PolygonShape polygon2;
		polygon2.Set(vertices, 4);

		CHECK(b2Abs(polygon2.m_centroid.x - center.x) < absTol + relTol * b2Abs(center.x));
		CHECK(b2Abs(polygon2.m_centroid.y - center.y) < absTol + relTol * b2Abs(center.y));

		const float mass = 4.0f * hx * hy;
		const float inertia = (mass / 3.0f) * (hx * hx + hy * hy) + mass * b2Dot(center, center);

		b2MassData massData1;
		polygon1.ComputeMass(&massData1, 1.0f);

		CHECK(b2Abs(massData1.center.x - center.x) < absTol + relTol * b2Abs(center.x));
		CHECK(b2Abs(massData1.center.y - center.y) < absTol + relTol * b2Abs(center.y));
		CHECK(b2Abs(massData1.mass - mass) < 20.0f * (absTol + relTol * mass));
		CHECK(b2Abs(massData1.I - inertia) < 40.0f * (absTol + relTol * inertia));

		b2MassData massData2;
		polygon2.ComputeMass(&massData2, 1.0f);

		CHECK(b2Abs(massData2.center.x - center.x) < absTol + relTol * b2Abs(center.x));
		CHECK(b2Abs(massData2.center.y - center.y) < absTol + relTol * b2Abs(center.y));
		CHECK(b2Abs(massData2.mass - mass) < 20.0f * (absTol + relTol * mass));
		CHECK(b2Abs(massData2.I - inertia) < 40.0f * (absTol + relTol * inertia));
	}
}
