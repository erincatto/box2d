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

#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_block_allocator.h"

#include <new>

struct b2Hull
{
	b2Vec2 points[b2_maxPolygonVertices];
	int32 count;
};

static b2Hull b2RecurseHull(b2Vec2 p1, b2Vec2 p2, b2Vec2* ps, int32 count)
{
	b2Hull hull;
	hull.count = 0;

	if (count == 0)
	{
		return hull;
	}


	return hull;
}

static b2Hull b2CombineHulls(const b2Vec2& hull1, const b2Hull& hull2)
{
	b2Hull hull;
	hull.count = 0;
	return hull;
}

static b2Hull b2ComputeHull(const b2Vec2* points, int32 count)
{
	b2Hull hull;
	hull.count = 0;

	if (count < 3 || count > b2_maxPolygonVertices)
	{
		return hull;
	}

	int32 n = b2Min(count, b2_maxPolygonVertices);

	b2AABB aabb = { {b2_maxFloat, b2_maxFloat}, {-b2_maxFloat, -b2_maxFloat} };

	// Perform aggressive welding. First vertex always remains as a candidate.
	b2Vec2 ps[b2_maxPolygonVertices];
	int32 m = 0;
	const float tolSqr = 16.0f * b2_linearSlop * b2_linearSlop;
	for (int32 i = 0; i < n; ++i)
	{
		aabb.lowerBound = b2Min(aabb.lowerBound, points[i]);
		aabb.upperBound = b2Max(aabb.upperBound, points[i]);

		b2Vec2 vi = points[i];

		bool unique = true;
		for (int32 j = 0; j < i; ++j)
		{
			b2Vec2 vj = points[j];

			float distSqr = b2DistanceSquared(vi, vj);
			if (distSqr < tolSqr)
			{
				unique = false;
				break;
			}
		}

		if (unique)
		{
			ps[i] = vi;
			++m;
		}
	}

	if (m < 3)
	{
		return hull;
	}

	// Find an extreme point as the first point on the hull
	b2Vec2 c = aabb.GetCenter();
	int32 i1 = 0;
	float d1 = b2DistanceSquared(c, ps[i1]);
	for (int32 i = 1; i < m; ++i)
	{
		float d = b2DistanceSquared(c, ps[i]);
		if (d > d1)
		{
			i1 = i;
			d1 = d;
		}
	}

	// remove p1 from working set
	b2Vec2 p1 = ps[i1];
	ps[i1] = ps[m - 1];
	m = m - 1;

	int32 i2 = 0;
	float d2 = b2DistanceSquared(p1, ps[i2]);
	for (int32 i = 1; i < m; ++i)
	{
		float d = b2DistanceSquared(p1, ps[i]);
		if (d > d2)
		{
			i2 = i;
			d2 = d;
		}
	}

	// remove p2 from working set
	b2Vec2 p2 = ps[i2];
	ps[i2] = ps[m - 1];
	m = m - 1;

	b2Vec2 above[b2_maxPolygonVertices - 2];
	int32 aboveCount = 0;

	b2Vec2 below[b2_maxPolygonVertices - 2];
	int32 belowCount = 0;

	b2Vec2 r = p2 - p1;
	for (int32 i = 0; i < m; ++i)
	{
		b2Vec2 e = ps[i] - p1;
		float f = b2Cross(e, r);
		if (f >= 0.0f)
		{
			above[aboveCount++] = ps[i];
		}
		else
		{
			below[belowCount++] = ps[i];
		}
	}

	b2Hull hullAbove = b2RecurseHull(p1, p2, above, aboveCount);
	b2Hull hullBelow = b2RecurseHull(p2, p1, below, belowCount);
	hull = b2CombineHulls(hullAbove, hullBelow);
	return hull;
}

b2PolygonShape::b2PolygonShape()
{
	m_type = e_polygon;
	m_radius = b2_polygonRadius;
	m_count = 0;
	m_centroid.SetZero();
}

b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2PolygonShape));
	b2PolygonShape* clone = new (mem) b2PolygonShape;
	*clone = *this;
	return clone;
}

void b2PolygonShape::SetAsBox(float hx, float hy)
{
	m_count = 4;
	m_vertices[0].Set(-hx, -hy);
	m_vertices[1].Set( hx, -hy);
	m_vertices[2].Set( hx,  hy);
	m_vertices[3].Set(-hx,  hy);
	m_normals[0].Set(0.0f, -1.0f);
	m_normals[1].Set(1.0f, 0.0f);
	m_normals[2].Set(0.0f, 1.0f);
	m_normals[3].Set(-1.0f, 0.0f);
	m_centroid.SetZero();
}

void b2PolygonShape::SetAsBox(float hx, float hy, const b2Vec2& center, float angle)
{
	m_count = 4;
	m_vertices[0].Set(-hx, -hy);
	m_vertices[1].Set( hx, -hy);
	m_vertices[2].Set( hx,  hy);
	m_vertices[3].Set(-hx,  hy);
	m_normals[0].Set(0.0f, -1.0f);
	m_normals[1].Set(1.0f, 0.0f);
	m_normals[2].Set(0.0f, 1.0f);
	m_normals[3].Set(-1.0f, 0.0f);
	m_centroid = center;

	b2Transform xf;
	xf.p = center;
	xf.q.Set(angle);

	// Transform vertices and normals.
	for (int32 i = 0; i < m_count; ++i)
	{
		m_vertices[i] = b2Mul(xf, m_vertices[i]);
		m_normals[i] = b2Mul(xf.q, m_normals[i]);
	}
}

int32 b2PolygonShape::GetChildCount() const
{
	return 1;
}

static b2Vec2 ComputeCentroid(const b2Vec2* vs, int32 count)
{
	b2Assert(count >= 3);

	b2Vec2 c(0.0f, 0.0f);
	float area = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 s = vs[0];

	const float inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; ++i)
	{
		// Triangle vertices.
		b2Vec2 p1 = vs[0] - s;
		b2Vec2 p2 = vs[i] - s;
		b2Vec2 p3 = i + 1 < count ? vs[i+1] - s : vs[0] - s;

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	b2Assert(area > b2_epsilon);
	c = (1.0f / area) * c + s;
	return c;
}

enum class b2HullState
{
	Welded,
	Candidate,
	Collinear,
	HullRoot,
	Hull
};

bool b2PolygonShape::Set(const b2Vec2* vertices, int32 count)
{
	if (count < 3 || count > b2_maxPolygonVertices)
	{
		return false;
	}
	
	int32 n = b2Min(count, b2_maxPolygonVertices);

	b2HullState states[b2_maxPolygonVertices] = {};
	b2AABB aabb = { {b2_maxFloat, b2_maxFloat}, {-b2_maxFloat, -b2_maxFloat} };

	for (int32 i = 0; i < n; ++i)
	{
		states[i] = b2HullState::Candidate;

		aabb.lowerBound = b2Min(aabb.lowerBound, vertices[i]);
		aabb.upperBound = b2Max(aabb.upperBound, vertices[i]);
	}

	// Perform aggressive welding. First vertex always remains as a candidate.
	int32 candidateCount = 0;
	const float tolSqr = 16.0f * b2_linearSlop * b2_linearSlop;
	for (int32 i = 0; i < n; ++i)
	{
		b2Vec2 vi = vertices[i];

		bool unique = true;
		for (int32 j = 0; j < i; ++j)
		{
			b2Vec2 vj = vertices[j];

			float distSqr = b2DistanceSquared(vi, vj);
			if (distSqr < tolSqr)
			{
				unique = false;
				states[i] = b2HullState::Welded;
				break;
			}
		}

		if (unique)
		{
			// Keep track of how many candidate points remain
			++candidateCount;
		}
	}

	if (candidateCount < 3)
	{
		// Polygon is degenerate
		return false;
	}

	// Create the convex hull using the Gift wrapping algorithm
	// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

	// Find an extreme point as the first point on the hull
	b2Vec2 c = aabb.GetCenter();
	b2Vec2 p0 = vertices[0];
	float d0 = b2DistanceSquared(c, p0);
	int32 i0 = 0;
	for (int32 i = 1; i < n; ++i)
	{
		if (states[i] == b2HullState::Welded)
		{
			continue;
		}

		float d = b2DistanceSquared(c, vertices[i]);
		if (d > d0)
		{
			p0 = vertices[i];
			i0 = i;
			d0 = d;
		}
	}

	states[i0] = b2HullState::HullRoot;

	b2Vec2 hull[b2_maxPolygonVertices];

	// m is the current size of the hull
	int32 m = 0;
	b2Vec2 pivot = p0;
	int32 pivotIndex = i0;

	// build hull in CCW order
	for (;;)
	{
		b2Assert(m < b2_maxPolygonVertices);

		hull[m] = pivot;
		++m;

		int32 ie = 0;

		for (int32 j = 1; j < n; ++j)
		{
			if (states[j] == b2HullState::Welded ||
				states[j] == b2HullState::Hull ||
				states[j] == b2HullState::Collinear ||
				j == pivotIndex)
			{
				// don't consider welded or hull points (allow root)
				continue;
			}

			if (states[ie] == b2HullState::Welded ||
				states[ie] == b2HullState::Hull ||
				states[j] == b2HullState::Collinear ||
				ie == pivotIndex)
			{
				// endpoint must be a candidate, try next
				ie = j;
				continue;
			}

			b2Vec2 re = vertices[ie] - pivot;
			b2Vec2 rj = vertices[j] - pivot;

			// If c is positive then rj is to the right of re
			float c = b2Cross(rj, re);

			// Collinear check
			float reLen = re.Length();
			float rjLen = rj.Length();
			float tol = 2.0f * b2_linearSlop * reLen;

			if (c > tol)
			{
				// rj is clearly to the right
				ie = j;
				continue;
			}

			if (c > -tol && rjLen > reLen)
			{
				// the angle is small and rj is longer than re
				// consider vertex ie to be collinear

				if (ie != i0)
				{
					states[ie] = b2HullState::Collinear;
				}

				ie = j;
			}
		}

		pivot = vertices[ie];
		pivotIndex = ie;
		states[ie] = b2HullState::Hull;

		if (ie == i0)
		{
			break;
		}
	}
	
	if (m < 3)
	{
		// Polygon is degenerate.
		return false;
	}

	m_count = m;

	// Copy vertices.
	for (int32 i = 0; i < m; ++i)
	{
		m_vertices[i] = hull[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (int32 i = 0; i < m; ++i)
	{
		int32 i1 = i;
		int32 i2 = i + 1 < m ? i + 1 : 0;
		b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
		b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
		m_normals[i] = b2Cross(edge, 1.0f);
		m_normals[i].Normalize();
	}

	// Compute the polygon centroid.
	m_centroid = ComputeCentroid(m_vertices, m);

	return true;
}

bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

	for (int32 i = 0; i < m_count; ++i)
	{
		float dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
								const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	// Put the ray into the polygon's frame of reference.
	b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
	b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
	b2Vec2 d = p2 - p1;

	float lower = 0.0f, upper = input.maxFraction;

	int32 index = -1;

	for (int32 i = 0; i < m_count; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
		float denominator = b2Dot(m_normals[i], d);

		if (denominator == 0.0f)
		{	
			if (numerator < 0.0f)
			{
				return false;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			return false;
		}
	}

	b2Assert(0.0f <= lower && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		output->normal = b2Mul(xf.q, m_normals[index]);
		return true;
	}

	return false;
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 lower = b2Mul(xf, m_vertices[0]);
	b2Vec2 upper = lower;

	for (int32 i = 1; i < m_count; ++i)
	{
		b2Vec2 v = b2Mul(xf, m_vertices[i]);
		lower = b2Min(lower, v);
		upper = b2Max(upper, v);
	}

	b2Vec2 r(m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

void b2PolygonShape::ComputeMass(b2MassData* massData, float density) const
{
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	b2Assert(m_count >= 3);

	b2Vec2 center(0.0f, 0.0f);
	float area = 0.0f;
	float I = 0.0f;

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	b2Vec2 s = m_vertices[0];

	const float k_inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < m_count; ++i)
	{
		// Triangle vertices.
		b2Vec2 e1 = m_vertices[i] - s;
		b2Vec2 e2 = i + 1 < m_count ? m_vertices[i+1] - s : m_vertices[0] - s;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);

		float ex1 = e1.x, ey1 = e1.y;
		float ex2 = e2.x, ey2 = e2.y;

		float intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
		float inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

		I += (0.25f * k_inv3 * D) * (intx2 + inty2);
	}

	// Total mass
	massData->mass = density * area;

	// Center of mass
	b2Assert(area > b2_epsilon);
	center *= 1.0f / area;
	massData->center = center + s;

	// Inertia tensor relative to the local origin (point s).
	massData->I = density * I;
	
	// Shift to center of mass then to original body origin.
	massData->I += massData->mass * (b2Dot(massData->center, massData->center) - b2Dot(center, center));
}

bool b2PolygonShape::Validate() const
{
	// Test that every point is behind every edge
	for (int32 i = 0; i < m_count; ++i)
	{
		int32 i1 = i;
		int32 i2 = i < m_count - 1 ? i1 + 1 : 0;
		b2Vec2 p = m_vertices[i1];
		b2Vec2 e = m_vertices[i2] - p;

		for (int32 j = 0; j < m_count; ++j)
		{
			if (j == i1 || j == i2)
			{
				continue;
			}

			b2Vec2 v = m_vertices[j] - p;
			float c = b2Cross(e, v);
			if (c < 0.0f)
			{
				return false;
			}
		}
	}

	return true;
}
