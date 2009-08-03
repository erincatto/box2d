/*
* Copyright (c) 2007-2009 Erin Catto http://www.gphysics.com
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

#include <Box2D/Collision/b2Collision.h>

void b2WorldManifold::Initialize(const b2Manifold* manifold,
						  const b2Transform& xfA, float32 radiusA,
						  const b2Transform& xfB, float32 radiusB)
{
	if (manifold->m_pointCount == 0)
	{
		return;
	}

	switch (manifold->m_type)
	{
	case b2Manifold::e_circles:
		{
			b2Vec2 pointA = b2Mul(xfA, manifold->m_localPoint);
			b2Vec2 pointB = b2Mul(xfB, manifold->m_points[0].m_localPoint);
			b2Vec2 normal(1.0f, 0.0f);
			if (b2DistanceSquared(pointA, pointB) > B2_FLT_EPSILON * B2_FLT_EPSILON)
			{
				normal = pointB - pointA;
				normal.Normalize();
			}

			m_normal = normal;

			b2Vec2 cA = pointA + radiusA * normal;
			b2Vec2 cB = pointB - radiusB * normal;
			m_points[0] = 0.5f * (cA + cB);
		}
		break;

	case b2Manifold::e_faceA:
		{
			b2Vec2 normal = b2Mul(xfA.R, manifold->m_localPlaneNormal);
			b2Vec2 planePoint = b2Mul(xfA, manifold->m_localPoint);

			// Ensure normal points from A to B.
			m_normal = normal;
			
			for (int32 i = 0; i < manifold->m_pointCount; ++i)
			{
				b2Vec2 clipPoint = b2Mul(xfB, manifold->m_points[i].m_localPoint);
				b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
				b2Vec2 cB = clipPoint - radiusB * normal;
				m_points[i] = 0.5f * (cA + cB);
			}
		}
		break;

	case b2Manifold::e_faceB:
		{
			b2Vec2 normal = b2Mul(xfB.R, manifold->m_localPlaneNormal);
			b2Vec2 planePoint = b2Mul(xfB, manifold->m_localPoint);

			// Ensure normal points from A to B.
			m_normal = -normal;

			for (int32 i = 0; i < manifold->m_pointCount; ++i)
			{
				b2Vec2 clipPoint = b2Mul(xfA, manifold->m_points[i].m_localPoint);
				b2Vec2 cA = clipPoint - radiusA * normal;
				b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
				m_points[i] = 0.5f * (cA + cB);
			}
		}
		break;
	}
}

void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2)
{
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
	{
		state1[i] = b2_nullState;
		state2[i] = b2_nullState;
	}

	// Detect persists and removes.
	for (int32 i = 0; i < manifold1->m_pointCount; ++i)
	{
		b2ContactID id = manifold1->m_points[i].m_id;

		state1[i] = b2_removeState;

		for (int32 j = 0; j < manifold2->m_pointCount; ++j)
		{
			if (manifold2->m_points[j].m_id.key == id.key)
			{
				state1[i] = b2_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (int32 i = 0; i < manifold2->m_pointCount; ++i)
	{
		b2ContactID id = manifold2->m_points[i].m_id;

		state2[i] = b2_addState;

		for (int32 j = 0; j < manifold1->m_pointCount; ++j)
		{
			if (manifold1->m_points[j].m_id.key == id.key)
			{
				state2[i] = b2_persistState;
				break;
			}
		}
	}
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.4.1
// x = mu1 * p1 + mu2 * p2
// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
// mu1 = 1 - mu2;
// x = (1 - mu2) * p1 + mu2 * p2
//   = p1 + mu2 * (p2 - p1)
// x = s + a * r (s := start, r := end - start)
// s + a * r = p1 + mu2 * d (d := p2 - p1)
// -a * r + mu2 * d = b (b := s - p1)
// [-r d] * [a; mu2] = b
// Cramer's rule:
// denom = det[-r d]
// a = det[b d] / denom
// mu2 = det[-r b] / denom
bool b2Segment::TestSegment(float32* lambda, b2Vec2* normal, const b2Segment& segment, float32 maxLambda) const
{
	b2Vec2 s = segment.p1;
	b2Vec2 r = segment.p2 - s;
	b2Vec2 d = p2 - p1;
	b2Vec2 n = b2Cross(d, 1.0f);

	const float32 k_slop = 100.0f * B2_FLT_EPSILON;
	float32 denom = -b2Dot(r, n);

	// Cull back facing collision and ignore parallel segments.
	if (denom > k_slop)
	{
		// Does the segment intersect the infinite line associated with this segment?
		b2Vec2 b = s - p1;
		float32 a = b2Dot(b, n);

		if (0.0f <= a && a <= maxLambda * denom)
		{
			float32 mu2 = -r.x * b.y + r.y * b.x;

			// Does the segment intersect this segment?
			if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
			{
				a /= denom;
				n.Normalize();
				*lambda = a;
				*normal = n;
				return true;
			}
		}
	}

	return false;
}

// From Real-time Collision Detection, p179.
void b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input)
{
	float32 tmin = -B2_FLT_MAX;
	float32 tmax = B2_FLT_MAX;

	output->hit = false;

	b2Vec2 p = input.p1;
	b2Vec2 d = input.p2 - input.p1;
	b2Vec2 absD = b2Abs(d);

	b2Vec2 normal;

	for (int32 i = 0; i < 2; ++i)
	{
		if (absD(i) < B2_FLT_EPSILON)
		{
			// Parallel.
			if (p(i) < lowerBound(i) || upperBound(i) < p(i))
			{
				return;
			}
		}
		else
		{
			float32 inv_d = 1.0f / d(i);
			float32 t1 = (lowerBound(i) - p(i)) * inv_d;
			float32 t2 = (upperBound(i) - p(i)) * inv_d;

			// Sign of the normal vector.
			float32 s = -1.0f;

			if (t1 > t2)
			{
				b2Swap(t1, t2);
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin)
			{
				normal.SetZero();
				normal(i) = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = b2Min(tmax, t2);

			if (tmin > tmax)
			{
				return;
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if (tmin < 0.0f || input.maxFraction < tmin)
	{
		return;
	}

	// Intersection.
	output->fraction = tmin;
	output->normal = normal;
	output->hit = true;
}

// Sutherland-Hodgman clipping.
int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
						const b2Vec2& normal, float32 offset)
{
	// Start with no output points
	int32 numOut = 0;

	// Calculate the distance of end points to the line
	float32 distance0 = b2Dot(normal, vIn[0].v) - offset;
	float32 distance1 = b2Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float32 interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > 0.0f)
		{
			vOut[numOut].id = vIn[0].id;
		}
		else
		{
			vOut[numOut].id = vIn[1].id;
		}
		++numOut;
	}

	return numOut;
}
