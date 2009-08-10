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
#include <Box2D/Collision/b2Distance.h>
#include <Box2D/Collision/b2TimeOfImpact.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

#include <cstdio>

int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
int32 b2_toiRootIters, b2_toiMaxRootIters;

struct b2SeparationFunction
{
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	void Initialize(const b2SimplexCache* cache,
		const b2DistanceProxy* proxyA, const b2Transform& transformA,
		const b2DistanceProxy* proxyB, const b2Transform& transformB)
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int32 count = cache->count;
		b2Assert(0 < count && count < 3);

		if (count == 1)
		{
			m_type = e_points;
			b2Vec2 localPointA = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 pointA = b2Mul(transformA, localPointA);
			b2Vec2 pointB = b2Mul(transformB, localPointB);
			m_axis = pointB - pointA;
			m_axis.Normalize();
		}
		else if (cache->indexB[0] == cache->indexB[1])
		{
			// Two points on A and one on B
			m_type = e_faceA;
			b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
			b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			m_axis = b2Cross(localPointA2 - localPointA1, 1.0f);
			m_axis.Normalize();

			b2Vec2 normal = b2Mul(transformA.R, m_axis);
			b2Vec2 pointA = b2Mul(transformA, m_localPoint);
			b2Vec2 pointB = b2Mul(transformB, localPointB);

			float32 s = b2Dot(pointB - pointA, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
			}
		}
		else if (cache->indexA[0] == cache->indexA[1])
		{
			// Two points on B and one on A.
			m_type = e_faceB;
			b2Vec2 localPointA = proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointB1 = proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 localPointB2 = proxyB->GetVertex(cache->indexB[1]);
			m_localPoint = 0.5f * (localPointB1 + localPointB2);
			m_axis = b2Cross(localPointB2 - localPointB1, 1.0f);
			m_axis.Normalize();

			b2Vec2 normal = b2Mul(transformB.R, m_axis);
			b2Vec2 pointB = b2Mul(transformB, m_localPoint);
			b2Vec2 pointA = b2Mul(transformA, localPointA);

			float32 s = b2Dot(pointA - pointB, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
			}
		}
		else
		{
			// Two points on B and two points on A.
			// The faces are parallel.
			b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
			b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
			b2Vec2 localPointB1 = m_proxyB->GetVertex(cache->indexB[0]);
			b2Vec2 localPointB2 = m_proxyB->GetVertex(cache->indexB[1]);

			b2Vec2 pA = b2Mul(transformA, localPointA1);
			b2Vec2 dA = b2Mul(transformA.R, localPointA2 - localPointA1);
			b2Vec2 pB = b2Mul(transformB, localPointB1);
			b2Vec2 dB = b2Mul(transformB.R, localPointB2 - localPointB1);

			float32 a = b2Dot(dA, dA);
			float32 e = b2Dot(dB, dB);
			b2Vec2 r = pA - pB;
			float32 c = b2Dot(dA, r);
			float32 f = b2Dot(dB, r);

			float32 b = b2Dot(dA, dB);
			float32 denom = a * e - b * b;

			float32 s = 0.0f;
			if (denom != 0.0f)
			{
				s = b2Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}

			float32 t = (b * s + f) / e;

			if (t < 0.0f)
			{
				t = 0.0f;
				s = b2Clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = b2Clamp((b - c) / a, 0.0f, 1.0f);
			}

			b2Vec2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			b2Vec2 localPointB = localPointB1 + t * (localPointB2 - localPointB1);

			if (s == 0.0f || s == 1.0f)
			{
				m_type = e_faceB;
				m_axis = b2Cross(localPointB2 - localPointB1, 1.0f);
				m_axis.Normalize();

				m_localPoint = localPointB;

				b2Vec2 normal = b2Mul(transformB.R, m_axis);
				b2Vec2 pointA = b2Mul(transformA, localPointA);
				b2Vec2 pointB = b2Mul(transformB, localPointB);

				float32 sgn = b2Dot(pointA - pointB, normal);
				if (sgn < 0.0f)
				{
					m_axis = -m_axis;
				}
			}
			else
			{
				m_type = e_faceA;
				m_axis = b2Cross(localPointA2 - localPointA1, 1.0f);
				m_axis.Normalize();

				m_localPoint = localPointA;

				b2Vec2 normal = b2Mul(transformA.R, m_axis);
				b2Vec2 pointA = b2Mul(transformA, localPointA);
				b2Vec2 pointB = b2Mul(transformB, localPointB);

				float32 sgn = b2Dot(pointB - pointA, normal);
				if (sgn < 0.0f)
				{
					m_axis = -m_axis;
				}
			}
		}
	}

	float32 Evaluate(const b2Transform& transformA, const b2Transform& transformB)
	{
		switch (m_type)
		{
		case e_points:
			{
				b2Vec2 axisA = b2MulT(transformA.R,  m_axis);
				b2Vec2 axisB = b2MulT(transformB.R, -m_axis);
				b2Vec2 localPointA = m_proxyA->GetSupportVertex(axisA);
				b2Vec2 localPointB = m_proxyB->GetSupportVertex(axisB);
				b2Vec2 pointA = b2Mul(transformA, localPointA);
				b2Vec2 pointB = b2Mul(transformB, localPointB);
				float32 separation = b2Dot(pointB - pointA, m_axis);
				return separation;
			}

		case e_faceA:
			{
				b2Vec2 normal = b2Mul(transformA.R, m_axis);
				b2Vec2 pointA = b2Mul(transformA, m_localPoint);

				b2Vec2 axisB = b2MulT(transformB.R, -normal);

				b2Vec2 localPointB = m_proxyB->GetSupportVertex(axisB);
				b2Vec2 pointB = b2Mul(transformB, localPointB);

				float32 separation = b2Dot(pointB - pointA, normal);
				return separation;
			}

		case e_faceB:
			{
				b2Vec2 normal = b2Mul(transformB.R, m_axis);
				b2Vec2 pointB = b2Mul(transformB, m_localPoint);

				b2Vec2 axisA = b2MulT(transformA.R, -normal);

				b2Vec2 localPointA = m_proxyA->GetSupportVertex(axisA);
				b2Vec2 pointA = b2Mul(transformA, localPointA);

				float32 separation = b2Dot(pointA - pointB, normal);
				return separation;
			}

		default:
			b2Assert(false);
			return 0.0f;
		}
	}

	const b2DistanceProxy* m_proxyA;
	const b2DistanceProxy* m_proxyB;
	Type m_type;
	b2Vec2 m_localPoint;
	b2Vec2 m_axis;
};

// CCD via the secant method.
float32 b2TimeOfImpact(const b2TOIInput* input)
{
	++b2_toiCalls;

	const b2DistanceProxy* proxyA = &input->proxyA;
	const b2DistanceProxy* proxyB = &input->proxyB;

	b2Sweep sweepA = input->sweepA;
	b2Sweep sweepB = input->sweepB;

	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	float32 radius = proxyA->m_radius + proxyB->m_radius;
	float32 tolerance = input->tolerance;

	float32 alpha = 0.0f;

	const int32 k_maxIterations = 1000;	// TODO_ERIN b2Settings
	int32 iter = 0;
	float32 target = 0.0f;

	// Prepare input for distance query.
	b2SimplexCache cache;
	cache.count = 0;
	b2DistanceInput distanceInput;
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	for(;;)
	{
		b2Transform xfA, xfB;
		sweepA.GetTransform(&xfA, alpha);
		sweepB.GetTransform(&xfB, alpha);

		// Get the distance between shapes.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2DistanceOutput distanceOutput;
		b2Distance(&distanceOutput, &cache, &distanceInput);

		if (distanceOutput.distance <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		b2SeparationFunction fcn;
		fcn.Initialize(&cache, proxyA, xfA, proxyB, xfB);

		float32 separation = fcn.Evaluate(xfA, xfB);
		if (separation <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement. We take advantage of the shape radii
			// to create additional clearance.
			if (separation > radius)
			{
				target = b2Max(radius - tolerance, 0.75f * radius);
			}
			else
			{
				target = b2Max(separation - tolerance, 0.02f * radius);
			}
		}

		if (separation - target < 0.5f * tolerance)
		{
			if (iter == 0)
			{
				alpha = 1.0f;
				break;
			}

			break;
		}

#if 0
		// Dump the curve seen by the root finder
		{
			const int32 N = 100;
			float32 dx = 1.0f / N;
			float32 xs[N+1];
			float32 fs[N+1];

			float32 x = 0.0f;

			for (int32 i = 0; i <= N; ++i)
			{
				sweepA.GetTransform(&xfA, x);
				sweepB.GetTransform(&xfB, x);
				float32 f = fcn.Evaluate(xfA, xfB) - target;

				printf("%g %g\n", x, f);

				xs[i] = x;
				fs[i] = f;

				x += dx;
			}
		}
#endif

		// Compute 1D root of: f(x) - target = 0
		float32 newAlpha = alpha;
		{
			float32 x1 = alpha, x2 = 1.0f;

			float32 f1 = separation;

			sweepA.GetTransform(&xfA, x2);
			sweepB.GetTransform(&xfB, x2);
			float32 f2 = fcn.Evaluate(xfA, xfB);

			// If intervals don't overlap at t2, then we are done.
			if (f2 >= target)
			{
				alpha = 1.0f;
				break;
			}

			// Determine when intervals intersect.
			int32 rootIterCount = 0;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float32 x;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
				}
				else
				{
					// Bisection to guarantee progress.
					x = 0.5f * (x1 + x2);
				}

				sweepA.GetTransform(&xfA, x);
				sweepB.GetTransform(&xfB, x);

				float32 f = fcn.Evaluate(xfA, xfB);

				if (b2Abs(f - target) < 0.025f * tolerance)
				{
					newAlpha = x;
					break;
				}

				// Ensure we continue to bracket the root.
				if (f > target)
				{
					x1 = x;
					f1 = f;
				}
				else
				{
					x2 = x;
					f2 = f;
				}

				++rootIterCount;
				++b2_toiRootIters;

				if (rootIterCount == 50)
				{
					break;
				}
			}

			b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);
		}

		// Ensure significant advancement.
		if (newAlpha < (1.0f + 100.0f * B2_FLT_EPSILON) * alpha)
		{
			break;
		}

		alpha = newAlpha;

		++iter;
		++b2_toiIters;

		if (iter == k_maxIterations)
		{
			break;
		}
	}

	b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);

	return alpha;
}
