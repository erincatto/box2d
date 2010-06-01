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
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

enum b2EdgeType
{
	e_isolated,
	e_concave,
	e_flat,
	e_convex
};

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
							const b2EdgeShape* edgeA, const b2Transform& xfA,
							const b2CircleShape* circleB, const b2Transform& xfB)
{
	manifold->pointCount = 0;

	// Compute circle in frame of edge
	b2Vec2 Q = b2MulT(xfA, b2Mul(xfB, circleB->m_p));

	b2Vec2 A = edgeA->m_vertex1, B = edgeA->m_vertex2;
	b2Vec2 e = B - A;

	// Barycentric coordinates
	float32 u = b2Dot(e, B - Q);
	float32 v = b2Dot(e, Q - A);

	float32 radius = edgeA->m_radius + circleB->m_radius;

	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2ContactFeature::e_vertex;

	// Region A
	if (v <= 0.0f)
	{
		b2Vec2 P = A;
		b2Vec2 d = Q - P;
		float32 dd = b2Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to A?
		if (edgeA->m_hasVertex0)
		{
			b2Vec2 A1 = edgeA->m_vertex0;
			b2Vec2 B1 = A;
			b2Vec2 e1 = B1 - A1;
			float32 u1 = b2Dot(e1, B1 - Q);

			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
			{
				return;
			}
		}

		cf.indexA = edgeA->m_index1;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = b2Manifold::e_circles;
		manifold->localNormal.SetZero();
		manifold->localPoint = P;
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint = circleB->m_p;
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		b2Vec2 P = B;
		b2Vec2 d = Q - P;
		float32 dd = b2Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to B?
		if (edgeA->m_hasVertex3)
		{
			b2Vec2 B2 = edgeA->m_vertex3;
			b2Vec2 A2 = B;
			b2Vec2 e2 = B2 - A2;
			float32 v2 = b2Dot(e2, Q - A2);

			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
			{
				return;
			}
		}

		cf.indexA = edgeA->m_index2;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = b2Manifold::e_circles;
		manifold->localNormal.SetZero();
		manifold->localPoint = P;
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint = circleB->m_p;
		return;
	}

	// Region AB
	float32 den = b2Dot(e, e);
	b2Assert(den > 0.0f);
	b2Vec2 P = (1.0f / den) * (u * A + v * B);
	b2Vec2 d = Q - P;
	float32 dd = b2Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}

	b2Vec2 n(-e.y, e.x);
	if (b2Dot(n, Q - A) < 0.0f)
	{
		n.Set(-n.x, -n.y);
	}
	n.Normalize();

	cf.indexA = 0;
	cf.typeA = b2ContactFeature::e_edge;
	manifold->pointCount = 1;
	manifold->type = b2Manifold::e_faceA;
	manifold->localNormal = n;
	manifold->localPoint = A;
	manifold->points[0].id.key = 0;
	manifold->points[0].id.cf = cf;
	manifold->points[0].localPoint = circleB->m_p;
}

struct b2EPAxis
{
	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB,
	};

	Type type;
	int32 index;
	float32 separation;
};

static b2EPAxis b2EPSeparation(const b2Vec2& v1, const b2Vec2& v2, const b2Vec2& n, const b2PolygonShape* polygonB)
{
	// EdgeA separation
	b2EPAxis axisA;
	axisA.type = b2EPAxis::e_edgeA;
	axisA.index = 0;
	axisA.separation = b2Dot(n, polygonB->m_vertices[0] - v1);
	for (int32 i = 1; i < polygonB->m_vertexCount; ++i)
	{
		float32 s = b2Dot(n, polygonB->m_vertices[i] - v1);
		if (s > axisA.separation)
		{
			axisA.separation = s;
			if (s > 0.0f)
			{
				return axisA;
			}
		}
	}

	// PolygonB separation
	b2EPAxis axisB;
	axisB.type = b2EPAxis::e_edgeB;
	axisB.index = 0;
	axisB.separation = -FLT_MAX;
	for (int32 i = 0; i < polygonB->m_vertexCount; ++i)
	{
		float32 s1 = b2Dot(polygonB->m_normals[i], v1 - polygonB->m_vertices[i]);	
		float32 s2 = b2Dot(polygonB->m_normals[i], v2 - polygonB->m_vertices[i]);
		float32 s = b2Min(s1, s2);
		if (s > axisB.separation)
		{
			axisB.index = i;
			axisB.separation = s;

			if (s > 0.0f)
			{
				return axisB;
			}
		}
	}

	// Return the best axis, using hysteresis for jitter reduction.
	const float32 k_relativeTol = 0.98f;
	const float32 k_absoluteTol = 0.001f;

	if (axisB.separation > k_relativeTol * axisA.separation + k_absoluteTol)
	{
		return axisB;
	}
	else
	{
		return axisA;
	}

}

// Collide and edge and polygon. This uses the SAT and clipping to produce up to 2 contact points.
// Edge adjacency is handle to produce locally valid contact points and normals. This is intended
// to allow the polygon to slide smoothly over an edge chain.
//
// Algorithm
// 1. Classify front-side or back-side collision with edge.
// 2. Compute separation
// 3. Process adjacent edges
// 4. Classify adjacent edge as convex, flat, null, or concave
// 5. Skip null or concave edges. Concave edges get a separate manifold.
// 6. If the edge is flat, compute contact points as normal. Discard boundary points.
// 7. If the edge is convex, compute it's separation.
// 8. Use the minimum separation of up to three edges. If the minimum separation
//    is not the primary edge, return.
// 9. If the minimum separation is the primary edge, compute the contact points and return.
void b2CollideEdgeAndPolygon(	b2Manifold* manifold,
								const b2EdgeShape* edgeA, const b2Transform& xfA,
								const b2PolygonShape* polygonB_in, const b2Transform& xfB)
{
	manifold->pointCount = 0;

	b2Transform xf = b2MulT(xfA, xfB);

	// Compute polygon in frame A
	b2PolygonShape polygonB;
	polygonB.m_vertexCount = polygonB_in->m_vertexCount;
	polygonB.m_centroid = b2Mul(xf, polygonB_in->m_centroid);
	for (int32 i = 0; i < polygonB.m_vertexCount; ++i)
	{
		polygonB.m_vertices[i] = b2Mul(xf, polygonB_in->m_vertices[i]);
		polygonB.m_normals[i] = b2Mul(xf.R, polygonB_in->m_normals[i]);
	}

	// Edge geometry
	b2Vec2 v1 = edgeA->m_vertex1;
	b2Vec2 v2 = edgeA->m_vertex2;
	b2Vec2 e = v2 - v1;
	b2Vec2 n(e.y, -e.x);
	n.Normalize();

	// Determine side
	bool isFrontSide = b2Dot(n, polygonB.m_centroid - v1) >= 0.0f;
	if (isFrontSide == false)
	{
		// flip normal for backside collision
		n = -n;
	}

	// Compute primary separating axis
	b2EPAxis primaryAxis = b2EPSeparation(v1, v2, n, &polygonB);
	if (primaryAxis.separation > 0.0f)
	{
		// Shapes are separated
		return;
	}

	// Classify adjacent edges
	b2EdgeType type1 = e_isolated, type2 = e_isolated;
	if (edgeA->m_hasVertex0)
	{
		b2Vec2 v0 = edgeA->m_vertex0;
		float32 s = b2Dot(n, v0 - v1);
		if (s > 0.1f * b2_linearSlop)
		{
			type1 = e_concave;
		}
		else if (s >= -0.1f * b2_linearSlop)
		{
			type1 = e_flat;
		}
		else
		{
			type1 = e_convex;
		}
	}

	if (edgeA->m_hasVertex3)
	{
		b2Vec2 v3 = edgeA->m_vertex3;
		float32 s = b2Dot(n, v3 - v2);
		if (s > 0.1f * b2_linearSlop)
		{
			type2 = e_concave;
		}
		else if (s >= -0.1f * b2_linearSlop)
		{
			type2 = e_flat;
		}
		else
		{
			type2 = e_convex;
		}
	}

	if (type1 == e_convex)
	{
		// Check separation on previous edge.
		b2Vec2 v0 = edgeA->m_vertex0;
		b2Vec2 e0 = v1 - v0;

		b2Vec2 n0(e0.y, -e0.x);
		n0.Normalize();
		if (isFrontSide == false)
		{
			n0 = -n0;
		}

		b2EPAxis axis1 = b2EPSeparation(v0, v1, n0, &polygonB);
		if (axis1.separation > primaryAxis.separation)
		{
			return;
		}
	}

	if (type2 == e_convex)
	{
		// Check separation on next edge.
		b2Vec2 v3 = edgeA->m_vertex3;
		b2Vec2 e2 = v3 - v2;

		b2Vec2 n2(e2.y, -e2.x);
		n2.Normalize();
		if (isFrontSide == false)
		{
			n2 = -n2;
		}

		b2EPAxis axis2 = b2EPSeparation(v2, v3, n2, &polygonB);
		if (axis2.separation > primaryAxis.separation)
		{
			return;
		}
	}

	//if (primaryAxis.type == b2EPAxis::e_edgeA)
	//{

	//}
}
