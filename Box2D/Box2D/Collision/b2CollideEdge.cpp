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
	b2_isolated,
	b2_concave,
	b2_flat,
	b2_convex
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

		cf.indexA = 0;
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

		cf.indexA = 1;
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
	cf.typeA = b2ContactFeature::e_face;
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

#if 0
struct b2FatEdge
{
	b2Vec2 v0, v1, v2, v3;
	b2Vec2 normal;
	bool hasVertex0, hasVertex3;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
struct b2EPCollider
{
	b2EPCollider(const b2EdgeShape* edgeA, const b2Transform& xfA,
				const b2PolygonShape* polygonB_in, const b2Transform& xfB);

	void Collide(b2Manifold* manifold);

	void ComputeAdjacency();
	b2EPAxis ComputeEdgeSeparation();
	b2EPAxis ComputePolygonSeparation();
	void FindIncidentEdge(b2ClipVertex c[2], const b2PolygonShape* poly1, int32 edge1, const b2PolygonShape* poly2);

	b2FatEdge m_edgeA;
	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;
	b2Transform m_xf;
	b2Vec2 m_normal0, m_normal2;
	b2Vec2 m_limit11, m_limit12;
	b2Vec2 m_limit21, m_limit22;
	float32 m_radius;
};

b2EPCollider::b2EPCollider(const b2EdgeShape* edgeA, const b2Transform& xfA,
				const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	m_xf = b2MulT(xfA, xfB);

	// Create a polygon for edge shape A
	m_polygonA.SetAsEdge(edgeA->m_vertex1, edgeA->m_vertex2);

	// Build polygonB in frame A
	m_polygonB.m_radius = polygonB->m_radius;
	m_polygonB.m_vertexCount = polygonB->m_vertexCount;
	m_polygonB.m_centroid = b2Mul(m_xf, polygonB->m_centroid);
	for (int32 i = 0; i < m_polygonB.m_vertexCount; ++i)
	{
		m_polygonB.m_vertices[i] = b2Mul(m_xf, polygonB->m_vertices[i]);
		m_polygonB.m_normals[i] = b2Mul(m_xf.R, polygonB->m_normals[i]);
	}

	m_radius = m_polygonA.m_radius + m_polygonB.m_radius;

	// Edge geometry
	m_edgeA.v0 = edgeA->m_vertex0;
	m_edgeA.v1 = edgeA->m_vertex1;
	m_edgeA.v2 = edgeA->m_vertex2;
	m_edgeA.v3 = edgeA->m_vertex3;
	b2Vec2 e = m_edgeA.v2 - m_edgeA.v1;

	// Normal points outwards in CCW order.
	m_edgeA.normal.Set(e.y, -e.x);
	m_edgeA.normal.Normalize();
	m_edgeA.hasVertex0 = edgeA->m_hasVertex0;
	m_edgeA.hasVertex3 = edgeA->m_hasVertex3;

	m_limit11.SetZero();
	m_limit12.SetZero();
	m_limit21.SetZero();
	m_limit22.SetZero();
}

// Collide an edge and polygon. This uses the SAT and clipping to produce up to 2 contact points.
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
void b2EPCollider::Collide(b2Manifold* manifold)
{
	manifold->pointCount = 0;

	ComputeAdjacency();

	b2EPAxis edgeAxis = ComputeEdgeSeparation();

	// If no valid normal can be found than this edge should not collide.
	// This can happen on the middle edge of a 3-edge zig-zag chain.
	if (edgeAxis.type == b2EPAxis::e_unknown)
	{
		return;
	}

	if (edgeAxis.separation > m_radius)
	{
		return;
	}

	b2EPAxis polygonAxis = ComputePolygonSeparation();
	if (polygonAxis.type != b2EPAxis::e_unknown && polygonAxis.separation > m_radius)
	{
		return;
	}

	// Use hysteresis for jitter reduction.
	const float32 k_relativeTol = 0.98f;
	const float32 k_absoluteTol = 0.001f;

	b2EPAxis primaryAxis;
	if (polygonAxis.type == b2EPAxis::e_unknown)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}

	b2PolygonShape* poly1;
	b2PolygonShape* poly2;
	b2ClipVertex incidentEdge[2];
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		poly1 = &m_polygonA;
		poly2 = &m_polygonB;
		manifold->type = b2Manifold::e_faceA;
	}
	else
	{
		poly1 = &m_polygonB;
		poly2 = &m_polygonA;
		manifold->type = b2Manifold::e_faceB;
	}

	int32 edge1 = primaryAxis.index;

	FindIncidentEdge(incidentEdge, poly1, primaryAxis.index, poly2);
	int32 count1 = poly1->m_vertexCount;
	const b2Vec2* vertices1 = poly1->m_vertices;

	int32 iv1 = edge1;
	int32 iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	b2Vec2 v11 = vertices1[iv1];
	b2Vec2 v12 = vertices1[iv2];

	b2Vec2 tangent = v12 - v11;
	tangent.Normalize();
	
	b2Vec2 normal = b2Cross(tangent, 1.0f);
	b2Vec2 planePoint = 0.5f * (v11 + v12);

	// Face offset.
	float32 frontOffset = b2Dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	float32 sideOffset1 = -b2Dot(tangent, v11) + m_radius;
	float32 sideOffset2 = b2Dot(tangent, v12) + m_radius;

	// Clip incident edge against extruded edge1 side edges.
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

	if (np < b2_maxManifoldPoints)
	{
		return;
	}

	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < b2_maxManifoldPoints)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->localNormal = normal;
		manifold->localPoint = planePoint;
	}
	else
	{
		manifold->localNormal = b2MulT(m_xf.R, normal);
		manifold->localPoint = b2MulT(m_xf, planePoint);
	}

	int32 pointCount = 0;
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
	{
		float32 separation;
		
		separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;

		if (separation <= m_radius)
		{
			b2ManifoldPoint* cp = manifold->points + pointCount;

			if (primaryAxis.type == b2EPAxis::e_edgeA)
			{
				cp->localPoint = b2MulT(m_xf, clipPoints2[i].v);
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint = clipPoints2[i].v;
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}

			++pointCount;
		}
	}

	manifold->pointCount = pointCount;
}

// Compute allowable normal ranges based on adjacency.
// A normal n is allowable iff:
// cross(n, n1) >= 0.0f && cross(n2, n) >= 0.0f
// n points from A to B (edge to polygon)
void b2EPCollider::ComputeAdjacency()
{
	b2Vec2 v0 = m_edgeA.v0;
	b2Vec2 v1 = m_edgeA.v1;
	b2Vec2 v2 = m_edgeA.v2;
	b2Vec2 v3 = m_edgeA.v3;

	// Determine allowable the normal regions based on adjacency.
	// Note: it may be possible that no normal is admissable.
	b2Vec2 centerB = m_polygonB.m_centroid;
	if (m_edgeA.hasVertex0)
	{
		b2Vec2 e0 = v1 - v0;
		b2Vec2 e1 = v2 - v1;
		b2Vec2 n0(e0.y, -e0.x);
		b2Vec2 n1(e1.y, -e1.x);
		n0.Normalize();
		n1.Normalize();

		bool convex = b2Cross(n0, n1) >= 0.0f;
		bool front0 = b2Dot(n0, centerB - v0) >= 0.0f;
		bool front1 = b2Dot(n1, centerB - v1) >= 0.0f;

		if (convex)
		{
			if (front0 || front1)
			{
				m_limit11 = n1;
				m_limit12 = n0;
			}
			else
			{
				m_limit11 = -n1;
				m_limit12 = -n0;
			}
		}
		else
		{
			if (front0 && front1)
			{
				m_limit11 = n0;
				m_limit12 = n1;
			}
			else
			{
				m_limit11 = -n0;
				m_limit12 = -n1;
			}
		}
	}
	else
	{
		m_limit11.SetZero();
		m_limit12.SetZero();
	}

	if (m_edgeA.hasVertex3)
	{
		b2Vec2 e1 = v2 - v1;
		b2Vec2 e2 = v3 - v2;
		b2Vec2 n1(e1.y, -e1.x);
		b2Vec2 n2(e2.y, -e2.x);
		n1.Normalize();
		n2.Normalize();

		bool convex = b2Cross(n1, n2) >= 0.0f;
		bool front1 = b2Dot(n1, centerB - v1) >= 0.0f;
		bool front2 = b2Dot(n2, centerB - v2) >= 0.0f;

		if (convex)
		{
			if (front1 || front2)
			{
				m_limit21 = n2;
				m_limit22 = n1;
			}
			else
			{
				m_limit21 = -n2;
				m_limit22 = -n1;
			}
		}
		else
		{
			if (front1 && front2)
			{
				m_limit21 = n1;
				m_limit22 = n2;
			}
			else
			{
				m_limit21 = -n1;
				m_limit22 = -n2;
			}
		}
	}
	else
	{
		m_limit21.SetZero();
		m_limit22.SetZero();
	}
}

b2EPAxis b2EPCollider::ComputeEdgeSeparation()
{
	// EdgeA separation
	b2EPAxis bestAxis;
	bestAxis.type = b2EPAxis::e_unknown;
	bestAxis.index = -1;
	bestAxis.separation = -FLT_MAX;
	b2Vec2 normals[2] = {m_edgeA.normal, -m_edgeA.normal};
	
	for (int32 i = 0; i < 2; ++i)
	{
		b2Vec2 n = normals[i];

		// Adjacency
		bool valid1 = b2Cross(n, m_limit11) >= -b2_angularSlop && b2Cross(m_limit12, n) >= -b2_angularSlop;
		bool valid2 = b2Cross(n, m_limit21) >= -b2_angularSlop && b2Cross(m_limit22, n) >= -b2_angularSlop;

		if (valid1 == false || valid2 == false)
		{
			continue;
		}
		
		b2EPAxis axis;
		axis.type = b2EPAxis::e_edgeA;
		axis.index = i;
		axis.separation = FLT_MAX;

		for (int32 j = 0; j < m_polygonB.m_vertexCount; ++j)
		{
			float32 s = b2Dot(n, m_polygonB.m_vertices[j] - m_edgeA.v1);
			if (s < axis.separation)
			{
				axis.separation = s;
			}
		}

		if (axis.separation > m_radius)
		{
			return axis;
		}

		if (axis.separation > bestAxis.separation)
		{
			bestAxis = axis;
		}
	}

	return bestAxis;
}

b2EPAxis b2EPCollider::ComputePolygonSeparation()
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -FLT_MAX;
	for (int32 i = 0; i < m_polygonB.m_vertexCount; ++i)
	{
		b2Vec2 n = -m_polygonB.m_normals[i];

		// Adjacency
		bool valid1 = b2Cross(n, m_limit11) >= -b2_angularSlop && b2Cross(m_limit12, n) >= -b2_angularSlop;
		bool valid2 = b2Cross(n, m_limit21) >= -b2_angularSlop && b2Cross(m_limit22, n) >= -b2_angularSlop;

		if (valid1 == false || valid2 == false)
		{
			continue;
		}

		float32 s1 = b2Dot(n, m_polygonB.m_vertices[i] - m_edgeA.v1);
		float32 s2 = b2Dot(n, m_polygonB.m_vertices[i] - m_edgeA.v2);
		float32 s = b2Min(s1, s2);

		if (s > m_radius)
		{
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}

		if (s > axis.separation)
		{
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}

	return axis;
}

void b2EPCollider::FindIncidentEdge(b2ClipVertex c[2], const b2PolygonShape* poly1, int32 edge1, const b2PolygonShape* poly2)
{
	int32 count1 = poly1->m_vertexCount;
	const b2Vec2* normals1 = poly1->m_normals;

	int32 count2 = poly2->m_vertexCount;
	const b2Vec2* vertices2 = poly2->m_vertices;
	const b2Vec2* normals2 = poly2->m_normals;

	b2Assert(0 <= edge1 && edge1 < count1);

	// Get the normal of the reference edge in poly2's frame.
	b2Vec2 normal1 = normals1[edge1];

	// Find the incident edge on poly2.
	int32 index = 0;
	float32 minDot = b2_maxFloat;
	for (int32 i = 0; i < count2; ++i)
	{
		float32 dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int32 i1 = index;
	int32 i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = vertices2[i1];
	c[0].id.cf.indexA = (uint8)edge1;
	c[0].id.cf.indexB = (uint8)i1;
	c[0].id.cf.typeA = b2ContactFeature::e_face;
	c[0].id.cf.typeB = b2ContactFeature::e_vertex;

	c[1].v = vertices2[i2];
	c[1].id.cf.indexA = (uint8)edge1;
	c[1].id.cf.indexB = (uint8)i2;
	c[1].id.cf.typeA = b2ContactFeature::e_face;
	c[1].id.cf.typeB = b2ContactFeature::e_vertex;
}

void b2CollideEdgeAndPolygon(	b2Manifold* manifold,
								const b2EdgeShape* edgeA, const b2Transform& xfA,
								const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	b2EPCollider collider(edgeA, xfA, polygonB, xfB);
	collider.Collide(manifold);
}

#else

static b2EPAxis b2EPEdgeSeparation(const b2Vec2& v1, const b2Vec2& v2, const b2Vec2& n, const b2PolygonShape* polygonB, float32 radius)
{
	// EdgeA separation
	b2EPAxis axis;
	axis.type = b2EPAxis::e_edgeA;
	axis.index = 0;
	axis.separation = b2Dot(n, polygonB->m_vertices[0] - v1);
	for (int32 i = 1; i < polygonB->m_vertexCount; ++i)
	{
		float32 s = b2Dot(n, polygonB->m_vertices[i] - v1);
		if (s < axis.separation)
		{
			axis.separation = s;
		}
	}

	return axis;
}

static b2EPAxis b2EPPolygonSeparation(const b2Vec2& v1, const b2Vec2& v2, const b2Vec2& n, const b2PolygonShape* polygonB, float32 radius)
{
	// PolygonB separation
	b2EPAxis axis;
	axis.type = b2EPAxis::e_edgeB;
	axis.index = 0;
	axis.separation = -FLT_MAX;
	for (int32 i = 0; i < polygonB->m_vertexCount; ++i)
	{
		float32 s1 = b2Dot(polygonB->m_normals[i], v1 - polygonB->m_vertices[i]);	
		float32 s2 = b2Dot(polygonB->m_normals[i], v2 - polygonB->m_vertices[i]);
		float32 s = b2Min(s1, s2);
		if (s > axis.separation)
		{
			axis.index = i;
			axis.separation = s;
			if (s > radius)
			{
				return axis;
			}
		}
	}

	return axis;
}

static void b2FindIncidentEdge(b2ClipVertex c[2], const b2PolygonShape* poly1, int32 edge1, const b2PolygonShape* poly2)
{
	int32 count1 = poly1->m_vertexCount;
	const b2Vec2* normals1 = poly1->m_normals;

	int32 count2 = poly2->m_vertexCount;
	const b2Vec2* vertices2 = poly2->m_vertices;
	const b2Vec2* normals2 = poly2->m_normals;

	b2Assert(0 <= edge1 && edge1 < count1);

	// Get the normal of the reference edge in poly2's frame.
	b2Vec2 normal1 = normals1[edge1];

	// Find the incident edge on poly2.
	int32 index = 0;
	float32 minDot = b2_maxFloat;
	for (int32 i = 0; i < count2; ++i)
	{
		float32 dot = b2Dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int32 i1 = index;
	int32 i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v = vertices2[i1];
	c[0].id.cf.indexA = (uint8)edge1;
	c[0].id.cf.indexB = (uint8)i1;
	c[0].id.cf.typeA = b2ContactFeature::e_face;
	c[0].id.cf.typeB = b2ContactFeature::e_vertex;

	c[1].v = vertices2[i2];
	c[1].id.cf.indexA = (uint8)edge1;
	c[1].id.cf.indexB = (uint8)i2;
	c[1].id.cf.typeA = b2ContactFeature::e_face;
	c[1].id.cf.typeB = b2ContactFeature::e_vertex;
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

	// Create a polygon for edge shape A
	b2PolygonShape polygonA;
	polygonA.SetAsEdge(edgeA->m_vertex1, edgeA->m_vertex2);

	// Build polygonB in frame A
	b2PolygonShape polygonB;
	polygonB.m_radius = polygonB_in->m_radius;
	polygonB.m_vertexCount = polygonB_in->m_vertexCount;
	polygonB.m_centroid = b2Mul(xf, polygonB_in->m_centroid);
	for (int32 i = 0; i < polygonB.m_vertexCount; ++i)
	{
		polygonB.m_vertices[i] = b2Mul(xf, polygonB_in->m_vertices[i]);
		polygonB.m_normals[i] = b2Mul(xf.R, polygonB_in->m_normals[i]);
	}

	float32 totalRadius = polygonA.m_radius + polygonB.m_radius;

	// Edge geometry
	b2Vec2 v1 = edgeA->m_vertex1;
	b2Vec2 v2 = edgeA->m_vertex2;
	b2Vec2 e = v2 - v1;
	b2Vec2 edgeNormal(e.y, -e.x);
	edgeNormal.Normalize();

	// Determine side
	bool isFrontSide = b2Dot(edgeNormal, polygonB.m_centroid - v1) >= 0.0f;
	if (isFrontSide == false)
	{
		edgeNormal = -edgeNormal;
	}

	// Compute primary separating axis
	b2EPAxis edgeAxis = b2EPEdgeSeparation(v1, v2, edgeNormal, &polygonB, totalRadius);
	if (edgeAxis.separation > totalRadius)
	{
		// Shapes are separated
		return;
	}

	// Classify adjacent edges
	b2EdgeType types[2] = {b2_isolated, b2_isolated};
	if (edgeA->m_hasVertex0)
	{
		b2Vec2 v0 = edgeA->m_vertex0;
		float32 s = b2Dot(edgeNormal, v0 - v1);

		if (s > 0.1f * b2_linearSlop)
		{
			types[0] = b2_concave;
		}
		else if (s >= -0.1f * b2_linearSlop)
		{
			types[0] = b2_flat;
		}
		else
		{
			types[0] = b2_convex;
		}
	}

	if (edgeA->m_hasVertex3)
	{
		b2Vec2 v3 = edgeA->m_vertex3;
		float32 s = b2Dot(edgeNormal, v3 - v2);
		if (s > 0.1f * b2_linearSlop)
		{
			types[1] = b2_concave;
		}
		else if (s >= -0.1f * b2_linearSlop)
		{
			types[1] = b2_flat;
		}
		else
		{
			types[1] = b2_convex;
		}
	}

	if (types[0] == b2_convex)
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

		b2EPAxis axis1 = b2EPEdgeSeparation(v0, v1, n0, &polygonB, totalRadius);
		if (axis1.separation > edgeAxis.separation)
		{
			// The polygon should collide with previous edge
			return;
		}
	}

	if (types[1] == b2_convex)
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

		b2EPAxis axis2 = b2EPEdgeSeparation(v2, v3, n2, &polygonB, totalRadius);
		if (axis2.separation > edgeAxis.separation)
		{
			// The polygon should collide with the next edge
			return;
		}
	}

	b2EPAxis polygonAxis = b2EPPolygonSeparation(v1, v2, edgeNormal, &polygonB, totalRadius);
	if (polygonAxis.separation > totalRadius)
	{
		return;
	}

	// Use hysteresis for jitter reduction.
	const float32 k_relativeTol = 0.98f;
	const float32 k_absoluteTol = 0.001f;

	b2EPAxis primaryAxis;
	if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}

	b2PolygonShape* poly1;
	b2PolygonShape* poly2;
	b2ClipVertex incidentEdge[2];
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		poly1 = &polygonA;
		poly2 = &polygonB;
		if (isFrontSide == false)
		{
			primaryAxis.index = 1;
		}
		manifold->type = b2Manifold::e_faceA;
	}
	else
	{
		poly1 = &polygonB;
		poly2 = &polygonA;
		manifold->type = b2Manifold::e_faceB;
	}

	int32 edge1 = primaryAxis.index;

	b2FindIncidentEdge(incidentEdge, poly1, primaryAxis.index, poly2);
	int32 count1 = poly1->m_vertexCount;
	const b2Vec2* vertices1 = poly1->m_vertices;

	int32 iv1 = edge1;
	int32 iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	b2Vec2 v11 = vertices1[iv1];
	b2Vec2 v12 = vertices1[iv2];

	b2Vec2 tangent = v12 - v11;
	tangent.Normalize();
	
	b2Vec2 normal = b2Cross(tangent, 1.0f);
	b2Vec2 planePoint = 0.5f * (v11 + v12);

	// Face offset.
	float32 frontOffset = b2Dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	float32 sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
	float32 sideOffset2 = b2Dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

	if (np < b2_maxManifoldPoints)
	{
		return;
	}

	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < b2_maxManifoldPoints)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->localNormal = normal;
		manifold->localPoint = planePoint;
	}
	else
	{
		manifold->localNormal = b2MulT(xf.R, normal);
		manifold->localPoint = b2MulT(xf, planePoint);
	}

	int32 pointCount = 0;
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
	{
		float32 separation;
		
		separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;

		if (separation <= totalRadius)
		{
			b2ManifoldPoint* cp = manifold->points + pointCount;

			if (primaryAxis.type == b2EPAxis::e_edgeA)
			{
				cp->localPoint = b2MulT(xf, clipPoints2[i].v);
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint = clipPoints2[i].v;
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}

			if (cp->id.cf.typeA == b2ContactFeature::e_vertex && types[cp->id.cf.indexA] == b2_flat)
			{
				continue;
			}

			++pointCount;
		}
	}

	manifold->pointCount = pointCount;
}
#endif
