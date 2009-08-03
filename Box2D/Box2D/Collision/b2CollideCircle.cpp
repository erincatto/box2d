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
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

void b2CollideCircles(
	b2Manifold* manifold,
	const b2CircleShape* circle1, const b2Transform& xf1,
	const b2CircleShape* circle2, const b2Transform& xf2)
{
	manifold->m_pointCount = 0;

	b2Vec2 p1 = b2Mul(xf1, circle1->m_p);
	b2Vec2 p2 = b2Mul(xf2, circle2->m_p);

	b2Vec2 d = p2 - p1;
	float32 distSqr = b2Dot(d, d);
	float32 radius = circle1->m_radius + circle2->m_radius;
	if (distSqr > radius * radius)
	{
		return;
	}

	manifold->m_type = b2Manifold::e_circles;
	manifold->m_localPoint = circle1->m_p;
	manifold->m_localPlaneNormal.SetZero();
	manifold->m_pointCount = 1;

	manifold->m_points[0].m_localPoint = circle2->m_p;
	manifold->m_points[0].m_id.key = 0;
}

void b2CollidePolygonAndCircle(
	b2Manifold* manifold,
	const b2PolygonShape* polygon, const b2Transform& xf1,
	const b2CircleShape* circle, const b2Transform& xf2)
{
	manifold->m_pointCount = 0;

	// Compute circle position in the frame of the polygon.
	b2Vec2 c = b2Mul(xf2, circle->m_p);
	b2Vec2 cLocal = b2MulT(xf1, c);

	// Find the min separating edge.
	int32 normalIndex = 0;
	float32 separation = -B2_FLT_MAX;
	float32 radius = polygon->m_radius + circle->m_radius;
	int32 vertexCount = polygon->m_vertexCount;
	const b2Vec2* vertices = polygon->m_vertices;
	const b2Vec2* normals = polygon->m_normals;

	for (int32 i = 0; i < vertexCount; ++i)
	{
		float32 s = b2Dot(normals[i], cLocal - vertices[i]);

		if (s > radius)
		{
			// Early out.
			return;
		}

		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	int32 vertIndex1 = normalIndex;
	int32 vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	b2Vec2 v1 = vertices[vertIndex1];
	b2Vec2 v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < B2_FLT_EPSILON)
	{
		manifold->m_pointCount = 1;
		manifold->m_type = b2Manifold::e_faceA;
		manifold->m_localPlaneNormal = normals[normalIndex];
		manifold->m_localPoint = 0.5f * (v1 + v2);
		manifold->m_points[0].m_localPoint = circle->m_p;
		manifold->m_points[0].m_id.key = 0;
		return;
	}

	// Compute barycentric coordinates
	float32 u1 = b2Dot(cLocal - v1, v2 - v1);
	float32 u2 = b2Dot(cLocal - v2, v1 - v2);
	if (u1 <= 0.0f)
	{
		if (b2DistanceSquared(cLocal, v1) > radius * radius)
		{
			return;
		}

		manifold->m_pointCount = 1;
		manifold->m_type = b2Manifold::e_faceA;
		manifold->m_localPlaneNormal = cLocal - v1;
		manifold->m_localPlaneNormal.Normalize();
		manifold->m_localPoint = v1;
		manifold->m_points[0].m_localPoint = circle->m_p;
		manifold->m_points[0].m_id.key = 0;
	}
	else if (u2 <= 0.0f)
	{
		if (b2DistanceSquared(cLocal, v2) > radius * radius)
		{
			return;
		}

		manifold->m_pointCount = 1;
		manifold->m_type = b2Manifold::e_faceA;
		manifold->m_localPlaneNormal = cLocal - v2;
		manifold->m_localPlaneNormal.Normalize();
		manifold->m_localPoint = v2;
		manifold->m_points[0].m_localPoint = circle->m_p;
		manifold->m_points[0].m_id.key = 0;
	}
	else
	{
		b2Vec2 faceCenter = 0.5f * (v1 + v2);
		float32 separation = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
		if (separation > radius)
		{
			return;
		}

		manifold->m_pointCount = 1;
		manifold->m_type = b2Manifold::e_faceA;
		manifold->m_localPlaneNormal = normals[vertIndex1];
		manifold->m_localPoint = faceCenter;
		manifold->m_points[0].m_localPoint = circle->m_p;
		manifold->m_points[0].m_id.key = 0;
	}
}
