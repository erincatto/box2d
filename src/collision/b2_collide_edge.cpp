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

#include "box2d/b2_collision.h"
#include "box2d/b2_circle_shape.h"
#include "box2d/b2_edge_shape.h"
#include "box2d/b2_polygon_shape.h"


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
	float u = b2Dot(e, B - Q);
	float v = b2Dot(e, Q - A);
	
	float radius = edgeA->m_radius + circleB->m_radius;
	
	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		b2Vec2 P = A;
		b2Vec2 d = Q - P;
		float dd = b2Dot(d, d);
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
			float u1 = b2Dot(e1, B1 - Q);
			
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
		float dd = b2Dot(d, d);
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
			float v2 = b2Dot(e2, Q - A2);
			
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
	float den = b2Dot(e, e);
	b2Assert(den > 0.0f);
	b2Vec2 P = (1.0f / den) * (u * A + v * B);
	b2Vec2 d = Q - P;
	float dd = b2Dot(d, d);
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

// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	};
	
	b2Vec2 normal;
	Type type;
	int32 index;
	float separation;
};

// This holds polygon B expressed in frame A.
struct b2TempPolygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	int32 count;
};

// Reference face used for clipping
struct b2ReferenceFace
{
	int32 i1, i2;
	
	b2Vec2 v1, v2;
	
	b2Vec2 normal;
	
	b2Vec2 sideNormal1;
	float sideOffset1;
	
	b2Vec2 sideNormal2;
	float sideOffset2;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
struct b2EPCollider
{
	// Chain has normals pointing to the left
	// Algorithm:
	// 1. Classify v1 and v2
	// 2. Classify polygon centroid as front or back
	// 3. Flip normal if necessary
	// 4. Initialize normal range to [-pi, pi] about face normal
	// 5. Adjust normal range according to adjacent edges
	// 6. Visit each separating axes, only accept axes within the range
	// 7. Return if _any_ axis indicates separation
	// 8. Clip
	void b2EPCollider::Collide(b2Manifold* manifold,
		const b2EdgeShape* edgeA, const b2Transform& xfA,
		const b2PolygonShape* polygonB, const b2Transform& xfB)
	{
		b2Transform xf = b2MulT(xfA, xfB);

		b2Vec2 centroidB = b2Mul(xf, polygonB->m_centroid);

		m_v1 = edgeA->m_vertex1;
		m_v2 = edgeA->m_vertex2;

		b2Vec2 v0 = edgeA->m_vertex0;
		b2Vec2 v3 = edgeA->m_vertex3;

		bool hasVertex0 = edgeA->m_hasVertex0;
		bool hasVertex3 = edgeA->m_hasVertex3;

		b2Vec2 edge1 = m_v2 - m_v1;
		edge1.Normalize();
		m_normal1.Set(-edge1.y, edge1.x);
		float offset1 = b2Dot(m_normal1, centroidB - m_v1);
		float offset0 = 0.0f, offset2 = 0.0f;
		bool convex1 = false, convex2 = false;
		b2Vec2 normal0, normal2;
		bool front = false;

		// Is there a preceding edge?
		if (hasVertex0)
		{
			b2Vec2 edge0 = m_v1 - v0;
			edge0.Normalize();
			normal0.Set(-edge0.y, edge0.x);
			convex1 = b2Cross(edge0, edge1) <= 0.0f;
			offset0 = b2Dot(normal0, centroidB - v0);
		}

		// Is there a following edge?
		if (hasVertex3)
		{
			b2Vec2 edge2 = v3 - m_v2;
			edge2.Normalize();
			normal2.Set(-edge2.y, edge2.x);
			convex2 = b2Cross(edge1, edge2) <= 0.0f;
			offset2 = b2Dot(normal2, centroidB - m_v2);
		}

		// Determine front or back collision. Determine collision normal limits.
		if (hasVertex0 && hasVertex3)
		{
			bool front0 = offset0 >= 0.0f;
			bool front1 = offset1 >= 0.0f;
			bool front2 = offset2 >= 0.0f;

			if (convex1 && convex2)
			{
				// convex-convex
				front = front0 || front1 || front2;
			}
			else if (convex1)
			{
				// convex-concave
				if (front2)
				{
					if (front1)
					{
						if (front0)
						{
							// FFF
							front = true;
						}
						else
						{
							// BFF
							front = true;
						}
					}
					else
					{
						if (front0)
						{
							// FBF
							front = true;
							return;
						}
						else
						{
							// BBF
							front = false;
						}
					}
				}
				else
				{
					if (front1)
					{
						if (front0)
						{
							// FFB
							front = false;
							return;
						}
						else
						{
							// BFB
							front = false;
							return;
						}
					}
					else
					{
						if (front0)
						{
							// FBB
							front = true;
							return;
						}
						else
						{
							// BBB
							front = false;
						}
					}
				}
			}
			else if (convex2)
			{
				// concave-convex
				if (front0)
				{
					if (front1)
					{
						if (front2)
						{
							// FFF
							front = true;
						}
						else
						{
							// FFB
							front = true;
						}
					}
					else
					{
						if (front2)
						{
							// FBF
							front = true;
							return;
						}
						else
						{
							// FBB
							front = false;
						}
					}
				}
				else
				{
					if (front1)
					{
						if (front2)
						{
							// BFF
							front = false;
							return;
						}
						else
						{
							// BFB
							front = false;
							return;
						}
					}
					else
					{
						if (front2)
						{
							// BBF
							front = true;
							return;
						}
						else
						{
							// BBB
							front = false;
						}
					}
				}
			}
			else
			{
				// concave-concave
				front = front0 && front1 && front2;
			}
		}
		else if (hasVertex0)
		{
			if (convex1)
			{
				front = offset0 >= 0.0f || offset1 >= 0.0f;
			}
			else
			{
				front = offset0 >= 0.0f && offset1 >= 0.0f;
			}
		}
		else if (hasVertex3)
		{
			if (convex2)
			{
				front = offset1 >= 0.0f || offset2 >= 0.0f;
			}
			else
			{
				front = offset1 >= 0.0f && offset2 >= 0.0f;
			}		
		}
		else
		{
			front = offset1 >= 0.0f;
		}

		// Get polygonB in frameA
		m_polygonB.count = polygonB->m_count;
		for (int32 i = 0; i < polygonB->m_count; ++i)
		{
			m_polygonB.vertices[i] = b2Mul(xf, polygonB->m_vertices[i]);
			m_polygonB.normals[i] = b2Mul(xf.q, polygonB->m_normals[i]);
		}

		m_radius = polygonB->m_radius + edgeA->m_radius;

		manifold->pointCount = 0;

		b2EPAxis edgeAxis = ComputeEdgeSeparation();
		if (edgeAxis.separation > m_radius)
		{
			return;
		}

		b2EPAxis polygonAxis = ComputePolygonSeparation();
		if (polygonAxis.separation > m_radius)
		{
			return;
		}

		// Use hysteresis for jitter reduction.
		const float k_relativeTol = 0.98f;
		const float k_absoluteTol = 0.001f;

		b2EPAxis primaryAxis;
		if (polygonAxis.separation - m_radius > k_relativeTol * (edgeAxis.separation - m_radius) + k_absoluteTol)
		{
			primaryAxis = polygonAxis;
		}
		else
		{
			primaryAxis = edgeAxis;
		}

		const float sinTol = 0.1f;
		bool side1 = b2Dot(primaryAxis.normal, edge1) <= 0.0f;
		if (hasVertex0 && hasVertex3)
		{
			// Check Gauss Map
			if (front)
			{
				if (side1)
				{
					if (convex1)
					{
						if (b2Cross(normal0, primaryAxis.normal) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
				else
				{
					if (convex2)
					{
						if (b2Cross(primaryAxis.normal, normal2) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
			else
			{
				// Welcome to the upside-down
				if (side1)
				{
					if (convex1 == false)
					{
						if (b2Cross(primaryAxis.normal, -normal0) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
				else
				{
					if (convex2 == false)
					{
						if (b2Cross(-normal2, primaryAxis.normal) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
		}
		else if (hasVertex0)
		{
			// Check Gauss Map
			if (front)
			{
				if (side1)
				{
					if (convex1)
					{
						if (b2Cross(normal0, primaryAxis.normal) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
			else
			{
				// Welcome to the upside-down
				if (side1)
				{
					if (convex1 == false)
					{
						if (b2Cross(primaryAxis.normal, -normal0) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
		}
		else if (hasVertex3)
		{
			// Check Gauss Map
			if (front)
			{
				if (side1 == false)
				{
					if (convex2)
					{
						if (b2Cross(primaryAxis.normal, normal2) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
			else
			{
				// Welcome to the upside-down
				if (side1 == false)
				{
					if (convex2 == false)
					{
						if (b2Cross(-normal2, primaryAxis.normal) > sinTol)
						{
							// Skip region
							return;
						}
					}
					else
					{
						// Snap region
						primaryAxis = edgeAxis;
					}
				}
			}
		}

		b2ClipVertex clipPoints[2];
		b2ReferenceFace ref;
		if (primaryAxis.type == b2EPAxis::e_edgeA)
		{
			manifold->type = b2Manifold::e_faceA;

			// Search for the polygon normal that is most anti-parallel to the edge normal.
			int32 bestIndex = 0;
			float bestValue = b2Dot(primaryAxis.normal, m_polygonB.normals[0]);
			for (int32 i = 1; i < m_polygonB.count; ++i)
			{
				float value = b2Dot(primaryAxis.normal, m_polygonB.normals[i]);
				if (value < bestValue)
				{
					bestValue = value;
					bestIndex = i;
				}
			}

			int32 i1 = bestIndex;
			int32 i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;

			clipPoints[0].v = m_polygonB.vertices[i1];
			clipPoints[0].id.cf.indexA = 0;
			clipPoints[0].id.cf.indexB = static_cast<uint8>(i1);
			clipPoints[0].id.cf.typeA = b2ContactFeature::e_face;
			clipPoints[0].id.cf.typeB = b2ContactFeature::e_vertex;

			clipPoints[1].v = m_polygonB.vertices[i2];
			clipPoints[1].id.cf.indexA = 0;
			clipPoints[1].id.cf.indexB = static_cast<uint8>(i2);
			clipPoints[1].id.cf.typeA = b2ContactFeature::e_face;
			clipPoints[1].id.cf.typeB = b2ContactFeature::e_vertex;

			// TODO does order matter?
			if (front)
			{
				ref.i1 = 1;
				ref.i2 = 0;
				ref.v1 = m_v2;
				ref.v2 = m_v1;
				ref.normal = primaryAxis.normal;
				ref.sideNormal1 = edge1;
				ref.sideNormal2 = -edge1;
			}
			else
			{
				ref.i1 = 0;
				ref.i2 = 1;
				ref.v1 = m_v1;
				ref.v2 = m_v2;
				ref.normal = primaryAxis.normal;
				ref.sideNormal1 = -edge1;
				ref.sideNormal2 = edge1;
			}		
		}
		else
		{
			manifold->type = b2Manifold::e_faceB;

			clipPoints[0].v = m_v2;
			clipPoints[0].id.cf.indexA = 1;
			clipPoints[0].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
			clipPoints[0].id.cf.typeA = b2ContactFeature::e_vertex;
			clipPoints[0].id.cf.typeB = b2ContactFeature::e_face;

			clipPoints[1].v = m_v1;
			clipPoints[1].id.cf.indexA = 0;
			clipPoints[1].id.cf.indexB = static_cast<uint8>(primaryAxis.index);		
			clipPoints[1].id.cf.typeA = b2ContactFeature::e_vertex;
			clipPoints[1].id.cf.typeB = b2ContactFeature::e_face;

			ref.i1 = primaryAxis.index;
			ref.i2 = ref.i1 + 1 < m_polygonB.count ? ref.i1 + 1 : 0;
			ref.v1 = m_polygonB.vertices[ref.i1];
			ref.v2 = m_polygonB.vertices[ref.i2];
			ref.normal = m_polygonB.normals[ref.i1];

			// CCW winding
			ref.sideNormal1.Set(ref.normal.y, -ref.normal.x);
			ref.sideNormal2 = -ref.sideNormal1;
		}

		ref.sideOffset1 = b2Dot(ref.sideNormal1, ref.v1);
		ref.sideOffset2 = b2Dot(ref.sideNormal2, ref.v2);

		// Clip incident edge against reference face side planes
		b2ClipVertex clipPoints1[2];
		b2ClipVertex clipPoints2[2];
		int32 np;

		// Clip to side 1
		np = b2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

		if (np < b2_maxManifoldPoints)
		{
			return;
		}

		// Clip to side 2
		np = b2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

		if (np < b2_maxManifoldPoints)
		{
			return;
		}

		// Now clipPoints2 contains the clipped points.
		if (primaryAxis.type == b2EPAxis::e_edgeA)
		{
			manifold->localNormal = ref.normal;
			manifold->localPoint = ref.v1;
		}
		else
		{
			manifold->localNormal = polygonB->m_normals[ref.i1];
			manifold->localPoint = polygonB->m_vertices[ref.i1];
		}

		int32 pointCount = 0;
		for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
		{
			float separation;

			separation = b2Dot(ref.normal, clipPoints2[i].v - ref.v1);

			if (separation <= m_radius)
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

				++pointCount;
			}
		}

		manifold->pointCount = pointCount;
	}

	b2EPAxis ComputeEdgeSeparation()
	{
		b2EPAxis axis;
		axis.type = b2EPAxis::e_edgeA;
		axis.index = -1;
		axis.separation = -FLT_MAX;
		axis.normal.SetZero();

		b2Vec2 axes[2] = { m_normal1, -m_normal1 };

		// Find axis with least overlap (min-max problem)
		for (int32 j = 0; j < 2; ++j)
		{
			float sj = FLT_MAX;

			// Find deepest polygon vertex along axis j
			for (int32 i = 0; i < m_polygonB.count; ++i)
			{
				float si = b2Dot(axes[j], m_polygonB.vertices[i] - m_v1);
				if (si < sj)
				{
					sj = si;
				}
			}

			if (sj > axis.separation)
			{
				axis.index = j;
				axis.separation = sj;
				axis.normal = axes[j];
			}
		}

		return axis;
	}

	b2EPAxis ComputePolygonSeparation()
	{
		b2EPAxis axis;
		axis.type = b2EPAxis::e_unknown;
		axis.index = -1;
		axis.separation = -FLT_MAX;
		axis.normal.SetZero();

		for (int32 i = 0; i < m_polygonB.count; ++i)
		{
			b2Vec2 n = -m_polygonB.normals[i];

			float s1 = b2Dot(n, m_polygonB.vertices[i] - m_v1);
			float s2 = b2Dot(n, m_polygonB.vertices[i] - m_v2);
			float s = b2Min(s1, s2);

			if (s > axis.separation)
			{
				axis.type = b2EPAxis::e_edgeB;
				axis.index = i;
				axis.separation = s;
				axis.normal = n;
			}
		}

		return axis;
	}

	enum VertexType
	{
		e_isolated,
		e_concave,
		e_convex
	};
	
	b2TempPolygon m_polygonB;
	
	b2Vec2 m_v1, m_v2;
	b2Vec2 m_normal1;
	VertexType m_type1, m_type2;
	float m_radius;
};

void b2CollideEdgeAndPolygon(	b2Manifold* manifold,
							 const b2EdgeShape* edgeA, const b2Transform& xfA,
							 const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	b2EPCollider collider;
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
