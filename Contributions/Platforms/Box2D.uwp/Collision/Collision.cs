/* Box2D.uwp port of Box2D.Xna: Copyright (c) 2015 Nukepayload2
* Box2D.Xna port of Box2D:
* Copyright (c) 2009 Brandon Furtwangler, Nathan Furtwangler
*
* Original source Box2D:
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com 
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
using Box2D.uwp.UWPExtensions;
using System;
using System.Diagnostics;
using System.Numerics;

namespace Box2D.UWP
{
	public struct Features
    {
        public byte ReferenceEdge;	    ///< The edge that defines the outward contact normal.
        public byte IncidentEdge;		///< The edge most anti-parallel to the reference edge.
        public byte IncidentVertex;	///< The vertex (0 or 1) on the incident edge that was clipped.
        public byte Flip;				///< A value of 1 indicates that the reference edge is on shape2.
    };

    /// Contact ids to facilitate warm starting.
    [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
    public struct ContactID
    {
        [System.Runtime.InteropServices.FieldOffset(0)]
        public Features Features;

	    /// The features that intersect to form the contact point
        [System.Runtime.InteropServices.FieldOffset(0)]
        public uint Key;					///< Used to quickly compare contact ids.
    };

    /// A manifold point is a contact point belonging to a contact
    /// manifold. It holds details related to the geometry and dynamics
    /// of the contact points.
    /// The local point usage depends on the manifold type:
    /// -ShapeType.Circles: the local center of circleB
    /// -SeparationFunction.FaceA: the local center of cirlceB or the clip point of polygonB
    /// -SeparationFunction.FaceB: the clip point of polygonA
    /// This structure is stored across time steps, so we keep it small.
    /// Note: the impulses are used for internal caching and may not
    /// provide reliable contact forces, especially for high speed collisions.
    public struct ManifoldPoint
    {
        public Vector2 LocalPoint;		///< usage depends on manifold type
        public float NormalImpulse;	///< the non-penetration impulse
        public float TangentImpulse;	///< the friction impulse
        public ContactID Id;			///< uniquely identifies a contact point between two shapes
    };

    public enum ManifoldType
    {
	    Circles,
	    FaceA,
	    FaceB
    };


    /// A manifold for two touching convex shapes.
    /// Box2D supports multiple types of contact:
    /// - clip point versus plane with radius
    /// - point versus point with radius (circles)
    /// The local point usage depends on the manifold type:
    /// -ShapeType.Circles: the local center of circleA
    /// -SeparationFunction.FaceA: the center of faceA
    /// -SeparationFunction.FaceB: the center of faceB
    /// Similarly the local normal usage:
    /// -ShapeType.Circles: not used
    /// -SeparationFunction.FaceA: the normal on polygonA
    /// -SeparationFunction.FaceB: the normal on polygonB
    /// We store contacts in this way so that position correction can
    /// account for movement, which is critical for continuous physics.
    /// All contact scenarios must be expressed in one of these types.
    /// This structure is stored across time steps, so we keep it small.
    public struct Manifold
    {
	    public FixedArray2<ManifoldPoint> _points;	        ///< the points of contact
	    public Vector2 _localPlaneNormal;						///< not use for Type.SeparationFunction.Points
	    public Vector2 _localPoint;							///< usage depends on manifold type
	    public ManifoldType _type;
	    public int _pointCount;								///< the number of manifold points
    };

    /// This is used to compute the current state of a contact manifold.
    public struct WorldManifold
    {
	    /// Evaluate the manifold with supplied transforms. This assumes
	    /// modest motion from the original state. This does not change the
	    /// point count, impulses, etc. The radii must come from the shapes
	    /// that generated the manifold.
        public WorldManifold(ref Manifold manifold,
					    ref XForm xfA, float radiusA,
					    ref XForm xfB, float radiusB)
        {
            _normal = Vector2.Zero;
            _points = new FixedArray2<Vector2>();

	        if (manifold._pointCount == 0)
	        {
		        return;
	        }

	        switch (manifold._type)
	        {
	        case ManifoldType.Circles:
		        {
			        Vector2 pointA = MathUtils.Multiply(ref xfA, manifold._localPoint);
                    Vector2 pointB = MathUtils.Multiply(ref xfB, manifold._points[0].LocalPoint);
                    Vector2 normal = new Vector2(1.0f, 0.0f);
			        if (Vector2.DistanceSquared(pointA, pointB) > Settings.b2_FLT_EPSILON * Settings.b2_FLT_EPSILON)
			        {
				        normal = pointB - pointA;
				        normal.Normalize();
			        }

			        _normal = normal;

			        Vector2 cA = pointA + radiusA * normal;
			        Vector2 cB = pointB - radiusB * normal;
			        _points[0] = 0.5f * (cA + cB);
		        }
		        break;

	        case ManifoldType.FaceA:
		        {
			        Vector2 normal = MathUtils.Multiply(ref xfA.R, manifold._localPlaneNormal);
			        Vector2 planePoint = MathUtils.Multiply(ref xfA, manifold._localPoint);

			        // Ensure normal points from A to B.
			        _normal = normal;
        			
			        for (int i = 0; i < manifold._pointCount; ++i)
			        {
				        Vector2 clipPoint = MathUtils.Multiply(ref xfB, manifold._points[i].LocalPoint);
				        Vector2 cA = clipPoint + (radiusA - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
				        Vector2 cB = clipPoint - radiusB * normal;
				        _points[i] = 0.5f * (cA + cB);
			        }
		        }
		        break;

	        case ManifoldType.FaceB:
		        {
			        Vector2 normal = MathUtils.Multiply(ref xfB.R, manifold._localPlaneNormal);
			        Vector2 planePoint = MathUtils.Multiply(ref xfB, manifold._localPoint);

			        // Ensure normal points from A to B.
			        _normal = -normal;

			        for (int i = 0; i < manifold._pointCount; ++i)
			        {
                        Vector2 clipPoint = MathUtils.Multiply(ref xfA, manifold._points[i].LocalPoint);
				        Vector2 cA = clipPoint - radiusA * normal;
				        Vector2 cB = clipPoint + (radiusB - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
				        _points[i] = 0.5f * (cA + cB);
			        }
		        }
		        break;
	        }
        }

	    public Vector2 _normal;						///< world vector pointing from A to B
	    public FixedArray2<Vector2> _points;   	///< world contact point (point of intersection)
    };

    /// This is used for determining the state of contact points.
    public enum PointState
    {
	    Null,		///< point does not exist
	    Add,		///< point was added in the update
	    Persist,	///< point persisted across the update
	    Remove,		///< point was removed in the update
    };

    /// Used for computing contact manifolds.
    public struct ClipVertex
    {
	    public Vector2 v;
	    public ContactID id;
    };

    /// Ray-cast input data.
    public struct RayCastInput
    {
	    public Vector2 p1, p2;
	    public float maxFraction;
    };

    /// Ray-cast output data.
    public struct RayCastOutput
    {
	    public Vector2 normal;
        public float fraction;
        public bool hit;
    };

    /// A line segment.
    public struct Segment
    {
	    /// Ray cast against this segment with another segment.
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
	    public bool TestSegment(out float lambda, out Vector2 normal, ref Segment segment, float maxLambda)
        {
            lambda = 0;
            normal = Vector2.Zero;

	        Vector2 s = segment.p1;
	        Vector2 r = segment.p2 - s;
	        Vector2 d = p2 - p1;
	        Vector2 n = MathUtils.Cross(d, 1.0f);

	        float k_slop = 100.0f * Settings.b2_FLT_EPSILON;
	        float denom = -Vector2.Dot(r, n);

	        // Cull back facing collision and ignore parallel segments.
	        if (denom > k_slop)
	        {
		        // Does the segment intersect the infinite line associated with this segment?
		        Vector2 b = s - p1;
		        float a = Vector2.Dot(b, n);

		        if (0.0f <= a && a <= maxLambda * denom)
		        {
			        float mu2 = -r.X * b.Y + r.Y * b.X;

			        // Does the segment intersect this segment?
			        if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
			        {
				        a /= denom;
				        n.Normalize();
				        lambda = a;
				        normal = n;
				        return true;
			        }
		        }
	        }

	        return false;
        }

	    public Vector2 p1;	///< the starting point
	    public Vector2 p2;	///< the ending point
    };

    /// An axis aligned bounding box.
    public struct AABB
    {
	    /// Verify that the bounds are sorted.
	    public bool IsValid()
        {
	        Vector2 d = upperBound - lowerBound;
	        bool valid = d.X >= 0.0f && d.Y >= 0.0f;
	        valid = valid && lowerBound.IsValid() && upperBound.IsValid();
	        return valid;
        }

	    /// Get the center of the AABB.
	    public Vector2 GetCenter()
	    {
		    return 0.5f * (lowerBound + upperBound);
	    }

	    /// Get the extents of the AABB (half-widths).
	    public Vector2 GetExtents()
	    {
		    return 0.5f * (upperBound - lowerBound);
	    }

	    /// Combine two AABBs into this one.
	    public void Combine(ref AABB aabb1, ref AABB aabb2)
	    {
		    lowerBound = Vector2.Min(aabb1.lowerBound, aabb2.lowerBound);
		    upperBound = Vector2.Max(aabb1.upperBound, aabb2.upperBound);
	    }

	    /// Does this aabb contain the provided AABB.
	    public bool Contains(ref AABB aabb)
	    {
		    bool result = true;
		    result = result && lowerBound.X <= aabb.lowerBound.X;
		    result = result && lowerBound.Y <= aabb.lowerBound.Y;
		    result = result && aabb.upperBound.X <= upperBound.X;
		    result = result && aabb.upperBound.Y <= upperBound.Y;
		    return result;
	    }

        public static bool TestOverlap(ref AABB a, ref AABB b)
        {
            Vector2 d1, d2;
            d1 = b.lowerBound - a.upperBound;
            d2 = a.lowerBound - b.upperBound;

            if (d1.X > 0.0f || d1.Y > 0.0f)
	            return false;

            if (d2.X > 0.0f || d2.Y > 0.0f)
	            return false;

            return true;
        }

        // From Real-time Collision Detection, p179.
	    public void RayCast(out RayCastOutput output, ref RayCastInput input)
        {
            output = new RayCastOutput();

            float tmin = -Settings.b2_FLT_MAX;
            float tmax = Settings.b2_FLT_MAX;

	        output.hit = false;

	        Vector2 p = input.p1;
	        Vector2 d = input.p2 - input.p1;
	        Vector2 absD = MathUtils.Abs(d);

            Vector2 normal = Vector2.Zero;

	        for (int i = 0; i < 2; ++i)
	        {
                float absD_i = i == 0 ? absD.X : absD.Y;
                float lowerBound_i = i == 0 ? lowerBound.X : lowerBound.Y;
                float upperBound_i = i == 0 ? upperBound.X : upperBound.Y;
                float p_i = i == 0 ? p.X : p.Y;

                if (absD_i < Settings.b2_FLT_EPSILON)
		        {
			        // Parallel.
                    if (p_i < lowerBound_i || upperBound_i < p_i)
			        {
				        return;
			        }
		        }
		        else
		        {
                    float d_i = i == 0 ? d.X : d.Y;

                    float inv_d = 1.0f / d_i;
                    float t1 = (lowerBound_i - p_i) * inv_d;
                    float t2 = (upperBound_i - p_i) * inv_d;

			        // Sign of the normal vector.
			        float s = -1.0f;

			        if (t1 > t2)
			        {
				        MathUtils.Swap<float>(ref t1, ref t2);
				        s = 1.0f;
			        }

			        // Push the min up
			        if (t1 > tmin)
			        {
                        if (i == 0)
                        {
                            normal.X = s;
                        }
                        else
                        {
                            normal.Y = s;
                        }

				        tmin = t1;
			        }

			        // Pull the max down
			        tmax = Math.Min(tmax, t2);

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
	        output.fraction = tmin;
	        output.normal = normal;
	        output.hit = true;
        }

	    public Vector2 lowerBound;	///< the lower vertex
	    public Vector2 upperBound;	///< the upper vertex
    };

    public static class Collision
    {
        public static void GetPointStates(out FixedArray2<PointState> state1, out FixedArray2<PointState> state2,
					          ref Manifold manifold1, ref Manifold manifold2)
        {
            state1 = new FixedArray2<PointState>();
            state2 = new FixedArray2<PointState>();

	        // Detect persists and removes.
	        for (int i = 0; i < manifold1._pointCount; ++i)
	        {
		        ContactID id = manifold1._points[i].Id;

                state1[i] = PointState.Remove;

		        for (int j = 0; j < manifold2._pointCount; ++j)
		        {
			        if (manifold2._points[j].Id.Key == id.Key)
			        {
                        state1[i] = PointState.Persist;
				        break;
			        }
		        }
	        }

	        // Detect persists and adds.
	        for (int i = 0; i < manifold2._pointCount; ++i)
	        {
		        ContactID id = manifold2._points[i].Id;

                state2[i] = PointState.Add;

		        for (int j = 0; j < manifold1._pointCount; ++j)
		        {
			        if (manifold1._points[j].Id.Key == id.Key)
			        {
                        state2[i] = PointState.Persist;
				        break;
			        }
		        }
	        }
        }


        /// Compute the collision manifold between two circles.
        public static void CollideCircles(ref Manifold manifold,
					          CircleShape circle1, ref XForm xf1,
					          CircleShape circle2, ref XForm xf2)
        {
	        manifold._pointCount = 0;

	        Vector2 p1 = MathUtils.Multiply(ref xf1, circle1._p);
	        Vector2 p2 = MathUtils.Multiply(ref xf2, circle2._p);

	        Vector2 d = p2 - p1;
	        float distSqr = Vector2.Dot(d, d);
	        float radius = circle1._radius + circle2._radius;
	        if (distSqr > radius * radius)
	        {
		        return;
	        }

	        manifold._type = ManifoldType.Circles;
	        manifold._localPoint = circle1._p;
	        manifold._localPlaneNormal = Vector2.Zero;
	        manifold._pointCount = 1;

            var p0 = manifold._points[0];

            p0.LocalPoint = circle2._p;
            p0.Id.Key = 0;

            manifold._points[0] = p0;
        }

        /// Compute the collision manifold between a polygon and a circle.
        public static void CollidePolygonAndCircle(ref Manifold manifold,
							           PolygonShape polygon, ref XForm xf1,
							           CircleShape circle, ref XForm xf2)
        {
	        manifold._pointCount = 0;

	        // Compute circle position in the frame of the polygon.
	        Vector2 c = MathUtils.Multiply(ref xf2, circle._p);
	        Vector2 cLocal = MathUtils.MultiplyT(ref xf1, c);

	        // Find the min separating edge.
	        int normalIndex = 0;
	        float separation = -Settings.b2_FLT_MAX;
	        float radius = polygon._radius + circle._radius;
	        int vertexCount = polygon._vertexCount;

	        for (int i = 0; i < vertexCount; ++i)
	        {
                float s = Vector2.Dot(polygon._normals[i], cLocal - polygon._vertices[i]);

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
	        int vertIndex1 = normalIndex;
	        int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vector2 v1 = polygon._vertices[vertIndex1];
            Vector2 v2 = polygon._vertices[vertIndex2];

	        // If the center is inside the polygon ...
	        if (separation < Settings.b2_FLT_EPSILON)
	        {
		        manifold._pointCount = 1;
		        manifold._type = ManifoldType.FaceA;
                manifold._localPlaneNormal = polygon._normals[normalIndex];
		        manifold._localPoint = 0.5f * (v1 + v2);

                var p0 = manifold._points[0];

                p0.LocalPoint = circle._p;
                p0.Id.Key = 0;

                manifold._points[0] = p0;

		        return;
	        }

	        // Compute barycentric coordinates
	        float u1 = Vector2.Dot(cLocal - v1, v2 - v1);
	        float u2 = Vector2.Dot(cLocal - v2, v1 - v2);
	        if (u1 <= 0.0f)
	        {
		        if (Vector2.DistanceSquared(cLocal, v1) > radius * radius)
		        {
			        return;
		        }

		        manifold._pointCount = 1;
		        manifold._type = ManifoldType.FaceA;
		        manifold._localPlaneNormal = cLocal - v1;
		        manifold._localPlaneNormal.Normalize();
		        manifold._localPoint = v1;

                var p0b = manifold._points[0];

                p0b.LocalPoint = circle._p;
                p0b.Id.Key = 0;

                manifold._points[0] = p0b;

	        }
	        else if (u2 <= 0.0f)
	        {
		        if (Vector2.DistanceSquared(cLocal, v2) > radius * radius)
		        {
			        return;
		        }

		        manifold._pointCount = 1;
		        manifold._type = ManifoldType.FaceA;
		        manifold._localPlaneNormal = cLocal - v2;
		        manifold._localPlaneNormal.Normalize();
		        manifold._localPoint = v2;

                var p0c = manifold._points[0];

                p0c.LocalPoint = circle._p;
                p0c.Id.Key = 0;

                manifold._points[0] = p0c;
	        }
	        else
	        {
		        Vector2 faceCenter = 0.5f * (v1 + v2);
                float separation2 = Vector2.Dot(cLocal - faceCenter, polygon._normals[vertIndex1]);
		        if (separation2 > radius)
		        {
			        return;
		        }

		        manifold._pointCount = 1;
		        manifold._type = ManifoldType.FaceA;
                manifold._localPlaneNormal = polygon._normals[vertIndex1];
		        manifold._localPoint = faceCenter;

                var p0d = manifold._points[0];

                p0d.LocalPoint = circle._p;
                p0d.Id.Key = 0;

                manifold._points[0] = p0d;
	        }
        }

        /// Compute the collision manifold between two polygons.
        public static void CollidePolygons(ref Manifold manifold,
                               PolygonShape polyA, ref XForm xfA,
                               PolygonShape polyB, ref XForm xfB)
        {
	        manifold._pointCount = 0;
	        float totalRadius = polyA._radius + polyB._radius;

	        int edgeA = 0;
	        float separationA = FindMaxSeparation(out edgeA, polyA, ref xfA, polyB, ref xfB);
	        if (separationA > totalRadius)
		        return;

	        int edgeB = 0;
	        float separationB = FindMaxSeparation(out edgeB, polyB, ref xfB, polyA, ref xfA);
	        if (separationB > totalRadius)
		        return;

	        PolygonShape poly1;	// reference polygon
	        PolygonShape poly2;	// incident polygon
	        XForm xf1, xf2;
	        int edge1;		// reference edge
	        byte flip;
	        float k_relativeTol = 0.98f;
	        float k_absoluteTol = 0.001f;

	        if (separationB > k_relativeTol * separationA + k_absoluteTol)
	        {
		        poly1 = polyB;
		        poly2 = polyA;
		        xf1 = xfB;
		        xf2 = xfA;
		        edge1 = edgeB;
		        manifold._type = ManifoldType.FaceB;
		        flip = 1;
	        }
	        else
	        {
		        poly1 = polyA;
		        poly2 = polyB;
		        xf1 = xfA;
		        xf2 = xfB;
		        edge1 = edgeA;
		        manifold._type = ManifoldType.FaceA;
		        flip = 0;
	        }

	        FixedArray2<ClipVertex> incidentEdge;
	        FindIncidentEdge(out incidentEdge, poly1, ref xf1, edge1, poly2, ref xf2);

	        int count1 = poly1._vertexCount;

	        Vector2 v11 = poly1._vertices[edge1];
	        Vector2 v12 = edge1 + 1 < count1 ? poly1._vertices[edge1+1] : poly1._vertices[0];

	        Vector2 dv = v12 - v11;

	        Vector2 localNormal = MathUtils.Cross(dv, 1.0f);
	        localNormal.Normalize();
	        Vector2 planePoint = 0.5f * (v11 + v12);

	        Vector2 sideNormal = MathUtils.Multiply(ref xf1.R, v12 - v11);
	        sideNormal.Normalize();
	        Vector2 frontNormal = MathUtils.Cross(sideNormal, 1.0f);
        	
	        v11 = MathUtils.Multiply(ref xf1, v11);
	        v12 = MathUtils.Multiply(ref xf1, v12);

	        float frontOffset = Vector2.Dot(frontNormal, v11);
	        float sideOffset1 = -Vector2.Dot(sideNormal, v11);
	        float sideOffset2 = Vector2.Dot(sideNormal, v12);

	        // Clip incident edge against extruded edge1 side edges.
	        FixedArray2<ClipVertex> clipPoints1;
	        FixedArray2<ClipVertex> clipPoints2;
	        int np;

	        // Clip to box side 1
	        np = ClipSegmentToLine(out clipPoints1, ref incidentEdge, -sideNormal, sideOffset1);

	        if (np < 2)
		        return;

	        // Clip to negative box side 1
            np = ClipSegmentToLine(out clipPoints2, ref clipPoints1, sideNormal, sideOffset2);

	        if (np < 2)
	        {
		        return;
	        }

	        // Now clipPoints2 contains the clipped points.
	        manifold._localPlaneNormal = localNormal;
	        manifold._localPoint = planePoint;

	        int pointCount = 0;
	        for (int i = 0; i < Settings.b2_maxManifoldPoints; ++i)
	        {
		        float separation = Vector2.Dot(frontNormal, clipPoints2[i].v) - frontOffset;

		        if (separation <= totalRadius)
		        {
			        ManifoldPoint cp = manifold._points[pointCount];
			        cp.LocalPoint = MathUtils.MultiplyT(ref xf2, clipPoints2[i].v);
			        cp.Id = clipPoints2[i].id;
			        cp.Id.Features.Flip = flip;
                    manifold._points[pointCount] = cp;

			        ++pointCount;
		        }
	        }

	        manifold._pointCount = pointCount;
        }

        /// Clipping for contact manifolds.
        public static int ClipSegmentToLine(out FixedArray2<ClipVertex> vOut, ref FixedArray2<ClipVertex> vIn,
							        Vector2 normal, float offset)
        {
            vOut = new FixedArray2<ClipVertex>();

	        // Start with no output points
	        int numOut = 0;

	        // Calculate the distance of end points to the line
	        float distance0 = Vector2.Dot(normal, vIn[0].v) - offset;
	        float distance1 = Vector2.Dot(normal, vIn[1].v) - offset;

	        // If the points are behind the plane
	        if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	        if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	        // If the points are on different sides of the plane
	        if (distance0 * distance1 < 0.0f)
	        {
		        // Find intersection point of edge and plane
		        float interp = distance0 / (distance0 - distance1);

                var cv = vOut[numOut];

                cv.v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		        if (distance0 > 0.0f)
		        {
                    cv.id = vIn[0].id;
		        }
		        else
		        {
                    cv.id = vIn[1].id;
		        }

                vOut[numOut] = cv;

		        ++numOut;
	        }

	        return numOut;
        }

        // Find the separation between poly1 and poly2 for a give edge normal on poly1.
        static float EdgeSeparation(PolygonShape poly1, ref XForm xf1, int edge1,
							        PolygonShape poly2, ref XForm xf2)
        {
	        int count1 = poly1._vertexCount;
	        int count2 = poly2._vertexCount;

	        Debug.Assert(0 <= edge1 && edge1 < count1);

	        // Convert normal from poly1's frame into poly2's frame.
	        Vector2 normal1World = MathUtils.Multiply(ref xf1.R, poly1._normals[edge1]);
            Vector2 normal1 = MathUtils.MultiplyT(ref xf2.R, normal1World);

	        // Find support vertex on poly2 for -normal.
	        int index = 0;
	        float minDot = Settings.b2_FLT_MAX;

	        for (int i = 0; i < count2; ++i)
	        {
		        float dot = Vector2.Dot(poly2._vertices[i], normal1);
		        if (dot < minDot)
		        {
			        minDot = dot;
			        index = i;
		        }
	        }

	        Vector2 v1 = MathUtils.Multiply(ref xf1, poly1._vertices[edge1]);
	        Vector2 v2 = MathUtils.Multiply(ref xf2, poly2._vertices[index]);
	        float separation = Vector2.Dot(v2 - v1, normal1World);
	        return separation;
        }

         // Find the max separation between poly1 and poly2 using edge normals from poly1.
        static float FindMaxSeparation( out int edgeIndex,
								        PolygonShape poly1, ref XForm xf1,
								        PolygonShape poly2, ref XForm xf2)
        {
            edgeIndex = -1;
	        int count1 = poly1._vertexCount;

	        // Vector pointing from the centroid of poly1 to the centroid of poly2.
	        Vector2 d = MathUtils.Multiply(ref xf2, poly2._centroid) - MathUtils.Multiply(ref xf1, poly1._centroid);
	        Vector2 dLocal1 = MathUtils.MultiplyT(ref xf1.R, d);

	        // Find edge normal on poly1 that has the largest projection onto d.
	        int edge = 0;
	        float maxDot = -Settings.b2_FLT_MAX;
	        for (int i = 0; i < count1; ++i)
	        {
		        float dot = Vector2.Dot(poly1._normals[i], dLocal1);
		        if (dot > maxDot)
		        {
			        maxDot = dot;
			        edge = i;
		        }
	        }

	        // Get the separation for the edge normal.
	        float s = EdgeSeparation(poly1, ref xf1, edge, poly2, ref xf2);

	        // Check the separation for the previous edge normal.
	        int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
	        float sPrev = EdgeSeparation(poly1, ref xf1, prevEdge, poly2, ref xf2);

	        // Check the separation for the next edge normal.
	        int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
	        float sNext = EdgeSeparation(poly1, ref xf1, nextEdge, poly2, ref xf2);

	        // Find the best edge and the search direction.
	        int bestEdge;
	        float bestSeparation;
	        int increment;
	        if (sPrev > s && sPrev > sNext)
	        {
		        increment = -1;
		        bestEdge = prevEdge;
		        bestSeparation = sPrev;
	        }
	        else if (sNext > s)
	        {
		        increment = 1;
		        bestEdge = nextEdge;
		        bestSeparation = sNext;
	        }
	        else
	        {
		        edgeIndex = edge;
		        return s;
	        }

	        // Perform a local search for the best edge normal.
	        for ( ; ; )
	        {
		        if (increment == -1)
			        edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
		        else
			        edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

		        s = EdgeSeparation(poly1, ref xf1, edge, poly2, ref xf2);

		        if (s > bestSeparation)
		        {
			        bestEdge = edge;
			        bestSeparation = s;
		        }
		        else
		        {
			        break;
		        }
	        }

	        edgeIndex = bestEdge;
	        return bestSeparation;
        }

        static void FindIncidentEdge(out FixedArray2<ClipVertex> c,
							         PolygonShape poly1, ref XForm xf1, int edge1,
							         PolygonShape poly2, ref XForm xf2)
        {
            c = new FixedArray2<ClipVertex>();

	        int count1 = poly1._vertexCount;
	        int count2 = poly2._vertexCount;

	        Debug.Assert(0 <= edge1 && edge1 < count1);

	        // Get the normal of the reference edge in poly2's frame.
	        Vector2 normal1 = MathUtils.MultiplyT(ref xf2.R, MathUtils.Multiply(ref xf1.R, poly1._normals[edge1]));

	        // Find the incident edge on poly2.
	        int index = 0;
	        float minDot = Settings.b2_FLT_MAX;
	        for (int i = 0; i < count2; ++i)
	        {
		        float dot = Vector2.Dot(normal1, poly2._normals[i]);
		        if (dot < minDot)
		        {
			        minDot = dot;
			        index = i;
		        }
	        }

	        // Build the clip vertices for the incident edge.
	        int i1 = index;
	        int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

            var cv0 = c[0];

            cv0.v = MathUtils.Multiply(ref xf2, poly2._vertices[i1]);
            cv0.id.Features.ReferenceEdge = (byte)edge1;
            cv0.id.Features.IncidentEdge = (byte)i1;
            cv0.id.Features.IncidentVertex = 0;

            c[0] = cv0;

            var cv1 = c[1];
            cv1.v = MathUtils.Multiply(ref xf2, poly2._vertices[i2]);
            cv1.id.Features.ReferenceEdge = (byte)edge1;
            cv1.id.Features.IncidentEdge = (byte)i2;
            cv1.id.Features.IncidentVertex = 1;

            c[1] = cv1;
        }
    }
}

