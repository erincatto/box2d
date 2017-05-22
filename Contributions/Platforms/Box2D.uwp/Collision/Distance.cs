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

using System.Diagnostics;
using System;
using System.Numerics;
using Box2D.uwp.UWPExtensions;

namespace Box2D.UWP
{
    /// Used to warm start ComputeDistance.
    /// Set count to zero on first call.
    public struct SimplexCache
    {
	    public float metric;		///< length or area
        public UInt16 count;
        public FixedArray3<byte> indexA;	///< vertices on shape A
        public FixedArray3<byte> indexB;	///< vertices on shape B
    };

    /// Input for ComputeDistance.
    /// You have to option to use the shape radii
    /// in the computation. Even 
    public struct DistanceInput
    {
        public XForm transformA;
        public XForm transformB;
        public bool useRadii;
    };

    /// Output for ComputeDistance.
    public struct DistanceOutput
    {
        public Vector2 pointA;		///< closest point on shapeA
        public Vector2 pointB;		///< closest point on shapeB
        public float distance;
        public int iterations;	///< number of GJK iterations used
    };

    internal struct SimplexVertex
    {
        public Vector2 wA;		// support point in shapeA
        public Vector2 wB;		// support point in shapeB
        public Vector2 w;		// wB - wA
        public float a;		// barycentric coordinate for closest point
        public int indexA;	// wA index
        public int indexB;	// wB index
    };

    internal struct Simplex
    {
	    internal void ReadCache(	ref SimplexCache cache,
					    Shape shapeA, ref XForm transformA,
                        Shape shapeB, ref XForm transformB)
	    {
		    Debug.Assert(0 <= cache.count && cache.count <= 3);
    		
		    // Copy data from cache.
		    _count = cache.count;
		    for (int i = 0; i < _count; ++i)
		    {
			    SimplexVertex v = _v[i];
			    v.indexA = cache.indexA[i];
			    v.indexB = cache.indexB[i];
			    Vector2 wALocal = shapeA.GetVertex(v.indexA);
			    Vector2 wBLocal = shapeB.GetVertex(v.indexB);
			    v.wA = MathUtils.Multiply(ref transformA, wALocal);
			    v.wB = MathUtils.Multiply(ref transformB, wBLocal);
			    v.w = v.wB - v.wA;
			    v.a = 0.0f;
                _v[i] = v;
		    }

		    // Compute the new simplex metric, if it is substantially different than
		    // old metric then flush the simplex.
		    if (_count > 1)
		    {
			    float metric1 = cache.metric;
			    float metric2 = GetMetric();
			    if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.b2_FLT_EPSILON)
			    {
				    // Reset the simplex.
				    _count = 0;
			    }
		    }

		    // If the cache is empty or invalid ...
		    if (_count == 0)
		    {
			    SimplexVertex v = _v[0];
			    v.indexA = 0;
			    v.indexB = 0;
			    Vector2 wALocal = shapeA.GetVertex(0);
			    Vector2 wBLocal = shapeB.GetVertex(0);
			    v.wA = MathUtils.Multiply(ref transformA, wALocal);
			    v.wB = MathUtils.Multiply(ref transformB, wBLocal);
			    v.w = v.wB - v.wA;
                _v[0] = v;
			    _count = 1;
		    }
	    }

        internal void WriteCache(ref SimplexCache cache) 
	    {
		    cache.metric = GetMetric();
		    cache.count = (UInt16)_count;
		    for (int i = 0; i < _count; ++i)
		    {
                cache.indexA[i] = (byte)(_v[i].indexA);
                cache.indexB[i] = (byte)(_v[i].indexB);
		    }
	    }

        internal Vector2 GetSearchDirection() 
	    {
		    switch (_count)
		    {
		    case 1:
			    return -_v[0].w;

		    case 2:
			    {
				    Vector2 e12 = _v[1].w - _v[0].w;
				    float sgn = MathUtils.Cross(e12, -_v[0].w);
				    if (sgn > 0.0f)
				    {
					    // Origin is left of e12.
					    return MathUtils.Cross(1.0f, e12);
				    }
				    else
				    {
					    // Origin is right of e12.
					    return MathUtils.Cross(e12, 1.0f);
				    }
			    }

		    default:
			    Debug.Assert(false);
			    return Vector2.Zero;
		    }
	    }

        internal Vector2 GetClosestPoint() 
	    {
		    switch (_count)
		    {
		    case 0:
			    Debug.Assert(false);
			    return Vector2.Zero;

		    case 1:
			    return _v[0].w;

		    case 2:
			    return _v[0].a * _v[0].w + _v[1].a * _v[1].w;

		    case 3:
			    return Vector2.Zero;

		    default:
			    Debug.Assert(false);
			    return Vector2.Zero;
		    }
	    }

        internal void GetWitnessPoints(out Vector2 pA, out Vector2 pB) 
	    {
		    switch (_count)
		    {
		    case 0:
                pA = Vector2.Zero;
                pB = Vector2.Zero;
			    Debug.Assert(false);
			    break;

		    case 1:
			    pA = _v[0].wA;
			    pB = _v[0].wB;
			    break;

		    case 2:
			    pA = _v[0].a * _v[0].wA + _v[1].a * _v[1].wA;
			    pB = _v[0].a * _v[0].wB + _v[1].a * _v[1].wB;
			    break;

		    case 3:
			    pA = _v[0].a * _v[0].wA + _v[1].a * _v[1].wA + _v[2].a * _v[2].wA;
			    pB = pA;
			    break;

		    default:
                throw new Exception();
		    }
	    }

        internal float GetMetric() 
	    {
		    switch (_count)
		    {
		    case 0:
			    Debug.Assert(false);
			    return 0.0f;

		    case 1:
			    return 0.0f;

		    case 2:
			    return (_v[0].w - _v[1].w).Length();

		    case 3:
			    return MathUtils.Cross(_v[1].w - _v[0].w, _v[2].w - _v[0].w);

		    default:
			    Debug.Assert(false);
			    return 0.0f;
		    }
	    }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12

        internal void Solve2()
        {
            Vector2 w1 = _v[0].w;
            Vector2 w2 = _v[1].w;
            Vector2 e12 = w2 - w1;

            // w1 region
            float d12_2 = -Vector2.Dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                var v0 = _v[0];
                v0.a = 1.0f;
                _v[0] = v0;
                _count = 1;
                return;
            }

            // w2 region
            float d12_1 = Vector2.Dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                var v1 = _v[1];
                v1.a = 1.0f;
                _v[1] = v1;
                _count = 1;
                _v[0] = _v[1];
                return;
            }

            // Must be in e12 region.
            float inv_d12 = 1.0f / (d12_1 + d12_2);
            var v0_2 = _v[0];
            var v1_2 = _v[1];
            v0_2.a = d12_1 * inv_d12;
            v1_2.a = d12_2 * inv_d12;
            _v[0] = v0_2;
            _v[1] = v1_2;
            _count = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        internal void Solve3()
        {
            Vector2 w1 = _v[0].w;
            Vector2 w2 = _v[1].w;
            Vector2 w3 = _v[2].w;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            Vector2 e12 = w2 - w1;
            float w1e12 = Vector2.Dot(w1, e12);
            float w2e12 = Vector2.Dot(w2, e12);
            float d12_1 = w2e12;
            float d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            Vector2 e13 = w3 - w1;
            float w1e13 = Vector2.Dot(w1, e13);
            float w3e13 = Vector2.Dot(w3, e13);
            float d13_1 = w3e13;
            float d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            Vector2 e23 = w3 - w2;
            float w2e23 = Vector2.Dot(w2, e23);
            float w3e23 = Vector2.Dot(w3, e23);
            float d23_1 = w3e23;
            float d23_2 = -w2e23;

            // Triangle123
            float n123 = MathUtils.Cross(e12, e13);

            float d123_1 = n123 * MathUtils.Cross(w2, w3);
            float d123_2 = n123 * MathUtils.Cross(w3, w1);
            float d123_3 = n123 * MathUtils.Cross(w1, w2);

            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                var v0_1 = _v[0];
                v0_1.a = 1.0f;
                _v[0] = v0_1;
                _count = 1;
                return;
            }

            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                var v0_2 = _v[0];
                var v1_2 = _v[1];
                v0_2.a = d12_1 * inv_d12;
                v1_2.a = d12_1 * inv_d12;
                _v[0] = v0_2;
                _v[1] = v1_2;
                _count = 2;
                return;
            }

            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                float inv_d13 = 1.0f / (d13_1 + d13_2);
                var v0_3 = _v[0];
                var v2_3 = _v[2];
                v0_3.a = d13_1 * inv_d13;
                v2_3.a = d13_2 * inv_d13;
                _v[0] = v0_3;
                _v[2] = v2_3;
                _count = 2;
                _v[1] = _v[2];
                return;
            }

            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                var v1_4 = _v[1];
                v1_4.a = 1.0f;
                _v[1] = v1_4;
                _count = 1;
                _v[0] = _v[1];
                return;
            }

            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                var v2_5 = _v[2];
                v2_5.a = 1.0f;
                _v[2] = v2_5;
                _count = 1;
                _v[0] = _v[2];
                return;
            }

            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                float inv_d23 = 1.0f / (d23_1 + d23_2);
                var v1_6 = _v[1];
                var v2_6 = _v[2];
                v1_6.a = d23_1 * inv_d23;
                v2_6.a = d23_2 * inv_d23;
                _v[1] = v1_6;
                _v[2] = v2_6;
                _count = 2;
                _v[0] = _v[2];
                return;
            }

            // Must be in triangle123
            float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            var v0_7 = _v[0];
            var v1_7 = _v[1];
            var v2_7 = _v[2];
            v0_7.a = d123_1 * inv_d123;
            v1_7.a = d123_2 * inv_d123;
            v2_7.a = d123_3 * inv_d123;
            _v[0] = v0_7;
            _v[1] = v1_7;
            _v[2] = v2_7;
            _count = 3;
        }

	    internal FixedArray3<SimplexVertex> _v;
        internal int _count;
    };

    public static class Distance
    {
        public static void ComputeDistance(out DistanceOutput output,
				        out SimplexCache cache,
				        ref DistanceInput input,
				        Shape shapeA,
                        Shape shapeB)
        {
            cache = new SimplexCache();
	        ++b2_gjkCalls;

	        XForm transformA = input.transformA;
	        XForm transformB = input.transformB;

	        // Initialize the simplex.
	        Simplex simplex = new Simplex();
	        simplex.ReadCache(ref cache, shapeA, ref transformA, shapeB, ref transformB);

	        // Get simplex vertices as an array.
	        int k_maxIters = 20;

	        // These store the vertices of the last simplex so that we
	        // can check for duplicates and prevent cycling.
            FixedArray3<int> saveA = new FixedArray3<int>();
            FixedArray3<int> saveB = new FixedArray3<int>();
	        int saveCount = 0;

	        Vector2 closestPoint = simplex.GetClosestPoint();
	        float distanceSqr1 = closestPoint.LengthSquared();
	        float distanceSqr2 = distanceSqr1;

	        // Main iteration loop.
	        int iter = 0;
	        while (iter < k_maxIters)
	        {
		        // Copy simplex so we can identify duplicates.
		        saveCount = simplex._count;
		        for (int i = 0; i < saveCount; ++i)
		        {
			        saveA[i] = simplex._v[i].indexA;
                    saveB[i] = simplex._v[i].indexB;
		        }

		        switch (simplex._count)
		        {
		        case 1:
			        break;

		        case 2:
			        simplex.Solve2();
			        break;

		        case 3:
			        simplex.Solve3();
			        break;

		        default:
			        Debug.Assert(false);
                    break;
		        }

		        // If we have 3 points, then the origin is in the corresponding triangle.
		        if (simplex._count == 3)
		        {
			        break;
		        }

		        // Compute closest point.
		        Vector2 p = simplex.GetClosestPoint();
		        distanceSqr2 = p.LengthSquared();

		        // Ensure progress
		        if (distanceSqr2 >= distanceSqr1)
		        {
			        //break;
		        }
		        distanceSqr1 = distanceSqr2;

		        // Get search direction.
		        Vector2 d = simplex.GetSearchDirection();

		        // Ensure the search direction is numerically fit.
		        if (d.LengthSquared() < Settings.b2_FLT_EPSILON * Settings.b2_FLT_EPSILON)
		        {
			        // The origin is probably contained by a line segment
			        // or triangle. Thus the shapes are overlapped.

			        // We can't return zero here even though there may be overlap.
			        // In case the simplex is a point, segment, or triangle it is difficult
			        // to determine if the origin is contained in the CSO or very close to it.
			        break;
		        }

		        // Compute a tentative new simplex vertex using support points.
                SimplexVertex vertex = simplex._v[simplex._count];
		        vertex.indexA = shapeA.GetSupport(MathUtils.MultiplyT(ref transformA.R, -d));
		        vertex.wA = MathUtils.Multiply(ref transformA, shapeA.GetVertex(vertex.indexA));
		        //Vector2 wBLocal;
		        vertex.indexB = shapeB.GetSupport(MathUtils.MultiplyT(ref transformB.R, d));
		        vertex.wB = MathUtils.Multiply(ref transformB, shapeB.GetVertex(vertex.indexB));
		        vertex.w = vertex.wB - vertex.wA;
                simplex._v[simplex._count] = vertex;

		        // Iteration count is equated to the number of support point calls.
		        ++iter;
		        ++b2_gjkIters;

		        // Check for duplicate support points. This is the main termination criteria.
		        bool duplicate = false;
		        for (int i = 0; i < saveCount; ++i)
		        {
			        if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
			        {
				        duplicate = true;
				        break;
			        }
		        }

		        // If we found a duplicate support point we must exit to avoid cycling.
		        if (duplicate)
		        {
			        break;
		        }

		        // New vertex is ok and needed.
		        ++simplex._count;
	        }

	        b2_gjkMaxIters = Math.Max(b2_gjkMaxIters, iter);

	        // Prepare output.
	        simplex.GetWitnessPoints(out output.pointA, out output.pointB);
	        output.distance = (output.pointA - output.pointB).Length();
	        output.iterations = iter;

	        // Cache the simplex.
	        simplex.WriteCache(ref cache);

	        // Apply radii if requested.
	        if (input.useRadii)
	        {
		        float rA = shapeA._radius;
		        float rB = shapeB._radius;

		        if (output.distance > rA + rB && output.distance > Settings.b2_FLT_EPSILON)
		        {
			        // Shapes are still no overlapped.
			        // Move the witness points to the outer surface.
			        output.distance -= rA + rB;
			        Vector2 normal = output.pointB - output.pointA;
			        normal.Normalize();
			        output.pointA += rA * normal;
			        output.pointB -= rB * normal;
		        }
		        else
		        {
			        // Shapes are overlapped when radii are considered.
			        // Move the witness points to the middle.
			        Vector2 p = 0.5f * (output.pointA + output.pointB);
			        output.pointA = p;
			        output.pointB = p;
			        output.distance = 0.0f;
		        }
	        }
        }

        static int b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
    }
}
