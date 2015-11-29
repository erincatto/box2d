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
using System.Diagnostics;
using System;
using System.Numerics;

namespace Box2D.UWP
{
    /// Inpute parameters for CalculateTimeOfImpact
    public struct TOIInput
    {
        public Sweep sweepA;
        public Sweep sweepB;
        public float tolerance;
    };
    
    public enum SeparationFunctionType
    {
	    Points,
	    FaceA,
	    FaceB
    };

    public struct SeparationFunction
    {
        public SeparationFunction(ref SimplexCache cache,
		    Shape shapeA, ref XForm transformA,
            Shape shapeB, ref XForm transformB)
	    {
            _localPoint = Vector2.Zero;
		    _shapeA = shapeA;
		    _shapeB = shapeB;
		    int count = cache.count;
		    Debug.Assert(0 < count && count < 3);

		    if (count == 1)
		    {
                _type = SeparationFunctionType.Points;
			    Vector2 localPointA = _shapeA.GetVertex(cache.indexA[0]);
			    Vector2 localPointB = _shapeB.GetVertex(cache.indexB[0]);
			    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);
			    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);
			    _axis = pointB - pointA;
			    _axis.Normalize();
		    }
		    else if (cache.indexB[0] == cache.indexB[1])
		    {
			    // Two points on A and one on B
                _type = SeparationFunctionType.FaceA;
			    Vector2 localPointA1 = _shapeA.GetVertex(cache.indexA[0]);
			    Vector2 localPointA2 = _shapeA.GetVertex(cache.indexA[1]);
			    Vector2 localPointB = _shapeB.GetVertex(cache.indexB[0]);
			    _localPoint = 0.5f * (localPointA1 + localPointA2);
			    _axis = MathUtils.Cross(localPointA2 - localPointA1, 1.0f);
			    _axis.Normalize();

			    Vector2 normal = MathUtils.Multiply(ref transformA.R, _axis);
			    Vector2 pointA = MathUtils.Multiply(ref transformA, _localPoint);
			    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);

			    float s = Vector2.Dot(pointB - pointA, normal);
			    if (s < 0.0f)
			    {
				    _axis = -_axis;
			    }
		    }
		    else if (cache.indexA[0] == cache.indexA[1])
		    {
			    // Two points on B and one on A.
                _type = SeparationFunctionType.FaceB;
			    Vector2 localPointA = shapeA.GetVertex(cache.indexA[0]);
			    Vector2 localPointB1 = shapeB.GetVertex(cache.indexB[0]);
			    Vector2 localPointB2 = shapeB.GetVertex(cache.indexB[1]);
			    _localPoint = 0.5f * (localPointB1 + localPointB2);
			    _axis = MathUtils.Cross(localPointB2 - localPointB1, 1.0f);
			    _axis.Normalize();

			    Vector2 normal = MathUtils.Multiply(ref transformB.R, _axis);
			    Vector2 pointB = MathUtils.Multiply(ref transformB, _localPoint);
			    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);

			    float s = Vector2.Dot(pointA - pointB, normal);
			    if (s < 0.0f)
			    {
				    _axis = -_axis;
			    }
		    }
		    else
		    {
			    // Two points on B and two points on A.
			    // The faces are parallel.
			    Vector2 localPointA1 = _shapeA.GetVertex(cache.indexA[0]);
			    Vector2 localPointA2 = _shapeA.GetVertex(cache.indexA[1]);
			    Vector2 localPointB1 = _shapeB.GetVertex(cache.indexB[0]);
			    Vector2 localPointB2 = _shapeB.GetVertex(cache.indexB[1]);

			    Vector2 pA = MathUtils.Multiply(ref transformA, localPointA1);
			    Vector2 dA = MathUtils.Multiply(ref transformA.R, localPointA2 - localPointA1);
			    Vector2 pB = MathUtils.Multiply(ref transformB, localPointB1);
			    Vector2 dB = MathUtils.Multiply(ref transformB.R, localPointB2 - localPointB1);

			    float a = Vector2.Dot(dA, dA);
			    float e = Vector2.Dot(dB, dB);
			    Vector2 r = pA - pB;
			    float c = Vector2.Dot(dA, r);
			    float f = Vector2.Dot(dB, r);

			    float b = Vector2.Dot(dA, dB);
			    float denom = a * e - b * b;

			    float s = 0.0f;
			    if (denom != 0.0f)
			    {
				    s = MathUtils.Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			    }

			    float t = (b * s + f) / e;

			    if (t < 0.0f)
			    {
				    t = 0.0f;
				    s = MathUtils.Clamp(-c / a, 0.0f, 1.0f);
			    }
			    else if (t > 1.0f)
			    {
				    t = 1.0f;
				    s = MathUtils.Clamp((b - c) / a, 0.0f, 1.0f);
			    }

			    Vector2 localPointA = localPointA1 + s * (localPointA2 - localPointA1);
			    Vector2 localPointB = localPointB1 + t * (localPointB2 - localPointB1);

			    if (s == 0.0f || s == 1.0f)
			    {
                    _type = SeparationFunctionType.FaceB;
				    _axis = MathUtils.Cross(localPointB2 - localPointB1, 1.0f);
				    _axis.Normalize();

				    _localPoint = localPointB;

				    Vector2 normal = MathUtils.Multiply(ref transformB.R, _axis);
				    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);
				    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);

				    float sgn = Vector2.Dot(pointA - pointB, normal);
				    if (sgn < 0.0f)
				    {
					    _axis = -_axis;
				    }
			    }
			    else
			    {
                    _type = SeparationFunctionType.FaceA;
				    _axis = MathUtils.Cross(localPointA2 - localPointA1, 1.0f);
				    _axis.Normalize();

				    _localPoint = localPointA;

				    Vector2 normal = MathUtils.Multiply(ref transformA.R, _axis);
				    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);
				    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);

				    float sgn = Vector2.Dot(pointB - pointA, normal);
				    if (sgn < 0.0f)
				    {
					    _axis = -_axis;
				    }
			    }
		    }
	    }

	    public float Evaluate(ref XForm transformA, ref XForm transformB)
	    {
		    switch (_type)
		    {
                case SeparationFunctionType.Points:
			    {
				    Vector2 axisA = MathUtils.MultiplyT(ref transformA.R,  _axis);
				    Vector2 axisB = MathUtils.MultiplyT(ref transformB.R, -_axis);
				    Vector2 localPointA = _shapeA.GetSupportVertex(axisA);
				    Vector2 localPointB = _shapeB.GetSupportVertex(axisB);
				    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);
				    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);
				    float separation = Vector2.Dot(pointB - pointA, _axis);
				    return separation;
			    }

                case SeparationFunctionType.FaceA:
			    {
				    Vector2 normal = MathUtils.Multiply(ref transformA.R, _axis);
				    Vector2 pointA = MathUtils.Multiply(ref transformA, _localPoint);

				    Vector2 axisB = MathUtils.MultiplyT(ref transformB.R, -normal);

				    Vector2 localPointB = _shapeB.GetSupportVertex(axisB);
				    Vector2 pointB = MathUtils.Multiply(ref transformB, localPointB);

				    float separation = Vector2.Dot(pointB - pointA, normal);
				    return separation;
			    }

                case SeparationFunctionType.FaceB:
			    {
				    Vector2 normal = MathUtils.Multiply(ref transformB.R, _axis);
				    Vector2 pointB = MathUtils.Multiply(ref transformB, _localPoint);

				    Vector2 axisA = MathUtils.MultiplyT(ref transformA.R, -normal);

				    Vector2 localPointA = _shapeA.GetSupportVertex(axisA);
				    Vector2 pointA = MathUtils.Multiply(ref transformA, localPointA);

				    float separation = Vector2.Dot(pointA - pointB, normal);
				    return separation;
			    }

		    default:
			    Debug.Assert(false);
			    return 0.0f;
		    }
	    }

        Shape _shapeA;
        Shape _shapeB;
        SeparationFunctionType _type;
	    Vector2 _localPoint;
	    Vector2 _axis;
    };



    public static class TimeOfImpact
    {
        /// Compute the time when two shapes begin to touch or touch at a closer distance.
        /// TOI considers the shape radii. It attempts to have the radii overlap by the tolerance.
        /// Iterations terminate with the overlap is within 0.5 * tolerance. The tolerance should be
        /// smaller than sum of the shape radii.
        /// @warning the sweeps must have the same time interval.
        /// @return the fraction between [0,1] in which the shapes first touch.
        /// fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
        public static float CalculateTimeOfImpact(ref TOIInput input, Shape shapeA, Shape shapeB)
        {
	        ++b2_toiCalls;

	        Sweep sweepA = input.sweepA;
	        Sweep sweepB = input.sweepB;

	        Debug.Assert(sweepA.t0 == sweepB.t0);
	        Debug.Assert(1.0f - sweepA.t0 > Settings.b2_FLT_EPSILON);

	        float radius = shapeA._radius + shapeB._radius;
	        float tolerance = input.tolerance;

	        float alpha = 0.0f;

	        int k_maxIterations = 1000;	// TODO_ERIN b2Settings
	        int iter = 0;
	        float target = 0.0f;

	        // Prepare input for distance query.
            SimplexCache cache;
	        DistanceInput distanceInput;
	        distanceInput.useRadii = false;

	        for(;;)
	        {
		        XForm xfA, xfB;
		        sweepA.GetTransform(out xfA, alpha);
                sweepB.GetTransform(out xfB, alpha);

		        // Get the distance between shapes.
		        distanceInput.transformA = xfA;
		        distanceInput.transformB = xfB;
		        DistanceOutput distanceOutput;
		        Distance.ComputeDistance(out distanceOutput, out cache, ref distanceInput, shapeA, shapeB);

		        if (distanceOutput.distance <= 0.0f)
		        {
			        alpha = 1.0f;
			        break;
		        }

                SeparationFunction fcn = new SeparationFunction(ref cache, shapeA, ref xfA, shapeB, ref xfB);

		        float separation = fcn.Evaluate(ref xfA, ref xfB);
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
				        target = Math.Max(radius - tolerance, 0.75f * radius);
			        }
			        else
			        {
				        target = Math.Max(separation - tolerance, 0.02f * radius);
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

        #if false
		        // Dump the curve seen by the root finder
		        {
			        int N = 100;
			        float dx = 1.0f / N;
			        float xs[N+1];
			        float fs[N+1];

			        float x = 0.0f;

			        for (int i = 0; i <= N; ++i)
			        {
				        sweepA.GetTransform(&xfA, x);
				        sweepB.GetTransform(&xfB, x);
				        float f = fcn.Evaluate(xfA, xfB) - target;

				        printf("%g %g\n", x, f);

				        xs[i] = x;
				        fs[i] = f;

				        x += dx;
			        }
		        }
        #endif

		        // Compute 1D root of: f(x) - target = 0
		        float newAlpha = alpha;
		        {
			        float x1 = alpha, x2 = 1.0f;

			        float f1 = separation;

			        sweepA.GetTransform(out xfA, x2);
                    sweepB.GetTransform(out xfB, x2);
			        float f2 = fcn.Evaluate(ref xfA, ref xfB);

			        // If intervals don't overlap at t2, then we are done.
			        if (f2 >= target)
			        {
				        alpha = 1.0f;
				        break;
			        }

			        // Determine when intervals intersect.
			        int rootIterCount = 0;
			        for (;;)
			        {
				        // Use a mix of the secant rule and bisection.
				        float x;
				        if ((rootIterCount & 1) != 0)
				        {
					        // Secant rule to improve convergence.
					        x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
				        }
				        else
				        {
					        // Bisection to guarantee progress.
					        x = 0.5f * (x1 + x2);
				        }

				        sweepA.GetTransform(out xfA, x);
                        sweepB.GetTransform(out xfB, x);

				        float f = fcn.Evaluate(ref xfA, ref xfB);

				        if (Math.Abs(f - target) < 0.025f * tolerance)
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

			        b2_toiMaxRootIters = Math.Max(b2_toiMaxRootIters, rootIterCount);
		        }

		        // Ensure significant advancement.
		        if (newAlpha < (1.0f + 100.0f * Settings.b2_FLT_EPSILON) * alpha)
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

	        b2_toiMaxIters = Math.Max(b2_toiMaxIters, iter);

	        return alpha;
        }

        static int b2_toiCalls, b2_toiIters, b2_toiMaxIters;
        static int b2_toiRootIters, b2_toiMaxRootIters;
    }
}
