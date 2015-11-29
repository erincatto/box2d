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
    public class CircleShape : Shape
    {
        public CircleShape()
        {
	        ShapeType = ShapeType.Circle;
	        _radius = 0.0f;
	        _p = Vector2.Zero;
        }

        /// Implement Shape.
        public override Shape Clone()
        {
            CircleShape shape = new CircleShape();
            shape.ShapeType = ShapeType;
            shape._radius = _radius;
            shape._p = _p;

            return shape;
        }

        /// @see Shape.TestPoint
        public override bool TestPoint(ref XForm transform, Vector2 p)
        {
            Vector2 center = transform.Position + MathUtils.Multiply(ref transform.R, _p);
	        Vector2 d = p - center;
	        return Vector2.Dot(d, d) <= _radius * _radius;
        }

        /// @see Shape.TestSegment
        public override SegmentCollide TestSegment(	ref XForm transform,
					        out float lambda,
                            out Vector2 normal,
					        ref Segment segment,
					        float maxLambda)
        {
            lambda = 0.0f;
            normal = Vector2.Zero;

	        Vector2 position = transform.Position + MathUtils.Multiply(ref transform.R, _p);
	        Vector2 s = segment.p1 - position;
	        float b = Vector2.Dot(s, s) - _radius * _radius;

	        // Does the segment start inside the circle?
	        if (b < 0.0f)
	        {
                return SegmentCollide.StartsInside;
	        }

	        // Solve quadratic equation.
	        Vector2 r = segment.p2 - segment.p1;
	        float c =  Vector2.Dot(s, r);
	        float rr = Vector2.Dot(r, r);
	        float sigma = c * c - rr * b;

	        // Check for negative discriminant and short segment.
	        if (sigma < 0.0f || rr < Settings.b2_FLT_EPSILON)
	        {
                return SegmentCollide.Miss;
	        }

	        // Find the point of intersection of the line with the circle.
	        float a = -(c + (float)Math.Sqrt((double)sigma));

	        // Is the intersection point on the segment?
	        if (0.0f <= a && a <= maxLambda * rr)
	        {
		        a /= rr;
		        lambda = a;
		        normal = s + a * r;
		        normal.Normalize();
                return SegmentCollide.Hit;
	        }

            return SegmentCollide.Miss;
        }

        /// @see Shape.ComputeAABB
        public override void ComputeAABB(out AABB aabb, ref XForm transform)
        {
            Vector2 p = transform.Position + MathUtils.Multiply(ref transform.R, _p);
	        aabb.lowerBound = new Vector2(p.X - _radius, p.Y - _radius);
	        aabb.upperBound = new Vector2(p.X + _radius, p.Y + _radius);
        }

        /// @see Shape.ComputeMass
        public override void ComputeMass(out MassData massData, float density)
        {
            massData.mass = density * Settings.b2_pi * _radius * _radius;
	        massData.center = _p;

	        // inertia about the local origin
	        massData.i = massData.mass * (0.5f * _radius * _radius + Vector2.Dot(_p, _p));
        }

        /// Get the supporting vertex index in the given direction.
        public override int GetSupport(Vector2 d)
        {
            return 0;
        }

        /// Get the supporting vertex in the given direction.
        public override Vector2 GetSupportVertex(Vector2 d)
        {
            return _p;
        }

        /// Get the vertex count.
        public override int GetVertexCount() { return 1; }

        /// Get a vertex by index. Used by b2Distance.
        public override Vector2 GetVertex(int index)
        {
            Debug.Assert(index == 0);
            return _p;
        }

        /// Position
        public Vector2 _p;
    }
}
