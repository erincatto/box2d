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
using System.Numerics;

namespace Box2D.UWP
{
    public class PolygonShape : Shape
    {
	    public PolygonShape()
        {
	        ShapeType = ShapeType.Polygon;
	        _radius = Settings.b2_polygonRadius;
	        _vertexCount = 0;
	        _centroid = Vector2.Zero;
        }

	    /// Implement Shape.
	    public override Shape Clone() 
        { 
            var clone = new PolygonShape();
            clone.ShapeType = ShapeType;
            clone._radius = _radius;
            clone._vertexCount = _vertexCount;
            clone._centroid = _centroid;
            clone._vertices = _vertices;
            clone._normals = _normals;

            return clone;
        }

	    /// Copy vertices. This assumes the vertices define a convex polygon.
	    /// It is assumed that the exterior is the the right of each edge.
        public void Set(Vector2[] vertices, int count)
        {
            Debug.Assert(2 <= count && count <= Settings.b2_maxPolygonVertices);
            _vertexCount = count;

            // Copy vertices.
            for (int i = 0; i < _vertexCount; ++i)
            {
	            _vertices[i] = vertices[i];
            }

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < _vertexCount; ++i)
            {
	            int i1 = i;
	            int i2 = i + 1 < _vertexCount ? i + 1 : 0;
	            Vector2 edge = _vertices[i2] - _vertices[i1];
	            Debug.Assert(edge.LengthSquared() > Settings.b2_FLT_EPSILON * Settings.b2_FLT_EPSILON);
                
                var temp = MathUtils.Cross(edge, 1.0f);
                temp.Normalize();
                _normals[i] = temp;
            }

        #if _DEBUG
            // Ensure the polygon is convex and the interior
            // is to the left of each edge.
            for (int i = 0; i < _vertexCount; ++i)
            {
	            int i1 = i;
	            int i2 = i + 1 < _vertexCount ? i + 1 : 0;
	            Vector2 edge = _vertices[i2] - _vertices[i1];

	            for (int j = 0; j < _vertexCount; ++j)
	            {
		            // Don't check vertices on the current edge.
		            if (j == i1 || j == i2)
		            {
			            continue;
		            }
        			
		            Vector2 r = _vertices[j] - _vertices[i1];

		            // Your polygon is non-convex (it has an indentation) or
		            // has colinear edges.
		            float s = MathUtils.Cross(edge, r);
		            Debug.Assert(s > 0.0f);
	            }
            }
        #endif

            // Compute the polygon centroid.
            _centroid = ComputeCentroid(ref _vertices, _vertexCount);
        }

        static Vector2 ComputeCentroid(ref FixedArray8<Vector2> vs, int count)
        {
	        Debug.Assert(count >= 2);

	        Vector2 c = new Vector2(0.0f, 0.0f);
	        float area = 0.0f;

	        if (count == 2)
	        {
		        c = 0.5f * (vs[0] + vs[1]);
		        return c;
	        }

	        // pRef is the reference point for forming triangles.
	        // It's location doesn't change the result (except for rounding error).
	        Vector2 pRef = new Vector2(0.0f, 0.0f);
        #if false
	        // This code would put the reference point inside the polygon.
	        for (int i = 0; i < count; ++i)
	        {
		        pRef += vs[i];
	        }
	        pRef *= 1.0f / count;
        #endif

	        float inv3 = 1.0f / 3.0f;

	        for (int i = 0; i < count; ++i)
	        {
		        // Triangle vertices.
		        Vector2 p1 = pRef;
		        Vector2 p2 = vs[i];
		        Vector2 p3 = i + 1 < count ? vs[i+1] : vs[0];

		        Vector2 e1 = p2 - p1;
		        Vector2 e2 = p3 - p1;

		        float D = MathUtils.Cross(e1, e2);

		        float triangleArea = 0.5f * D;
		        area += triangleArea;

		        // Area weighted centroid
		        c += triangleArea * inv3 * (p1 + p2 + p3);
	        }

	        // Centroid
	        Debug.Assert(area > Settings.b2_FLT_EPSILON);
	        c *= 1.0f / area;
	        return c;
        }

	    /// Build vertices to represent an axis-aligned box.
	    /// @param hx the half-width.
	    /// @param hy the half-height.
        public void SetAsBox(float hx, float hy)
        {
	        _vertexCount = 4;
	        _vertices[0] = new Vector2(-hx, -hy);
	        _vertices[1] = new Vector2( hx, -hy);
	        _vertices[2] = new Vector2( hx,  hy);
	        _vertices[3] = new Vector2(-hx,  hy);
	        _normals[0] = new Vector2(0.0f, -1.0f);
	        _normals[1] = new Vector2(1.0f, 0.0f);
	        _normals[2] = new Vector2(0.0f, 1.0f);
	        _normals[3] = new Vector2(-1.0f, 0.0f);
	        _centroid = Vector2.Zero;
        }

	    /// Build vertices to represent an oriented box.
	    /// @param hx the half-width.
	    /// @param hy the half-height.
	    /// @param center the center of the box in local coordinates.
	    /// @param angle the rotation of the box in local coordinates.
        public void SetAsBox(float hx, float hy, Vector2 center, float angle)
        {
	        _vertexCount = 4;
	        _vertices[0] = new Vector2(-hx, -hy);
	        _vertices[1] = new Vector2( hx, -hy);
	        _vertices[2] = new Vector2( hx,  hy);
	        _vertices[3] = new Vector2(-hx,  hy);
	        _normals[0] = new Vector2(0.0f, -1.0f);
	        _normals[1] = new Vector2(1.0f, 0.0f);
	        _normals[2] = new Vector2(0.0f, 1.0f);
	        _normals[3] = new Vector2(-1.0f, 0.0f);
	        _centroid = center;

            XForm xf = new XForm();
	        xf.Position = center;
	        xf.R.Set(angle);

	        // Transform vertices and normals.
	        for (int i = 0; i < _vertexCount; ++i)
	        {
		        _vertices[i] = MathUtils.Multiply(ref xf, _vertices[i]);
		        _normals[i] = MathUtils.Multiply(ref xf.R, _normals[i]);
	        }
        }

	    /// Set this as a single edge.
        public void SetAsEdge(Vector2 v1, Vector2 v2)
        {
	        _vertexCount = 2;
	        _vertices[0] = v1;
	        _vertices[1] = v2;
	        _centroid = 0.5f * (v1 + v2);
            
            var temp = MathUtils.Cross(v2 - v1, 1.0f);
            temp.Normalize();
            _normals[0] = temp;

	        _normals[1] = -_normals[0];
        }

	    /// @see Shape.TestPoint
	    public override bool TestPoint(ref XForm xf, Vector2 p)
        {
	        Vector2 pLocal = MathUtils.MultiplyT(ref xf.R, p - xf.Position);

	        for (int i = 0; i < _vertexCount; ++i)
	        {
		        float dot = Vector2.Dot(_normals[i], pLocal - _vertices[i]);
		        if (dot > 0.0f)
		        {
			        return false;
		        }
	        }

	        return true;
        }

	    /// @see Shape.TestSegment
        public override SegmentCollide TestSegment(ref XForm xf,
		    out float lambda,
		    out Vector2 normal,
		    ref Segment segment,
		    float maxLambda)
        {
            lambda = 0;
            normal = Vector2.Zero;
            float lower = 0.0f, upper = maxLambda;

	        Vector2 p1 = MathUtils.MultiplyT(ref xf.R, segment.p1 - xf.Position);
	        Vector2 p2 = MathUtils.MultiplyT(ref xf.R, segment.p2 - xf.Position);
	        Vector2 d = p2 - p1;
	        int index = -1;

	        for (int i = 0; i < _vertexCount; ++i)
	        {
		        // p = p1 + a * d
		        // dot(normal, p - v) = 0
		        // dot(normal, p1 - v) + a * dot(normal, d) = 0
		        float numerator = Vector2.Dot(_normals[i], _vertices[i] - p1);
		        float denominator = Vector2.Dot(_normals[i], d);

		        if (denominator == 0.0f)
		        {	
			        if (numerator < 0.0f)
			        {
				        return SegmentCollide.Miss;
			        }
		        }
		        else
		        {
			        // Note: we want this predicate without division:
			        // lower < numerator / denominator, where denominator < 0
			        // Since denominator < 0, we have to flip the inequality:
			        // lower < numerator / denominator <==> denominator * lower > numerator.
			        if (denominator < 0.0f && numerator < lower * denominator)
			        {
				        // Increase lower.
				        // The segment enters this half-space.
				        lower = numerator / denominator;
				        index = i;
			        }
			        else if (denominator > 0.0f && numerator < upper * denominator)
			        {
				        // Decrease upper.
				        // The segment exits this half-space.
				        upper = numerator / denominator;
			        }
		        }

		        if (upper < lower)
		        {
                    return SegmentCollide.Miss;
		        }
	        }

	        Debug.Assert(0.0f <= lower && lower <= maxLambda);

	        if (index >= 0)
	        {
		        lambda = lower;
		        normal = MathUtils.Multiply(ref xf.R, _normals[index]);
                return SegmentCollide.Hit;
	        }

	        lambda = 0;
            return SegmentCollide.StartsInside;
        }

	    /// @see Shape.ComputeAABB
        public override void ComputeAABB(out AABB aabb, ref XForm xf)
        {
	        Vector2 lower = MathUtils.Multiply(ref xf, _vertices[0]);
	        Vector2 upper = lower;

	        for (int i = 1; i < _vertexCount; ++i)
	        {
		        Vector2 v = MathUtils.Multiply(ref xf, _vertices[i]);
		        lower = Vector2.Min(lower, v);
		        upper = Vector2.Max(upper, v);
	        }

	        Vector2 r = new Vector2(_radius, _radius);
	        aabb.lowerBound = lower - r;
	        aabb.upperBound = upper + r;
        }

	    /// @see Shape.ComputeMass
	    public override void ComputeMass(out MassData massData, float density)
        {
	        // Polygon mass, centroid, and inertia.
	        // Let rho be the polygon density in mass per unit area.
	        // Then:
	        // mass = rho * int(dA)
	        // centroid.X = (1/mass) * rho * int(x * dA)
	        // centroid.Y = (1/mass) * rho * int(y * dA)
	        // I = rho * int((x*x + y*y) * dA)
	        //
	        // We can compute these integrals by summing all the integrals
	        // for each triangle of the polygon. To evaluate the integral
	        // for a single triangle, we make a change of variables to
	        // the (u,v) coordinates of the triangle:
	        // x = x0 + e1x * u + e2x * v
	        // y = y0 + e1y * u + e2y * v
	        // where 0 <= u && 0 <= v && u + v <= 1.
	        //
	        // We integrate u from [0,1-v] and then v from [0,1].
	        // We also need to use the Jacobian of the transformation:
	        // D = cross(e1, e2)
	        //
	        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	        //
	        // The rest of the derivation is handled by computer algebra.

	        Debug.Assert(_vertexCount >= 3);

	        Vector2 center = new Vector2(0.0f, 0.0f);
	        float area = 0.0f;
	        float I = 0.0f;

	        // pRef is the reference point for forming triangles.
	        // It's location doesn't change the result (except for rounding error).
	        Vector2 pRef = new Vector2(0.0f, 0.0f);
        #if false
	        // This code would put the reference point inside the polygon.
	        for (int i = 0; i < _vertexCount; ++i)
	        {
		        pRef += _vertices[i];
	        }
	        pRef *= 1.0f / count;
        #endif

	        float k_inv3 = 1.0f / 3.0f;

	        for (int i = 0; i < _vertexCount; ++i)
	        {
		        // Triangle vertices.
		        Vector2 p1 = pRef;
		        Vector2 p2 = _vertices[i];
		        Vector2 p3 = i + 1 < _vertexCount ? _vertices[i+1] : _vertices[0];

		        Vector2 e1 = p2 - p1;
		        Vector2 e2 = p3 - p1;

		        float D = MathUtils.Cross(e1, e2);

		        float triangleArea = 0.5f * D;
		        area += triangleArea;

		        // Area weighted centroid
		        center += triangleArea * k_inv3 * (p1 + p2 + p3);

		        float px = p1.X, py = p1.Y;
		        float ex1 = e1.X, ey1 = e1.Y;
		        float ex2 = e2.X, ey2 = e2.Y;

		        float intx2 = k_inv3 * (0.25f * (ex1*ex1 + ex2*ex1 + ex2*ex2) + (px*ex1 + px*ex2)) + 0.5f*px*px;
		        float inty2 = k_inv3 * (0.25f * (ey1*ey1 + ey2*ey1 + ey2*ey2) + (py*ey1 + py*ey2)) + 0.5f*py*py;

		        I += D * (intx2 + inty2);
	        }

	        // Total mass
	        massData.mass = density * area;

	        // Center of mass
	        Debug.Assert(area > Settings.b2_FLT_EPSILON);
	        center *= 1.0f / area;
	        massData.center = center;

	        // Inertia tensor relative to the local origin.
	        massData.i = density * I;
        }

	    /// Get the supporting vertex index in the given direction.
	    public override int GetSupport(Vector2 d)
        {
	        int bestIndex = 0;
	        float bestValue = Vector2.Dot(_vertices[0], d);
	        for (int i = 1; i < _vertexCount; ++i)
	        {
		        float value = Vector2.Dot(_vertices[i], d);
		        if (value > bestValue)
		        {
			        bestIndex = i;
			        bestValue = value;
		        }
	        }

	        return bestIndex;
        }

	    /// Get the supporting vertex in the given direction.
        public override Vector2 GetSupportVertex(Vector2 d)
        {
	        int bestIndex = 0;
	        float bestValue = Vector2.Dot(_vertices[0], d);
	        for (int i = 1; i < _vertexCount; ++i)
	        {
		        float value = Vector2.Dot(_vertices[i], d);
		        if (value > bestValue)
		        {
			        bestIndex = i;
			        bestValue = value;
		        }
	        }

	        return _vertices[bestIndex];
        }

	    /// Get the vertex count.
        public override int GetVertexCount() { return _vertexCount; }

	    /// Get a vertex by index.
        public override Vector2 GetVertex(int index)
        {
	        Debug.Assert(0 <= index && index < _vertexCount);
	        return _vertices[index];
        }

        public Vector2 _centroid;
        public FixedArray8<Vector2> _vertices;
        public FixedArray8<Vector2> _normals;
	    public int _vertexCount;
    }
}
