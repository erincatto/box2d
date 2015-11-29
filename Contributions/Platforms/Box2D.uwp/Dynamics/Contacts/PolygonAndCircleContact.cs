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
namespace Box2D.UWP
{
    internal class PolygonAndCircleContact : Contact
    {
	    internal PolygonAndCircleContact(Fixture fixtureA, Fixture fixtureB)
            : base(fixtureA, fixtureB)
        {
            Debug.Assert(_fixtureA.ShapeType == ShapeType.Polygon);
	        Debug.Assert(_fixtureB.ShapeType == ShapeType.Circle);
        }

        internal override void Evaluate()  
        {
            Body b1 = _fixtureA.GetBody();
            Body b2 = _fixtureB.GetBody();

            XForm xf1, xf2;
            b1.GetXForm(out xf1);
            b2.GetXForm(out xf2);

	        Collision.CollidePolygonAndCircle(ref _manifold,
                                        (PolygonShape)_fixtureA.GetShape(), ref xf1,
                                        (CircleShape)_fixtureB.GetShape(), ref xf2);
        }

        internal override float ComputeTOI(ref Sweep sweepA, ref Sweep sweepB)
        {
            TOIInput input;
	        input.sweepA = sweepA;
	        input.sweepB = sweepB;
	        input.tolerance = Settings.b2_linearSlop;

	        return TimeOfImpact.CalculateTimeOfImpact(ref input, (PolygonShape)_fixtureA.GetShape(), (CircleShape)_fixtureB.GetShape());
        }
    }
}
