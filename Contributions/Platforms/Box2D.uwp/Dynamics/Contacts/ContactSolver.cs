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

//#define MATH_OVERLOADS

using Box2D.uwp.UWPExtensions;

using System.Diagnostics;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace Box2D.UWP
{
    public struct ContactConstraintPoint
    {
        public Vector2 localPoint;
        public Vector2 rA;
        public Vector2 rB;
        public float normalImpulse;
        public float tangentImpulse;
        public float normalMass;
        public float tangentMass;
        public float equalizedMass;
        public float velocityBias;
    };

    public struct ContactConstraint
    {
        public FixedArray2<ContactConstraintPoint> points;
        public Vector2 localPlaneNormal;
        public Vector2 localPoint;
        public Vector2 normal;
        public Mat22 normalMass;
        public Mat22 K;
        public Body bodyA;
        public Body bodyB;
        public ManifoldType type;
        public float radius;
        public float friction;
        public float restitution;
        public int pointCount;
        public Manifold manifold;
    };

    public class ContactSolver
    {
        public ContactSolver() { }

        public void Reset(ref TimeStep step, List<Contact> contacts)
        {
            _step = step;
            _contacts = contacts;

            int contactCount = contacts.Count;
            int constraintCount = contactCount;

            _constraints.Clear();
            for (int i = 0; i < constraintCount; i++)
            {
                _constraints.Add(new ContactConstraint());
            }

            for (int i = 0; i < constraintCount; ++i)
	        {
		        Contact contact = contacts[i];

		        Fixture fixtureA = contact._fixtureA;
		        Fixture fixtureB = contact._fixtureB;
		        Shape shapeA = fixtureA.GetShape();
		        Shape shapeB = fixtureB.GetShape();
		        float radiusA = shapeA._radius;
		        float radiusB = shapeB._radius;
		        Body bodyA = fixtureA.GetBody();
		        Body bodyB = fixtureB.GetBody();
                Manifold manifold;
                contact.GetManifold(out manifold);

		        float friction = Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
                float restitution = Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());

		        Vector2 vA = bodyA._linearVelocity;
		        Vector2 vB = bodyB._linearVelocity;
		        float wA = bodyA._angularVelocity;
		        float wB = bodyB._angularVelocity;

		        Debug.Assert(manifold._pointCount > 0);

		        WorldManifold worldManifold = new WorldManifold(ref manifold, ref bodyA._xf, radiusA, ref bodyB._xf, radiusB);

		        ContactConstraint cc = _constraints[i];
		        cc.bodyA = bodyA;
		        cc.bodyB = bodyB;
		        cc.manifold = manifold;
		        cc.normal = worldManifold._normal;
		        cc.pointCount = manifold._pointCount;
		        cc.friction = friction;
		        cc.restitution = restitution;

		        cc.localPlaneNormal = manifold._localPlaneNormal;
		        cc.localPoint = manifold._localPoint;
		        cc.radius = radiusA + radiusB;
		        cc.type = manifold._type;

		        for (int j = 0; j < cc.pointCount; ++j)
		        {
			        ManifoldPoint cp = manifold._points[j];
			        ContactConstraintPoint ccp = cc.points[j];

                    ccp.normalImpulse = cp.NormalImpulse;
			        ccp.tangentImpulse = cp.TangentImpulse;

			        ccp.localPoint = cp.LocalPoint;

			        ccp.rA = worldManifold._points[j] - bodyA._sweep.c;
			        ccp.rB = worldManifold._points[j] - bodyB._sweep.c;

			        float rnA = MathUtils.Cross(ccp.rA, cc.normal);
			        float rnB = MathUtils.Cross(ccp.rB, cc.normal);
			        rnA *= rnA;
			        rnB *= rnB;

			        float kNormal = bodyA._invMass + bodyB._invMass + bodyA._invI * rnA + bodyB._invI * rnB;

			        Debug.Assert(kNormal > Settings.b2_FLT_EPSILON);
			        ccp.normalMass = 1.0f / kNormal;

			        float kEqualized = bodyA._mass * bodyA._invMass + bodyB._mass * bodyB._invMass;
			        kEqualized += bodyA._mass * bodyA._invI * rnA + bodyB._mass * bodyB._invI * rnB;

			        Debug.Assert(kEqualized > Settings.b2_FLT_EPSILON);
			        ccp.equalizedMass = 1.0f / kEqualized;

			        Vector2 tangent = MathUtils.Cross(cc.normal, 1.0f);

			        float rtA = MathUtils.Cross(ccp.rA, tangent);
			        float rtB = MathUtils.Cross(ccp.rB, tangent);
			        rtA *= rtA;
			        rtB *= rtB;

			        float kTangent = bodyA._invMass + bodyB._invMass + bodyA._invI * rtA + bodyB._invI * rtB;

			        Debug.Assert(kTangent > Settings.b2_FLT_EPSILON);
			        ccp.tangentMass = 1.0f /  kTangent;

			        // Setup a velocity bias for restitution.
			        ccp.velocityBias = 0.0f;
			        float vRel = Vector2.Dot(cc.normal, vB + MathUtils.Cross(wB, ccp.rB) - vA - MathUtils.Cross(wA, ccp.rA));
			        if (vRel < -Settings.b2_velocityThreshold)
			        {
				        ccp.velocityBias = -cc.restitution * vRel;
			        }

                    cc.points[j] = ccp;
		        }

		        // If we have two points, then prepare the block solver.
		        if (cc.pointCount == 2)
		        {
			        ContactConstraintPoint ccp1 = cc.points[0];
			        ContactConstraintPoint ccp2 = cc.points[1];
        			
			        float invMassA = bodyA._invMass;
			        float invIA = bodyA._invI;
			        float invMassB = bodyB._invMass;
			        float invIB = bodyB._invI;

			        float rn1A = MathUtils.Cross(ccp1.rA, cc.normal);
			        float rn1B = MathUtils.Cross(ccp1.rB, cc.normal);
			        float rn2A = MathUtils.Cross(ccp2.rA, cc.normal);
			        float rn2B = MathUtils.Cross(ccp2.rB, cc.normal);

			        float k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
			        float k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
			        float k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

			        // Ensure a reasonable condition number.
			        float k_maxConditionNumber = 100.0f;
			        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			        {
				        // K is safe to invert.
				        cc.K = new Mat22(new Vector2(k11, k12), new Vector2(k12, k22));
				        cc.normalMass = cc.K.GetInverse();
			        }
			        else
			        {
				        // The constraints are redundant, just use one.
				        // TODO_ERIN use deepest?
				        cc.pointCount = 1;
			        }
		        }

                _constraints[i] = cc;
	        }
        }

        public void InitVelocityConstraints(ref TimeStep step)
        {
            int constraintCount = _constraints.Count;

            // Warm start.
            for (int i = 0; i < constraintCount; ++i)
            {
	            ContactConstraint c = _constraints[i];

	            Body bodyA = c.bodyA;
	            Body bodyB = c.bodyB;
	            float invMassA = bodyA._invMass;
	            float invIA = bodyA._invI;
	            float invMassB = bodyB._invMass;
	            float invIB = bodyB._invI;
	            Vector2 normal = c.normal;

#if MATH_OVERLOADS
	            Vector2 tangent = MathUtils.Cross(normal, 1.0f);
#else
                Vector2 tangent = new Vector2(normal.Y, -normal.X);
#endif

	            if (step.warmStarting)
	            {
		            for (int j = 0; j < c.pointCount; ++j)
		            {
			            ContactConstraintPoint ccp = c.points[j];
			            ccp.normalImpulse *= step.dtRatio;
			            ccp.tangentImpulse *= step.dtRatio;

#if MATH_OVERLOADS
			            Vector2 P = ccp.normalImpulse * normal + ccp.tangentImpulse * tangent;
			            bodyA._angularVelocity -= invIA * MathUtils.Cross(ccp.rA, P);
			            bodyA._linearVelocity -= invMassA * P;
			            bodyB._angularVelocity += invIB * MathUtils.Cross(ccp.rB, P);
			            bodyB._linearVelocity += invMassB * P;
#else
                        Vector2 P = new Vector2(ccp.normalImpulse * normal.X + ccp.tangentImpulse * tangent.X,
                                                ccp.normalImpulse * normal.Y + ccp.tangentImpulse * tangent.Y);
                        bodyA._angularVelocity -= invIA * (ccp.rA.X * P.Y - ccp.rA.Y * P.X);
                        bodyA._linearVelocity.X -= invMassA * P.X;
                        bodyA._linearVelocity.Y -= invMassA * P.Y;
                        bodyB._angularVelocity += invIB * (ccp.rB.X * P.Y - ccp.rB.Y * P.X);
                        bodyB._linearVelocity.X += invMassB * P.X;
                        bodyB._linearVelocity.Y += invMassB * P.Y;
#endif
                        c.points[j] = ccp;
		            }
	            }
	            else
	            {
		            for (int j = 0; j < c.pointCount; ++j)
		            {
			            ContactConstraintPoint ccp = c.points[j];
			            ccp.normalImpulse = 0.0f;
			            ccp.tangentImpulse = 0.0f;
                        c.points[j] = ccp;
		            }
	            }

                _constraints[i] = c;
            }
        }

        public void SolveVelocityConstraints()
        {
            int constraintCount = _constraints.Count;
            for (int i = 0; i < constraintCount; ++i)
	        {
		        ContactConstraint c = _constraints[i];
		        Body bodyA = c.bodyA;
		        Body bodyB = c.bodyB;
		        float wA = bodyA._angularVelocity;
		        float wB = bodyB._angularVelocity;
		        Vector2 vA = bodyA._linearVelocity;
		        Vector2 vB = bodyB._linearVelocity;
		        float invMassA = bodyA._invMass;
		        float invIA = bodyA._invI;
		        float invMassB = bodyB._invMass;
		        float invIB = bodyB._invI;
		        Vector2 normal = c.normal;

#if MATH_OVERLOADS
				Vector2 tangent = ZoomEngine.Physics.Common.Math.Cross(normal, 1.0f);
#else
                Vector2 tangent = new Vector2(normal.Y, -normal.X);
#endif
		        float friction = c.friction;

		        Debug.Assert(c.pointCount == 1 || c.pointCount == 2);

		        // Solve tangent constraints
		        for (int j = 0; j < c.pointCount; ++j)
		        {
			        ContactConstraintPoint ccp = c.points[j];

#if MATH_OVERLOADS
			        // Relative velocity at contact
			        Vector2 dv = vB + MathUtils.Cross(wB, ccp.rB) - vA - MathUtils.Cross(wA, ccp.rA);

			        // Compute tangent force
			        float vt = Vector2.Dot(dv, tangent);
#else
                    // Relative velocity at contact
                    Vector2 dv = new Vector2(vB.X + (-wB * ccp.rB.Y) - vA.X - (-wA * ccp.rA.Y),
                                             vB.Y + (wB * ccp.rB.X) - vA.Y - (wA * ccp.rA.X));

                    // Compute tangent force
                    float vt = dv.X * tangent.X + dv.Y * tangent.Y;
#endif
			        float lambda = ccp.tangentMass * (-vt);

			        // MathUtils.Clamp the accumulated force
			        float maxFriction = friction * ccp.normalImpulse;
			        float newImpulse = MathUtils.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
			        lambda = newImpulse - ccp.tangentImpulse;

#if MATH_OVERLOADS
			        // Apply contact impulse
			        Vector2 P = lambda * tangent;

			        vA -= invMassA * P;
			        wA -= invIA * MathUtils.Cross(ccp.rA, P);

			        vB += invMassB * P;
			        wB += invIB * MathUtils.Cross(ccp.rB, P);
#else
                    // Apply contact impulse
                    Vector2 P = new Vector2(lambda * tangent.X, lambda * tangent.Y);

                    vA.X -= invMassA * P.X;
                    vA.Y -= invMassA * P.Y;
                    wA -= invIA * (ccp.rA.X * P.Y - ccp.rA.Y * P.X);

                    vB.X += invMassB * P.X;
                    vB.Y += invMassB * P.Y;
                    wB += invIB * (ccp.rB.X * P.Y - ccp.rB.Y * P.X);
#endif
			        ccp.tangentImpulse = newImpulse;
                    c.points[j] = ccp;
		        }

		        // Solve normal constraints
		        if (c.pointCount == 1)
		        {
			        ContactConstraintPoint ccp = c.points[0];

#if MATH_OVERLOADS
			        // Relative velocity at contact
			        Vector2 dv = vB + MathUtils.Cross(wB, ccp.rB) - vA - MathUtils.Cross(wA, ccp.rA);

			        // Compute normal impulse
			        float vn = Vector2.Dot(dv, normal);
			        float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

			        // MathUtils.Clamp the accumulated impulse
			        float newImpulse = Math.Max(ccp.normalImpulse + lambda, 0.0f);
			        lambda = newImpulse - ccp.normalImpulse;

			        // Apply contact impulse
			        Vector2 P = lambda * normal;
			        vA -= invMassA * P;
			        wA -= invIA * MathUtils.Cross(ccp.rA, P);

			        vB += invMassB * P;
			        wB += invIB * MathUtils.Cross(ccp.rB, P);
#else
                    // Relative velocity at contact
                    Vector2 dv = new Vector2(vB.X + (-wB * ccp.rB.Y) - vA.X - (-wA * ccp.rA.Y),
                                             vB.Y + (wB * ccp.rB.X) - vA.Y - (wA * ccp.rA.X));

                    // Compute normal impulse
                    float vn = dv.X * normal.X + dv.Y * normal.Y;
                    float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

                    // Clamp the accumulated impulse
                    float newImpulse = Math.Max(ccp.normalImpulse + lambda, 0.0f);
                    lambda = newImpulse - ccp.normalImpulse;

                    // Apply contact impulse
                    var P = new Vector2(lambda * normal.X, lambda * normal.Y);

                    vA.X -= invMassA * P.X;
                    vA.Y -= invMassA * P.Y;
                    wA -= invIA * (ccp.rA.X * P.Y - ccp.rA.Y * P.X);

                    vB.X += invMassB * P.X;
                    vB.Y += invMassB * P.Y;
                    wB += invIB * (ccp.rB.X * P.Y - ccp.rB.Y * P.X);
#endif
                    ccp.normalImpulse = newImpulse;
                    c.points[0] = ccp;
		        }
		        else
		        {
			        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			        // Build the mini LCP for this contact patch
			        //
			        // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			        //
			        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			        // b = vn_0 - velocityBias
			        //
			        // The system is solved using the "Total enumeration method" (s. Murty). The complementary raint vn_i * x_i
			        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			        // solution that satisfies the problem is chosen.
			        // 
			        // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			        // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			        //
			        // Substitute:
			        // 
			        // x = x' - a
			        // 
			        // Plug into above equation:
			        //
			        // vn = A * x + b
			        //    = A * (x' - a) + b
			        //    = A * x' + b - A * a
			        //    = A * x' + b'
			        // b' = b - A * a;

			        ContactConstraintPoint cp1 = c.points[0];
			        ContactConstraintPoint cp2 = c.points[1];

			        Vector2 a = new Vector2(cp1.normalImpulse, cp2.normalImpulse);
			        Debug.Assert(a.X >= 0.0f && a.Y >= 0.0f);

#if MATH_OVERLOADS
			        // Relative velocity at contact
			        Vector2 dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
			        Vector2 dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

			        // Compute normal velocity
			        float vn1 = Vector2.Dot(dv1, normal);
			        float vn2 = Vector2.Dot(dv2, normal);

                    Vector2 b = new Vector2(vn1 - cp1.velocityBias, vn2 - cp2.velocityBias);
			        b -= MathUtils.Multiply(ref c.K, a);
#else
                    // Relative velocity at contact
                    Vector2 dv1 = new Vector2(vB.X + (-wB * cp1.rB.Y) - vA.X - (-wA * cp1.rA.Y),
                                              vB.Y + (wB * cp1.rB.X) - vA.Y - (wA * cp1.rA.X));
                    Vector2 dv2 = new Vector2(vB.X + (-wB * cp2.rB.Y) - vA.X - (-wA * cp2.rA.Y),
                                              vB.Y + (wB * cp2.rB.X) - vA.Y - (wA * cp2.rA.X));

                    // Compute normal velocity
                    float vn1 = dv1.X * normal.X + dv1.Y * normal.Y;
                    float vn2 = dv2.X * normal.X + dv2.Y * normal.Y;

                    Vector2 b = new Vector2(vn1 - cp1.velocityBias, vn2 - cp2.velocityBias);
                    b -= MathUtils.Multiply(ref c.K, a); // Inlining didn't help for the multiply.
#endif
                    while (true)
			        {
				        //
				        // Case 1: vn = 0
				        //
				        // 0 = A * x' + b'
				        //
				        // Solve for x':
				        //
				        // x' = - inv(A) * b'
				        //
				        Vector2 x = - MathUtils.Multiply(ref c.normalMass, b);

				        if (x.X >= 0.0f && x.Y >= 0.0f)
                        {
#if MATH_OVERLOADS
					        // Resubstitute for the incremental impulse
					        Vector2 d = x - a;

					        // Apply incremental impulse
					        Vector2 P1 = d.X * normal;
					        Vector2 P2 = d.Y * normal;
					        vA -= invMassA * (P1 + P2);
					        wA -= invIA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

					        vB += invMassB * (P1 + P2);
					        wB += invIB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));
#else
                            // Resubstitute for the incremental impulse
                            Vector2 d = new Vector2(x.X - a.X, x.Y - a.Y);

                            // Apply incremental impulse
                            Vector2 P1 = new Vector2(d.X * normal.X, d.X * normal.Y);
                            Vector2 P2 = new Vector2(d.Y * normal.X, d.Y * normal.Y);
                            Vector2 P12 = new Vector2(P1.X + P2.X, P1.Y + P2.Y);

                            vA.X -= invMassA * P12.X;
                            vA.Y -= invMassA * P12.Y;
                            wA -= invIA * ((cp1.rA.X * P1.Y - cp1.rA.Y * P1.X) + (cp2.rA.X * P2.Y - cp2.rA.Y * P2.X));

                            vB.X += invMassB * P12.X;
                            vB.Y += invMassB * P12.Y;
                            wB += invIB * ((cp1.rB.X * P1.Y - cp1.rB.Y * P1.X) + (cp2.rB.X * P2.Y - cp2.rB.Y * P2.X));
#endif
					        // Accumulate
					        cp1.normalImpulse = x.X;
					        cp2.normalImpulse = x.Y;

#if B2_DEBUG_SOLVER 
                            
			                float k_errorTol = 1e-3f;

					        // Postconditions
					        dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
					        dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

					        // Compute normal velocity
					        vn1 = Vector2.Dot(dv1, normal);
					        vn2 = Vector2.Dot(dv2, normal);

					        Debug.Assert(MathUtils.Abs(vn1 - cp1.velocityBias) < k_errorTol);
					        Debug.Assert(MathUtils.Abs(vn2 - cp2.velocityBias) < k_errorTol);
#endif
                            break;
				        }

				        //
				        // Case 2: vn1 = 0 and x2 = 0
				        //
				        //   0 = a11 * x1' + a12 * 0 + b1' 
				        // vn2 = a21 * x1' + a22 * 0 + b2'
				        //
				        x.X = - cp1.normalMass * b.X;
				        x.Y = 0.0f;
				        vn1 = 0.0f;
				        vn2 = c.K.col1.Y * x.X + b.Y;

				        if (x.X >= 0.0f && vn2 >= 0.0f)
				        {
#if MATH_OVERLOADS
					        // Resubstitute for the incremental impulse
					        Vector2 d = x - a;

					        // Apply incremental impulse
					        Vector2 P1 = d.X * normal;
					        Vector2 P2 = d.Y * normal;
					        vA -= invMassA * (P1 + P2);
					        wA -= invIA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

					        vB += invMassB * (P1 + P2);
					        wB += invIB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));
#else
                            // Resubstitute for the incremental impulse
                            Vector2 d = new Vector2(x.X - a.X, x.Y - a.Y);

                            // Apply incremental impulse
                            Vector2 P1 = new Vector2(d.X * normal.X, d.X * normal.Y);
                            Vector2 P2 = new Vector2(d.Y * normal.X, d.Y * normal.Y);
                            Vector2 P12 = new Vector2(P1.X + P2.X, P1.Y + P2.Y);

                            vA.X -= invMassA * P12.X;
                            vA.Y -= invMassA * P12.Y;
                            wA -= invIA * ((cp1.rA.X * P1.Y - cp1.rA.Y * P1.X) + (cp2.rA.X * P2.Y - cp2.rA.Y * P2.X));

                            vB.X += invMassB * P12.X;
                            vB.Y += invMassB * P12.Y;
                            wB += invIB * ((cp1.rB.X * P1.Y - cp1.rB.Y * P1.X) + (cp2.rB.X * P2.Y - cp2.rB.Y * P2.X));
#endif
					        // Accumulate
					        cp1.normalImpulse = x.X;
					        cp2.normalImpulse = x.Y;

        #if B2_DEBUG_SOLVER 
					        // Postconditions
					        dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);

					        // Compute normal velocity
					        vn1 = Vector2.Dot(dv1, normal);

					        Debug.Assert(MathUtils.Abs(vn1 - cp1.velocityBias) < k_errorTol);
        #endif
					        break;
				        }


				        //
				        // Case 3: wB = 0 and x1 = 0
				        //
				        // vn1 = a11 * 0 + a12 * x2' + b1' 
				        //   0 = a21 * 0 + a22 * x2' + b2'
				        //
				        x.X = 0.0f;
				        x.Y = - cp2.normalMass * b.Y;
				        vn1 = c.K.col2.X * x.Y + b.X;
				        vn2 = 0.0f;

				        if (x.Y >= 0.0f && vn1 >= 0.0f)
				        {
#if MATH_OVERLOADS
					        // Resubstitute for the incremental impulse
					        Vector2 d = x - a;

					        // Apply incremental impulse
					        Vector2 P1 = d.X * normal;
					        Vector2 P2 = d.Y * normal;
					        vA -= invMassA * (P1 + P2);
					        wA -= invIA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

					        vB += invMassB * (P1 + P2);
					        wB += invIB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));
#else
                            // Resubstitute for the incremental impulse
                            Vector2 d = new Vector2(x.X - a.X, x.Y - a.Y);

                            // Apply incremental impulse
                            Vector2 P1 = new Vector2(d.X * normal.X, d.X * normal.Y);
                            Vector2 P2 = new Vector2(d.Y * normal.X, d.Y * normal.Y);
                            Vector2 P12 = new Vector2(P1.X + P2.X, P1.Y + P2.Y);

                            vA.X -= invMassA * P12.X;
                            vA.Y -= invMassA * P12.Y;
                            wA -= invIA * ((cp1.rA.X * P1.Y - cp1.rA.Y * P1.X) + (cp2.rA.X * P2.Y - cp2.rA.Y * P2.X));

                            vB.X += invMassB * P12.X;
                            vB.Y += invMassB * P12.Y;
                            wB += invIB * ((cp1.rB.X * P1.Y - cp1.rB.Y * P1.X) + (cp2.rB.X * P2.Y - cp2.rB.Y * P2.X));
#endif
					        // Accumulate
					        cp1.normalImpulse = x.X;
					        cp2.normalImpulse = x.Y;

        #if B2_DEBUG_SOLVER 
					        // Postconditions
					        dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

					        // Compute normal velocity
					        vn2 = Vector2.Dot(dv2, normal);

					        Debug.Assert(MathUtils.Abs(vn2 - cp2.velocityBias) < k_errorTol);
        #endif
					        break;
				        }

				        //
				        // Case 4: x1 = 0 and x2 = 0
				        // 
				        // vn1 = b1
				        // vn2 = b2;
				        x.X = 0.0f;
				        x.Y = 0.0f;
				        vn1 = b.X;
				        vn2 = b.Y;

				        if (vn1 >= 0.0f && vn2 >= 0.0f )
				        {
#if MATH_OVERLOADS
					        // Resubstitute for the incremental impulse
					        Vector2 d = x - a;

					        // Apply incremental impulse
					        Vector2 P1 = d.X * normal;
					        Vector2 P2 = d.Y * normal;
					        vA -= invMassA * (P1 + P2);
					        wA -= invIA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

					        vB += invMassB * (P1 + P2);
					        wB += invIB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));
#else
                            // Resubstitute for the incremental impulse
                            Vector2 d = new Vector2(x.X - a.X, x.Y - a.Y);

                            // Apply incremental impulse
                            Vector2 P1 = new Vector2(d.X * normal.X, d.X * normal.Y);
                            Vector2 P2 = new Vector2(d.Y * normal.X, d.Y * normal.Y);
                            Vector2 P12 = new Vector2(P1.X + P2.X, P1.Y + P2.Y);

                            vA.X -= invMassA * P12.X;
                            vA.Y -= invMassA * P12.Y;
                            wA -= invIA * ((cp1.rA.X * P1.Y - cp1.rA.Y * P1.X) + (cp2.rA.X * P2.Y - cp2.rA.Y * P2.X));

                            vB.X += invMassB * P12.X;
                            vB.Y += invMassB * P12.Y;
                            wB += invIB * ((cp1.rB.X * P1.Y - cp1.rB.Y * P1.X) + (cp2.rB.X * P2.Y - cp2.rB.Y * P2.X));
#endif
					        // Accumulate
					        cp1.normalImpulse = x.X;
					        cp2.normalImpulse = x.Y;

					        break;
				        }

				        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				        break;
			        }

                    c.points[0] = cp1;
                    c.points[1] = cp2;
		        }

                _constraints[i] = c;

		        bodyA._linearVelocity = vA;
		        bodyA._angularVelocity = wA;
		        bodyB._linearVelocity = vB;
		        bodyB._angularVelocity = wB;
	        }
        }

        public void FinalizeVelocityConstraints()
        {
            int constraintCount = _constraints.Count;
            for (int i = 0; i < constraintCount; ++i)
	        {
		        ContactConstraint c = _constraints[i];
		        Manifold m = c.manifold;

		        for (int j = 0; j < c.pointCount; ++j)
		        {
                    var pj = m._points[j];
                    var cp = c.points[j];

                    pj.NormalImpulse = cp.normalImpulse;
                    pj.TangentImpulse = cp.tangentImpulse;

                    m._points[j] = pj;
		        }

                // TODO: look for better ways of doing this.
                c.manifold = m;
                _constraints[i] = c;
                _contacts[i]._manifold = m; 
	        }
        }

        public bool SolvePositionConstraints(float baumgarte)
        {
            float minSeparation = 0.0f;

            int constraintCount = _constraints.Count;
            for (int i = 0; i < constraintCount; ++i)
	        {
		        ContactConstraint c = _constraints[i];

		        Body bodyA = c.bodyA;
		        Body bodyB = c.bodyB;

		        float invMassA = bodyA._mass * bodyA._invMass;
		        float invIA = bodyA._mass * bodyA._invI;
		        float invMassB = bodyB._mass * bodyB._invMass;
		        float invIB = bodyB._mass * bodyB._invI;

                PositionSolverManifold psm = new PositionSolverManifold(ref c);
		        Vector2 normal = psm._normal;

		        // Solve normal constraints
		        for (int j = 0; j < c.pointCount; ++j)
		        {
			        ContactConstraintPoint ccp = c.points[j];

			        Vector2 point = psm._points[j];
			        float separation = psm._separations[j];

			        Vector2 rA = point - bodyA._sweep.c;
			        Vector2 rB = point - bodyB._sweep.c;

			        // Track max raint error.
			        minSeparation = Math.Min(minSeparation, separation);

			        // Prevent large corrections and allow slop.
			        float C = baumgarte * MathUtils.Clamp(separation + Settings.b2_linearSlop, -Settings.b2_maxLinearCorrection, 0.0f);

			        // Compute normal impulse
			        float impulse = -ccp.equalizedMass * C;
#if MATH_OVERLOADS
			        Vector2 P = impulse * normal;

			        bodyA._sweep.c -= invMassA * P;
			        bodyA._sweep.a -= invIA * MathUtils.Cross(rA, P);
			        
			        bodyB._sweep.c += invMassB * P;
			        bodyB._sweep.a += invIB * MathUtils.Cross(rB, P);
#else
                    Vector2 P = new Vector2(impulse * normal.X, impulse * normal.Y);

                    bodyA._sweep.c.X -= invMassA * P.X;
                    bodyA._sweep.c.Y -= invMassA * P.Y;
                    bodyA._sweep.a -= invIA * (rA.X * P.Y - rA.Y * P.X);

                    bodyB._sweep.c.X += invMassB * P.X;
                    bodyB._sweep.c.Y += invMassB * P.Y;
                    bodyB._sweep.a += invIB * (rB.X * P.Y - rB.Y * P.X);
#endif
                    bodyA.SynchronizeTransform();
			        bodyB.SynchronizeTransform();
		        }
	        }

	        // We can't expect minSpeparation >= -Settings.b2_linearSlop because we don't
	        // push the separation above -Settings.b2_linearSlop.
	        return minSeparation >= -1.5f * Settings.b2_linearSlop;
        }

        public TimeStep _step;
        public List<ContactConstraint> _constraints = new List<ContactConstraint>(50);
        List<Contact> _contacts;
    };

    internal struct PositionSolverManifold
    {
        internal PositionSolverManifold(ref ContactConstraint cc)
        {
            _points = new FixedArray2<Vector2>();
            _separations = new FixedArray2<float>();
            _normal = new Vector2();

	        Debug.Assert(cc.pointCount > 0);

	        switch (cc.type)
	        {
	        case ManifoldType.Circles:
		        {
			        Vector2 pointA = cc.bodyA.GetWorldPoint(cc.localPoint);
			        Vector2 pointB = cc.bodyB.GetWorldPoint(cc.points[0].localPoint);
			        if (Vector2.DistanceSquared(pointA, pointB) > Settings.b2_FLT_EPSILON * Settings.b2_FLT_EPSILON)
			        {
				        _normal = pointB - pointA;
				        _normal.Normalize();
			        }
			        else
			        {
				        _normal = new Vector2(1.0f, 0.0f);
			        }

			        _points[0] = 0.5f * (pointA + pointB);
			        _separations[0] = Vector2.Dot(pointB - pointA, _normal) - cc.radius;
		        }
		        break;

	        case ManifoldType.FaceA:
		        {
			        _normal = cc.bodyA.GetWorldVector(cc.localPlaneNormal);
			        Vector2 planePoint = cc.bodyA.GetWorldPoint(cc.localPoint);

			        for (int i = 0; i < cc.pointCount; ++i)
			        {
				        Vector2 clipPoint = cc.bodyB.GetWorldPoint(cc.points[i].localPoint);
				        _separations[i] = Vector2.Dot(clipPoint - planePoint, _normal) - cc.radius;
				        _points[i] = clipPoint;
			        }
		        }
		        break;

	        case ManifoldType.FaceB:
		        {
			        _normal = cc.bodyB.GetWorldVector(cc.localPlaneNormal);
			        Vector2 planePoint = cc.bodyB.GetWorldPoint(cc.localPoint);

			        for (int i = 0; i < cc.pointCount; ++i)
			        {
				        Vector2 clipPoint = cc.bodyA.GetWorldPoint(cc.points[i].localPoint);
				        _separations[i] = Vector2.Dot(clipPoint - planePoint, _normal) - cc.radius;
				        _points[i] = clipPoint;
			        }

			        // Ensure normal points from A to B
			        _normal = -_normal;
		        }
		        break;
	        }
        }

        internal Vector2 _normal;
        internal FixedArray2<Vector2> _points;
        internal FixedArray2<float> _separations;
    };
}
