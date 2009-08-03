/*
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

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2StackAllocator.h>

#define B2_DEBUG_SOLVER 0

b2ContactSolver::b2ContactSolver(const b2TimeStep& step, b2Contact** contacts, int32 contactCount, b2StackAllocator* allocator)
{
	m_step = step;
	m_allocator = allocator;

	m_constraintCount = contactCount;
	m_constraints = (b2ContactConstraint*)m_allocator->Allocate(m_constraintCount * sizeof(b2ContactConstraint));

	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2Contact* contact = contacts[i];

		b2Fixture* fixtureA = contact->m_fixtureA;
		b2Fixture* fixtureB = contact->m_fixtureB;
		b2Shape* shapeA = fixtureA->GetShape();
		b2Shape* shapeB = fixtureB->GetShape();
		float32 radiusA = shapeA->m_radius;
		float32 radiusB = shapeB->m_radius;
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();
		b2Manifold* manifold = contact->GetManifold();

		float32 friction = b2MixFriction(fixtureA->GetFriction(), fixtureB->GetFriction());
		float32 restitution = b2MixRestitution(fixtureA->GetRestitution(), fixtureB->GetRestitution());

		b2Vec2 vA = bodyA->m_linearVelocity;
		b2Vec2 vB = bodyB->m_linearVelocity;
		float32 wA = bodyA->m_angularVelocity;
		float32 wB = bodyB->m_angularVelocity;

		b2Assert(manifold->m_pointCount > 0);

		b2WorldManifold worldManifold;
		worldManifold.Initialize(manifold, bodyA->m_xf, radiusA, bodyB->m_xf, radiusB);

		b2ContactConstraint* cc = m_constraints + i;
		cc->bodyA = bodyA;
		cc->bodyB = bodyB;
		cc->manifold = manifold;
		cc->normal = worldManifold.m_normal;
		cc->pointCount = manifold->m_pointCount;
		cc->friction = friction;
		cc->restitution = restitution;

		cc->localPlaneNormal = manifold->m_localPlaneNormal;
		cc->localPoint = manifold->m_localPoint;
		cc->radius = radiusA + radiusB;
		cc->type = manifold->m_type;

		for (int32 j = 0; j < cc->pointCount; ++j)
		{
			b2ManifoldPoint* cp = manifold->m_points + j;
			b2ContactConstraintPoint* ccp = cc->points + j;

			ccp->normalImpulse = cp->m_normalImpulse;
			ccp->tangentImpulse = cp->m_tangentImpulse;

			ccp->localPoint = cp->m_localPoint;

			ccp->rA = worldManifold.m_points[j] - bodyA->m_sweep.c;
			ccp->rB = worldManifold.m_points[j] - bodyB->m_sweep.c;

			float32 rnA = b2Cross(ccp->rA, cc->normal);
			float32 rnB = b2Cross(ccp->rB, cc->normal);
			rnA *= rnA;
			rnB *= rnB;

			float32 kNormal = bodyA->m_invMass + bodyB->m_invMass + bodyA->m_invI * rnA + bodyB->m_invI * rnB;

			b2Assert(kNormal > B2_FLT_EPSILON);
			ccp->normalMass = 1.0f / kNormal;

			float32 kEqualized = bodyA->m_mass * bodyA->m_invMass + bodyB->m_mass * bodyB->m_invMass;
			kEqualized += bodyA->m_mass * bodyA->m_invI * rnA + bodyB->m_mass * bodyB->m_invI * rnB;

			b2Assert(kEqualized > B2_FLT_EPSILON);
			ccp->equalizedMass = 1.0f / kEqualized;

			b2Vec2 tangent = b2Cross(cc->normal, 1.0f);

			float32 rtA = b2Cross(ccp->rA, tangent);
			float32 rtB = b2Cross(ccp->rB, tangent);
			rtA *= rtA;
			rtB *= rtB;

			float32 kTangent = bodyA->m_invMass + bodyB->m_invMass + bodyA->m_invI * rtA + bodyB->m_invI * rtB;

			b2Assert(kTangent > B2_FLT_EPSILON);
			ccp->tangentMass = 1.0f /  kTangent;

			// Setup a velocity bias for restitution.
			ccp->velocityBias = 0.0f;
			float32 vRel = b2Dot(cc->normal, vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA));
			if (vRel < -b2_velocityThreshold)
			{
				ccp->velocityBias = -cc->restitution * vRel;
			}
		}

		// If we have two points, then prepare the block solver.
		if (cc->pointCount == 2)
		{
			b2ContactConstraintPoint* ccp1 = cc->points + 0;
			b2ContactConstraintPoint* ccp2 = cc->points + 1;
			
			float32 invMassA = bodyA->m_invMass;
			float32 invIA = bodyA->m_invI;
			float32 invMassB = bodyB->m_invMass;
			float32 invIB = bodyB->m_invI;

			float32 rn1A = b2Cross(ccp1->rA, cc->normal);
			float32 rn1B = b2Cross(ccp1->rB, cc->normal);
			float32 rn2A = b2Cross(ccp2->rA, cc->normal);
			float32 rn2B = b2Cross(ccp2->rB, cc->normal);

			float32 k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
			float32 k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
			float32 k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const float32 k_maxConditionNumber = 100.0f;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				cc->K.col1.Set(k11, k12);
				cc->K.col2.Set(k12, k22);
				cc->normalMass = cc->K.GetInverse();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				cc->pointCount = 1;
			}
		}
	}
}

b2ContactSolver::~b2ContactSolver()
{
	m_allocator->Free(m_constraints);
}

void b2ContactSolver::InitVelocityConstraints(const b2TimeStep& step)
{
	// Warm start.
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;

		b2Body* bodyA = c->bodyA;
		b2Body* bodyB = c->bodyB;
		float32 invMassA = bodyA->m_invMass;
		float32 invIA = bodyA->m_invI;
		float32 invMassB = bodyB->m_invMass;
		float32 invIB = bodyB->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);

		if (step.warmStarting)
		{
			for (int32 j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				ccp->normalImpulse *= step.dtRatio;
				ccp->tangentImpulse *= step.dtRatio;
				b2Vec2 P = ccp->normalImpulse * normal + ccp->tangentImpulse * tangent;
				bodyA->m_angularVelocity -= invIA * b2Cross(ccp->rA, P);
				bodyA->m_linearVelocity -= invMassA * P;
				bodyB->m_angularVelocity += invIB * b2Cross(ccp->rB, P);
				bodyB->m_linearVelocity += invMassB * P;
			}
		}
		else
		{
			for (int32 j = 0; j < c->pointCount; ++j)
			{
				b2ContactConstraintPoint* ccp = c->points + j;
				ccp->normalImpulse = 0.0f;
				ccp->tangentImpulse = 0.0f;
			}
		}
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* bodyA = c->bodyA;
		b2Body* bodyB = c->bodyB;
		float32 wA = bodyA->m_angularVelocity;
		float32 wB = bodyB->m_angularVelocity;
		b2Vec2 vA = bodyA->m_linearVelocity;
		b2Vec2 vB = bodyB->m_linearVelocity;
		float32 invMassA = bodyA->m_invMass;
		float32 invIA = bodyA->m_invI;
		float32 invMassB = bodyB->m_invMass;
		float32 invIB = bodyB->m_invI;
		b2Vec2 normal = c->normal;
		b2Vec2 tangent = b2Cross(normal, 1.0f);
		float32 friction = c->friction;

		b2Assert(c->pointCount == 1 || c->pointCount == 2);

		// Solve tangent constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			// Relative velocity at contact
			b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);

			// Compute tangent force
			float32 vt = b2Dot(dv, tangent);
			float32 lambda = ccp->tangentMass * (-vt);

			// b2Clamp the accumulated force
			float32 maxFriction = friction * ccp->normalImpulse;
			float32 newImpulse = b2Clamp(ccp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - ccp->tangentImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * tangent;

			vA -= invMassA * P;
			wA -= invIA * b2Cross(ccp->rA, P);

			vB += invMassB * P;
			wB += invIB * b2Cross(ccp->rB, P);

			ccp->tangentImpulse = newImpulse;
		}

		// Solve normal constraints
		if (c->pointCount == 1)
		{
			b2ContactConstraintPoint* ccp = c->points + 0;

			// Relative velocity at contact
			b2Vec2 dv = vB + b2Cross(wB, ccp->rB) - vA - b2Cross(wA, ccp->rA);

			// Compute normal impulse
			float32 vn = b2Dot(dv, normal);
			float32 lambda = -ccp->normalMass * (vn - ccp->velocityBias);

			// b2Clamp the accumulated impulse
			float32 newImpulse = b2Max(ccp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - ccp->normalImpulse;

			// Apply contact impulse
			b2Vec2 P = lambda * normal;
			vA -= invMassA * P;
			wA -= invIA * b2Cross(ccp->rA, P);

			vB += invMassB * P;
			wB += invIB * b2Cross(ccp->rB, P);
			ccp->normalImpulse = newImpulse;
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
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
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

			b2ContactConstraintPoint* cp1 = c->points + 0;
			b2ContactConstraintPoint* cp2 = c->points + 1;

			b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
			b2Assert(a.x >= 0.0f && a.y >= 0.0f);

			// Relative velocity at contact
			b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
			b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

			// Compute normal velocity
			float32 vn1 = b2Dot(dv1, normal);
			float32 vn2 = b2Dot(dv2, normal);

			b2Vec2 b;
			b.x = vn1 - cp1->velocityBias;
			b.y = vn2 - cp2->velocityBias;
			b -= b2Mul(c->K, a);

			const float32 k_errorTol = 1e-3f;
			B2_NOT_USED(k_errorTol);

			for (;;)
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
				b2Vec2 x = - b2Mul(c->normalMass, b);

				if (x.x >= 0.0f && x.y >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= invMassA * (P1 + P2);
					wA -= invIA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += invMassB * (P1 + P2);
					wB += invIB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1' + a12 * 0 + b1' 
				// vn2 = a21 * x1' + a22 * 0 + b2'
				//
				x.x = - cp1->normalMass * b.x;
				x.y = 0.0f;
				vn1 = 0.0f;
				vn2 = c->K.col1.y * x.x + b.y;

				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= invMassA * (P1 + P2);
					wA -= invIA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += invMassB * (P1 + P2);
					wB += invIB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}


				//
				// Case 3: wB = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2' + b1' 
				//   0 = a21 * 0 + a22 * x2' + b2'
				//
				x.x = 0.0f;
				x.y = - cp2->normalMass * b.y;
				vn1 = c->K.col2.x * x.y + b.x;
				vn2 = 0.0f;

				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= invMassA * (P1 + P2);
					wA -= invIA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += invMassB * (P1 + P2);
					wB += invIB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				// 
				// vn1 = b1
				// vn2 = b2;
				x.x = 0.0f;
				x.y = 0.0f;
				vn1 = b.x;
				vn2 = b.y;

				if (vn1 >= 0.0f && vn2 >= 0.0f )
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = x - a;

					// Apply incremental impulse
					b2Vec2 P1 = d.x * normal;
					b2Vec2 P2 = d.y * normal;
					vA -= invMassA * (P1 + P2);
					wA -= invIA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));

					vB += invMassB * (P1 + P2);
					wB += invIB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		bodyA->m_linearVelocity = vA;
		bodyA->m_angularVelocity = wA;
		bodyB->m_linearVelocity = vB;
		bodyB->m_angularVelocity = wB;
	}
}

void b2ContactSolver::FinalizeVelocityConstraints()
{
	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Manifold* m = c->manifold;

		for (int32 j = 0; j < c->pointCount; ++j)
		{
			m->m_points[j].m_normalImpulse = c->points[j].normalImpulse;
			m->m_points[j].m_tangentImpulse = c->points[j].tangentImpulse;
		}
	}
}

#if 0
// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints(float32 baumgarte)
{
	float32 minSeparation = 0.0f;

	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* bodyA = c->bodyA;
		b2Body* bodyB = c->bodyB;
		float32 invMassA = bodyA->m_mass * bodyA->m_invMass;
		float32 invIA = bodyA->m_mass * bodyA->m_invI;
		float32 invMassB = bodyB->m_mass * bodyB->m_invMass;
		float32 invIB = bodyB->m_mass * bodyB->m_invI;

		b2Vec2 normal = c->normal;

		// Solve normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 r1 = b2Mul(bodyA->GetXForm().R, ccp->localAnchorA - bodyA->GetLocalCenter());
			b2Vec2 r2 = b2Mul(bodyB->GetXForm().R, ccp->localAnchorB - bodyB->GetLocalCenter());

			b2Vec2 p1 = bodyA->m_sweep.c + r1;
			b2Vec2 p2 = bodyB->m_sweep.c + r2;
			b2Vec2 dp = p2 - p1;

			// Approximate the current separation.
			float32 separation = b2Dot(dp, normal) + ccp->separation;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = baumgarte * b2Clamp(separation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);

			// Compute normal impulse
			float32 impulse = -ccp->equalizedMass * C;

			b2Vec2 P = impulse * normal;

			bodyA->m_sweep.c -= invMassA * P;
			bodyA->m_sweep.a -= invIA * b2Cross(r1, P);
			bodyA->SynchronizeTransform();

			bodyB->m_sweep.c += invMassB * P;
			bodyB->m_sweep.a += invIB * b2Cross(r2, P);
			bodyB->SynchronizeTransform();
		}
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5f * b2_linearSlop;
}

#elif 1

struct b2PositionSolverManifold
{
	void Initialize(b2ContactConstraint* cc)
	{
		b2Assert(cc->pointCount > 0);

		switch (cc->type)
		{
		case b2Manifold::e_circles:
			{
				b2Vec2 pointA = cc->bodyA->GetWorldPoint(cc->localPoint);
				b2Vec2 pointB = cc->bodyB->GetWorldPoint(cc->points[0].localPoint);
				if (b2DistanceSquared(pointA, pointB) > B2_FLT_EPSILON * B2_FLT_EPSILON)
				{
					m_normal = pointB - pointA;
					m_normal.Normalize();
				}
				else
				{
					m_normal.Set(1.0f, 0.0f);
				}

				m_points[0] = 0.5f * (pointA + pointB);
				m_separations[0] = b2Dot(pointB - pointA, m_normal) - cc->radius;
			}
			break;

		case b2Manifold::e_faceA:
			{
				m_normal = cc->bodyA->GetWorldVector(cc->localPlaneNormal);
				b2Vec2 planePoint = cc->bodyA->GetWorldPoint(cc->localPoint);

				for (int32 i = 0; i < cc->pointCount; ++i)
				{
					b2Vec2 clipPoint = cc->bodyB->GetWorldPoint(cc->points[i].localPoint);
					m_separations[i] = b2Dot(clipPoint - planePoint, m_normal) - cc->radius;
					m_points[i] = clipPoint;
				}
			}
			break;

		case b2Manifold::e_faceB:
			{
				m_normal = cc->bodyB->GetWorldVector(cc->localPlaneNormal);
				b2Vec2 planePoint = cc->bodyB->GetWorldPoint(cc->localPoint);

				for (int32 i = 0; i < cc->pointCount; ++i)
				{
					b2Vec2 clipPoint = cc->bodyA->GetWorldPoint(cc->points[i].localPoint);
					m_separations[i] = b2Dot(clipPoint - planePoint, m_normal) - cc->radius;
					m_points[i] = clipPoint;
				}

				// Ensure normal points from A to B
				m_normal = -m_normal;
			}
			break;
		}
	}

	b2Vec2 m_normal;
	b2Vec2 m_points[b2_maxManifoldPoints];
	float32 m_separations[b2_maxManifoldPoints];
};

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints(float32 baumgarte)
{
	float32 minSeparation = 0.0f;

	for (int32 i = 0; i < m_constraintCount; ++i)
	{
		b2ContactConstraint* c = m_constraints + i;
		b2Body* bodyA = c->bodyA;
		b2Body* bodyB = c->bodyB;

		float32 invMassA = bodyA->m_mass * bodyA->m_invMass;
		float32 invIA = bodyA->m_mass * bodyA->m_invI;
		float32 invMassB = bodyB->m_mass * bodyB->m_invMass;
		float32 invIB = bodyB->m_mass * bodyB->m_invI;

		b2PositionSolverManifold psm;
		psm.Initialize(c);
		b2Vec2 normal = psm.m_normal;

		// Solve normal constraints
		for (int32 j = 0; j < c->pointCount; ++j)
		{
			b2ContactConstraintPoint* ccp = c->points + j;

			b2Vec2 point = psm.m_points[j];
			float32 separation = psm.m_separations[j];

			b2Vec2 rA = point - bodyA->m_sweep.c;
			b2Vec2 rB = point - bodyB->m_sweep.c;

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = baumgarte * b2Clamp(separation + b2_linearSlop, -b2_maxLinearCorrection, 0.0f);

			// Compute normal impulse
			float32 impulse = -ccp->equalizedMass * C;

			b2Vec2 P = impulse * normal;

			bodyA->m_sweep.c -= invMassA * P;
			bodyA->m_sweep.a -= invIA * b2Cross(rA, P);
			bodyA->SynchronizeTransform();

			bodyB->m_sweep.c += invMassB * P;
			bodyB->m_sweep.a += invIB * b2Cross(rB, P);
			bodyB->SynchronizeTransform();
		}
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5f * b2_linearSlop;
}

#endif