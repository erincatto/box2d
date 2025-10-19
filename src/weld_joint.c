// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "core.h"
#include "joint.h"
#include "physics_world.h"
#include "solver.h"
#include "solver_set.h"

// needed for dll export
#include "box2d/box2d.h"

#define B2_WELD_BLOCK_SOLVE 0

#if B2_WELD_BLOCK_SOLVE
typedef struct
{
	float x, y, z;
} b2Vec3;

// A 3-by-3 matrix. Stored in column-major order.
typedef struct
{
	b2Vec3 cx, cy, cz;
} b2Mat33;

static inline float b2Dot3( b2Vec3 a, b2Vec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline b2Vec3 b2Cross3( b2Vec3 a, b2Vec3 b )
{
	return (b2Vec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

static inline b2Vec3 b2Solve33(const b2Mat33* m, b2Vec3 b )
{
	float det = b2Dot3( m->cx, b2Cross3( m->cy, m->cz ) );
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

	b2Vec3 x;
	x.x = det * b2Dot3( b, b2Cross3( m->cy, m->cz ) );
	x.y = det * b2Dot3( m->cx, b2Cross3( b, m->cz ) );
	x.z = det * b2Dot3( m->cx, b2Cross3( m->cy, b ) );
	return x;
}
#endif

void b2WeldJoint_SetLinearHertz( b2JointId jointId, float hertz )
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearHertz = hertz;
}

float b2WeldJoint_GetLinearHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearHertz;
}

void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float dampingRatio )
{
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearDampingRatio = dampingRatio;
}

float b2WeldJoint_GetLinearDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearDampingRatio;
}

void b2WeldJoint_SetAngularHertz( b2JointId jointId, float hertz )
{
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularHertz = hertz;
}

float b2WeldJoint_GetAngularHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularHertz;
}

void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float dampingRatio )
{
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularDampingRatio = dampingRatio;
}

float b2WeldJoint_GetAngularDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularDampingRatio;
}

b2Vec2 b2GetWeldJointForce( b2World* world, b2JointSim* base )
{
	b2Vec2 force = b2MulSV( world->inv_h, base->weldJoint.linearImpulse );
	return force;
}

float b2GetWeldJointTorque( b2World* world, b2JointSim* base )
{
	return world->inv_h * base->weldJoint.angularImpulse;
}

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-E -r1_skew E r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

// 3x3 Block
// K = [J1] * invM * [J1T J2T]
//     [J2]
//   = [J1] * [invM * J1T invM * J2T]
//     [J2]
//   = [J1 * invM * J1T J1 * invM * J2T]
//     [J2 * invM * J1T J2 * invM * J2T]

void b2PrepareWeldJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_weldJoint );

	// chase body id to the solver set where the body lives
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2BodyArray_Get( &world->bodies, idA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setA = b2SolverSetArray_Get( &world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2SolverSetArray_Get( &world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2BodySimArray_Get( &setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2BodySimArray_Get( &setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2WeldJoint* joint = &base->weldJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	float ka = iA + iB;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if ( joint->linearHertz == 0.0f )
	{
		joint->linearSpring = base->constraintSoftness;
	}
	else
	{
		joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	}

	if ( joint->angularHertz == 0.0f )
	{
		joint->angularSpring = base->constraintSoftness;
	}
	else
	{
		joint->angularSpring = b2MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );
	}

	if ( context->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartWeldJoint( b2JointSim* base, b2StepContext* context )
{
	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
		stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + joint->angularImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
		stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + joint->angularImpulse );
	}
}

void b2SolveWeldJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_weldJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// Block solve doesn't work correctly with mixed stiffness values
#if B2_WELD_BLOCK_SOLVE
	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Mat33 K;
	K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.cz.x = -rA.y * iA - rB.y * iB;
	K.cx.y = K.cy.x;
	K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.cz.y = rA.x * iA + rB.x * iB;
	K.cx.z = K.cz.x;
	K.cy.z = K.cz.y;
	K.cz.z = iA + iB;

	b2Vec3 bias = {0.0f, 0.0f, 0.0f};
	float linearMassScale = 1.0f;
	float linearImpulseScale = 0.0f;
	if ( useBias || joint->linearHertz > 0.0f )
	{
		// linear
		b2Vec2 dcA = stateA->deltaPosition;
		b2Vec2 dcB = stateB->deltaPosition;
		b2Vec2 jointTranslation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

		bias.x = joint->linearSpring.biasRate * jointTranslation.x;
		bias.y = joint->linearSpring.biasRate * jointTranslation.y;
		
		linearMassScale = joint->linearSpring.massScale;
		linearImpulseScale = joint->linearSpring.impulseScale;
	}

	float angularMassScale = 1.0f;
	float angularImpulseScale = 0.0f;
	if ( useBias || joint->angularHertz > 0.0f )
	{
		// angular
		b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		b2Rot relQ = b2InvMulRot( qA, qB );
		float jointAngle = b2Rot_GetAngle( relQ );

		bias.z = joint->angularSpring.biasRate * jointAngle;

		angularMassScale = joint->angularSpring.massScale;
		angularImpulseScale = joint->angularSpring.impulseScale;
	}

	b2Vec2 Cdot1 = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float Cdot2 = wB - wA;

	b2Vec3 Cdot = {Cdot1.x + bias.x, Cdot1.y + bias.y, Cdot2 + bias.z};

	b2Vec3 b = b2Solve33( &K, Cdot );

	b2Vec2 linearImpulse = {
		-linearMassScale * b.x - linearImpulseScale * joint->linearImpulse.x,
		-linearMassScale * b.y - linearImpulseScale * joint->linearImpulse.y,
	};
	joint->linearImpulse = b2Add( joint->linearImpulse, linearImpulse );

	float angularImpulse = -angularMassScale * b.z - angularImpulseScale * joint->angularImpulse;
	joint->angularImpulse += angularImpulse;

	vA = b2MulSub( vA, mA, linearImpulse );
	wA -= iA * (b2Cross( rA, linearImpulse ) + angularImpulse);
	vB = b2MulAdd( vB, mB, linearImpulse );
	wB += iB * (b2Cross( rB, linearImpulse ) + angularImpulse);

	// todo debugging
	Cdot1 = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	Cdot2 = wB - wA;

	if ( useBias == false && b2Length(Cdot1) > 0.0001f )
	{
		Cdot1.x += 0.0f;
	}

	if ( useBias == false && b2AbsFloat( Cdot2 ) > 0.0001f )
	{
		Cdot2 += 0.0f;
	}

#else

	// angular constraint
	{
		b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		b2Rot relQ = b2InvMulRot( qA, qB );
		float jointAngle = b2Rot_GetAngle( relQ );

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias || joint->angularHertz > 0.0f )
		{
			float C = jointAngle;
			bias = joint->angularSpring.biasRate * C;
			massScale = joint->angularSpring.massScale;
			impulseScale = joint->angularSpring.impulseScale;
		}

		float Cdot = wB - wA;
		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// linear constraint
	{
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias || joint->linearHertz > 0.0f )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;
			b2Vec2 C = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

			bias = b2MulSV( joint->linearSpring.biasRate, C );
			massScale = joint->linearSpring.massScale;
			impulseScale = joint->linearSpring.impulseScale;
		}

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		b2Vec2 impulse = {
			-massScale * b.x - impulseScale * joint->linearImpulse.x,
			-massScale * b.y - impulseScale * joint->linearImpulse.y,
		};

		joint->linearImpulse = b2Add( joint->linearImpulse, impulse );

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
	}
#endif

	B2_ASSERT( b2IsValidVec2( vA ) );
	B2_ASSERT( b2IsValidFloat( wA ) );
	B2_ASSERT( b2IsValidVec2( vB ) );
	B2_ASSERT( b2IsValidFloat( wB ) );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

#if 0
void b2DumpWeldJoint()
{
	int32 indexA = bodyA->islandIndex;
	int32 indexB = bodyB->islandIndex;

	b2Dump("  b2WeldJointDef jd;\n");
	b2Dump("  jd.bodyA = sims[%d];\n", indexA);
	b2Dump("  jd.bodyB = sims[%d];\n", indexB);
	b2Dump("  jd.collideConnected = bool(%d);\n", collideConnected);
	b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", localAnchorA.x, localAnchorA.y);
	b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", localAnchorB.x, localAnchorB.y);
	b2Dump("  jd.referenceAngle = %.9g;\n", referenceAngle);
	b2Dump("  jd.stiffness = %.9g;\n", stiffness);
	b2Dump("  jd.damping = %.9g;\n", damping);
	b2Dump("  joints[%d] = world->CreateJoint(&jd);\n", index);
}
#endif

void b2DrawWeldJoint( b2DebugDraw* draw, b2JointSim* base, b2Transform transformA, b2Transform transformB, float drawScale )
{
	B2_ASSERT( base->type == b2_weldJoint );

	b2Transform frameA = b2MulTransforms( transformA, base->localFrameA );
	b2Transform frameB = b2MulTransforms( transformB, base->localFrameB );

	b2Polygon box = b2MakeBox( 0.25f * drawScale, 0.125f * drawScale );

	b2Vec2 points[4];

	for ( int i = 0; i < 4; ++i )
	{
		points[i] = b2TransformPoint( frameA, box.vertices[i] );
	}
	draw->DrawPolygonFcn( points, 4, b2_colorDarkOrange, draw->context );

	for ( int i = 0; i < 4; ++i )
	{
		points[i] = b2TransformPoint( frameB, box.vertices[i] );
	}

	draw->DrawPolygonFcn( points, 4, b2_colorDarkCyan, draw->context );
}
