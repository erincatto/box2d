// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

typedef struct MotionLockScene
{
	b2WorldId worldId;
	b2BodyId groundId;
	b2BodyId bodyId;
	b2JointId jointId;
} MotionLockScene;

static const b2Vec2 lockOrigin = { 1.0f, 4.0f };

// A box under gravity locked to the ground. An anchor at the center of mass takes the decoupled path.
static MotionLockScene CreateMotionLockScene( b2MotionLockJointDef def, b2Vec2 localAnchor )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };

	MotionLockScene s;
	s.worldId = b2CreateWorld( &worldDef );

	b2BodyDef groundDef = b2DefaultBodyDef();
	s.groundId = b2CreateBody( s.worldId, &groundDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2ToPos( lockOrigin );
	bodyDef.enableSleep = false;
	s.bodyId = b2CreateBody( s.worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.25f );
	b2CreatePolygonShape( s.bodyId, &shapeDef, &box );

	def.base.bodyIdA = s.groundId;
	def.base.bodyIdB = s.bodyId;
	def.base.localFrameA.p = b2Add( lockOrigin, localAnchor );
	def.base.localFrameB.p = localAnchor;
	s.jointId = b2CreateMotionLockJoint( s.worldId, &def );
	return s;
}

static void StepScene( b2WorldId worldId, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
}

static int MotionLockJointApi( void )
{
	b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
	def.lockLinearY = true;
	MotionLockScene s = CreateMotionLockScene( def, b2Vec2_zero );

	ENSURE( b2Joint_GetType( s.jointId ) == b2_motionLockJoint );
	ENSURE( b2MotionLockJoint_IsLinearLockEnabled( s.jointId, 0 ) == false );
	ENSURE( b2MotionLockJoint_IsLinearLockEnabled( s.jointId, 1 ) );

	b2MotionLockJoint_EnableLinearLock( s.jointId, 0, true );
	ENSURE( b2MotionLockJoint_IsLinearLockEnabled( s.jointId, 0 ) );

	b2MotionLockJoint_EnableLinearLock( s.jointId, 1, false );
	ENSURE( b2MotionLockJoint_IsLinearLockEnabled( s.jointId, 1 ) == false );

	StepScene( s.worldId, 10 );

	b2DestroyJoint( s.jointId );
	StepScene( s.worldId, 1 );

	b2DestroyWorld( s.worldId );
	return 0;
}

// Every combination of linear locks holds its axes against gravity and a push while the free axes move.
static int MotionLockJointLinear( void )
{
	b2Vec2 push = { 2.0f, 3.0f };

	for ( int mask = 1; mask < 4; ++mask )
	{
		b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
		def.lockLinearX = ( mask & 1 ) != 0;
		def.lockLinearY = ( mask & 2 ) != 0;

		MotionLockScene s = CreateMotionLockScene( def, b2Vec2_zero );
		b2Body_SetLinearVelocity( s.bodyId, push );
		StepScene( s.worldId, 60 );

		b2Vec2 delta = b2SubPos( b2Body_GetPosition( s.bodyId ), b2ToPos( lockOrigin ) );

		ENSURE( def.lockLinearX ? b2AbsFloat( delta.x ) < 0.01f : b2AbsFloat( delta.x ) > 1.0f );
		ENSURE( def.lockLinearY ? b2AbsFloat( delta.y ) < 0.01f : b2AbsFloat( delta.y ) > 1.0f );
		ENSURE( b2Joint_GetLinearSeparation( s.jointId ) < 0.01f );

		b2DestroyWorld( s.worldId );
	}

	return 0;
}

// An anchor off the center of mass couples the rows. The point lock becomes a pendulum pivot.
static int MotionLockJointCoupled( void )
{
	b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
	def.lockLinearX = true;
	def.lockLinearY = true;

	MotionLockScene s = CreateMotionLockScene( def, (b2Vec2){ 0.5f, 0.25f } );
	StepScene( s.worldId, 120 );

	ENSURE( b2Joint_GetLinearSeparation( s.jointId ) < 0.01f );
	ENSURE( b2IsValidVec2( b2Body_GetLinearVelocity( s.bodyId ) ) );
	ENSURE( b2AbsFloat( b2Body_GetAngularVelocity( s.bodyId ) ) > 0.1f );

	b2MotionLockJoint_EnableLinearLock( s.jointId, 1, false );
	StepScene( s.worldId, 60 );
	ENSURE( b2Joint_GetLinearSeparation( s.jointId ) < 0.01f );

	b2DestroyWorld( s.worldId );
	return 0;
}

// A heavy circle resting on a light platform held by a point lock and fixed rotation.
// The lock rows iterate with the contact, so the platform supports the circle like a static body.
static int MotionLockJointPlatform( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -20.0f };
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );

	b2BodyDef platformDef = b2DefaultBodyDef();
	platformDef.type = b2_dynamicBody;
	platformDef.enableSleep = false;
	platformDef.fixedRotation = true;
	b2BodyId platformId = b2CreateBody( worldId, &platformDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 2.0f, 0.05f );
	b2CreatePolygonShape( platformId, &shapeDef, &box );

	b2MassData massData = b2Body_GetMassData( platformId );
	massData.rotationalInertia = massData.rotationalInertia / massData.mass;
	massData.mass = 1.0f;
	b2Body_SetMassData( platformId, massData );

	b2MotionLockJointDef lockDef = b2DefaultMotionLockJointDef();
	lockDef.base.bodyIdA = groundId;
	lockDef.base.bodyIdB = platformId;
	lockDef.lockLinearX = true;
	lockDef.lockLinearY = true;
	b2CreateMotionLockJoint( worldId, &lockDef );

	b2BodyDef circleDef = b2DefaultBodyDef();
	circleDef.type = b2_dynamicBody;
	circleDef.position = (b2Pos){ 0.0f, 0.55f };
	circleDef.enableSleep = false;
	b2BodyId circleId = b2CreateBody( worldId, &circleDef );

	b2Circle circle = { b2Vec2_zero, 0.5f };
	b2CreateCircleShape( circleId, &shapeDef, &circle );

	massData = b2Body_GetMassData( circleId );
	massData.rotationalInertia = 10.0f * massData.rotationalInertia / massData.mass;
	massData.mass = 10.0f;
	b2Body_SetMassData( circleId, massData );

	for ( int i = 0; i < 200; ++i )
	{
		b2World_Step( worldId, 1.0f / 50.0f, 4 );
	}

	float gap = (float)b2Body_GetPosition( circleId ).y - 0.55f;
	ENSURE( b2AbsFloat( gap ) < 0.02f );
	ENSURE( b2Body_GetRotation( platformId ).s == 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

// A revolute pivot off the center of mass plus a point lock at the center is redundant. The two
// constraints share the load through iteration instead of forming one singular block.
static int MotionLockJointRedundant( void )
{
	b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
	def.lockLinearX = true;
	def.lockLinearY = true;

	MotionLockScene s = CreateMotionLockScene( def, b2Vec2_zero );

	b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
	revoluteDef.base.bodyIdA = s.groundId;
	revoluteDef.base.bodyIdB = s.bodyId;
	revoluteDef.base.localFrameA.p = (b2Vec2){ lockOrigin.x - 0.5f, lockOrigin.y };
	revoluteDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
	b2CreateRevoluteJoint( s.worldId, &revoluteDef );

	StepScene( s.worldId, 120 );

	ENSURE( b2Length( b2Body_GetLinearVelocity( s.bodyId ) ) < 0.1f );
	ENSURE( b2AbsFloat( b2Body_GetAngularVelocity( s.bodyId ) ) < 0.1f );
	ENSURE( b2Joint_GetLinearSeparation( s.jointId ) < 0.01f );

	b2DestroyWorld( s.worldId );
	return 0;
}

// A box held still under gravity. The force at the anchor carries the weight.
static int MotionLockJointForce( void )
{
	b2Rot frameRotation = b2MakeRot( 0.5f );

	for ( int i = 0; i < 2; ++i )
	{
		b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
		def.lockLinearX = true;
		def.lockLinearY = true;
		def.base.localFrameA.q = frameRotation;
		def.base.localFrameB.q = frameRotation;

		b2Vec2 anchor = i == 0 ? b2Vec2_zero : (b2Vec2){ 0.0f, 0.5f };
		MotionLockScene s = CreateMotionLockScene( def, anchor );
		StepScene( s.worldId, 120 );

		float weight = 10.0f * b2Body_GetMass( s.bodyId );
		b2Vec2 force = b2Joint_GetConstraintForce( s.jointId );
		ENSURE_SMALL( force.x, 0.02f * weight );
		ENSURE_SMALL( force.y - weight, 0.02f * weight );
		ENSURE( b2Joint_GetConstraintTorque( s.jointId ) == 0.0f );

		b2DestroyWorld( s.worldId );
	}

	return 0;
}

#if defined( BOX2D_DOUBLE_PRECISION )
// Ground center at the origin with the lock 1e7 m away. The separation must stay in double precision, a
// float difference of the two centers rounds to 1 m and lets the body sag under gravity. Far from the
// origin the lock must hold as well as it does near the origin.
static float RunFarMotionLock( b2Vec2 localAnchorB, bool lockY, double far )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ far - localAnchorB.x, far - localAnchorB.y };
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	b2MotionLockJointDef def = b2DefaultMotionLockJointDef();
	def.base.bodyIdA = groundId;
	def.base.bodyIdB = bodyId;
	def.base.localFrameA.p = (b2Vec2){ (float)far, (float)far };
	def.base.localFrameB.p = localAnchorB;
	def.lockLinearX = true;
	def.lockLinearY = lockY;
	b2JointId jointId = b2CreateMotionLockJoint( worldId, &def );

	float maxSeparation = 0.0f;
	for ( int i = 0; i < 240; ++i )
	{
		float push = 20.0f * sinf( 0.05f * i );
		b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ push, 0.0f }, true );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		maxSeparation = b2MaxFloat( maxSeparation, b2Joint_GetLinearSeparation( jointId ) );
	}

	b2DestroyWorld( worldId );
	return maxSeparation;
}

static int MotionLockJointFar( void )
{
	b2Vec2 anchors[2] = { b2Vec2_zero, { 0.5f, 0.5f } };
	for ( int i = 0; i < 2; ++i )
	{
		float nearSeparation = RunFarMotionLock( anchors[i], i == 1, 0.0 );
		float farSeparation = RunFarMotionLock( anchors[i], i == 1, 1.0e7 );
		ENSURE( farSeparation < 0.01f );
		ENSURE_SMALL( farSeparation - nearSeparation, 1.0e-4f );
	}

	return 0;
}
#endif

// Fixed rotation zeroes only the inverse inertia. The caller's inertia survives so unlocking restores it
// without recomputing mass from the shapes.
static int FixedRotationMassData( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.fixedRotation = true;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsFixedRotation( bodyId ) );

	b2MassData massData = { 5.0f, { 0.0f, 0.0f }, 2.0f };
	b2Body_SetMassData( bodyId, massData );

	ENSURE_SMALL( b2Body_GetMass( bodyId ) - 5.0f, 1e-6f );
	ENSURE_SMALL( b2Body_GetRotationalInertia( bodyId ) - 2.0f, 1e-6f );
	ENSURE_SMALL( b2Body_GetMassData( bodyId ).rotationalInertia - 2.0f, 1e-6f );

	b2Body_ApplyAngularImpulse( bodyId, 1.0f, true );
	ENSURE( b2Body_GetAngularVelocity( bodyId ) == 0.0f );

	b2Body_SetFixedRotation( bodyId, false );
	ENSURE( b2Body_IsFixedRotation( bodyId ) == false );
	ENSURE_SMALL( b2Body_GetRotationalInertia( bodyId ) - 2.0f, 1e-6f );

	b2Body_ApplyAngularImpulse( bodyId, 1.0f, true );
	ENSURE_SMALL( b2Body_GetAngularVelocity( bodyId ) - 0.5f, 1e-6f );

	b2Body_SetFixedRotation( bodyId, true );
	ENSURE( b2Body_IsFixedRotation( bodyId ) );
	ENSURE( b2Body_GetAngularVelocity( bodyId ) == 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

// Zero inverse inertia only stops impulses from changing the spin, so every path that writes angular
// velocity directly has to drop it or the body spins forever.
static int FixedRotationRejectsSpin( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeSquare( 0.5f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.fixedRotation = true;
	bodyDef.angularVelocity = 5.0f;
	b2BodyId dynamicId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( dynamicId, &shapeDef, &box );

	ENSURE( b2Body_GetAngularVelocity( dynamicId ) == 0.0f );

	b2Body_SetAngularVelocity( dynamicId, 3.0f );
	ENSURE( b2Body_GetAngularVelocity( dynamicId ) == 0.0f );

	bodyDef.type = b2_kinematicBody;
	bodyDef.angularVelocity = 0.0f;
	bodyDef.position = (b2Pos){ 4.0f, 0.0f };
	b2BodyId kinematicId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( kinematicId, &shapeDef, &box );

	b2WorldTransform target = { { 4.0f, 1.0f }, b2MakeRot( 1.0f ) };
	b2Body_SetTargetTransform( kinematicId, target, 1.0f / 60.0f, true );
	ENSURE( b2Body_GetAngularVelocity( kinematicId ) == 0.0f );
	ENSURE( b2Length( b2Body_GetLinearVelocity( kinematicId ) ) > 0.0f );

	b2Body_SetType( kinematicId, b2_dynamicBody );
	ENSURE( b2Body_GetAngularVelocity( kinematicId ) == 0.0f );

	StepScene( worldId, 10 );

	ENSURE( b2Body_GetRotation( dynamicId ).s == 0.0f );
	ENSURE( b2Body_GetRotation( kinematicId ).s == 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

// The body cannot turn toward the target rotation, so the target center must use the current one
static int FixedRotationTargetTransform( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.fixedRotation = true;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeOffsetBox( 0.5f, 0.5f, (b2Vec2){ 1.0f, 0.0f }, b2Rot_identity );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	b2WorldTransform target = { { 2.0f, 3.0f }, b2MakeRot( 0.5f * B2_PI ) };
	b2Body_SetTargetTransform( bodyId, target, 1.0f / 60.0f, true );
	StepScene( worldId, 1 );

	b2Pos p = b2Body_GetPosition( bodyId );
	ENSURE_SMALL( p.x - 2.0f, 1.0e-4f );
	ENSURE_SMALL( p.y - 3.0f, 1.0e-4f );
	ENSURE( b2Body_GetRotation( bodyId ).s == 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

int JointTest( void )
{
	RUN_SUBTEST( MotionLockJointApi );
	RUN_SUBTEST( MotionLockJointLinear );
	RUN_SUBTEST( MotionLockJointCoupled );
	RUN_SUBTEST( MotionLockJointPlatform );
	RUN_SUBTEST( MotionLockJointRedundant );
	RUN_SUBTEST( MotionLockJointForce );
#if defined( BOX2D_DOUBLE_PRECISION )
	RUN_SUBTEST( MotionLockJointFar );
#endif
	RUN_SUBTEST( FixedRotationMassData );
	RUN_SUBTEST( FixedRotationRejectsSpin );
	RUN_SUBTEST( FixedRotationTargetTransform );

	return 0;
}
