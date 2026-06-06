// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "test_macros.h"

#include "physics_world.h"
#include "world_snapshot.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* s_recPath = "recording_test.b2rec";

// Query callbacks used by RecordingTest
static int s_overlapCount = 0;
static bool s_overlapFcn( b2ShapeId id, void* ctx )
{
	(void)id;
	(void)ctx;
	s_overlapCount++;
	return true;
}

static float s_closestCastFcn( b2ShapeId id, b2Vec2 point, b2Vec2 normal, float fraction, void* ctx )
{
	(void)id; (void)point; (void)normal; (void)ctx;
	return fraction;
}

static float s_allHitsCastFcn( b2ShapeId id, b2Vec2 point, b2Vec2 normal, float fraction, void* ctx )
{
	(void)id; (void)point; (void)normal; (void)ctx;
	return fraction;
}

static bool s_planeFcn( b2ShapeId id, const b2PlaneResult* plane, void* ctx )
{
	(void)id; (void)plane; (void)ctx;
	return true;
}

// No-op draw callbacks for the headless draw-path exercise
static void s_DrawLine( b2Vec2 p1, b2Vec2 p2, b2HexColor c, void* ctx ) { (void)p1; (void)p2; (void)c; (void)ctx; }
static void s_DrawPoint( b2Vec2 p, float sz, b2HexColor c, void* ctx ) { (void)p; (void)sz; (void)c; (void)ctx; }
static void s_DrawPoly( const b2Vec2* v, int n, b2HexColor c, void* ctx ) { (void)v; (void)n; (void)c; (void)ctx; }
static void s_DrawCapsule( b2Vec2 p1, b2Vec2 p2, float r, b2HexColor c, void* ctx ) { (void)p1; (void)p2; (void)r; (void)c; (void)ctx; }

// Issue all 9 spatial query types against worldId. groundShapeId and a known position
// are used for the shape-level queries.
static void IssueAllQueries( b2WorldId worldId, b2ShapeId groundShapeId )
{
	b2QueryFilter filter = b2DefaultQueryFilter();

	// OverlapAABB
	b2AABB aabb = { { -5.0f, -15.0f }, { 5.0f, 5.0f } };
	b2World_OverlapAABB( worldId, aabb, filter, s_overlapFcn, NULL );

	// OverlapShape (small box proxy)
	b2ShapeProxy proxy = b2MakeProxy( (b2Vec2[]){ { -0.5f, -0.5f }, { 0.5f, -0.5f }, { 0.5f, 0.5f }, { -0.5f, 0.5f } }, 4, 0.0f );
	b2World_OverlapShape( worldId, &proxy, filter, s_overlapFcn, NULL );

	// CastRay (all hits)
	b2Vec2 rayOrigin = { 0.0f, 10.0f };
	b2Vec2 rayDir = { 0.0f, -20.0f };
	b2World_CastRay( worldId, rayOrigin, rayDir, filter, s_allHitsCastFcn, NULL );

	// CastRayClosest
	b2World_CastRayClosest( worldId, rayOrigin, rayDir, filter );

	// CastShape (circle proxy)
	b2ShapeProxy circProxy = b2MakeProxy( (b2Vec2[]){ { 0.0f, 0.0f } }, 1, 0.3f );
	b2World_CastShape( worldId, &circProxy, rayDir, filter, s_closestCastFcn, NULL );

	// CollideMover (capsule with radius > 2*B2_LINEAR_SLOP)
	b2Capsule moverCap = { { -0.3f, 0.0f }, { 0.3f, 0.0f }, 0.5f };
	b2World_CollideMover( worldId, &moverCap, filter, s_planeFcn, NULL );

	// CastMover
	b2Vec2 moverTranslation = { 0.0f, -5.0f };
	b2World_CastMover( worldId, &moverCap, moverTranslation, filter );

	// Shape_TestPoint: test inside (0,0 local, well inside the r=10 ground circle at y=-10)
	// and outside
	b2Shape_TestPoint( groundShapeId, (b2Vec2){ 0.0f, -10.0f } );   // inside center of ground
	b2Shape_TestPoint( groundShapeId, (b2Vec2){ 0.0f, 100.0f } );   // outside

	// Shape_RayCast against the ground shape
	b2RayCastInput rcIn = { { 0.0f, 5.0f }, { 0.0f, -20.0f }, 1.0f };
	b2Shape_RayCast( groundShapeId, &rcIn );
}

int RecordingTest( void )
{
	// Record a session

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.workerCount = 1;

	b2WorldId worldId = b2CreateWorld( &worldDef );
	ENSURE( b2World_IsValid( worldId ) );

	// Record from before the first step so the whole session is captured
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );

	// Static ground body with a circle shape
	b2BodyDef groundDef = b2DefaultBodyDef();
	groundDef.position = (b2Vec2){ 0.0f, -10.0f };
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	ENSURE( b2Body_IsValid( groundId ) );

	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2Circle groundCircle = { { 0.0f, 0.0f }, 10.0f };
	b2ShapeId groundShapeId = b2CreateCircleShape( groundId, &groundShapeDef, &groundCircle );
	ENSURE( b2Shape_IsValid( groundShapeId ) );

	// Dynamic body with a circle shape. The name is intentionally longer than B2_NAME_LENGTH so
	// replay exercises the over-length name path in the body def reader.
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Vec2){ 0.0f, 4.0f };
	bodyDef.name = "testBodyWithLongName";
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId ) );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2ShapeId shapeId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
	ENSURE( b2Shape_IsValid( shapeId ) );

	// Polygon shape on the dynamic body
	b2Polygon box = b2MakeBox( 0.25f, 0.25f );
	b2ShapeDef boxDef = b2DefaultShapeDef();
	boxDef.density = 2.0f;
	b2ShapeId boxShapeId = b2CreatePolygonShape( bodyId, &boxDef, &box );
	ENSURE( b2Shape_IsValid( boxShapeId ) );

	// Capsule on a second dynamic body
	b2BodyDef capsuleBodyDef = b2DefaultBodyDef();
	capsuleBodyDef.type = b2_dynamicBody;
	capsuleBodyDef.position = (b2Vec2){ 2.0f, 6.0f };
	b2BodyId capsuleBodyId = b2CreateBody( worldId, &capsuleBodyDef );
	b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f };
	b2ShapeDef capsuleDef = b2DefaultShapeDef();
	capsuleDef.density = 1.0f;
	b2ShapeId capsuleShapeId = b2CreateCapsuleShape( capsuleBodyId, &capsuleDef, &capsule );
	ENSURE( b2Shape_IsValid( capsuleShapeId ) );

	// Static segment extending the ground
	b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
	b2ShapeDef segmentDef = b2DefaultShapeDef();
	b2ShapeId segmentShapeId = b2CreateSegmentShape( groundId, &segmentDef, &segment );
	ENSURE( b2Shape_IsValid( segmentShapeId ) );

	// Chain segment shape on the ground
	b2ChainSegment chainSeg = { { -2.0f, 1.0f }, { { -1.0f, 1.0f }, { 1.0f, 1.0f } }, { 2.0f, 1.0f }, -1 };
	b2ShapeId chainSegShapeId = b2CreateChainSegmentShape( groundId, &segmentDef, &chainSeg );
	ENSURE( b2Shape_IsValid( chainSegShapeId ) );

	// Exercise the recorded shape mutators
	b2Shape_SetFriction( boxShapeId, 0.3f );
	b2Shape_SetRestitution( capsuleShapeId, 0.5f );
	b2Shape_SetDensity( boxShapeId, 3.0f, true );
	b2Shape_SetUserMaterial( boxShapeId, 0x1234u );
	b2SurfaceMaterial surface = b2DefaultSurfaceMaterial();
	surface.friction = 0.7f;
	surface.restitution = 0.1f;
	b2Shape_SetSurfaceMaterial( capsuleShapeId, &surface );
	b2Filter filter = b2DefaultFilter();
	filter.categoryBits = 0x2;
	b2Shape_SetFilter( boxShapeId, filter );
	b2Shape_EnableContactEvents( capsuleShapeId, true );
	b2Shape_EnableSensorEvents( capsuleShapeId, true );
	b2Shape_EnableHitEvents( boxShapeId, true );
	b2Shape_EnablePreSolveEvents( boxShapeId, true );
	b2Shape_ApplyWind( capsuleShapeId, (b2Vec2){ 1.0f, 0.0f }, 0.1f, 0.0f, true );

	// Change geometry in place
	b2Circle newCircle = { { 0.0f, 0.0f }, 0.4f };
	b2Shape_SetCircle( shapeId, &newCircle );

	// Throwaway shape to exercise DestroyShape
	b2Circle tmpCircle = { { 0.0f, 0.0f }, 0.1f };
	b2ShapeId tmpShapeId = b2CreateCircleShape( capsuleBodyId, &capsuleDef, &tmpCircle );
	b2DestroyShape( tmpShapeId, true );

	// A kinematic body to exercise SetType and SetTargetTransform
	b2BodyDef kinematicDef = b2DefaultBodyDef();
	kinematicDef.type = b2_kinematicBody;
	kinematicDef.position = (b2Vec2){ -3.0f, 5.0f };
	b2BodyId kinematicId = b2CreateBody( worldId, &kinematicDef );
	b2ShapeDef kinematicShapeDef = b2DefaultShapeDef();
	b2Circle kinematicCircle = { { 0.0f, 0.0f }, 0.3f };
	b2CreateCircleShape( kinematicId, &kinematicShapeDef, &kinematicCircle );

	// A body to exercise Disable/Enable
	b2BodyDef disableDef = b2DefaultBodyDef();
	disableDef.type = b2_dynamicBody;
	disableDef.position = (b2Vec2){ 5.0f, 5.0f };
	b2BodyId disableId = b2CreateBody( worldId, &disableDef );
	b2Circle disableCircle = { { 0.0f, 0.0f }, 0.3f };
	b2CreateCircleShape( disableId, &shapeDef, &disableCircle );

	// Exercise the recorded body mutators
	b2Body_SetTransform( bodyId, (b2Vec2){ 1.0f, 5.0f }, b2Rot_identity );
	b2Body_SetLinearVelocity( bodyId, (b2Vec2){ 0.5f, 0.0f } );
	b2Body_SetAngularVelocity( bodyId, 0.25f );
	b2Body_SetName( bodyId, "renamedBody" );
	b2Body_SetLinearDamping( bodyId, 0.1f );
	b2Body_SetAngularDamping( bodyId, 0.05f );
	b2Body_SetGravityScale( bodyId, 0.9f );
	b2Body_SetSleepThreshold( bodyId, 0.02f );
	b2Body_EnableSleep( bodyId, false );
	b2Body_SetBullet( bodyId, true );
	b2Body_EnableContactRecycling( bodyId, false );
	b2Body_EnableContactEvents( bodyId, true );
	b2Body_EnableHitEvents( bodyId, true );
	b2Body_SetMotionLocks( bodyId, (b2MotionLocks){ false, false, true } );
	b2MassData md = { 2.0f, { 0.0f, 0.0f }, 0.5f };
	b2Body_SetMassData( bodyId, md );
	b2Body_ApplyMassFromShapes( bodyId );
	b2Body_SetType( capsuleBodyId, b2_kinematicBody );
	b2Body_SetType( capsuleBodyId, b2_dynamicBody );
	b2Body_SetTargetTransform( kinematicId, (b2Transform){ { -2.0f, 5.0f }, b2Rot_identity }, 1.0f / 60.0f, true );
	b2Body_Disable( disableId );
	b2Body_Enable( disableId );
	b2Body_SetAwake( bodyId, true );
	b2Body_WakeTouching( bodyId );

	// Per-step forces and impulses applied before the first step
	b2Body_ApplyForce( bodyId, (b2Vec2){ 0.0f, 50.0f }, (b2Vec2){ 1.0f, 5.0f }, true );
	b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ 5.0f, 0.0f }, true );
	b2Body_ApplyTorque( bodyId, 1.0f, true );
	b2Body_ApplyLinearImpulse( bodyId, (b2Vec2){ 0.1f, 0.0f }, (b2Vec2){ 1.0f, 5.0f }, true );
	b2Body_ApplyLinearImpulseToCenter( bodyId, (b2Vec2){ 0.0f, 0.1f }, true );
	b2Body_ApplyAngularImpulse( bodyId, 0.05f, true );

	// Chain shape on a static body, plus a material change and a throwaway chain destroyed
	b2BodyDef chainBodyDef = b2DefaultBodyDef();
	chainBodyDef.position = (b2Vec2){ 0.0f, -2.0f };
	b2BodyId chainBodyId = b2CreateBody( worldId, &chainBodyDef );
	b2Vec2 chainPoints[6] = { { -8.0f, 0.0f }, { -4.0f, 0.0f }, { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 8.0f, 0.0f }, { 8.0f, 4.0f } };
	b2SurfaceMaterial chainMats[1] = { b2DefaultSurfaceMaterial() };
	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = chainPoints;
	chainDef.count = 6;
	chainDef.materials = chainMats;
	chainDef.materialCount = 1;
	chainDef.isLoop = false;
	b2ChainId chainId = b2CreateChain( chainBodyId, &chainDef );
	ENSURE( b2Chain_IsValid( chainId ) );

	b2SurfaceMaterial chainSurface = b2DefaultSurfaceMaterial();
	chainSurface.friction = 0.4f;
	b2Chain_SetSurfaceMaterial( chainId, &chainSurface, 0 );

	b2ChainId tmpChainId = b2CreateChain( chainBodyId, &chainDef );
	b2DestroyChain( tmpChainId );

	// Joints: a row of dynamic bodies connected by each joint type
	b2BodyId jb[8];
	for ( int i = 0; i < 8; ++i )
	{
		b2BodyDef jbd = b2DefaultBodyDef();
		jbd.type = b2_dynamicBody;
		jbd.position = (b2Vec2){ -7.0f + (float)i, 8.0f };
		jb[i] = b2CreateBody( worldId, &jbd );
		b2Circle jc = { { 0.0f, 0.0f }, 0.25f };
		b2CreateCircleShape( jb[i], &shapeDef, &jc );
	}

	// Revolute joint with full setter coverage and the generic mutators
	b2RevoluteJointDef revDef = b2DefaultRevoluteJointDef();
	revDef.base.bodyIdA = jb[0];
	revDef.base.bodyIdB = jb[1];
	revDef.base.localFrameA.p = (b2Vec2){ 0.5f, 0.0f };
	revDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
	b2JointId revId = b2CreateRevoluteJoint( worldId, &revDef );
	ENSURE( b2Joint_IsValid( revId ) );
	b2RevoluteJoint_EnableLimit( revId, true );
	b2RevoluteJoint_SetLimits( revId, -1.0f, 1.0f );
	b2RevoluteJoint_EnableMotor( revId, true );
	b2RevoluteJoint_SetMotorSpeed( revId, 0.5f );
	b2RevoluteJoint_SetMaxMotorTorque( revId, 10.0f );
	b2RevoluteJoint_EnableSpring( revId, true );
	b2RevoluteJoint_SetSpringHertz( revId, 2.0f );
	b2RevoluteJoint_SetSpringDampingRatio( revId, 0.5f );
	b2RevoluteJoint_SetTargetAngle( revId, 0.25f );
	b2Joint_SetLocalFrameA( revId, (b2Transform){ { 0.5f, 0.0f }, b2Rot_identity } );
	b2Joint_SetLocalFrameB( revId, (b2Transform){ { -0.5f, 0.0f }, b2Rot_identity } );
	b2Joint_SetConstraintTuning( revId, 60.0f, 2.0f );
	b2Joint_SetForceThreshold( revId, 100.0f );
	b2Joint_SetTorqueThreshold( revId, 50.0f );
	b2Joint_SetCollideConnected( revId, false );
	b2Joint_WakeBodies( revId );

	// Distance joint
	b2DistanceJointDef distDef = b2DefaultDistanceJointDef();
	distDef.base.bodyIdA = jb[1];
	distDef.base.bodyIdB = jb[2];
	distDef.length = 1.0f;
	b2JointId distId = b2CreateDistanceJoint( worldId, &distDef );
	b2DistanceJoint_SetLength( distId, 1.2f );
	b2DistanceJoint_EnableSpring( distId, true );
	b2DistanceJoint_SetSpringHertz( distId, 3.0f );
	b2DistanceJoint_SetSpringDampingRatio( distId, 0.4f );
	b2DistanceJoint_SetSpringForceRange( distId, -50.0f, 50.0f );
	b2DistanceJoint_EnableLimit( distId, true );
	b2DistanceJoint_SetLengthRange( distId, 0.5f, 2.0f );
	b2DistanceJoint_EnableMotor( distId, true );
	b2DistanceJoint_SetMotorSpeed( distId, 0.3f );
	b2DistanceJoint_SetMaxMotorForce( distId, 5.0f );

	// Prismatic joint
	b2PrismaticJointDef prisDef = b2DefaultPrismaticJointDef();
	prisDef.base.bodyIdA = jb[2];
	prisDef.base.bodyIdB = jb[3];
	b2JointId prisId = b2CreatePrismaticJoint( worldId, &prisDef );
	b2PrismaticJoint_EnableSpring( prisId, true );
	b2PrismaticJoint_SetSpringHertz( prisId, 2.0f );
	b2PrismaticJoint_SetSpringDampingRatio( prisId, 0.5f );
	b2PrismaticJoint_SetTargetTranslation( prisId, 0.1f );
	b2PrismaticJoint_EnableLimit( prisId, true );
	b2PrismaticJoint_SetLimits( prisId, -1.0f, 1.0f );
	b2PrismaticJoint_EnableMotor( prisId, true );
	b2PrismaticJoint_SetMotorSpeed( prisId, 0.2f );
	b2PrismaticJoint_SetMaxMotorForce( prisId, 8.0f );

	// Wheel joint
	b2WheelJointDef wheelDef = b2DefaultWheelJointDef();
	wheelDef.base.bodyIdA = jb[3];
	wheelDef.base.bodyIdB = jb[4];
	b2JointId wheelId = b2CreateWheelJoint( worldId, &wheelDef );
	b2WheelJoint_EnableSpring( wheelId, true );
	b2WheelJoint_SetSpringHertz( wheelId, 4.0f );
	b2WheelJoint_SetSpringDampingRatio( wheelId, 0.7f );
	b2WheelJoint_EnableLimit( wheelId, true );
	b2WheelJoint_SetLimits( wheelId, -0.5f, 0.5f );
	b2WheelJoint_EnableMotor( wheelId, true );
	b2WheelJoint_SetMotorSpeed( wheelId, 1.0f );
	b2WheelJoint_SetMaxMotorTorque( wheelId, 6.0f );

	// Weld joint
	b2WeldJointDef weldDef = b2DefaultWeldJointDef();
	weldDef.base.bodyIdA = jb[4];
	weldDef.base.bodyIdB = jb[5];
	b2JointId weldId = b2CreateWeldJoint( worldId, &weldDef );
	b2WeldJoint_SetLinearHertz( weldId, 5.0f );
	b2WeldJoint_SetLinearDampingRatio( weldId, 0.6f );
	b2WeldJoint_SetAngularHertz( weldId, 5.0f );
	b2WeldJoint_SetAngularDampingRatio( weldId, 0.6f );

	// Motor joint
	b2MotorJointDef motorDef = b2DefaultMotorJointDef();
	motorDef.base.bodyIdA = jb[5];
	motorDef.base.bodyIdB = jb[6];
	b2JointId motorId = b2CreateMotorJoint( worldId, &motorDef );
	b2MotorJoint_SetLinearVelocity( motorId, (b2Vec2){ 0.1f, 0.0f } );
	b2MotorJoint_SetAngularVelocity( motorId, 0.2f );
	b2MotorJoint_SetMaxVelocityForce( motorId, 10.0f );
	b2MotorJoint_SetMaxVelocityTorque( motorId, 10.0f );
	b2MotorJoint_SetLinearHertz( motorId, 2.0f );
	b2MotorJoint_SetLinearDampingRatio( motorId, 0.5f );
	b2MotorJoint_SetAngularHertz( motorId, 2.0f );
	b2MotorJoint_SetAngularDampingRatio( motorId, 0.5f );
	b2MotorJoint_SetMaxSpringForce( motorId, 20.0f );
	b2MotorJoint_SetMaxSpringTorque( motorId, 20.0f );

	// Filter joint, plus a throwaway joint to exercise DestroyJoint
	b2FilterJointDef filterDef = b2DefaultFilterJointDef();
	filterDef.base.bodyIdA = jb[6];
	filterDef.base.bodyIdB = jb[7];
	b2JointId filterId = b2CreateFilterJoint( worldId, &filterDef );
	ENSURE( b2Joint_IsValid( filterId ) );

	b2DistanceJointDef tmpJointDef = b2DefaultDistanceJointDef();
	tmpJointDef.base.bodyIdA = jb[0];
	tmpJointDef.base.bodyIdB = jb[7];
	tmpJointDef.length = 5.0f;
	b2JointId tmpJointId = b2CreateDistanceJoint( worldId, &tmpJointDef );
	b2DestroyJoint( tmpJointId, true );

	// Exercise world config mutators
	b2World_SetGravity( worldId, (b2Vec2){ 0.0f, -9.8f } );
	b2World_EnableSleeping( worldId, true );
	b2World_EnableContinuous( worldId, true );
	b2World_EnableWarmStarting( worldId, true );
	b2World_EnableSpeculative( worldId, true );
	b2World_SetRestitutionThreshold( worldId, 1.5f );
	b2World_SetHitEventThreshold( worldId, 2.0f );
	b2World_SetContactTuning( worldId, 30.0f, 10.0f, 3.0f );
	b2World_SetContactRecycleDistance( worldId, 0.05f );
	b2World_SetMaximumLinearSpeed( worldId, 100.0f );
	b2World_RebuildStaticTree( worldId );
	b2ExplosionDef explosion = b2DefaultExplosionDef();
	explosion.position = (b2Vec2){ 0.0f, 4.0f };
	explosion.radius = 3.0f;
	explosion.falloff = 1.0f;
	explosion.impulsePerLength = 5.0f;
	b2World_Explode( worldId, &explosion );

	// Issue all 9 query types before the first step (pre-step path)
	IssueAllQueries( worldId, groundShapeId );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	for ( int i = 0; i < 60; ++i )
	{
		// Inject mutators mid-simulation to exercise interleaving with steps
		if ( i == 30 )
		{
			b2Body_ApplyLinearImpulseToCenter( capsuleBodyId, (b2Vec2){ 2.0f, 0.0f }, true );
			b2Body_ClearForces( bodyId );
			b2Body_SetGravityScale( bodyId, 1.0f );
		}

		// Also issue queries mid-loop to exercise recording across steps
		if ( i == 15 )
		{
			IssueAllQueries( worldId, groundShapeId );
		}

		b2World_Step( worldId, timeStep, subStepCount );
	}

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	// The recording buffer now holds the full session
	const uint8_t* recData = b2Recording_GetData( rec );
	int recSize = b2Recording_GetSize( rec );
	ENSURE( recSize > 0 );

	// Replay from the buffer with the recorded worker count
	ENSURE( b2ValidateReplay( recData, recSize, 0 ) );

	// Replay with a different worker count to prove cross-thread determinism
	ENSURE( b2ValidateReplay( recData, recSize, 4 ) );

	// The reserved header bytes (offsets 8 and 16, formerly buildHash and simdWidth) must stay
	// ignored on read. Guards a future change that starts validating them or shrinks the header.
	{
		uint8_t* patched = b2Alloc( recSize );
		memcpy( patched, recData, recSize );
		patched[8] = 0xAB;
		patched[9] = 0xCD;
		patched[10] = 0xEF;
		patched[11] = 0x12;
		patched[16] = 0x34;
		ENSURE( b2ValidateReplay( patched, recSize, 0 ) );
		b2Free( patched, recSize );
	}

	// File round-trip: save the buffer, load it back, and replay the loaded copy
	ENSURE( b2SaveRecordingToFile( rec, s_recPath ) );
	b2Recording* loaded = b2LoadRecordingFromFile( s_recPath );
	ENSURE( loaded != NULL );
	ENSURE( b2Recording_GetSize( loaded ) == recSize );
	ENSURE( b2ValidateReplay( b2Recording_GetData( loaded ), b2Recording_GetSize( loaded ), 0 ) );
	ENSURE( b2ValidateReplay( b2Recording_GetData( loaded ), b2Recording_GetSize( loaded ), 4 ) );
	b2DestroyRecording( loaded );

	// Drive the incremental player directly. It underpins the viewer and exercises
	// per-frame stepping, restart, and the getters beyond what b2ValidateReplay covers.
	{
		b2RecPlayer* player = b2RecPlayer_Create( recData, recSize, 0 );
		ENSURE( player != NULL );

		// Build a no-op b2DebugDraw to exercise the draw path headlessly
		b2DebugDraw dd = b2DefaultDebugDraw();
		dd.DrawLineFcn = s_DrawLine;
		dd.DrawPointFcn = s_DrawPoint;
		dd.DrawPolygonFcn = s_DrawPoly;
		dd.DrawSolidCapsuleFcn = s_DrawCapsule;

		int frames = 0;
		while ( b2RecPlayer_StepFrame( player ) )
		{
			// Exercise the draw path on every other frame
			if ( frames % 2 == 0 )
			{
				b2RecPlayer_DrawFrameQueries( player, &dd, -1 );
			}
			frames += 1;
		}
		ENSURE( frames == 60 );
		ENSURE( b2RecPlayer_GetFrame( player ) == 60 );
		ENSURE( b2RecPlayer_IsAtEnd( player ) );
		ENSURE( b2RecPlayer_HasDiverged( player ) == false );

		// The trailing DestroyWorld is an end marker; the world stays valid so a viewer can keep
		// drawing the final step rather than blanking at the end
		ENSURE( b2World_IsValid( b2RecPlayer_GetWorldId( player ) ) );

		// Restart reproduces the same run without reloading the file
		b2RecPlayer_Restart( player );
		ENSURE( b2RecPlayer_GetFrame( player ) == 0 );
		ENSURE( b2RecPlayer_IsAtEnd( player ) == false );

		int frames2 = 0;
		while ( b2RecPlayer_StepFrame( player ) )
		{
			frames2 += 1;
		}
		ENSURE( frames2 == 60 );
		ENSURE( b2RecPlayer_HasDiverged( player ) == false );

		b2RecPlayer_Destroy( player );
	}

	b2DestroyRecording( rec );
	remove( s_recPath );
	return 0;
}

// Recording started mid-stream snapshots the live world as its seed. Those seed bodies are restored
// as a struct image on replay and never pass through the CreateBody hook, so the player must seed
// its outliner body list from the restored world. Guards that path, which drives the viewer Outline.
int RecordingOutlinerTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Build a scene before recording so the bodies live in the seed snapshot, not the op stream
	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2Circle groundCircle = { { 0.0f, 0.0f }, 10.0f };
	b2CreateCircleShape( groundId, &groundShapeDef, &groundCircle );

	int dynamicCount = 3;
	for ( int i = 0; i < dynamicCount; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ (float)i, 4.0f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );
	}
	int expectedBodies = 1 + dynamicCount;

	// Settle a step, then start recording with the scene already present (non-empty seed)
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );
	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	const uint8_t* recData = b2Recording_GetData( rec );
	int recSize = b2Recording_GetSize( rec );
	ENSURE( recSize > 0 );

	b2RecPlayer* player = b2RecPlayer_Create( recData, recSize, 0 );
	ENSURE( player != NULL );

	// The outliner list must be populated from the seed snapshot before any frame is stepped, and
	// match the live body count of the restored world (no destroys yet, so no nulled holes)
	b2WorldId replayWorldId = b2RecPlayer_GetWorldId( player );
	int seedCount = b2RecPlayer_GetBodyCount( player );
	ENSURE( seedCount == expectedBodies );
	ENSURE( seedCount == b2World_GetCounters( replayWorldId ).bodyCount );

	// Each seeded id is a valid handle into the replay world
	for ( int ord = 0; ord < seedCount; ++ord )
	{
		ENSURE( b2Body_IsValid( b2RecPlayer_GetBodyId( player, ord ) ) );
	}

	while ( b2RecPlayer_StepFrame( player ) )
	{
	}

	// Restart rolls the outliner list back to its frame-0 seed contents
	b2RecPlayer_Restart( player );
	ENSURE( b2RecPlayer_GetBodyCount( player ) == seedCount );

	b2RecPlayer_Destroy( player );
	b2DestroyRecording( rec );
	return 0;
}

// Deep hash of a replay world, the ground truth a keyframe seek must reproduce
static uint64_t ReplayDeepHash( b2RecPlayer* player )
{
	return b2HashWorldStateDeep( b2GetWorldFromId( b2RecPlayer_GetWorldId( player ) ) );
}

// A backward seek restores the nearest keyframe and re-steps the gap, so it must land on the exact
// state a linear forward replay would. Compares scattered backward and forward seeks against a
// forward-only deep-hash table at the given worker count. A positive budgetBytes tightens the
// keyframe policy on the player under test to force repeated budget eviction; the forward-only
// reference is left at the default policy since it never seeks back.
static int CheckKeyframeSeek( const uint8_t* recData, int recSize, int workerCount, int budgetBytes, int minInterval )
{
	// Forward-only reference: a fresh player never seeks backward, so it never restores a keyframe
	// and gives the linear ground truth deep hash at every frame
	b2RecPlayer* ref = b2RecPlayer_Create( recData, recSize, workerCount );
	ENSURE( ref != NULL );
	int frameCount = b2RecPlayer_GetInfo( ref ).frameCount;
	ENSURE( frameCount > 0 );

	uint64_t* refHash = malloc( (size_t)( frameCount + 1 ) * sizeof( uint64_t ) );
	ENSURE( refHash != NULL );
	refHash[0] = ReplayDeepHash( ref );
	for ( int f = 1; f <= frameCount; ++f )
	{
		ENSURE( b2RecPlayer_StepFrame( ref ) );
		refHash[f] = ReplayDeepHash( ref );
	}
	ENSURE( b2RecPlayer_HasDiverged( ref ) == false );
	b2RecPlayer_Destroy( ref );

	// Player under test: play to the end so the keyframe ring is fully populated (and an eviction
	// has fired under a tight budget), then seek around it
	b2RecPlayer* player = b2RecPlayer_Create( recData, recSize, workerCount );
	ENSURE( player != NULL );
	if ( budgetBytes > 0 )
	{
		b2RecPlayer_SetKeyframePolicy( player, budgetBytes, minInterval );
	}
	while ( b2RecPlayer_StepFrame( player ) )
	{
	}
	ENSURE( b2RecPlayer_GetFrame( player ) == frameCount );

	// Targets jump backward and forward: below the first keyframe (1, 5), onto exact interval
	// multiples (128, 256), and around the eviction boundary near frame 272
	int targets[] = { frameCount, 1, frameCount - 1, 290, 17, 271, 256, 128, 33, 200, 5, 300, 100, frameCount };
	for ( int i = 0; i < ARRAY_COUNT( targets ); ++i )
	{
		int t = targets[i];
		if ( t > frameCount )
		{
			t = frameCount;
		}
		b2RecPlayer_SeekFrame( player, t );
		ENSURE( b2RecPlayer_GetFrame( player ) == t );
		ENSURE( b2RecPlayer_HasDiverged( player ) == false );
		uint64_t got = ReplayDeepHash( player );
		if ( got != refHash[t] )
		{
			printf( "keyframe seek mismatch at frame %d (wc %d): got %llu want %llu\n", t, workerCount,
					(unsigned long long)got, (unsigned long long)refHash[t] );
			free( refHash );
			b2RecPlayer_Destroy( player );
			return 1;
		}
	}

	b2RecPlayer_Destroy( player );
	free( refHash );
	return 0;
}

// Fast backward seeking via keyframes must be bit-identical to a linear replay. Records enough frames
// to fill the keyframe ring and trigger one eviction, then verifies seeks at two worker counts.
int RecordingKeyframeTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Ground
	b2BodyDef groundDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2Polygon groundBox = b2MakeBox( 20.0f, 1.0f );
	b2CreatePolygonShape( groundId, &groundShapeDef, &groundBox );

	// A light stack of dynamic boxes so the world keeps evolving each step (settling, then sleeping)
	// without the cost of joints. The offset start makes the stack topple a little, lengthening the
	// dynamic phase that discriminates a faithful restore.
	for ( int i = 0; i < 8; ++i )
	{
		b2BodyDef bd = b2DefaultBodyDef();
		bd.type = b2_dynamicBody;
		bd.position = (b2Vec2){ 0.05f * (float)i, 2.0f + 1.1f * (float)i };
		b2BodyId id = b2CreateBody( worldId, &bd );
		b2ShapeDef sd = b2DefaultShapeDef();
		sd.density = 1.0f;
		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		b2CreatePolygonShape( id, &sd, &box );
	}

	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );

	// 320 steps fills the 16-deep ring (keyframes at 16..256) and fires the eviction at 272
	for ( int i = 0; i < 320; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	const uint8_t* recData = b2Recording_GetData( rec );
	int recSize = b2Recording_GetSize( rec );
	ENSURE( recSize > 0 );

	// Measure one snapshot so the tight budget holds only a handful of keyframes, forcing the
	// interval-doubling eviction during the 320-frame replay
	b2RecPlayer* probe = b2RecPlayer_Create( recData, recSize, 0 );
	ENSURE( probe != NULL );
	int snapSize = b2World_Snapshot( b2RecPlayer_GetWorldId( probe ), NULL, 0 );
	b2RecPlayer_Destroy( probe );
	ENSURE( snapSize > 0 );
	int tightBudget = 6 * snapSize;

	// Default policy (capture + restore, no eviction) and a tight budget (forces eviction and the
	// SetKeyframePolicy path), each at the recorded worker count and a different one
	ENSURE( CheckKeyframeSeek( recData, recSize, 0, 0, 0 ) == 0 );
	ENSURE( CheckKeyframeSeek( recData, recSize, 4, 0, 0 ) == 0 );
	ENSURE( CheckKeyframeSeek( recData, recSize, 0, tightBudget, 8 ) == 0 );
	ENSURE( CheckKeyframeSeek( recData, recSize, 4, tightBudget, 8 ) == 0 );

	b2DestroyRecording( rec );
	return 0;
}
