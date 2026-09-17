// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <math.h>
#include <stdint.h>

// The guard logs once per rejected call in every build config, while the assert only fires in
// debug. So the log count is the portable signal that bad input was turned away.
static int logCount;

static void CountingLogFcn( const char* message )
{
	MAYBE_UNUSED( message );
	logCount += 1;
}

static int SilentAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	MAYBE_UNUSED( condition );
	MAYBE_UNUSED( fileName );
	MAYBE_UNUSED( lineNumber );

	// Keep going so the subtest can report rather than break to the debugger
	return 0;
}

static void BeginRejectCount( void )
{
	logCount = 0;
	b2SetLogFcn( CountingLogFcn );
	b2SetAssertFcn( SilentAssertFcn );
}

static void EndRejectCount( void )
{
	b2SetAssertFcn( TestAssertFcn );
	b2SetLogFcn( TestLogFcn );
}

#define REJECTS( CALL )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		int countBefore = logCount;                                                                                              \
		CALL;                                                                                                                    \
		if ( logCount != countBefore + 1 )                                                                                       \
		{                                                                                                                        \
			printf( "not rejected: " #CALL "\n" );                                                                               \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

#define ACCEPTS( CALL )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		int countBefore = logCount;                                                                                              \
		CALL;                                                                                                                    \
		if ( logCount != countBefore )                                                                                           \
		{                                                                                                                        \
			printf( "wrongly rejected: " #CALL "\n" );                                                                           \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( false )

static const float badValues[] = { NAN, INFINITY, -INFINITY };

static bool IsBodyFinite( b2BodyId bodyId )
{
	return b2IsValidPosition( b2Body_GetPosition( bodyId ) ) && b2IsValidRotation( b2Body_GetRotation( bodyId ) ) &&
		   b2IsValidVec2( b2Body_GetLinearVelocity( bodyId ) ) && b2IsValidFloat( b2Body_GetAngularVelocity( bodyId ) );
}

static int WorldInputTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2Vec2 gravity = b2World_GetGravity( worldId );
	float restitutionThreshold = b2World_GetRestitutionThreshold( worldId );
	float hitEventThreshold = b2World_GetHitEventThreshold( worldId );
	float recycleDistance = b2World_GetContactRecycleDistance( worldId );
	float maximumLinearSpeed = b2World_GetMaximumLinearSpeed( worldId );

	BeginRejectCount();

	for ( int i = 0; i < ARRAY_COUNT( badValues ); ++i )
	{
		float bad = badValues[i];

		REJECTS( b2World_SetGravity( worldId, (b2Vec2){ bad, 0.0f } ) );
		REJECTS( b2World_SetGravity( worldId, (b2Vec2){ 0.0f, bad } ) );
		REJECTS( b2World_SetRestitutionThreshold( worldId, bad ) );
		REJECTS( b2World_SetHitEventThreshold( worldId, bad ) );
		REJECTS( b2World_SetContactRecycleDistance( worldId, bad ) );
		REJECTS( b2World_SetMaximumLinearSpeed( worldId, bad ) );
		REJECTS( b2World_SetContactTuning( worldId, bad, 10.0f, 3.0f ) );
		REJECTS( b2World_SetContactTuning( worldId, 30.0f, bad, 3.0f ) );
		REJECTS( b2World_SetContactTuning( worldId, 30.0f, 10.0f, bad ) );
		REJECTS( b2World_Step( worldId, bad, 4 ) );
	}

	ACCEPTS( b2World_Step( worldId, 1.0f / 60.0f, 4 ) );

	EndRejectCount();

	ENSURE( b2World_GetGravity( worldId ).x == gravity.x && b2World_GetGravity( worldId ).y == gravity.y );
	ENSURE( b2World_GetRestitutionThreshold( worldId ) == restitutionThreshold );
	ENSURE( b2World_GetHitEventThreshold( worldId ) == hitEventThreshold );
	ENSURE( b2World_GetContactRecycleDistance( worldId ) == recycleDistance );
	ENSURE( b2World_GetMaximumLinearSpeed( worldId ) == maximumLinearSpeed );

	b2DestroyWorld( worldId );
	return 0;
}

static int BodyInputTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	BeginRejectCount();

	for ( int i = 0; i < ARRAY_COUNT( badValues ); ++i )
	{
		float bad = badValues[i];
		b2Vec2 badVec = { bad, 0.0f };
		b2Pos badPos = { bad, 0.0f };
		b2Rot badRot = { bad, 0.0f };
		b2WorldTransform badTarget = { badPos, b2Rot_identity };

		REJECTS( b2Body_SetLinearVelocity( bodyId, badVec ) );
		REJECTS( b2Body_SetAngularVelocity( bodyId, bad ) );
		REJECTS( b2Body_SetTransform( bodyId, badPos, b2Rot_identity ) );
		REJECTS( b2Body_SetTransform( bodyId, b2Pos_zero, badRot ) );
		REJECTS( b2Body_SetTargetTransform( bodyId, badTarget, 1.0f / 60.0f, true ) );
		REJECTS( b2Body_SetTargetTransform( bodyId, b2WorldTransform_identity, bad, true ) );
		REJECTS( b2Body_ApplyForce( bodyId, badVec, b2Pos_zero, true ) );
		REJECTS( b2Body_ApplyForce( bodyId, b2Vec2_zero, badPos, true ) );
		REJECTS( b2Body_ApplyForceToCenter( bodyId, badVec, true ) );
		REJECTS( b2Body_ApplyTorque( bodyId, bad, true ) );
		REJECTS( b2Body_ApplyLinearImpulse( bodyId, badVec, b2Pos_zero, true ) );
		REJECTS( b2Body_ApplyLinearImpulse( bodyId, b2Vec2_zero, badPos, true ) );
		REJECTS( b2Body_ApplyLinearImpulseToCenter( bodyId, badVec, true ) );
		REJECTS( b2Body_ApplyAngularImpulse( bodyId, bad, true ) );
		REJECTS( b2Body_SetMassData( bodyId, (b2MassData){ bad, b2Vec2_zero, 1.0f } ) );
		REJECTS( b2Body_SetMassData( bodyId, (b2MassData){ 1.0f, badVec, 1.0f } ) );
		REJECTS( b2Body_SetMassData( bodyId, (b2MassData){ 1.0f, b2Vec2_zero, bad } ) );
		REJECTS( b2Body_SetLinearDamping( bodyId, bad ) );
		REJECTS( b2Body_SetAngularDamping( bodyId, bad ) );
		REJECTS( b2Body_SetGravityScale( bodyId, bad ) );
		REJECTS( b2Body_SetSleepThreshold( bodyId, bad ) );
		REJECTS( b2Body_SetSafetyFactor( bodyId, bad ) );
	}

	// A negative value on a quantity the API documents as non-negative is turned away too,
	// since negative damping grows velocity without bound.
	REJECTS( b2Body_SetLinearDamping( bodyId, -1.0f ) );
	REJECTS( b2Body_SetAngularDamping( bodyId, -1.0f ) );

	ACCEPTS( b2Body_SetLinearVelocity( bodyId, (b2Vec2){ 1.0f, 2.0f } ) );
	ACCEPTS( b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ 0.0f, 10.0f }, true ) );

	for ( int i = 0; i < 30; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	EndRejectCount();

	ENSURE( IsBodyFinite( bodyId ) );

	b2DestroyWorld( worldId );
	return 0;
}

static int ShapeInputTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2ShapeId shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );

	float density = b2Shape_GetDensity( shapeId );
	float friction = b2Shape_GetFriction( shapeId );
	float restitution = b2Shape_GetRestitution( shapeId );

	BeginRejectCount();

	for ( int i = 0; i < ARRAY_COUNT( badValues ); ++i )
	{
		float bad = badValues[i];
		b2Vec2 badVec = { bad, 0.0f };

		b2Circle badCircle = { badVec, 1.0f };
		b2Capsule badCapsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, bad };
		b2Segment badSegment = { badVec, { 1.0f, 0.0f } };
		b2ChainSegment badChainSegment = { badVec, { { 0.0f, 0.0f }, { 1.0f, 0.0f } }, { 2.0f, 0.0f }, -1 };

		b2Polygon badPolygon = box;
		badPolygon.radius = bad;

		b2Polygon badVertexPolygon = box;
		badVertexPolygon.vertices[0] = badVec;

		b2SurfaceMaterial badMaterial = b2DefaultSurfaceMaterial();
		badMaterial.friction = bad;

		REJECTS( b2Shape_SetDensity( shapeId, bad, true ) );
		REJECTS( b2Shape_SetFriction( shapeId, bad ) );
		REJECTS( b2Shape_SetRestitution( shapeId, bad ) );
		REJECTS( b2Shape_SetSurfaceMaterial( shapeId, &badMaterial ) );
		REJECTS( b2Shape_ApplyWind( shapeId, badVec, 1.0f, 1.0f, true ) );
		REJECTS( b2Shape_ApplyWind( shapeId, b2Vec2_zero, bad, 1.0f, true ) );
		REJECTS( b2Shape_ApplyWind( shapeId, b2Vec2_zero, 1.0f, bad, true ) );
		REJECTS( b2Shape_SetCircle( shapeId, &badCircle ) );
		REJECTS( b2Shape_SetCapsule( shapeId, &badCapsule ) );
		REJECTS( b2Shape_SetSegment( shapeId, &badSegment ) );
		REJECTS( b2Shape_SetChainSegment( shapeId, &badChainSegment ) );
		REJECTS( b2Shape_SetPolygon( shapeId, &badPolygon ) );
		REJECTS( b2Shape_SetPolygon( shapeId, &badVertexPolygon ) );
	}

	ACCEPTS( b2Shape_SetFriction( shapeId, 0.4f ) );

	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	EndRejectCount();

	// The rejected geometry never reached the shape, so the proxy is still the original box
	ENSURE( b2Shape_GetType( shapeId ) == b2_polygonShape );
	ENSURE( b2IsValidAABB( b2Shape_GetAABB( shapeId ) ) );
	ENSURE( b2IsValidAABB( b2World_GetBounds( worldId ) ) );
	ENSURE( b2Shape_GetDensity( shapeId ) == density );
	ENSURE( b2Shape_GetFriction( shapeId ) == 0.4f && friction == friction );
	ENSURE( b2Shape_GetRestitution( shapeId ) == restitution );
	ENSURE( IsBodyFinite( bodyId ) );

	b2DestroyWorld( worldId );
	return 0;
}

static int JointInputTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, -1.0f };
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
	jointDef.base.bodyIdA = groundId;
	jointDef.base.bodyIdB = bodyId;
	jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 1.0f };
	b2JointId jointId = b2CreateRevoluteJoint( worldId, &jointDef );

	BeginRejectCount();

	for ( int i = 0; i < ARRAY_COUNT( badValues ); ++i )
	{
		float bad = badValues[i];
		b2Transform badFrame = { { bad, 0.0f }, b2Rot_identity };

		REJECTS( b2Joint_SetLocalFrameA( jointId, badFrame ) );
		REJECTS( b2Joint_SetLocalFrameB( jointId, badFrame ) );
		REJECTS( b2Joint_SetConstraintTuning( jointId, bad, 1.0f ) );
		REJECTS( b2Joint_SetConstraintTuning( jointId, 60.0f, bad ) );
		REJECTS( b2Joint_SetForceThreshold( jointId, bad ) );
		REJECTS( b2Joint_SetTorqueThreshold( jointId, bad ) );
		REJECTS( b2RevoluteJoint_SetSpringHertz( jointId, bad ) );
		REJECTS( b2RevoluteJoint_SetSpringDampingRatio( jointId, bad ) );
		REJECTS( b2RevoluteJoint_SetTargetAngle( jointId, bad ) );
		REJECTS( b2RevoluteJoint_SetLimits( jointId, bad, 1.0f ) );
		REJECTS( b2RevoluteJoint_SetLimits( jointId, -1.0f, bad ) );
		REJECTS( b2RevoluteJoint_SetMotorSpeed( jointId, bad ) );
		REJECTS( b2RevoluteJoint_SetMaxMotorTorque( jointId, bad ) );
	}

	ACCEPTS( b2RevoluteJoint_SetMotorSpeed( jointId, 2.0f ) );

	for ( int i = 0; i < 30; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	EndRejectCount();

	ENSURE( IsBodyFinite( bodyId ) );
	ENSURE( b2IsValidVec2( b2Joint_GetConstraintForce( jointId ) ) );
	ENSURE( b2IsValidFloat( b2Joint_GetConstraintTorque( jointId ) ) );

	b2DestroyWorld( worldId );
	return 0;
}

static int CreateInputTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	BeginRejectCount();

	for ( int i = 0; i < ARRAY_COUNT( badValues ); ++i )
	{
		float bad = badValues[i];
		b2Vec2 badVec = { bad, 0.0f };

		b2WorldDef badWorldDef = b2DefaultWorldDef();
		badWorldDef.gravity = badVec;
		b2WorldId badWorldId = b2CreateWorld( &badWorldDef );
		ENSURE( b2World_IsValid( badWorldId ) == false );

		b2BodyDef badBodyDef = b2DefaultBodyDef();
		badBodyDef.position = (b2Pos){ bad, 0.0f };
		ENSURE( b2Body_IsValid( b2CreateBody( worldId, &badBodyDef ) ) == false );

		b2ShapeDef badShapeDef = b2DefaultShapeDef();
		badShapeDef.density = bad;
		ENSURE( b2Shape_IsValid( b2CreatePolygonShape( bodyId, &badShapeDef, &box ) ) == false );

		b2Circle badCircle = { badVec, 1.0f };
		ENSURE( b2Shape_IsValid( b2CreateCircleShape( bodyId, &shapeDef, &badCircle ) ) == false );

		b2Capsule badCapsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, bad };
		ENSURE( b2Shape_IsValid( b2CreateCapsuleShape( bodyId, &shapeDef, &badCapsule ) ) == false );

		b2Segment badSegment = { badVec, { 1.0f, 0.0f } };
		ENSURE( b2Shape_IsValid( b2CreateSegmentShape( bodyId, &shapeDef, &badSegment ) ) == false );

		b2Polygon badPolygon = box;
		badPolygon.radius = bad;
		ENSURE( b2Shape_IsValid( b2CreatePolygonShape( bodyId, &shapeDef, &badPolygon ) ) == false );

		b2Vec2 chainPoints[4] = { { -4.0f, 0.0f }, { 0.0f, 0.0f }, { 4.0f, 0.0f }, { 8.0f, 0.0f } };
		chainPoints[2] = badVec;
		b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
		b2ChainDef badChainDef = b2DefaultChainDef();
		badChainDef.points = chainPoints;
		badChainDef.pointCount = 4;
		badChainDef.materials = &material;
		badChainDef.materialCount = 1;
		badChainDef.isLoop = true;
		ENSURE( b2Chain_IsValid( b2CreateChain( bodyId, &badChainDef ) ) == false );

		b2RevoluteJointDef badJointDef = b2DefaultRevoluteJointDef();
		badJointDef.base.bodyIdA = bodyId;
		badJointDef.base.bodyIdB = b2CreateBody( worldId, &bodyDef );
		badJointDef.hertz = bad;
		ENSURE( b2Joint_IsValid( b2CreateRevoluteJoint( worldId, &badJointDef ) ) == false );

		b2ExplosionDef badExplosionDef = b2DefaultExplosionDef();
		badExplosionDef.radius = bad;
		REJECTS( b2World_Explode( worldId, &badExplosionDef ) );
	}

	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	EndRejectCount();

	ENSURE( IsBodyFinite( bodyId ) );
	ENSURE( b2IsValidAABB( b2World_GetBounds( worldId ) ) );

	b2DestroyWorld( worldId );
	return 0;
}

static int RecordReplayTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = (b2Vec2){ 0.0f, -10.0f };
	worldDef.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2Recording* recording = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, recording );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Segment ground = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
	b2CreateSegmentShape( groundId, &shapeDef, &ground );

	bodyDef.type = b2_dynamicBody;
	bodyDef.position = (b2Pos){ 0.0f, 4.0f };
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

	b2Polygon box = b2MakeBox( 0.5f, 0.5f );
	b2CreatePolygonShape( bodyId, &shapeDef, &box );

	BeginRejectCount();

	// Interleave turned away calls with real ones so the op stream carries both. A mutator is
	// rejected before it is recorded, while a create is recorded with the null id it returned.
	for ( int i = 0; i < 30; ++i )
	{
		float bad = badValues[i % ARRAY_COUNT( badValues )];

		REJECTS( b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ bad, 0.0f }, true ) );
		REJECTS( b2Body_SetAngularVelocity( bodyId, bad ) );

		b2ShapeDef badShapeDef = b2DefaultShapeDef();
		badShapeDef.density = bad;
		ENSURE( b2Shape_IsValid( b2CreatePolygonShape( bodyId, &badShapeDef, &box ) ) == false );

		b2Body_ApplyForceToCenter( bodyId, (b2Vec2){ 0.0f, 1.0f }, true );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( IsBodyFinite( bodyId ) );

	b2World_StopRecording( worldId );
	b2DestroyWorld( worldId );

	const uint8_t* data = b2Recording_GetData( recording );
	int size = b2Recording_GetSize( recording );
	ENSURE( size > 0 );
	ENSURE( b2ValidateReplay( data, size, 0 ) );
	ENSURE( b2ValidateReplay( data, size, 4 ) );

	EndRejectCount();

	b2DestroyRecording( recording );
	return 0;
}

int InvalidInputTest( void )
{
	RUN_SUBTEST( WorldInputTest );
	RUN_SUBTEST( BodyInputTest );
	RUN_SUBTEST( ShapeInputTest );
	RUN_SUBTEST( JointInputTest );
	RUN_SUBTEST( CreateInputTest );
	RUN_SUBTEST( RecordReplayTest );

	return 0;
}
