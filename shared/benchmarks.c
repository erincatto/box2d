// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"

#include "human.h"

#include "box2d/box2d.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef NDEBUG
#define BENCHMARK_DEBUG 0
#else
#define BENCHMARK_DEBUG 1
#endif

void CreateJointGrid( b2WorldId worldId )
{
	b2World_EnableSleeping( worldId, false );

	int N = BENCHMARK_DEBUG ? 20 : 100;

	// Allocate to avoid huge stack usage
	b2BodyId* bodies = malloc( N * N * sizeof( b2BodyId ) );
	int index = 0;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ~2u;

	b2Circle circle = { { 0.0f, 0.0f }, 0.4f };

	b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
	jointDef.base.drawScale = 0.4f;

	b2BodyDef bodyDef = b2DefaultBodyDef();

	for ( int k = 0; k < N; ++k )
	{
		for ( int i = 0; i < N; ++i )
		{
			float fk = (float)k;
			float fi = (float)i;

			if ( k >= N / 2 - 3 && k <= N / 2 + 3 && i == 0 )
			{
				bodyDef.type = b2_staticBody;
			}
			else
			{
				bodyDef.type = b2_dynamicBody;
			}

			bodyDef.position = (b2Pos){ fk, -fi };

			b2BodyId body = b2CreateBody( worldId, &bodyDef );

			b2CreateCircleShape( body, &shapeDef, &circle );

			if ( i > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - 1];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b2Vec2){ 0.0f, -0.5f };
				jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.5f };
				b2CreateRevoluteJoint( worldId, &jointDef );
			}

			if ( k > 0 )
			{
				jointDef.base.bodyIdA = bodies[index - N];
				jointDef.base.bodyIdB = body;
				jointDef.base.localFrameA.p = (b2Vec2){ 0.5f, 0.0f };
				jointDef.base.localFrameB.p = (b2Vec2){ -0.5f, 0.0f };
				b2CreateRevoluteJoint( worldId, &jointDef );
			}

			bodies[index++] = body;
		}
	}

	free( bodies );
}

void CreateLargePyramid( b2WorldId worldId )
{
	b2World_EnableSleeping( worldId, false );

	int baseCount = BENCHMARK_DEBUG ? 20 : 100;

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 100.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	// bodyDef.enableSleep = false;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;

	float a = 0.5f;
	b2Polygon box = b2MakeSquare( a );

	float shift = 1.0f * a;

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - a * baseCount;

			bodyDef.position = (b2Pos){ x, y };

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}
}

static void CreateSmallPyramid( b2WorldId worldId, int baseCount, float extent, float centerX, float baseY )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2Polygon box = b2MakeSquare( extent );

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * extent + baseY;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * extent + 2.0f * ( j - i ) * extent + centerX - 0.5f;
			bodyDef.position = (b2Pos){ x, y };

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}
}

void CreateManyPyramids( b2WorldId worldId )
{
	b2World_EnableSleeping( worldId, false );

	int baseCount = 10;
	float extent = 0.5f;
	int rowCount = BENCHMARK_DEBUG ? 5 : 20;
	int columnCount = BENCHMARK_DEBUG ? 5 : 20;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	float groundDeltaY = 2.0f * extent * ( baseCount + 1.0f );
	float groundWidth = 2.0f * extent * columnCount * ( baseCount + 1.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	float groundY = 0.0f;

	for ( int i = 0; i < rowCount; ++i )
	{
		b2Segment segment = { { -0.5f * groundWidth, groundY }, { 0.5f * groundWidth, groundY } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		groundY += groundDeltaY;
	}

	float baseWidth = 2.0f * extent * baseCount;
	float baseY = 0.0f;

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			float centerX = -0.5f * groundWidth + j * ( baseWidth + 2.0f * extent ) + 2.0f * extent;
			CreateSmallPyramid( worldId, baseCount, extent, centerX, baseY );
		}

		baseY += groundDeltaY;
	}
}

b2Capacity GetManyPyramidsCapacity( void )
{
	b2Capacity c = {
		.staticShapeCount = 20,
		.staticBodyCount = 1,
		.dynamicShapeCount = 22000,
		.dynamicBodyCount = 22000,
		.contactCount = 58000,
	};

	return c;
}

#ifdef NDEBUG
enum RainConstants
{
	RAIN_ROW_COUNT = 5,
	RAIN_COLUMN_COUNT = 40,
	RAIN_GROUP_SIZE = 5,
};
#else
enum RainConstants
{
	RAIN_ROW_COUNT = 3,
	RAIN_COLUMN_COUNT = 10,
	RAIN_GROUP_SIZE = 2,
};
#endif

typedef struct Group
{
	Human humans[RAIN_GROUP_SIZE];
} Group;

typedef struct RainData
{
	Group groups[RAIN_ROW_COUNT * RAIN_COLUMN_COUNT];
	float gridSize;
	int gridCount;
	int columnCount;
	int columnIndex;
} RainData;

RainData g_rainData;

void CreateRain( b2WorldId worldId )
{
	memset( &g_rainData, 0, sizeof( g_rainData ) );

	g_rainData.gridSize = 0.5f;
	g_rainData.gridCount = BENCHMARK_DEBUG ? 200 : 500;

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		float y = 0.0f;
		float width = g_rainData.gridSize;
		float height = g_rainData.gridSize;

		for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
		{
			float x = -0.5f * g_rainData.gridCount * g_rainData.gridSize;
			for ( int j = 0; j <= g_rainData.gridCount; ++j )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * width, 0.5f * height, (b2Vec2){ x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );

				// b2Segment segment = { { x - 0.5f * width, y }, { x + 0.5f * width, y } };
				// b2CreateSegmentShape( groundId, &shapeDef, &segment );

				x += g_rainData.gridSize;
			}

			y += 45.0f;
		}
	}

	g_rainData.columnCount = 0;
	g_rainData.columnIndex = 0;
}

void CreateGroup( b2WorldId worldId, int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_ROW_COUNT && columnIndex < RAIN_COLUMN_COUNT );

	int groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

	float span = g_rainData.gridCount * g_rainData.gridSize;
	float groupDistance = 1.0f * span / RAIN_COLUMN_COUNT;

	b2Pos position;
	position.x = -0.5f * span + groupDistance * ( columnIndex + 0.5f );
	position.y = 40.0f + 45.0f * rowIndex;

	float scale = 1.0f;
	float jointFriction = 0.05f;
	float jointHertz = 5.0f;
	float jointDamping = 0.5f;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		Human* human = g_rainData.groups[groupIndex].humans + i;
		CreateHuman( human, worldId, position, scale, jointFriction, jointHertz, jointDamping, i + 1, NULL, false );
		position.x += 0.5f;
	}
}

void DestroyGroup( int rowIndex, int columnIndex )
{
	assert( rowIndex < RAIN_ROW_COUNT && columnIndex < RAIN_COLUMN_COUNT );

	int groupIndex = rowIndex * RAIN_COLUMN_COUNT + columnIndex;

	for ( int i = 0; i < RAIN_GROUP_SIZE; ++i )
	{
		DestroyHuman( g_rainData.groups[groupIndex].humans + i );
	}
}

float StepRain( b2WorldId worldId, int stepCount )
{
	int delay = BENCHMARK_DEBUG ? 0x1F : 0x7;

	if ( ( stepCount & delay ) == 0 )
	{
		if ( g_rainData.columnCount < RAIN_COLUMN_COUNT )
		{
			for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
			{
				CreateGroup( worldId, i, g_rainData.columnCount );
			}

			g_rainData.columnCount += 1;
		}
		else
		{
			for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
			{
				DestroyGroup( i, g_rainData.columnIndex );
				CreateGroup( worldId, i, g_rainData.columnIndex );
			}

			g_rainData.columnIndex = ( g_rainData.columnIndex + 1 ) % RAIN_COLUMN_COUNT;
		}
	}

	return 0.0f;
}

#define SPINNER_POINT_COUNT 360

typedef struct
{
	b2JointId spinnerId;
} SpinnerData;

SpinnerData g_spinnerData;

void CreateSpinner( b2WorldId worldId )
{
	b2BodyId groundId;
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		groundId = b2CreateBody( worldId, &bodyDef );

		b2Vec2 points[SPINNER_POINT_COUNT];

		b2Rot q = b2MakeRot( -2.0f * B2_PI / SPINNER_POINT_COUNT );
		b2Vec2 p = { 40.0f, 0.0f };
		for ( int i = 0; i < SPINNER_POINT_COUNT; ++i )
		{
			points[i] = (b2Vec2){ p.x, p.y + 32.0f };
			p = b2RotateVector( q, p );
		}

		b2SurfaceMaterial material = { 0 };
		material.friction = 0.1f;

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = SPINNER_POINT_COUNT;
		chainDef.isLoop = true;
		chainDef.materials = &material;
		chainDef.materialCount = 1;

		b2CreateChain( groundId, &chainDef );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0, 12.0f };
		bodyDef.enableSleep = false;

		b2BodyId spinnerId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeRoundedBox( 0.4f, 20.0f, 0.2f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		b2CreatePolygonShape( spinnerId, &shapeDef, &box );

		float motorSpeed = 5.0f;
		// float maxMotorTorque = 100.0f * 40000.0f;
		float maxMotorTorque = FLT_MAX;
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = spinnerId;
		jointDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, bodyDef.position );
		jointDef.enableMotor = true;
		jointDef.motorSpeed = motorSpeed;
		jointDef.maxMotorTorque = maxMotorTorque;

		g_spinnerData.spinnerId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
	b2Circle circle = { { 0.0f, 0.0f }, 0.35f };
	b2Polygon square = b2MakeSquare( 0.35f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.1f;
	shapeDef.material.restitution = 0.1f;
	shapeDef.density = 0.25f;

	int bodyCount = BENCHMARK_DEBUG ? 499 : 2 * 3038;

	float x = -23.0f, y = 2.0f;
	for ( int i = 0; i < bodyCount; ++i )
	{
		bodyDef.position = (b2Pos){ x, y };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		int remainder = i % 3;
		if ( remainder == 0 )
		{
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
		else if ( remainder == 1 )
		{
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
		else if ( remainder == 2 )
		{
			b2CreatePolygonShape( bodyId, &shapeDef, &square );
		}

		x += 0.5f;

		if ( x >= 23.0f )
		{
			x = -23.0f;
			y += 0.5f;
		}
	}
}

float StepSpinner( b2WorldId worldId, int stepCount )
{
	(void)worldId;
	(void)stepCount;

	return b2RevoluteJoint_GetAngle( g_spinnerData.spinnerId );
}

void CreateSmash( b2WorldId worldId )
{
	b2World_SetGravity( worldId, b2Vec2_zero );

	{
		b2Polygon box = b2MakeBox( 4.0f, 4.0f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ -20.0f, 0.0f };
		bodyDef.linearVelocity = (b2Vec2){ 40.0f, 0.0f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 8.0f;
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}

	float d = 0.4f;
	b2Polygon box = b2MakeSquare( 0.5f * d );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isAwake = false;

	b2ShapeDef shapeDef = b2DefaultShapeDef();

	int columns = BENCHMARK_DEBUG ? 20 : 120;
	int rows = BENCHMARK_DEBUG ? 10 : 80;

	for ( int i = 0; i < columns; ++i )
	{
		for ( int j = 0; j < rows; ++j )
		{
			bodyDef.position.x = i * d + 30.0f;
			bodyDef.position.y = ( j - rows / 2.0f ) * d;
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}
}

void CreateTumbler( b2WorldId worldId )
{
	b2BodyId groundId;
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		groundId = b2CreateBody( worldId, &bodyDef );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Pos){ 0.0f, 10.0f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox( 0.5f, 10.0f, (b2Vec2){ 10.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 0.5f, 10.0f, (b2Vec2){ -10.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 10.0f, 0.5f, (b2Vec2){ 0.0f, 10.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 10.0f, 0.5f, (b2Vec2){ 0.0f, -10.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

		float motorSpeed = 25.0f;

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = groundId;
		jointDef.base.bodyIdB = bodyId;
		jointDef.base.localFrameA.p = (b2Vec2){ 0.0f, 10.0f };
		jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.0f };
		jointDef.motorSpeed = ( B2_PI / 180.0f ) * motorSpeed;
		jointDef.maxMotorTorque = 1e8f;
		jointDef.enableMotor = true;

		b2CreateRevoluteJoint( worldId, &jointDef );
	}

	int gridCount = BENCHMARK_DEBUG ? 20 : 45;

	b2Polygon polygon = b2MakeBox( 0.125f, 0.125f );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	float y = -0.2f * gridCount + 10.0f;
	for ( int i = 0; i < gridCount; ++i )
	{
		float x = -0.2f * gridCount;

		for ( int j = 0; j < gridCount; ++j )
		{
			bodyDef.position = (b2Pos){ x, y };
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

			x += 0.4f;
		}

		y += 0.4f;
	}
}

void CreateWasher( b2WorldId worldId )
{
	bool kinematic = true;

	b2BodyId groundId;
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		groundId = b2CreateBody( worldId, &bodyDef );
	}

	{
		float motorSpeed = 25.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Pos){ 0.0f, 10.0f };

		if ( kinematic == true )
		{
			bodyDef.type = b2_kinematicBody;
			bodyDef.angularVelocity = ( B2_PI / 180.0f ) * motorSpeed;
			bodyDef.linearVelocity = (b2Vec2){ 0.001f, -0.002f };
		}
		else
		{
			bodyDef.type = b2_dynamicBody;
		}

		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float r0 = 14.0f;
		float r1 = 16.0f;
		float r2 = 18.0f;

		float angle = B2_PI / 18.0f;
		b2Rot q = { cosf( angle ), sinf( angle ) };
		b2Rot qo = { cosf( 0.1f * angle ), sinf( 0.1f * angle ) };
		b2Vec2 u1 = { 1.0f, 0.0f };
		for ( int i = 0; i < 36; ++i )
		{
			b2Vec2 u2;
			if ( i == 35 )
			{
				u2 = (b2Vec2){ 1.0f, 0.0f };
			}
			else
			{
				u2 = b2RotateVector( q, u1 );
			}

			{
				b2Vec2 a1 = b2InvRotateVector( qo, u1 );
				b2Vec2 a2 = b2RotateVector( qo, u2 );

				b2Vec2 p1 = b2MulSV( r1, a1 );
				b2Vec2 p2 = b2MulSV( r2, a1 );
				b2Vec2 p3 = b2MulSV( r1, a2 );
				b2Vec2 p4 = b2MulSV( r2, a2 );

				b2Vec2 points[4] = { p1, p2, p3, p4 };
				b2Hull hull = b2ComputeHull( points, 4 );

				b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			}

			if ( i % 9 == 0 )
			{
				b2Vec2 p1 = b2MulSV( r0, u1 );
				b2Vec2 p2 = b2MulSV( r1, u1 );
				b2Vec2 p3 = b2MulSV( r0, u2 );
				b2Vec2 p4 = b2MulSV( r1, u2 );

				b2Vec2 points[4] = { p1, p2, p3, p4 };
				b2Hull hull = b2ComputeHull( points, 4 );

				b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
				b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			}

			u1 = u2;
		}

		if ( kinematic == false )
		{
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = (b2Vec2){ 0.0f, 10.0f };
			jointDef.base.localFrameB.p = (b2Vec2){ 0.0f, 0.0f };
			jointDef.motorSpeed = ( B2_PI / 180.0f ) * motorSpeed;
			jointDef.maxMotorTorque = 1e8f;
			jointDef.enableMotor = true;

			b2CreateRevoluteJoint( worldId, &jointDef );
		}
	}

	int gridCount = BENCHMARK_DEBUG ? 20 : 90;
	float a = 0.1f;

	b2Polygon polygon = b2MakeSquare( a );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.enableHitEvents = true;

	float y = -1.1f * a * gridCount + 10.0f;
	for ( int i = 0; i < gridCount; ++i )
	{
		float x = -1.1f * a * gridCount;

		for ( int j = 0; j < gridCount; ++j )
		{
			bodyDef.position = (b2Pos){ x, y };
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

			x += 2.1f * a;
		}

		y += 2.1f * a;
	}
}

typedef struct
{
	b2BodyId pusherId;
} JunkyardData;

static JunkyardData g_junkyardData;

void CreateJunkyard( b2WorldId worldId )
{
	{
		float gridSize = 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = 0.0f;
		float x = -80.0f * gridSize;
		for ( int i = 0; i < 161; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.55f * gridSize, 0.5f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			x += gridSize;
		}

		y = gridSize;
		x = -80.0f * gridSize;
		for ( int i = 0; i < 50; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			y += gridSize;
		}

		y = gridSize;
		x = 80.0f * gridSize;
		for ( int i = 0; i < 50; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			y += gridSize;
		}
	}

	int columnCount = 200;
	int rowCount = BENCHMARK_DEBUG ? 2 : 40;

	float radius = 0.25f;
	b2Polygon polygon;
	{
		// Fibonacci sphere algorithm
		const float phi = B2_PI * ( sqrtf( 5.0f ) - 1.0f );
		b2Vec2 points[5];

		for ( int i = 0; i < 5; ++i )
		{
			float theta = phi * i;
			b2CosSin cs = b2ComputeCosSin( theta );
			points[i].x = radius * cs.cosine;
			points[i].y = radius * cs.sine;
		}

		b2Hull hull = b2ComputeHull( points, 5 );
		polygon = b2MakePolygon( &hull, 0.0f );
	}

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	float side = -0.1f;
	float yStart = 15.0f;

	for ( int i = 0; i < columnCount; ++i )
	{
		float x = 1.5f * ( 2.0f * i - columnCount ) * radius;

		for ( int j = 0; j < rowCount; ++j )
		{
			float y = 4.0f * j * radius + yStart;

			bodyDef.position = (b2Pos){ x + side, y };
			side = -side;

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		}
	}

	bodyDef.type = b2_kinematicBody;
	bodyDef.position = b2Pos_zero;
	g_junkyardData.pusherId = b2CreateBody( worldId, &bodyDef );
	b2Polygon box = b2MakeOffsetBox( 2.0f, 4.0f, (b2Vec2){ 0.0f, 4.0f }, b2Rot_identity );
	b2CreatePolygonShape( g_junkyardData.pusherId, &shapeDef, &box );
}

float StepJunkyard( b2WorldId worldId, int stepCount )
{
	float timeStep = 1.0f / 60.0f;
	float time = timeStep * stepCount;
	b2CosSin cs = b2ComputeCosSin( 0.2f * time );
	b2WorldTransform target = { (b2Pos){ 60.0f * cs.sine, 0.0f }, b2Rot_identity };
	b2Body_SetTargetTransform( g_junkyardData.pusherId, target, timeStep, true );
	return 0.0f;
}

// Lifted from samples/sample_benchmark.cpp BenchmarkBarrel (e_compoundShape branch).
// Each dynamic body is a compound of two triangular polygon shapes.
void CreateCompounds( b2WorldId worldId )
{
	{
		float gridSize = 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = 0.0f;
		float x = -40.0f * gridSize;
		for ( int i = 0; i < 81; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.55f * gridSize, 0.5f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			x += gridSize;
		}

		y = gridSize;
		x = -40.0f * gridSize;
		for ( int i = 0; i < 100; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			y += gridSize;
		}

		y = gridSize;
		x = 40.0f * gridSize;
		for ( int i = 0; i < 100; ++i )
		{
			b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.55f * gridSize, (b2Vec2){ x, y }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			y += gridSize;
		}

		b2Segment segment = { { -800.0f, -80.0f }, { 800.0f, -80.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	int columnCount = BENCHMARK_DEBUG ? 10 : 20;
	int rowCount = BENCHMARK_DEBUG ? 40 : 150;

	b2Vec2 leftPoints[3] = { { -1.0f, 0.0f }, { 0.5f, 1.0f }, { 0.0f, 2.0f } };
	b2Hull leftHull = b2ComputeHull( leftPoints, 3 );
	b2Polygon left = b2MakePolygon( &leftHull, 0.0f );

	b2Vec2 rightPoints[3] = { { 1.0f, 0.0f }, { -0.5f, 1.0f }, { 0.0f, 2.0f } };
	b2Hull rightHull = b2ComputeHull( rightPoints, 3 );
	b2Polygon right = b2MakePolygon( &rightHull, 0.0f );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.material.friction = 0.5f;

	// Match the sample exactly: centery is computed before shift is reset for the compound branch.
	float shift = 2.0f;
	float extray = 0.25f;
	float side = 0.25f;
	float centerx = shift * columnCount / 2.0f - 1.0f;
	float centery = 1.15f / 2.0f;
	float yStart = 5.0f;

	for ( int i = 0; i < columnCount; ++i )
	{
		float x = i * shift - centerx;

		for ( int j = 0; j < rowCount; ++j )
		{
			float y = j * ( shift + extray ) + centery + yStart;

			bodyDef.position = (b2Pos){ x + side, y };
			side = -side;

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &left );
			b2CreatePolygonShape( bodyId, &shapeDef, &right );
		}
	}
}

// Query benchmark. A sparse grid of static boxes like the cast sample, a pile of dynamic boxes
// settling on a ground inside it with a kinematic paddle keeping some of them awake, and a few
// static sensors in the pile. Every step runs a fixed set of closest rays, circle casts and box
// overlaps, once with the default filter and once with a mask that hits one category in three.
// The queries are precomputed so only the tree work lands in the step time.

#define QUERY_COUNT ( BENCHMARK_DEBUG ? 100 : 1000 )

typedef struct QueryBenchmark
{
	b2Pos origins[1000];
	b2Vec2 translations[1000];
	b2BodyId paddleId;
	b2TreeStats stats;
	float extent;
	float paddleMinX;
	float paddleMaxX;
} QueryBenchmark;

static QueryBenchmark g_queryBenchmark;
static uint32_t g_queryRandomState;

// Repeatable is all this needs to be
static float QueryRandom( float lower, float upper )
{
	g_queryRandomState = 1664525u * g_queryRandomState + 1013904223u;
	float unit = (float)( g_queryRandomState >> 8 ) * ( 1.0f / 16777216.0f );
	return lower + ( upper - lower ) * unit;
}

void CreateQueries( b2WorldId worldId )
{
	g_queryRandomState = 1234;

	float extent = BENCHMARK_DEBUG ? 100.0f : 500.0f;
	int cellCount = BENCHMARK_DEBUG ? 100 : 500;
	float fill = 0.1f;

	// The pile keeps this region of the grid clear
	float pileX = 0.4f * extent;
	float pileY = 0.3f * extent;
	b2AABB clear = { { pileX - 10.0f, pileY - 10.0f }, { pileX + 50.0f, pileY + 60.0f } };

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2ShapeDef shapeDef = b2DefaultShapeDef();

	for ( int i = 0; i < cellCount; ++i )
	{
		float y = (float)i;
		for ( int j = 0; j < cellCount; ++j )
		{
			float x = (float)j;

			float fillTest = QueryRandom( 0.0f, 1.0f );
			float ratio = QueryRandom( 1.0f, 5.0f );
			float halfWidth = QueryRandom( 0.05f, 0.25f );
			float orientation = QueryRandom( 0.0f, 1.0f );
			int category = (int)QueryRandom( 0.0f, 2.999f );

			if ( fillTest > fill )
			{
				continue;
			}

			if ( clear.lowerBound.x <= x && x <= clear.upperBound.x && clear.lowerBound.y <= y && y <= clear.upperBound.y )
			{
				continue;
			}

			bodyDef.position = (b2Pos){ x, y };
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			b2Polygon box = orientation > 0.5f ? b2MakeBox( ratio * halfWidth, halfWidth ) : b2MakeBox( halfWidth, ratio * halfWidth );
			shapeDef.filter.categoryBits = 1ull << category;
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	shapeDef.filter.categoryBits = 1;

	// Ground for the pile
	{
		bodyDef.position = (b2Pos){ 0.0f, 0.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
		b2Segment segment = { { pileX - 10.0f, pileY }, { pileX + 50.0f, pileY } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
	}

	// Static sensors sitting in the pile
	{
		b2ShapeDef sensorDef = b2DefaultShapeDef();
		sensorDef.isSensor = true;
		b2Polygon sensorBox = b2MakeSquare( 2.0f );
		for ( int i = 0; i < 8; ++i )
		{
			bodyDef.position = (b2Pos){ pileX + 4.0f * i, pileY + 2.0f };
			b2BodyId sensorId = b2CreateBody( worldId, &bodyDef );
			b2CreatePolygonShape( sensorId, &sensorDef, &sensorBox );
		}
	}

	// The pile, dropped from just above the ground so it settles quickly
	{
		int columnCount = BENCHMARK_DEBUG ? 20 : 40;
		int rowCount = BENCHMARK_DEBUG ? 25 : 50;
		float spacing = 0.55f;
		b2Polygon box = b2MakeSquare( 0.25f );

		bodyDef.type = b2_dynamicBody;
		for ( int i = 0; i < rowCount; ++i )
		{
			for ( int j = 0; j < columnCount; ++j )
			{
				bodyDef.position = (b2Pos){ pileX + spacing * j, pileY + 1.0f + spacing * i };
				b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	// Kinematic paddle sweeping through the bottom of the pile
	{
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = (b2Pos){ pileX - 5.0f, pileY + 1.2f };
		bodyDef.linearVelocity = (b2Vec2){ 5.0f, 0.0f };
		g_queryBenchmark.paddleId = b2CreateBody( worldId, &bodyDef );
		b2Polygon paddle = b2MakeBox( 1.0f, 1.0f );
		b2CreatePolygonShape( g_queryBenchmark.paddleId, &shapeDef, &paddle );
		g_queryBenchmark.paddleMinX = pileX - 5.0f;
		g_queryBenchmark.paddleMaxX = pileX + 30.0f;
	}

	// Rays and casts cross the whole world, overlaps are centered on the origins
	int queryCount = QUERY_COUNT;
	for ( int i = 0; i < queryCount; ++i )
	{
		float x1 = QueryRandom( 0.0f, extent );
		float y1 = QueryRandom( 0.0f, extent );
		float x2 = QueryRandom( 0.0f, extent );
		float y2 = QueryRandom( 0.0f, extent );
		g_queryBenchmark.origins[i] = (b2Pos){ x1, y1 };
		g_queryBenchmark.translations[i] = (b2Vec2){ x2 - x1, y2 - y1 };
	}

	g_queryBenchmark.stats = (b2TreeStats){ 0 };
	g_queryBenchmark.extent = extent;
}

static float QueryCastClosest( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	(void)shapeId;
	(void)point;
	(void)normal;
	*(float*)context = fraction;
	return fraction;
}

static bool QueryOverlapCount( b2ShapeId shapeId, void* context )
{
	(void)shapeId;
	*(int*)context += 1;
	return true;
}

float StepQueries( b2WorldId worldId, int stepCount )
{
	(void)stepCount;

	QueryBenchmark* data = &g_queryBenchmark;

	// Reverse the paddle at the ends of its sweep
	b2Pos paddlePosition = b2Body_GetPosition( data->paddleId );
	if ( paddlePosition.x > data->paddleMaxX )
	{
		b2Body_SetLinearVelocity( data->paddleId, (b2Vec2){ -5.0f, 0.0f } );
	}
	else if ( paddlePosition.x < data->paddleMinX )
	{
		b2Body_SetLinearVelocity( data->paddleId, (b2Vec2){ 5.0f, 0.0f } );
	}

	b2QueryFilter filters[2] = { b2DefaultQueryFilter(), b2DefaultQueryFilter() };
	filters[1].maskBits = 1;

	b2Vec2 circleCenter = b2Vec2_zero;
	b2ShapeProxy circle = b2MakeProxy( &circleCenter, 1, 0.1f );

	int queryCount = QUERY_COUNT;
	int hitCount = 0;
	b2TreeStats stats = { 0 };

	for ( int f = 0; f < 2; ++f )
	{
		b2QueryFilter filter = filters[f];

		for ( int i = 0; i < queryCount; ++i )
		{
			b2RayResult result = b2World_CastRayClosest( worldId, data->origins[i], data->translations[i], filter );
			stats.nodeVisits += result.nodeVisits;
			stats.leafVisits += result.leafVisits;
			hitCount += result.hit ? 1 : 0;
		}

		for ( int i = 0; i < queryCount; ++i )
		{
			float fraction = 1.0f;
			b2TreeStats castStats =
				b2World_CastShape( worldId, data->origins[i], &circle, data->translations[i], filter, QueryCastClosest, &fraction );
			stats.nodeVisits += castStats.nodeVisits;
			stats.leafVisits += castStats.leafVisits;
			hitCount += fraction < 1.0f ? 1 : 0;
		}

		for ( int i = 0; i < queryCount; ++i )
		{
			int overlapCount = 0;
			b2AABB aabb = { { -5.0f, -5.0f }, { 5.0f, 5.0f } };
			b2TreeStats overlapStats = b2World_OverlapAABB( worldId, data->origins[i], aabb, filter, QueryOverlapCount, &overlapCount );
			stats.nodeVisits += overlapStats.nodeVisits;
			stats.leafVisits += overlapStats.leafVisits;
			hitCount += overlapCount;
		}
	}

	data->stats.nodeVisits += stats.nodeVisits;
	data->stats.leafVisits += stats.leafVisits;

	return (float)hitCount;
}

b2TreeStats GetQueryBenchmarkStats( void )
{
	return g_queryBenchmark.stats;
}

int GetQueryBenchmarkCount( void )
{
	return QUERY_COUNT;
}

float GetQueryBenchmarkExtent( void )
{
	return g_queryBenchmark.extent;
}

void GetQueryBenchmarkRay( int index, b2Pos* origin, b2Vec2* translation )
{
	*origin = g_queryBenchmark.origins[index];
	*translation = g_queryBenchmark.translations[index];
}

// Tree cast benchmark. The queries grid goes into a bare dynamic tree one proxy at a time and is
// never rebuilt, which is the tree a game runs on. Every step casts the same rays through
// b2DynamicTree_RayCast at full extent, at 25 units and at 5 units, with the default mask and with
// a mask that hits one category in three, then the same directions as box casts. The callback
// clips to the tight box, so hits and fraction sums match across builds and nothing but the tree
// is in the step time. Full extent rays shrink the fraction as they go, 25 unit rays sit at the
// cache edge of this tree, 5 unit rays are the game case.

#define TREE_CAST_RAY_COUNT ( BENCHMARK_DEBUG ? 100 : 1000 )
#define TREE_CAST_LENGTH_COUNT 3

// Static proxies get the speculative distance twice, once in the shape box and once as the margin
#define TREE_CAST_MARGIN ( 2.0f * B2_SPECULATIVE_DISTANCE )

// The queries benchmark casts a circle of this radius, the tree sees its box
#define TREE_CAST_BOX_EXTENT 0.1f

typedef struct TreeCastBenchmark
{
	b2DynamicTree tree;
	b2AABB* tightBoxes;
	int proxyCount;
	int proxyCapacity;
	b2Vec2 origins[1000];
	b2Vec2 translations[TREE_CAST_LENGTH_COUNT][1000];
	b2TreeStats stats;
	bool created;
} TreeCastBenchmark;

typedef struct TreeCastContext
{
	const b2AABB* tightBoxes;
	float fraction;
} TreeCastContext;

static TreeCastBenchmark g_treeCast;

// Entry fraction of a ray into a box, negative for a miss. An origin inside or on the box is a miss,
// which matches the polygon cast the world runs, and zero would end the traversal.
static float TreeCastRayBox( b2AABB box, b2Vec2 p, b2Vec2 d, float maxFraction )
{
	float tmin = 0.0f;
	float tmax = maxFraction;

	if ( d.x == 0.0f )
	{
		if ( p.x < box.lowerBound.x || box.upperBound.x < p.x )
		{
			return -1.0f;
		}
	}
	else
	{
		float inv = 1.0f / d.x;
		float t1 = ( box.lowerBound.x - p.x ) * inv;
		float t2 = ( box.upperBound.x - p.x ) * inv;
		if ( t1 > t2 )
		{
			float t = t1;
			t1 = t2;
			t2 = t;
		}

		tmin = b2MaxFloat( tmin, t1 );
		tmax = b2MinFloat( tmax, t2 );
		if ( tmin > tmax )
		{
			return -1.0f;
		}
	}

	if ( d.y == 0.0f )
	{
		if ( p.y < box.lowerBound.y || box.upperBound.y < p.y )
		{
			return -1.0f;
		}
	}
	else
	{
		float inv = 1.0f / d.y;
		float t1 = ( box.lowerBound.y - p.y ) * inv;
		float t2 = ( box.upperBound.y - p.y ) * inv;
		if ( t1 > t2 )
		{
			float t = t1;
			t1 = t2;
			t2 = t;
		}

		tmin = b2MaxFloat( tmin, t1 );
		tmax = b2MinFloat( tmax, t2 );
		if ( tmin > tmax )
		{
			return -1.0f;
		}
	}

	if ( tmin <= 0.0f )
	{
		return -1.0f;
	}

	return tmin;
}

static float TreeCastRayCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;
	TreeCastContext* cast = context;
	float fraction = TreeCastRayBox( cast->tightBoxes[userData], input->origin, input->translation, input->maxFraction );
	if ( fraction < 0.0f )
	{
		return -1.0f;
	}

	cast->fraction = fraction;
	return fraction;
}

// A box sweep against a box is a ray from the cast box center against the target grown by the extents
static float TreeCastBoxCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;
	TreeCastContext* cast = context;
	b2AABB tight = cast->tightBoxes[userData];
	b2Vec2 extents = b2AABB_Extents( input->box );
	b2AABB grown = { b2Sub( tight.lowerBound, extents ), b2Add( tight.upperBound, extents ) };
	float fraction = TreeCastRayBox( grown, b2AABB_Center( input->box ), input->translation, input->maxFraction );
	if ( fraction < 0.0f )
	{
		return -1.0f;
	}

	cast->fraction = fraction;
	return fraction;
}

static void TreeCastAddBox( b2Vec2 center, float hx, float hy, uint64_t categoryBits )
{
	TreeCastBenchmark* data = &g_treeCast;
	assert( data->proxyCount < data->proxyCapacity );

	b2AABB tight = { { center.x - hx, center.y - hy }, { center.x + hx, center.y + hy } };
	b2AABB fat = {
		{ tight.lowerBound.x - TREE_CAST_MARGIN, tight.lowerBound.y - TREE_CAST_MARGIN },
		{ tight.upperBound.x + TREE_CAST_MARGIN, tight.upperBound.y + TREE_CAST_MARGIN },
	};

	int index = data->proxyCount;
	data->tightBoxes[index] = tight;
	data->proxyCount += 1;
	b2DynamicTree_CreateProxyInternal( &data->tree, fat, categoryBits, (uint64_t)index, false );
}

void DestroyTreeCast( void )
{
	TreeCastBenchmark* data = &g_treeCast;
	if ( data->created )
	{
		b2DynamicTree_Destroy( &data->tree );
		free( data->tightBoxes );
	}

	// The harness destroys after each run and prints the stats after the last one
	b2TreeStats stats = data->stats;
	memset( data, 0, sizeof( TreeCastBenchmark ) );
	data->stats = stats;
}

void CreateTreeCast( b2WorldId worldId )
{
	(void)worldId;
	DestroyTreeCast();

	TreeCastBenchmark* data = &g_treeCast;

	// Same generator and seed as the queries benchmark so the grid matches it
	g_queryRandomState = 1234;

	float extent = BENCHMARK_DEBUG ? 100.0f : 500.0f;
	int cellCount = BENCHMARK_DEBUG ? 100 : 500;
	float fill = 0.1f;

	// The pile keeps this region of the grid clear
	float pileX = 0.4f * extent;
	float pileY = 0.3f * extent;
	b2AABB clear = { { pileX - 10.0f, pileY - 10.0f }, { pileX + 50.0f, pileY + 60.0f } };

	// A tenth of the cells fill, the ground and the sensors come after
	data->proxyCapacity = cellCount * cellCount / 5;
	data->tightBoxes = malloc( data->proxyCapacity * sizeof( b2AABB ) );
	data->tree = b2DynamicTree_Create( data->proxyCapacity );
	data->created = true;

	for ( int i = 0; i < cellCount; ++i )
	{
		float y = (float)i;
		for ( int j = 0; j < cellCount; ++j )
		{
			float x = (float)j;

			float fillTest = QueryRandom( 0.0f, 1.0f );
			float ratio = QueryRandom( 1.0f, 5.0f );
			float halfWidth = QueryRandom( 0.05f, 0.25f );
			float orientation = QueryRandom( 0.0f, 1.0f );
			int category = (int)QueryRandom( 0.0f, 2.999f );

			if ( fillTest > fill )
			{
				continue;
			}

			if ( clear.lowerBound.x <= x && x <= clear.upperBound.x && clear.lowerBound.y <= y && y <= clear.upperBound.y )
			{
				continue;
			}

			float hx = orientation > 0.5f ? ratio * halfWidth : halfWidth;
			float hy = orientation > 0.5f ? halfWidth : ratio * halfWidth;
			TreeCastAddBox( (b2Vec2){ x, y }, hx, hy, 1ull << category );
		}
	}

	// Ground for the pile, a degenerate box
	TreeCastAddBox( (b2Vec2){ pileX + 20.0f, pileY }, 30.0f, 0.0f, 1 );

	// Static sensors sitting in the pile
	for ( int i = 0; i < 8; ++i )
	{
		TreeCastAddBox( (b2Vec2){ pileX + 4.0f * i, pileY + 2.0f }, 2.0f, 2.0f, 1 );
	}

	// Rays cross the whole world, then the same directions cut to the shorter lengths
	float lengths[TREE_CAST_LENGTH_COUNT] = { 0.0f, 25.0f, 5.0f };
	for ( int i = 0; i < TREE_CAST_RAY_COUNT; ++i )
	{
		float x1 = QueryRandom( 0.0f, extent );
		float y1 = QueryRandom( 0.0f, extent );
		float x2 = QueryRandom( 0.0f, extent );
		float y2 = QueryRandom( 0.0f, extent );
		b2Vec2 t = { x2 - x1, y2 - y1 };
		data->origins[i] = (b2Vec2){ x1, y1 };
		data->translations[0][i] = t;

		float length = b2Length( t );
		for ( int k = 1; k < TREE_CAST_LENGTH_COUNT; ++k )
		{
			float scale = length > 0.0f ? lengths[k] / length : 0.0f;
			data->translations[k][i] = b2MulSV( scale, t );
		}
	}

	data->stats = (b2TreeStats){ 0 };
}

float StepTreeCast( b2WorldId worldId, int stepCount )
{
	(void)worldId;
	(void)stepCount;

	TreeCastBenchmark* data = &g_treeCast;
	const b2DynamicTree* tree = &data->tree;
	uint64_t masks[2] = { B2_DEFAULT_MASK_BITS, 1 };
	int rayCount = TREE_CAST_RAY_COUNT;
	b2TreeStats stats = { 0 };
	int hitCount = 0;

	for ( int k = 0; k < TREE_CAST_LENGTH_COUNT; ++k )
	{
		const b2Vec2* translations = data->translations[k];

		for ( int m = 0; m < 2; ++m )
		{
			for ( int i = 0; i < rayCount; ++i )
			{
				b2RayCastInput input = { data->origins[i], translations[i], 1.0f };
				TreeCastContext context = { data->tightBoxes, 1.0f };
				b2TreeStats castStats = b2DynamicTree_CastRay( tree, &input, masks[m], TreeCastRayCallback, &context );
				stats.nodeVisits += castStats.nodeVisits;
				stats.leafVisits += castStats.leafVisits;
				hitCount += context.fraction < 1.0f ? 1 : 0;
			}
		}

		for ( int i = 0; i < rayCount; ++i )
		{
			b2Vec2 o = data->origins[i];
			b2AABB box = { { o.x - TREE_CAST_BOX_EXTENT, o.y - TREE_CAST_BOX_EXTENT },
						   { o.x + TREE_CAST_BOX_EXTENT, o.y + TREE_CAST_BOX_EXTENT } };
			b2BoxCastInput input = { box, translations[i], 1.0f };
			TreeCastContext context = { data->tightBoxes, 1.0f };
			b2TreeStats castStats = b2DynamicTree_CastBox( tree, &input, B2_DEFAULT_MASK_BITS, TreeCastBoxCallback, &context );
			stats.nodeVisits += castStats.nodeVisits;
			stats.leafVisits += castStats.leafVisits;
			hitCount += context.fraction < 1.0f ? 1 : 0;
		}
	}

	data->stats.nodeVisits += stats.nodeVisits;
	data->stats.leafVisits += stats.leafVisits;

	return (float)hitCount;
}

b2TreeStats GetTreeCastBenchmarkStats( void )
{
	return g_treeCast.stats;
}

// Tile world benchmark. The TileWorld sample's terrain, a wall of rounded boxes whose height follows
// a sine, on a static body every ten columns, with a box stack every twelfth cycle. Each step streams
// the terrain, the leftmost ground body is destroyed and a new one continues the sine on the right,
// then short rays and box overlaps are cast across the live span. A large static tree under churn,
// the shape of a level that streams.

#define TILE_PERIOD 40.0f
#define TILE_CYCLE_COUNT ( BENCHMARK_DEBUG ? 10 : 600 )
#define TILE_GRID_SIZE 1.0f
#define TILE_COLUMNS_PER_BODY 10
#define TILE_QUERY_COUNT ( BENCHMARK_DEBUG ? 50 : 500 )
#define TILE_RAY_LENGTH 25.0f

typedef struct TileWorldBenchmark
{
	b2BodyId* groundIds;
	int groundCount;
	int groundHead;
	float bodySpan;
	float xLeft;
	float xNext;
	b2TreeStats stats;
} TileWorldBenchmark;

static TileWorldBenchmark g_tileWorld;

static float TileRandom( uint32_t* state, float lower, float upper )
{
	*state = 1664525u * *state + 1013904223u;
	float unit = (float)( *state >> 8 ) * ( 1.0f / 16777216.0f );
	return lower + ( upper - lower ) * unit;
}

// Ten columns on one body, the column height from the sine at the column's world x
static b2BodyId CreateTileGround( b2WorldId worldId, float xBody )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.position.x = xBody;
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.invokeContactCreation = false;

	float omega = 2.0f * B2_PI / TILE_PERIOD;
	float xShape = 0.0f;
	for ( int i = 0; i < TILE_COLUMNS_PER_BODY; ++i )
	{
		int columnCount = (int)roundf( 4.0f * cosf( omega * ( xBody + xShape ) ) ) + 12;
		float y = 0.0f;
		for ( int j = 0; j < columnCount; ++j )
		{
			b2Polygon square =
				b2MakeOffsetBox( 0.4f * TILE_GRID_SIZE, 0.4f * TILE_GRID_SIZE, (b2Vec2){ xShape, y }, b2Rot_identity );
			square.radius = 0.1f;
			b2CreatePolygonShape( groundId, &shapeDef, &square );
			y += TILE_GRID_SIZE;
		}

		xShape += TILE_GRID_SIZE;
	}

	return groundId;
}

void DestroyTileWorld( void )
{
	TileWorldBenchmark* data = &g_tileWorld;
	free( data->groundIds );

	// The harness destroys after each run and prints the stats after the last one
	b2TreeStats stats = data->stats;
	memset( data, 0, sizeof( TileWorldBenchmark ) );
	data->stats = stats;
}

void CreateTileWorld( b2WorldId worldId )
{
	DestroyTileWorld();

	TileWorldBenchmark* data = &g_tileWorld;

	int gridCount = (int)( TILE_CYCLE_COUNT * TILE_PERIOD / TILE_GRID_SIZE );
	data->groundCount = gridCount / TILE_COLUMNS_PER_BODY;
	data->groundIds = malloc( data->groundCount * sizeof( b2BodyId ) );
	data->bodySpan = TILE_COLUMNS_PER_BODY * TILE_GRID_SIZE;
	data->xLeft = -0.5f * ( TILE_CYCLE_COUNT * TILE_PERIOD );
	data->xNext = data->xLeft;
	data->groundHead = 0;

	for ( int i = 0; i < data->groundCount; ++i )
	{
		data->groundIds[i] = CreateTileGround( worldId, data->xNext );
		data->xNext += data->bodySpan;
	}

	// Box stacks in the valleys of every twelfth cycle, past the fifth of the span that streams out
	// during the run
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeBox( 0.3f, 0.2f );

	for ( int cycleIndex = TILE_CYCLE_COUNT / 5; cycleIndex < TILE_CYCLE_COUNT; cycleIndex += 12 )
	{
		float xBase = ( 0.5f + cycleIndex ) * TILE_PERIOD + data->xLeft;
		bodyDef.position.x = xBase - 3.0f;
		for ( int i = 0; i < 10; ++i )
		{
			bodyDef.position.y = 10.0f;
			for ( int j = 0; j < 5; ++j )
			{
				b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
				bodyDef.position.y += 0.5f;
			}

			bodyDef.position.x += 0.6f;
		}
	}
}

static bool TileOverlapCount( b2ShapeId shapeId, void* context )
{
	(void)shapeId;
	*(int*)context += 1;
	return true;
}

float StepTileWorld( b2WorldId worldId, int stepCount )
{
	TileWorldBenchmark* data = &g_tileWorld;

	// Stream one ground body, the leftmost goes and the sine continues on the right
	b2DestroyBody( data->groundIds[data->groundHead] );
	data->groundIds[data->groundHead] = CreateTileGround( worldId, data->xNext );
	data->groundHead = ( data->groundHead + 1 ) % data->groundCount;
	data->xNext += data->bodySpan;
	data->xLeft += data->bodySpan;

	// A fresh set of short rays and box overlaps across the live span each step
	uint32_t state = 1234u + 7919u * (uint32_t)stepCount;
	float span = data->groundCount * data->bodySpan;
	b2QueryFilter filter = b2DefaultQueryFilter();
	b2AABB aabb = { { -5.0f, -5.0f }, { 5.0f, 5.0f } };
	b2TreeStats stats = { 0 };
	int hitCount = 0;

	for ( int i = 0; i < TILE_QUERY_COUNT; ++i )
	{
		b2Pos origin = { data->xLeft + TileRandom( &state, 0.0f, span ), TileRandom( &state, -2.0f, 18.0f ) };
		float angle = TileRandom( &state, 0.0f, 2.0f * B2_PI );
		b2Vec2 translation = { TILE_RAY_LENGTH * cosf( angle ), TILE_RAY_LENGTH * sinf( angle ) };

		b2RayResult result = b2World_CastRayClosest( worldId, origin, translation, filter );
		stats.nodeVisits += result.nodeVisits;
		stats.leafVisits += result.leafVisits;
		hitCount += result.hit ? 1 : 0;

		int overlapCount = 0;
		b2TreeStats overlapStats = b2World_OverlapAABB( worldId, origin, aabb, filter, TileOverlapCount, &overlapCount );
		stats.nodeVisits += overlapStats.nodeVisits;
		stats.leafVisits += overlapStats.leafVisits;
		hitCount += overlapCount;
	}

	data->stats.nodeVisits += stats.nodeVisits;
	data->stats.leafVisits += stats.leafVisits;

	return (float)hitCount;
}

b2TreeStats GetTileWorldBenchmarkStats( void )
{
	return g_tileWorld.stats;
}
