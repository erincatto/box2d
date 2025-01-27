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

	int N = BENCHMARK_DEBUG ? 10 : 100;

	// Allocate to avoid huge stack usage
	b2BodyId* bodies = malloc( N * N * sizeof( b2BodyId ) );
	int index = 0;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ~2u;

	b2Circle circle = { { 0.0f, 0.0f }, 0.4f };

	b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
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

			bodyDef.position = ( b2Vec2 ){ fk, -fi };

			b2BodyId body = b2CreateBody( worldId, &bodyDef );

			b2CreateCircleShape( body, &shapeDef, &circle );

			if ( i > 0 )
			{
				jd.bodyIdA = bodies[index - 1];
				jd.bodyIdB = body;
				jd.localAnchorA = ( b2Vec2 ){ 0.0f, -0.5f };
				jd.localAnchorB = ( b2Vec2 ){ 0.0f, 0.5f };
				b2CreateRevoluteJoint( worldId, &jd );
			}

			if ( k > 0 )
			{
				jd.bodyIdA = bodies[index - N];
				jd.bodyIdB = body;
				jd.localAnchorA = ( b2Vec2 ){ 0.5f, 0.0f };
				jd.localAnchorB = ( b2Vec2 ){ -0.5f, 0.0f };
				b2CreateRevoluteJoint( worldId, &jd );
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
		bodyDef.position = ( b2Vec2 ){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 100.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.enableSleep = false;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;

	float h = 0.5f;
	b2Polygon box = b2MakeSquare( h );

	float shift = 1.0f * h;

	for ( int i = 0; i < baseCount; ++i )
	{
		float y = ( 2.0f * i + 1.0f ) * shift;

		for ( int j = i; j < baseCount; ++j )
		{
			float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;

			bodyDef.position = ( b2Vec2 ){ x, y };

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
			bodyDef.position = ( b2Vec2 ){ x, y };

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
		b2Segment segment = { { -0.5f * 2.0f * groundWidth, groundY }, { 0.5f * 2.0f * groundWidth, groundY } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		groundY += groundDeltaY;
	}

	float baseWidth = 2.0f * extent * baseCount;
	float baseY = 0.0f;

	for ( int i = 0; i < rowCount; ++i )
	{
		for ( int j = 0; j < columnCount; ++j )
		{
			float centerX = -0.5f * groundWidth + j * ( baseWidth + 2.0f * extent ) + extent;
			CreateSmallPyramid( worldId, baseCount, extent, centerX, baseY );
		}

		baseY += groundDeltaY;
	}
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
				b2Polygon box = b2MakeOffsetBox( 0.5f * width, 0.5f * height, ( b2Vec2 ){ x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );

				//b2Segment segment = { { x - 0.5f * width, y }, { x + 0.5f * width, y } };
				//b2CreateSegmentShape( groundId, &shapeDef, &segment );

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

	b2Vec2 position;
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
			points[i] = ( b2Vec2 ){ p.x, p.y + 32.0f };
			p = b2RotateVector( q, p );
		}

		b2SurfaceMaterial material = {0};
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
		bodyDef.position = ( b2Vec2 ){ 0.0, 12.0f };
		bodyDef.enableSleep = false;

		b2BodyId spinnerId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeRoundedBox( 0.4f, 20.0f, 0.2f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.0f;
		b2CreatePolygonShape( spinnerId, &shapeDef, &box );

		float motorSpeed = 5.0f;
		float maxMotorTorque = 40000.0f;
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = groundId;
		jointDef.bodyIdB = spinnerId;
		jointDef.localAnchorA = bodyDef.position;
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
	shapeDef.friction = 0.1f;
	shapeDef.restitution = 0.1f;
	shapeDef.density = 0.25f;

	int bodyCount = BENCHMARK_DEBUG ? 499 : 3038;

	float x = -24.0f, y = 2.0f;
	for ( int i = 0; i < bodyCount; ++i )
	{
		bodyDef.position = ( b2Vec2 ){ x, y };
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

		x += 1.0f;

		if ( x > 24.0f )
		{
			x = -24.0f;
			y += 1.0f;
		}
	}
}

float StepSpinner( b2WorldId worldId, int stepCount )
{
	(void)worldId;
	(void)stepCount;

	return b2RevoluteJoint_GetAngle(g_spinnerData.spinnerId);
}

void CreateSmash( b2WorldId worldId )
{
	b2World_SetGravity( worldId, b2Vec2_zero );

	{
		b2Polygon box = b2MakeBox( 4.0f, 4.0f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = ( b2Vec2 ){ -20.0f, 0.0f };
		bodyDef.linearVelocity = ( b2Vec2 ){ 40.0f, 0.0f };
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
		bodyDef.position = ( b2Vec2 ){ 0.0f, 10.0f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox( 0.5f, 10.0f, ( b2Vec2 ){ 10.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 0.5f, 10.0f, ( b2Vec2 ){ -10.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 10.0f, 0.5f, ( b2Vec2 ){ 0.0f, 10.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 10.0f, 0.5f, ( b2Vec2 ){ 0.0f, -10.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

		float motorSpeed = 25.0f;

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		jd.bodyIdA = groundId;
		jd.bodyIdB = bodyId;
		jd.localAnchorA = ( b2Vec2 ){ 0.0f, 10.0f };
		jd.localAnchorB = ( b2Vec2 ){ 0.0f, 0.0f };
		jd.referenceAngle = 0.0f;
		jd.motorSpeed = ( B2_PI / 180.0f ) * motorSpeed;
		jd.maxMotorTorque = 1e8f;
		jd.enableMotor = true;

		b2CreateRevoluteJoint( worldId, &jd );
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
			bodyDef.position = ( b2Vec2 ){ x, y };
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

			x += 0.4f;
		}

		y += 0.4f;
	}
}
