// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"

#include "box2d/box2d.h"

#include <assert.h>
#include <string.h>

#ifdef NDEBUG
#define BENCHMARK_DEBUG 0
#else
#define BENCHMARK_DEBUG 1
#endif

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
		float w = 0.5f * g_rainData.gridSize;
		float h = 0.5f * g_rainData.gridSize;

		for ( int i = 0; i < RAIN_ROW_COUNT; ++i )
		{
			float x = -0.5f * g_rainData.gridCount * g_rainData.gridSize;
			for ( int j = 0; j <= g_rainData.gridCount; ++j )
			{
				b2Polygon box = b2MakeOffsetBox( w, h, ( b2Vec2 ){ x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
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

void StepRain( b2WorldId worldId, int stepCount )
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
}
