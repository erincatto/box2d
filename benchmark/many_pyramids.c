// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

static void CreatePyramid( b2WorldId worldId, int baseCount, float extent, float centerX, float baseY )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	// bodyDef.enableSleep = false;

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

b2WorldId ManyPyramids( b2WorldDef* worldDef )
{
	int baseCount = 10;
	float extent = 0.5f;
#ifdef NDEBUG
	int rowCount = 20;
	int columnCount = 20;
#else
	int rowCount = 5;
	int columnCount = 5;
#endif

	worldDef->enableSleep = false;

	b2WorldId worldId = b2CreateWorld( worldDef );

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
			CreatePyramid( worldId, baseCount, extent, centerX, baseY );
		}

		baseY += groundDeltaY;
	}

	return worldId;
}
