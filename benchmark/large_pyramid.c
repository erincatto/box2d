// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

b2WorldId LargePyramid( b2WorldDef* worldDef )
{
#ifdef NDEBUG
	int baseCount = 100;
#else
	int baseCount = 20;
#endif

	b2WorldId worldId = b2CreateWorld( worldDef );

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

	return worldId;
}
