// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

b2WorldId Smash( b2WorldDef* worldDef )
{
	b2WorldId worldId = b2CreateWorld( worldDef );

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

#ifdef NDEBUG
	int columns = 120;
	int rows = 80;
#else
	int columns = 20;
	int rows = 10;
#endif

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

	return worldId;
}
