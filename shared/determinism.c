// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"

#include "box2d/box2d.h"

#include <assert.h>
#include <stdlib.h>

FallingHingeData CreateFallingHinges( b2WorldId worldId )
{
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = (b2Vec2){ 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 20.0f, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );
	}

	int columnCount = 4;
	int rowCount = 30;
	int bodyCount = rowCount * columnCount;

	b2BodyId* bodyIds = calloc( bodyCount, sizeof( b2BodyId ) );

	float h = 0.25f;
	float r = 0.1f * h;
	b2Polygon box = b2MakeRoundedBox( h - r, h - r, r );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.3f;

	float offset = 0.4f * h;
	float dx = 10.0f * h;
	float xroot = -0.5f * dx * ( columnCount - 1.0f );

	b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
	jointDef.enableLimit = true;
	jointDef.lowerAngle = -0.1f * B2_PI;
	jointDef.upperAngle = 0.2f * B2_PI;
	jointDef.enableSpring = true;
	jointDef.hertz = 0.5f;
	jointDef.dampingRatio = 0.5f;
	jointDef.localAnchorA = (b2Vec2){ h, h };
	jointDef.localAnchorB = (b2Vec2){ offset, -h };
	jointDef.drawSize = 0.1f;

	int bodyIndex = 0;

	for ( int j = 0; j < columnCount; ++j )
	{
		float x = xroot + j * dx;

		b2BodyId prevBodyId = b2_nullBodyId;

		for ( int i = 0; i < rowCount; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			bodyDef.position.x = x + offset * i;
			bodyDef.position.y = h + 2.0f * h * i;

			// this tests the deterministic cosine and sine functions
			bodyDef.rotation = b2MakeRot( 0.1f * i - 1.0f );

			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			if ( ( i & 1 ) == 0 )
			{
				prevBodyId = bodyId;
			}
			else
			{
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				b2CreateRevoluteJoint( worldId, &jointDef );
				prevBodyId = b2_nullBodyId;
			}

			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			assert( bodyIndex < bodyCount );
			bodyIds[bodyIndex] = bodyId;

			bodyIndex += 1;
		}
	}

	assert( bodyIndex == bodyCount );

	FallingHingeData data = {
		.bodyIds = bodyIds,
		.bodyCount = bodyCount,
		.stepCount = 0,
		.sleepStep = -1,
		.hash = 0,
	};
	return data;
}

bool UpdateFallingHinges( b2WorldId worldId, FallingHingeData* data )
{
	if ( data->hash == 0 )
	{
		b2BodyEvents bodyEvents = b2World_GetBodyEvents( worldId );

		if ( bodyEvents.moveCount == 0 )
		{
			int awakeCount = b2World_GetAwakeBodyCount( worldId );
			assert( awakeCount == 0 );

			data->hash = B2_HASH_INIT;
			for ( int i = 0; i < data->bodyCount; ++i )
			{
				b2Transform xf = b2Body_GetTransform( data->bodyIds[i] );
				data->hash = b2Hash( data->hash, (uint8_t*)( &xf ), sizeof( b2Transform ) );
			}

			data->sleepStep = data->stepCount;
		}
	}

	data->stepCount += 1;

	return data->hash != 0;
}

void DestroyFallingHinges( FallingHingeData* data )
{
	free( data->bodyIds );
	data->bodyIds = NULL;
}