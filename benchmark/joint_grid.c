// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <stdlib.h>

b2WorldId JointGrid( b2WorldDef* worldDef )
{
	// Turning gravity off to isolate joint performance.
	worldDef->gravity = b2Vec2_zero;

	b2WorldId worldId = b2CreateWorld( worldDef );

#ifdef NDEBUG
	int N = 100;
#else
	int N = 10;
#endif

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
	bodyDef.enableSleep = false;

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

	return worldId;
}
