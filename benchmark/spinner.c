// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

enum
{
	e_count = 3038,
	e_pointCount = 360,
};

b2WorldId Spinner( b2WorldDef* worldDef )
{
	b2WorldId worldId = b2CreateWorld( worldDef );

		b2BodyId groundId;
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		groundId = b2CreateBody( worldId, &bodyDef );

		b2Vec2 points[e_pointCount];

		b2Rot q = b2MakeRot( -2.0f * b2_pi / e_pointCount );
		b2Vec2 p = { 40.0f, 0.0f };
		for ( int i = 0; i < e_pointCount; ++i )
		{
			points[i] = (b2Vec2){ p.x, p.y + 32.0f };
			p = b2RotateVector( q, p );
		}

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = e_pointCount;
		chainDef.isLoop = true;
		chainDef.friction = 0.1f;

		b2CreateChain( groundId, &chainDef );
	}

	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = (b2Vec2){ 0.0, 12.0f };
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

#ifdef NDEBUG
	int bodyCount = e_count;
#else
	int bodyCount = 499;
#endif

	float x = -24.0f, y = 2.0f;
	for ( int i = 0; i < bodyCount; ++i )
	{
		bodyDef.position = (b2Vec2){ x, y };
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

	return worldId;
}
