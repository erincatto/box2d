// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

b2WorldId Tumbler( b2WorldDef* worldDef )
{
	b2WorldId worldId = b2CreateWorld( worldDef );

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
		jd.motorSpeed = ( b2_pi / 180.0f ) * motorSpeed;
		jd.maxMotorTorque = 1e8f;
		jd.enableMotor = true;

		b2CreateRevoluteJoint( worldId, &jd );
	}

#ifdef NDEBUG
	int gridCount = 45;
#else
	int gridCount = 20;
#endif

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

	return worldId;
}
