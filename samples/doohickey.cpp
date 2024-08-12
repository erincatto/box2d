// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "doohickey.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Doohickey::Doohickey()
{
	m_wheelId1 = {};
	m_wheelId2 = {};
	m_barId1 = {};
	m_barId2 = {};

	m_axleId1 = {};
	m_axleId2 = {};
	m_sliderId = {};

	m_isSpawned = false;
}

void Doohickey::Spawn( b2WorldId worldId, b2Vec2 position, float scale )
{
	assert( m_isSpawned == false );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Circle circle = { { 0.0f, 0.0f }, 1.0f * scale };
	b2Capsule capsule = { { -3.5f * scale, 0.0f }, { 3.5f * scale, 0.0f }, 0.15f * scale };

	bodyDef.position = b2MulAdd( position, scale, { -5.0f, 3.0f } );
	m_wheelId1 = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_wheelId1, &shapeDef, &circle );

	bodyDef.position = b2MulAdd( position, scale, { 5.0f, 3.0f } );
	m_wheelId2 = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_wheelId2, &shapeDef, &circle );

	bodyDef.position = b2MulAdd( position, scale, { -1.5f, 3.0f } );
	m_barId1 = b2CreateBody( worldId, &bodyDef );
	b2CreateCapsuleShape( m_barId1, &shapeDef, &capsule );

	bodyDef.position = b2MulAdd( position, scale, { 1.5f, 3.0f } );
	m_barId2 = b2CreateBody( worldId, &bodyDef );
	b2CreateCapsuleShape( m_barId2, &shapeDef, &capsule );

	b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();

	revoluteDef.bodyIdA = m_wheelId1;
	revoluteDef.bodyIdB = m_barId1;
	revoluteDef.localAnchorA = { 0.0f, 0.0f };
	revoluteDef.localAnchorB = { -3.5f * scale, 0.0f };
	revoluteDef.enableMotor = true;
	revoluteDef.maxMotorTorque = 2.0f * scale;
	b2CreateRevoluteJoint( worldId, &revoluteDef );

	revoluteDef.bodyIdA = m_wheelId2;
	revoluteDef.bodyIdB = m_barId2;
	revoluteDef.localAnchorA = { 0.0f, 0.0f };
	revoluteDef.localAnchorB = { 3.5f * scale, 0.0f };
	revoluteDef.enableMotor = true;
	revoluteDef.maxMotorTorque = 2.0f * scale;
	b2CreateRevoluteJoint( worldId, &revoluteDef );

	b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
	prismaticDef.bodyIdA = m_barId1;
	prismaticDef.bodyIdB = m_barId2;
	prismaticDef.localAxisA = { 1.0f, 0.0f };
	prismaticDef.localAnchorA = { 2.0f * scale, 0.0f };
	prismaticDef.localAnchorB = { -2.0f * scale, 0.0f };
	prismaticDef.lowerTranslation = -2.0f * scale;
	prismaticDef.upperTranslation = 2.0f * scale;
	prismaticDef.enableLimit = true;
	prismaticDef.enableMotor = true;
	prismaticDef.maxMotorForce = 2.0f * scale;
	prismaticDef.enableSpring = true;
	prismaticDef.hertz = 1.0f;
	prismaticDef.dampingRatio = 0.5;
	b2CreatePrismaticJoint( worldId, &prismaticDef );
}

void Doohickey::Despawn()
{
	assert( m_isSpawned == true );

	b2DestroyJoint( m_axleId1 );
	b2DestroyJoint( m_axleId2 );
	b2DestroyJoint( m_sliderId );

	b2DestroyBody( m_wheelId1 );
	b2DestroyBody( m_wheelId2 );
	b2DestroyBody( m_barId1 );
	b2DestroyBody( m_barId2 );

	m_isSpawned = false;
}
