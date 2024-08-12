// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "donut.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Donut::Donut()
{
	for ( int i = 0; i < e_sides; ++i )
	{
		m_bodyIds[i] = b2_nullBodyId;
		m_jointIds[i] = b2_nullJointId;
	}

	m_isSpawned = false;
}

void Donut::Spawn( b2WorldId worldId, b2Vec2 position, float scale, int groupIndex, void* userData )
{
	assert( m_isSpawned == false );

	for ( int i = 0; i < e_sides; ++i )
	{
		assert( B2_IS_NULL( m_bodyIds[i] ) );
		assert( B2_IS_NULL( m_jointIds[i] ) );
	}

	float radius = 1.0f * scale;
	float deltaAngle = 2.0f * b2_pi / e_sides;
	float length = 2.0f * b2_pi * radius / e_sides;

	b2Capsule capsule = { { 0.0f, -0.5f * length }, { 0.0f, 0.5f * length }, 0.25f * scale };

	b2Vec2 center = position;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.userData = userData;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.friction = 0.3f;

	// Create bodies
	float angle = 0.0f;
	for ( int i = 0; i < e_sides; ++i )
	{
		bodyDef.position = { radius * cosf( angle ) + center.x, radius * sinf( angle ) + center.y };
		bodyDef.rotation = b2MakeRot( angle );

		m_bodyIds[i] = b2CreateBody( worldId, &bodyDef );
		b2CreateCapsuleShape( m_bodyIds[i], &shapeDef, &capsule );

		angle += deltaAngle;
	}

	// Create joints
	b2WeldJointDef weldDef = b2DefaultWeldJointDef();
	weldDef.angularHertz = 5.0f;
	weldDef.angularDampingRatio = 0.0f;
	weldDef.localAnchorA = { 0.0f, 0.5f * length };
	weldDef.localAnchorB = { 0.0f, -0.5f * length };

	b2BodyId prevBodyId = m_bodyIds[e_sides - 1];
	for ( int i = 0; i < e_sides; ++i )
	{
		weldDef.bodyIdA = prevBodyId;
		weldDef.bodyIdB = m_bodyIds[i];
		b2Rot rotA = b2Body_GetRotation( prevBodyId );
		b2Rot rotB = b2Body_GetRotation( m_bodyIds[i] );
		weldDef.referenceAngle = b2RelativeAngle( rotB, rotA );
		m_jointIds[i] = b2CreateWeldJoint( worldId, &weldDef );
		prevBodyId = weldDef.bodyIdB;
	}

	m_isSpawned = true;
}

void Donut::Despawn()
{
	assert( m_isSpawned == true );

	for ( int i = 0; i < e_sides; ++i )
	{
		b2DestroyBody( m_bodyIds[i] );
		m_bodyIds[i] = b2_nullBodyId;
		m_jointIds[i] = b2_nullJointId;
	}

	m_isSpawned = false;
}
