// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "car.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Car::Car()
{
	m_chassisId = {};
	m_rearWheelId = {};
	m_frontWheelId = {};
	m_rearAxleId = {};
	m_frontAxleId = {};
	m_isSpawned = false;
}

void Car::Spawn( b2WorldId worldId, b2Vec2 position, float scale, float hertz, float dampingRatio, float torque, void* userData )
{
	assert( m_isSpawned == false );

	assert( B2_IS_NULL( m_chassisId ) );
	assert( B2_IS_NULL( m_frontWheelId ) );
	assert( B2_IS_NULL( m_rearWheelId ) );

	b2Vec2 vertices[6] = {
		{ -1.5f, -0.5f }, { 1.5f, -0.5f }, { 1.5f, 0.0f }, { 0.0f, 0.9f }, { -1.15f, 0.9f }, { -1.5f, 0.2f },
	};

	for ( int i = 0; i < 6; ++i )
	{
		vertices[i].x *= 0.85f * scale;
		vertices[i].y *= 0.85f * scale;
	}

	b2Hull hull = b2ComputeHull( vertices, 6 );
	b2Polygon chassis = b2MakePolygon( &hull, 0.15f * scale );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f / scale;
	shapeDef.friction = 0.2f;

	b2Circle circle = { { 0.0f, 0.0f }, 0.4f * scale };

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Add( { 0.0f, 1.0f * scale }, position );
	m_chassisId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( m_chassisId, &shapeDef, &chassis );

	shapeDef.density = 2.0f / scale;
	shapeDef.friction = 1.5f;
	shapeDef.rollingResistance = 0.1f;

	bodyDef.position = b2Add( { -1.0f * scale, 0.35f * scale }, position );
	bodyDef.allowFastRotation = true;
	m_rearWheelId = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_rearWheelId, &shapeDef, &circle );

	bodyDef.position = b2Add( { 1.0f * scale, 0.4f * scale }, position );
	bodyDef.allowFastRotation = true;
	m_frontWheelId = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_frontWheelId, &shapeDef, &circle );

	b2Vec2 axis = { 0.0f, 1.0f };
	b2Vec2 pivot = b2Body_GetPosition( m_rearWheelId );

	// float throttle = 0.0f;
	// float speed = 35.0f;
	// float torque = 2.5f * scale;
	// float hertz = 5.0f;
	// float dampingRatio = 0.7f;

	b2WheelJointDef jointDef = b2DefaultWheelJointDef();

	jointDef.bodyIdA = m_chassisId;
	jointDef.bodyIdB = m_rearWheelId;
	jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	m_rearAxleId = b2CreateWheelJoint( worldId, &jointDef );

	pivot = b2Body_GetPosition( m_frontWheelId );
	jointDef.bodyIdA = m_chassisId;
	jointDef.bodyIdB = m_frontWheelId;
	jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	m_frontAxleId = b2CreateWheelJoint( worldId, &jointDef );
}

void Car::Despawn()
{
	assert( m_isSpawned == true );

	b2DestroyJoint( m_rearAxleId );
	b2DestroyJoint( m_frontAxleId );
	b2DestroyBody( m_rearWheelId );
	b2DestroyBody( m_frontWheelId );
	b2DestroyBody( m_chassisId );

	m_isSpawned = false;
}

void Car::SetSpeed( float speed )
{
	b2WheelJoint_SetMotorSpeed( m_rearAxleId, speed );
	b2WheelJoint_SetMotorSpeed( m_frontAxleId, speed );
	b2Joint_WakeBodies( m_rearAxleId );
}

void Car::SetTorque( float torque )
{
	b2WheelJoint_SetMaxMotorTorque( m_rearAxleId, torque );
	b2WheelJoint_SetMaxMotorTorque( m_frontAxleId, torque );
}

void Car::SetHertz( float hertz )
{
	b2WheelJoint_SetSpringHertz( m_rearAxleId, hertz );
	b2WheelJoint_SetSpringHertz( m_frontAxleId, hertz );
}

void Car::SetDampingRadio( float dampingRatio )
{
	b2WheelJoint_SetSpringDampingRatio( m_rearAxleId, dampingRatio );
	b2WheelJoint_SetSpringDampingRatio( m_frontAxleId, dampingRatio );
}

Truck::Truck()
{
	m_chassisId = {};
	m_rearWheelId = {};
	m_frontWheelId = {};
	m_rearAxleId = {};
	m_frontAxleId = {};
	m_isSpawned = false;
}

void Truck::Spawn( b2WorldId worldId, b2Vec2 position, float scale, float hertz, float dampingRatio, float torque, float density,
				   void* userData )
{
	assert( m_isSpawned == false );

	assert( B2_IS_NULL( m_chassisId ) );
	assert( B2_IS_NULL( m_frontWheelId ) );
	assert( B2_IS_NULL( m_rearWheelId ) );

	// b2Vec2 vertices[6] = {
	//	{ -1.5f, -0.5f }, { 1.5f, -0.5f }, { 1.5f, 0.0f }, { 0.0f, 0.9f }, { -1.15f, 0.9f }, { -1.5f, 0.2f },
	// };

	b2Vec2 vertices[5] = {
		{ -0.65f, -0.4f }, { 1.5f, -0.4f }, { 1.5f, 0.0f }, { 0.0f, 0.9f }, { -0.65f, 0.9f },
	};

	for ( int i = 0; i < 5; ++i )
	{
		vertices[i].x *= 0.85f * scale;
		vertices[i].y *= 0.85f * scale;
	}

	b2Hull hull = b2ComputeHull( vertices, 5 );
	b2Polygon chassis = b2MakePolygon( &hull, 0.15f * scale );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = density;
	shapeDef.friction = 0.2f;
	shapeDef.customColor = b2_colorHotPink;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Add( { 0.0f, 1.0f * scale }, position );
	m_chassisId = b2CreateBody( worldId, &bodyDef );
	b2CreatePolygonShape( m_chassisId, &shapeDef, &chassis );

	b2Polygon box = b2MakeOffsetBox( 1.25f * scale, 0.1f * scale,  { -2.05f * scale, -0.275f * scale }, b2Rot_identity );
	box.radius = 0.1f * scale;
	b2CreatePolygonShape( m_chassisId, &shapeDef, &box );

	box = b2MakeOffsetBox( 0.05f * scale, 0.35f * scale,  { -3.25f * scale, 0.375f * scale }, b2Rot_identity  );
	box.radius = 0.1f * scale;
	b2CreatePolygonShape( m_chassisId, &shapeDef, &box );

	shapeDef.density = 2.0f * density;
	shapeDef.friction = 2.5f;
	shapeDef.customColor = b2_colorSilver;

	b2Circle circle = { { 0.0f, 0.0f }, 0.4f * scale };
	bodyDef.position = b2Add( { -2.75f * scale, 0.3f * scale }, position );
	m_rearWheelId = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_rearWheelId, &shapeDef, &circle );

	bodyDef.position = b2Add( { 0.8f * scale, 0.3f * scale }, position );
	m_frontWheelId = b2CreateBody( worldId, &bodyDef );
	b2CreateCircleShape( m_frontWheelId, &shapeDef, &circle );

	b2Vec2 axis = { 0.0f, 1.0f };
	b2Vec2 pivot = b2Body_GetPosition( m_rearWheelId );

	// float throttle = 0.0f;
	// float speed = 35.0f;
	// float torque = 2.5f * scale;
	// float hertz = 5.0f;
	// float dampingRatio = 0.7f;

	b2WheelJointDef jointDef = b2DefaultWheelJointDef();

	jointDef.bodyIdA = m_chassisId;
	jointDef.bodyIdB = m_rearWheelId;
	jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	m_rearAxleId = b2CreateWheelJoint( worldId, &jointDef );

	pivot = b2Body_GetPosition( m_frontWheelId );
	jointDef.bodyIdA = m_chassisId;
	jointDef.bodyIdB = m_frontWheelId;
	jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	jointDef.motorSpeed = 0.0f;
	jointDef.maxMotorTorque = torque;
	jointDef.enableMotor = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;
	jointDef.lowerTranslation = -0.25f * scale;
	jointDef.upperTranslation = 0.25f * scale;
	jointDef.enableLimit = true;
	m_frontAxleId = b2CreateWheelJoint( worldId, &jointDef );
}

void Truck::Despawn()
{
	assert( m_isSpawned == true );

	b2DestroyJoint( m_rearAxleId );
	b2DestroyJoint( m_frontAxleId );
	b2DestroyBody( m_rearWheelId );
	b2DestroyBody( m_frontWheelId );
	b2DestroyBody( m_chassisId );

	m_isSpawned = false;
}

void Truck::SetSpeed( float speed )
{
	b2WheelJoint_SetMotorSpeed( m_rearAxleId, speed );
	b2WheelJoint_SetMotorSpeed( m_frontAxleId, speed );
	b2Joint_WakeBodies( m_rearAxleId );
}

void Truck::SetTorque( float torque )
{
	b2WheelJoint_SetMaxMotorTorque( m_rearAxleId, torque );
	b2WheelJoint_SetMaxMotorTorque( m_frontAxleId, torque );
}

void Truck::SetHertz( float hertz )
{
	b2WheelJoint_SetSpringHertz( m_rearAxleId, hertz );
	b2WheelJoint_SetSpringHertz( m_frontAxleId, hertz );
}

void Truck::SetDampingRadio( float dampingRatio )
{
	b2WheelJoint_SetSpringDampingRatio( m_rearAxleId, dampingRatio );
	b2WheelJoint_SetSpringDampingRatio( m_frontAxleId, dampingRatio );
}
