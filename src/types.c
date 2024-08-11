// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/types.h"

#include "core.h"

b2WorldDef b2DefaultWorldDef( void )
{
	b2WorldDef def = { 0 };
	def.gravity.x = 0.0f;
	def.gravity.y = -10.0f;
	def.hitEventThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.contactPushoutVelocity = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0f;
	def.jointHertz = 60.0;
	def.jointDampingRatio = 2.0f;
	// 400 meters per second, faster than the speed of sound
	def.maximumLinearVelocity = 400.0f * b2_lengthUnitsPerMeter;
	def.enableSleep = true;
	def.enableContinous = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2BodyDef b2DefaultBodyDef( void )
{
	b2BodyDef def = { 0 };
	def.type = b2_staticBody;
	def.rotation = b2Rot_identity;
	def.sleepThreshold = 0.05f * b2_lengthUnitsPerMeter;
	def.gravityScale = 1.0f;
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.automaticMass = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2Filter b2DefaultFilter( void )
{
	b2Filter filter = { 0x00000001, 0xFFFFFFFF, 0 };
	return filter;
}

b2QueryFilter b2DefaultQueryFilter( void )
{
	b2QueryFilter filter = { 0x00000001, 0xFFFFFFFF };
	return filter;
}

b2ShapeDef b2DefaultShapeDef( void )
{
	b2ShapeDef def = { 0 };
	def.friction = 0.6f;
	def.density = 1.0f;
	def.filter = b2DefaultFilter();
	def.enableSensorEvents = true;
	def.enableContactEvents = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2ChainDef b2DefaultChainDef( void )
{
	b2ChainDef def = { 0 };
	def.friction = 0.6f;
	def.filter = b2DefaultFilter();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
