// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/types.h"

#include "constants.h"
#include "core.h"

b2WorldDef b2DefaultWorldDef( void )
{
	b2WorldDef def = { 0 };
	def.gravity.x = 0.0f;
	def.gravity.y = -10.0f;
	def.hitEventThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
	def.contactPushVelocity = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0f;
	def.jointHertz = 60.0;
	def.jointDampingRatio = 2.0f;
	// 400 meters per second, faster than the speed of sound
	def.maximumLinearVelocity = 400.0f * b2_lengthUnitsPerMeter;
	def.frictionMixingRule = b2_mixGeometricMean;
	def.restitutionMixingRule = b2_mixMaximum;
	def.enableSleep = true;
	def.enableContinuous = true;
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
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2Filter b2DefaultFilter( void )
{
	b2Filter filter = { B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS, 0 };
	return filter;
}

b2QueryFilter b2DefaultQueryFilter( void )
{
	b2QueryFilter filter = { B2_DEFAULT_CATEGORY_BITS, B2_DEFAULT_MASK_BITS };
	return filter;
}

b2ShapeDef b2DefaultShapeDef( void )
{
	b2ShapeDef def = { 0 };
	def.friction = 0.6f;
	def.density = 1.0f;
	def.filter = b2DefaultFilter();
	def.enableSensorEvents = true;
	def.updateBodyMass = true;
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

static void b2EmptyDrawPolygon( const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( vertices );
	B2_MAYBE_UNUSED( vertexCount );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawSolidPolygon( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
							void* context )
{
	B2_MAYBE_UNUSED( transform );
	B2_MAYBE_UNUSED( vertices );
	B2_MAYBE_UNUSED( vertexCount );
	B2_MAYBE_UNUSED( radius );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawCircle( b2Vec2 center, float radius, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( center );
	B2_MAYBE_UNUSED( radius );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawSolidCircle( b2Transform transform, float radius, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( transform );
	B2_MAYBE_UNUSED( radius );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawSolidCapsule( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( p1 );
	B2_MAYBE_UNUSED( p2 );
	B2_MAYBE_UNUSED( radius );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawSegment( b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( p1 );
	B2_MAYBE_UNUSED( p2 );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawTransform( b2Transform transform, void* context )
{
	B2_MAYBE_UNUSED( transform );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawPoint( b2Vec2 p, float size, b2HexColor color, void* context )
{
	B2_MAYBE_UNUSED( p );
	B2_MAYBE_UNUSED( size );
	B2_MAYBE_UNUSED( color );
	B2_MAYBE_UNUSED( context );
}

static void b2EmptyDrawString( b2Vec2 p, const char* s, void* context )
{
	B2_MAYBE_UNUSED( p );
	B2_MAYBE_UNUSED( s );
	B2_MAYBE_UNUSED( context );
}

b2DebugDraw b2DefaultDebugDraw(void)
{
	b2DebugDraw draw = { 0 };

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawPolygon = b2EmptyDrawPolygon;
	draw.DrawSolidPolygon = b2EmptyDrawSolidPolygon;
	draw.DrawCircle = b2EmptyDrawCircle;
	draw.DrawSolidCircle = b2EmptyDrawSolidCircle;
	draw.DrawSolidCapsule = b2EmptyDrawSolidCapsule;
	draw.DrawSegment = b2EmptyDrawSegment;
	draw.DrawTransform = b2EmptyDrawTransform;
	draw.DrawPoint = b2EmptyDrawPoint;
	draw.DrawString = b2EmptyDrawString;
	return draw;
}
