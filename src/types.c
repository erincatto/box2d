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
	def.maxContactPushSpeed = 3.0f * b2_lengthUnitsPerMeter;
	def.contactHertz = 30.0;
	def.contactDampingRatio = 10.0f;

	// 400 meters per second, faster than the speed of sound
	def.maximumLinearSpeed = 400.0f * b2_lengthUnitsPerMeter;

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
	def.material.friction = 0.6f;
	def.density = 1.0f;
	def.filter = b2DefaultFilter();
	def.updateBodyMass = true;
	def.invokeContactCreation = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2SurfaceMaterial b2DefaultSurfaceMaterial( void )
{
	b2SurfaceMaterial material = {
		.friction = 0.6f,
	};

	return material;
}

b2ChainDef b2DefaultChainDef( void )
{
	static b2SurfaceMaterial defaultMaterial = {
		.friction = 0.6f,
	};

	b2ChainDef def = { 0 };
	def.materials = &defaultMaterial;
	def.materialCount = 1;
	def.filter = b2DefaultFilter();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

static void b2EmptyDrawPolygon( const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	B2_UNUSED( vertices, vertexCount, color, context );
}

static void b2EmptyDrawSolidPolygon( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius,
									 b2HexColor color, void* context )
{
	B2_UNUSED( transform, vertices, vertexCount, radius, color, context );
}

static void b2EmptyDrawCircle( b2Vec2 center, float radius, b2HexColor color, void* context )
{
	B2_UNUSED( center, radius, color, context );
}

static void b2EmptyDrawSolidCircle( b2Transform transform, float radius, b2HexColor color, void* context )
{
	B2_UNUSED( transform, radius, color, context );
}

static void b2EmptyDrawSolidCapsule( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context )
{
	B2_UNUSED( p1, p2, radius, color, context );
}

static void b2EmptyDrawSegment( b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context )
{
	B2_UNUSED( p1, p2, color, context );
}

static void b2EmptyDrawTransform( b2Transform transform, void* context )
{
	B2_UNUSED( transform, context );
}

static void b2EmptyDrawPoint( b2Vec2 p, float size, b2HexColor color, void* context )
{
	B2_UNUSED( p, size, color, context );
}

static void b2EmptyDrawString( b2Vec2 p, const char* s, b2HexColor color, void* context )
{
	B2_UNUSED( p, s, color, context );
}

b2DebugDraw b2DefaultDebugDraw( void )
{
	b2DebugDraw draw = { 0 };

	// These allow the user to skip some implementations and not hit null exceptions.
	draw.DrawPolygonFcn = b2EmptyDrawPolygon;
	draw.DrawSolidPolygonFcn = b2EmptyDrawSolidPolygon;
	draw.DrawCircleFcn = b2EmptyDrawCircle;
	draw.DrawSolidCircleFcn = b2EmptyDrawSolidCircle;
	draw.DrawSolidCapsuleFcn = b2EmptyDrawSolidCapsule;
	draw.DrawSegmentFcn = b2EmptyDrawSegment;
	draw.DrawTransformFcn = b2EmptyDrawTransform;
	draw.DrawPointFcn = b2EmptyDrawPoint;
	draw.DrawStringFcn = b2EmptyDrawString;
	return draw;
}
