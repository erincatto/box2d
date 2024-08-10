// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "world.h"

#include "box2d/types.h"

typedef struct b2BroadPhase b2BroadPhase;
typedef struct b2World b2World;

typedef struct b2Shape
{
	int id;
	int bodyId;
	int prevShapeId;
	int nextShapeId;
	b2ShapeType type;
	float density;
	float friction;
	float restitution;

	b2AABB aabb;
	b2AABB fatAABB;
	b2Vec2 localCentroid;
	int proxyKey;

	b2Filter filter;
	void* userData;
	uint32_t customColor;

	union
	{
		b2Capsule capsule;
		b2Circle circle;
		b2Polygon polygon;
		b2Segment segment;
		b2SmoothSegment smoothSegment;
	};

	uint16_t revision;
	bool isSensor;
	bool enableSensorEvents;
	bool enableContactEvents;
	bool enableHitEvents;
	bool enablePreSolveEvents;
	bool enlargedAABB;
	bool isFast;
} b2Shape;

typedef struct b2ChainShape
{
	int id;
	int bodyId;
	int nextChainId;
	int* shapeIndices;
	int count;
	uint16_t revision;
} b2ChainShape;

typedef struct b2ShapeExtent
{
	float minExtent;
	float maxExtent;
} b2ShapeExtent;

void b2CreateShapeProxy( b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform transform, bool forcePairCreation );
void b2DestroyShapeProxy( b2Shape* shape, b2BroadPhase* bp );

b2MassData b2ComputeShapeMass( const b2Shape* shape );
b2ShapeExtent b2ComputeShapeExtent( const b2Shape* shape, b2Vec2 localCenter );
b2AABB b2ComputeShapeAABB( const b2Shape* shape, b2Transform transform );
b2Vec2 b2GetShapeCentroid( const b2Shape* shape );
float b2GetShapePerimeter( const b2Shape* shape );

b2DistanceProxy b2MakeShapeDistanceProxy( const b2Shape* shape );

b2CastOutput b2RayCastShape( const b2RayCastInput* input, const b2Shape* shape, b2Transform transform );
b2CastOutput b2ShapeCastShape( const b2ShapeCastInput* input, const b2Shape* shape, b2Transform transform );

b2Transform b2GetOwnerTransform( b2World* world, b2Shape* shape );
