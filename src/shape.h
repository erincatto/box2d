// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

#include "box2d/types.h"

typedef struct b2BroadPhase b2BroadPhase;
typedef struct b2World b2World;

typedef struct b2Shape
{
	int id;
	int bodyId;
	int prevShapeId;
	int nextShapeId;
	int sensorIndex;
	b2ShapeType type;
	float density;
	float friction;
	float restitution;
	float rollingResistance;
	float tangentSpeed;
	int userMaterialId;

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
		b2ChainSegment chainSegment;
	};

	uint16_t generation;
	bool enableSensorEvents;
	bool enableContactEvents;
	bool enableHitEvents;
	bool enablePreSolveEvents;
	bool enlargedAABB;
} b2Shape;

typedef struct b2ChainShape
{
	int id;
	int bodyId;
	int nextChainId;
	int count;
	int materialCount;
	int* shapeIndices;
	b2SurfaceMaterial* materials;
	uint16_t generation;
} b2ChainShape;

typedef struct b2ShapeExtent
{
	float minExtent;
	float maxExtent;
} b2ShapeExtent;

// Sensors are shapes that live in the broad-phase but never have contacts.
// At the end of the time step all sensors are queried for overlap with any other shapes.
// Sensors ignore body type and sleeping.
// Sensors generate events when there is a new overlap or and overlap disappears.
// The sensor overlaps don't get cleared until the next time step regardless of the overlapped
// shapes being destroyed.
// When a sensor is destroyed.
typedef struct
{
	b2IntArray overlaps;
} b2SensorOverlaps;

void b2CreateShapeProxy( b2Shape* shape, b2BroadPhase* bp, b2BodyType type, b2Transform transform, bool forcePairCreation );
void b2DestroyShapeProxy( b2Shape* shape, b2BroadPhase* bp );

void b2FreeChainData( b2ChainShape* chain );

b2MassData b2ComputeShapeMass( const b2Shape* shape );
b2ShapeExtent b2ComputeShapeExtent( const b2Shape* shape, b2Vec2 localCenter );
b2AABB b2ComputeShapeAABB( const b2Shape* shape, b2Transform transform );
b2Vec2 b2GetShapeCentroid( const b2Shape* shape );
float b2GetShapePerimeter( const b2Shape* shape );
float b2GetShapeProjectedPerimeter( const b2Shape* shape, b2Vec2 line );

b2ShapeProxy b2MakeShapeDistanceProxy( const b2Shape* shape );

b2CastOutput b2RayCastShape( const b2RayCastInput* input, const b2Shape* shape, b2Transform transform );
b2CastOutput b2ShapeCastShape( const b2ShapeCastInput* input, const b2Shape* shape, b2Transform transform );

b2PlaneResult b2CollideMoverAndCircle( const b2Circle* shape, const b2Capsule* mover );
b2PlaneResult b2CollideMoverAndCapsule( const b2Capsule* shape, const b2Capsule* mover );
b2PlaneResult b2CollideMoverAndPolygon( const b2Polygon* shape, const b2Capsule* mover );
b2PlaneResult b2CollideMoverAndSegment( const b2Segment* shape, const b2Capsule* mover );
b2PlaneResult b2CollideMover( const b2Shape* shape, b2Transform transform, const b2Capsule* mover );

static inline float b2GetShapeRadius( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return shape->capsule.radius;
		case b2_circleShape:
			return shape->circle.radius;
		case b2_polygonShape:
			return shape->polygon.radius;
		default:
			return 0.0f;
	}
}

static inline bool b2ShouldShapesCollide( b2Filter filterA, b2Filter filterB )
{
	if ( filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0 )
	{
		return filterA.groupIndex > 0;
	}

	return ( filterA.maskBits & filterB.categoryBits ) != 0 && ( filterA.categoryBits & filterB.maskBits ) != 0;
}

static inline bool b2ShouldQueryCollide( b2Filter shapeFilter, b2QueryFilter queryFilter )
{
	return ( shapeFilter.categoryBits & queryFilter.maskBits ) != 0 && ( shapeFilter.maskBits & queryFilter.categoryBits ) != 0;
}

B2_ARRAY_INLINE( b2ChainShape, b2ChainShape )
B2_ARRAY_INLINE( b2Shape, b2Shape )
