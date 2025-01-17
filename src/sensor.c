// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sensor.h"

#include "body.h"
#include "shape.h"
#include "world.h"

#include "box2d/collision.h"

#include <stddef.h>

B2_ARRAY_SOURCE( b2Sensor, b2Sensor );

struct b2SensorContext
{
	b2World* world;
	b2Shape* sensorShape;
	b2Transform transform;
	int* resultBuffer;
	int resultCount;
	int resultCapacity;
};

static bool b2SensorQueryCallback( int proxyId, int shapeId, void* context )
{
	struct b2SensorContext* sensorContext = context;
	b2Shape* sensorShape = sensorContext->sensorShape;
	int sensorShapeId = sensorShape->id;

	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b2World* world = sensorContext->world;
	b2Shape* otherShape = b2ShapeArray_Get( &world->shapes, shapeId );

	// test for overlap

	return true;
}

void b2OverlapSensors( b2World* world )
{
	struct b2SensorContext context = { 0 };
	context.world = world;
	context.resultBuffer = b2AllocateArenaItem( &world->stackAllocator, 1024 * sizeof( int ), "sensor overlaps" );
	context.resultCapacity = 1024;

	b2DynamicTree* trees = world->broadPhase.trees;
	int sensorCount = world->sensors.count;
	for (int sensorIndex = 0; sensorIndex < sensorCount; ++sensorIndex)
	{
		b2Sensor* sensor = world->sensors.data + sensorIndex;
		b2Shape* sensorShape = b2ShapeArray_Get( &world->shapes, sensor->shapeId );

		context.sensorShape = sensorShape;
		context.transform = b2GetBodyTransform( world, sensorShape->bodyId );

		B2_ASSERT( sensorShape->sensorIndex == sensorIndex );
		b2AABB queryBounds = sensorShape->aabb;

		// Query all trees
		b2DynamicTree_Query( trees + 0, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &context );
		b2DynamicTree_Query( trees + 1, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &context );
		b2DynamicTree_Query( trees + 2, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &context );
	}
}
