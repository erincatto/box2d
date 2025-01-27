// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sensor.h"

#include "array.h"
#include "body.h"
#include "contact.h"
#include "ctz.h"
#include "shape.h"
#include "world.h"

#include "box2d/collision.h"

#include <stddef.h>
#include <stdlib.h>

B2_ARRAY_SOURCE( b2ShapeRef, b2ShapeRef );
B2_ARRAY_SOURCE( b2Sensor, b2Sensor );
B2_ARRAY_SOURCE( b2SensorTaskContext, b2SensorTaskContext );

struct b2SensorQueryContext
{
	b2World* world;
	b2SensorTaskContext* taskContext;
	b2Sensor* sensor;
	b2Shape* sensorShape;
	b2Transform transform;
};

// Sensor shapes need to
// - detect begin and end overlap events
// - events must be reported in deterministic order
// - maintain an active list of overlaps for query

// Assumption
// - sensors don't detect other sensors

// Algorithm
// Query all sensors for overlaps
// Check against previous overlaps

// Data structures
// Each sensor has an double buffered array of overlaps
// These overlaps use a shape reference with index and generation

static bool b2SensorQueryCallback( int proxyId, int shapeId, void* context )
{
	B2_UNUSED( proxyId );

	struct b2SensorQueryContext* queryContext = context;
	b2Shape* sensorShape = queryContext->sensorShape;
	int sensorShapeId = sensorShape->id;

	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b2World* world = queryContext->world;
	b2Shape* otherShape = b2ShapeArray_Get( &world->shapes, shapeId );

	// Sensors don't overlap with other sensors
	if ( otherShape->sensorIndex != B2_NULL_INDEX )
	{
		return true;
	}

	// Check filter
	if ( b2ShouldShapesCollide( sensorShape->filter, otherShape->filter ) == false )
	{
		return true;
	}

	b2Transform otherTransform = b2GetBodyTransform( world, otherShape->bodyId );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( sensorShape );
	input.proxyB = b2MakeShapeDistanceProxy( otherShape );
	input.transformA = queryContext->transform;
	input.transformB = otherTransform;
	input.useRadii = true;
	b2SimplexCache cache = { 0 };
	b2DistanceOutput output = b2ShapeDistance( &cache, &input, NULL, 0 );

	bool overlaps = output.distance < 10.0f * FLT_EPSILON;
	if ( overlaps == false )
	{
		return true;
	}

	// Record the overlap
	b2Sensor* sensor = queryContext->sensor;
	b2ShapeRef* shapeRef = b2ShapeRefArray_Add( &sensor->overlaps2 );
	shapeRef->shapeId = shapeId;
	shapeRef->generation = otherShape->generation;

	return true;
}

static int b2CompareShapeRefs( const void* a, const void* b )
{
	const b2ShapeRef* sa = a;
	const b2ShapeRef* sb = b;

	if ( sa->shapeId < sb->shapeId )
	{
		return -1;
	}

	if ( sa->shapeId == sb->shapeId )
	{
		if ( sa->generation < sb->generation )
		{
			return -1;
		}

		if ( sa->generation == sb->generation )
		{
			return 0;
		}
	}

	return 1;
}

static void b2SensorTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( sensor_task, "Overlap", b2_colorBrown, true );

	b2World* world = context;
	B2_ASSERT( (int)threadIndex < world->workerCount );
	b2SensorTaskContext* taskContext = world->sensorTaskContexts.data + threadIndex;

	B2_ASSERT( startIndex < endIndex );

	b2DynamicTree* trees = world->broadPhase.trees;
	for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
	{
		b2Sensor* sensor = b2SensorArray_Get( &world->sensors, sensorIndex );
		b2Shape* sensorShape = b2ShapeArray_Get( &world->shapes, sensor->shapeId );

		// swap overlap arrays
		b2ShapeRefArray temp = sensor->overlaps1;
		sensor->overlaps1 = sensor->overlaps2;
		sensor->overlaps2 = temp;
		b2ShapeRefArray_Clear( &sensor->overlaps2 );

		b2Transform transform = b2GetBodyTransform( world, sensorShape->bodyId );

		struct b2SensorQueryContext queryContext = {
			.world = world,
			.taskContext = taskContext,
			.sensorShape = sensorShape,
			.sensor = sensor,
			.transform = transform,
		};

		B2_ASSERT( sensorShape->sensorIndex == sensorIndex );
		b2AABB queryBounds = sensorShape->aabb;

		// Query all trees
		b2DynamicTree_Query( trees + 0, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );
		b2DynamicTree_Query( trees + 1, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );
		b2DynamicTree_Query( trees + 2, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );

		// Sort the overlaps to enable finding begin and end events.
		qsort( sensor->overlaps2.data, sensor->overlaps2.count, sizeof( b2ShapeRef ), b2CompareShapeRefs );

		int count1 = sensor->overlaps1.count;
		int count2 = sensor->overlaps2.count;
		if ( count1 != count2 )
		{
			// something changed
			b2SetBit( &taskContext->eventBits, sensorIndex );
		}
		else
		{
			for ( int i = 0; i < count1; ++i )
			{
				b2ShapeRef* s1 = sensor->overlaps1.data + i;
				b2ShapeRef* s2 = sensor->overlaps2.data + i;

				if ( s1->shapeId != s2->shapeId || s1->generation != s2->generation )
				{
					// something changed
					b2SetBit( &taskContext->eventBits, sensorIndex );
					break;
				}
			}
		}
	}

	b2TracyCZoneEnd( sensor_task );
}

void b2OverlapSensors( b2World* world )
{
	int sensorCount = world->sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}

	B2_ASSERT( world->workerCount > 0 );

	b2TracyCZoneNC( overlap_sensors, "Sensors", b2_colorMediumPurple, true );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->sensorTaskContexts.data[i].eventBits, sensorCount );
	}

	// Parallel-for sensors overlaps
	int minRange = 16;
	void* userSensorTask = world->enqueueTaskFcn( &b2SensorTask, sensorCount, minRange, world, world->userTaskContext );
	world->taskCount += 1;
	if ( userSensorTask != NULL )
	{
		world->finishTaskFcn( userSensorTask, world->userTaskContext );
	}

	b2TracyCZoneNC( sensor_state, "Events", b2_colorLightSlateGray, true );

	b2BitSet* bitSet = &world->sensorTaskContexts.data[0].eventBits;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->sensorTaskContexts.data[i].eventBits );
	}

	// Iterate sensors bits and publish events
	// Process contact state changes. Iterate over set bits
	uint64_t* bits = bitSet->bits;
	uint32_t blockCount = bitSet->blockCount;

	for ( uint32_t k = 0; k < blockCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = b2CTZ64( word );
			int sensorIndex = (int)( 64 * k + ctz );

			b2Sensor* sensor = b2SensorArray_Get( &world->sensors, sensorIndex );
			b2Shape* sensorShape = b2ShapeArray_Get( &world->shapes, sensor->shapeId );
			b2ShapeId sensorId = { sensor->shapeId + 1, world->worldId, sensorShape->generation };

			int count1 = sensor->overlaps1.count;
			int count2 = sensor->overlaps2.count;
			const b2ShapeRef* refs1 = sensor->overlaps1.data;
			const b2ShapeRef* refs2 = sensor->overlaps2.data;

			// overlaps1 can have overlaps that end
			// overlaps2 can have overlaps that begin
			int index1 = 0, index2 = 0;
			while ( index1 < count1 && index2 < count2 )
			{
				const b2ShapeRef* r1 = refs1 + index1;
				const b2ShapeRef* r2 = refs2 + index2;
				if ( r1->shapeId == r2->shapeId )
				{
					if ( r1->generation < r2->generation )
					{
						// end
						b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
						b2SensorEndTouchEvent event = { sensorId, visitorId };
						b2SensorEndTouchEventArray_Push( &world->sensorEndEvents[world->endEventArrayIndex], event );
						index1 += 1;
					}
					else if ( r1->generation > r2->generation )
					{
						// begin
						b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
						b2SensorBeginTouchEvent event = { sensorId, visitorId };
						b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
						index2 += 1;
					}
					else
					{
						// persisted
						index1 += 1;
						index2 += 1;
					}
				}
				else if ( r1->shapeId < r2->shapeId )
				{
					// end
					b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
					b2SensorEndTouchEvent event = { sensorId, visitorId };
					b2SensorEndTouchEventArray_Push( &world->sensorEndEvents[world->endEventArrayIndex], event );
					index1 += 1;
				}
				else
				{
					// begin
					b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
					b2SensorBeginTouchEvent event = { sensorId, visitorId };
					b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
					index2 += 1;
				}
			}

			while ( index1 < count1 )
			{
				// end
				const b2ShapeRef* r1 = refs1 + index1;
				b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
				b2SensorEndTouchEvent event = { sensorId, visitorId };
				b2SensorEndTouchEventArray_Push( &world->sensorEndEvents[world->endEventArrayIndex], event );
				index1 += 1;
			}

			while ( index2 < count2 )
			{
				// begin
				const b2ShapeRef* r2 = refs2 + index2;
				b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
				b2SensorBeginTouchEvent event = { sensorId, visitorId };
				b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
				index2 += 1;
			}

			// Clear the smallest set bit
			word = word & ( word - 1 );
		}
	}

	b2TracyCZoneEnd( sensor_state );
	b2TracyCZoneEnd( overlap_sensors );
}

void b2DestroySensor( b2World* world, b2Shape* sensorShape )
{
	b2Sensor* sensor = b2SensorArray_Get( &world->sensors, sensorShape->sensorIndex );
	for ( int i = 0; i < sensor->overlaps2.count; ++i )
	{
		b2ShapeRef* ref = sensor->overlaps2.data + i;
		b2SensorEndTouchEvent event = {
			.sensorShapeId =
				{
					.index1 = sensorShape->id + 1,
					.generation = sensorShape->generation,
					.world0 = world->worldId,
				},
			.visitorShapeId =
				{
					.index1 = ref->shapeId + 1,
					.generation = ref->generation,
					.world0 = world->worldId,
				},
		};

		b2SensorEndTouchEventArray_Push( world->sensorEndEvents + world->endEventArrayIndex, event );
	}

	// Destroy sensor
	b2ShapeRefArray_Destroy( &sensor->overlaps1 );
	b2ShapeRefArray_Destroy( &sensor->overlaps2 );

	int movedIndex = b2SensorArray_RemoveSwap( &world->sensors, sensorShape->sensorIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fixup moved sensor
		b2Sensor* movedSensor = b2SensorArray_Get( &world->sensors, sensorShape->sensorIndex );
		b2Shape* otherSensorShape = b2ShapeArray_Get( &world->shapes, movedSensor->shapeId );
		otherSensorShape->sensorIndex = sensorShape->sensorIndex;
	}
}
