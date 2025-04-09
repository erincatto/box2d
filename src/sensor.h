// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"

typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

typedef struct b2ShapeRef
{
	int shapeId;
	uint16_t generation;
} b2ShapeRef;

typedef struct b2Sensor
{
	b2ShapeRefArray overlaps1;
	b2ShapeRefArray overlaps2;
	int shapeId;
} b2Sensor;

typedef struct b2SensorTaskContext
{
	b2BitSet eventBits;
} b2SensorTaskContext;

void b2OverlapSensors( b2World* world );

void b2DestroySensor( b2World* world, b2Shape* sensorShape );

B2_ARRAY_INLINE( b2ShapeRef, b2ShapeRef )
B2_ARRAY_INLINE( b2Sensor, b2Sensor )
B2_ARRAY_INLINE( b2SensorTaskContext, b2SensorTaskContext )
