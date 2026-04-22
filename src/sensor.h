// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "container.h"

typedef struct b2Shape b2Shape;
typedef struct b2World b2World;

// Used to track shapes that hit sensors using time of impact
typedef struct b2SensorHit
{
	int sensorId;
	int visitorId;
} b2SensorHit;

b2DeclareArray( b2SensorHit );

typedef struct b2Visitor
{
	int shapeId;
	uint16_t generation;
} b2Visitor;

b2DeclareArray( b2Visitor );

typedef struct b2Sensor
{
	// todo find a way to pool these
	b2Array( b2Visitor ) hits;
	b2Array( b2Visitor ) overlaps1;
	b2Array( b2Visitor ) overlaps2;
	int shapeId;
} b2Sensor;

b2DeclareArray( b2Sensor );

typedef struct b2SensorTaskContext
{
	b2BitSet eventBits;
} b2SensorTaskContext;

b2DeclareArray( b2SensorTaskContext );

void b2OverlapSensors( b2World* world );

void b2DestroySensor( b2World* world, b2Shape* sensorShape );
