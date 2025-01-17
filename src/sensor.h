// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

typedef struct b2World b2World;

typedef struct b2Sensor
{
	int shapeId;
	int overlapStartIndex;
	int overlapCount;
} b2Sensor;

void b2OverlapSensors( b2World* world );

B2_ARRAY_INLINE(b2Sensor, b2Sensor);
