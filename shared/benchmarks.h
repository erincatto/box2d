// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/id.h"
#include "box2d/collision.h"
#include "box2d/types.h"

#include <stdbool.h>

// This allows benchmarks to be tested on the benchmark app and also visualized in the samples app

#ifdef __cplusplus
extern "C"
{
#endif

void CreateJointGrid( b2WorldId worldId );
void CreateLargePyramid( b2WorldId worldId );
void CreateManyPyramids( b2WorldId worldId );
b2Capacity GetManyPyramidsCapacity( void );
void CreateRain( b2WorldId worldId );
float StepRain( b2WorldId worldId, int stepCount );
void CreateSpinner( b2WorldId worldId );
float StepSpinner( b2WorldId worldId, int stepCount );
void CreateSmash( b2WorldId worldId );
void CreateTumbler( b2WorldId worldId );
void CreateWasher( b2WorldId worldId );
void CreateJunkyard( b2WorldId worldId );
float StepJunkyard( b2WorldId worldId, int stepCount );
void CreateCompounds( b2WorldId worldId );

void CreateQueries( b2WorldId worldId );
float StepQueries( b2WorldId worldId, int stepCount );
b2TreeStats GetQueryBenchmarkStats( void );
int GetQueryBenchmarkCount( void );
float GetQueryBenchmarkExtent( void );
void GetQueryBenchmarkRay( int index, b2Pos* origin, b2Vec2* translation );

#ifdef __cplusplus
}
#endif
