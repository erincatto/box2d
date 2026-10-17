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

b2Capacity GetJointGridCapacity( void );
void CreateJointGrid( b2WorldId worldId );
b2Capacity GetLargePyramidCapacity( void );
void CreateLargePyramid( b2WorldId worldId );
b2Capacity GetManyPyramidsCapacity( void );
void CreateManyPyramids( b2WorldId worldId );
b2Capacity GetRainCapacity( void );
void CreateRain( b2WorldId worldId );
float StepRain( b2WorldId worldId, int stepCount );
b2Capacity GetSpinnerCapacity( void );
void CreateSpinner( b2WorldId worldId );
float StepSpinner( b2WorldId worldId, int stepCount );
b2Capacity GetSmashCapacity( void );
void CreateSmash( b2WorldId worldId );
b2Capacity GetTumblerCapacity( void );
void CreateTumbler( b2WorldId worldId );
b2Capacity GetWasherCapacity( void );
void CreateWasher( b2WorldId worldId );
b2Capacity GetJunkyardCapacity( void );
void CreateJunkyard( b2WorldId worldId );
float StepJunkyard( b2WorldId worldId, int stepCount );

b2Capacity GetSleepCapacity( void );
void CreateSleep( b2WorldId worldId );
float StepSleep( b2WorldId worldId, int stepCount );

b2Capacity GetCompoundsCapacity( void );
void CreateCompounds( b2WorldId worldId );

b2Capacity GetQueriesCapacity( void );
void CreateQueries( b2WorldId worldId );
float StepQueries( b2WorldId worldId, int stepCount );
b2TreeStats GetQueryBenchmarkStats( void );
int GetQueryBenchmarkCount( void );
float GetQueryBenchmarkExtent( void );
void GetQueryBenchmarkRay( int index, b2Pos* origin, b2Vec2* translation );

void CreateTreeCast( b2WorldId worldId );
float StepTreeCast( b2WorldId worldId, int stepCount );
void DestroyTreeCast( void );
b2TreeStats GetTreeCastBenchmarkStats( void );

b2Capacity GetTileWorldCapacity( void );
void CreateTileWorld( b2WorldId worldId );
float StepTileWorld( b2WorldId worldId, int stepCount );
void DestroyTileWorld( void );
b2TreeStats GetTileWorldBenchmarkStats( void );

#ifdef __cplusplus
}
#endif
