// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/id.h"

// This allows benchmarks to be tested on the benchmark app and also visualized in the samples app

#ifdef __cplusplus
extern "C"
{
#endif

void CreateJointGrid( b2WorldId worldId );
void CreateLargePyramid( b2WorldId worldId );
void CreateManyPyramids( b2WorldId worldId );
void CreateRain( b2WorldId worldId );
void StepRain( b2WorldId worldId, int stepCount );
void CreateSpinner( b2WorldId worldId );
void CreateSmash( b2WorldId worldId );
void CreateTumbler( b2WorldId worldId );

#ifdef __cplusplus
}
#endif
