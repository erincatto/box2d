// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/id.h"

// This allows benchmarks to be tested on the benchmark app and also visualized in the samples app

B2_API void CreateJointGrid( b2WorldId worldId );
B2_API void CreateLargePyramid( b2WorldId worldId );
B2_API void CreateManyPyramids( b2WorldId worldId );
B2_API void CreateRain( b2WorldId worldId );
B2_API void StepRain( b2WorldId worldId, int stepCount );
B2_API void CreateSpinner( b2WorldId worldId );
B2_API void CreateSmash( b2WorldId worldId );
B2_API void CreateTumbler( b2WorldId worldId );
