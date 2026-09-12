// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/id.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined( BOX2D_DOUBLE_PRECISION )
#define EXPECTED_SLEEP_STEP 256
#define EXPECTED_HASH 0x29D0B92D
#else
#define EXPECTED_SLEEP_STEP 295
#define EXPECTED_HASH 0x36564C94
#endif

typedef struct FallingHingeData
{
	b2BodyId* bodyIds;
	int bodyCount;
	int stepCount;
	int sleepStep;
	uint32_t hash;
} FallingHingeData;

FallingHingeData CreateFallingHinges( b2WorldId worldId );
bool UpdateFallingHinges( b2WorldId worldId, FallingHingeData* data );
void DestroyFallingHinges( FallingHingeData* data );

#ifdef __cplusplus
}
#endif
