// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include "box2d/id.h"

#include <stdint.h>

typedef struct b2World b2World;

// Serialize the complete simulation state of world into buf.
// Must be called at a step boundary (between b2World_Step calls).
// Reuses b2RecBuffer/b2RecBufAppend for output.
void b2SerializeWorld( b2World* world, b2RecBuffer* buf );

// Reconstruct a world from a snapshot image produced by b2SerializeWorld.
// workerCount 0 uses the serial single-worker fallback.
// Returns b2_nullWorldId on failure (corrupt image, layout mismatch, no free world slot).
b2WorldId b2DeserializeWorld( const uint8_t* image, int size, int workerCount );

// Extends b2HashWorldState with index bookkeeping and accumulated impulses,
// making round-trip bugs visible immediately rather than several steps later.
uint64_t b2HashWorldStateDeep( b2World* world );
