// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdint.h>

typedef struct b2World b2World;

// Serialize the complete simulation state of world into buf. Backs the public
// b2World_Snapshot. Must be called at a step boundary (between b2World_Step calls).
// Reuses b2RecBuffer/b2RecBufAppend for output.
void b2SerializeWorld( b2World* world, b2RecBuffer* buf );

// Extensive hash for testing
uint64_t b2HashWorldStateDeep( b2World* world );
