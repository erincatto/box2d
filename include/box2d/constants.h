// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_functions.h"

// Used to detect bad values. Positions greater than about 16km will have precision
// problems, so 100km as a limit should be fine in all cases.
#define B2_HUGE ( 100000.0f * b2GetLengthUnitsPerMeter() )

// Maximum parallel workers. Used to size some static arrays.
#define B2_MAX_WORKERS 64

// Maximum number of colors in the constraint graph. Constraints that cannot
// find a color are added to the overflow set which are solved single-threaded.
// The compound barrel benchmark has minor overflow with 24 colors 
#define B2_GRAPH_COLOR_COUNT 24

// A small length used as a collision and constraint tolerance. Usually it is
// chosen to be numerically significant, but visually insignificant. In meters.
// Normally this is 0.5cm.
// @warning modifying this can have a significant impact on stability
#define B2_LINEAR_SLOP ( 0.005f * b2GetLengthUnitsPerMeter() )

// Maximum number of simultaneous worlds that can be allocated
#ifndef B2_MAX_WORLDS
#define B2_MAX_WORLDS 128
#endif

// The maximum rotation of a body per time step. This limit is very large and is used
// to prevent numerical problems. You shouldn't need to adjust this.
// @warning increasing this to 0.5f * b2_pi or greater will break continuous collision.
#define B2_MAX_ROTATION ( 0.25f * B2_PI )

// Box2D uses limited speculative collision. This reduces jitter.
// Normally this is 2cm.
// @warning modifying this can have a significant impact on performance and stability
#define B2_SPECULATIVE_DISTANCE ( 4.0f * B2_LINEAR_SLOP )

// The default contact recycling distance.
#define B2_RECYCLING_DISTANCE ( 10.0f * B2_LINEAR_SLOP )

// This is used to fatten AABBs in the dynamic tree. This allows proxies
// to move by a small amount without triggering a tree adjustment. This is in meters.
// Normally this is 5cm.
// @warning modifying this can have a significant impact on performance
#define B2_MAX_AABB_MARGIN ( 0.05f * b2GetLengthUnitsPerMeter() )

// For small objects the margin is limited to this fraction times the maximum extent
#define B2_AABB_MARGIN_FRACTION 0.125f

// The time that a body must be still before it will go to sleep. In seconds.
#define B2_TIME_TO_SLEEP 0.5f
