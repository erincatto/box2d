// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

#include <stdint.h>

/**
 * @defgroup id Ids
 * These ids serve as handles to internal Box2D objects.
 * These should be considered opaque data and passed by value.
 * Include this header if you need the id types and not the whole Box2D API.
 * All ids are considered null if initialized to zero.
 *
 * For example in C++:
 *
 * @code{.cxx}
 * b2WorldId worldId = {};
 * @endcode
 *
 * Or in C:
 *
 * @code{.c}
 * b2WorldId worldId = {0};
 * @endcode
 *
 * These are both considered null.
 *
 * @warning Do not use the internals of these ids. They are subject to change. Ids should be treated as opaque objects.
 * @warning You should use ids to access objects in Box2D. Do not access files within the src folder. Such usage is unsupported.
 * @{
 */

/// World id references a world instance. This should be treated as an opaque handle.
typedef struct b2WorldId
{
	uint16_t index1;
	uint16_t revision;
} b2WorldId;

/// Body id references a body instance. This should be treated as an opaque handle.
typedef struct b2BodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2BodyId;

/// Shape id references a shape instance. This should be treated as an opaque handle.
typedef struct b2ShapeId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2ShapeId;

/// Joint id references a joint instance. This should be treated as an opaque handle.
typedef struct b2JointId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2JointId;

/// Chain id references a chain instances. This should be treated as an opaque handle.
typedef struct b2ChainId
{
	int32_t index1;
	uint16_t world0;
	uint16_t revision;
} b2ChainId;

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
static const b2WorldId b2_nullWorldId = B2_ZERO_INIT;
static const b2BodyId b2_nullBodyId = B2_ZERO_INIT;
static const b2ShapeId b2_nullShapeId = B2_ZERO_INIT;
static const b2JointId b2_nullJointId = B2_ZERO_INIT;
static const b2ChainId b2_nullChainId = B2_ZERO_INIT;

/// Macro to determine if any id is null.
#define B2_IS_NULL( id ) ( id.index1 == 0 )

/// Macro to determine if any id is non-null.
#define B2_IS_NON_NULL( id ) ( id.index1 != 0 )

/// Compare two ids for equality. Doesn't work for b2WorldId.
#define B2_ID_EQUALS( id1, id2 ) ( id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.revision == id2.revision )

/**@}*/
