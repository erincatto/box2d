// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"

#include <stdint.h>

typedef struct b2World b2World;

// Build a dynamic tree from a flat array of leaf AABBs in one shot.
//
// The tree is reset in place: any existing contents are discarded. The result
// has leafCount leaves at indices [0, leafCount) (input element i becomes
// leaf i) and leafCount-1 internal nodes at indices [leafCount, 2*leafCount-1)
// assigned in DFS preorder. The build uses the midpoint-of-centroid heuristic
// (longest centroid-AABB axis, split at midpoint).
//
// userData and categoryBits may be NULL. NULL userData zeros each leaf's
// userData. NULL categoryBits applies B2_DEFAULT_CATEGORY_BITS.
//
// world enables parallelism. If world is NULL or world->workerCount is 1, the
// build runs serially. The result is bit-identical for a given leaf set
// regardless of worker count.
void b2DynamicTree_BuildFromLeaves( b2DynamicTree* tree, const b2AABB* leafAabbs, const uint64_t* leafUserData,
									const uint64_t* leafCategoryBits, int leafCount, b2World* world );
