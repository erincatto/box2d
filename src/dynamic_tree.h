// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"

#define B2_TREE_STACK_SIZE 1024

static inline bool b2IsLeaf( const b2TreeNode* node )
{
	return node->flags & b2_leafNode;
}

static inline bool b2IsAllocated( const b2TreeNode* node )
{
	return node->flags & b2_allocatedNode;
}

void b2DynamicTree_MarkEnlargedFlag( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_RefitEnlarged( b2DynamicTree* tree, int proxyId );
void b2DynamicTree_ClearEnlarged( b2DynamicTree* tree );
