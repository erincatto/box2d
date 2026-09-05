// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_functions.h"

typedef struct b2DynamicTree b2DynamicTree;

void b2DynamicTree_MarkEnlarged( b2DynamicTree* tree, int proxyId, b2AABB aabb );
void b2DynamicTree_RefitEnlarged( b2DynamicTree* tree, int proxyId );
