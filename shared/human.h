// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct Human Human;

B2_API Human* CreateHuman( b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio,
						   int groupIndex, void* userData, bool colorize );

B2_API void DestroyHuman( Human* human );

B2_API void Human_ApplyRandomAngularImpulse( Human* human, float magnitude );
B2_API void Human_SetJointFrictionTorque( Human* human, float torque );
B2_API void Human_SetJointSpringHertz( Human* human, float hertz );
B2_API void Human_SetJointDampingRatio( Human* human, float dampingRatio );
