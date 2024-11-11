// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef enum BoneId
{
	boneId_hip = 0,
	boneId_torso = 1,
	boneId_head = 2,
	boneId_upperLeftLeg = 3,
	boneId_lowerLeftLeg = 4,
	boneId_upperRightLeg = 5,
	boneId_lowerRightLeg = 6,
	boneId_upperLeftArm = 7,
	boneId_lowerLeftArm = 8,
	boneId_upperRightArm = 9,
	boneId_lowerRightArm = 10,
	boneId_count = 11,
} BoneId;

typedef struct Bone
{
	b2BodyId bodyId;
	b2JointId jointId;
	float frictionScale;
	int parentIndex;
} Bone;

typedef struct Human
{
	Bone bones[boneId_count];
	float scale;
	bool isSpawned;
} Human;

B2_API void CreateHuman( Human* human, b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio,
						   int groupIndex, void* userData, bool colorize );

B2_API void DestroyHuman( Human* human );

B2_API void Human_ApplyRandomAngularImpulse( Human* human, float magnitude );
B2_API void Human_SetJointFrictionTorque( Human* human, float torque );
B2_API void Human_SetJointSpringHertz( Human* human, float hertz );
B2_API void Human_SetJointDampingRatio( Human* human, float dampingRatio );
