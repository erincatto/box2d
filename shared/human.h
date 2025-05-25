// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef enum BoneId
{
	bone_hip = 0,
	bone_torso = 1,
	bone_head = 2,
	bone_upperLeftLeg = 3,
	bone_lowerLeftLeg = 4,
	bone_upperRightLeg = 5,
	bone_lowerRightLeg = 6,
	bone_upperLeftArm = 7,
	bone_lowerLeftArm = 8,
	bone_upperRightArm = 9,
	bone_lowerRightArm = 10,
	bone_count = 11,
} BoneId;

typedef struct Bone
{
	b2BodyId bodyId;
	b2JointId jointId;
	float frictionScale;
	float maxTorque;
	int parentIndex;
} Bone;

typedef struct Human
{
	Bone bones[bone_count];
	float frictionTorque;
	float originalScale;
	float scale;
	bool isSpawned;
} Human;

#ifdef __cplusplus
extern "C"
{
#endif

void CreateHuman( Human* human, b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz,
				  float dampingRatio, int groupIndex, void* userData, bool colorize );

void DestroyHuman( Human* human );

void Human_SetVelocity( Human* human, b2Vec2 velocity );
void Human_ApplyRandomAngularImpulse( Human* human, float magnitude );
void Human_SetJointFrictionTorque( Human* human, float torque );
void Human_SetJointSpringHertz( Human* human, float hertz );
void Human_SetJointDampingRatio( Human* human, float dampingRatio );
void Human_EnableSensorEvents( Human* human, bool enable );
void Human_SetScale( Human* human, float scale );

#ifdef __cplusplus
}
#endif
