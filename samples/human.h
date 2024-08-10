// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

struct Bone
{
	enum
	{
		e_hip = 0,
		e_torso = 1,
		e_head = 2,
		e_upperLeftLeg = 3,
		e_lowerLeftLeg = 4,
		e_upperRightLeg = 5,
		e_lowerRightLeg = 6,
		e_upperLeftArm = 7,
		e_lowerLeftArm = 8,
		e_upperRightArm = 9,
		e_lowerRightArm = 10,
		e_count = 11,
	};

	b2BodyId bodyId;
	b2JointId jointId;
	float frictionScale;
	int parentIndex;
};

class Human
{
public:
	Human();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio,
				int groupIndex, void* userData, bool colorize );
	void Despawn();

	void ApplyRandomAngularImpulse( float magnitude );
	void SetJointFrictionTorque( float torque );
	void SetJointSpringHertz( float hertz );
	void SetJointDampingRatio( float dampingRatio );

	Bone m_bones[Bone::e_count];
	float m_scale;
	bool m_isSpawned;
};
