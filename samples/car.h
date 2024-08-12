// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

class Car
{
public:
	Car();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale, float hertz, float dampingRatio, float torque, void* userData );
	void Despawn();

	void SetSpeed( float speed );
	void SetTorque( float torque );
	void SetHertz( float hertz );
	void SetDampingRadio( float dampingRatio );

	b2BodyId m_chassisId;
	b2BodyId m_rearWheelId;
	b2BodyId m_frontWheelId;
	b2JointId m_rearAxleId;
	b2JointId m_frontAxleId;
	bool m_isSpawned;
};

class Truck
{
public:
	Truck();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale, float hertz, float dampingRatio, float torque, float density,
				void* userData );
	void Despawn();

	void SetSpeed( float speed );
	void SetTorque( float torque );
	void SetHertz( float hertz );
	void SetDampingRadio( float dampingRatio );

	b2BodyId m_chassisId;
	b2BodyId m_rearWheelId;
	b2BodyId m_frontWheelId;
	b2JointId m_rearAxleId;
	b2JointId m_frontAxleId;
	bool m_isSpawned;
};
