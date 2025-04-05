// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

class Donut
{
public:
	Donut();

	void Create( b2WorldId worldId, b2Vec2 position, float scale, int groupIndex, bool enableSensorEvents, void* userData );
	void Destroy();

	static constexpr int m_sides = 7;
	b2BodyId m_bodyIds[m_sides];
	b2JointId m_jointIds[m_sides];
	bool m_isSpawned;
};
