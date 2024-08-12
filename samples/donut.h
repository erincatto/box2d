// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

class Donut
{
	enum
	{
		e_sides = 7
	};

public:
	Donut();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale, int groupIndex, void* userData );
	void Despawn();

	b2BodyId m_bodyIds[e_sides];
	b2JointId m_jointIds[e_sides];
	bool m_isSpawned;
};
