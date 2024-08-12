// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

class Doohickey
{
public:
	Doohickey();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale );
	void Despawn();

	b2BodyId m_wheelId1;
	b2BodyId m_wheelId2;
	b2BodyId m_barId1;
	b2BodyId m_barId2;

	b2JointId m_axleId1;
	b2JointId m_axleId2;
	b2JointId m_sliderId;

	bool m_isSpawned;
};
