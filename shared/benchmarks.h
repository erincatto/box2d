// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "human.h"

#ifdef NDEBUG
enum RainConstants
{
	RAIN_ROW_COUNT = 5,
	RAIN_COLUMN_COUNT = 40,
	RAIN_GROUP_SIZE = 5,
};
#else
enum RainConstants
{
	RAIN_ROW_COUNT = 3,
	RAIN_COLUMN_COUNT = 10,
	RAIN_GROUP_SIZE = 2,
};
#endif

typedef struct Group
{
	Human humans[RAIN_GROUP_SIZE];
} Group;

typedef struct RainData
{
	Group groups[RAIN_ROW_COUNT * RAIN_COLUMN_COUNT];
	float gridSize;
	int gridCount;
	int columnCount;
	int columnIndex;
} RainData;

B2_API void CreateRain( b2WorldId worldId );
B2_API void StepRain( b2WorldId worldId, int stepCount );
