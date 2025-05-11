// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "random.h"

uint32_t g_randomSeed = RAND_SEED;

b2Polygon RandomPolygon( float extent )
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	int count = 3 + RandomInt() % 6;
	for ( int i = 0; i < count; ++i )
	{
		points[i] = RandomVec2( -extent, extent );
	}

	b2Hull hull = b2ComputeHull( points, count );
	if ( hull.count > 0 )
	{
		return b2MakePolygon( &hull, 0.0f );
	}

	return b2MakeSquare( extent );
}
