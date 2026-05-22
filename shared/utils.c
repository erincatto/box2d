// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "utils.h"

#if defined( _WIN64 )
#include <Windows.h>
#elif defined( __APPLE__ )
#include <unistd.h>
#elif defined( __linux__ )
#include <unistd.h>
#elif defined( __EMSCRIPTEN__ )
#include <unistd.h>
#endif

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

int GetNumberOfCores( void )
{
#if defined( _WIN64 )
	SYSTEM_INFO sysinfo;
	GetSystemInfo( &sysinfo );
	return sysinfo.dwNumberOfProcessors;
#elif defined( __APPLE__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __linux__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __EMSCRIPTEN__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#else
	return 1;
#endif
}
