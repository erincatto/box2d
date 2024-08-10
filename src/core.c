// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/math_functions.h"

#include <stdio.h>

float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	B2_ASSERT( b2IsValid( lengthUnits ) && lengthUnits > 0.0f );
	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter( void )
{
	return b2_lengthUnitsPerMeter;
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );

	// return non-zero to break to debugger
	return 1;
}

b2AssertFcn* b2AssertHandler = b2DefaultAssertFcn;

void b2SetAssertFcn( b2AssertFcn* assertFcn )
{
	B2_ASSERT( assertFcn != NULL );
	b2AssertHandler = assertFcn;
}

b2Version b2GetVersion( void )
{
	return ( b2Version ){ 3, 0, 0 };
}
