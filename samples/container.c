// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "container.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

void* GrowAlloc( void* oldMem, int oldSize, int newSize )
{
	assert( newSize > oldSize );
	void* newMem = malloc( newSize );
	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		free( oldMem );
	}

	return newMem;
}
