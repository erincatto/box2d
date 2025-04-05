// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "id_pool.h"

b2IdPool b2CreateIdPool( void )
{
	b2IdPool pool = { 0 };
	pool.freeArray = b2IntArray_Create( 32 );
	return pool;
}

void b2DestroyIdPool( b2IdPool* pool )
{
	b2IntArray_Destroy( &pool->freeArray );
	*pool = ( b2IdPool ){ 0 };
}

int b2AllocId( b2IdPool* pool )
{
	int count = pool->freeArray.count;
	if ( count > 0 )
	{
		int id = b2IntArray_Pop( &pool->freeArray );
		return id;
	}

	int id = pool->nextIndex;
	pool->nextIndex += 1;
	return id;
}

void b2FreeId( b2IdPool* pool, int id )
{
	B2_ASSERT( pool->nextIndex > 0 );
	B2_ASSERT( 0 <= id && id < pool->nextIndex );
	b2IntArray_Push( &pool->freeArray, id );
}

#if B2_VALIDATE

void b2ValidateFreeId( b2IdPool* pool, int id )
{
	int freeCount = pool->freeArray.count;
	for ( int i = 0; i < freeCount; ++i )
	{
		if ( pool->freeArray.data[i] == id )
		{
			return;
		}
	}

	B2_ASSERT( 0 );
}

void b2ValidateUsedId( b2IdPool* pool, int id )
{
	int freeCount = pool->freeArray.count;
	for ( int i = 0; i < freeCount; ++i )
	{
		if ( pool->freeArray.data[i] == id )
		{
			B2_ASSERT( 0 );
		}
	}
}

#else

void b2ValidateFreeId( b2IdPool* pool, int id )
{
	B2_UNUSED( pool, id );
}

void b2ValidateUsedId( b2IdPool* pool, int id )
{
	B2_UNUSED( pool, id );
}
#endif
