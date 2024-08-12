// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"

typedef struct b2IdPool
{
	int* freeArray;
	int nextIndex;
} b2IdPool;

b2IdPool b2CreateIdPool();
void b2DestroyIdPool( b2IdPool* pool );

int b2AllocId( b2IdPool* pool );
void b2FreeId( b2IdPool* pool, int id );
void b2ValidateFreeId( b2IdPool* pool, int id );

static inline int b2GetIdCount( b2IdPool* pool )
{
	return pool->nextIndex - b2Array( pool->freeArray ).count;
}

static inline int b2GetIdCapacity( b2IdPool* pool )
{
	return pool->nextIndex;
}

static inline int b2GetIdBytes( b2IdPool* pool )
{
	return b2GetArrayBytes( pool->freeArray, sizeof( int ) );
}
