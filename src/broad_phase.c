// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "broad_phase.h"

#include "aabb.h"
#include "allocate.h"
#include "array.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "shape.h"
#include "stack_allocator.h"
#include "world.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>

// #include <stdio.h>

// static FILE* s_file = NULL;

void b2CreateBroadPhase( b2BroadPhase* bp )
{
	_Static_assert( b2_bodyTypeCount == 3, "must be three body types" );

	// if (s_file == NULL)
	//{
	//	s_file = fopen("pairs01.txt", "a");
	//	fprintf(s_file, "============\n\n");
	// }

	bp->proxyCount = 0;
	bp->moveSet = b2CreateSet( 16 );
	bp->moveArray = b2CreateArray( sizeof( int ), 16 );
	bp->moveResults = NULL;
	bp->movePairs = NULL;
	bp->movePairCapacity = 0;
	bp->movePairIndex = 0;
	bp->pairSet = b2CreateSet( 32 );

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		bp->trees[i] = b2DynamicTree_Create();
	}
}

void b2DestroyBroadPhase( b2BroadPhase* bp )
{
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Destroy( bp->trees + i );
	}

	b2DestroySet( &bp->moveSet );
	b2DestroyArray( bp->moveArray, sizeof( int ) );
	b2DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b2BroadPhase ) );

	// if (s_file != NULL)
	//{
	//	fclose(s_file);
	//	s_file = NULL;
	// }
}

static inline void b2UnBufferMove( b2BroadPhase* bp, int proxyKey )
{
	bool found = b2RemoveKey( &bp->moveSet, proxyKey + 1 );

	if ( found )
	{
		// Purge from move buffer. Linear search.
		// todo if I can iterate the move set then I don't need the moveArray
		int count = b2Array( bp->moveArray ).count;
		for ( int i = 0; i < count; ++i )
		{
			if ( bp->moveArray[i] == proxyKey )
			{
				b2Array_RemoveSwap( bp->moveArray, i );
				break;
			}
		}
	}
}

int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint32_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );
	int proxyId = b2DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, shapeIndex );
	int proxyKey = B2_PROXY_KEY( proxyId, proxyType );
	if ( proxyType != b2_staticBody || forcePairCreation )
	{
		b2BufferMove( bp, proxyKey );
	}
	return proxyKey;
}

void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey )
{
	B2_ASSERT( b2Array( bp->moveArray ).count == (int)bp->moveSet.count );
	b2UnBufferMove( bp, proxyKey );

	--bp->proxyCount;

	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( 0 <= proxyType && proxyType <= b2_bodyTypeCount );
	b2DynamicTree_DestroyProxy( bp->trees + proxyType, proxyId );
}

void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	b2DynamicTree_MoveProxy( bp->trees + proxyType, proxyId, aabb );
	b2BufferMove( bp, proxyKey );
}

void b2BroadPhase_EnlargeProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
	B2_ASSERT( proxyKey != B2_NULL_INDEX );
	int typeIndex = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( typeIndex != b2_staticBody );

	b2DynamicTree_EnlargeProxy( bp->trees + typeIndex, proxyId, aabb );
	b2BufferMove( bp, proxyKey );
}

typedef struct b2MovePair
{
	int shapeIndexA;
	int shapeIndexB;
	b2MovePair* next;
	bool heap;
} b2MovePair;

typedef struct b2MoveResult
{
	b2MovePair* pairList;
} b2MoveResult;

typedef struct b2QueryPairContext
{
	b2World* world;
	b2MoveResult* moveResult;
	b2BodyType queryTreeType;
	int queryProxyKey;
	int queryShapeIndex;
} b2QueryPairContext;

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2PairQueryCallback( int proxyId, int shapeId, void* context )
{
	b2QueryPairContext* queryContext = context;
	b2BroadPhase* bp = &queryContext->world->broadPhase;

	int proxyKey = B2_PROXY_KEY( proxyId, queryContext->queryTreeType );

	// A proxy cannot form a pair with itself.
	if ( proxyKey == queryContext->queryProxyKey )
	{
		return true;
	}

	// Is this proxy also moving?
	if ( queryContext->queryTreeType != b2_staticBody )
	{
		bool moved = b2ContainsKey( &bp->moveSet, proxyKey + 1 );
		if ( moved && proxyKey < queryContext->queryProxyKey )
		{
			// Both proxies are moving. Avoid duplicate pairs.
			return true;
		}
	}

	uint64_t pairKey = B2_SHAPE_PAIR_KEY( shapeId, queryContext->queryShapeIndex );
	if ( b2ContainsKey( &bp->pairSet, pairKey ) )
	{
		// contact exists
		return true;
	}

	int shapeIdA, shapeIdB;
	if ( proxyKey < queryContext->queryProxyKey )
	{
		shapeIdA = shapeId;
		shapeIdB = queryContext->queryShapeIndex;
	}
	else
	{
		shapeIdA = queryContext->queryShapeIndex;
		shapeIdB = shapeId;
	}

	b2World* world = queryContext->world;

	b2CheckId( world->shapeArray, shapeIdA );
	b2CheckId( world->shapeArray, shapeIdB );

	b2Shape* shapeA = world->shapeArray + shapeIdA;
	b2Shape* shapeB = world->shapeArray + shapeIdB;

	int bodyIdA = shapeA->bodyId;
	int bodyIdB = shapeB->bodyId;

	// Are the shapes on the same body?
	if ( bodyIdA == bodyIdB )
	{
		return true;
	}

	if ( b2ShouldShapesCollide( shapeA->filter, shapeB->filter ) == false )
	{
		return true;
	}

	// Sensors don't collide with other sensors
	if ( shapeA->isSensor == true && shapeB->isSensor == true )
	{
		return true;
	}

	// Does a joint override collision?
	b2Body* bodyA = b2GetBody( world, bodyIdA );
	b2Body* bodyB = b2GetBody( world, bodyIdB );
	if ( b2ShouldBodiesCollide( world, bodyA, bodyB ) == false )
	{
		return true;
	}

	// Custom user filter
	b2CustomFilterFcn* customFilterFcn = queryContext->world->customFilterFcn;
	if ( customFilterFcn != NULL )
	{
		b2ShapeId idA = { shapeIdA + 1, world->worldId, shapeA->revision };
		b2ShapeId idB = { shapeIdB + 1, world->worldId, shapeB->revision };
		bool shouldCollide = customFilterFcn( idA, idB, queryContext->world->customFilterContext );
		if ( shouldCollide == false )
		{
			return true;
		}
	}

	// #todo per thread to eliminate atomic?
	int pairIndex = atomic_fetch_add( &bp->movePairIndex, 1 );

	b2MovePair* pair;
	if ( pairIndex < bp->movePairCapacity )
	{
		pair = bp->movePairs + pairIndex;
		pair->heap = false;
	}
	else
	{
		pair = b2Alloc( sizeof( b2MovePair ) );
		pair->heap = true;
	}

	pair->shapeIndexA = shapeIdA;
	pair->shapeIndexB = shapeIdB;
	pair->next = queryContext->moveResult->pairList;
	queryContext->moveResult->pairList = pair;

	// continue the query
	return true;
}

void b2FindPairsTask( int startIndex, int endIndex, uint32_t threadIndex, void* context )
{
	b2TracyCZoneNC( pair_task, "Pair Task", b2_colorAquamarine, true );

	B2_MAYBE_UNUSED( threadIndex );

	b2World* world = context;
	b2BroadPhase* bp = &world->broadPhase;

	b2QueryPairContext queryContext;
	queryContext.world = world;

	for ( int i = startIndex; i < endIndex; ++i )
	{
		// Initialize move result for this moved proxy
		queryContext.moveResult = bp->moveResults + i;
		queryContext.moveResult->pairList = NULL;

		int proxyKey = bp->moveArray[i];
		if ( proxyKey == B2_NULL_INDEX )
		{
			// proxy was destroyed after it moved
			continue;
		}

		b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );

		int proxyId = B2_PROXY_ID( proxyKey );
		queryContext.queryProxyKey = proxyKey;

		const b2DynamicTree* baseTree = bp->trees + proxyType;

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a contact that may touch later.
		b2AABB fatAABB = b2DynamicTree_GetAABB( baseTree, proxyId );
		queryContext.queryShapeIndex = b2DynamicTree_GetUserData( baseTree, proxyId );

		// Query trees. Only dynamic proxies collide with kinematic and static proxies
		if ( proxyType == b2_dynamicBody )
		{
			queryContext.queryTreeType = b2_kinematicBody;
			b2DynamicTree_Query( bp->trees + b2_kinematicBody, fatAABB, b2_defaultMaskBits, b2PairQueryCallback, &queryContext );

			queryContext.queryTreeType = b2_staticBody;
			b2DynamicTree_Query( bp->trees + b2_staticBody, fatAABB, b2_defaultMaskBits, b2PairQueryCallback, &queryContext );
		}

		// All proxies collide with dynamic proxies
		queryContext.queryTreeType = b2_dynamicBody;
		b2DynamicTree_Query( bp->trees + b2_dynamicBody, fatAABB, b2_defaultMaskBits, b2PairQueryCallback, &queryContext );
	}

	b2TracyCZoneEnd( pair_task );
}

void b2UpdateBroadPhasePairs( b2World* world )
{
	b2BroadPhase* bp = &world->broadPhase;

	int moveCount = b2Array( bp->moveArray ).count;
	B2_ASSERT( moveCount == (int)bp->moveSet.count );

	if ( moveCount == 0 )
	{
		return;
	}

	b2TracyCZoneNC( update_pairs, "Pairs", b2_colorFuchsia, true );

	b2StackAllocator* alloc = &world->stackAllocator;

	// todo these could be in the step context
	bp->moveResults = b2AllocateStackItem( alloc, moveCount * sizeof( b2MoveResult ), "move results" );
	bp->movePairCapacity = 16 * moveCount;
	bp->movePairs = b2AllocateStackItem( alloc, bp->movePairCapacity * sizeof( b2MovePair ), "move pairs" );
	bp->movePairIndex = 0;

#ifndef NDEBUG
	extern _Atomic int g_probeCount;
	g_probeCount = 0;
#endif

	int minRange = 64;
	void* userPairTask = world->enqueueTaskFcn( &b2FindPairsTask, moveCount, minRange, world, world->userTaskContext );
	world->finishTaskFcn( userPairTask, world->userTaskContext );
	world->taskCount += 1;

	b2TracyCZoneNC( create_contacts, "Create Contacts", b2_colorGold, true );

	// Single-threaded work
	// - Clear move flags
	// - Create contacts in deterministic order
	b2Shape* shapes = world->shapeArray;

	for ( int i = 0; i < moveCount; ++i )
	{
		b2MoveResult* result = bp->moveResults + i;
		b2MovePair* pair = result->pairList;
		while ( pair != NULL )
		{
			// TODO_ERIN Check user filtering.
			// if (m_contactFilter && m_contactFilter->ShouldCollide(shapeA, shapeB) == false)
			//{
			//	return;
			//}

			int shapeIdA = pair->shapeIndexA;
			int shapeIdB = pair->shapeIndexB;

			// if (s_file != NULL)
			//{
			//	fprintf(s_file, "%d %d\n", shapeIdA, shapeIdB);
			// }

			//++pairCount;

			b2CheckId( shapes, shapeIdA );
			b2CheckId( shapes, shapeIdB );

			b2CreateContact( world, shapes + shapeIdA, shapes + shapeIdB );

			if ( pair->heap )
			{
				b2MovePair* temp = pair;
				pair = pair->next;
				b2Free( temp, sizeof( b2MovePair ) );
			}
			else
			{
				pair = pair->next;
			}
		}

		// if (s_file != NULL)
		//{
		//	fprintf(s_file, "\n");
		// }
	}

	// if (s_file != NULL)
	//{
	//	fprintf(s_file, "count = %d\n\n", pairCount);
	// }

	// Reset move buffer
	b2Array_Clear( bp->moveArray );
	b2ClearSet( &bp->moveSet );

	b2FreeStackItem( alloc, bp->movePairs );
	bp->movePairs = NULL;
	b2FreeStackItem( alloc, bp->moveResults );
	bp->moveResults = NULL;

	b2ValidateSolverSets( world );

	b2TracyCZoneEnd( create_contacts );

	b2TracyCZoneEnd( update_pairs );
}

bool b2BroadPhase_TestOverlap( const b2BroadPhase* bp, int proxyKeyA, int proxyKeyB )
{
	int typeIndexA = B2_PROXY_TYPE( proxyKeyA );
	int proxyIdA = B2_PROXY_ID( proxyKeyA );
	int typeIndexB = B2_PROXY_TYPE( proxyKeyB );
	int proxyIdB = B2_PROXY_ID( proxyKeyB );

	b2AABB aabbA = b2DynamicTree_GetAABB( bp->trees + typeIndexA, proxyIdA );
	b2AABB aabbB = b2DynamicTree_GetAABB( bp->trees + typeIndexB, proxyIdB );
	return b2AABB_Overlaps( aabbA, aabbB );
}

void b2BroadPhase_RebuildTrees( b2BroadPhase* bp )
{
	b2DynamicTree_Rebuild( bp->trees + b2_dynamicBody, false );
	b2DynamicTree_Rebuild( bp->trees + b2_kinematicBody, false );
}

int b2BroadPhase_GetShapeIndex( b2BroadPhase* bp, int proxyKey )
{
	int typeIndex = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	return b2DynamicTree_GetUserData( bp->trees + typeIndex, proxyId );
}

void b2ValidateBroadphase( const b2BroadPhase* bp )
{
	b2DynamicTree_Validate( bp->trees + b2_dynamicBody );
	b2DynamicTree_Validate( bp->trees + b2_kinematicBody );

	// TODO_ERIN validate every shape AABB is contained in tree AABB
}

void b2ValidateNoEnlarged( const b2BroadPhase* bp )
{
#if B2_VALIDATE == 1
	for ( int j = 0; j < b2_bodyTypeCount; ++j )
	{
		const b2DynamicTree* tree = bp->trees + j;
		int capacity = tree->nodeCapacity;
		const b2TreeNode* nodes = tree->nodes;
		for ( int i = 0; i < capacity; ++i )
		{
			const b2TreeNode* node = nodes + i;
			if ( node->height < 0 )
			{
				continue;
			}

			if ( node->enlarged == true )
			{
				capacity += 0;
			}

			B2_ASSERT( node->enlarged == false );
		}
	}
#else
	B2_MAYBE_UNUSED( bp );
#endif
}
