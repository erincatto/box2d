// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "broad_phase.h"

#include "aabb.h"
#include "arena_allocator.h"
#include "atomic.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "ctz.h"
#include "dynamic_tree.h"
#include "parallel_for.h"
#include "physics_world.h"
#include "shape.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

// #include <stdio.h>

// static FILE* s_file = NULL;

// Totals over the life of the world, printed when the broad phase is destroyed.
// Tasks add their local sums once so the workers never share a cache line.
#if B2_SNOOP_PAIR_COUNTERS
static b2AtomicInt b2_queryNodeVisits;
static b2AtomicInt b2_queryLeafVisits;
static b2AtomicInt b2_queryDynamicNodeVisits;
static b2AtomicInt b2_queryDynamicLeafVisits;
static b2AtomicInt b2_queryCandidates;
static b2AtomicInt b2_querySurvivors;
static b2AtomicInt b2_selfPops;
static b2AtomicInt b2_selfCandidates;
static b2AtomicInt b2_selfSurvivors;
static b2AtomicInt b2_pairSteps;
static int b2_queryHeapPairs;
static int b2_selfHeapPairs;
static int b2_queryMaxSurvivors;
static int b2_selfMaxSurvivors;
static int b2_selfMaxCapacity;
#endif

// Pass timing for the C.6 gate. The passes alternate order each step so each is timed cold
// as often as warm, otherwise the second pass inherits the first one's cache.
#define B2_SNOOP_PAIR_TIMING 0

#if B2_SNOOP_PAIR_TIMING
static float b2_queryFirstMs;
static float b2_querySecondMs;
static float b2_selfFirstMs;
static float b2_selfSecondMs;
static int b2_queryFirstCount;
static int b2_selfFirstCount;
static int b2_timingSteps;
#endif

void b2CreateBroadPhase( b2BroadPhase* bp, const b2Capacity* capacity )
{
	_Static_assert( b2_bodyTypeCount == 3, "must be three body types" );

	// if (s_file == NULL)
	//{
	//	s_file = fopen("pairs01.txt", "a");
	//	fprintf(s_file, "============\n\n");
	// }

	bp->movedProxies[b2_staticBody] = b2CreateBitSet( b2MaxInt( 16, capacity->staticShapeCount ) );
	bp->movedProxies[b2_kinematicBody] = b2CreateBitSet( 16 );
	bp->movedProxies[b2_dynamicBody] = b2CreateBitSet( b2MaxInt( 16, capacity->dynamicShapeCount ) );
	b2Array_CreateN( bp->moveArray, b2MaxInt( 16, capacity->dynamicShapeCount ) );
	bp->moveResults = NULL;
	bp->movePairs = NULL;
	bp->movePairCapacity = 0;
	b2AtomicStoreInt( &bp->movePairIndex, 0 );
	bp->pairSet = b2CreateSet( b2MaxInt( 32, 2 * capacity->contactCount ) );

	int staticCapacity = b2MaxInt( 16, capacity->staticShapeCount );
	bp->trees[b2_staticBody] = b2DynamicTree_Create( staticCapacity );

	int kinematicCapacity = 16;
	bp->trees[b2_kinematicBody] = b2DynamicTree_Create( kinematicCapacity );

	int dynamicCapacity = b2MaxInt( 16, capacity->dynamicShapeCount );
	bp->trees[b2_dynamicBody] = b2DynamicTree_Create( dynamicCapacity );

	bp->movePairs2 = NULL;
	b2AtomicStoreInt( &bp->movePairIndex2, 0 );
	bp->movePairCapacity2 = 16;
	bp->moveCount2 = 0;
}

void b2DestroyBroadPhase( b2BroadPhase* bp )
{
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Destroy( bp->trees + i );
	}

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DestroyBitSet( &bp->movedProxies[i] );
	}
	b2Array_Destroy( bp->moveArray );
	b2DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b2BroadPhase ) );

#if B2_SNOOP_PAIR_COUNTERS
	int steps = b2AtomicLoadInt( &b2_pairSteps );
	// Unit tests run short, keep them quiet
	if ( steps >= 100 )
	{
		b2Log( "pair steps %d: query visits %d node %d leaf (dynamic tree %d node %d leaf), candidates %d, survivors %d | self "
			   "pops %d, candidates %d, survivors %d",
			   steps, b2AtomicLoadInt( &b2_queryNodeVisits ), b2AtomicLoadInt( &b2_queryLeafVisits ),
			   b2AtomicLoadInt( &b2_queryDynamicNodeVisits ), b2AtomicLoadInt( &b2_queryDynamicLeafVisits ),
			   b2AtomicLoadInt( &b2_queryCandidates ), b2AtomicLoadInt( &b2_querySurvivors ), b2AtomicLoadInt( &b2_selfPops ),
			   b2AtomicLoadInt( &b2_selfCandidates ), b2AtomicLoadInt( &b2_selfSurvivors ) );
		b2Log( "pair pools: query max survivors %d heap pairs %d | self max survivors %d heap pairs %d max capacity %d",
			   b2_queryMaxSurvivors, b2_queryHeapPairs, b2_selfMaxSurvivors, b2_selfHeapPairs, b2_selfMaxCapacity );

		b2AtomicStoreInt( &b2_queryNodeVisits, 0 );
		b2AtomicStoreInt( &b2_queryLeafVisits, 0 );
		b2AtomicStoreInt( &b2_queryDynamicNodeVisits, 0 );
		b2AtomicStoreInt( &b2_queryDynamicLeafVisits, 0 );
		b2AtomicStoreInt( &b2_queryCandidates, 0 );
		b2AtomicStoreInt( &b2_querySurvivors, 0 );
		b2AtomicStoreInt( &b2_selfPops, 0 );
		b2AtomicStoreInt( &b2_selfCandidates, 0 );
		b2AtomicStoreInt( &b2_selfSurvivors, 0 );
		b2AtomicStoreInt( &b2_pairSteps, 0 );
		b2_queryHeapPairs = 0;
		b2_selfHeapPairs = 0;
		b2_queryMaxSurvivors = 0;
		b2_selfMaxSurvivors = 0;
		b2_selfMaxCapacity = 0;
	}
#endif

#if B2_SNOOP_PAIR_TIMING
	if ( b2_timingSteps > 0 )
	{
		int querySecondCount = b2_timingSteps - b2_queryFirstCount;
		int selfSecondCount = b2_timingSteps - b2_selfFirstCount;
		b2Log( "pair timing %d steps, ms per step: query first %.4f second %.4f | self first %.4f second %.4f", b2_timingSteps,
			   b2_queryFirstMs / b2MaxInt( b2_queryFirstCount, 1 ), b2_querySecondMs / b2MaxInt( querySecondCount, 1 ),
			   b2_selfFirstMs / b2MaxInt( b2_selfFirstCount, 1 ), b2_selfSecondMs / b2MaxInt( selfSecondCount, 1 ) );

		b2_queryFirstMs = 0.0f;
		b2_querySecondMs = 0.0f;
		b2_selfFirstMs = 0.0f;
		b2_selfSecondMs = 0.0f;
		b2_queryFirstCount = 0;
		b2_selfFirstCount = 0;
		b2_timingSteps = 0;
	}
#endif

	// if (s_file != NULL)
	//{
	//	fclose(s_file);
	//	s_file = NULL;
	// }
}

static inline void b2UnBufferMove( b2BroadPhase* bp, int proxyKey )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );
	b2BitSet* set = &bp->movedProxies[proxyType];

	if ( b2GetBit( set, proxyId ) )
	{
		b2ClearBit( set, proxyId );

		// Purge from move buffer. Linear search.
		// todo if I can iterate the move set then I don't need the moveArray
		int count = bp->moveArray.count;
		for ( int i = 0; i < count; ++i )
		{
			if ( bp->moveArray.data[i] == proxyKey )
			{
				b2Array_RemoveSwap( bp->moveArray, i );
				break;
			}
		}
	}
}

int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );

	uint16_t flags = ( proxyType != b2_staticBody || forcePairCreation ) ? b2_enlargedNode : 0;

	int proxyId = b2DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, shapeIndex, flags );
	int proxyKey = B2_PROXY_KEY( proxyId, proxyType );

	if ( proxyType != b2_staticBody || forcePairCreation )
	{
		b2BufferMove( bp, proxyKey );
	}
	return proxyKey;
}

void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey )
{
	b2UnBufferMove( bp, proxyKey );

	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( 0 <= proxyType && proxyType <= b2_bodyTypeCount );
	b2DynamicTree_DestroyProxy( bp->trees + proxyType, proxyId );
}

void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	b2DynamicTree_MoveProxy( bp->trees + proxyType, proxyId, aabb, b2_enlargedNode );
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

static int b2GatherEnlargedNodes( const b2DynamicTree* tree, int* items )
{
	const b2TreeNode* nodes = tree->nodes;
	int capacity = tree->nodeCapacity;

	uint16_t mask = b2_allocatedNode | b2_enlargedNode | b2_leafNode;
	uint16_t required = b2_allocatedNode | b2_enlargedNode;

	int count = 0;
	for ( int i = 0; i < capacity; ++i )
	{
		if ( ( nodes[i].flags & mask ) == required )
		{
			items[count] = i;
			count += 1;
		}
	}

	return count;
}

#define B2_CANDIDATE_BATCH 32

typedef struct b2Candidate
{
	int shapeIdA;
	int shapeIdB;
	int item;
} b2Candidate;

typedef struct b2PairContext
{
	b2World* world;
	b2Candidate batch[B2_CANDIDATE_BATCH];
	int batchCount;
	int workerIndex;
	int item;

#if B2_SNOOP_PAIR_COUNTERS
	int pops;
	int candidates;
#endif
} b2PairContext;

typedef struct b2NodePair
{
	int a, b;
} b2NodePair;

typedef struct b2MovePair
{
	int shapeIdA;
	int shapeIdB;
	b2MovePair* next;
	bool heap;
} b2MovePair;

typedef struct b2MoveResult
{
	b2MovePair* pairList;
} b2MoveResult;

// todo profile with and without prefetch
static void b2DrainCandidates( b2PairContext* context )
{
	b2World* world = context->world;
	b2BroadPhase* bp = &world->broadPhase;

	int count1 = context->batchCount;
	context->batchCount = 0;

#if B2_SNOOP_PAIR_COUNTERS
	context->candidates += count1;
#endif

	uint64_t keys[B2_CANDIDATE_BATCH];
	uint64_t hashes[B2_CANDIDATE_BATCH];
	for ( int i = 0; i < count1; ++i )
	{
		b2Candidate* candidate = context->batch + i;
		keys[i] = B2_SHAPE_PAIR_KEY( candidate->shapeIdA, candidate->shapeIdB );
		hashes[i] = b2KeyHash( keys[i] );
		b2PrefetchHash( &bp->pairSet, hashes[i] );
	}

	b2Candidate culled[B2_CANDIDATE_BATCH];
	int count2 = 0;
	for ( int i = 0; i < count1; ++i )
	{
		bool pairExists = b2ContainsHashedKey( &bp->pairSet, keys[i], hashes[i] );
		if ( pairExists == false )
		{
			culled[count2] = context->batch[i];
			count2 += 1;
		}
	}

	const b2Shape* shapes = world->shapes.data;

	for ( int i = 0; i < count2; ++i )
	{
		b2Prefetch( shapes + culled[i].shapeIdA );
		b2Prefetch( shapes + culled[i].shapeIdB );
	}

	int count3 = 0;
	for ( int i = 0; i < count2; ++i )
	{
		int shapeIdA = culled[i].shapeIdA;
		int shapeIdB = culled[i].shapeIdB;

		b2Shape* shapeA = b2Array_Get( world->shapes, shapeIdA );
		b2Shape* shapeB = b2Array_Get( world->shapes, shapeIdB );

		int bodyIdA = shapeA->bodyId;
		int bodyIdB = shapeB->bodyId;

		// Are the shapes on the same body?
		if ( bodyIdA == bodyIdB )
		{
			continue;
		}

		// Sensors are handled elsewhere
		if ( shapeA->sensorIndex != B2_NULL_INDEX || shapeB->sensorIndex != B2_NULL_INDEX )
		{
			continue;
		}

		if ( b2ShouldShapesCollide( shapeA->filter, shapeB->filter ) == false )
		{
			continue;
		}

		if ( b2CanCollide( shapeA->type, shapeB->type ) == false )
		{
			// For example, no segment vs segment collision
			continue;
		}

		// Does a joint override collision?
		b2Body* bodyA = b2Array_Get( world->bodies, bodyIdA );
		b2Body* bodyB = b2Array_Get( world->bodies, bodyIdB );
		if ( b2ShouldBodiesCollide( world, bodyA, bodyB ) == false )
		{
			continue;
		}

		// Custom user filter
		if ( shapeA->enableCustomFiltering || shapeB->enableCustomFiltering )
		{
			b2CustomFilterFcn* customFilterFcn = world->customFilterFcn;
			if ( customFilterFcn != NULL )
			{
				b2ShapeId idA = { shapeIdA + 1, world->worldId, shapeA->generation };
				b2ShapeId idB = { shapeIdB + 1, world->worldId, shapeB->generation };
				bool shouldCollide = customFilterFcn( idA, idB, world->customFilterContext );
				if ( shouldCollide == false )
				{
					continue;
				}
			}
		}

		culled[count3] = culled[i];
		count3 += 1;
	}

	int base = b2AtomicFetchAddInt( &bp->movePairIndex2, count3 );

	for ( int i = 0; i < count3; ++i )
	{
		int pairIndex = base + i;
		b2MovePair* pair;
		if ( pairIndex < bp->movePairCapacity2 )
		{
			pair = bp->movePairs2 + pairIndex;
			pair->heap = false;
		}
		else
		{
			static b2AtomicInt once = { 0 };
			if ( b2AtomicCompareExchangeInt( &once, 0, 1 ) == 0 )
			{
				// This means you have too many overlapping objects.
				b2Log( "Pair buffer capacity of %d exceeded, too many overlaps", bp->movePairCapacity2 );
			}

			pair = b2Alloc( sizeof( b2MovePair ) );
			pair->heap = true;
		}

		pair->shapeIdA = culled[i].shapeIdA;
		pair->shapeIdB = culled[i].shapeIdB;

		// A batch spans items. Link by the candidate's own item so the block split
		// cannot decide which list a pair lands in.
		b2MoveResult* result = bp->moveResults2 + culled[i].item;
		pair->next = result->pairList;
		result->pairList = pair;
	}
}

B2_FORCE_INLINE void b2ReportCandidate( int shapeIdA, int shapeIdB, b2PairContext* context )
{
	// Follow shape index order.
	b2Candidate* candidate = context->batch + context->batchCount;
	candidate->shapeIdA = b2MinInt( shapeIdA, shapeIdB );
	candidate->shapeIdB = b2MaxInt( shapeIdA, shapeIdB );
	candidate->item = context->item;
	context->batchCount += 1;
	if ( context->batchCount == B2_CANDIDATE_BATCH )
	{
		b2DrainCandidates( context );
	}
}

static void b2CrossPairs( const b2TreeNode* nodesA, const b2TreeNode* nodesB, int indexA, int indexB, b2PairContext* context )
{
	b2NodePair stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = (b2NodePair){ .a = indexA, .b = indexB };

	while ( stackCount > 0 )
	{
		b2NodePair pair = stack[--stackCount];

#if B2_SNOOP_PAIR_COUNTERS
		context->pops += 1;
#endif

		const b2TreeNode* a = nodesA + pair.a;
		const b2TreeNode* b = nodesB + pair.b;

		// Any enlarged?
		if ( ( ( a->flags | b->flags ) & b2_enlargedNode ) == 0 )
		{
			continue;
		}

		if ( b2AABB_Overlaps( a->aabb, b->aabb ) == false )
		{
			continue;
		}

		bool leafA = b2IsLeaf( a );
		bool leafB = b2IsLeaf( b );

		if ( leafA && leafB )
		{
			b2ReportCandidate( (int)a->userData, (int)b->userData, context );
			continue;
		}

		// At least one node is internal. If one node is a leaf, then decend into the other
		// one. If both are internal, then decend into the larger box.
		if ( leafB || ( leafA == false && b2Perimeter( a->aabb ) > b2Perimeter( b->aabb ) ) )
		{
			stack[stackCount++] = (b2NodePair){ .a = a->children.child1, .b = pair.b };
			stack[stackCount++] = (b2NodePair){ .a = a->children.child2, .b = pair.b };
		}
		else
		{
			stack[stackCount++] = (b2NodePair){ .a = pair.a, .b = b->children.child1 };
			stack[stackCount++] = (b2NodePair){ .a = pair.a, .b = b->children.child2 };
		}
	}
}

static void b2SelfPairsTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2World* world = context;
	b2BroadPhase* bp = &world->broadPhase;
	const b2DynamicTree* tree = bp->trees + b2_dynamicBody;
	const b2TreeNode* nodes = tree->nodes;
	const int* items = bp->enlargedNodes;

	b2PairContext pairContext = { .world = world, .workerIndex = workerIndex };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		const b2TreeNode* node = nodes + items[i];
		bp->moveResults2[i].pairList = NULL;
		pairContext.item = i;
		b2CrossPairs( tree->nodes, tree->nodes, node->children.child1, node->children.child2, &pairContext );
	}

	b2DrainCandidates( &pairContext );

#if B2_SNOOP_PAIR_COUNTERS
	b2AtomicFetchAddInt( &b2_selfPops, pairContext.pops );
	b2AtomicFetchAddInt( &b2_selfCandidates, pairContext.candidates );
#endif
}

// Must be a power of 2
#define B2_CROSS_SEED_COUNT 64
_Static_assert( ( B2_CROSS_SEED_COUNT & ( B2_CROSS_SEED_COUNT - 1 ) ) == 0, "must be power of 2" );

static int b2ExpandCrossSeeds( const b2DynamicTree* treeA, const b2DynamicTree* treeB, b2NodePair* seeds )
{
	if ( treeA->root == B2_NULL_INDEX || treeB->root == B2_NULL_INDEX )
	{
		return 0;
	}

	const b2TreeNode* nodesA = treeA->nodes;
	const b2TreeNode* nodesB = treeB->nodes;

	// Bread-first search
	b2NodePair queue[2 * B2_CROSS_SEED_COUNT];
	int mask = 2 * B2_CROSS_SEED_COUNT - 1;
	int head = 0;
	int tail = 0;
	queue[tail & mask] = (b2NodePair){ .a = treeA->root, .b = treeB->root };
	tail += 1;

	int seedCount = 0;
	while ( head < tail && seedCount + ( tail - head ) < B2_CROSS_SEED_COUNT )
	{
		b2NodePair pair = queue[head & mask];
		head += 1;

		const b2TreeNode* a = nodesA + pair.a;
		const b2TreeNode* b = nodesB + pair.b;

		// Either enlarged?
		if ( ( ( a->flags | b->flags ) & b2_enlargedNode ) == 0 )
		{
			continue;
		}

		if ( b2AABB_Overlaps( a->aabb, b->aabb ) == false )
		{
			continue;
		}

		bool leafA = b2IsLeaf( a );
		bool leafB = b2IsLeaf( b );

		if ( leafA && leafB )
		{
			seeds[seedCount] = pair;
			seedCount += 1;
			continue;
		}

		// At least one node is internal. If one node is a leaf, then decend into the other
		// one. If both are internal, then decend into the larger box.
		if ( leafB || ( leafA == false && b2Perimeter( a->aabb ) > b2Perimeter( b->aabb ) ) )
		{
			queue[tail & mask] = (b2NodePair){ .a = a->children.child1, .b = pair.b };
			queue[( tail + 1 ) & mask] = (b2NodePair){ .a = a->children.child2, .b = pair.b };
		}
		else
		{
			queue[tail & mask] = (b2NodePair){ .a = pair.a, .b = b->children.child1 };
			queue[( tail + 1 ) & mask] = (b2NodePair){ .a = pair.a, .b = b->children.child2 };
		}

		tail += 2;
	}

	while ( head < tail )
	{
		seeds[seedCount] = queue[head & mask];
		seedCount += 1;
		head += 1;
	}

	return seedCount;
}

typedef struct b2CrossContext
{
	b2World* world;
	const b2NodePair* seeds;
	int staticSeedCount;
	int itemBase;
} b2CrossContext;

static void b2CrossPairsTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2CrossContext* crossContext = context;
	b2World* world = crossContext->world;
	b2BroadPhase* bp = &world->broadPhase;
	const b2TreeNode* staticNodes = bp->trees[b2_staticBody].nodes;
	const b2TreeNode* kinematicNodes = bp->trees[b2_kinematicBody].nodes;
	const b2TreeNode* dynamicNodes = bp->trees[b2_dynamicBody].nodes;

	b2PairContext pairContext = { .world = world, .workerIndex = workerIndex };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		const b2TreeNode* nodesB = i < crossContext->staticSeedCount ? staticNodes : kinematicNodes;
		b2NodePair seed = crossContext->seeds[i];
		int item = crossContext->itemBase + i;
		bp->moveResults2[item].pairList = NULL;
		pairContext.item = item;
		b2CrossPairs( dynamicNodes, nodesB, seed.a, seed.b, &pairContext );
	}

	b2DrainCandidates( &pairContext );
}

typedef struct b2QueryPairContext
{
	b2World* world;
	b2MoveResult* moveResult;
	b2BodyType queryTreeType;
	int queryProxyKey;
	int queryShapeIndex;

#if B2_SNOOP_PAIR_COUNTERS
	int candidates;
#endif
} b2QueryPairContext;

// This is called from b2DynamicTree::Query when we are gathering pairs.
static bool b2PairQueryCallback( int proxyId, uint64_t userData, void* context )
{
	int shapeId = (int)userData;

	b2QueryPairContext* queryContext = context;
	b2BroadPhase* broadPhase = &queryContext->world->broadPhase;

	int proxyKey = B2_PROXY_KEY( proxyId, queryContext->queryTreeType );
	int queryProxyKey = queryContext->queryProxyKey;

	// A proxy cannot form a pair with itself.
	if ( proxyKey == queryContext->queryProxyKey )
	{
		return true;
	}

	b2BodyType treeType = queryContext->queryTreeType;
	b2BodyType queryProxyType = B2_PROXY_TYPE( queryProxyKey );

	// De-duplication
	// It is important to prevent duplicate contacts from being created. Ideally I can prevent duplicates
	// early and in the worker. Most of the time the movedProxies bit sets contain dynamic and kinematic
	// proxies, but sometimes static proxies are in there too (b2ShapeDef::invokeContactCreation or a
	// modified static shape), so we always have to check.

	// Is this proxy also moving?
	if ( queryProxyType == b2_dynamicBody )
	{
		if ( treeType == b2_dynamicBody && proxyKey < queryProxyKey )
		{
			bool moved = b2GetBit( &broadPhase->movedProxies[treeType], proxyId );
			if ( moved )
			{
				// Both proxies are moving. Avoid duplicate pairs.
				return true;
			}
		}
	}
	else
	{
		B2_ASSERT( treeType == b2_dynamicBody );
		bool moved = b2GetBit( &broadPhase->movedProxies[treeType], proxyId );
		if ( moved )
		{
			// Both proxies are moving. Avoid duplicate pairs.
			return true;
		}
	}

#if B2_SNOOP_PAIR_COUNTERS
	queryContext->candidates += 1;
#endif

	uint64_t pairKey = B2_SHAPE_PAIR_KEY( shapeId, queryContext->queryShapeIndex );
	bool pairExists = b2ContainsKey( &broadPhase->pairSet, pairKey );
	if ( pairExists )
	{
		// contact exists
		return true;
	}

	int shapeIdA, shapeIdB;
	if ( proxyKey < queryProxyKey )
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

	b2Shape* shapeA = b2Array_Get( world->shapes, shapeIdA );
	b2Shape* shapeB = b2Array_Get( world->shapes, shapeIdB );

	int bodyIdA = shapeA->bodyId;
	int bodyIdB = shapeB->bodyId;

	// Are the shapes on the same body?
	if ( bodyIdA == bodyIdB )
	{
		return true;
	}

	// Sensors are handled elsewhere
	if ( shapeA->sensorIndex != B2_NULL_INDEX || shapeB->sensorIndex != B2_NULL_INDEX )
	{
		return true;
	}

	if ( b2ShouldShapesCollide( shapeA->filter, shapeB->filter ) == false )
	{
		return true;
	}

	if ( b2CanCollide( shapeA->type, shapeB->type ) == false )
	{
		// For example, no segment vs segment collision
		return true;
	}

	// Does a joint override collision?
	b2Body* bodyA = b2Array_Get( world->bodies, bodyIdA );
	b2Body* bodyB = b2Array_Get( world->bodies, bodyIdB );
	if ( b2ShouldBodiesCollide( world, bodyA, bodyB ) == false )
	{
		return true;
	}

	// Custom user filter
	if ( shapeA->enableCustomFiltering || shapeB->enableCustomFiltering )
	{
		b2CustomFilterFcn* customFilterFcn = queryContext->world->customFilterFcn;
		if ( customFilterFcn != NULL )
		{
			b2ShapeId idA = { shapeIdA + 1, world->worldId, shapeA->generation };
			b2ShapeId idB = { shapeIdB + 1, world->worldId, shapeB->generation };
			bool shouldCollide = customFilterFcn( idA, idB, queryContext->world->customFilterContext );
			if ( shouldCollide == false )
			{
				return true;
			}
		}
	}

	int pairIndex = b2AtomicFetchAddInt( &broadPhase->movePairIndex, 1 );

	b2MovePair* pair;
	if ( pairIndex < broadPhase->movePairCapacity )
	{
		pair = broadPhase->movePairs + pairIndex;
		pair->heap = false;
	}
	else
	{
		static b2AtomicInt once = { 0 };
		if ( b2AtomicCompareExchangeInt( &once, 0, 1 ) == 0 )
		{
			// This means you have too many overlapping objects.
			b2Log( "Pair buffer capacity of %d exceeded, too many overlaps", broadPhase->movePairCapacity );
		}

		pair = b2Alloc( sizeof( b2MovePair ) );
		pair->heap = true;
	}

	pair->shapeIdA = shapeIdA;
	pair->shapeIdB = shapeIdB;
	pair->next = queryContext->moveResult->pairList;
	queryContext->moveResult->pairList = pair;

	// continue the query
	return true;
}

// Warning: writing to these globals significantly slows multithreading performance
#if B2_SNOOP_PAIR_COUNTERS
b2TreeStats b2_dynamicStats;
b2TreeStats b2_kinematicStats;
b2TreeStats b2_staticStats;
#endif

static void b2FindPairsTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	B2_UNUSED( workerIndex );

	b2TracyCZoneNC( pair_task, "Pair", b2_colorMediumSlateBlue, true );

	b2World* world = context;
	b2BroadPhase* bp = &world->broadPhase;

	b2QueryPairContext queryContext;
	queryContext.world = world;

#if B2_SNOOP_PAIR_COUNTERS
	int nodeVisits = 0;
	int leafVisits = 0;
	int dynamicNodeVisits = 0;
	int dynamicLeafVisits = 0;
	queryContext.candidates = 0;
#endif

	for ( int i = startIndex; i < endIndex; ++i )
	{
		// Initialize move result for this moved proxy
		queryContext.moveResult = bp->moveResults + i;
		queryContext.moveResult->pairList = NULL;

		int proxyKey = bp->moveArray.data[i];
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
		queryContext.queryShapeIndex = (int)b2DynamicTree_GetUserData( baseTree, proxyId );

		// Query trees. Only dynamic proxies collide with kinematic and static proxies.
		// Using B2_DEFAULT_MASK_BITS so that b2Filter::groupIndex works.
		b2TreeStats stats = { 0 };
		if ( proxyType == b2_dynamicBody )
		{
			// consider using bits = groupIndex > 0 ? B2_DEFAULT_MASK_BITS : maskBits
			queryContext.queryTreeType = b2_kinematicBody;
			b2TreeStats statsKinematic = b2DynamicTree_Query( bp->trees + b2_kinematicBody, fatAABB, B2_DEFAULT_MASK_BITS,
															  b2PairQueryCallback, &queryContext );
			stats.nodeVisits += statsKinematic.nodeVisits;
			stats.leafVisits += statsKinematic.leafVisits;

			queryContext.queryTreeType = b2_staticBody;
			b2TreeStats statsStatic = b2DynamicTree_Query( bp->trees + b2_staticBody, fatAABB, B2_DEFAULT_MASK_BITS,
														   b2PairQueryCallback, &queryContext );
			stats.nodeVisits += statsStatic.nodeVisits;
			stats.leafVisits += statsStatic.leafVisits;
		}

		// All proxies collide with dynamic proxies
		// Using B2_DEFAULT_MASK_BITS so that b2Filter::groupIndex works.
		queryContext.queryTreeType = b2_dynamicBody;
		b2TreeStats statsDynamic =
			b2DynamicTree_Query( bp->trees + b2_dynamicBody, fatAABB, B2_DEFAULT_MASK_BITS, b2PairQueryCallback, &queryContext );
		stats.nodeVisits += statsDynamic.nodeVisits;
		stats.leafVisits += statsDynamic.leafVisits;

#if B2_SNOOP_PAIR_COUNTERS
		nodeVisits += stats.nodeVisits;
		leafVisits += stats.leafVisits;
		dynamicNodeVisits += statsDynamic.nodeVisits;
		dynamicLeafVisits += statsDynamic.leafVisits;
#endif
	}

#if B2_SNOOP_PAIR_COUNTERS
	b2AtomicFetchAddInt( &b2_queryNodeVisits, nodeVisits );
	b2AtomicFetchAddInt( &b2_queryLeafVisits, leafVisits );
	b2AtomicFetchAddInt( &b2_queryDynamicNodeVisits, dynamicNodeVisits );
	b2AtomicFetchAddInt( &b2_queryDynamicLeafVisits, dynamicLeafVisits );
	b2AtomicFetchAddInt( &b2_queryCandidates, queryContext.candidates );
#endif

	b2TracyCZoneEnd( pair_task );
}

static void b2UpdateTreesTask( void* context )
{
	b2TracyCZoneNC( tree_task, "Rebuild BVH", b2_colorFireBrick, true );

	b2World* world = context;
	b2DynamicTree_Rebuild( world->broadPhase.trees + b2_dynamicBody, false );
	b2DynamicTree_Rebuild( world->broadPhase.trees + b2_kinematicBody, false );

	b2TracyCZoneEnd( tree_task );
}

// Task that can be done in parallel with the narrow-phase
// - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
static void b2EnqueueTreeUpdate( b2World* world )
{
	if ( world->taskCount < B2_MAX_TASKS )
	{
		world->userTreeTask = world->enqueueTaskFcn( &b2UpdateTreesTask, world, world->userTaskContext );
		world->taskCount += 1;
		world->activeTaskCount += world->userTreeTask == NULL ? 0 : 1;
	}
	else
	{
		world->userTreeTask = NULL;
		b2UpdateTreesTask( world );
	}
}

#if B2_ENABLE_VALIDATION == 1
static int b2CompareKeys( const void* a, const void* b )
{
	uint64_t keyA = *(const uint64_t*)a;
	uint64_t keyB = *(const uint64_t*)b;
	return ( keyA > keyB ) - ( keyA < keyB );
}
#endif

// The tree passes must find exactly the pairs the proxy queries found with a marked leaf on
// at least one side. A proxy can be moved without a mark (created,
// teleported, collide connected changed). Only the queries find those, so they are excluded.
static void b2ValidateSelfPairs( b2World* world, int moveCount, int itemCount )
{
#if B2_ENABLE_VALIDATION == 1
	b2BroadPhase* bp = &world->broadPhase;
	b2Stack* alloc = &world->stack;

	int queryTotal = b2AtomicLoadInt( &bp->movePairIndex );
	int selfTotal = b2AtomicLoadInt( &bp->movePairIndex2 );
	uint64_t* queryKeys = b2StackAlloc( alloc, b2MaxInt( queryTotal, 1 ) * sizeof( uint64_t ), "query keys" );
	uint64_t* selfKeys = b2StackAlloc( alloc, b2MaxInt( selfTotal, 1 ) * sizeof( uint64_t ), "self keys" );

	int queryCount = 0;
	for ( int i = 0; i < moveCount; ++i )
	{
		for ( const b2MovePair* pair = bp->moveResults[i].pairList; pair != NULL; pair = pair->next )
		{
			// int proxyKeyA = b2Array_Get( world->shapes, pair->shapeIdA )->proxyKey;
			// int proxyKeyB = b2Array_Get( world->shapes, pair->shapeIdB )->proxyKey;
			// const b2TreeNode* leafA = bp->trees[B2_PROXY_TYPE( proxyKeyA )].nodes + B2_PROXY_ID( proxyKeyA );
			// const b2TreeNode* leafB = bp->trees[B2_PROXY_TYPE( proxyKeyB )].nodes + B2_PROXY_ID( proxyKeyB );

			// Moved without a mark, only the queries find these until C.3 marks on insert
			// if ( ( ( leafA->flags | leafB->flags ) & b2_enlargedNode ) == 0 )
			//{
			//	continue;
			//}

			B2_ASSERT( queryCount < queryTotal );
			queryKeys[queryCount] = B2_SHAPE_PAIR_KEY( pair->shapeIdA, pair->shapeIdB );
			queryCount += 1;
		}
	}

	int selfCount = 0;
	for ( int i = 0; i < itemCount; ++i )
	{
		for ( const b2MovePair* pair = bp->moveResults2[i].pairList; pair != NULL; pair = pair->next )
		{
			B2_ASSERT( selfCount < selfTotal );
			selfKeys[selfCount] = B2_SHAPE_PAIR_KEY( pair->shapeIdA, pair->shapeIdB );
			selfCount += 1;
		}
	}

	// Every survivor is linked exactly once
	B2_ASSERT( selfCount == selfTotal );

	qsort( queryKeys, queryCount, sizeof( uint64_t ), b2CompareKeys );
	qsort( selfKeys, selfCount, sizeof( uint64_t ), b2CompareKeys );

	// Merge walk. A pair the self pass produced twice shows up as extra.
	int i = 0;
	int j = 0;
	int missing = 0;
	int extra = 0;
	while ( i < queryCount || j < selfCount )
	{
		if ( j == selfCount || ( i < queryCount && queryKeys[i] < selfKeys[j] ) )
		{
			if ( missing < 4 )
			{
				b2Log( "self pairs: missing %d %d", (int)( queryKeys[i] >> 32 ), (int)( queryKeys[i] & 0xFFFFFFFF ) );
			}
			missing += 1;
			i += 1;
		}
		else if ( i == queryCount || selfKeys[j] < queryKeys[i] )
		{
			if ( extra < 4 )
			{
				b2Log( "self pairs: extra %d %d", (int)( selfKeys[j] >> 32 ), (int)( selfKeys[j] & 0xFFFFFFFF ) );
			}
			extra += 1;
			j += 1;
		}
		else
		{
			i += 1;
			j += 1;
		}
	}

	if ( missing + extra > 0 )
	{
		b2Log( "self pairs: %d missing, %d extra, %d from queries, %d from self pass", missing, extra, queryCount, selfCount );
		fflush( stdout );
	}

	B2_ASSERT( missing == 0 && extra == 0 );

	b2StackFree( alloc, selfKeys );
	b2StackFree( alloc, queryKeys );
#else
	B2_UNUSED( world );
	B2_UNUSED( moveCount );
	B2_UNUSED( itemCount );
#endif
}

// Generate pairs by querying the dynamic body tree against itself and against
// the kinematic and static trees.
static void b2SelfPass( b2World* world, int moveCount, int minRange )
{
	b2BroadPhase* bp = &world->broadPhase;
	b2Stack* alloc = &world->stack;

	const b2DynamicTree* dynamicTree = bp->trees + b2_dynamicBody;
	int nodeCount = dynamicTree->nodeCount;
	bp->enlargedNodes = b2StackAlloc( alloc, nodeCount * sizeof( int ), "enlarged nodes" );
	int enlargedCount = b2GatherEnlargedNodes( dynamicTree, bp->enlargedNodes );

	// todo need a better capacity heuristic
	bp->movePairCapacity2 = b2MaxInt( 16 * enlargedCount, bp->movePairCapacity2 );
	bp->movePairs2 = b2StackAlloc( alloc, bp->movePairCapacity2 * sizeof( b2MovePair ), "mp2" );

	b2NodePair seeds[2 * B2_CROSS_SEED_COUNT];
	int staticSeedCount = b2ExpandCrossSeeds( dynamicTree, bp->trees + b2_staticBody, seeds );
	B2_ASSERT( staticSeedCount <= B2_CROSS_SEED_COUNT );
	int kinematicSeedCount = b2ExpandCrossSeeds( dynamicTree, bp->trees + b2_kinematicBody, seeds + staticSeedCount );
	B2_ASSERT( kinematicSeedCount <= B2_CROSS_SEED_COUNT );
	int crossCount = staticSeedCount + kinematicSeedCount;
	int itemCount = enlargedCount + crossCount;

	bp->moveResults2 = b2StackAlloc( alloc, itemCount * sizeof( b2MoveResult ), "move results" );
	bp->moveCount2 = itemCount;

	b2AtomicStoreInt( &bp->movePairIndex2, 0 );

	b2CrossContext crossContext = {
		.world = world,
		.seeds = seeds,
		.staticSeedCount = staticSeedCount,
		.itemBase = enlargedCount,
	};
	b2ParallelFor( world, &b2CrossPairsTask, crossCount, 1, &crossContext );
	b2ParallelFor( world, &b2SelfPairsTask, enlargedCount, minRange, world );

	b2ValidateSelfPairs( world, moveCount, itemCount );

#if B2_SNOOP_PAIR_COUNTERS
	int querySurvivors = b2AtomicLoadInt( &bp->movePairIndex );
	int selfSurvivors = b2AtomicLoadInt( &bp->movePairIndex2 );
	b2AtomicFetchAddInt( &b2_querySurvivors, querySurvivors );
	b2AtomicFetchAddInt( &b2_selfSurvivors, selfSurvivors );
	b2AtomicFetchAddInt( &b2_pairSteps, 1 );
	b2_queryHeapPairs += b2MaxInt( querySurvivors - bp->movePairCapacity, 0 );
	b2_selfHeapPairs += b2MaxInt( selfSurvivors - bp->movePairCapacity2, 0 );
	b2_queryMaxSurvivors = b2MaxInt( b2_queryMaxSurvivors, querySurvivors );
	b2_selfMaxSurvivors = b2MaxInt( b2_selfMaxSurvivors, selfSurvivors );
	b2_selfMaxCapacity = b2MaxInt( b2_selfMaxCapacity, bp->movePairCapacity2 );
#endif

	for ( int i = 0; i < itemCount; ++i )
	{
		b2MovePair* pair = bp->moveResults2[i].pairList;
		while ( pair != NULL )
		{
			b2MovePair* next = pair->next;
			if ( pair->heap )
			{
				b2Free( pair, sizeof( b2MovePair ) );
			}

			pair = next;
		}
	}
}

void b2UpdateBroadPhasePairs( b2World* world )
{
	b2BroadPhase* bp = &world->broadPhase;

	b2ValidateMovedProxies( bp );

	int moveCount = bp->moveArray.count;

	if ( moveCount == 0 )
	{
		// A destroyed shape may lead to no moves, but the tree could still be enlarged.
		b2DynamicTree_ClearEnlarged( bp->trees + b2_staticBody );
		b2EnqueueTreeUpdate( world );
		return;
	}

	b2TracyCZoneNC( update_pairs, "Find Pairs", b2_colorMediumSlateBlue, true );

	b2Stack* alloc = &world->stack;

	// todo these could be in the step context
	bp->moveResults = b2StackAlloc( alloc, moveCount * sizeof( b2MoveResult ), "move results" );

	// This capacity can be exceeded if there are many overlapping pairs (e.g. all shapes at the origin)
	// todo if this remains then it should hit a high water mark and account for heap allocated pairs
	bp->movePairCapacity = 32 * moveCount;
	bp->movePairs = b2StackAlloc( alloc, bp->movePairCapacity * sizeof( b2MovePair ), "move pairs" );
	b2AtomicStoreInt( &bp->movePairIndex, 0 );

#if B2_SNOOP_TABLE_COUNTERS
	extern b2AtomicInt b2_probeCount;
	b2AtomicStoreInt( &b2_probeCount, 0 );
#endif

	int minRange = 64;

#if B2_SNOOP_PAIR_TIMING
	// The validate compare reads the query results, so the order only alternates without it
	bool selfFirst = B2_ENABLE_VALIDATION == 0 && ( b2_timingSteps & 1 ) == 1;
	uint64_t ticks = b2GetTicks();
	if ( selfFirst )
	{
		b2SelfPass( world, moveCount, minRange );
		b2_selfFirstMs += b2GetMillisecondsAndReset( &ticks );
		b2_selfFirstCount += 1;
		b2ParallelFor( world, &b2FindPairsTask, moveCount, minRange, world );
		b2_querySecondMs += b2GetMillisecondsAndReset( &ticks );
	}
	else
	{
		b2ParallelFor( world, &b2FindPairsTask, moveCount, minRange, world );
		b2_queryFirstMs += b2GetMillisecondsAndReset( &ticks );
		b2_queryFirstCount += 1;
		b2SelfPass( world, moveCount, minRange );
		b2_selfSecondMs += b2GetMillisecondsAndReset( &ticks );
	}
	b2_timingSteps += 1;
#else
	b2ParallelFor( world, &b2FindPairsTask, moveCount, minRange, world );
	b2SelfPass( world, moveCount, minRange );
#endif

	b2DynamicTree_ClearEnlarged( bp->trees + b2_staticBody );

	b2TracyCZoneNC( create_contacts, "Create Contacts", b2_colorCoral, true );

	// Update stale trees.
	b2EnqueueTreeUpdate( world );

	// Single-threaded work
	// - Clear move flags
	// - Create contacts in deterministic order
	// This is deterministic because the results follow the order of b2BroadPhase::moveArray.
	int count = bp->moveCount2;
	for ( int i = 0; i < count; ++i )
	{
		b2MoveResult* result = bp->moveResults2 + i;
		b2MovePair* pair = result->pairList;
		while ( pair != NULL )
		{
			int shapeIdA = pair->shapeIdA;
			int shapeIdB = pair->shapeIdB;

			// if (s_file != NULL)
			//{
			//	fprintf(s_file, "%d %d\n", shapeIdA, shapeIdB);
			// }

			b2Shape* shapeA = b2Array_Get( world->shapes, shapeIdA );
			b2Shape* shapeB = b2Array_Get( world->shapes, shapeIdB );

			b2CreateContact( world, shapeA, shapeB );

			if ( pair->heap )
			{
				// Note: I tried adding to the pair set in parallel with contact creation
				// but that didn't work with with pair heap allocation. I could make it
				// work with a task context bump allocator with heap fallback. The perf
				// gain was small or zero.
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

	// Reset move buffer: clear only the bits that were set this step.
	// Invariant: bit set in movedProxies[type] iff proxyKey is present in moveArray.
	for ( int i = 0; i < bp->moveArray.count; ++i )
	{
		int proxyKey = bp->moveArray.data[i];
		b2ClearBit( &bp->movedProxies[B2_PROXY_TYPE( proxyKey )], B2_PROXY_ID( proxyKey ) );
	}
	b2Array_Clear( bp->moveArray );
	
	b2StackFree( alloc, bp->moveResults2 );
	bp->moveResults2 = NULL;
	b2StackFree( alloc, bp->movePairs2 );
	bp->movePairs2 = NULL;
	b2StackFree( alloc, bp->enlargedNodes );
	bp->enlargedNodes = NULL;

	b2StackFree( alloc, bp->movePairs );
	bp->movePairs = NULL;
	b2StackFree( alloc, bp->moveResults );
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

int b2BroadPhase_GetShapeIndex( b2BroadPhase* bp, int proxyKey )
{
	int typeIndex = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	return (int)b2DynamicTree_GetUserData( bp->trees + typeIndex, proxyId );
}

void b2ValidateBroadphase( const b2BroadPhase* bp )
{
	b2DynamicTree_Validate( bp->trees + b2_dynamicBody );
	b2DynamicTree_Validate( bp->trees + b2_kinematicBody );

	// TODO_ERIN validate every shape AABB is contained in tree AABB
}

void b2ValidateNoEnlarged( const b2BroadPhase* bp )
{
#if B2_ENABLE_VALIDATION == 1
	for ( int j = 0; j < b2_bodyTypeCount; ++j )
	{
		const b2DynamicTree* tree = bp->trees + j;
		b2DynamicTree_ValidateNoEnlarged( tree );
	}
#else
	B2_UNUSED( bp );
#endif
}

void b2ValidateMovedProxies( const b2BroadPhase* bp )
{
#if B2_ENABLE_VALIDATION == 1
	// Invariant: bit set in movedProxies[type] iff proxyKey is present in moveArray.
	int moveCount = bp->moveArray.count;
	for ( int i = 0; i < moveCount; ++i )
	{
		int proxyKey = bp->moveArray.data[i];
		b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
		int proxyId = B2_PROXY_ID( proxyKey );
		B2_ASSERT( b2GetBit( &bp->movedProxies[proxyType], proxyId ) );
	}

	int totalSetBits = 0;
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		totalSetBits += b2CountSetBits( (b2BitSet*)&bp->movedProxies[i] );
	}
	B2_ASSERT( totalSetBits == moveCount );
#else
	B2_UNUSED( bp );
#endif
}
