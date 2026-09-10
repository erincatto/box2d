// SPDX-FileCopyrightText: 2026 Erin Catto
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
#include "qsort.h"
#include "shape.h"

#include <stdbool.h>
#include <string.h>

// #include <stdio.h>

// static FILE* s_file = NULL;

void b2CreateBroadPhase( b2BroadPhase* bp, const b2Capacity* capacity )
{
	_Static_assert( b2_bodyTypeCount == 3, "must be three body types" );

	// if (s_file == NULL)
	//{
	//	s_file = fopen("pairs01.txt", "a");
	//	fprintf(s_file, "============\n\n");
	// }

	bp->pairSet = b2CreateSet( b2MaxInt( 32, 2 * capacity->contactCount ) );

	int staticCapacity = b2MaxInt( 16, capacity->staticShapeCount );
	bp->trees[b2_staticBody] = b2DynamicTree_Create( staticCapacity );

	int kinematicCapacity = 16;
	bp->trees[b2_kinematicBody] = b2DynamicTree_Create( kinematicCapacity );

	int dynamicCapacity = b2MaxInt( 16, capacity->dynamicShapeCount );
	bp->trees[b2_dynamicBody] = b2DynamicTree_Create( dynamicCapacity );

	bp->movePairs = NULL;
	b2AtomicStoreInt( &bp->movePairIndex, 0 );
	bp->movePairCapacity = 16;
	bp->moveCount = 0;
}

void b2DestroyBroadPhase( b2BroadPhase* bp )
{
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_Destroy( bp->trees + i );
	}

	b2DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b2BroadPhase ) );
}

int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );

	bool mark = ( proxyType != b2_staticBody || forcePairCreation );

	int proxyId = b2DynamicTree_CreateProxy( bp->trees + proxyType, aabb, categoryBits, shapeIndex, mark );
	int proxyKey = B2_PROXY_KEY( proxyId, proxyType );
	return proxyKey;
}

void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( 0 <= proxyType && proxyType <= b2_bodyTypeCount );
	b2DynamicTree_DestroyProxy( bp->trees + proxyType, proxyId );
}

void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	bool mark = true;
	b2DynamicTree_MoveProxy( bp->trees + proxyType, proxyId, aabb, mark );
}

// Gather internal nodes that have moved. This is done serially but it is cache friendly and fast.
static int b2GatherMovedInternalNodes( const b2DynamicTree* tree, int* nodeIndices )
{
	const b2TreeNode* nodes = tree->nodes;
	const b2TreeLink* links = tree->links;
	int capacity = tree->nodeCapacity;

	int count = 0;
	for ( int i = 0; i < capacity; ++i )
	{
		if ( b2IsAllocated( links + i ) == false )
		{
			continue;
		}

		if ( ( nodes[i].children[0].flagIndex | nodes[i].children[1].flagIndex ) & B2_MOVED_NODE )
		{
			nodeIndices[count++] = i;
		}
	}

	return count;
}

#define B2_CANDIDATE_BATCH 32

typedef struct b2Candidate
{
	int shapeIdA;
	int shapeIdB;
	int moveIndex;
} b2CandidatePair;

typedef struct b2PairContext
{
	b2World* world;
	b2CandidatePair batch[B2_CANDIDATE_BATCH];
	int batchCount;
	int workerIndex;
	int moveIndex;
} b2PairContext;

typedef struct b2RecordPair
{
	b2TreeChild a;
	b2TreeChild b;
} b2RecordPair;

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
static void b2FlushPairs( b2PairContext* context )
{
	b2World* world = context->world;
	b2BroadPhase* bp = &world->broadPhase;

	int count1 = context->batchCount;
	context->batchCount = 0;

	// Prefetch hash set entries.
	uint64_t keys[B2_CANDIDATE_BATCH];
	uint64_t hashes[B2_CANDIDATE_BATCH];
	for ( int i = 0; i < count1; ++i )
	{
		b2CandidatePair* candidate = context->batch + i;
		keys[i] = B2_SHAPE_PAIR_KEY( candidate->shapeIdA, candidate->shapeIdB );
		hashes[i] = b2KeyHash( keys[i] );
		b2PrefetchHash( &bp->pairSet, hashes[i] );
	}

	// Cull existing pairs.
	b2CandidatePair candidates[B2_CANDIDATE_BATCH];
	int count2 = 0;
	for ( int i = 0; i < count1; ++i )
	{
		bool pairExists = b2ContainsHashedKey( &bp->pairSet, keys[i], hashes[i] );
		if ( pairExists == false )
		{
			candidates[count2] = context->batch[i];
			count2 += 1;
		}
	}

	// Prefetch shapes.
	const b2Shape* shapes = world->shapes.data;
	for ( int i = 0; i < count2; ++i )
	{
		b2Prefetch( shapes + candidates[i].shapeIdA );
		b2Prefetch( shapes + candidates[i].shapeIdB );
	}

	// Filter candidates.
	int count3 = 0;
	for ( int i = 0; i < count2; ++i )
	{
		int shapeIdA = candidates[i].shapeIdA;
		int shapeIdB = candidates[i].shapeIdB;

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

		candidates[count3] = candidates[i];
		count3 += 1;
	}

	// Claim results space.
	int base = b2AtomicFetchAddInt( &bp->movePairIndex, count3 );

	for ( int i = 0; i < count3; ++i )
	{
		int pairIndex = base + i;
		b2MovePair* pair;
		if ( pairIndex < bp->movePairCapacity )
		{
			pair = bp->movePairs + pairIndex;
			pair->heap = false;
		}
		else
		{
			static b2AtomicInt once = { 0 };
			if ( b2AtomicCompareExchangeInt( &once, 0, 1 ) == 0 )
			{
				// This means you have too many overlapping objects.
				b2Log( "Pair buffer capacity of %d exceeded, too many overlaps", bp->movePairCapacity );
			}

			pair = b2Alloc( sizeof( b2MovePair ) );
			pair->heap = true;
		}

		pair->shapeIdA = candidates[i].shapeIdA;
		pair->shapeIdB = candidates[i].shapeIdB;

		// Append to linked list.
		b2MoveResult* result = bp->moveResults + candidates[i].moveIndex;
		pair->next = result->pairList;
		result->pairList = pair;
	}
}

B2_FORCE_INLINE void b2AddPair( int shapeIdA, int shapeIdB, b2PairContext* context )
{
	// Follow shape index order.
	b2CandidatePair* candidate = context->batch + context->batchCount;
	candidate->shapeIdA = b2MinInt( shapeIdA, shapeIdB );
	candidate->shapeIdB = b2MaxInt( shapeIdA, shapeIdB );
	candidate->moveIndex = context->moveIndex;
	context->batchCount += 1;
	if ( context->batchCount == B2_CANDIDATE_BATCH )
	{
		b2FlushPairs( context );
	}
}

// Did either move and if so do they overlap?
B2_FORCE_INLINE bool b2RecordPairSurvives( const b2TreeChild* a, const b2TreeChild* b )
{
	if ( ( ( a->flagIndex | b->flagIndex ) & B2_MOVED_NODE ) == 0 )
	{
		return false;
	}

	return b2AABB_Overlaps( a->aabb, b->aabb );
}

static void b2CollideProxyAndSubtree( const b2TreeChild* proxy, const b2TreeNode* nodes, int nodeIndex, b2PairContext* context )
{
	uint32_t proxyMark = proxy->flagIndex & B2_MOVED_NODE;
	__m128 boxv = _mm_loadu_ps(&proxy->aabb.lowerBound.x);
	int shapeId = (int)proxy->truncatedUserData;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = nodeIndex;

	while ( stackCount > 0 )
	{
		const b2TreeNode* node = nodes + stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeChild* child = node->children + i;
			if ( ( ( child->flagIndex | proxyMark ) & B2_MOVED_NODE ) == 0 )
			{
				continue;
			}

			if ( b2OverlapChild( boxv, child ) == false )
			{
				continue;
			}

			if ( b2IsLeaf( child ) )
			{
				b2AddPair( shapeId, (int)child->truncatedUserData, context );
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				stack[stackCount++] = b2GetChildIndex( child );
			}
		}
	}
}

B2_FORCE_INLINE void b2VisitRecordPair( const b2TreeNode* nodesA, const b2TreeNode* nodesB, const b2TreeChild* a,
										const b2TreeChild* b, b2NodePair* stack, int* stackCount, b2PairContext* context )
{
	if ( b2RecordPairSurvives( a, b ) == false )
	{
		return;
	}

	bool leafA = b2IsLeaf( a );
	bool leafB = b2IsLeaf( b );
	if ( leafA && leafB )
	{
		b2AddPair( (int)a->truncatedUserData, (int)b->truncatedUserData, context );
	}
	else if ( leafA )
	{
		b2CollideProxyAndSubtree( a, nodesB, b2GetChildIndex( b ), context );
	}
	else if ( leafB )
	{
		b2CollideProxyAndSubtree( b, nodesA, b2GetChildIndex( a ), context );
	}
	else
	{
		B2_ASSERT( *stackCount < B2_TREE_STACK_SIZE );
		stack[*stackCount] = (b2NodePair){ .a = b2GetChildIndex( a ), .b = b2GetChildIndex( b ) };
		*stackCount += 1;
	}
}

// This collides two sub-trees against each other. They can live in the same dynamic tree.
// This can only generate pairs cross sub-tree, but not within a sub-tree. This fact means
// this does not generate duplicate pairs.
// For example consider the full binary tree A (B (D  E) C (F G))
// Colliding children of A (B and C) can give pairs (D,F) (D,G) (E,F) and (E,G).
// Then colliding children of B can give the pair (D,E) and for C (F,G).
// So no duplicates even when used for self-collision.
// See Real-time collision detection section 6.3.2.
static void b2CollideCrossPairs( const b2TreeNode* nodesA, const b2TreeNode* nodesB, const b2TreeChild* a, const b2TreeChild* b,
								 b2PairContext* context )
{
	b2NodePair stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;

	b2VisitRecordPair( nodesA, nodesB, a, b, stack, &stackCount, context );

	while ( stackCount > 0 )
	{
		b2NodePair pair = stack[--stackCount];
		const b2TreeNode* nodeA = nodesA + pair.a;
		const b2TreeNode* nodeB = nodesB + pair.b;

		for ( int i = 0; i < 2; ++i )
		{
			for ( int j = 0; j < 2; ++j )
			{
				b2VisitRecordPair( nodesA, nodesB, nodeA->children + i, nodeB->children + j, stack, &stackCount, context );
			}
		}
	}
}

// This takes moved internal nodes and collides their sub-trees against each other.
static void b2SelfPairsTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2TracyCZoneNC( self_pairs, "Self", b2_colorCoral, true );

	b2World* world = context;
	b2BroadPhase* bp = &world->broadPhase;
	const b2DynamicTree* tree = bp->trees + b2_dynamicBody;
	const b2TreeNode* nodes = tree->nodes;
	const int* nodeIndices = bp->movedNodes;

	b2PairContext pairContext = { .world = world, .workerIndex = workerIndex };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		const b2TreeNode* node = nodes + nodeIndices[i];
		bp->moveResults[i].pairList = NULL;
		pairContext.moveIndex = i;

		b2CollideCrossPairs( tree->nodes, tree->nodes, node->children + 0, node->children + 1, &pairContext );
	}

	b2FlushPairs( &pairContext );

	b2TracyCZoneEnd( self_pairs );
}

#define B2_CROSS_SEED_COUNT 64
_Static_assert( ( B2_CROSS_SEED_COUNT & ( B2_CROSS_SEED_COUNT - 1 ) ) == 0, "must be power of 2" );

// This does a serial cross-tree breadth first search until the queue is full. Then returns
// the queue pairs as seeds for a parallel search.
static int b2GatherCrossSeeds( const b2DynamicTree* treeA, const b2DynamicTree* treeB, b2RecordPair* seeds )
{
	const b2TreeNode* nodesA = treeA->nodes;
	const b2TreeNode* nodesB = treeB->nodes;

	const b2TreeNode* rootA = nodesA + B2_ROOT_NODE;
	const b2TreeNode* rootB = nodesB + B2_ROOT_NODE;

	// Bread-first search
	b2RecordPair queue[2 * B2_CROSS_SEED_COUNT];
	int mask = 2 * B2_CROSS_SEED_COUNT - 1;
	int head = 0;
	int tail = 0;

	for ( int i = 0; i < 2; ++i )
	{
		const b2TreeChild* a = rootA->children + i;
		if ( a->flagIndex != B2_NODE_SENTINEL )
		{
			for ( int j = 0; j < 2; ++j )
			{
				const b2TreeChild* b = rootB->children + j;
				if ( b->flagIndex != B2_NODE_SENTINEL )
				{
					if ( b2RecordPairSurvives( a, b ) )
					{
						queue[tail & mask] = (b2RecordPair){ .a = *a, .b = *b };
						tail += 1;
					}
				}
			}
		}
	}

	int seedCount = 0;
	while ( head < tail && seedCount + ( tail - head ) + 3 < B2_CROSS_SEED_COUNT )
	{
		b2RecordPair pair = queue[head & mask];
		head += 1;

		if ( b2IsLeaf( &pair.a ) || b2IsLeaf( &pair.b ) )
		{
			seeds[seedCount++] = pair;
			continue;
		}

		const b2TreeNode* a = nodesA + b2GetChildIndex( &pair.a );
		const b2TreeNode* b = nodesB + b2GetChildIndex( &pair.b );

		for ( int i = 0; i < 2; ++i )
		{
			for ( int j = 0; j < 2; ++j )
			{
				if ( b2RecordPairSurvives( a->children + i, b->children + j ) )
				{
					queue[tail & mask] = (b2RecordPair){ a->children[i], b->children[j] };
					tail += 1;
				}
			}
		}
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
	const b2RecordPair* seeds;
	int staticSeedCount;
	int itemBase;
} b2CrossContext;

static void b2CrossPairsTask( int startIndex, int endIndex, int workerIndex, void* context )
{
	b2TracyCZoneNC( cross_pairs, "Cross", b2_colorCoral, true );

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
		int item = crossContext->itemBase + i;
		bp->moveResults[item].pairList = NULL;
		pairContext.moveIndex = item;
		b2RecordPair seed = crossContext->seeds[i];
		b2CollideCrossPairs( dynamicNodes, nodesB, &seed.a, &seed.b, &pairContext );
	}

	b2FlushPairs( &pairContext );

	b2TracyCZoneEnd( cross_pairs );
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

void b2UpdateBroadPhasePairs( b2World* world )
{
	b2BroadPhase* bp = &world->broadPhase;

	bool moved = b2HasTreeMoved( bp->trees + b2_staticBody );
	moved = moved || b2HasTreeMoved( bp->trees + b2_kinematicBody );
	moved = moved || b2HasTreeMoved( bp->trees + b2_dynamicBody );

	if ( moved == false )
	{
		// A destroyed shape may lead to no moves, but the tree could still be moved
		// todo I think this is no longer true since the move array is gone
		b2DynamicTree_ClearEnlarged( bp->trees + b2_staticBody );
		b2EnqueueTreeUpdate( world );
		return;
	}

	b2TracyCZoneNC( update_pairs, "Find Pairs", b2_colorMediumSlateBlue, true );

	b2Stack* alloc = &world->stack;

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the internal nodes of the dynamic body tree that have moved.
		const b2DynamicTree* dynamicTree = bp->trees + b2_dynamicBody;
		int nodeCount = dynamicTree->nodeCount;
		bp->movedNodes = b2StackAlloc( alloc, nodeCount * sizeof( int ), "moved nodes" );
		int dynamicMoveCount = b2GatherMovedInternalNodes( dynamicTree, bp->movedNodes );

		// Get seeds for colliding against the static and kinematic trees.
		b2RecordPair crossSeeds[2 * B2_CROSS_SEED_COUNT];
		int staticSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_staticBody, crossSeeds );
		B2_ASSERT( staticSeedCount <= B2_CROSS_SEED_COUNT );
		int kinematicSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_kinematicBody, crossSeeds + staticSeedCount );
		B2_ASSERT( kinematicSeedCount <= B2_CROSS_SEED_COUNT );
		int crossMoveCount = staticSeedCount + kinematicSeedCount;
		int totalMovedCount = dynamicMoveCount + crossMoveCount;

		// todo need a better capacity heuristic
		bp->movePairCapacity = b2MaxInt( 16 * totalMovedCount, bp->movePairCapacity );
		bp->movePairs = b2StackAlloc( alloc, bp->movePairCapacity * sizeof( b2MovePair ), "move pairs" );
		bp->moveResults = b2StackAlloc( alloc, totalMovedCount * sizeof( b2MoveResult ), "move results" );
		bp->moveCount = totalMovedCount;

		b2AtomicStoreInt( &bp->movePairIndex, 0 );

		// Collide the dynamic body tree against the static and kinematic trees.
		b2CrossContext crossContext = {
			.world = world,
			.seeds = crossSeeds,
			.staticSeedCount = staticSeedCount,
			.itemBase = dynamicMoveCount,
		};
		b2ParallelFor( world, &b2CrossPairsTask, crossMoveCount, 1, &crossContext );

		// Collide the dynamic body tree against itself.
		b2ParallelFor( world, &b2SelfPairsTask, dynamicMoveCount, 64, world );
	}

	b2DynamicTree_ClearEnlarged( bp->trees + b2_staticBody );

	b2TracyCZoneEnd( update_pairs );

	b2TracyCZoneNC( create_contacts, "Create Contacts", b2_colorCoral, true );

	// Update stale trees.
	b2EnqueueTreeUpdate( world );

	// Pairs arrive in deterministic order but scrambled relative to body and shape order
	// sorting them here improves solver performance.
	int itemCount = bp->moveCount;
	int pairCount = b2AtomicLoadInt( &bp->movePairIndex );
	uint64_t* pairKeys = b2StackAlloc( alloc, b2MaxInt( pairCount, 1 ) * sizeof( uint64_t ), "pair keys" );
	int keyCount = 0;
	for ( int i = 0; i < itemCount; ++i )
	{
		b2MovePair* pair = bp->moveResults[i].pairList;
		while ( pair != NULL )
		{
			pairKeys[keyCount] = B2_SHAPE_PAIR_KEY( pair->shapeIdA, pair->shapeIdB );
			keyCount += 1;

			b2MovePair* next = pair->next;
			if ( pair->heap )
			{
				b2Free( pair, sizeof( b2MovePair ) );
			}

			pair = next;
		}
	}

	B2_ASSERT( keyCount == pairCount );

	{
#define LESS( i, j ) ( pairKeys[(int)( i )] < pairKeys[(int)( j )] )
#define SWAP( i, j )                                                                                                             \
	do                                                                                                                           \
	{                                                                                                                            \
		uint64_t tmp_ = pairKeys[(int)( i )];                                                                                    \
		pairKeys[(int)( i )] = pairKeys[(int)( j )];                                                                             \
		pairKeys[(int)( j )] = tmp_;                                                                                             \
	}                                                                                                                            \
	while ( 0 )

		QSORT( pairCount, LESS, SWAP );

#undef LESS
#undef SWAP
	}

	for ( int i = 0; i < keyCount; ++i )
	{
		int shapeIdA = (int)( pairKeys[i] >> 32 );
		int shapeIdB = (int)( pairKeys[i] & 0xFFFFFFFF );
		b2Shape* shapeA = b2Array_Get( world->shapes, shapeIdA );
		b2Shape* shapeB = b2Array_Get( world->shapes, shapeIdB );
		b2CreateContact( world, shapeA, shapeB );
	}

	b2StackFree( alloc, pairKeys );

	b2StackFree( alloc, bp->moveResults );
	bp->moveResults = NULL;
	b2StackFree( alloc, bp->movePairs );
	bp->movePairs = NULL;
	b2StackFree( alloc, bp->movedNodes );
	bp->movedNodes = NULL;

	b2ValidateSolverSets( world );

	b2TracyCZoneEnd( create_contacts );
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
