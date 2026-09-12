// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "broad_phase.h"

#include "aabb.h"
#include "arena_allocator.h"
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

	bp->movedSiblings = NULL;
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

// Gather the sibling pairs with a moved node. This is done serially it is cache friendly.
static int b2GatherMovedSiblings( const b2DynamicTree* tree, int* pairIndices )
{
	const b2TreeNode* nodes = tree->nodes;
	int nodeEnd = tree->nodeEnd;

	int count = 0;

	// Skip the root.
	for ( int pair = 2; pair < nodeEnd; pair += 2 )
	{
		// Push when either sibling moved.
		if ( ( nodes[pair].flagIndex | nodes[pair + 1].flagIndex ) & B2_MOVED_NODE )
		{
			pairIndices[count++] = pair;
		}
	}

	return count;
}

#define B2_CANDIDATE_BATCH 32

typedef struct b2Candidate
{
	int shapeIdA;
	int shapeIdB;
} b2CandidatePair;

typedef struct b2PairContext
{
	b2World* world;
	b2Array( uint64_t ) * pairKeys;
	b2CandidatePair batch[B2_CANDIDATE_BATCH];
	int batchCount;
} b2PairContext;

typedef struct b2NodePair
{
	b2TreeNode a;
	b2TreeNode b;
} b2NodePair;

typedef struct b2IndexPair
{
	int a, b;
} b2IndexPair;

static void b2FlushCandidatePairs( b2PairContext* context )
{
	b2World* world = context->world;
	b2BroadPhase* bp = &world->broadPhase;

	int count1 = context->batchCount;
	context->batchCount = 0;

	// Prefetch hash set entries. Less than 1% gain from this but it might
	// matter more for Box3D. Also, if I switch to verstable then this might
	// not be possible.
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

	// Prefetch shapes. Again, less than a 1% gain from this. Might matter more for Box3D.
	const b2Shape* shapes = world->shapes.data;
	for ( int i = 0; i < count2; ++i )
	{
		b2Prefetch( shapes + candidates[i].shapeIdA );
		b2Prefetch( shapes + candidates[i].shapeIdB );
	}

	// Filter candidates.
	b2Array( uint64_t )* pairKeys = context->pairKeys;
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

		// The pair passed the gauntlet. A new contact will be created.
		b2Array_Push( *pairKeys, B2_SHAPE_PAIR_KEY( shapeIdA, shapeIdB ) );
	}
}

B2_FORCE_INLINE void b2AddCandidatePair( int shapeIdA, int shapeIdB, b2PairContext* context )
{
	// Follow shape index order.
	b2CandidatePair* candidate = context->batch + context->batchCount;
	candidate->shapeIdA = b2MinInt( shapeIdA, shapeIdB );
	candidate->shapeIdB = b2MaxInt( shapeIdA, shapeIdB );
	context->batchCount += 1;
	if ( context->batchCount == B2_CANDIDATE_BATCH )
	{
		b2FlushCandidatePairs( context );
	}
}

// Did either move and if so do they overlap?
B2_FORCE_INLINE bool b2TestPair( const b2TreeNode* a, const b2TreeNode* b )
{
	if ( ( ( a->flagIndex | b->flagIndex ) & B2_MOVED_NODE ) == 0 )
	{
		return false;
	}

	return b2AABB_Overlaps( a->aabb, b->aabb );
}

static void b2CollideProxyAndSubtree( const b2TreeNode* proxy, const b2TreeNode* nodes, int pair, b2PairContext* context )
{
	uint32_t proxyMark = proxy->flagIndex & B2_MOVED_NODE;
	b2AABBV boxv = b2LoadAABBV( &proxy->aabb );
	int shapeId = proxy->shapeIndex;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = pair;

	while ( stackCount > 0 )
	{
		pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( ( ( node->flagIndex | proxyMark ) & B2_MOVED_NODE ) == 0 )
			{
				continue;
			}

			if ( b2OverlapNode( boxv, node ) == false )
			{
				continue;
			}

			if ( b2IsLeaf( node ) )
			{
				b2AddCandidatePair( shapeId, node->shapeIndex, context );
			}
			else
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				stack[stackCount++] = b2GetLeftChild( node );
			}
		}
	}
}

// Helper for b2CollideCrossPairs to avoid code duplication.
B2_FORCE_INLINE void b2VisitPair( const b2TreeNode* arrayA, const b2TreeNode* arrayB, const b2TreeNode* nodeA,
								  const b2TreeNode* nodeB, b2IndexPair* stack, int* stackCount, b2PairContext* context )
{
	if ( b2TestPair( nodeA, nodeB ) == false )
	{
		return;
	}

	bool leafA = b2IsLeaf( nodeA );
	bool leafB = b2IsLeaf( nodeB );
	if ( leafA && leafB )
	{
		b2AddCandidatePair( nodeA->shapeIndex, nodeB->shapeIndex, context );
	}
	else if ( leafA )
	{
		b2CollideProxyAndSubtree( nodeA, arrayB, b2GetLeftChild( nodeB ), context );
	}
	else if ( leafB )
	{
		b2CollideProxyAndSubtree( nodeB, arrayA, b2GetLeftChild( nodeA ), context );
	}
	else
	{
		B2_ASSERT( *stackCount < B2_TREE_STACK_SIZE );
		stack[*stackCount] = (b2IndexPair){ .a = b2GetLeftChild( nodeA ), .b = b2GetLeftChild( nodeB ) };
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
// When ever a proxy is moved, the flag is propagated up the hierachy to the root. So
// self collision gathers all those moved internal nodes and collides their subtrees together.
// See Real-time collision detection section 6.3.2. This is faster than querying every moved
// proxy against the whole tree. Scaling is linear instead of linear * log.
// Many other physics engines do this (Bepu, Rapier, etc). So nothing new here.
static void b2CollideCrossPairs( const b2TreeNode* arrayA, const b2TreeNode* arrayB, const b2TreeNode* subtreeA,
								 const b2TreeNode* subtreeB, b2PairContext* context )
{
	b2IndexPair stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;

	// Seed the stack.
	b2VisitPair( arrayA, arrayB, subtreeA, subtreeB, stack, &stackCount, context );

	while ( stackCount > 0 )
	{
		b2IndexPair pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			for ( int j = 0; j < 2; ++j )
			{
				b2VisitPair( arrayA, arrayB, arrayA + pair.a + i, arrayB + pair.b + j, stack, &stackCount, context );
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
	const int* siblingIndices = bp->movedSiblings;

	b2PairContext pairContext = { .world = world, .pairKeys = &world->taskContexts.data[workerIndex].pairKeys };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		int nodeIndex = siblingIndices[i];
		b2CollideCrossPairs( nodes, nodes, nodes + nodeIndex, nodes + nodeIndex + 1, &pairContext );
	}

	b2FlushCandidatePairs( &pairContext );

	b2TracyCZoneEnd( self_pairs );
}

#define B2_CROSS_SEED_COUNT 64
_Static_assert( ( B2_CROSS_SEED_COUNT & ( B2_CROSS_SEED_COUNT - 1 ) ) == 0, "must be power of 2" );

// This does a serial cross-tree breadth first search until the queue is full. Then it returns
// the queue pairs as seeds for a parallel search.
static int b2GatherCrossSeeds( const b2DynamicTree* treeA, const b2DynamicTree* treeB, b2NodePair* seeds )
{
	const b2TreeNode* nodesA = treeA->nodes;
	const b2TreeNode* nodesB = treeB->nodes;

	// Breadth-first search from the two roots as one pair. An empty tree has a sentinel root that
	// survives nothing.
	b2NodePair queue[2 * B2_CROSS_SEED_COUNT];
	int mask = 2 * B2_CROSS_SEED_COUNT - 1;
	int head = 0;
	int tail = 0;

	const b2TreeNode* rootA = nodesA + B2_ROOT_NODE;
	const b2TreeNode* rootB = nodesB + B2_ROOT_NODE;
	if ( b2TestPair( rootA, rootB ) )
	{
		queue[tail & mask] = (b2NodePair){ .a = *rootA, .b = *rootB };
		tail += 1;
	}

	int seedCount = 0;
	while ( head < tail && seedCount + ( tail - head ) + 3 < B2_CROSS_SEED_COUNT )
	{
		b2NodePair pair = queue[head & mask];
		head += 1;

		if ( b2IsLeaf( &pair.a ) || b2IsLeaf( &pair.b ) )
		{
			seeds[seedCount++] = pair;
			continue;
		}

		const b2TreeNode* a = nodesA + b2GetLeftChild( &pair.a );
		const b2TreeNode* b = nodesB + b2GetLeftChild( &pair.b );

		// Nodes have two children each, so four combinations.
		for ( int i = 0; i < 2; ++i )
		{
			for ( int j = 0; j < 2; ++j )
			{
				if ( b2TestPair( a + i, b + j ) )
				{
					queue[tail & mask] = (b2NodePair){ a[i], b[j] };
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
	const b2NodePair* seeds;
	int staticSeedCount;
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

	b2PairContext pairContext = { .world = world, .pairKeys = &world->taskContexts.data[workerIndex].pairKeys };

	for ( int i = startIndex; i < endIndex; ++i )
	{
		const b2TreeNode* nodesB = i < crossContext->staticSeedCount ? staticNodes : kinematicNodes;
		b2NodePair seed = crossContext->seeds[i];
		b2CollideCrossPairs( dynamicNodes, nodesB, &seed.a, &seed.b, &pairContext );
	}

	b2FlushCandidatePairs( &pairContext );

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
		B2_VALIDATE( bp->trees[b2_kinematicBody].dfsOrdered );
		B2_VALIDATE( bp->trees[b2_dynamicBody].dfsOrdered );
		return;
	}

	b2TracyCZoneNC( update_pairs, "Find Pairs", b2_colorMediumSlateBlue, true );

	b2Stack* alloc = &world->stack;

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2Array_Clear( world->taskContexts.data[i].pairKeys );
	}

	// Generate pairs by querying the dynamic body tree against itself and against
	// the kinematic and static trees.
	{
		// Get the sibling pairs of the dynamic body tree that have moved.
		const b2DynamicTree* dynamicTree = bp->trees + b2_dynamicBody;
		int pairCapacity = dynamicTree->nodeEnd / 2;
		bp->movedSiblings = b2StackAlloc( alloc, pairCapacity * sizeof( int ), "moved pairs" );
		int dynamicMoveCount = b2GatherMovedSiblings( dynamicTree, bp->movedSiblings );

		// Get seeds for colliding against the static and kinematic trees.
		b2NodePair crossSeeds[2 * B2_CROSS_SEED_COUNT];
		int staticSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_staticBody, crossSeeds );
		B2_ASSERT( staticSeedCount <= B2_CROSS_SEED_COUNT );
		int kinematicSeedCount = b2GatherCrossSeeds( dynamicTree, bp->trees + b2_kinematicBody, crossSeeds + staticSeedCount );
		B2_ASSERT( kinematicSeedCount <= B2_CROSS_SEED_COUNT );
		int crossMoveCount = staticSeedCount + kinematicSeedCount;

		// Collide the dynamic body tree against the static and kinematic trees.
		b2CrossContext crossContext = {
			.world = world,
			.seeds = crossSeeds,
			.staticSeedCount = staticSeedCount,
		};
		b2ParallelFor( world, &b2CrossPairsTask, crossMoveCount, 1, &crossContext );

		// Collide the dynamic body tree against itself.
		b2ParallelFor( world, &b2SelfPairsTask, dynamicMoveCount, 64, world );
	}

	b2DynamicTree_ClearMoved( bp->trees + b2_staticBody );

	b2TracyCZoneEnd( update_pairs );

	b2TracyCZoneNC( create_contacts, "Create Contacts", b2_colorCoral, true );

	// Update stale trees.
	b2EnqueueTreeUpdate( world );

	// Pairs arrive in deterministic order but scrambled relative to body and shape order
	// sorting them here improves solver performance.
	int pairCount = 0;
	for ( int i = 0; i < world->workerCount; ++i )
	{
		pairCount += world->taskContexts.data[i].pairKeys.count;
	}

	uint64_t* pairKeys = b2StackAlloc( alloc, b2MaxInt( pairCount, 1 ) * sizeof( uint64_t ), "pair keys" );
	int keyCount = 0;
	for ( int i = 0; i < world->workerCount; ++i )
	{
		const b2Array( uint64_t )* workerKeys = &world->taskContexts.data[i].pairKeys;
		if ( workerKeys->count > 0 )
		{
			memcpy( pairKeys + keyCount, workerKeys->data, workerKeys->count * sizeof( uint64_t ) );
			keyCount += workerKeys->count;
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

	b2StackFree( alloc, bp->movedSiblings );
	bp->movedSiblings = NULL;

	b2ValidateSolverSets( world );

	b2TracyCZoneEnd( create_contacts );
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
