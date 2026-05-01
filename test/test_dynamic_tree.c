// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"

#include "dynamic_tree.h"
#include "physics_world.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static int TreeCreateDestroy( void )
{
	b2AABB a = {
		.lowerBound = { -1.0f, -1.0f },
		.upperBound = { 2.0f, 2.0f },
	};

	b2DynamicTree tree = b2DynamicTree_Create( 16 );
	b2DynamicTree_CreateProxy( &tree, a, 1, 0 );

	ENSURE( tree.nodeCount > 0 );
	ENSURE( tree.proxyCount == 1 );

	b2DynamicTree_Destroy( &tree );

	ENSURE( tree.nodeCount == 0 );
	ENSURE( tree.proxyCount == 0 );

	return 0;
}

float RayCastCallbackFcn( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)input;
	(void)userData;

	int* proxyHit = context;
	*proxyHit = proxyId;
	return 0.0f;
}

static int TreeRayCastTest( void )
{
	// Test AABB centered at origin with bounds [-1, -1] to [1, 1]
	b2AABB a = { .lowerBound = { -1.0f, -1.0f }, .upperBound = { 1.0f, 1.0f }, };
	b2DynamicTree tree = b2DynamicTree_Create( 16 );
	int proxyId = b2DynamicTree_CreateProxy( &tree, a, 1, 0 );

	b2RayCastInput input = {};
	input.maxFraction = 1.0f;

	// Test 1: Ray hits AABB from left side
	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { 3.0f, 0.0f };

		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 2: Ray hits AABB from right side
	{
		b2Vec2 p1 = { 3.0f, 0.0f };
		b2Vec2 p2 = { -3.0f, 0.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 3: Ray hits AABB from bottom
	{
		b2Vec2 p1 = { 0.0f, -3.0f };
		b2Vec2 p2 = { 0.0f, 3.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 4: Ray hits AABB from top
	{
		b2Vec2 p1 = { 0.0f, 3.0f };
		b2Vec2 p2 = { 0.0f, -3.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 5: Ray misses AABB completely (parallel to x-axis)
	{
		b2Vec2 p1 = { -3.0f, 2.0f };
		b2Vec2 p2 = { 3.0f, 2.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
	}

	// Test 6: Ray misses AABB completely (parallel to y-axis)
	{
		b2Vec2 p1 = { 2.0f, -3.0f };
		b2Vec2 p2 = { 2.0f, 3.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
	}

	// Test 7: Ray starts inside AABB
	{
		b2Vec2 p1 = { 0.0f, 0.0f };
		b2Vec2 p2 = { 2.0f, 0.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 8: Ray hits corner of AABB (diagonal ray)
	{
		b2Vec2 p1 = { -2.0f, -2.0f };
		b2Vec2 p2 = { 2.0f, 2.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 9: Ray parallel to AABB edge but outside
	{
		b2Vec2 p1 = { -2.0f, 1.5f };
		b2Vec2 p2 = { 2.0f, 1.5f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
	}

	// Test 10: Ray parallel to AABB edge and exactly on boundary
	{
		b2Vec2 p1 = { -2.0f, 1.0f };
		b2Vec2 p2 = { 2.0f, 1.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 11: Very short ray that doesn't reach AABB
	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { -2.5f, 0.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
	}

	// Test 12: Zero-length ray (degenerate case)
	{
		b2Vec2 p1 = { 0.0f, 0.0f };
		b2Vec2 p2 = { 0.0f, 0.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	// Test 13: Ray hits AABB at exact boundary condition (t = 1.0)
	{
		b2Vec2 p1 = { -2.0f, 0.0f };
		b2Vec2 p2 = { -1.0f, 0.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyId );
	}

	b2DynamicTree_Destroy( &tree );

	return 0;
}

static bool QueryCollectCallback( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	int* out = context;
	out[proxyId] = 1;
	return true; // continue the query
}

static bool QueryCollectListCallback( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	int* list = context;
	int count = list[0];
	list[count + 1] = proxyId;
	list[0] = count + 1;
	return true;
}

static int TreeMultipleProxiesTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	b2AABB a1 = { .lowerBound = { -5.0f, -1.0f }, .upperBound = { -3.0f, 1.0f } };
	b2AABB a2 = { .lowerBound = { -1.0f, -1.0f }, .upperBound = { 1.0f, 1.0f } };
	b2AABB a3 = { .lowerBound = { 3.0f, -1.0f }, .upperBound = { 5.0f, 1.0f } };

	int id1 = b2DynamicTree_CreateProxy( &tree, a1, 0x1ull, 42 );
	int id2 = b2DynamicTree_CreateProxy( &tree, a2, 0x2ull, 43 );
	int id3 = b2DynamicTree_CreateProxy( &tree, a3, 0x4ull, 44 );

	ENSURE( b2DynamicTree_GetProxyCount( &tree ) == 3 );

	ENSURE( b2DynamicTree_GetUserData( &tree, id1 ) == 42 );
	ENSURE( b2DynamicTree_GetUserData( &tree, id2 ) == 43 );
	ENSURE( b2DynamicTree_GetUserData( &tree, id3 ) == 44 );

	ENSURE( b2DynamicTree_GetCategoryBits( &tree, id1 ) == 0x1ull );
	ENSURE( b2DynamicTree_GetCategoryBits( &tree, id2 ) == 0x2ull );
	ENSURE( b2DynamicTree_GetCategoryBits( &tree, id3 ) == 0x4ull );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeQueryTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	b2AABB a1 = { .lowerBound = { -5.0f, -1.0f }, .upperBound = { -3.0f, 1.0f } };
	b2AABB a2 = { .lowerBound = { -1.0f, -1.0f }, .upperBound = { 1.0f, 1.0f } };
	b2AABB a3 = { .lowerBound = { 3.0f, -1.0f }, .upperBound = { 5.0f, 1.0f } };

	int id1 = b2DynamicTree_CreateProxy( &tree, a1, 0xFFull, 0 );
	int id2 = b2DynamicTree_CreateProxy( &tree, a2, 0xFFull, 0 );
	int id3 = b2DynamicTree_CreateProxy( &tree, a3, 0xFFull, 0 );

	b2AABB queryA = { .lowerBound = { -2.0f, -2.0f }, .upperBound = { 2.0f, 2.0f } };

	int foundFlags[32] = { 0 };
	b2TreeStats stats = b2DynamicTree_Query( &tree, queryA, 0xFFFFFFFFull, QueryCollectCallback, foundFlags );

	// We expect at least the middle proxy to be visited.
	ENSURE( foundFlags[id2] == 1 );
	ENSURE( stats.leafVisits >= 1 );

	// Test QueryAll using list collector
	int list[16] = { 0 }; // list[0] holds count, following entries are ids
	b2TreeStats allStats = b2DynamicTree_QueryAll( &tree, queryA, QueryCollectListCallback, list );
	ENSURE( list[0] >= 1 ); // at least one proxy should be collected
	ENSURE( allStats.leafVisits >= 1 );

	b2DynamicTree_Destroy( &tree );
	(void)id1; (void)id3;
	return 0;
}

static int TreeMoveAndEnlargeTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	b2AABB a = { .lowerBound = { 0.0f, 0.0f }, .upperBound = { 1.0f, 1.0f } };
	int id = b2DynamicTree_CreateProxy( &tree, a, 0x1ull, 100 );

	// Move proxy to a new place
	b2AABB moved = { .lowerBound = { 10.0f, 10.0f }, .upperBound = { 11.0f, 11.0f } };
	b2DynamicTree_MoveProxy( &tree, id, moved );

	b2AABB got = b2DynamicTree_GetAABB( &tree, id );
	ENSURE( got.lowerBound.x == moved.lowerBound.x );
	ENSURE( got.lowerBound.y == moved.lowerBound.y );
	ENSURE( got.upperBound.x == moved.upperBound.x );
	ENSURE( got.upperBound.y == moved.upperBound.y );

	// Now enlarge the proxy
	b2AABB enlarge = { .lowerBound = { 9.5f, 9.5f }, .upperBound = { 11.5f, 11.5f } };
	b2DynamicTree_EnlargeProxy( &tree, id, enlarge );

	b2AABB got2 = b2DynamicTree_GetAABB( &tree, id );
	ENSURE( got2.lowerBound.x <= enlarge.lowerBound.x + 1e-6f );
	ENSURE( got2.upperBound.x >= enlarge.upperBound.x - 1e-6f );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeRebuildAndValidateTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	// Create a number of proxies to make rebuild meaningful
	for ( int i = 0; i < 12; ++i )
	{
		float x = (float)i * 2.0f;
		b2AABB a = { .lowerBound = { x - 0.5f, -0.5f }, .upperBound = { x + 0.5f, 0.5f } };
		b2DynamicTree_CreateProxy( &tree, a, 0xFFull, (uint64_t)i );
	}

	int sorted = b2DynamicTree_Rebuild( &tree, true );
	
	ENSURE( sorted >= 0 );
	ENSURE( b2DynamicTree_GetByteCount( &tree ) > 0 );
	ENSURE( b2DynamicTree_GetHeight( &tree ) > 0 );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeRowHeightTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	int columnCount = 200;
	for (int i = 0; i < columnCount; ++i)
	{
		float x = 1.0f * i;
		b2AABB a = { .lowerBound = { x, 0.0f }, .upperBound = { x + 1.0f, 1.0f } };
		b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i );
	}

	float minHeight = log2f((float)columnCount);

	ENSURE( b2DynamicTree_GetHeight( &tree ) < 2.0f * minHeight );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeGridHeightTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	int columnCount = 20;
	int rowCount = 20;
	for (int i = 0; i < columnCount; ++i)
	{
		float x = 1.0f * i;
		for (int j = 0; j < rowCount; ++j)
		{
			float y = 1.0f * j;
			b2AABB a = { .lowerBound = { x, y }, .upperBound = { x + 1.0f, y + 1.0f } };
			b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i );
		}
	}

	float minHeight = log2f( (float)(rowCount * columnCount) );

	ENSURE( b2DynamicTree_GetHeight( &tree ) < 2.0f * minHeight );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

#define GRID_COUNT 20

static int TreeGridMovementTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );

	int proxyIds[GRID_COUNT * GRID_COUNT];
	int index = 0;
	for (int i = 0; i < GRID_COUNT; ++i)
	{
		float x = 1.0f * i;
		for (int j = 0; j < GRID_COUNT; ++j)
		{
			float y = 1.0f * j;
			b2AABB a = { .lowerBound = { x, y }, .upperBound = { x + 1.0f, y + 1.0f } };
			proxyIds[index] = b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i );
			index += 1;
		}
	}

	ENSURE( index == GRID_COUNT * GRID_COUNT );

	float minHeight = log2f( (float)( GRID_COUNT * GRID_COUNT ) );

	int height1 = b2DynamicTree_GetHeight( &tree );
	ENSURE( height1 < 2.0f * minHeight );

	b2Vec2 offset = {10.0f, 20.0f};
	index = 0;
	for (int i = 0; i < GRID_COUNT; ++i)
	{
		for (int j = 0; j < GRID_COUNT; ++j)
		{
			b2AABB a = b2DynamicTree_GetAABB( &tree, proxyIds[index] );
			a.lowerBound = b2Add( a.lowerBound, offset );
			a.upperBound = b2Add( a.upperBound, offset );
			b2DynamicTree_MoveProxy( &tree, proxyIds[index], a );
			index += 1;
		}
	}

	int height2 = b2DynamicTree_GetHeight( &tree );
	ENSURE( height2 < 3.0f * minHeight );

	b2DynamicTree_Rebuild( &tree, true );

	int height3 = b2DynamicTree_GetHeight( &tree );
	ENSURE( height3 < 2.0f * minHeight );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

typedef struct BuildBruteForceContext
{
	const b2AABB* aabbs;
	int leafCount;
	int* hitFlags;
	b2AABB queryAabb;
} BuildBruteForceContext;

static bool BuildQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	BuildBruteForceContext* ctx = (BuildBruteForceContext*)context;
	if ( 0 <= proxyId && proxyId < ctx->leafCount )
	{
		ctx->hitFlags[proxyId] = 1;
	}
	return true;
}

static bool AabbOverlap( b2AABB a, b2AABB b )
{
	if ( a.upperBound.x < b.lowerBound.x || b.upperBound.x < a.lowerBound.x )
	{
		return false;
	}
	if ( a.upperBound.y < b.lowerBound.y || b.upperBound.y < a.lowerBound.y )
	{
		return false;
	}
	return true;
}

static int BuildFromLeavesCheck( int leafCount, const b2AABB* aabbs )
{
	uint64_t* userData = NULL;
	uint64_t* categoryBits = NULL;
	if ( leafCount > 0 )
	{
		userData = (uint64_t*)malloc( leafCount * sizeof( uint64_t ) );
		categoryBits = (uint64_t*)malloc( leafCount * sizeof( uint64_t ) );
		for ( int i = 0; i < leafCount; ++i )
		{
			userData[i] = (uint64_t)( i + 1 );
			categoryBits[i] = ( (uint64_t)1 ) << ( i & 63 );
		}
	}

	b2DynamicTree tree = b2DynamicTree_Create();
	b2DynamicTree_BuildFromLeaves( &tree, aabbs, userData, categoryBits, leafCount, NULL );

	if ( leafCount == 0 )
	{
		ENSURE( tree.proxyCount == 0 );
		ENSURE( tree.nodeCount == 0 );
		ENSURE( tree.root == -1 );
		b2DynamicTree_Destroy( &tree );
		return 0;
	}

	ENSURE( tree.proxyCount == leafCount );
	ENSURE( tree.nodeCount == ( leafCount == 1 ? 1 : 2 * leafCount - 1 ) );

	if ( leafCount == 1 )
	{
		ENSURE( tree.root == 0 );
	}
	else
	{
		ENSURE( tree.root >= leafCount && tree.root < tree.nodeCount );
	}

	// Validate that each leaf carries the input AABB and metadata.
	for ( int i = 0; i < leafCount; ++i )
	{
		b2AABB stored = b2DynamicTree_GetAABB( &tree, i );
		ENSURE( stored.lowerBound.x == aabbs[i].lowerBound.x );
		ENSURE( stored.lowerBound.y == aabbs[i].lowerBound.y );
		ENSURE( stored.upperBound.x == aabbs[i].upperBound.x );
		ENSURE( stored.upperBound.y == aabbs[i].upperBound.y );
		ENSURE( b2DynamicTree_GetUserData( &tree, i ) == userData[i] );
		ENSURE( b2DynamicTree_GetCategoryBits( &tree, i ) == categoryBits[i] );
	}

	// Height should be near log2(N) for a balanced median-split build.
	int height = b2DynamicTree_GetHeight( &tree );
	if ( leafCount > 1 )
	{
		float minHeight = log2f( (float)leafCount );
		ENSURE( (float)height < 3.0f * minHeight + 4.0f );
	}

	// Brute-force query verification: hit set from the tree must match the
	// brute-force overlap set for several query AABBs.
	if ( leafCount > 0 )
	{
		b2AABB queries[3] = {
			{ .lowerBound = { -1000.0f, -1000.0f }, .upperBound = { 1000.0f, 1000.0f } }, // all
			{ .lowerBound = { 0.0f, 0.0f }, .upperBound = { 5.0f, 5.0f } },
			{ .lowerBound = { 50.0f, 50.0f }, .upperBound = { 60.0f, 60.0f } },
		};

		int* hitFlags = (int*)calloc( leafCount, sizeof( int ) );
		int* expectedFlags = (int*)calloc( leafCount, sizeof( int ) );

		for ( int q = 0; q < 3; ++q )
		{
			memset( hitFlags, 0, leafCount * sizeof( int ) );
			memset( expectedFlags, 0, leafCount * sizeof( int ) );

			BuildBruteForceContext ctx = {
				.aabbs = aabbs,
				.leafCount = leafCount,
				.hitFlags = hitFlags,
				.queryAabb = queries[q],
			};
			b2DynamicTree_Query( &tree, queries[q], UINT64_MAX, BuildQueryCallback, &ctx );

			for ( int i = 0; i < leafCount; ++i )
			{
				expectedFlags[i] = AabbOverlap( aabbs[i], queries[q] ) ? 1 : 0;
			}

			for ( int i = 0; i < leafCount; ++i )
			{
				if ( hitFlags[i] != expectedFlags[i] )
				{
					printf( "  query %d leaf %d hit=%d expected=%d\n", q, i, hitFlags[i], expectedFlags[i] );
					free( hitFlags );
					free( expectedFlags );
					b2DynamicTree_Destroy( &tree );
					free( userData );
					free( categoryBits );
					return 1;
				}
			}
		}

		free( hitFlags );
		free( expectedFlags );
	}

	b2DynamicTree_Destroy( &tree );
	free( userData );
	free( categoryBits );
	return 0;
}

static int TreeBuildFromLeavesEdgeCases( void )
{
	// Empty tree.
	{
		ENSURE( BuildFromLeavesCheck( 0, NULL ) == 0 );
	}

	// Single leaf.
	{
		b2AABB aabb = { .lowerBound = { -1.0f, -1.0f }, .upperBound = { 1.0f, 1.0f } };
		ENSURE( BuildFromLeavesCheck( 1, &aabb ) == 0 );
	}

	// Two leaves.
	{
		b2AABB aabbs[2] = {
			{ .lowerBound = { -2.0f, -2.0f }, .upperBound = { -1.0f, -1.0f } },
			{ .lowerBound = { 1.0f, 1.0f }, .upperBound = { 2.0f, 2.0f } },
		};
		ENSURE( BuildFromLeavesCheck( 2, aabbs ) == 0 );
	}

	// Three leaves with all-coincident centers (forces the degenerate split path).
	{
		b2AABB aabbs[3] = {
			{ .lowerBound = { -0.5f, -0.5f }, .upperBound = { 0.5f, 0.5f } },
			{ .lowerBound = { -0.5f, -0.5f }, .upperBound = { 0.5f, 0.5f } },
			{ .lowerBound = { -0.5f, -0.5f }, .upperBound = { 0.5f, 0.5f } },
		};
		ENSURE( BuildFromLeavesCheck( 3, aabbs ) == 0 );
	}

	return 0;
}

static int TreeBuildFromLeavesGrid( void )
{
	// 30x30 grid + jitter. Stays under the parallel threshold (1024) so the
	// serial path is exercised; world is NULL anyway.
	const int side = 30;
	const int leafCount = side * side;
	b2AABB* aabbs = (b2AABB*)malloc( leafCount * sizeof( b2AABB ) );

	uint32_t rng = 0xC0FFEE;
	for ( int i = 0; i < side; ++i )
	{
		for ( int j = 0; j < side; ++j )
		{
			float x = (float)i;
			float y = (float)j;
			rng = rng * 1664525u + 1013904223u;
			float jx = ( (float)( rng & 0xFFFF ) / 65536.0f ) - 0.5f;
			rng = rng * 1664525u + 1013904223u;
			float jy = ( (float)( rng & 0xFFFF ) / 65536.0f ) - 0.5f;
			b2AABB a = {
				.lowerBound = { x + jx, y + jy },
				.upperBound = { x + jx + 1.0f, y + jy + 1.0f },
			};
			aabbs[i * side + j] = a;
		}
	}

	int rc = BuildFromLeavesCheck( leafCount, aabbs );
	free( aabbs );
	return rc;
}

// Synchronous fake task system: enqueueTask runs the task immediately. Lets
// the parallel build scaffolding execute its prefix-sum / scatter / spawn
// math without requiring a real worker pool. With one logical worker behind
// the dispatcher, blocks are claimed serially in order, so this also gives
// us a deterministic reference for what parallel execution should produce.
static void* SyncEnqueueTask( b2TaskCallback* task, void* taskContext, void* userContext )
{
	(void)userContext;
	task( taskContext );
	return NULL;
}

static void SyncFinishTask( void* userTask, void* userContext )
{
	(void)userTask;
	(void)userContext;
}

static void SetupSyncFakeWorld( b2World* world, int workerCount )
{
	memset( world, 0, sizeof( b2World ) );
	world->workerCount = workerCount;
	world->enqueueTaskFcn = &SyncEnqueueTask;
	world->finishTaskFcn = &SyncFinishTask;
	world->userTaskContext = NULL;
	world->taskCount = 0;
}

static int TreeBuildFromLeavesParallelScaffolding( void )
{
	// Cross the parallel-build floor (1024) and the parallel-partition
	// threshold (2048) so the scaffolding actually fires.
	const int side = 50;
	const int leafCount = side * side;
	b2AABB* aabbs = (b2AABB*)malloc( leafCount * sizeof( b2AABB ) );
	uint64_t* userData = (uint64_t*)malloc( leafCount * sizeof( uint64_t ) );
	uint64_t* categoryBits = (uint64_t*)malloc( leafCount * sizeof( uint64_t ) );

	uint32_t rng = 0xBADC0DE;
	for ( int i = 0; i < side; ++i )
	{
		for ( int j = 0; j < side; ++j )
		{
			float x = (float)i;
			float y = (float)j;
			rng = rng * 1664525u + 1013904223u;
			float jx = ( (float)( rng & 0xFFFF ) / 65536.0f ) - 0.5f;
			rng = rng * 1664525u + 1013904223u;
			float jy = ( (float)( rng & 0xFFFF ) / 65536.0f ) - 0.5f;
			int idx = i * side + j;
			aabbs[idx].lowerBound = (b2Vec2){ x + jx, y + jy };
			aabbs[idx].upperBound = (b2Vec2){ x + jx + 1.0f, y + jy + 1.0f };
			userData[idx] = (uint64_t)( idx + 1 );
			categoryBits[idx] = ( (uint64_t)1 ) << ( idx & 63 );
		}
	}

	b2DynamicTree treeSerial = b2DynamicTree_Create();
	b2DynamicTree_BuildFromLeaves( &treeSerial, aabbs, userData, categoryBits, leafCount, NULL );

	// Run the parallel path with a synchronous fake at workerCount = 4. Should
	// produce the same tree structure (parent/child relationships, AABBs,
	// heights, root) as the serial path because the partition is stable.
	b2World fakeWorld;
	SetupSyncFakeWorld( &fakeWorld, 4 );
	b2DynamicTree treeParallel = b2DynamicTree_Create();
	b2DynamicTree_BuildFromLeaves( &treeParallel, aabbs, userData, categoryBits, leafCount, &fakeWorld );

	ENSURE( treeSerial.proxyCount == treeParallel.proxyCount );
	ENSURE( treeSerial.nodeCount == treeParallel.nodeCount );
	ENSURE( treeSerial.root == treeParallel.root );
	ENSURE( b2DynamicTree_GetHeight( &treeSerial ) == b2DynamicTree_GetHeight( &treeParallel ) );

	for ( int i = 0; i < treeSerial.nodeCount; ++i )
	{
		b2AABB s = b2DynamicTree_GetAABB( &treeSerial, i );
		b2AABB p = b2DynamicTree_GetAABB( &treeParallel, i );
		ENSURE( s.lowerBound.x == p.lowerBound.x );
		ENSURE( s.lowerBound.y == p.lowerBound.y );
		ENSURE( s.upperBound.x == p.upperBound.x );
		ENSURE( s.upperBound.y == p.upperBound.y );
		ENSURE( b2DynamicTree_GetCategoryBits( &treeSerial, i ) == b2DynamicTree_GetCategoryBits( &treeParallel, i ) );
	}

	b2DynamicTree_Destroy( &treeSerial );
	b2DynamicTree_Destroy( &treeParallel );
	free( aabbs );
	free( userData );
	free( categoryBits );
	return 0;
}

static int TreeBuildFromLeavesRebuildable( void )
{
	// After BuildFromLeaves, the tree should still accept incremental ops.
	const int leafCount = 8;
	b2AABB aabbs[8];
	uint64_t userData[8];
	uint64_t categoryBits[8];
	for ( int i = 0; i < leafCount; ++i )
	{
		float x = (float)i * 2.0f;
		aabbs[i].lowerBound = (b2Vec2){ x - 0.5f, -0.5f };
		aabbs[i].upperBound = (b2Vec2){ x + 0.5f, 0.5f };
		userData[i] = (uint64_t)i;
		categoryBits[i] = 0xFFull;
	}

	b2DynamicTree tree = b2DynamicTree_Create();
	b2DynamicTree_BuildFromLeaves( &tree, aabbs, userData, categoryBits, leafCount, NULL );

	ENSURE( tree.proxyCount == leafCount );
	ENSURE( b2DynamicTree_GetHeight( &tree ) > 0 );

	// Move each leaf and confirm queries still find them.
	for ( int i = 0; i < leafCount; ++i )
	{
		b2AABB a = b2DynamicTree_GetAABB( &tree, i );
		a.lowerBound.y += 100.0f;
		a.upperBound.y += 100.0f;
		b2DynamicTree_MoveProxy( &tree, i, a );
	}

	b2AABB query = { .lowerBound = { -1000.0f, 99.0f }, .upperBound = { 1000.0f, 101.0f } };
	int hits[8] = { 0 };
	BuildBruteForceContext ctx = { .aabbs = aabbs, .leafCount = leafCount, .hitFlags = hits, .queryAabb = query };
	b2DynamicTree_Query( &tree, query, UINT64_MAX, BuildQueryCallback, &ctx );

	for ( int i = 0; i < leafCount; ++i )
	{
		ENSURE( hits[i] == 1 );
	}

	b2DynamicTree_Destroy( &tree );
	return 0;
}

int DynamicTreeTest( void )
{
	RUN_SUBTEST( TreeCreateDestroy );
	RUN_SUBTEST( TreeRayCastTest );
	RUN_SUBTEST( TreeMultipleProxiesTest );
	RUN_SUBTEST( TreeQueryTest );
	RUN_SUBTEST( TreeMoveAndEnlargeTest );
	RUN_SUBTEST( TreeRebuildAndValidateTest );
	RUN_SUBTEST( TreeRowHeightTest );
	RUN_SUBTEST( TreeGridHeightTest );
	RUN_SUBTEST( TreeGridMovementTest );
	RUN_SUBTEST( TreeBuildFromLeavesEdgeCases );
	RUN_SUBTEST( TreeBuildFromLeavesGrid );
	RUN_SUBTEST( TreeBuildFromLeavesParallelScaffolding );
	RUN_SUBTEST( TreeBuildFromLeavesRebuildable );

	// todo test queries versus brute force

	return 0;
}
