// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"

static int TreeCreateDestroy( void )
{
	b2AABB a = {
		.lowerBound = { -1.0f, -1.0f },
		.upperBound = { 2.0f, 2.0f },
	};

	b2DynamicTree tree = b2DynamicTree_Create();
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
	b2DynamicTree tree = b2DynamicTree_Create();
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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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
	b2DynamicTree tree = b2DynamicTree_Create();

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

	return 0;
}
