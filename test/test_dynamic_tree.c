// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "aabb.h"
#include "core.h"
#include "dynamic_tree.h"

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <string.h>

static int TreeCreateDestroy( void )
{
	b2AABB a = {
		.lowerBound = { -1.0f, -1.0f },
		.upperBound = { 2.0f, 2.0f },
	};

	b2DynamicTree tree = b2DynamicTree_Create( 16 );
	b2DynamicTree_CreateProxy( &tree, a, 1, 0, 0 );

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
	int proxyId = b2DynamicTree_CreateProxy( &tree, a, 1, 0, 0 );

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

	// An off center box catches an axis aligned ray tested against the wrong axis, which the origin
	// centered box above cannot. Two leaves also give the root a child pair.
	b2AABB b = { .lowerBound = { 10.0f, 4.0f }, .upperBound = { 11.0f, 6.0f } };
	int proxyIdB = b2DynamicTree_CreateProxy( &tree, b, 1, 1, 0 );

	// Test 14: Horizontal ray through the off center box
	{
		b2Vec2 p1 = { 0.0f, 5.0f };
		b2Vec2 p2 = { 21.0f, 5.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyIdB );
	}

	// Test 15: Vertical ray through the off center box
	{
		b2Vec2 p1 = { 10.5f, 0.0f };
		b2Vec2 p2 = { 10.5f, 21.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == proxyIdB );
	}

	// Test 16: Horizontal ray passing above the off center box
	{
		b2Vec2 p1 = { 0.0f, 7.0f };
		b2Vec2 p2 = { 21.0f, 7.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
	}

	// Test 17: Vertical ray passing beside the off center box
	{
		b2Vec2 p1 = { 12.0f, 0.0f };
		b2Vec2 p2 = { 12.0f, 21.0f };
		input.origin = p1;
		input.translation = b2Sub( p2, p1 );

		int proxyHit = -1;
		b2DynamicTree_RayCast( &tree, &input, 1, RayCastCallbackFcn, &proxyHit );

		ENSURE( proxyHit == -1 );
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

	int id1 = b2DynamicTree_CreateProxy( &tree, a1, 0x1ull, 42, 0 );
	int id2 = b2DynamicTree_CreateProxy( &tree, a2, 0x2ull, 43, 0 );
	int id3 = b2DynamicTree_CreateProxy( &tree, a3, 0x4ull, 44, 0 );

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

	int id1 = b2DynamicTree_CreateProxy( &tree, a1, 0xFFull, 0, 0 );
	int id2 = b2DynamicTree_CreateProxy( &tree, a2, 0xFFull, 0, 0 );
	int id3 = b2DynamicTree_CreateProxy( &tree, a3, 0xFFull, 0, 0 );

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
	int id = b2DynamicTree_CreateProxy( &tree, a, 0x1ull, 100, 0 );

	// Move proxy to a new place
	b2AABB moved = { .lowerBound = { 10.0f, 10.0f }, .upperBound = { 11.0f, 11.0f } };
	b2DynamicTree_MoveProxy( &tree, id, moved, 0 );

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
		b2DynamicTree_CreateProxy( &tree, a, 0xFFull, (uint64_t)i, 0 );
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
		b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i, 0 );
	}

	float minHeight = log2f((float)columnCount);

	b2DynamicTree_Validate( &tree );
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
			b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i, 0 );
		}
	}

	float minHeight = log2f( (float)(rowCount * columnCount) );

	b2DynamicTree_Validate( &tree );
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
			proxyIds[index] = b2DynamicTree_CreateProxy( &tree, a, 1, (uint64_t)i, 0 );
			index += 1;
		}
	}

	ENSURE( index == GRID_COUNT * GRID_COUNT );

	float minHeight = log2f( (float)( GRID_COUNT * GRID_COUNT ) );

	b2DynamicTree_Validate( &tree );
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
			b2DynamicTree_MoveProxy( &tree, proxyIds[index], a, 0 );
			index += 1;
		}
	}

	b2DynamicTree_Validate( &tree );
	int height2 = b2DynamicTree_GetHeight( &tree );
	ENSURE( height2 < 3.0f * minHeight );

	b2DynamicTree_Rebuild( &tree, true );

	b2DynamicTree_Validate( &tree );
	int height3 = b2DynamicTree_GetHeight( &tree );
	ENSURE( height3 < 2.0f * minHeight );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

#define TREE_TEST_PROXY_COUNT 200

// Small generator so the random tests repeat run to run
static uint32_t treeRandomState = 12345u;

static float RandomTreeFloat( float lower, float upper )
{
	treeRandomState = 1664525u * treeRandomState + 1013904223u;
	float unit = (float)( treeRandomState >> 8 ) * ( 1.0f / 16777216.0f );
	return lower + ( upper - lower ) * unit;
}

static b2AABB RandomTreeBox( float maxHalfExtent )
{
	float x = RandomTreeFloat( -50.0f, 50.0f );
	float y = RandomTreeFloat( -50.0f, 50.0f );
	float hx = RandomTreeFloat( 0.25f, maxHalfExtent );
	float hy = RandomTreeFloat( 0.25f, maxHalfExtent );
	return (b2AABB){ { x - hx, y - hy }, { x + hx, y + hy } };
}

static bool BoxEquals( b2AABB a, b2AABB b )
{
	return a.lowerBound.x == b.lowerBound.x && a.lowerBound.y == b.lowerBound.y && a.upperBound.x == b.upperBound.x &&
		   a.upperBound.y == b.upperBound.y;
}

static b2AABB EnlargeTreeBox( b2AABB a, float margin )
{
	return (b2AABB){ { a.lowerBound.x - margin, a.lowerBound.y - margin }, { a.upperBound.x + margin, a.upperBound.y + margin } };
}

typedef struct TreeHitSet
{
	bool hit[TREE_TEST_PROXY_COUNT];
	int count;
	bool bad;
} TreeHitSet;

static bool CollectHitCallback( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	TreeHitSet* hits = context;
	if ( proxyId < 0 || TREE_TEST_PROXY_COUNT <= proxyId || hits->hit[proxyId] )
	{
		// Out of range or reported twice
		hits->bad = true;
		return true;
	}

	hits->hit[proxyId] = true;
	hits->count += 1;
	return true;
}

// Query the tree and compare against every live proxy box
static int CompareQuery( const b2DynamicTree* tree, const int* proxyIds, int proxyCount, b2AABB box )
{
	TreeHitSet hits = { 0 };
	b2DynamicTree_Query( tree, box, B2_DEFAULT_MASK_BITS, CollectHitCallback, &hits );
	ENSURE( hits.bad == false );

	int expectedCount = 0;
	for ( int i = 0; i < proxyCount; ++i )
	{
		int proxyId = proxyIds[i];
		bool overlaps = b2AABB_Overlaps( b2DynamicTree_GetAABB( tree, proxyId ), box );
		ENSURE( hits.hit[proxyId] == overlaps );
		expectedCount += overlaps ? 1 : 0;
	}

	ENSURE( hits.count == expectedCount );
	return 0;
}

typedef struct TreeCastHit
{
	const b2DynamicTree* tree;
	int proxyId;
	float fraction;
} TreeCastHit;

// Entry fraction along the full translation, negative for a miss
static float RayBoxFraction( b2AABB box, b2Vec2 origin, b2Vec2 translation )
{
	b2CastOutput output = b2AABB_RayCast( box, origin, b2Add( origin, translation ) );
	return output.hit ? output.fraction : -1.0f;
}

// A box sweep against a box is a ray against the box grown by the swept extents
static float BoxCastFraction( b2AABB box, const b2BoxCastInput* input )
{
	b2Vec2 extents = b2AABB_Extents( input->box );
	b2AABB grown = { b2Sub( box.lowerBound, extents ), b2Add( box.upperBound, extents ) };
	return RayBoxFraction( grown, b2AABB_Center( input->box ), input->translation );
}

// The fraction is computed against the full translation rather than the clipped input so it
// matches the brute force bit for bit
static float ClosestRayCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	TreeCastHit* hit = context;
	float fraction = RayBoxFraction( b2DynamicTree_GetAABB( hit->tree, proxyId ), input->origin, input->translation );
	if ( fraction < 0.0f )
	{
		return -1.0f;
	}

	if ( fraction < hit->fraction )
	{
		hit->proxyId = proxyId;
		hit->fraction = fraction;
	}

	return fraction;
}

static float ClosestBoxCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	TreeCastHit* hit = context;
	float fraction = BoxCastFraction( b2DynamicTree_GetAABB( hit->tree, proxyId ), input );
	if ( fraction < 0.0f )
	{
		return -1.0f;
	}

	if ( fraction < hit->fraction )
	{
		hit->proxyId = proxyId;
		hit->fraction = fraction;
	}

	return fraction;
}

// Closest ray hit from the tree against the closest over every live proxy
static int CompareRayCast( const b2DynamicTree* tree, const int* proxyIds, int proxyCount, b2Vec2 origin, b2Vec2 translation )
{
	TreeCastHit hit = { tree, B2_NULL_INDEX, 1.0f };
	b2RayCastInput input = { .origin = origin, .translation = translation, .maxFraction = 1.0f };
	b2DynamicTree_RayCast( tree, &input, B2_DEFAULT_MASK_BITS, ClosestRayCallback, &hit );

	int bestId = B2_NULL_INDEX;
	float bestFraction = 1.0f;
	bool unique = true;
	for ( int i = 0; i < proxyCount; ++i )
	{
		int proxyId = proxyIds[i];
		float fraction = RayBoxFraction( b2DynamicTree_GetAABB( tree, proxyId ), origin, translation );
		if ( fraction < 0.0f )
		{
			continue;
		}

		if ( fraction < bestFraction )
		{
			bestId = proxyId;
			bestFraction = fraction;
			unique = true;
		}
		else if ( fraction == bestFraction )
		{
			unique = false;
		}
	}

	ENSURE( hit.fraction == bestFraction );
	if ( unique )
	{
		ENSURE( hit.proxyId == bestId );
	}

	return 0;
}

static int CompareBoxCast( const b2DynamicTree* tree, const int* proxyIds, int proxyCount, b2AABB box, b2Vec2 translation )
{
	TreeCastHit hit = { tree, B2_NULL_INDEX, 1.0f };
	b2BoxCastInput input = { .box = box, .translation = translation, .maxFraction = 1.0f };
	b2DynamicTree_BoxCast( tree, &input, B2_DEFAULT_MASK_BITS, ClosestBoxCallback, &hit );

	int bestId = B2_NULL_INDEX;
	float bestFraction = 1.0f;
	bool unique = true;
	for ( int i = 0; i < proxyCount; ++i )
	{
		int proxyId = proxyIds[i];
		float fraction = BoxCastFraction( b2DynamicTree_GetAABB( tree, proxyId ), &input );
		if ( fraction < 0.0f )
		{
			continue;
		}

		if ( fraction < bestFraction )
		{
			bestId = proxyId;
			bestFraction = fraction;
			unique = true;
		}
		else if ( fraction == bestFraction )
		{
			unique = false;
		}
	}

	ENSURE( hit.fraction == bestFraction );
	if ( unique )
	{
		ENSURE( hit.proxyId == bestId );
	}

	return 0;
}

// A record is marked exactly when a leaf below it is marked. Leaves are checked against the
// set the test marked itself.
static int CheckMarks( const b2DynamicTree* tree, int nodeIndex, const bool* markedProxies, bool* markedBelow )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	bool marked = false;
	for ( int slot = 0; slot < 2; ++slot )
	{
		const b2TreeChild* child = node->children + slot;
		if ( child->flagIndex == B2_NODE_SENTINEL )
		{
			continue;
		}

		bool childMarked = b2IsChildMoved( child );
		if ( b2IsLeaf( child ) )
		{
			ENSURE( childMarked == markedProxies[b2GetChildIndex( child )] );
		}
		else
		{
			bool below = false;
			if ( CheckMarks( tree, b2GetChildIndex( child ), markedProxies, &below ) != 0 )
			{
				return 1;
			}

			ENSURE( childMarked == below );
		}

		marked = marked || childMarked;
	}

	*markedBelow = marked;
	return 0;
}

// Every internal record holds exactly the union of its node's two records
static int CheckBounds( const b2DynamicTree* tree, int nodeIndex, b2AABB* bounds )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	b2AABB boxes[2];
	int boxCount = 0;
	for ( int slot = 0; slot < 2; ++slot )
	{
		const b2TreeChild* child = node->children + slot;
		if ( child->flagIndex == B2_NODE_SENTINEL )
		{
			continue;
		}

		if ( b2IsLeaf( child ) == false )
		{
			b2AABB below;
			if ( CheckBounds( tree, b2GetChildIndex( child ), &below ) != 0 )
			{
				return 1;
			}

			ENSURE( BoxEquals( below, child->aabb ) );
		}

		boxes[boxCount] = child->aabb;
		boxCount += 1;
	}

	ENSURE( boxCount > 0 );
	*bounds = boxCount == 2 ? b2AABB_Union( boxes[0], boxes[1] ) : boxes[0];
	return 0;
}

static bool RootMarked( const b2DynamicTree* tree )
{
	const b2TreeNode* root = tree->nodes + B2_ROOT_NODE;
	return b2IsChildMoved( root->children + 0 ) || b2IsChildMoved( root->children + 1 );
}

static int TreeOneProxyTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 1 );
	b2AABB box = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	int proxyId = b2DynamicTree_CreateProxy( &tree, box, 1, 7, false );
	int proxyIds[1] = { proxyId };
	b2DynamicTree_Validate( &tree );

	// The root is always internal, so one proxy leaves it with an empty slot
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.proxyCount == 1 );
	ENSURE( b2DynamicTree_GetHeight( &tree ) == 1 );
	ENSURE( b2DynamicTree_GetUserData( &tree, proxyId ) == 7 );
	ENSURE( b2DynamicTree_GetCategoryBits( &tree, proxyId ) == 1 );
	ENSURE( BoxEquals( b2DynamicTree_GetAABB( &tree, proxyId ), box ) );
	ENSURE( BoxEquals( b2DynamicTree_GetRootBounds( &tree ), box ) );

	b2AABB inside = { { 0.4f, 0.4f }, { 0.6f, 0.6f } };
	b2AABB outside = { { 5.0f, 5.0f }, { 6.0f, 6.0f } };
	ENSURE( CompareQuery( &tree, proxyIds, 1, inside ) == 0 );
	ENSURE( CompareQuery( &tree, proxyIds, 1, outside ) == 0 );

	b2Vec2 origin = { -1.0f, 0.5f };
	b2Vec2 translation = { 3.0f, 0.0f };
	TreeCastHit hit = { &tree, B2_NULL_INDEX, 1.0f };
	b2RayCastInput input = { .origin = origin, .translation = translation, .maxFraction = 1.0f };
	b2DynamicTree_RayCast( &tree, &input, B2_DEFAULT_MASK_BITS, ClosestRayCallback, &hit );
	ENSURE( hit.proxyId == proxyId );
	ENSURE( CompareRayCast( &tree, proxyIds, 1, origin, translation ) == 0 );

	b2Vec2 missOrigin = { -1.0f, 5.0f };
	ENSURE( CompareRayCast( &tree, proxyIds, 1, missOrigin, translation ) == 0 );

	b2AABB castBox = { { -3.0f, 0.25f }, { -2.0f, 0.75f } };
	b2Vec2 castTranslation = { 10.0f, 0.0f };
	ENSURE( CompareBoxCast( &tree, proxyIds, 1, castBox, castTranslation ) == 0 );

	// A full build keeps the empty slot
	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == 1 );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.dfsNodeCount == 1 );
	ENSURE( CompareQuery( &tree, proxyIds, 1, inside ) == 0 );

	// The leaf record is the root bounds, so a mark and refit show up there directly
	b2AABB enlarged = EnlargeTreeBox( box, 0.5f );
	b2DynamicTree_MarkEnlarged( &tree, proxyId, enlarged );
	ENSURE( RootMarked( &tree ) );
	b2DynamicTree_Refit( &tree );
	b2DynamicTree_Validate( &tree );
	ENSURE( BoxEquals( b2DynamicTree_GetAABB( &tree, proxyId ), enlarged ) );
	ENSURE( BoxEquals( b2DynamicTree_GetRootBounds( &tree ), enlarged ) );
	ENSURE( b2DynamicTree_Rebuild( &tree, false ) == 1 );
	ENSURE( RootMarked( &tree ) == false );
	b2DynamicTree_ValidateNoEnlarged( &tree );

	b2DynamicTree_DestroyProxy( &tree, proxyId );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.proxyCount == 0 );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( b2DynamicTree_GetHeight( &tree ) == 0 );
	ENSURE( CompareQuery( &tree, proxyIds, 0, inside ) == 0 );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

// The root never collapses. Removing down to one proxy empties a slot and the next insert fills it.
static int TreeEmptySlotTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( 16 );
	b2AABB boxA = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	b2AABB boxB = { { 10.0f, 0.0f }, { 11.0f, 1.0f } };
	b2AABB boxC = { { 20.0f, 0.0f }, { 21.0f, 1.0f } };
	b2AABB boxD = { { 30.0f, 0.0f }, { 31.0f, 1.0f } };
	b2AABB all = { { -1.0f, -1.0f }, { 40.0f, 2.0f } };

	int idA = b2DynamicTree_CreateProxy( &tree, boxA, 1, 0, false );
	int idB = b2DynamicTree_CreateProxy( &tree, boxB, 1, 1, false );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.proxyCount == 2 );

	b2DynamicTree_DestroyProxy( &tree, idA );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.proxyCount == 1 );
	int idsB[1] = { idB };
	ENSURE( CompareQuery( &tree, idsB, 1, all ) == 0 );
	ENSURE( CompareQuery( &tree, idsB, 1, boxA ) == 0 );

	// Fills the empty slot without a new internal node
	int idC = b2DynamicTree_CreateProxy( &tree, boxC, 1, 2, false );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.proxyCount == 2 );
	int idsBC[2] = { idB, idC };
	ENSURE( CompareQuery( &tree, idsBC, 2, all ) == 0 );

	// A third proxy needs an internal node below the root
	int idD = b2DynamicTree_CreateProxy( &tree, boxD, 1, 3, false );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 2 );
	ENSURE( tree.proxyCount == 3 );
	int idsBCD[3] = { idB, idC, idD };
	ENSURE( CompareQuery( &tree, idsBCD, 3, all ) == 0 );
	ENSURE( CompareQuery( &tree, idsBCD, 3, boxC ) == 0 );

	b2DynamicTree_DestroyProxy( &tree, idB );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	int idsCD[2] = { idC, idD };
	ENSURE( CompareQuery( &tree, idsCD, 2, all ) == 0 );

	b2DynamicTree_DestroyProxy( &tree, idC );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );

	b2DynamicTree_DestroyProxy( &tree, idD );
	b2DynamicTree_Validate( &tree );
	ENSURE( tree.nodeCount == 1 );
	ENSURE( tree.proxyCount == 0 );
	ENSURE( b2DynamicTree_GetHeight( &tree ) == 0 );
	ENSURE( CompareQuery( &tree, idsBCD, 0, all ) == 0 );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeQueryBruteForceTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( TREE_TEST_PROXY_COUNT );
	int proxyIds[TREE_TEST_PROXY_COUNT];
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
	{
		proxyIds[i] = b2DynamicTree_CreateProxy( &tree, RandomTreeBox( 2.0f ), 1, (uint64_t)i, false );
	}

	b2DynamicTree_Validate( &tree );
	for ( int i = 0; i < 32; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
	}

	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == TREE_TEST_PROXY_COUNT );
	b2DynamicTree_Validate( &tree );
	for ( int i = 0; i < 32; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
	}

	// Moves leave the built order behind
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; i += 2 )
	{
		b2DynamicTree_MoveProxy( &tree, proxyIds[i], RandomTreeBox( 2.0f ), false );
	}

	b2DynamicTree_Validate( &tree );
	for ( int i = 0; i < 32; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
	}

	b2DynamicTree_Destroy( &tree );
	return 0;
}

static int TreeCastBruteForceTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( TREE_TEST_PROXY_COUNT );
	int proxyIds[TREE_TEST_PROXY_COUNT];
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
	{
		proxyIds[i] = b2DynamicTree_CreateProxy( &tree, RandomTreeBox( 2.0f ), 1, (uint64_t)i, false );
	}

	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == TREE_TEST_PROXY_COUNT );
	b2DynamicTree_Validate( &tree );

	for ( int i = 0; i < 64; ++i )
	{
		b2Vec2 origin = { RandomTreeFloat( -60.0f, 60.0f ), RandomTreeFloat( -60.0f, 60.0f ) };
		float angle = RandomTreeFloat( -B2_PI, B2_PI );
		float length = RandomTreeFloat( 40.0f, 120.0f );
		b2Vec2 translation = { length * cosf( angle ), length * sinf( angle ) };
		ENSURE( CompareRayCast( &tree, proxyIds, TREE_TEST_PROXY_COUNT, origin, translation ) == 0 );

		b2Vec2 halfExtent = { RandomTreeFloat( 0.25f, 2.0f ), RandomTreeFloat( 0.25f, 2.0f ) };
		b2AABB box = { b2Sub( origin, halfExtent ), b2Add( origin, halfExtent ) };
		ENSURE( CompareBoxCast( &tree, proxyIds, TREE_TEST_PROXY_COUNT, box, translation ) == 0 );
	}

	b2DynamicTree_Destroy( &tree );
	return 0;
}

// Marks reach the root from any leaf, the sweep refit writes exact unions, and the rebuild clears
static int TreeMarkAndRefitTest( void )
{
	b2DynamicTree tree = b2DynamicTree_Create( TREE_TEST_PROXY_COUNT );
	int proxyIds[TREE_TEST_PROXY_COUNT];
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
	{
		proxyIds[i] = b2DynamicTree_CreateProxy( &tree, RandomTreeBox( 2.0f ), 1, (uint64_t)i, false );
	}

	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == TREE_TEST_PROXY_COUNT );
	ENSURE( tree.dfsNodeCount == tree.nodeCount );
	ENSURE( tree.nodeCount == TREE_TEST_PROXY_COUNT - 1 );
	ENSURE( RootMarked( &tree ) == false );

	bool marked[TREE_TEST_PROXY_COUNT] = { 0 };
	b2AABB enlarged[TREE_TEST_PROXY_COUNT] = { 0 };
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; i += 3 )
	{
		int proxyId = proxyIds[i];
		enlarged[proxyId] = EnlargeTreeBox( b2DynamicTree_GetAABB( &tree, proxyId ), 0.5f );
		b2DynamicTree_MarkEnlarged( &tree, proxyId, enlarged[proxyId] );
		marked[proxyId] = true;
	}

	bool markedBelow = false;
	ENSURE( CheckMarks( &tree, B2_ROOT_NODE, marked, &markedBelow ) == 0 );
	ENSURE( markedBelow );
	ENSURE( RootMarked( &tree ) );

	// The leaf records took the fat boxes before the refit
	for ( int i = 0; i < TREE_TEST_PROXY_COUNT; ++i )
	{
		if ( marked[i] )
		{
			ENSURE( BoxEquals( b2DynamicTree_GetAABB( &tree, i ), enlarged[i] ) );
		}
	}

	b2DynamicTree_Refit( &tree );
	b2DynamicTree_Validate( &tree );
	b2AABB bounds;
	ENSURE( CheckBounds( &tree, B2_ROOT_NODE, &bounds ) == 0 );
	ENSURE( BoxEquals( bounds, b2DynamicTree_GetRootBounds( &tree ) ) );

	// Marks survive the refit
	ENSURE( CheckMarks( &tree, B2_ROOT_NODE, marked, &markedBelow ) == 0 );
	ENSURE( markedBelow );
	for ( int i = 0; i < 16; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
	}

	// The rebuild clears every mark and restores the order
	ENSURE( b2DynamicTree_Rebuild( &tree, false ) > 0 );
	ENSURE( RootMarked( &tree ) == false );
	memset( marked, 0, sizeof( marked ) );
	ENSURE( CheckMarks( &tree, B2_ROOT_NODE, marked, &markedBelow ) == 0 );
	ENSURE( markedBelow == false );
	ENSURE( tree.dfsNodeCount == tree.nodeCount );
	ENSURE( CheckBounds( &tree, B2_ROOT_NODE, &bounds ) == 0 );
	for ( int i = 0; i < 16; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, TREE_TEST_PROXY_COUNT, RandomTreeBox( 8.0f ) ) == 0 );
	}

	// A single deep leaf still marks the whole path to the root
	int loneId = proxyIds[TREE_TEST_PROXY_COUNT / 2];
	enlarged[loneId] = EnlargeTreeBox( b2DynamicTree_GetAABB( &tree, loneId ), 0.5f );
	b2DynamicTree_MarkEnlarged( &tree, loneId, enlarged[loneId] );
	marked[loneId] = true;
	ENSURE( CheckMarks( &tree, B2_ROOT_NODE, marked, &markedBelow ) == 0 );
	ENSURE( markedBelow );
	b2DynamicTree_Refit( &tree );
	ENSURE( CheckBounds( &tree, B2_ROOT_NODE, &bounds ) == 0 );
	ENSURE( b2DynamicTree_Rebuild( &tree, false ) > 0 );
	b2DynamicTree_ValidateNoEnlarged( &tree );

	b2DynamicTree_Destroy( &tree );
	return 0;
}

// The built order is lost by an insert and kept by a remove, and the sweep steps over the hole
static int TreeDfsOrderTest( void )
{
	int proxyCount = 50;
	b2DynamicTree tree = b2DynamicTree_Create( TREE_TEST_PROXY_COUNT );
	int proxyIds[TREE_TEST_PROXY_COUNT];
	for ( int i = 0; i < proxyCount; ++i )
	{
		proxyIds[i] = b2DynamicTree_CreateProxy( &tree, RandomTreeBox( 2.0f ), 1, (uint64_t)i, false );
	}

	ENSURE( tree.dfsNodeCount == 0 );
	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == proxyCount );
	ENSURE( tree.dfsNodeCount == tree.nodeCount );
	ENSURE( tree.nodeCount == proxyCount - 1 );

	proxyIds[proxyCount] = b2DynamicTree_CreateProxy( &tree, RandomTreeBox( 2.0f ), 1, (uint64_t)proxyCount, false );
	proxyCount += 1;
	ENSURE( tree.dfsNodeCount == 0 );
	b2DynamicTree_Validate( &tree );

	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == proxyCount );
	ENSURE( tree.dfsNodeCount == tree.nodeCount );

	b2DynamicTree_MoveProxy( &tree, proxyIds[7], RandomTreeBox( 2.0f ), false );
	ENSURE( tree.dfsNodeCount == 0 );
	b2DynamicTree_Validate( &tree );

	ENSURE( b2DynamicTree_Rebuild( &tree, true ) == proxyCount );
	int orderedCount = tree.dfsNodeCount;
	ENSURE( orderedCount == tree.nodeCount );

	// Removes leave holes but every survivor stays above its parent
	b2DynamicTree_DestroyProxy( &tree, proxyIds[3] );
	b2DynamicTree_DestroyProxy( &tree, proxyIds[20] );
	proxyIds[3] = proxyIds[proxyCount - 1];
	proxyIds[20] = proxyIds[proxyCount - 2];
	proxyCount -= 2;
	ENSURE( tree.dfsNodeCount == orderedCount );
	ENSURE( tree.nodeCount == orderedCount - 2 );
	b2DynamicTree_Validate( &tree );

	b2AABB enlarged = EnlargeTreeBox( b2DynamicTree_GetAABB( &tree, proxyIds[10] ), 0.5f );
	b2DynamicTree_MarkEnlarged( &tree, proxyIds[10], enlarged );
	ENSURE( RootMarked( &tree ) );
	b2DynamicTree_Refit( &tree );
	b2DynamicTree_Validate( &tree );
	b2AABB bounds;
	ENSURE( CheckBounds( &tree, B2_ROOT_NODE, &bounds ) == 0 );
	ENSURE( BoxEquals( bounds, b2DynamicTree_GetRootBounds( &tree ) ) );
	for ( int i = 0; i < 16; ++i )
	{
		ENSURE( CompareQuery( &tree, proxyIds, proxyCount, RandomTreeBox( 8.0f ) ) == 0 );
	}

	ENSURE( b2DynamicTree_Rebuild( &tree, false ) > 0 );
	ENSURE( tree.dfsNodeCount == tree.nodeCount );
	ENSURE( tree.nodeCount == proxyCount - 1 );
	b2DynamicTree_Validate( &tree );

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
	RUN_SUBTEST( TreeOneProxyTest );
	RUN_SUBTEST( TreeEmptySlotTest );
	RUN_SUBTEST( TreeQueryBruteForceTest );
	RUN_SUBTEST( TreeCastBruteForceTest );
	RUN_SUBTEST( TreeMarkAndRefitTest );
	RUN_SUBTEST( TreeDfsOrderTest );

	// todo test queries versus brute force

	return 0;
}
