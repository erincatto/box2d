// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "world_snapshot.h"

#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "container.h"
#include "core.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "physics_world.h"
#include "recording.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/types.h"

#include <string.h>

// Snapshot image magic and version
#define B2_SNAP_MAGIC 0x32534E42u // 'BNS2'

// Bump this if any of the data structures below get modified. The layout hash only catches
// size changes, a same-size reinterpretation like the contact cache reshape needs this bump.
#define B2_SNAP_VERSION 3u

// Header flag bits
#define B2_SNAP_FLAG_VALIDATION 0x1u	   // image was built with validation, only used for diagnostics
#define B2_SNAP_FLAG_DOUBLE_PRECISION 0x2u // image was built with double precision world positions

// Layout hash seeds from all structs we memcpy, plus key constants.
// Changing any struct or constant updates the hash. Re-purposing or swapping
// fields might not change the hash
static uint32_t b2ComputeLayoutHash( void )
{
	// FNV-1a hash
	uint32_t h = 2166136261u;
#define MIX( x )                                                                                                                 \
	h ^= (uint32_t)( x );                                                                                                        \
	h *= 16777619u;
	MIX( sizeof( b2Body ) )
	MIX( sizeof( b2BodySim ) )
	MIX( sizeof( b2BodyState ) )
	MIX( sizeof( b2Shape ) )
	MIX( sizeof( b2ChainShape ) )
	MIX( sizeof( b2Contact ) )
	MIX( sizeof( b2ContactSim ) )
	MIX( sizeof( b2Joint ) )
	MIX( sizeof( b2JointSim ) )
	MIX( sizeof( b2Island ) )
	MIX( sizeof( b2IslandSim ) )
	MIX( sizeof( b2ContactLink ) )
	MIX( sizeof( b2JointLink ) )
	MIX( sizeof( b2Sensor ) )
	MIX( sizeof( b2Visitor ) )
	MIX( sizeof( b2SolverSet ) )
	MIX( sizeof( b2GraphColor ) )
	MIX( sizeof( b2DynamicTree ) )
	MIX( sizeof( b2TreeNode ) )
	MIX( sizeof( b2SetItem ) )
	MIX( sizeof( b2IdPool ) )
	MIX( sizeof( b2SurfaceMaterial ) )
	MIX( B2_GRAPH_COLOR_COUNT )
	MIX( b2_bodyTypeCount )
	MIX( sizeof( void* ) )
#undef MIX
	return h;
}

// Snapshot image header, written at offset 0
typedef struct b2SnapHeader
{
	uint32_t magic;
	uint32_t version;
	uint32_t layoutHash;
	uint32_t flags; // B2_SNAP_FLAG_*, keeps 16-byte alignment
} b2SnapHeader;

// Compile-time tripwires for the structs serialized field-by-field. If one fires, a field was added or
// reordered: resync the matching code in b2SerializeWorld / b2DeserializeIntoShell and bump
// B2_SNAP_VERSION.
#if INTPTR_MAX == INT64_MAX
_Static_assert( sizeof( b2ChainShape ) == 48, "b2ChainShape layout changed; resync snapshot chain serialization" );
_Static_assert( sizeof( b2Sensor ) == 56, "b2Sensor layout changed; resync snapshot sensor serialization" );
_Static_assert( sizeof( b2Island ) == 64, "b2Island layout changed; resync snapshot island serialization" );
#endif

// Bounds-checked read cursor, mirrors b2RecReader discipline
typedef struct b2SnapReader
{
	const uint8_t* data;
	int cursor;
	int size;
	bool ok;
} b2SnapReader;

static void b2SnapRCheck( b2SnapReader* r, int need )
{
	if ( need < 0 || (int64_t)r->cursor + (int64_t)need > (int64_t)r->size )
	{
		r->ok = false;
	}
}

static void b2SnapR_Bytes( b2SnapReader* r, void* dst, int n )
{
	b2SnapRCheck( r, n );
	if ( !r->ok )
	{
		return;
	}
	memcpy( dst, r->data + r->cursor, n );
	r->cursor += n;
}

static int b2SnapR_I32( b2SnapReader* r )
{
	int32_t v = 0;
	b2SnapR_Bytes( r, &v, 4 );
	return (int)v;
}

static uint32_t b2SnapR_U32( b2SnapReader* r )
{
	uint32_t v = 0;
	b2SnapR_Bytes( r, &v, 4 );
	return v;
}

static void b2SnapW_I32( b2RecBuffer* buf, int v )
{
	int32_t w = (int32_t)v;
	b2RecBufAppend( buf, &w, 4 );
}

static void b2SnapW_U32( b2RecBuffer* buf, uint32_t v )
{
	b2RecBufAppend( buf, &v, 4 );
}

static void b2SnapW_Bytes( b2RecBuffer* buf, const void* src, int n )
{
	b2RecBufAppend( buf, src, n );
}

// Reject a count read from the image before it reaches an allocation or memset. count must be non
// negative, its in-memory footprint must fit in int, and the stream must hold at least minStreamBytes
// per element. A corrupt or truncated image then fails.
static bool b2SnapCheckCount( const b2SnapReader* r, int count, int memSize, int minStreamBytes )
{
	if ( count < 0 || memSize < 0 || minStreamBytes < 0 )
	{
		return false;
	}
	if ( memSize > 0 && count > INT32_MAX / memSize )
	{
		return false;
	}
	int64_t remaining = (int64_t)r->size - (int64_t)r->cursor;
	return (int64_t)count * (int64_t)minStreamBytes <= remaining;
}

// Serialize a POD array: count then raw bytes
#define b2SerPodArray( buf, arr )                                                                                                \
	do                                                                                                                           \
	{                                                                                                                            \
		b2SnapW_I32( buf, ( arr ).count );                                                                                       \
		if ( ( arr ).count > 0 )                                                                                                 \
		{                                                                                                                        \
			b2SnapW_Bytes( buf, ( arr ).data, ( arr ).count * (int)sizeof( *( arr ).data ) );                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Serialize a sparse struct array whose element carries a host userData pointer. userData is host
// wiring, not simulation state, so write it as NULL: images stay reproducible and never persist host
// addresses across a save. On restore the field reads back NULL.
#define b2SerSimArray( buf, arr, type )                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		b2SnapW_I32( buf, ( arr ).count );                                                                                       \
		for ( int slot = 0; slot < ( arr ).count; ++slot )                                                                       \
		{                                                                                                                        \
			type elem = ( arr ).data[slot];                                                                                      \
			elem.userData = NULL;                                                                                                \
			b2SnapW_Bytes( buf, &elem, (int)sizeof( type ) );                                                                    \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Deserialize a POD array: validate count, resize, then memcpy
#define b2DesPodArray( r, arr )                                                                                                  \
	do                                                                                                                           \
	{                                                                                                                            \
		int cnt = b2SnapR_I32( r );                                                                                              \
		int elemSize = (int)sizeof( *( arr ).data );                                                                             \
		if ( ( r )->ok && b2SnapCheckCount( r, cnt, elemSize, elemSize ) == false )                                              \
		{                                                                                                                        \
			( r )->ok = false;                                                                                                   \
		}                                                                                                                        \
		if ( ( r )->ok && cnt > 0 )                                                                                              \
		{                                                                                                                        \
			b2Array_Resize( arr, cnt );                                                                                          \
			b2SnapR_Bytes( r, ( arr ).data, cnt * elemSize );                                                                    \
		}                                                                                                                        \
		else if ( ( r )->ok )                                                                                                    \
		{                                                                                                                        \
			( arr ).count = 0;                                                                                                   \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Id pool: nextIndex + freeArray
static void b2SerIdPool( b2RecBuffer* buf, const b2IdPool* pool )
{
	b2SnapW_I32( buf, pool->nextIndex );
	b2SerPodArray( buf, pool->freeArray );
}

static void b2DesIdPool( b2SnapReader* r, b2IdPool* pool )
{
	pool->nextIndex = b2SnapR_I32( r );
	b2DesPodArray( r, pool->freeArray );
}

// BitSet: blockCount + raw uint64_t words
static void b2SerBitSet( b2RecBuffer* buf, const b2BitSet* bs )
{
	b2SnapW_U32( buf, bs->blockCount );
	if ( bs->blockCount > 0 )
	{
		b2SnapW_Bytes( buf, bs->bits, (int)( bs->blockCount * sizeof( uint64_t ) ) );
	}
}

// Restore a bitset, leak-clean: destroy the existing one, alloc fresh at exact blockCount capacity.
// Always keep bits non-NULL, matching b2CreateBitSet, so a later b2GrowBitSet on a count-0 bitset
// has a buffer to grow from. An empty color bitset serializes as 0 blocks but must restore usable.
static void b2DesBitSet( b2SnapReader* r, b2BitSet* bs )
{
	uint32_t blockCount = b2SnapR_U32( r );
	if ( r->ok && b2SnapCheckCount( r, (int)blockCount, (int)sizeof( uint64_t ), (int)sizeof( uint64_t ) ) == false )
	{
		r->ok = false;
	}
	b2DestroyBitSet( bs );
	if ( !r->ok )
	{
		return;
	}
	uint32_t blockCapacity = blockCount > 0 ? blockCount : 1;
	bs->bits = b2Alloc( blockCapacity * sizeof( uint64_t ) );
	memset( bs->bits, 0, blockCapacity * sizeof( uint64_t ) );
	bs->blockCapacity = blockCapacity;
	bs->blockCount = blockCount;
	if ( blockCount > 0 )
	{
		b2SnapR_Bytes( r, bs->bits, (int)( blockCount * sizeof( uint64_t ) ) );
	}
}

// HashSet (pairSet): raw items at full capacity, probe order depends on it
static void b2SerHashSet( b2RecBuffer* buf, const b2HashSet* hs )
{
	b2SnapW_U32( buf, hs->capacity );
	b2SnapW_U32( buf, hs->count );
	if ( hs->capacity > 0 )
	{
		b2SnapW_Bytes( buf, hs->items, (int)( hs->capacity * sizeof( b2SetItem ) ) );
	}
}

static void b2DesHashSet( b2SnapReader* r, b2HashSet* hs )
{
	uint32_t cap = b2SnapR_U32( r );
	uint32_t cnt = b2SnapR_U32( r );
	// Probing masks with capacity-1, so capacity must be a power of two and count can't exceed it.
	// (cap & (cap-1)) == 0 also accepts 0, which the empty branch handles.
	bool valid = b2SnapCheckCount( r, (int)cap, (int)sizeof( b2SetItem ), (int)sizeof( b2SetItem ) ) && ( cap & ( cap - 1 ) ) == 0 &&
				 cnt <= cap;
	if ( r->ok && valid == false )
	{
		r->ok = false;
	}
	// Destroy the fresh empty set the shell gave us
	b2DestroySet( hs );
	if ( !r->ok )
	{
		return;
	}
	if ( cap > 0 )
	{
		hs->items = b2Alloc( cap * sizeof( b2SetItem ) );
		hs->capacity = cap;
		hs->count = cnt;
		b2SnapR_Bytes( r, hs->items, (int)( cap * sizeof( b2SetItem ) ) );
	}
	else
	{
		hs->items = NULL;
		hs->capacity = 0;
		hs->count = 0;
	}
}

// DynamicTree: scalars + full nodeCapacity nodes (freeList chains through free slots)
static void b2SerTree( b2RecBuffer* buf, const b2DynamicTree* tree )
{
	b2SnapW_I32( buf, tree->root );
	b2SnapW_I32( buf, tree->nodeCount );
	b2SnapW_I32( buf, tree->nodeCapacity );
	b2SnapW_I32( buf, tree->freeList );
	b2SnapW_I32( buf, tree->proxyCount );
	if ( tree->nodeCapacity > 0 )
	{
		b2SnapW_Bytes( buf, tree->nodes, tree->nodeCapacity * (int)sizeof( b2TreeNode ) );
	}
}

static void b2DesTree( b2SnapReader* r, b2DynamicTree* tree )
{
	int root = b2SnapR_I32( r );
	int nodeCount = b2SnapR_I32( r );
	int nodeCapacity = b2SnapR_I32( r );
	int freeList = b2SnapR_I32( r );
	int proxyCount = b2SnapR_I32( r );

	if ( r->ok && b2SnapCheckCount( r, nodeCapacity, (int)sizeof( b2TreeNode ), (int)sizeof( b2TreeNode ) ) == false )
	{
		r->ok = false;
	}

	// Free what the shell or a live world holds. A live tree that ran a rebuild also owns
	// rebuild scratch, so free that too. Null everything so a failure here leaves the tree
	// safe to destroy.
	b2Free( tree->nodes, tree->nodeCapacity * (int)sizeof( b2TreeNode ) );
	b2Free( tree->leafIndices, tree->rebuildCapacity * (int)sizeof( int ) );
	b2Free( tree->leafBoxes, tree->rebuildCapacity * (int)sizeof( b2AABB ) );
	b2Free( tree->leafCenters, tree->rebuildCapacity * (int)sizeof( b2Vec2 ) );
	b2Free( tree->binIndices, tree->rebuildCapacity * (int)sizeof( int ) );
	tree->nodes = NULL;
	tree->leafIndices = NULL;
	tree->leafBoxes = NULL;
	tree->leafCenters = NULL;
	tree->binIndices = NULL;
	tree->nodeCapacity = 0;
	tree->rebuildCapacity = 0;

	if ( !r->ok )
	{
		return;
	}

	tree->root = root;
	tree->nodeCount = nodeCount;
	tree->nodeCapacity = nodeCapacity;
	tree->freeList = freeList;
	tree->proxyCount = proxyCount;

	if ( nodeCapacity > 0 )
	{
		tree->nodes = b2Alloc( nodeCapacity * (int)sizeof( b2TreeNode ) );
		b2SnapR_Bytes( r, tree->nodes, nodeCapacity * (int)sizeof( b2TreeNode ) );
	}
}

// Solver set: setIndex + 5 POD arrays
static void b2SerSolverSet( b2RecBuffer* buf, const b2SolverSet* set )
{
	b2SnapW_I32( buf, set->setIndex );
	b2SerPodArray( buf, set->bodySims );
	b2SerPodArray( buf, set->bodyStates );
	b2SerPodArray( buf, set->jointSims );
	b2SerPodArray( buf, set->contactSims );
	b2SerPodArray( buf, set->islandSims );
}

static void b2DesSolverSet( b2SnapReader* r, b2SolverSet* set )
{
	set->setIndex = b2SnapR_I32( r );
	b2DesPodArray( r, set->bodySims );
	b2DesPodArray( r, set->bodyStates );
	b2DesPodArray( r, set->jointSims );
	b2DesPodArray( r, set->contactSims );
	b2DesPodArray( r, set->islandSims );
}

// Graph color: bodySet + contactSims + jointSims (overflow color has no bodySet)
static void b2SerGraphColor( b2RecBuffer* buf, const b2GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b2SerBitSet( buf, &color->bodySet );
	}
	b2SerPodArray( buf, color->contactSims );
	b2SerPodArray( buf, color->jointSims );
}

static void b2DesGraphColor( b2SnapReader* r, b2GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b2DesBitSet( r, &color->bodySet );
	}
	b2DesPodArray( r, color->contactSims );
	b2DesPodArray( r, color->jointSims );
	// Transient wideConstraints/overflowConstraints stay NULL, wideConstraintCount 0
}

// World scalar config (simulation settings only, no runtime/shell fields)
// Only simulation scalars belong here, never host or worker state (workerCount,
// scheduler, callbacks, user data). b2World_Restore relies on that so an in-place
// restore preserves the live world's wiring.
static void b2SerWorldConfig( b2RecBuffer* buf, const b2World* world )
{
	b2SnapW_Bytes( buf, &world->gravity, sizeof( b2Vec2 ) );
	b2SnapW_Bytes( buf, &world->hitEventThreshold, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->restitutionThreshold, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->maxLinearSpeed, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactSpeed, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactHertz, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactDampingRatio, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactRecycleDistance, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->stepIndex, sizeof( uint64_t ) );
	b2SnapW_I32( buf, world->splitIslandId );
	// Step scaling cached for the force/torque reporting getters, which run between steps
	b2SnapW_Bytes( buf, &world->inv_h, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->inv_dt, sizeof( float ) );
	// End-event double-buffer parity, so the first post-restore event query reads the right half
	b2SnapW_I32( buf, world->endEventArrayIndex );
	// maxCapacity (b2Capacity struct)
	b2SnapW_Bytes( buf, &world->maxCapacity, sizeof( b2Capacity ) );
	// bool flags packed as individual bytes for layout stability
	uint8_t flags = 0;
	flags |= world->enableSleep ? 0x01u : 0u;
	flags |= world->enableWarmStarting ? 0x02u : 0u;
	flags |= world->enableContactSoftening ? 0x04u : 0u;
	flags |= world->enableContinuous ? 0x08u : 0u;
	flags |= world->enableSpeculative ? 0x10u : 0u;
	b2RecBufAppend( buf, &flags, 1 );
}

static void b2DesWorldConfig( b2SnapReader* r, b2World* world )
{
	b2SnapR_Bytes( r, &world->gravity, sizeof( b2Vec2 ) );
	b2SnapR_Bytes( r, &world->hitEventThreshold, sizeof( float ) );
	b2SnapR_Bytes( r, &world->restitutionThreshold, sizeof( float ) );
	b2SnapR_Bytes( r, &world->maxLinearSpeed, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactSpeed, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactHertz, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactDampingRatio, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactRecycleDistance, sizeof( float ) );
	b2SnapR_Bytes( r, &world->stepIndex, sizeof( uint64_t ) );
	world->splitIslandId = b2SnapR_I32( r );
	b2SnapR_Bytes( r, &world->inv_h, sizeof( float ) );
	b2SnapR_Bytes( r, &world->inv_dt, sizeof( float ) );
	world->endEventArrayIndex = b2SnapR_I32( r );
	b2SnapR_Bytes( r, &world->maxCapacity, sizeof( b2Capacity ) );
	uint8_t flags = 0;
	b2SnapR_Bytes( r, &flags, 1 );
	world->enableSleep = ( flags & 0x01u ) != 0;
	world->enableWarmStarting = ( flags & 0x02u ) != 0;
	world->enableContactSoftening = ( flags & 0x04u ) != 0;
	world->enableContinuous = ( flags & 0x08u ) != 0;
	world->enableSpeculative = ( flags & 0x10u ) != 0;
}

void b2SerializeWorld( b2World* world, b2RecBuffer* buf )
{
	// Image header
	b2SnapHeader hdr;
	hdr.magic = B2_SNAP_MAGIC;
	hdr.version = B2_SNAP_VERSION;
	hdr.layoutHash = b2ComputeLayoutHash();
	hdr.flags = B2_ENABLE_VALIDATION ? B2_SNAP_FLAG_VALIDATION : 0u;
	if ( b2IsDoublePrecision() )
	{
		hdr.flags |= B2_SNAP_FLAG_DOUBLE_PRECISION;
	}
	b2SnapW_Bytes( buf, &hdr, (int)sizeof( hdr ) );

	// World config
	b2SerWorldConfig( buf, world );

	// 7 id pools
	b2SerIdPool( buf, &world->bodyIdPool );
	b2SerIdPool( buf, &world->shapeIdPool );
	b2SerIdPool( buf, &world->chainIdPool );
	b2SerIdPool( buf, &world->contactIdPool );
	b2SerIdPool( buf, &world->jointIdPool );
	b2SerIdPool( buf, &world->islandIdPool );
	b2SerIdPool( buf, &world->solverSetIdPool );

	// Solver sets
	int setCount = world->solverSets.count;
	b2SnapW_I32( buf, setCount );
	for ( int i = 0; i < setCount; ++i )
	{
		b2SerSolverSet( buf, world->solverSets.data + i );
	}

	// Sparse arrays. Bodies, shapes and joints carry a host userData pointer scrubbed to NULL on write.
	// Contacts have no userData, so they go out as raw POD.
	b2SerSimArray( buf, world->bodies, b2Body );
	b2SerSimArray( buf, world->shapes, b2Shape );
	b2SerPodArray( buf, world->contacts );
	b2SerSimArray( buf, world->joints, b2Joint );

	// Chain shapes: POD scalars then per-live-slot heap arrays
	int chainCount = world->chainShapes.count;
	b2SnapW_I32( buf, chainCount );
	for ( int i = 0; i < chainCount; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		// Write POD scalars
		b2SnapW_I32( buf, chain->id );
		b2SnapW_I32( buf, chain->bodyId );
		b2SnapW_I32( buf, chain->nextChainId );
		b2SnapW_I32( buf, chain->count );
		b2SnapW_I32( buf, chain->materialCount );
		b2SnapW_Bytes( buf, &chain->generation, sizeof( uint16_t ) );
		if ( chain->id != B2_NULL_INDEX )
		{
			// Live slot: write the two heap arrays
			b2SnapW_Bytes( buf, chain->shapeIndices, chain->count * (int)sizeof( int ) );
			b2SnapW_Bytes( buf, chain->materials, chain->materialCount * (int)sizeof( b2SurfaceMaterial ) );
		}
	}

	// Sensors: shapeId + 3 visitor arrays per slot
	int sensorCount = world->sensors.count;
	b2SnapW_I32( buf, sensorCount );
	for ( int i = 0; i < sensorCount; ++i )
	{
		b2Sensor* s = world->sensors.data + i;
		b2SnapW_I32( buf, s->shapeId );
		b2SerPodArray( buf, s->hits );
		b2SerPodArray( buf, s->overlaps1 );
		b2SerPodArray( buf, s->overlaps2 );
	}

	// Islands: POD scalars + 3 inner arrays per slot
	int islandCount = world->islands.count;
	b2SnapW_I32( buf, islandCount );
	for ( int i = 0; i < islandCount; ++i )
	{
		b2Island* island = world->islands.data + i;
		b2SnapW_I32( buf, island->setIndex );
		b2SnapW_I32( buf, island->localIndex );
		b2SnapW_I32( buf, island->islandId );
		b2SnapW_I32( buf, island->constraintRemoveCount );
		b2SerPodArray( buf, island->bodies );
		b2SerPodArray( buf, island->contacts );
		b2SerPodArray( buf, island->joints );
	}

	// Broad phase
	b2BroadPhase* bp = &world->broadPhase;
	for ( int t = 0; t < b2_bodyTypeCount; ++t )
	{
		b2SerTree( buf, &bp->trees[t] );
	}
	for ( int t = 0; t < b2_bodyTypeCount; ++t )
	{
		b2SerBitSet( buf, &bp->movedProxies[t] );
	}
	b2SerPodArray( buf, bp->moveArray );
	b2SerHashSet( buf, &bp->pairSet );

	// Constraint graph: B2_GRAPH_COLOR_COUNT colors
	b2ConstraintGraph* graph = &world->constraintGraph;
	for ( int c = 0; c < B2_GRAPH_COLOR_COUNT; ++c )
	{
		b2SerGraphColor( buf, &graph->colors[c], c == B2_OVERFLOW_INDEX );
	}
}

// Free per-object heap the overwrite steps below don't reach, so restoring over a
// populated world doesn't leak. Outer arrays stay alive for the steps to reuse.
// Mirrors the per-object teardown in b2DestroyWorld.
static void b2FreeLiveSimElements( b2World* world )
{
	for ( int i = 0; i < world->chainShapes.count; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		if ( chain->id != B2_NULL_INDEX )
		{
			b2FreeChainData( chain );
		}
	}

	for ( int i = 0; i < world->sensors.count; ++i )
	{
		b2Sensor* sensor = world->sensors.data + i;
		b2Array_Destroy( sensor->hits );
		b2Array_Destroy( sensor->overlaps1 );
		b2Array_Destroy( sensor->overlaps2 );
	}

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b2Island* island = world->islands.data + i;
		b2Array_Destroy( island->bodies );
		b2Array_Destroy( island->contacts );
		b2Array_Destroy( island->joints );
	}
}

// Overwrite a world with the simulation state from the reader. Mirrors the write
// order in b2SerializeWorld. The world must be a clean shell from b2CreateWorld, or
// a live world whose per-object heap was first freed by b2FreeLiveSimElements. Host
// wiring (scheduler, callbacks, user data) is never touched. Returns false on a
// corrupt image.
static bool b2DeserializeIntoShell( b2SnapReader* r, b2World* world )
{
	// Step 1: world scalars
	b2DesWorldConfig( r, world );

	// Step 2: 7 id pools (overwrite entirely, including solverSetIdPool from the 3 pre-created sets)
	b2DesIdPool( r, &world->bodyIdPool );
	b2DesIdPool( r, &world->shapeIdPool );
	b2DesIdPool( r, &world->chainIdPool );
	b2DesIdPool( r, &world->contactIdPool );
	b2DesIdPool( r, &world->jointIdPool );
	b2DesIdPool( r, &world->islandIdPool );
	b2DesIdPool( r, &world->solverSetIdPool );

	// Step 3: solver sets
	// (a) Destroy the 5 inner arrays of the 3 pre-created sets WITHOUT freeing the set id
	//     (b2DestroySolverSet would do that, but we already authored the id pool above)
	for ( int i = 0; i < world->solverSets.count; ++i )
	{
		b2SolverSet* set = world->solverSets.data + i;
		b2Array_Destroy( set->bodySims );
		b2Array_Destroy( set->bodyStates );
		b2Array_Destroy( set->contactSims );
		b2Array_Destroy( set->jointSims );
		b2Array_Destroy( set->islandSims );
	}

	// Each set writes at least setIndex plus 5 array counts
	int setCount = b2SnapR_I32( r );
	if ( r->ok && b2SnapCheckCount( r, setCount, (int)sizeof( b2SolverSet ), 6 * (int)sizeof( int ) ) == false )
	{
		r->ok = false;
	}

	if ( r->ok )
	{
		// (b) Resize outer array to hold all sets, zeroing new slots so their headers are clean
		b2Array_ResizeAndSetZero( world->solverSets, setCount );

		// (c) Deserialize each set
		for ( int i = 0; i < setCount; ++i )
		{
			b2DesSolverSet( r, world->solverSets.data + i );
		}
	}

	// Step 4: sparse arrays. userData was written as NULL by the serializer, so a plain copy restores
	// it cleanly with no host pointers to scrub here.
	b2DesPodArray( r, world->bodies );
	b2DesPodArray( r, world->shapes );
	b2DesPodArray( r, world->contacts );
	b2DesPodArray( r, world->joints );

	// Step 5: chain shapes
	{
		// Destroy the shell's chainShapes array (empty, but has a backing allocation)
		b2Array_Destroy( world->chainShapes );
		b2Array_Create( world->chainShapes );

		// Each chain writes 5 ints plus a uint16 generation
		int chainCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, chainCount, (int)sizeof( b2ChainShape ), 5 * (int)sizeof( int ) + (int)sizeof( uint16_t ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->chainShapes, chainCount );
			// Zero the whole array so free slots have NULL pointers
			memset( world->chainShapes.data, 0, chainCount * sizeof( b2ChainShape ) );
		}

		for ( int i = 0; i < chainCount && r->ok; ++i )
		{
			b2ChainShape* chain = world->chainShapes.data + i;
			chain->id = b2SnapR_I32( r );
			chain->bodyId = b2SnapR_I32( r );
			chain->nextChainId = b2SnapR_I32( r );
			chain->count = b2SnapR_I32( r );
			chain->materialCount = b2SnapR_I32( r );
			b2SnapR_Bytes( r, &chain->generation, sizeof( uint16_t ) );
			// A partial read leaves id as 0, which is a valid slot value, so gate the live branch on r->ok
			if ( r->ok && chain->id != B2_NULL_INDEX )
			{
				if ( b2SnapCheckCount( r, chain->count, (int)sizeof( int ), (int)sizeof( int ) ) == false ||
					 b2SnapCheckCount( r, chain->materialCount, (int)sizeof( b2SurfaceMaterial ), (int)sizeof( b2SurfaceMaterial ) ) == false )
				{
					r->ok = false;
					break;
				}
				// Live slot: allocate and copy heap arrays
				chain->shapeIndices = b2Alloc( chain->count * (int)sizeof( int ) );
				b2SnapR_Bytes( r, chain->shapeIndices, chain->count * (int)sizeof( int ) );
				chain->materials = b2Alloc( chain->materialCount * (int)sizeof( b2SurfaceMaterial ) );
				b2SnapR_Bytes( r, chain->materials, chain->materialCount * (int)sizeof( b2SurfaceMaterial ) );
			}
			else
			{
				// Free slot must have NULL pointers; zero init above handles this
				chain->shapeIndices = NULL;
				chain->materials = NULL;
			}
		}
	}

	// Step 6: sensors
	{
		// Destroy the shell's sensors array
		b2Array_Destroy( world->sensors );
		b2Array_Create( world->sensors );

		// Each sensor writes shapeId plus 3 array counts
		int sensorCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, sensorCount, (int)sizeof( b2Sensor ), 4 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->sensors, sensorCount );
			// Zero so inner array headers start clean
			memset( world->sensors.data, 0, sensorCount * sizeof( b2Sensor ) );
		}

		for ( int i = 0; i < sensorCount && r->ok; ++i )
		{
			b2Sensor* s = world->sensors.data + i;
			s->shapeId = b2SnapR_I32( r );
			// Re-init inner arrays then fill them
			b2Array_Create( s->hits );
			b2Array_Create( s->overlaps1 );
			b2Array_Create( s->overlaps2 );
			b2DesPodArray( r, s->hits );
			b2DesPodArray( r, s->overlaps1 );
			b2DesPodArray( r, s->overlaps2 );
		}
	}

	// Step 7: islands
	{
		// Destroy the shell's islands array
		b2Array_Destroy( world->islands );
		b2Array_Create( world->islands );

		// Each island writes 4 ints plus 3 array counts
		int islandCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, islandCount, (int)sizeof( b2Island ), 7 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->islands, islandCount );
			memset( world->islands.data, 0, islandCount * sizeof( b2Island ) );
		}

		for ( int i = 0; i < islandCount && r->ok; ++i )
		{
			b2Island* island = world->islands.data + i;
			island->setIndex = b2SnapR_I32( r );
			island->localIndex = b2SnapR_I32( r );
			island->islandId = b2SnapR_I32( r );
			island->constraintRemoveCount = b2SnapR_I32( r );
			b2Array_Create( island->bodies );
			b2Array_Create( island->contacts );
			b2Array_Create( island->joints );
			b2DesPodArray( r, island->bodies );
			b2DesPodArray( r, island->contacts );
			b2DesPodArray( r, island->joints );
		}
	}

	// Step 8: broad phase
	{
		b2BroadPhase* bp = &world->broadPhase;

		// Trees: the shell already allocated tree nodes; free them and replace
		for ( int t = 0; t < b2_bodyTypeCount; ++t )
		{
			b2DesTree( r, &bp->trees[t] );
		}

		// movedProxies bitsets: destroy shell's and replace
		for ( int t = 0; t < b2_bodyTypeCount; ++t )
		{
			b2DesBitSet( r, &bp->movedProxies[t] );
		}

		// moveArray
		b2Array_Destroy( bp->moveArray );
		b2Array_Create( bp->moveArray );
		b2DesPodArray( r, bp->moveArray );

		// pairSet
		b2DesHashSet( r, &bp->pairSet );

		// Transient move results stay at shell's NULL/0
	}

	// Step 9: constraint graph
	{
		b2ConstraintGraph* graph = &world->constraintGraph;
		for ( int c = 0; c < B2_GRAPH_COLOR_COUNT; ++c )
		{
			b2DesGraphColor( r, &graph->colors[c], c == B2_OVERFLOW_INDEX );
		}
	}

	return r->ok;
}

// Validate a snapshot image header and arm the reader just past it. Returns false on a rejected image
// (too small, bad magic, wrong version, layout mismatch), leaving the caller's world untouched. A
// layout mismatch caused purely by differing validation builds is called out, since the validation
// gated fields change struct sizes and such images can never share a layout.
static bool b2OpenSnapshotImage( const uint8_t* image, int size, b2SnapReader* r )
{
	if ( image == NULL || size < (int)sizeof( b2SnapHeader ) )
	{
		return false;
	}

	b2SnapHeader hdr;
	memcpy( &hdr, image, sizeof( hdr ) );
	if ( hdr.magic != B2_SNAP_MAGIC || hdr.version != B2_SNAP_VERSION )
	{
		return false;
	}

	// World positions are stored at the build precision, so the layout differs irreconcilably across
	// modes. Called out before the layout hash so the cause is clear rather than a generic mismatch.
	bool imageDouble = ( hdr.flags & B2_SNAP_FLAG_DOUBLE_PRECISION ) != 0;
	bool buildDouble = b2IsDoublePrecision();
	if ( imageDouble != buildDouble )
	{
		b2Log( "snapshot precision mismatch: image %s, this build %s\n", imageDouble ? "double" : "float",
			   buildDouble ? "double" : "float" );
		return false;
	}

	if ( hdr.layoutHash != b2ComputeLayoutHash() )
	{
		bool imageValidation = ( hdr.flags & B2_SNAP_FLAG_VALIDATION ) != 0;
		if ( imageValidation != (bool)B2_ENABLE_VALIDATION )
		{
			b2Log( "snapshot layout mismatch: image built with validation %s, this build %s\n", imageValidation ? "on" : "off",
				   B2_ENABLE_VALIDATION ? "on" : "off" );
		}
		else
		{
			b2Log( "snapshot layout mismatch\n" );
		}
		return false;
	}

	r->data = image;
	r->cursor = (int)sizeof( hdr );
	r->size = size;
	r->ok = true;
	return true;
}

b2WorldId b2CreateWorldFromSnapshot( const uint8_t* image, int size, int workerCount )
{
	b2WorldId nullId = b2_nullWorldId;

	b2SnapReader readerStorage;
	b2SnapReader* r = &readerStorage;
	if ( b2OpenSnapshotImage( image, size, r ) == false )
	{
		return nullId;
	}

	// Build a minimal valid def so b2CreateWorld produces a fully valid shell
	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = workerCount;
	def.enqueueTask = NULL;
	def.finishTask = NULL;
	def.userTaskContext = NULL;

	// Capacity is only a sizing hint. Every container is resized from the image below
	b2WorldId id = b2CreateWorld( &def );
	if ( !b2World_IsValid( id ) )
	{
		return nullId;
	}

	b2World* world = b2GetWorldFromId( id );

	if ( b2DeserializeIntoShell( r, world ) == false )
	{
		// Image was corrupt; clean up by destroying the world
		b2DestroyWorld( id );
		return nullId;
	}

	// A world loaded from scratch carries no host pointers
	world->preSolveFcn = NULL;
	world->preSolveContext = NULL;
	world->customFilterFcn = NULL;
	world->customFilterContext = NULL;
	world->userData = NULL;

	return id;
}

bool b2World_Restore( b2WorldId worldId, const uint8_t* image, int size )
{
	// Validate the image fully before touching the world so a bad image leaves it intact
	b2SnapReader readerStorage;
	b2SnapReader* r = &readerStorage;
	if ( b2OpenSnapshotImage( image, size, r ) == false )
	{
		return false;
	}

	b2World* world = b2GetWorldFromId( worldId );

	// Restoring mid step would corrupt an in-flight solve
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return false;
	}

	// Point of no return. The slot, generation, and host wiring are kept, so held ids
	// resolve into the rebuilt world. A truncated payload past here leaves the world
	// unusable and the caller must destroy it.
	b2FreeLiveSimElements( world );

	return b2DeserializeIntoShell( r, world );
}

int b2World_Snapshot( b2WorldId worldId, uint8_t* image, int capacity )
{
	b2World* world = b2GetWorldFromId( worldId );

	// Serializing mid step would capture an inconsistent, in-flight world
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return 0;
	}

	// Size query: count the bytes without allocating or copying the whole image
	if ( image == NULL )
	{
		b2RecBuffer counter = { 0 };
		counter.countOnly = true;
		b2SerializeWorld( world, &counter );
		return counter.size;
	}

	b2RecBuffer buf = { 0 };
	b2SerializeWorld( world, &buf );
	int size = buf.size;

	if ( size <= capacity )
	{
		memcpy( image, buf.data, size );
	}

	b2RecBufFree( &buf );
	return size;
}

static uint64_t b2FnvMixBytes( uint64_t hash, const void* data, int n )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( int i = 0; i < n; ++i )
	{
		hash = ( hash ^ (uint64_t)p[i] ) * B2_SNAP_FNV_PRIME;
	}
	return hash;
}

static uint64_t b2FnvMixFloat( uint64_t hash, float f )
{
	uint32_t bits;
	memcpy( &bits, &f, 4 );
	return ( hash ^ (uint64_t)bits ) * B2_SNAP_FNV_PRIME;
}

static uint64_t b2FnvMixInt( uint64_t hash, int v )
{
	return ( hash ^ (uint64_t)(uint32_t)v ) * B2_SNAP_FNV_PRIME;
}

uint64_t b2HashWorldStateDeep( b2World* world )
{
	uint64_t hash = B2_SNAP_FNV_INIT;

	// Bodies: same iteration order as b2HashWorldState (sparse array, skip free slots)
	int bodyCount = world->bodies.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			continue;
		}

		b2BodySim* sim = b2GetBodySim( world, body );

		hash = b2FnvMixPosition( hash, sim->transform.p );
		hash = b2FnvMixFloat( hash, sim->transform.q.c );
		hash = b2FnvMixFloat( hash, sim->transform.q.s );

		b2BodyState* state = b2GetBodyState( world, body );
		if ( state != NULL )
		{
			hash = b2FnvMixFloat( hash, state->linearVelocity.x );
			hash = b2FnvMixFloat( hash, state->linearVelocity.y );
			hash = b2FnvMixFloat( hash, state->angularVelocity );
		}

		// Index bookkeeping
		hash = b2FnvMixInt( hash, body->setIndex );
		hash = b2FnvMixInt( hash, body->localIndex );
	}

	// Contacts: sparse array, skip free slots (colorIndex == B2_NULL_INDEX and setIndex == B2_NULL_INDEX for free)
	int contactCount = world->contacts.count;
	for ( int i = 0; i < contactCount; ++i )
	{
		b2Contact* contact = world->contacts.data + i;
		if ( contact->contactId != i )
		{
			continue;
		}

		hash = b2FnvMixInt( hash, contact->setIndex );
		hash = b2FnvMixInt( hash, contact->colorIndex );
		hash = b2FnvMixInt( hash, contact->localIndex );

		// Manifold points + impulses from the contact sim
		// Exclude B2_ENABLE_VALIDATION-gated bodyIdA/bodyIdB for build-config stability
		b2ContactSim* sim = b2GetContactSim( world, contact );
		if ( sim != NULL )
		{
			b2Manifold* m = &sim->manifold;
			hash = b2FnvMixInt( hash, m->pointCount );
			for ( int p = 0; p < m->pointCount; ++p )
			{
				hash = b2FnvMixFloat( hash, m->points[p].normalImpulse );
				hash = b2FnvMixFloat( hash, m->points[p].tangentImpulse );
				hash = b2FnvMixFloat( hash, m->points[p].totalNormalImpulse );
			}
		}
	}

	// Joints: sparse array, skip free slots
	int jointCount = world->joints.count;
	for ( int i = 0; i < jointCount; ++i )
	{
		b2Joint* joint = world->joints.data + i;
		if ( joint->jointId != i )
		{
			continue;
		}

		hash = b2FnvMixInt( hash, joint->setIndex );
		hash = b2FnvMixInt( hash, joint->colorIndex );
		hash = b2FnvMixInt( hash, joint->localIndex );

		b2JointSim* sim = b2GetJointSim( world, joint );
		if ( sim != NULL )
		{
			// Hash accumulated impulses per joint type
			switch ( sim->type )
			{
				case b2_distanceJoint:
					hash = b2FnvMixFloat( hash, sim->distanceJoint.impulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.upperImpulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.motorImpulse );
					break;
				case b2_motorJoint:
					hash = b2FnvMixFloat( hash, sim->motorJoint.linearVelocityImpulse.x );
					hash = b2FnvMixFloat( hash, sim->motorJoint.linearVelocityImpulse.y );
					hash = b2FnvMixFloat( hash, sim->motorJoint.angularVelocityImpulse );
					hash = b2FnvMixFloat( hash, sim->motorJoint.linearSpringImpulse.x );
					hash = b2FnvMixFloat( hash, sim->motorJoint.linearSpringImpulse.y );
					hash = b2FnvMixFloat( hash, sim->motorJoint.angularSpringImpulse );
					break;
				case b2_prismaticJoint:
					hash = b2FnvMixBytes( hash, &sim->prismaticJoint.impulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.upperImpulse );
					break;
				case b2_revoluteJoint:
					hash = b2FnvMixBytes( hash, &sim->revoluteJoint.linearImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.upperImpulse );
					break;
				case b2_weldJoint:
					hash = b2FnvMixBytes( hash, &sim->weldJoint.linearImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->weldJoint.angularImpulse );
					break;
				case b2_wheelJoint:
					hash = b2FnvMixFloat( hash, sim->wheelJoint.perpImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.upperImpulse );
					break;
				default:
					break;
			}
		}
	}

	// 7 id pools: nextIndex + count
	hash = b2FnvMixInt( hash, world->bodyIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->bodyIdPool ) );
	hash = b2FnvMixInt( hash, world->shapeIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->shapeIdPool ) );
	hash = b2FnvMixInt( hash, world->chainIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->chainIdPool ) );
	hash = b2FnvMixInt( hash, world->contactIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->contactIdPool ) );
	hash = b2FnvMixInt( hash, world->jointIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->jointIdPool ) );
	hash = b2FnvMixInt( hash, world->islandIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->islandIdPool ) );
	hash = b2FnvMixInt( hash, world->solverSetIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->solverSetIdPool ) );

	// Solver set count
	hash = b2FnvMixInt( hash, world->solverSets.count );

	return hash;
}
