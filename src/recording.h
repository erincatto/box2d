// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box2d/id.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// FNV-1a 64-bit constants
#define B2_SNAP_FNV_INIT 14695981039346656037ull
#define B2_SNAP_FNV_PRIME 1099511628211ull

// Mix a world position at full width, or the determinism gates would validate only to float
// precision and pass vacuously far from the origin
static inline uint64_t b2FnvMixPosition( uint64_t hash, b2Pos p )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	uint64_t bx, by;
	memcpy( &bx, &p.x, 8 );
	memcpy( &by, &p.y, 8 );
#else
	uint32_t fx, fy;
	memcpy( &fx, &p.x, 4 );
	memcpy( &fy, &p.y, 4 );
	uint64_t bx = fx, by = fy;
#endif
	hash = ( hash ^ bx ) * B2_SNAP_FNV_PRIME;
	hash = ( hash ^ by ) * B2_SNAP_FNV_PRIME;
	return hash;
}

typedef struct b2World b2World;

// Magic value 'B2RC' in little-endian: bytes B2, R, C yield 0x43523242
#define B2_REC_MAGIC 0x43523242u

// Recording format version. Any mismatch refuses to load. The minor tracks op stream layout
// changes that keep the 32 byte header shape, such as the query origin args.
#define B2_REC_VERSION_MAJOR 3
#define B2_REC_VERSION_MINOR 2

// File header, fixed 32 bytes, little-endian
typedef struct b2RecHeader
{
	uint32_t magic;			 // 'B2RC' = 0x43523242
	uint16_t versionMajor;	 // B2_REC_VERSION_MAJOR
	uint16_t versionMinor;	 // B2_REC_VERSION_MINOR
	uint32_t reserved2;
	float lengthScale;		 // The world length scale
	uint8_t reserved3;
	uint8_t pointerWidth;	 // sizeof(void*), gates POD-def memcpy
	uint8_t bigEndian;		 // 0 on all supported targets
	uint8_t validationEnabled; // 1 if built with validation, only for diagnostics on a layout mismatch
	uint32_t reserved1;
	uint64_t snapshotSize;	 // bytes of snapshot blob after the header
} b2RecHeader;

_Static_assert( sizeof( b2RecHeader ) == 32, "recording header must be 32 bytes" );

// Growable append-only byte buffer. Doubles on demand. In countOnly mode it tallies size without
// allocating, so a serialize can be sized cheaply before a second pass fills a real buffer.
typedef struct b2RecBuffer
{
	uint8_t* data;
	int capacity;
	int size;
	bool countOnly;
} b2RecBuffer;

// User-owned recording buffer. The world appends into it while recording; the user saves and
// destroys it. Opaque across the public API.
typedef struct b2Recording
{
	b2RecBuffer buffer;
	int recordStart; // offset of the 3-byte size field for u24 backpatch
	b2Mutex* lock;	 // serializes query record commits across concurrent query threads

	// Union of world bounds over every recorded step, written out at stop so a replay can frame
	// the whole motion. haveBounds gates the first union the same way b2World_GetBounds does.
	b2AABB accumulatedBounds;
	bool haveBounds;
} b2Recording;

// C type aliases per TAG, used in codegen arg structs
typedef bool b2RecCType_BOOL;
typedef int32_t b2RecCType_I32;
typedef uint32_t b2RecCType_U32;
typedef uint64_t b2RecCType_U64;
typedef float b2RecCType_F32;
typedef b2Vec2 b2RecCType_VEC2;
typedef b2Rot b2RecCType_ROT;
typedef b2Transform b2RecCType_XF;
typedef b2Pos b2RecCType_POSITION;
typedef b2WorldTransform b2RecCType_WORLDXF;
typedef const char* b2RecCType_STR;
typedef b2WorldId b2RecCType_WORLDID;
typedef b2BodyId b2RecCType_BODYID;
typedef b2ShapeId b2RecCType_SHAPEID;
typedef b2ChainId b2RecCType_CHAINID;
typedef b2JointId b2RecCType_JOINTID;
typedef b2Circle b2RecCType_CIRCLE;
typedef b2Capsule b2RecCType_CAPSULE;
typedef b2Segment b2RecCType_SEGMENT;
typedef b2Polygon b2RecCType_POLYGON;
typedef b2ChainSegment b2RecCType_CHAINSEG;
typedef b2Filter b2RecCType_FILTER;
typedef b2SurfaceMaterial b2RecCType_MATERIAL;
typedef b2MassData b2RecCType_MASSDATA;
typedef b2MotionLocks b2RecCType_LOCKS;
typedef b2ExplosionDef b2RecCType_EXPLOSIONDEF;
typedef b2BodyDef b2RecCType_BODYDEF;
typedef b2ShapeDef b2RecCType_SHAPEDEF;
typedef b2ChainDef b2RecCType_CHAINDEF;
typedef b2DistanceJointDef b2RecCType_DISTANCEJOINTDEF;
typedef b2MotorJointDef b2RecCType_MOTORJOINTDEF;
typedef b2FilterJointDef b2RecCType_FILTERJOINTDEF;
typedef b2PrismaticJointDef b2RecCType_PRISMATICJOINTDEF;
typedef b2RevoluteJointDef b2RecCType_REVOLUTEJOINTDEF;
typedef b2WeldJointDef b2RecCType_WELDJOINTDEF;
typedef b2WheelJointDef b2RecCType_WHEELJOINTDEF;
typedef b2AABB b2RecCType_AABB;
typedef b2QueryFilter b2RecCType_QUERYFILTER;
typedef b2ShapeProxy b2RecCType_SHAPEPROXY;

// Codegen pass 1a: arg structs, generated in recording.c, declared here for call sites.
// These are typedef'd in recording.c before the write helpers, but must be visible
// in body.c and shape.c which use B2_REC. We generate them via the X-macro here.
// IMPORTANT: this block must not expand ARG or B2_REC_OP as functions.
//
// For example, this:
// B2_REC_OP( 0x80, Step, RET_NONE, ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )
// Becomes:
// typedef struct
// {
//   b2RecCType_WORLDID world;
//   b2RecCType_F32 dt;
//   b2RecCType_I32 subStepCount;
// } b2RecArgs_Step;
// Which are the arguments to b2World_Step
#define ARG( TAG, field ) b2RecCType_##TAG field;
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		__VA_ARGS__                                                                                                              \
	} b2RecArgs_##Name;
#include "recording_ops.inl"
#undef B2_REC_OP
#undef ARG

// Low level buffer helpers
void b2RecBufAppend( b2RecBuffer* buf, const void* data, int size );
void b2RecBufFree( b2RecBuffer* buf );

// Write primitives
void b2RecW_U8( b2RecBuffer* buf, uint8_t v );
void b2RecW_U16( b2RecBuffer* buf, uint16_t v );
void b2RecW_U32( b2RecBuffer* buf, uint32_t v );
void b2RecW_U64( b2RecBuffer* buf, uint64_t v );
void b2RecW_I32( b2RecBuffer* buf, int32_t v );
void b2RecW_F32( b2RecBuffer* buf, float v );
void b2RecW_BOOL( b2RecBuffer* buf, bool v );
void b2RecW_VEC2( b2RecBuffer* buf, b2Vec2 v );
void b2RecW_ROT( b2RecBuffer* buf, b2Rot v );
void b2RecW_XF( b2RecBuffer* buf, b2Transform v );
void b2RecW_F64( b2RecBuffer* buf, double v );
// World position and world transform. Two doubles per position in large world mode, two floats
// otherwise so the wire stays byte-identical to VEC2 / XF in the float build.
void b2RecW_POSITION( b2RecBuffer* buf, b2Pos v );
void b2RecW_WORLDXF( b2RecBuffer* buf, b2WorldTransform v );
void b2RecW_WORLDID( b2RecBuffer* buf, b2WorldId v );
void b2RecW_BODYID( b2RecBuffer* buf, b2BodyId v );
void b2RecW_SHAPEID( b2RecBuffer* buf, b2ShapeId v );
void b2RecW_CHAINID( b2RecBuffer* buf, b2ChainId v );
void b2RecW_JOINTID( b2RecBuffer* buf, b2JointId v );
void b2RecW_CIRCLE( b2RecBuffer* buf, b2Circle v );
void b2RecW_CAPSULE( b2RecBuffer* buf, b2Capsule v );
void b2RecW_SEGMENT( b2RecBuffer* buf, b2Segment v );
void b2RecW_POLYGON( b2RecBuffer* buf, b2Polygon v );
void b2RecW_CHAINSEG( b2RecBuffer* buf, b2ChainSegment v );
void b2RecW_FILTER( b2RecBuffer* buf, b2Filter v );
void b2RecW_MATERIAL( b2RecBuffer* buf, b2SurfaceMaterial v );
void b2RecW_MASSDATA( b2RecBuffer* buf, b2MassData v );
void b2RecW_LOCKS( b2RecBuffer* buf, b2MotionLocks v );
void b2RecW_STR( b2RecBuffer* buf, const char* s );
void b2RecW_EXPLOSIONDEF( b2RecBuffer* buf, b2ExplosionDef v );
void b2RecW_BODYDEF( b2RecBuffer* buf, b2BodyDef v );
void b2RecW_SHAPEDEF( b2RecBuffer* buf, b2ShapeDef v );
void b2RecW_CHAINDEF( b2RecBuffer* buf, b2ChainDef v );
void b2RecW_DISTANCEJOINTDEF( b2RecBuffer* buf, b2DistanceJointDef v );
void b2RecW_MOTORJOINTDEF( b2RecBuffer* buf, b2MotorJointDef v );
void b2RecW_FILTERJOINTDEF( b2RecBuffer* buf, b2FilterJointDef v );
void b2RecW_PRISMATICJOINTDEF( b2RecBuffer* buf, b2PrismaticJointDef v );
void b2RecW_REVOLUTEJOINTDEF( b2RecBuffer* buf, b2RevoluteJointDef v );
void b2RecW_WELDJOINTDEF( b2RecBuffer* buf, b2WeldJointDef v );
void b2RecW_WHEELJOINTDEF( b2RecBuffer* buf, b2WheelJointDef v );
void b2RecW_AABB( b2RecBuffer* buf, b2AABB v );
void b2RecW_QUERYFILTER( b2RecBuffer* buf, b2QueryFilter v );
void b2RecW_SHAPEPROXY( b2RecBuffer* buf, b2ShapeProxy v );
void b2RecW_WORLDCASTOUTPUT( b2RecBuffer* buf, b2WorldCastOutput v );
void b2RecW_RAYRESULT( b2RecBuffer* buf, b2RayResult v );
void b2RecW_PLANERESULT( b2RecBuffer* buf, b2PlaneResult v );
void b2RecW_TREESTATS( b2RecBuffer* buf, b2TreeStats v );

// Record framing
void b2RecBeginRecord( b2Recording* rec, uint8_t opcode );
void b2RecEndRecord( b2Recording* rec );

// Per op arg writers (no framing) and full writers (framing plus args), generated from the
// manifest. Create ops reach the arg writer directly so the call site can append the returned
// id inside the same record; void ops reach the full writer through B2_REC.
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	void b2RecWriteArgs_##Name( b2Recording* rec, const b2RecArgs_##Name* a );                                                   \
	void b2RecWrite_##Name( b2Recording* rec, const b2RecArgs_##Name* a );
#include "recording_ops.inl"
#undef B2_REC_OP

// Create ops also need a writer that appends the returned id inside the same record.
// Generated only for ops carrying a RET tag. The id type comes from that tag.
// Recording the returned ids allows verification of deterministic replay.
#define B2_REC_RETDECL_RET_NONE( Name )
#define B2_REC_RETDECL_RET_BODYID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2BodyId id );
#define B2_REC_RETDECL_RET_SHAPEID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2ShapeId id );
#define B2_REC_RETDECL_RET_CHAINID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2ChainId id );
#define B2_REC_RETDECL_RET_JOINTID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2JointId id );
#define B2_REC_OP( op, Name, RET, ... ) B2_REC_RETDECL_##RET( Name )
#include "recording_ops.inl"
#undef B2_REC_OP
#undef B2_REC_RETDECL_RET_NONE
#undef B2_REC_RETDECL_RET_BODYID
#undef B2_REC_RETDECL_RET_SHAPEID
#undef B2_REC_RETDECL_RET_CHAINID
#undef B2_REC_RETDECL_RET_JOINTID

// Record a void op. One branch when recording is off, args built inside the branch
#define B2_REC( world, Name, ... )                                                                                               \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( world )->recording != NULL )                                                                                      \
		{                                                                                                                        \
			b2RecArgs_##Name _a = { __VA_ARGS__ };                                                                               \
			b2RecWrite_##Name( ( world )->recording, &_a );                                                                      \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Record a create op and its returned id in one framed record. The id is appended after
// the args so replay can assert it matches. Place this after the real create call.
#define B2_REC_CREATE( world, Name, id, ... )                                                                                    \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( world )->recording != NULL )                                                                                      \
		{                                                                                                                        \
			b2RecArgs_##Name _ca = { __VA_ARGS__ };                                                                              \
			b2RecWriteRet_##Name( ( world )->recording, &_ca, id );                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// Patch helpers for the query hit-count backfill
int b2RecReserveU32( b2RecBuffer* buf );
void b2RecPatchU32( b2RecBuffer* buf, int offset, uint32_t v );

// Commit a finished query record under the lock. The local buffer is still owned by the caller.
void b2RecCommitRecord( b2Recording* rec, uint8_t opcode, const uint8_t* payload, int payloadSize );

// Per-query writer context: holds user fcn+ctx, the local payload buffer, and the hit counter
typedef struct b2RecQueryWriter
{
	union
	{
		b2OverlapResultFcn* overlapFcn;
		b2CastResultFcn* castFcn;
		b2PlaneResultFcn* planeFcn;
	} userFcn;
	void* userContext;
	b2RecBuffer buf; // per-call local payload, heap-backed
	int countOffset; // offset of the reserved u32 hit-count slot
	uint32_t hitCount;
} b2RecQueryWriter;

void b2RecQueryBegin( b2RecQueryWriter* w, void* context );
void b2RecQueryCommit( b2Recording* rec, uint8_t opcode, b2RecQueryWriter* w );

// Recording trampolines: replace the user fcn pointer so hits are captured before dispatch
bool b2RecOverlapTrampoline( b2ShapeId id, void* ctx );
float b2RecCastTrampoline( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx );
bool b2RecPlaneTrampoline( b2ShapeId id, const b2PlaneResult* plane, void* ctx );

// Lifecycle. Public create/destroy/save/load live in box2d.h; these are the engine-side hooks.
void b2StartRecordingIntoBuffer( b2World* world, b2Recording* recording );
void b2StopRecordingInternal( b2World* world );

// Fold one step's world bounds into the running union the recorder writes out at stop.
void b2RecAccumulateBounds( b2Recording* rec, b2AABB bounds );

// Deterministic hash over all body transforms and velocities.
// Called by both recorder and replayer to verify simulation reproduces exactly.
uint64_t b2HashWorldState( b2World* world );
