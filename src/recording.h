// SPDX-FileCopyrightText: 2023 Erin Catto
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

typedef struct b2World b2World;

// File header, fixed 32 bytes, little-endian, single-arch-gated
typedef struct b2RecHeader
{
	uint32_t magic;        // 'B2RC' = 0x43523242
	uint16_t versionMajor; // 1
	uint16_t versionMinor; // 0
	uint32_t buildHash;    // 0 for PR1
	uint8_t simdWidth;     // B2_SIMD_WIDTH, informational
	uint8_t pointerWidth;  // sizeof(void*), gates POD-def memcpy
	uint8_t bigEndian;     // 0 on all supported targets
	uint8_t reserved0;
	uint64_t wallClockUnix; // informational
	uint8_t reserved[8];
} b2RecHeader;
_Static_assert( sizeof( b2RecHeader ) == 32, "recording header must be 32 bytes" );

// Append-only write buffer. Flushed to disk on each Step and on Save/Stop.
typedef struct b2RecBuffer
{
	uint8_t* data;
	int capacity;
	int size;
} b2RecBuffer;

typedef struct b2Recording
{
	FILE* file;
	b2RecBuffer buffer;
	int recordStart; // offset of the 3-byte size field for u24 backpatch
} b2Recording;

// C type aliases per TAG, used in codegen arg structs (recording.c only)
typedef bool b2RecCType_BOOL;
typedef int32_t b2RecCType_I32;
typedef uint32_t b2RecCType_U32;
typedef uint64_t b2RecCType_U64;
typedef float b2RecCType_F32;
typedef b2Vec2 b2RecCType_VEC2;
typedef b2Rot b2RecCType_ROT;
typedef b2Transform b2RecCType_XF;
typedef b2WorldId b2RecCType_WORLDID;
typedef b2BodyId b2RecCType_BODYID;
typedef b2ShapeId b2RecCType_SHAPEID;
typedef b2Circle b2RecCType_CIRCLE;
typedef b2WorldDef b2RecCType_WORLDDEF;
typedef b2BodyDef b2RecCType_BODYDEF;
typedef b2ShapeDef b2RecCType_SHAPEDEF;

// Codegen pass 1a: arg structs, generated in recording.c, declared here for call sites.
// These are typedef'd in recording.c before the write helpers, but must be visible
// in body.c and shape.c which use B2_REC. We generate them via the X-macro here.
// IMPORTANT: this block must not expand ARG or B2_REC_OP as functions.
#define ARG( TAG, field ) b2RecCType_##TAG field;
#define B2_REC_OP( op, Name, RET, ... ) typedef struct { __VA_ARGS__ } b2RecArgs_##Name;
#include "recording_ops.inc"
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
void b2RecW_WORLDID( b2RecBuffer* buf, b2WorldId v );
void b2RecW_BODYID( b2RecBuffer* buf, b2BodyId v );
void b2RecW_SHAPEID( b2RecBuffer* buf, b2ShapeId v );
void b2RecW_CIRCLE( b2RecBuffer* buf, b2Circle v );
void b2RecW_STR( b2RecBuffer* buf, const char* s );
void b2RecW_WORLDDEF( b2RecBuffer* buf, b2WorldDef v );
void b2RecW_BODYDEF( b2RecBuffer* buf, b2BodyDef v );
void b2RecW_SHAPEDEF( b2RecBuffer* buf, b2ShapeDef v );

// Record framing
void b2RecBeginRecord( b2Recording* rec, uint8_t opcode );
void b2RecEndRecord( b2Recording* rec );

// Per op arg writers, no framing. Called from body.c and shape.c for create ops
void b2RecWriteArgs_CreateWorld( b2Recording* rec, const b2RecArgs_CreateWorld* a );
void b2RecWriteArgs_DestroyWorld( b2Recording* rec, const b2RecArgs_DestroyWorld* a );
void b2RecWriteArgs_Step( b2Recording* rec, const b2RecArgs_Step* a );
void b2RecWriteArgs_CreateBody( b2Recording* rec, const b2RecArgs_CreateBody* a );
void b2RecWriteArgs_DestroyBody( b2Recording* rec, const b2RecArgs_DestroyBody* a );
void b2RecWriteArgs_BodySetTransform( b2Recording* rec, const b2RecArgs_BodySetTransform* a );
void b2RecWriteArgs_BodySetLinearVelocity( b2Recording* rec, const b2RecArgs_BodySetLinearVelocity* a );
void b2RecWriteArgs_CreateCircleShape( b2Recording* rec, const b2RecArgs_CreateCircleShape* a );
void b2RecWriteArgs_StateHash( b2Recording* rec, const b2RecArgs_StateHash* a );

// Per op full writers, framing plus args. Void call sites reach these through B2_REC
void b2RecWrite_CreateWorld( b2Recording* rec, const b2RecArgs_CreateWorld* a );
void b2RecWrite_DestroyWorld( b2Recording* rec, const b2RecArgs_DestroyWorld* a );
void b2RecWrite_Step( b2Recording* rec, const b2RecArgs_Step* a );
void b2RecWrite_CreateBody( b2Recording* rec, const b2RecArgs_CreateBody* a );
void b2RecWrite_DestroyBody( b2Recording* rec, const b2RecArgs_DestroyBody* a );
void b2RecWrite_BodySetTransform( b2Recording* rec, const b2RecArgs_BodySetTransform* a );
void b2RecWrite_BodySetLinearVelocity( b2Recording* rec, const b2RecArgs_BodySetLinearVelocity* a );
void b2RecWrite_CreateCircleShape( b2Recording* rec, const b2RecArgs_CreateCircleShape* a );
void b2RecWrite_StateHash( b2Recording* rec, const b2RecArgs_StateHash* a );

// Record a void op. One branch when recording is off, args built inside the branch
#define B2_REC( world, Name, ... ) \
	do \
	{ \
		if ( (world)->recording != NULL ) \
		{ \
			b2RecArgs_##Name _a = { __VA_ARGS__ }; \
			b2RecWrite_##Name( (world)->recording, &_a ); \
		} \
	} while ( 0 )

// Lifecycle
void b2StartRecording( b2World* world, const b2WorldDef* def );
void b2FlushRecording( b2Recording* rec );
void b2StopRecordingInternal( b2World* world );

// Deterministic hash over all body transforms and velocities.
// Called by both recorder and replayer to verify simulation reproduces exactly.
uint64_t b2HashWorldState( b2World* world );
