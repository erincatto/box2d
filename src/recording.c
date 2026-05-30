// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording.h"

#include "body.h"
#include "physics_world.h"

#include <stddef.h>
#include <time.h>

// Magic value 'B2RC' in little-endian: bytes B2, R, C yield 0x43523242
#define B2_REC_MAGIC 0x43523242u

// Buffer helpers

void b2RecBufAppend( b2RecBuffer* buf, const void* data, int size )
{
	if ( size <= 0 )
	{
		return;
	}

	if ( buf->size + size > buf->capacity )
	{
		int newCap = buf->capacity * 2;
		if ( newCap < buf->size + size + 64 )
		{
			newCap = buf->size + size + 64;
		}
		if ( buf->data == NULL )
		{
			buf->data = b2Alloc( newCap );
		}
		else
		{
			buf->data = b2GrowAlloc( buf->data, buf->capacity, newCap );
		}
		buf->capacity = newCap;
	}

	memcpy( buf->data + buf->size, data, (size_t)size );
	buf->size += size;
}

void b2RecBufFree( b2RecBuffer* buf )
{
	if ( buf->data != NULL )
	{
		b2Free( buf->data, buf->capacity );
		buf->data = NULL;
		buf->capacity = 0;
		buf->size = 0;
	}
}

// Write primitives

void b2RecW_U8( b2RecBuffer* buf, uint8_t v )
{
	b2RecBufAppend( buf, &v, 1 );
}

void b2RecW_U16( b2RecBuffer* buf, uint16_t v )
{
	uint8_t b[2] = { (uint8_t)v, (uint8_t)( v >> 8 ) };
	b2RecBufAppend( buf, b, 2 );
}

void b2RecW_U32( b2RecBuffer* buf, uint32_t v )
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)( v >> 8 ), (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ) };
	b2RecBufAppend( buf, b, 4 );
}

void b2RecW_U64( b2RecBuffer* buf, uint64_t v )
{
	uint8_t b[8] = { (uint8_t)v,       (uint8_t)( v >> 8 ),  (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ),
		             (uint8_t)( v >> 32 ), (uint8_t)( v >> 40 ), (uint8_t)( v >> 48 ), (uint8_t)( v >> 56 ) };
	b2RecBufAppend( buf, b, 8 );
}

void b2RecW_I32( b2RecBuffer* buf, int32_t v )
{
	b2RecW_U32( buf, (uint32_t)v );
}

void b2RecW_F32( b2RecBuffer* buf, float v )
{
	uint32_t bits;
	memcpy( &bits, &v, 4 );
	b2RecW_U32( buf, bits );
}

void b2RecW_BOOL( b2RecBuffer* buf, bool v )
{
	b2RecW_U8( buf, v ? 1u : 0u );
}

void b2RecW_VEC2( b2RecBuffer* buf, b2Vec2 v )
{
	b2RecW_F32( buf, v.x );
	b2RecW_F32( buf, v.y );
}

void b2RecW_ROT( b2RecBuffer* buf, b2Rot v )
{
	b2RecW_F32( buf, v.c );
	b2RecW_F32( buf, v.s );
}

void b2RecW_XF( b2RecBuffer* buf, b2Transform v )
{
	b2RecW_VEC2( buf, v.p );
	b2RecW_ROT( buf, v.q );
}

void b2RecW_WORLDID( b2RecBuffer* buf, b2WorldId v )
{
	b2RecW_U32( buf, b2StoreWorldId( v ) );
}

void b2RecW_BODYID( b2RecBuffer* buf, b2BodyId v )
{
	b2RecW_U64( buf, b2StoreBodyId( v ) );
}

void b2RecW_SHAPEID( b2RecBuffer* buf, b2ShapeId v )
{
	b2RecW_U64( buf, b2StoreShapeId( v ) );
}

void b2RecW_CIRCLE( b2RecBuffer* buf, b2Circle v )
{
	// b2Circle is pointer-free POD, pointerWidth in header gates this
	b2RecBufAppend( buf, &v, (int)sizeof( b2Circle ) );
}

void b2RecW_STR( b2RecBuffer* buf, const char* s )
{
	if ( s == NULL )
	{
		b2RecW_U16( buf, 0xFFFFu );
		return;
	}
	int len = 0;
	while ( s[len] != '\0' && len < 65534 )
	{
		len++;
	}
	b2RecW_U16( buf, (uint16_t)len );
	if ( len > 0 )
	{
		b2RecBufAppend( buf, s, len );
	}
}

// Hand-written def helpers. Zero all pointer and cookie fields before serializing
// Readers call b2Default*Def() first to get the cookie, then overwrite fields

void b2RecW_WORLDDEF( b2RecBuffer* buf, b2WorldDef v )
{
	b2RecW_VEC2( buf, v.gravity );
	b2RecW_F32( buf, v.restitutionThreshold );
	b2RecW_F32( buf, v.hitEventThreshold );
	b2RecW_F32( buf, v.contactHertz );
	b2RecW_F32( buf, v.contactDampingRatio );
	b2RecW_F32( buf, v.contactSpeed );
	b2RecW_F32( buf, v.maximumLinearSpeed );
	b2RecW_BOOL( buf, v.enableSleep );
	b2RecW_BOOL( buf, v.enableContinuous );
	b2RecW_BOOL( buf, v.enableContactSoftening );
	b2RecW_I32( buf, v.workerCount );
	// userData written as zero, host pointer not preserved
	b2RecW_U64( buf, 0u );
	// capacity
	b2RecW_I32( buf, v.capacity.staticShapeCount );
	b2RecW_I32( buf, v.capacity.dynamicShapeCount );
	b2RecW_I32( buf, v.capacity.staticBodyCount );
	b2RecW_I32( buf, v.capacity.dynamicBodyCount );
	b2RecW_I32( buf, v.capacity.contactCount );
	// recordingPath, enqueueTask, finishTask, userTaskContext,
	// frictionCallback, restitutionCallback, internalValue: all omitted
}

void b2RecW_BODYDEF( b2RecBuffer* buf, b2BodyDef v )
{
	b2RecW_I32( buf, (int32_t)v.type );
	b2RecW_VEC2( buf, v.position );
	b2RecW_ROT( buf, v.rotation );
	b2RecW_VEC2( buf, v.linearVelocity );
	b2RecW_F32( buf, v.angularVelocity );
	b2RecW_F32( buf, v.linearDamping );
	b2RecW_F32( buf, v.angularDamping );
	b2RecW_F32( buf, v.gravityScale );
	b2RecW_F32( buf, v.sleepThreshold );
	b2RecW_STR( buf, v.name );
	// userData: not preserved
	b2RecW_U64( buf, 0u );
	b2RecW_BOOL( buf, v.motionLocks.linearX );
	b2RecW_BOOL( buf, v.motionLocks.linearY );
	b2RecW_BOOL( buf, v.motionLocks.angularZ );
	b2RecW_BOOL( buf, v.enableSleep );
	b2RecW_BOOL( buf, v.isAwake );
	b2RecW_BOOL( buf, v.isBullet );
	b2RecW_BOOL( buf, v.isEnabled );
	b2RecW_BOOL( buf, v.allowFastRotation );
	b2RecW_BOOL( buf, v.enableContactRecycling );
	// internalValue omitted
}

void b2RecW_SHAPEDEF( b2RecBuffer* buf, b2ShapeDef v )
{
	// userData: not preserved
	b2RecW_U64( buf, 0u );
	// surface material
	b2RecW_F32( buf, v.material.friction );
	b2RecW_F32( buf, v.material.restitution );
	b2RecW_F32( buf, v.material.rollingResistance );
	b2RecW_F32( buf, v.material.tangentSpeed );
	b2RecW_U64( buf, v.material.userMaterialId );
	b2RecW_U32( buf, v.material.customColor );
	b2RecW_F32( buf, v.density );
	// filter
	b2RecW_U64( buf, v.filter.categoryBits );
	b2RecW_U64( buf, v.filter.maskBits );
	b2RecW_I32( buf, v.filter.groupIndex );
	b2RecW_BOOL( buf, v.enableCustomFiltering );
	b2RecW_BOOL( buf, v.isSensor );
	b2RecW_BOOL( buf, v.enableSensorEvents );
	b2RecW_BOOL( buf, v.enableContactEvents );
	b2RecW_BOOL( buf, v.enableHitEvents );
	b2RecW_BOOL( buf, v.enablePreSolveEvents );
	b2RecW_BOOL( buf, v.invokeContactCreation );
	b2RecW_BOOL( buf, v.updateBodyMass );
	// internalValue omitted
}

// Record framing

void b2RecBeginRecord( b2Recording* rec, uint8_t opcode )
{
	b2RecW_U8( &rec->buffer, opcode );
	rec->recordStart = rec->buffer.size;
	// Placeholder 3 bytes for the u24 payload size (backpatched in b2RecEndRecord)
	uint8_t zero[3] = { 0, 0, 0 };
	b2RecBufAppend( &rec->buffer, zero, 3 );
}

void b2RecEndRecord( b2Recording* rec )
{
	int payloadSize = rec->buffer.size - rec->recordStart - 3;
	B2_ASSERT( payloadSize >= 0 && payloadSize < ( 1 << 24 ) );
	uint8_t* p = rec->buffer.data + rec->recordStart;
	p[0] = (uint8_t)payloadSize;
	p[1] = (uint8_t)( payloadSize >> 8 );
	p[2] = (uint8_t)( payloadSize >> 16 );
}

// Codegen pass 1b: arg writers. Each generated function writes its struct fields to the buffer

#define ARG( TAG, field ) b2RecW_##TAG( &rec->buffer, a->field );
#define B2_REC_OP( op, Name, RET, ... ) \
	void b2RecWriteArgs_##Name( b2Recording* rec, const b2RecArgs_##Name* a ) \
	{ \
		__VA_ARGS__ \
	}
#include "recording_ops.inc"
#undef B2_REC_OP
#undef ARG

// Codegen: full writers wrapping begin, arg writer, end

#define B2_REC_OP( op, Name, RET, ... ) \
	void b2RecWrite_##Name( b2Recording* rec, const b2RecArgs_##Name* a ) \
	{ \
		b2RecBeginRecord( rec, (uint8_t)( op ) ); \
		b2RecWriteArgs_##Name( rec, a ); \
		b2RecEndRecord( rec ); \
	}
#include "recording_ops.inc"
#undef B2_REC_OP

// Lifecycle

void b2StartRecording( b2World* world, const b2WorldDef* def )
{
	b2Recording* rec = b2Alloc( (int)sizeof( b2Recording ) );
	*rec = (b2Recording){ 0 };

	// Open for read+write so b2World_SaveRecording can copy the file while it stays open
	rec->file = fopen( def->recordingPath, "w+b" );
	if ( rec->file == NULL )
	{
		b2Free( rec, (int)sizeof( b2Recording ) );
		return;
	}

	int initCap = 65536;
	rec->buffer.data = b2Alloc( initCap );
	rec->buffer.capacity = initCap;
	rec->buffer.size = 0;

	b2RecHeader hdr;
	memset( &hdr, 0, sizeof( hdr ) );
	hdr.magic = B2_REC_MAGIC;
	hdr.versionMajor = 1;
	hdr.versionMinor = 0;
	hdr.buildHash = 0;
	hdr.simdWidth = (uint8_t)B2_SIMD_WIDTH;
	hdr.pointerWidth = (uint8_t)sizeof( void* );
	hdr.bigEndian = 0;
	hdr.wallClockUnix = (uint64_t)time( NULL );
	b2RecBufAppend( &rec->buffer, &hdr, (int)sizeof( hdr ) );

	world->recording = rec;

	// Write the CreateWorld record, first record in file, self-describes the world
	b2RecArgs_CreateWorld a = { *def };
	b2RecWrite_CreateWorld( rec, &a );
}

void b2FlushRecording( b2Recording* rec )
{
	if ( rec->buffer.size > 0 )
	{
		fwrite( rec->buffer.data, 1, (size_t)rec->buffer.size, rec->file );
		rec->buffer.size = 0;
	}
}

void b2StopRecordingInternal( b2World* world )
{
	if ( world->recording == NULL )
	{
		return;
	}

	b2Recording* rec = world->recording;
	world->recording = NULL;

	// Write DestroyWorld so the file is self-contained
	b2WorldId wid = { (uint16_t)( world->worldId + 1 ), world->generation };
	b2RecArgs_DestroyWorld a = { wid };
	b2RecWrite_DestroyWorld( rec, &a );

	b2FlushRecording( rec );
	fclose( rec->file );
	b2RecBufFree( &rec->buffer );
	b2Free( rec, (int)sizeof( b2Recording ) );
}

// Deterministic over worker count: walk the bodies sparse array in index order, not
// solver set order. Free slots have body->id == B2_NULL_INDEX, live slots have body->id == i

uint64_t b2HashWorldState( b2World* world )
{
	// FNV-1a 64-bit
	uint64_t hash = 14695981039346656037ull;
	const uint64_t prime = 1099511628211ull;

	int bodyCount = world->bodies.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			// Free or never-used slot
			continue;
		}

		b2BodySim* sim = b2GetBodySim( world, body );

		uint32_t bits;

#define B2_HASH_FLOAT( f ) \
	memcpy( &bits, &( f ), 4 ); \
	hash = ( hash ^ (uint64_t)bits ) * prime;

		B2_HASH_FLOAT( sim->transform.p.x )
		B2_HASH_FLOAT( sim->transform.p.y )
		B2_HASH_FLOAT( sim->transform.q.c )
		B2_HASH_FLOAT( sim->transform.q.s )

		b2BodyState* state = b2GetBodyState( world, body );
		if ( state != NULL )
		{
			B2_HASH_FLOAT( state->linearVelocity.x )
			B2_HASH_FLOAT( state->linearVelocity.y )
			B2_HASH_FLOAT( state->angularVelocity )
		}

#undef B2_HASH_FLOAT
	}

	return hash;
}
