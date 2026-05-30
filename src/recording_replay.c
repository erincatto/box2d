// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording_replay.h"

#include "physics_world.h"

#include "box2d/box2d.h"

#include <stdio.h>
#include <string.h>

#define B2_REC_MAGIC 0x43523242u

// Read primitives

static void b2RecRdrCheck( b2RecReader* rdr, int size )
{
	if ( rdr->cursor + size > rdr->size )
	{
		rdr->ok = false;
	}
}

uint8_t b2RecR_U8( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 1 );
	if ( !rdr->ok )
	{
		return 0;
	}
	return rdr->data[rdr->cursor++];
}

uint16_t b2RecR_U16( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 2 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint16_t v = (uint16_t)rdr->data[rdr->cursor] | ( (uint16_t)rdr->data[rdr->cursor + 1] << 8 );
	rdr->cursor += 2;
	return v;
}

uint32_t b2RecR_U24( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 3 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 );
	rdr->cursor += 3;
	return v;
}

uint32_t b2RecR_U32( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 4 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint32_t)rdr->data[rdr->cursor + 3] << 24 );
	rdr->cursor += 4;
	return v;
}

uint64_t b2RecR_U64( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 8 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint64_t v = (uint64_t)rdr->data[rdr->cursor] | ( (uint64_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint64_t)rdr->data[rdr->cursor + 3] << 24 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 4] << 32 ) | ( (uint64_t)rdr->data[rdr->cursor + 5] << 40 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 6] << 48 ) | ( (uint64_t)rdr->data[rdr->cursor + 7] << 56 );
	rdr->cursor += 8;
	return v;
}

int32_t b2RecR_I32( b2RecReader* rdr )
{
	return (int32_t)b2RecR_U32( rdr );
}

float b2RecR_F32( b2RecReader* rdr )
{
	uint32_t bits = b2RecR_U32( rdr );
	float v;
	memcpy( &v, &bits, 4 );
	return v;
}

bool b2RecR_BOOL( b2RecReader* rdr )
{
	return b2RecR_U8( rdr ) != 0u;
}

b2Vec2 b2RecR_VEC2( b2RecReader* rdr )
{
	b2Vec2 v;
	v.x = b2RecR_F32( rdr );
	v.y = b2RecR_F32( rdr );
	return v;
}

b2Rot b2RecR_ROT( b2RecReader* rdr )
{
	b2Rot r;
	r.c = b2RecR_F32( rdr );
	r.s = b2RecR_F32( rdr );
	return r;
}

b2Transform b2RecR_XF( b2RecReader* rdr )
{
	b2Transform xf;
	xf.p = b2RecR_VEC2( rdr );
	xf.q = b2RecR_ROT( rdr );
	return xf;
}

b2WorldId b2RecR_WORLDID( b2RecReader* rdr )
{
	return b2LoadWorldId( b2RecR_U32( rdr ) );
}

b2BodyId b2RecR_BODYID( b2RecReader* rdr )
{
	return b2LoadBodyId( b2RecR_U64( rdr ) );
}

b2ShapeId b2RecR_SHAPEID( b2RecReader* rdr )
{
	return b2LoadShapeId( b2RecR_U64( rdr ) );
}

b2Circle b2RecR_CIRCLE( b2RecReader* rdr )
{
	b2Circle c;
	b2RecRdrCheck( rdr, (int)sizeof( b2Circle ) );
	if ( !rdr->ok )
	{
		memset( &c, 0, sizeof( c ) );
		return c;
	}
	memcpy( &c, rdr->data + rdr->cursor, sizeof( b2Circle ) );
	rdr->cursor += (int)sizeof( b2Circle );
	return c;
}

// Def readers: start from b2Default*Def() to get cookie/internalValue, then overlay fields

b2WorldDef b2RecR_WORLDDEF( b2RecReader* rdr )
{
	b2WorldDef def = b2DefaultWorldDef();
	def.gravity = b2RecR_VEC2( rdr );
	def.restitutionThreshold = b2RecR_F32( rdr );
	def.hitEventThreshold = b2RecR_F32( rdr );
	def.contactHertz = b2RecR_F32( rdr );
	def.contactDampingRatio = b2RecR_F32( rdr );
	def.contactSpeed = b2RecR_F32( rdr );
	def.maximumLinearSpeed = b2RecR_F32( rdr );
	def.enableSleep = b2RecR_BOOL( rdr );
	def.enableContinuous = b2RecR_BOOL( rdr );
	def.enableContactSoftening = b2RecR_BOOL( rdr );
	def.workerCount = b2RecR_I32( rdr );
	(void)b2RecR_U64( rdr ); // userData (not preserved)
	def.capacity.staticShapeCount = b2RecR_I32( rdr );
	def.capacity.dynamicShapeCount = b2RecR_I32( rdr );
	def.capacity.staticBodyCount = b2RecR_I32( rdr );
	def.capacity.dynamicBodyCount = b2RecR_I32( rdr );
	def.capacity.contactCount = b2RecR_I32( rdr );
	// pointers/callbacks stay NULL from b2DefaultWorldDef()
	def.recordingPath = NULL;
	def.enqueueTask = NULL;
	def.finishTask = NULL;
	def.userTaskContext = NULL;
	def.frictionCallback = NULL;
	def.restitutionCallback = NULL;
	def.userData = NULL;
	return def;
}

b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = (b2BodyType)b2RecR_I32( rdr );
	def.position = b2RecR_VEC2( rdr );
	def.rotation = b2RecR_ROT( rdr );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.angularVelocity = b2RecR_F32( rdr );
	def.linearDamping = b2RecR_F32( rdr );
	def.angularDamping = b2RecR_F32( rdr );
	def.gravityScale = b2RecR_F32( rdr );
	def.sleepThreshold = b2RecR_F32( rdr );

	// Point def.name at a static scratch buffer, only needed for the b2CreateBody call
	static char s_nameBuf[B2_NAME_LENGTH + 1];
	uint16_t nameLen = b2RecR_U16( rdr );
	if ( nameLen == 0xFFFFu )
	{
		def.name = NULL;
	}
	else
	{
		int len = (int)nameLen;
		if ( len > B2_NAME_LENGTH )
		{
			len = B2_NAME_LENGTH;
		}
		b2RecRdrCheck( rdr, len );
		if ( rdr->ok && len > 0 )
		{
			memcpy( s_nameBuf, rdr->data + rdr->cursor, (size_t)len );
			rdr->cursor += len;
		}
		s_nameBuf[len] = '\0';
		def.name = s_nameBuf;
	}

	(void)b2RecR_U64( rdr ); // userData (not preserved)
	def.motionLocks.linearX = b2RecR_BOOL( rdr );
	def.motionLocks.linearY = b2RecR_BOOL( rdr );
	def.motionLocks.angularZ = b2RecR_BOOL( rdr );
	def.enableSleep = b2RecR_BOOL( rdr );
	def.isAwake = b2RecR_BOOL( rdr );
	def.isBullet = b2RecR_BOOL( rdr );
	def.isEnabled = b2RecR_BOOL( rdr );
	def.allowFastRotation = b2RecR_BOOL( rdr );
	def.enableContactRecycling = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

b2ShapeDef b2RecR_SHAPEDEF( b2RecReader* rdr )
{
	b2ShapeDef def = b2DefaultShapeDef();
	(void)b2RecR_U64( rdr ); // userData (not preserved)
	def.material.friction = b2RecR_F32( rdr );
	def.material.restitution = b2RecR_F32( rdr );
	def.material.rollingResistance = b2RecR_F32( rdr );
	def.material.tangentSpeed = b2RecR_F32( rdr );
	def.material.userMaterialId = b2RecR_U64( rdr );
	def.material.customColor = b2RecR_U32( rdr );
	def.density = b2RecR_F32( rdr );
	def.filter.categoryBits = b2RecR_U64( rdr );
	def.filter.maskBits = b2RecR_U64( rdr );
	def.filter.groupIndex = b2RecR_I32( rdr );
	def.enableCustomFiltering = b2RecR_BOOL( rdr );
	def.isSensor = b2RecR_BOOL( rdr );
	def.enableSensorEvents = b2RecR_BOOL( rdr );
	def.enableContactEvents = b2RecR_BOOL( rdr );
	def.enableHitEvents = b2RecR_BOOL( rdr );
	def.enablePreSolveEvents = b2RecR_BOOL( rdr );
	def.invokeContactCreation = b2RecR_BOOL( rdr );
	def.updateBodyMass = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

// Per op dispatch, the only place real public API names appear
// Body and shape ids have world0 replaced with the replay world's slot index

static b2BodyId b2RecMakeBodyId( b2RecReader* rdr, b2BodyId recorded )
{
	b2BodyId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static void b2RecDispatch_CreateWorld( const b2RecArgs_CreateWorld* a, b2RecReader* rdr )
{
	b2WorldDef def = a->def;
	// Never re-record during replay. Task pointers stay NULL so the internal scheduler is used
	def.recordingPath = NULL;
	def.enqueueTask = NULL;
	def.finishTask = NULL;
	def.userTaskContext = NULL;
	if ( rdr->workerCount > 0 )
	{
		def.workerCount = rdr->workerCount;
	}
	rdr->replayWorldId = b2CreateWorld( &def );
	if ( !b2World_IsValid( rdr->replayWorldId ) )
	{
		printf( "b2ReplayFile: b2CreateWorld failed during replay\n" );
		rdr->ok = false;
	}
}

static void b2RecDispatch_DestroyWorld( const b2RecArgs_DestroyWorld* a, b2RecReader* rdr )
{
	(void)a;
	(void)rdr;
	// The recorded session ended here. The player owns the replay world's lifetime and tears it
	// down in b2RecPlayer_Destroy/Restart, so a viewer can keep drawing the final step. There is
	// one world per recording and this is always the last record, so leaving it alive is safe.
}

static void b2RecDispatch_Step( const b2RecArgs_Step* a, b2RecReader* rdr )
{
	(void)a;
	b2World_Step( rdr->replayWorldId, a->dt, a->subStepCount );
}

static void b2RecDispatch_CreateBody( const b2RecArgs_CreateBody* a, b2RecReader* rdr )
{
	// Recorded id is appended after args (written before b2RecEndRecord)
	b2BodyId recId = b2RecR_BODYID( rdr );
	b2BodyId gotId = b2CreateBody( rdr->replayWorldId, &a->def );
	// Compare index1 and generation only; world0 may differ between record and replay
	if ( gotId.index1 != recId.index1 || gotId.generation != recId.generation )
	{
		printf( "b2ReplayFile: CreateBody id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n",
		        recId.index1, (unsigned)recId.generation, gotId.index1, (unsigned)gotId.generation );
		rdr->ok = false;
	}
}

static void b2RecDispatch_DestroyBody( const b2RecArgs_DestroyBody* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	b2DestroyBody( id );
}

static void b2RecDispatch_BodySetTransform( const b2RecArgs_BodySetTransform* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	b2Body_SetTransform( id, a->position, a->rotation );
}

static void b2RecDispatch_BodySetLinearVelocity( const b2RecArgs_BodySetLinearVelocity* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	b2Body_SetLinearVelocity( id, a->v );
}

static void b2RecDispatch_CreateCircleShape( const b2RecArgs_CreateCircleShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateCircleShape( bodyId, &a->def, &a->circle );
	if ( gotId.index1 != recId.index1 || gotId.generation != recId.generation )
	{
		printf( "b2ReplayFile: CreateCircleShape id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n",
		        recId.index1, (unsigned)recId.generation, gotId.index1, (unsigned)gotId.generation );
		rdr->ok = false;
	}
}

static void b2RecDispatch_StateHash( const b2RecArgs_StateHash* a, b2RecReader* rdr )
{
	b2World* world = b2GetWorldFromId( rdr->replayWorldId );
	uint64_t computed = b2HashWorldState( world );
	if ( computed != a->hash )
	{
		printf( "b2ReplayFile: StateHash mismatch (recorded=0x%llX, computed=0x%llX)\n",
		        (unsigned long long)a->hash, (unsigned long long)computed );
		// Non-fatal: reading continues so a viewer can show where divergence begins
		rdr->diverged = true;
	}
}

// Codegen pass 2 builds the read-and-dispatch switch cases. Each case reads the ARG fields
// into a b2RecArgs_<Name> then dispatches. Create ops read the returned id in their dispatcher.
// Returns the opcode just dispatched, or -1 at end of file or on a fatal read error.

static int b2RecDispatchOne( b2RecPlayer* player )
{
	b2RecReader* rdr = &player->rdr;
	if ( rdr->cursor >= rdr->size || !rdr->ok )
	{
		return -1;
	}

	uint8_t opcode = b2RecR_U8( rdr );
	if ( !rdr->ok )
	{
		return -1;
	}
	uint32_t payloadSize = b2RecR_U24( rdr );
	if ( !rdr->ok )
	{
		return -1;
	}
	int payloadStart = rdr->cursor;

	switch ( opcode )
	{
#define ARG( TAG, field ) a.field = b2RecR_##TAG( rdr );
#define B2_REC_OP( op, Name, RET, ... ) \
	case op: \
	{ \
		b2RecArgs_##Name a; \
		memset( &a, 0, sizeof( a ) ); \
		__VA_ARGS__ b2RecDispatch_##Name( &a, rdr ); \
		break; \
	}
#include "recording_ops.inc"
#undef B2_REC_OP
#undef ARG
		default:
			printf( "b2ReplayFile: unknown opcode 0x%02X, skipping %u bytes\n", opcode, payloadSize );
			rdr->cursor = payloadStart + (int)payloadSize;
			break;
	}

	return (int)opcode;
}

// Dispatch records until the CreateWorld record has produced a valid world.
static void b2RecPumpToWorld( b2RecPlayer* player )
{
	while ( b2World_IsValid( player->rdr.replayWorldId ) == false )
	{
		if ( b2RecDispatchOne( player ) < 0 )
		{
			break;
		}
	}
}

b2RecPlayer* b2RecPlayer_Create( const char* path, int workerCount )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		printf( "b2ReplayFile: cannot open '%s'\n", path );
		return NULL;
	}

	if ( fseek( f, 0, SEEK_END ) != 0 )
	{
		fclose( f );
		return NULL;
	}

	long fileSize = ftell( f );
	if ( fileSize < 0 )
	{
		fclose( f );
		return NULL;
	}
	fseek( f, 0, SEEK_SET );

	uint8_t* data = b2Alloc( (int)fileSize );
	size_t nread = fread( data, 1, (size_t)fileSize, f );
	fclose( f );

	if ( (long)nread != fileSize )
	{
		b2Free( data, (int)fileSize );
		return NULL;
	}

	// Validate header
	if ( fileSize < 32 )
	{
		printf( "b2ReplayFile: file too small\n" );
		b2Free( data, (int)fileSize );
		return NULL;
	}

	b2RecHeader hdr;
	memcpy( &hdr, data, sizeof( b2RecHeader ) );

	if ( hdr.magic != B2_REC_MAGIC )
	{
		printf( "b2ReplayFile: bad magic (got 0x%08X)\n", hdr.magic );
		b2Free( data, (int)fileSize );
		return NULL;
	}

	if ( hdr.versionMajor != 1 )
	{
		printf( "b2ReplayFile: version mismatch (file=%u, runtime=1)\n", hdr.versionMajor );
		b2Free( data, (int)fileSize );
		return NULL;
	}

	if ( hdr.pointerWidth != (uint8_t)sizeof( void* ) )
	{
		printf( "b2ReplayFile: pointer width mismatch (file=%u, runtime=%u)\n", hdr.pointerWidth,
		        (unsigned)sizeof( void* ) );
		b2Free( data, (int)fileSize );
		return NULL;
	}

	if ( hdr.bigEndian != 0 )
	{
		printf( "b2ReplayFile: big-endian recording not supported\n" );
		b2Free( data, (int)fileSize );
		return NULL;
	}

	b2RecPlayer* player = b2Alloc( (int)sizeof( b2RecPlayer ) );
	player->data = data;
	player->size = (int)fileSize;
	player->headerEnd = 32;
	player->frame = 0;
	player->atEnd = false;
	player->rdr.data = data;
	player->rdr.size = (int)fileSize;
	player->rdr.cursor = 32; // past header
	player->rdr.replayWorldId = b2_nullWorldId;
	player->rdr.workerCount = workerCount;
	player->rdr.ok = true;
	player->rdr.diverged = false;

	// The first record is CreateWorld; replay it so the world exists before the first step
	b2RecPumpToWorld( player );
	if ( b2World_IsValid( player->rdr.replayWorldId ) == false )
	{
		printf( "b2ReplayFile: no CreateWorld record\n" );
		b2RecPlayer_Destroy( player );
		return NULL;
	}

	return player;
}

bool b2RecPlayer_StepFrame( b2RecPlayer* player )
{
	if ( player->atEnd )
	{
		return false;
	}

	// Dispatch records until one Step has executed. A leading StateHash from the prior frame
	// is checked here against the current world, before any mutators of the next frame run.
	for ( ;; )
	{
		int opcode = b2RecDispatchOne( player );
		if ( opcode < 0 )
		{
			player->atEnd = true;
			return false;
		}
		if ( opcode == 0x80 ) // Step
		{
			player->frame += 1;
			return true;
		}
	}
}

void b2RecPlayer_Restart( b2RecPlayer* player )
{
	if ( b2World_IsValid( player->rdr.replayWorldId ) )
	{
		b2DestroyWorld( player->rdr.replayWorldId );
	}
	player->rdr.cursor = player->headerEnd;
	player->rdr.replayWorldId = b2_nullWorldId;
	player->rdr.ok = true;
	player->rdr.diverged = false;
	player->frame = 0;
	player->atEnd = false;
	b2RecPumpToWorld( player );
}

b2WorldId b2RecPlayer_GetWorldId( const b2RecPlayer* player )
{
	return player != NULL ? player->rdr.replayWorldId : b2_nullWorldId;
}

int b2RecPlayer_GetFrame( const b2RecPlayer* player )
{
	return player != NULL ? player->frame : 0;
}

bool b2RecPlayer_IsAtEnd( const b2RecPlayer* player )
{
	return player != NULL ? player->atEnd : true;
}

bool b2RecPlayer_HasDiverged( const b2RecPlayer* player )
{
	return player != NULL ? player->rdr.diverged : false;
}

void b2RecPlayer_Destroy( b2RecPlayer* player )
{
	if ( player == NULL )
	{
		return;
	}
	if ( b2World_IsValid( player->rdr.replayWorldId ) )
	{
		b2DestroyWorld( player->rdr.replayWorldId );
	}
	if ( player->data != NULL )
	{
		b2Free( player->data, player->size );
	}
	b2Free( player, (int)sizeof( b2RecPlayer ) );
}

bool b2ValidateReplayFile( const char* path, int workerCount )
{
	b2RecPlayer* player = b2RecPlayer_Create( path, workerCount );
	if ( player == NULL )
	{
		return false;
	}

	while ( b2RecPlayer_StepFrame( player ) )
	{
		if ( player->rdr.diverged )
		{
			break;
		}
	}

	bool ok = player->rdr.ok && player->rdr.diverged == false;
	b2RecPlayer_Destroy( player );
	return ok;
}
