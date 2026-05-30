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

b2ChainId b2RecR_CHAINID( b2RecReader* rdr )
{
	return b2LoadChainId( b2RecR_U64( rdr ) );
}

b2JointId b2RecR_JOINTID( b2RecReader* rdr )
{
	return b2LoadJointId( b2RecR_U64( rdr ) );
}

// Read a pointer-free POD blob of the given size into out, advancing the cursor.
// Zeroes out on overrun so a truncated file fails the read check rather than reading garbage.
static void b2RecRdrBlob( b2RecReader* rdr, void* out, int size )
{
	b2RecRdrCheck( rdr, size );
	if ( !rdr->ok )
	{
		memset( out, 0, (size_t)size );
		return;
	}
	memcpy( out, rdr->data + rdr->cursor, (size_t)size );
	rdr->cursor += size;
}

b2Circle b2RecR_CIRCLE( b2RecReader* rdr )
{
	b2Circle c;
	b2RecRdrBlob( rdr, &c, (int)sizeof( c ) );
	return c;
}

b2Capsule b2RecR_CAPSULE( b2RecReader* rdr )
{
	b2Capsule c;
	b2RecRdrBlob( rdr, &c, (int)sizeof( c ) );
	return c;
}

b2Segment b2RecR_SEGMENT( b2RecReader* rdr )
{
	b2Segment s;
	b2RecRdrBlob( rdr, &s, (int)sizeof( s ) );
	return s;
}

b2Polygon b2RecR_POLYGON( b2RecReader* rdr )
{
	b2Polygon p;
	b2RecRdrBlob( rdr, &p, (int)sizeof( p ) );
	return p;
}

b2ChainSegment b2RecR_CHAINSEG( b2RecReader* rdr )
{
	b2ChainSegment cs;
	b2RecRdrBlob( rdr, &cs, (int)sizeof( cs ) );
	return cs;
}

b2Filter b2RecR_FILTER( b2RecReader* rdr )
{
	b2Filter f;
	f.categoryBits = b2RecR_U64( rdr );
	f.maskBits = b2RecR_U64( rdr );
	f.groupIndex = b2RecR_I32( rdr );
	return f;
}

b2SurfaceMaterial b2RecR_MATERIAL( b2RecReader* rdr )
{
	b2SurfaceMaterial m = b2DefaultSurfaceMaterial();
	m.friction = b2RecR_F32( rdr );
	m.restitution = b2RecR_F32( rdr );
	m.rollingResistance = b2RecR_F32( rdr );
	m.tangentSpeed = b2RecR_F32( rdr );
	m.userMaterialId = b2RecR_U64( rdr );
	m.customColor = b2RecR_U32( rdr );
	return m;
}

b2MassData b2RecR_MASSDATA( b2RecReader* rdr )
{
	b2MassData md;
	md.mass = b2RecR_F32( rdr );
	md.center = b2RecR_VEC2( rdr );
	md.rotationalInertia = b2RecR_F32( rdr );
	return md;
}

b2MotionLocks b2RecR_LOCKS( b2RecReader* rdr )
{
	b2MotionLocks locks;
	locks.linearX = b2RecR_BOOL( rdr );
	locks.linearY = b2RecR_BOOL( rdr );
	locks.angularZ = b2RecR_BOOL( rdr );
	return locks;
}

// Returns a pointer into a rotating set of static buffers, valid until the next several
// STR reads. Only used to pass a name straight into a create/setter call during dispatch.
const char* b2RecR_STR( b2RecReader* rdr )
{
	static char s_bufs[4][B2_NAME_LENGTH + 1];
	static int s_next = 0;
	char* buf = s_bufs[s_next];
	s_next = ( s_next + 1 ) & 3;

	uint16_t len = b2RecR_U16( rdr );
	if ( len == 0xFFFFu )
	{
		return NULL;
	}

	int n = (int)len;
	if ( n > B2_NAME_LENGTH )
	{
		n = B2_NAME_LENGTH;
	}
	b2RecRdrCheck( rdr, (int)len );
	if ( rdr->ok && n > 0 )
	{
		memcpy( buf, rdr->data + rdr->cursor, (size_t)n );
	}
	// Skip the full recorded length even if it exceeds the clamp
	rdr->cursor += (int)len;
	buf[n] = '\0';
	return buf;
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

b2ChainDef b2RecR_CHAINDEF( b2RecReader* rdr )
{
	b2ChainDef def = b2DefaultChainDef();
	(void)b2RecR_U64( rdr ); // userData (not preserved)

	int count = b2RecR_I32( rdr );
	if ( count < 0 )
	{
		count = 0;
	}
	if ( count > rdr->chainPointCap )
	{
		if ( rdr->chainPoints != NULL )
		{
			b2Free( rdr->chainPoints, rdr->chainPointCap * (int)sizeof( b2Vec2 ) );
		}
		rdr->chainPoints = b2Alloc( count * (int)sizeof( b2Vec2 ) );
		rdr->chainPointCap = count;
	}
	for ( int i = 0; i < count; ++i )
	{
		rdr->chainPoints[i] = b2RecR_VEC2( rdr );
	}
	def.points = rdr->chainPoints;
	def.count = count;

	int materialCount = b2RecR_I32( rdr );
	if ( materialCount < 0 )
	{
		materialCount = 0;
	}
	if ( materialCount > rdr->chainMaterialCap )
	{
		if ( rdr->chainMaterials != NULL )
		{
			b2Free( rdr->chainMaterials, rdr->chainMaterialCap * (int)sizeof( b2SurfaceMaterial ) );
		}
		rdr->chainMaterials = b2Alloc( materialCount * (int)sizeof( b2SurfaceMaterial ) );
		rdr->chainMaterialCap = materialCount;
	}
	for ( int i = 0; i < materialCount; ++i )
	{
		rdr->chainMaterials[i] = b2RecR_MATERIAL( rdr );
	}
	def.materials = rdr->chainMaterials;
	def.materialCount = materialCount;

	def.filter = b2RecR_FILTER( rdr );
	def.isLoop = b2RecR_BOOL( rdr );
	def.enableSensorEvents = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

b2ExplosionDef b2RecR_EXPLOSIONDEF( b2RecReader* rdr )
{
	b2ExplosionDef def = b2DefaultExplosionDef();
	def.maskBits = b2RecR_U64( rdr );
	def.position = b2RecR_VEC2( rdr );
	def.radius = b2RecR_F32( rdr );
	def.falloff = b2RecR_F32( rdr );
	def.impulsePerLength = b2RecR_F32( rdr );
	return def;
}

// Body ids are read with their recorded world0; the create dispatcher remaps them.
static void b2RecR_JointBase( b2RecReader* rdr, b2JointDef* base )
{
	(void)b2RecR_U64( rdr ); // userData
	base->bodyIdA = b2RecR_BODYID( rdr );
	base->bodyIdB = b2RecR_BODYID( rdr );
	base->localFrameA = b2RecR_XF( rdr );
	base->localFrameB = b2RecR_XF( rdr );
	base->forceThreshold = b2RecR_F32( rdr );
	base->torqueThreshold = b2RecR_F32( rdr );
	base->constraintHertz = b2RecR_F32( rdr );
	base->constraintDampingRatio = b2RecR_F32( rdr );
	base->drawScale = b2RecR_F32( rdr );
	base->collideConnected = b2RecR_BOOL( rdr );
	base->userData = NULL;
}

b2DistanceJointDef b2RecR_DISTANCEJOINTDEF( b2RecReader* rdr )
{
	b2DistanceJointDef def = b2DefaultDistanceJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.length = b2RecR_F32( rdr );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.lowerSpringForce = b2RecR_F32( rdr );
	def.upperSpringForce = b2RecR_F32( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.minLength = b2RecR_F32( rdr );
	def.maxLength = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorForce = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2MotorJointDef b2RecR_MOTORJOINTDEF( b2RecReader* rdr )
{
	b2MotorJointDef def = b2DefaultMotorJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.maxVelocityForce = b2RecR_F32( rdr );
	def.angularVelocity = b2RecR_F32( rdr );
	def.maxVelocityTorque = b2RecR_F32( rdr );
	def.linearHertz = b2RecR_F32( rdr );
	def.linearDampingRatio = b2RecR_F32( rdr );
	def.maxSpringForce = b2RecR_F32( rdr );
	def.angularHertz = b2RecR_F32( rdr );
	def.angularDampingRatio = b2RecR_F32( rdr );
	def.maxSpringTorque = b2RecR_F32( rdr );
	return def;
}

b2FilterJointDef b2RecR_FILTERJOINTDEF( b2RecReader* rdr )
{
	b2FilterJointDef def = b2DefaultFilterJointDef();
	b2RecR_JointBase( rdr, &def.base );
	return def;
}

b2PrismaticJointDef b2RecR_PRISMATICJOINTDEF( b2RecReader* rdr )
{
	b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.targetTranslation = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerTranslation = b2RecR_F32( rdr );
	def.upperTranslation = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorForce = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2RevoluteJointDef b2RecR_REVOLUTEJOINTDEF( b2RecReader* rdr )
{
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.targetAngle = b2RecR_F32( rdr );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerAngle = b2RecR_F32( rdr );
	def.upperAngle = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorTorque = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2WeldJointDef b2RecR_WELDJOINTDEF( b2RecReader* rdr )
{
	b2WeldJointDef def = b2DefaultWeldJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.linearHertz = b2RecR_F32( rdr );
	def.angularHertz = b2RecR_F32( rdr );
	def.linearDampingRatio = b2RecR_F32( rdr );
	def.angularDampingRatio = b2RecR_F32( rdr );
	return def;
}

b2WheelJointDef b2RecR_WHEELJOINTDEF( b2RecReader* rdr )
{
	b2WheelJointDef def = b2DefaultWheelJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerTranslation = b2RecR_F32( rdr );
	def.upperTranslation = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorTorque = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
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

static b2ShapeId b2RecMakeShapeId( b2RecReader* rdr, b2ShapeId recorded )
{
	b2ShapeId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b2ChainId b2RecMakeChainId( b2RecReader* rdr, b2ChainId recorded )
{
	b2ChainId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b2JointId b2RecMakeJointId( b2RecReader* rdr, b2JointId recorded )
{
	b2JointId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

// A create op appends its returned id after the args. Replay compares index1 and generation
// only, since world0 differs between record and replay. A mismatch means structural drift.

static void b2RecCheckBodyId( b2RecReader* rdr, b2BodyId got, b2BodyId rec )
{
	if ( got.index1 != rec.index1 || got.generation != rec.generation )
	{
		printf( "b2ReplayFile: body id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", rec.index1,
		        (unsigned)rec.generation, got.index1, (unsigned)got.generation );
		rdr->ok = false;
	}
}

static void b2RecCheckShapeId( b2RecReader* rdr, b2ShapeId got, b2ShapeId rec )
{
	if ( got.index1 != rec.index1 || got.generation != rec.generation )
	{
		printf( "b2ReplayFile: shape id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", rec.index1,
		        (unsigned)rec.generation, got.index1, (unsigned)got.generation );
		rdr->ok = false;
	}
}

static void b2RecCheckChainId( b2RecReader* rdr, b2ChainId got, b2ChainId rec )
{
	if ( got.index1 != rec.index1 || got.generation != rec.generation )
	{
		printf( "b2ReplayFile: chain id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", rec.index1,
		        (unsigned)rec.generation, got.index1, (unsigned)got.generation );
		rdr->ok = false;
	}
}

static void b2RecCheckJointId( b2RecReader* rdr, b2JointId got, b2JointId rec )
{
	if ( got.index1 != rec.index1 || got.generation != rec.generation )
	{
		printf( "b2ReplayFile: joint id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", rec.index1,
		        (unsigned)rec.generation, got.index1, (unsigned)got.generation );
		rdr->ok = false;
	}
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

static void b2RecDispatch_WorldEnableSleeping( const b2RecArgs_WorldEnableSleeping* a, b2RecReader* rdr )
{
	b2World_EnableSleeping( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldEnableContinuous( const b2RecArgs_WorldEnableContinuous* a, b2RecReader* rdr )
{
	b2World_EnableContinuous( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldSetRestitutionThreshold( const b2RecArgs_WorldSetRestitutionThreshold* a, b2RecReader* rdr )
{
	b2World_SetRestitutionThreshold( rdr->replayWorldId, a->value );
}

static void b2RecDispatch_WorldSetHitEventThreshold( const b2RecArgs_WorldSetHitEventThreshold* a, b2RecReader* rdr )
{
	b2World_SetHitEventThreshold( rdr->replayWorldId, a->value );
}

static void b2RecDispatch_WorldSetGravity( const b2RecArgs_WorldSetGravity* a, b2RecReader* rdr )
{
	b2World_SetGravity( rdr->replayWorldId, a->gravity );
}

static void b2RecDispatch_WorldExplode( const b2RecArgs_WorldExplode* a, b2RecReader* rdr )
{
	b2World_Explode( rdr->replayWorldId, &a->def );
}

static void b2RecDispatch_WorldSetContactTuning( const b2RecArgs_WorldSetContactTuning* a, b2RecReader* rdr )
{
	b2World_SetContactTuning( rdr->replayWorldId, a->hertz, a->dampingRatio, a->pushSpeed );
}

static void b2RecDispatch_WorldSetContactRecycleDistance( const b2RecArgs_WorldSetContactRecycleDistance* a, b2RecReader* rdr )
{
	b2World_SetContactRecycleDistance( rdr->replayWorldId, a->recycleDistance );
}

static void b2RecDispatch_WorldSetMaximumLinearSpeed( const b2RecArgs_WorldSetMaximumLinearSpeed* a, b2RecReader* rdr )
{
	b2World_SetMaximumLinearSpeed( rdr->replayWorldId, a->maximumLinearSpeed );
}

static void b2RecDispatch_WorldEnableWarmStarting( const b2RecArgs_WorldEnableWarmStarting* a, b2RecReader* rdr )
{
	b2World_EnableWarmStarting( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldRebuildStaticTree( const b2RecArgs_WorldRebuildStaticTree* a, b2RecReader* rdr )
{
	(void)a;
	b2World_RebuildStaticTree( rdr->replayWorldId );
}

static void b2RecDispatch_WorldEnableSpeculative( const b2RecArgs_WorldEnableSpeculative* a, b2RecReader* rdr )
{
	b2World_EnableSpeculative( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_CreateBody( const b2RecArgs_CreateBody* a, b2RecReader* rdr )
{
	// Recorded id is appended after args (written before b2RecEndRecord)
	b2BodyId recId = b2RecR_BODYID( rdr );
	b2BodyId gotId = b2CreateBody( rdr->replayWorldId, &a->def );
	b2RecCheckBodyId( rdr, gotId, recId );
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

static void b2RecDispatch_BodySetType( const b2RecArgs_BodySetType* a, b2RecReader* rdr )
{
	b2Body_SetType( b2RecMakeBodyId( rdr, a->body ), (b2BodyType)a->type );
}

static void b2RecDispatch_BodySetName( const b2RecArgs_BodySetName* a, b2RecReader* rdr )
{
	b2Body_SetName( b2RecMakeBodyId( rdr, a->body ), a->name );
}

static void b2RecDispatch_BodySetAngularVelocity( const b2RecArgs_BodySetAngularVelocity* a, b2RecReader* rdr )
{
	b2Body_SetAngularVelocity( b2RecMakeBodyId( rdr, a->body ), a->w );
}

static void b2RecDispatch_BodySetTargetTransform( const b2RecArgs_BodySetTargetTransform* a, b2RecReader* rdr )
{
	b2Body_SetTargetTransform( b2RecMakeBodyId( rdr, a->body ), a->target, a->timeStep, a->wake );
}

static void b2RecDispatch_BodyApplyForce( const b2RecArgs_BodyApplyForce* a, b2RecReader* rdr )
{
	b2Body_ApplyForce( b2RecMakeBodyId( rdr, a->body ), a->force, a->point, a->wake );
}

static void b2RecDispatch_BodyApplyForceToCenter( const b2RecArgs_BodyApplyForceToCenter* a, b2RecReader* rdr )
{
	b2Body_ApplyForceToCenter( b2RecMakeBodyId( rdr, a->body ), a->force, a->wake );
}

static void b2RecDispatch_BodyApplyTorque( const b2RecArgs_BodyApplyTorque* a, b2RecReader* rdr )
{
	b2Body_ApplyTorque( b2RecMakeBodyId( rdr, a->body ), a->torque, a->wake );
}

static void b2RecDispatch_BodyClearForces( const b2RecArgs_BodyClearForces* a, b2RecReader* rdr )
{
	b2Body_ClearForces( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyApplyLinearImpulse( const b2RecArgs_BodyApplyLinearImpulse* a, b2RecReader* rdr )
{
	b2Body_ApplyLinearImpulse( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->point, a->wake );
}

static void b2RecDispatch_BodyApplyLinearImpulseToCenter( const b2RecArgs_BodyApplyLinearImpulseToCenter* a, b2RecReader* rdr )
{
	b2Body_ApplyLinearImpulseToCenter( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b2RecDispatch_BodyApplyAngularImpulse( const b2RecArgs_BodyApplyAngularImpulse* a, b2RecReader* rdr )
{
	b2Body_ApplyAngularImpulse( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b2RecDispatch_BodySetMassData( const b2RecArgs_BodySetMassData* a, b2RecReader* rdr )
{
	b2Body_SetMassData( b2RecMakeBodyId( rdr, a->body ), a->massData );
}

static void b2RecDispatch_BodyApplyMassFromShapes( const b2RecArgs_BodyApplyMassFromShapes* a, b2RecReader* rdr )
{
	b2Body_ApplyMassFromShapes( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodySetLinearDamping( const b2RecArgs_BodySetLinearDamping* a, b2RecReader* rdr )
{
	b2Body_SetLinearDamping( b2RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b2RecDispatch_BodySetAngularDamping( const b2RecArgs_BodySetAngularDamping* a, b2RecReader* rdr )
{
	b2Body_SetAngularDamping( b2RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b2RecDispatch_BodySetGravityScale( const b2RecArgs_BodySetGravityScale* a, b2RecReader* rdr )
{
	b2Body_SetGravityScale( b2RecMakeBodyId( rdr, a->body ), a->scale );
}

static void b2RecDispatch_BodySetAwake( const b2RecArgs_BodySetAwake* a, b2RecReader* rdr )
{
	b2Body_SetAwake( b2RecMakeBodyId( rdr, a->body ), a->awake );
}

static void b2RecDispatch_BodyWakeTouching( const b2RecArgs_BodyWakeTouching* a, b2RecReader* rdr )
{
	b2Body_WakeTouching( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyEnableSleep( const b2RecArgs_BodyEnableSleep* a, b2RecReader* rdr )
{
	b2Body_EnableSleep( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodySetSleepThreshold( const b2RecArgs_BodySetSleepThreshold* a, b2RecReader* rdr )
{
	b2Body_SetSleepThreshold( b2RecMakeBodyId( rdr, a->body ), a->threshold );
}

static void b2RecDispatch_BodyDisable( const b2RecArgs_BodyDisable* a, b2RecReader* rdr )
{
	b2Body_Disable( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyEnable( const b2RecArgs_BodyEnable* a, b2RecReader* rdr )
{
	b2Body_Enable( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodySetMotionLocks( const b2RecArgs_BodySetMotionLocks* a, b2RecReader* rdr )
{
	b2Body_SetMotionLocks( b2RecMakeBodyId( rdr, a->body ), a->locks );
}

static void b2RecDispatch_BodySetBullet( const b2RecArgs_BodySetBullet* a, b2RecReader* rdr )
{
	b2Body_SetBullet( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableContactRecycling( const b2RecArgs_BodyEnableContactRecycling* a, b2RecReader* rdr )
{
	b2Body_EnableContactRecycling( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableContactEvents( const b2RecArgs_BodyEnableContactEvents* a, b2RecReader* rdr )
{
	b2Body_EnableContactEvents( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableHitEvents( const b2RecArgs_BodyEnableHitEvents* a, b2RecReader* rdr )
{
	b2Body_EnableHitEvents( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_CreateCircleShape( const b2RecArgs_CreateCircleShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateCircleShape( bodyId, &a->def, &a->circle );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateCapsuleShape( const b2RecArgs_CreateCapsuleShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateCapsuleShape( bodyId, &a->def, &a->capsule );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateSegmentShape( const b2RecArgs_CreateSegmentShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateSegmentShape( bodyId, &a->def, &a->segment );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreatePolygonShape( const b2RecArgs_CreatePolygonShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreatePolygonShape( bodyId, &a->def, &a->polygon );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateChainSegmentShape( const b2RecArgs_CreateChainSegmentShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateChainSegmentShape( bodyId, &a->def, &a->chainSegment );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_DestroyShape( const b2RecArgs_DestroyShape* a, b2RecReader* rdr )
{
	b2DestroyShape( b2RecMakeShapeId( rdr, a->shape ), a->updateBodyMass );
}

static void b2RecDispatch_ShapeSetDensity( const b2RecArgs_ShapeSetDensity* a, b2RecReader* rdr )
{
	b2Shape_SetDensity( b2RecMakeShapeId( rdr, a->shape ), a->density, a->updateBodyMass );
}

static void b2RecDispatch_ShapeSetFriction( const b2RecArgs_ShapeSetFriction* a, b2RecReader* rdr )
{
	b2Shape_SetFriction( b2RecMakeShapeId( rdr, a->shape ), a->friction );
}

static void b2RecDispatch_ShapeSetRestitution( const b2RecArgs_ShapeSetRestitution* a, b2RecReader* rdr )
{
	b2Shape_SetRestitution( b2RecMakeShapeId( rdr, a->shape ), a->restitution );
}

static void b2RecDispatch_ShapeSetUserMaterial( const b2RecArgs_ShapeSetUserMaterial* a, b2RecReader* rdr )
{
	b2Shape_SetUserMaterial( b2RecMakeShapeId( rdr, a->shape ), a->material );
}

static void b2RecDispatch_ShapeSetSurfaceMaterial( const b2RecArgs_ShapeSetSurfaceMaterial* a, b2RecReader* rdr )
{
	b2Shape_SetSurfaceMaterial( b2RecMakeShapeId( rdr, a->shape ), &a->material );
}

static void b2RecDispatch_ShapeSetFilter( const b2RecArgs_ShapeSetFilter* a, b2RecReader* rdr )
{
	b2Shape_SetFilter( b2RecMakeShapeId( rdr, a->shape ), a->filter );
}

static void b2RecDispatch_ShapeEnableSensorEvents( const b2RecArgs_ShapeEnableSensorEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableSensorEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnableContactEvents( const b2RecArgs_ShapeEnableContactEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableContactEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnablePreSolveEvents( const b2RecArgs_ShapeEnablePreSolveEvents* a, b2RecReader* rdr )
{
	b2Shape_EnablePreSolveEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnableHitEvents( const b2RecArgs_ShapeEnableHitEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableHitEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeSetCircle( const b2RecArgs_ShapeSetCircle* a, b2RecReader* rdr )
{
	b2Shape_SetCircle( b2RecMakeShapeId( rdr, a->shape ), &a->circle );
}

static void b2RecDispatch_ShapeSetCapsule( const b2RecArgs_ShapeSetCapsule* a, b2RecReader* rdr )
{
	b2Shape_SetCapsule( b2RecMakeShapeId( rdr, a->shape ), &a->capsule );
}

static void b2RecDispatch_ShapeSetSegment( const b2RecArgs_ShapeSetSegment* a, b2RecReader* rdr )
{
	b2Shape_SetSegment( b2RecMakeShapeId( rdr, a->shape ), &a->segment );
}

static void b2RecDispatch_ShapeSetPolygon( const b2RecArgs_ShapeSetPolygon* a, b2RecReader* rdr )
{
	b2Shape_SetPolygon( b2RecMakeShapeId( rdr, a->shape ), &a->polygon );
}

static void b2RecDispatch_ShapeSetChainSegment( const b2RecArgs_ShapeSetChainSegment* a, b2RecReader* rdr )
{
	b2Shape_SetChainSegment( b2RecMakeShapeId( rdr, a->shape ), &a->chainSegment );
}

static void b2RecDispatch_ShapeApplyWind( const b2RecArgs_ShapeApplyWind* a, b2RecReader* rdr )
{
	b2Shape_ApplyWind( b2RecMakeShapeId( rdr, a->shape ), a->wind, a->drag, a->lift, a->wake );
}

static void b2RecDispatch_CreateChain( const b2RecArgs_CreateChain* a, b2RecReader* rdr )
{
	b2ChainId recId = b2RecR_CHAINID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ChainId gotId = b2CreateChain( bodyId, &a->def );
	b2RecCheckChainId( rdr, gotId, recId );
}

static void b2RecDispatch_DestroyChain( const b2RecArgs_DestroyChain* a, b2RecReader* rdr )
{
	b2DestroyChain( b2RecMakeChainId( rdr, a->chain ) );
}

static void b2RecDispatch_ChainSetSurfaceMaterial( const b2RecArgs_ChainSetSurfaceMaterial* a, b2RecReader* rdr )
{
	b2Chain_SetSurfaceMaterial( b2RecMakeChainId( rdr, a->chain ), &a->material, a->materialIndex );
}

// Joint create: body ids in the def are remapped to the replay world before the call.

static void b2RecDispatch_CreateDistanceJoint( const b2RecArgs_CreateDistanceJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2DistanceJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateDistanceJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateMotorJoint( const b2RecArgs_CreateMotorJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2MotorJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateMotorJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateFilterJoint( const b2RecArgs_CreateFilterJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2FilterJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateFilterJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreatePrismaticJoint( const b2RecArgs_CreatePrismaticJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2PrismaticJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreatePrismaticJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateRevoluteJoint( const b2RecArgs_CreateRevoluteJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2RevoluteJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateRevoluteJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateWeldJoint( const b2RecArgs_CreateWeldJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2WeldJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateWeldJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateWheelJoint( const b2RecArgs_CreateWheelJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2WheelJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateWheelJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_DestroyJoint( const b2RecArgs_DestroyJoint* a, b2RecReader* rdr )
{
	b2DestroyJoint( b2RecMakeJointId( rdr, a->joint ), a->wakeAttached );
}

// Generic joint mutators

static void b2RecDispatch_JointSetLocalFrameA( const b2RecArgs_JointSetLocalFrameA* a, b2RecReader* rdr )
{
	b2Joint_SetLocalFrameA( b2RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b2RecDispatch_JointSetLocalFrameB( const b2RecArgs_JointSetLocalFrameB* a, b2RecReader* rdr )
{
	b2Joint_SetLocalFrameB( b2RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b2RecDispatch_JointSetCollideConnected( const b2RecArgs_JointSetCollideConnected* a, b2RecReader* rdr )
{
	b2Joint_SetCollideConnected( b2RecMakeJointId( rdr, a->joint ), a->shouldCollide );
}

static void b2RecDispatch_JointWakeBodies( const b2RecArgs_JointWakeBodies* a, b2RecReader* rdr )
{
	b2Joint_WakeBodies( b2RecMakeJointId( rdr, a->joint ) );
}

static void b2RecDispatch_JointSetConstraintTuning( const b2RecArgs_JointSetConstraintTuning* a, b2RecReader* rdr )
{
	b2Joint_SetConstraintTuning( b2RecMakeJointId( rdr, a->joint ), a->hertz, a->dampingRatio );
}

static void b2RecDispatch_JointSetForceThreshold( const b2RecArgs_JointSetForceThreshold* a, b2RecReader* rdr )
{
	b2Joint_SetForceThreshold( b2RecMakeJointId( rdr, a->joint ), a->threshold );
}

static void b2RecDispatch_JointSetTorqueThreshold( const b2RecArgs_JointSetTorqueThreshold* a, b2RecReader* rdr )
{
	b2Joint_SetTorqueThreshold( b2RecMakeJointId( rdr, a->joint ), a->threshold );
}

// Distance joint

static void b2RecDispatch_DistanceJointSetLength( const b2RecArgs_DistanceJointSetLength* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetLength( b2RecMakeJointId( rdr, a->joint ), a->length );
}

static void b2RecDispatch_DistanceJointEnableSpring( const b2RecArgs_DistanceJointEnableSpring* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_DistanceJointSetSpringForceRange( const b2RecArgs_DistanceJointSetSpringForceRange* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringForceRange( b2RecMakeJointId( rdr, a->joint ), a->lowerForce, a->upperForce );
}

static void b2RecDispatch_DistanceJointSetSpringHertz( const b2RecArgs_DistanceJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_DistanceJointSetSpringDampingRatio( const b2RecArgs_DistanceJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_DistanceJointEnableLimit( const b2RecArgs_DistanceJointEnableLimit* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_DistanceJointSetLengthRange( const b2RecArgs_DistanceJointSetLengthRange* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetLengthRange( b2RecMakeJointId( rdr, a->joint ), a->minLength, a->maxLength );
}

static void b2RecDispatch_DistanceJointEnableMotor( const b2RecArgs_DistanceJointEnableMotor* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_DistanceJointSetMotorSpeed( const b2RecArgs_DistanceJointSetMotorSpeed* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_DistanceJointSetMaxMotorForce( const b2RecArgs_DistanceJointSetMaxMotorForce* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetMaxMotorForce( b2RecMakeJointId( rdr, a->joint ), a->force );
}

// Motor joint

static void b2RecDispatch_MotorJointSetLinearVelocity( const b2RecArgs_MotorJointSetLinearVelocity* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearVelocity( b2RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b2RecDispatch_MotorJointSetAngularVelocity( const b2RecArgs_MotorJointSetAngularVelocity* a, b2RecReader* rdr )
{
	b2MotorJoint_SetAngularVelocity( b2RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b2RecDispatch_MotorJointSetMaxVelocityForce( const b2RecArgs_MotorJointSetMaxVelocityForce* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxVelocityForce( b2RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b2RecDispatch_MotorJointSetMaxVelocityTorque( const b2RecArgs_MotorJointSetMaxVelocityTorque* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxVelocityTorque( b2RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

static void b2RecDispatch_MotorJointSetLinearHertz( const b2RecArgs_MotorJointSetLinearHertz* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_MotorJointSetLinearDampingRatio( const b2RecArgs_MotorJointSetLinearDampingRatio* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b2RecDispatch_MotorJointSetAngularHertz( const b2RecArgs_MotorJointSetAngularHertz* a, b2RecReader* rdr )
{
	b2MotorJoint_SetAngularHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_MotorJointSetAngularDampingRatio( const b2RecArgs_MotorJointSetAngularDampingRatio* a, b2RecReader* rdr )
{
	b2MotorJoint_SetAngularDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b2RecDispatch_MotorJointSetMaxSpringForce( const b2RecArgs_MotorJointSetMaxSpringForce* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxSpringForce( b2RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b2RecDispatch_MotorJointSetMaxSpringTorque( const b2RecArgs_MotorJointSetMaxSpringTorque* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxSpringTorque( b2RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

// Prismatic joint

static void b2RecDispatch_PrismaticJointEnableSpring( const b2RecArgs_PrismaticJointEnableSpring* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_PrismaticJointSetSpringHertz( const b2RecArgs_PrismaticJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_PrismaticJointSetSpringDampingRatio( const b2RecArgs_PrismaticJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_PrismaticJointSetTargetTranslation( const b2RecArgs_PrismaticJointSetTargetTranslation* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetTargetTranslation( b2RecMakeJointId( rdr, a->joint ), a->translation );
}

static void b2RecDispatch_PrismaticJointEnableLimit( const b2RecArgs_PrismaticJointEnableLimit* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_PrismaticJointSetLimits( const b2RecArgs_PrismaticJointSetLimits* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_PrismaticJointEnableMotor( const b2RecArgs_PrismaticJointEnableMotor* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_PrismaticJointSetMotorSpeed( const b2RecArgs_PrismaticJointSetMotorSpeed* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_PrismaticJointSetMaxMotorForce( const b2RecArgs_PrismaticJointSetMaxMotorForce* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetMaxMotorForce( b2RecMakeJointId( rdr, a->joint ), a->force );
}

// Revolute joint

static void b2RecDispatch_RevoluteJointEnableSpring( const b2RecArgs_RevoluteJointEnableSpring* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_RevoluteJointSetSpringHertz( const b2RecArgs_RevoluteJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_RevoluteJointSetSpringDampingRatio( const b2RecArgs_RevoluteJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_RevoluteJointSetTargetAngle( const b2RecArgs_RevoluteJointSetTargetAngle* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetTargetAngle( b2RecMakeJointId( rdr, a->joint ), a->angle );
}

static void b2RecDispatch_RevoluteJointEnableLimit( const b2RecArgs_RevoluteJointEnableLimit* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_RevoluteJointSetLimits( const b2RecArgs_RevoluteJointSetLimits* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_RevoluteJointEnableMotor( const b2RecArgs_RevoluteJointEnableMotor* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_RevoluteJointSetMotorSpeed( const b2RecArgs_RevoluteJointSetMotorSpeed* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_RevoluteJointSetMaxMotorTorque( const b2RecArgs_RevoluteJointSetMaxMotorTorque* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetMaxMotorTorque( b2RecMakeJointId( rdr, a->joint ), a->torque );
}

// Weld joint

static void b2RecDispatch_WeldJointSetLinearHertz( const b2RecArgs_WeldJointSetLinearHertz* a, b2RecReader* rdr )
{
	b2WeldJoint_SetLinearHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WeldJointSetLinearDampingRatio( const b2RecArgs_WeldJointSetLinearDampingRatio* a, b2RecReader* rdr )
{
	b2WeldJoint_SetLinearDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_WeldJointSetAngularHertz( const b2RecArgs_WeldJointSetAngularHertz* a, b2RecReader* rdr )
{
	b2WeldJoint_SetAngularHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WeldJointSetAngularDampingRatio( const b2RecArgs_WeldJointSetAngularDampingRatio* a, b2RecReader* rdr )
{
	b2WeldJoint_SetAngularDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

// Wheel joint

static void b2RecDispatch_WheelJointEnableSpring( const b2RecArgs_WheelJointEnableSpring* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_WheelJointSetSpringHertz( const b2RecArgs_WheelJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2WheelJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WheelJointSetSpringDampingRatio( const b2RecArgs_WheelJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2WheelJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_WheelJointEnableLimit( const b2RecArgs_WheelJointEnableLimit* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_WheelJointSetLimits( const b2RecArgs_WheelJointSetLimits* a, b2RecReader* rdr )
{
	b2WheelJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_WheelJointEnableMotor( const b2RecArgs_WheelJointEnableMotor* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_WheelJointSetMotorSpeed( const b2RecArgs_WheelJointSetMotorSpeed* a, b2RecReader* rdr )
{
	b2WheelJoint_SetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_WheelJointSetMaxMotorTorque( const b2RecArgs_WheelJointSetMaxMotorTorque* a, b2RecReader* rdr )
{
	b2WheelJoint_SetMaxMotorTorque( b2RecMakeJointId( rdr, a->joint ), a->torque );
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
#include "recording_ops.inl"
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
	player->rdr.chainPoints = NULL;
	player->rdr.chainPointCap = 0;
	player->rdr.chainMaterials = NULL;
	player->rdr.chainMaterialCap = 0;

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
	if ( player->rdr.chainPoints != NULL )
	{
		b2Free( player->rdr.chainPoints, player->rdr.chainPointCap * (int)sizeof( b2Vec2 ) );
	}
	if ( player->rdr.chainMaterials != NULL )
	{
		b2Free( player->rdr.chainMaterials, player->rdr.chainMaterialCap * (int)sizeof( b2SurfaceMaterial ) );
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
