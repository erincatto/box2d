// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording_replay.h"

#include "body.h"
#include "physics_world.h"
#include "world_snapshot.h"

#include "box2d/box2d.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

// Keyframe ring tuning. A memory budget caps the snapshots kept; the spacing starts at the min and
// doubles when adding the next keyframe would exceed the budget, so memory stays bounded and seek
// cost grows only once a recording outgrows the budget. The Replay sample exposes both as sliders.
#define B2_REC_KEYFRAME_INTERVAL_DEFAULT 16
#define B2_REC_KEYFRAME_BUDGET_DEFAULT ( 512 * 1024 * 1024 )

// Read primitives

static void b2RecRdrCheck( b2RecReader* rdr, int size )
{
	// 64-bit compare so a corrupt or oversized size can't overflow the cursor add
	if ( size < 0 || (int64_t)rdr->cursor + (int64_t)size > (int64_t)rdr->size )
	{
		rdr->ok = false;
	}
}

static bool b2RecReserveScratch( b2RecReader* rdr, void** data, int* cap, int need, int elemSize );

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

double b2RecR_F64( b2RecReader* rdr )
{
	uint64_t bits = b2RecR_U64( rdr );
	double v;
	memcpy( &v, &bits, 8 );
	return v;
}

b2Pos b2RecR_POSITION( b2RecReader* rdr )
{
	b2Pos p;
#if defined( BOX2D_DOUBLE_PRECISION )
	p.x = b2RecR_F64( rdr );
	p.y = b2RecR_F64( rdr );
#else
	p.x = b2RecR_F32( rdr );
	p.y = b2RecR_F32( rdr );
#endif
	return p;
}

b2WorldTransform b2RecR_WORLDXF( b2RecReader* rdr )
{
	b2WorldTransform t;
	t.p = b2RecR_POSITION( rdr );
	t.q = b2RecR_ROT( rdr );
	return t;
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

b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = (b2BodyType)b2RecR_I32( rdr );
	def.position = b2RecR_POSITION( rdr );
	def.rotation = b2RecR_ROT( rdr );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.angularVelocity = b2RecR_F32( rdr );
	def.linearDamping = b2RecR_F32( rdr );
	def.angularDamping = b2RecR_F32( rdr );
	def.gravityScale = b2RecR_F32( rdr );
	def.sleepThreshold = b2RecR_F32( rdr );

	// b2RecR_STR handles the over-length clamp and skips the full recorded length, so the
	// cursor stays aligned even for names longer than B2_NAME_LENGTH. Valid until the create call.
	def.name = b2RecR_STR( rdr );

	(void)b2RecR_U64( rdr ); // userData (not preserved)
	def.motionLocks = b2RecR_LOCKS( rdr );
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
	def.material = b2RecR_MATERIAL( rdr );
	def.density = b2RecR_F32( rdr );
	def.filter = b2RecR_FILTER( rdr );
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
	if ( b2RecReserveScratch( rdr, (void**)&rdr->chainPoints, &rdr->chainPointCap, count, (int)sizeof( b2Vec2 ) ) == false )
	{
		count = 0; // corrupt count, the read has already failed
	}
	for ( int i = 0; i < count; ++i )
	{
		rdr->chainPoints[i] = b2RecR_VEC2( rdr );
	}
	def.points = count > 0 ? rdr->chainPoints : NULL;
	def.count = count;

	int materialCount = b2RecR_I32( rdr );
	if ( materialCount < 0 )
	{
		materialCount = 0;
	}
	if ( b2RecReserveScratch( rdr, (void**)&rdr->chainMaterials, &rdr->chainMaterialCap, materialCount,
							  (int)sizeof( b2SurfaceMaterial ) ) == false )
	{
		materialCount = 0;
	}
	for ( int i = 0; i < materialCount; ++i )
	{
		rdr->chainMaterials[i] = b2RecR_MATERIAL( rdr );
	}
	def.materials = materialCount > 0 ? rdr->chainMaterials : NULL;
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
	def.position = b2RecR_POSITION( rdr );
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

b2AABB b2RecR_AABB( b2RecReader* rdr )
{
	b2AABB v;
	v.lowerBound = b2RecR_VEC2( rdr );
	v.upperBound = b2RecR_VEC2( rdr );
	return v;
}

b2QueryFilter b2RecR_QUERYFILTER( b2RecReader* rdr )
{
	b2QueryFilter f;
	f.categoryBits = b2RecR_U64( rdr );
	f.maskBits = b2RecR_U64( rdr );
	return f;
}

b2ShapeProxy b2RecR_SHAPEPROXY( b2RecReader* rdr )
{
	b2ShapeProxy p;
	memset( &p, 0, sizeof( p ) );
	int count = b2RecR_I32( rdr );
	if ( count < 0 )
		count = 0;
	if ( count > B2_MAX_POLYGON_VERTICES )
		count = B2_MAX_POLYGON_VERTICES;
	p.count = count;
	for ( int i = 0; i < count; ++i )
	{
		p.points[i] = b2RecR_VEC2( rdr );
	}
	p.radius = b2RecR_F32( rdr );
	return p;
}

b2WorldCastOutput b2RecR_WORLDCASTOUTPUT( b2RecReader* rdr )
{
	b2WorldCastOutput v;
	v.normal = b2RecR_VEC2( rdr );
	v.point = b2RecR_POSITION( rdr );
	v.fraction = b2RecR_F32( rdr );
	v.iterations = b2RecR_I32( rdr );
	v.hit = b2RecR_BOOL( rdr );
	return v;
}

b2RayResult b2RecR_RAYRESULT( b2RecReader* rdr )
{
	b2RayResult v;
	// shapeId keeps the recorded world0; b2RecMakeShapeId is applied at compare time
	v.shapeId = b2RecR_SHAPEID( rdr );
	v.point = b2RecR_POSITION( rdr );
	v.normal = b2RecR_VEC2( rdr );
	v.fraction = b2RecR_F32( rdr );
	v.nodeVisits = b2RecR_I32( rdr );
	v.leafVisits = b2RecR_I32( rdr );
	v.hit = b2RecR_BOOL( rdr );
	return v;
}

b2PlaneResult b2RecR_PLANERESULT( b2RecReader* rdr )
{
	b2PlaneResult v;
	v.plane.normal = b2RecR_VEC2( rdr );
	v.plane.offset = b2RecR_F32( rdr );
	v.point = b2RecR_VEC2( rdr );
	v.hit = b2RecR_BOOL( rdr );
	return v;
}

b2TreeStats b2RecR_TREESTATS( b2RecReader* rdr )
{
	b2TreeStats v;
	v.nodeVisits = b2RecR_I32( rdr );
	v.leafVisits = b2RecR_I32( rdr );
	return v;
}

// Reserve reader scratch for a count taken from an untrusted file. Every recorded element
// consumes at least one byte, so a valid count can never exceed the bytes left in the file.
// Reject anything larger (or negative, or that would overflow the byte size) by failing the read
// rather than allocating wildly. Contents are not preserved; callers overwrite before use.
static bool b2RecReserveScratch( b2RecReader* rdr, void** data, int* cap, int need, int elemSize )
{
	int remaining = rdr->size - rdr->cursor;
	if ( need < 0 || remaining < 0 || need > remaining || need > INT_MAX / elemSize )
	{
		rdr->ok = false;
		return false;
	}
	if ( need <= *cap )
	{
		return true;
	}
	int newCap = need <= INT_MAX / elemSize - 8 ? need + 8 : need;
	if ( *data != NULL )
	{
		b2Free( *data, (size_t)*cap * (size_t)elemSize );
	}
	*data = b2Alloc( (size_t)newCap * (size_t)elemSize );
	*cap = newCap;
	return true;
}

// Overflow-safe growth for the player's accumulating draw arrays. Counts come from the replay
// itself, not the file, so this only guards the byte-size multiply. Preserves keep elements.
static void b2RecGrow( void** data, int* capacity, int need, int keep, int elemSize )
{
	if ( need <= *capacity )
	{
		return;
	}
	int newCap = *capacity == 0 ? 8 : 2 * *capacity;
	if ( newCap < need )
	{
		newCap = need;
	}
	void* grown = b2Alloc( (size_t)newCap * (size_t)elemSize );
	if ( *data != NULL )
	{
		if ( keep > 0 )
		{
			memcpy( grown, *data, (size_t)keep * (size_t)elemSize );
		}
		b2Free( *data, (size_t)*capacity * (size_t)elemSize );
	}
	*data = grown;
	*capacity = newCap;
}

void b2RecEnsureHits( b2RecReader* rdr, int n )
{
	b2RecReserveScratch( rdr, (void**)&rdr->hits, &rdr->hitCap, n, (int)sizeof( b2RecRecordedHit ) );
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

static void b2RecCheckId( b2RecReader* rdr, const char* kind, int gotIndex, unsigned gotGen, int recIndex, unsigned recGen )
{
	if ( gotIndex != recIndex || gotGen != recGen )
	{
		printf( "b2ReplayFile: %s id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", kind, recIndex, recGen, gotIndex,
				gotGen );
		rdr->ok = false;
	}
}

static void b2RecCheckBodyId( b2RecReader* rdr, b2BodyId got, b2BodyId rec )
{
	b2RecCheckId( rdr, "body", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckShapeId( b2RecReader* rdr, b2ShapeId got, b2ShapeId rec )
{
	b2RecCheckId( rdr, "shape", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckChainId( b2RecReader* rdr, b2ChainId got, b2ChainId rec )
{
	b2RecCheckId( rdr, "chain", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckJointId( b2RecReader* rdr, b2JointId got, b2JointId rec )
{
	b2RecCheckId( rdr, "joint", got.index1, got.generation, rec.index1, rec.generation );
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

// Append a created body to the outliner tracking list. Ordinals are creation order and never reused.
static void b2RecTrackBodyCreate( b2RecPlayer* player, b2BodyId id )
{
	b2RecGrow( (void**)&player->bodyIds, &player->bodyIdCap, player->bodyIdCount + 1, player->bodyIdCount,
			   (int)sizeof( b2BodyId ) );
	player->bodyIds[player->bodyIdCount] = id;
	player->bodyIdCount += 1;
}

// Leave a hole so later ordinals do not shift, keeping a stored selection stable across the playthrough
static void b2RecTrackBodyDestroy( b2RecPlayer* player, b2BodyId id )
{
	for ( int i = 0; i < player->bodyIdCount; ++i )
	{
		if ( B2_ID_EQUALS( player->bodyIds[i], id ) )
		{
			player->bodyIds[i] = b2_nullBodyId;
			return;
		}
	}
}

// Snapshot bodies are restored as a struct image and never hit the CreateBody hook the tracker keys
// on, so the seed world must be walked once to populate the outliner list. Slot order is stable.
static void b2RecSeedBodyIds( b2RecPlayer* player )
{
	b2World* world = b2GetWorldFromId( player->rdr.replayWorldId );
	player->bodyIdCount = 0;
	int count = world->bodies.count;
	for ( int i = 0; i < count; ++i )
	{
		if ( world->bodies.data[i].id != i )
		{
			continue; // free slot
		}
		b2RecTrackBodyCreate( player, b2MakeBodyId( world, i ) );
	}
}

static void b2RecDispatch_CreateBody( const b2RecArgs_CreateBody* a, b2RecReader* rdr )
{
	// Recorded id is appended after args (written before b2RecEndRecord)
	b2BodyId recId = b2RecR_BODYID( rdr );
	b2BodyId gotId = b2CreateBody( rdr->replayWorldId, &a->def );
	b2RecCheckBodyId( rdr, gotId, recId );
	if ( rdr->owner != NULL )
	{
		b2RecTrackBodyCreate( rdr->owner, gotId );
	}
}

static void b2RecDispatch_DestroyBody( const b2RecArgs_DestroyBody* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	if ( rdr->owner != NULL )
	{
		b2RecTrackBodyDestroy( rdr->owner, id );
	}
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
	if ( !rdr->ok )
	{
		// A corrupt point/material count left the def degenerate, do not build a chain from it
		return;
	}
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

static void b2RecDispatch_DistanceJointSetSpringForceRange( const b2RecArgs_DistanceJointSetSpringForceRange* a,
															b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringForceRange( b2RecMakeJointId( rdr, a->joint ), a->lowerForce, a->upperForce );
}

static void b2RecDispatch_DistanceJointSetSpringHertz( const b2RecArgs_DistanceJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_DistanceJointSetSpringDampingRatio( const b2RecArgs_DistanceJointSetSpringDampingRatio* a,
															  b2RecReader* rdr )
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

static void b2RecDispatch_MotorJointSetAngularDampingRatio( const b2RecArgs_MotorJointSetAngularDampingRatio* a,
															b2RecReader* rdr )
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

static void b2RecDispatch_PrismaticJointSetSpringDampingRatio( const b2RecArgs_PrismaticJointSetSpringDampingRatio* a,
															   b2RecReader* rdr )
{
	b2PrismaticJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_PrismaticJointSetTargetTranslation( const b2RecArgs_PrismaticJointSetTargetTranslation* a,
															  b2RecReader* rdr )
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

static void b2RecDispatch_RevoluteJointSetSpringDampingRatio( const b2RecArgs_RevoluteJointSetSpringDampingRatio* a,
															  b2RecReader* rdr )
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

// Float bit comparators: compare raw bits so NaN != NaN is handled consistently

static bool b2RecF32Differs( float a, float b )
{
	uint32_t ua, ub;
	memcpy( &ua, &a, 4 );
	memcpy( &ub, &b, 4 );
	return ua != ub;
}

static bool b2RecVec2Differs( b2Vec2 a, b2Vec2 b )
{
	return b2RecF32Differs( a.x, b.x ) || b2RecF32Differs( a.y, b.y );
}

// Per-frame query stash: push a draw record and copy its hits into frameHits.
// Ids in hits[] are already remapped to the replay world by the caller.

static void b2RecGrowFrameQueries( b2RecPlayer* player )
{
	b2RecGrow( (void**)&player->frameQueries, &player->frameQueryCap, player->frameQueryCount + 1, player->frameQueryCount,
			   (int)sizeof( b2RecDrawQuery ) );
}

static void b2RecGrowFrameHits( b2RecPlayer* player, int need )
{
	b2RecGrow( (void**)&player->frameHits, &player->frameHitCap, player->frameHitCount + need, player->frameHitCount,
			   (int)sizeof( b2RecRecordedHit ) );
}

static b2RecDrawQuery* b2RecStashQueryBegin( b2RecPlayer* player, int kind, const b2RecRecordedHit* hits, int hitCount )
{
	b2RecGrowFrameQueries( player );
	b2RecDrawQuery* q = &player->frameQueries[player->frameQueryCount];
	memset( q, 0, sizeof( *q ) );
	q->kind = kind;
	q->hitStart = player->frameHitCount;
	q->hitCount = hitCount;
	b2RecGrowFrameHits( player, hitCount );
	for ( int i = 0; i < hitCount; ++i )
	{
		player->frameHits[player->frameHitCount + i] = hits[i];
	}
	player->frameHitCount += hitCount;
	player->frameQueryCount++;
	return q;
}

// Overlap AABB dispatcher

// Shared context for the query replay trampolines: walks the recorded hits in order and flags
// any divergence from the re-issued query
typedef struct b2RecReplayQueryCtx
{
	b2RecReader* rdr;
	const b2RecRecordedHit* hits;
	int count;
	int cursor;
} b2RecReplayQueryCtx;

static bool b2RecReplayOverlapTrampoline( b2ShapeId id, void* ctx )
{
	b2RecReplayQueryCtx* rc = ctx;
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return false;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnB;
}

static void b2RecDispatch_QueryOverlapAABB( const b2RecArgs_QueryOverlapAABB* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_OverlapAABB( rdr->replayWorldId, a->origin, a->aabb, a->filter, b2RecReplayOverlapTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_OVERLAP_AABB, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->aabb = a->aabb;
	}
}

static void b2RecDispatch_QueryOverlapShape( const b2RecArgs_QueryOverlapShape* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_OverlapShape( rdr->replayWorldId, a->origin, &a->proxy, a->filter, b2RecReplayOverlapTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_OVERLAP_SHAPE, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->proxy = a->proxy;
	}
}

// Cast ray dispatcher

static float b2RecReplayCastTrampoline( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx )
{
	b2RecReplayQueryCtx* rc = ctx;
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return 0.0f;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	// Compare positions through the full width delta, truncating both sides would pass vacuously
	// far from the origin
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation ||
		 b2RecVec2Differs( b2SubPos( point, h->point ), b2Vec2_zero ) || b2RecVec2Differs( normal, h->normal ) ||
		 b2RecF32Differs( fraction, h->fraction ) )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnF;
}

static void b2RecDispatch_QueryCastRay( const b2RecArgs_QueryCastRay* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].point = b2RecR_POSITION( rdr );
		rdr->hits[i].normal = b2RecR_VEC2( rdr );
		rdr->hits[i].fraction = b2RecR_F32( rdr );
		rdr->hits[i].userReturnF = b2RecR_F32( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CastRay( rdr->replayWorldId, a->origin, a->translation, a->filter, b2RecReplayCastTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_RAY, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->translation = a->translation;
	}
}

static void b2RecDispatch_QueryCastShape( const b2RecArgs_QueryCastShape* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].point = b2RecR_POSITION( rdr );
		rdr->hits[i].normal = b2RecR_VEC2( rdr );
		rdr->hits[i].fraction = b2RecR_F32( rdr );
		rdr->hits[i].userReturnF = b2RecR_F32( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CastShape( rdr->replayWorldId, a->origin, &a->proxy, a->translation, a->filter, b2RecReplayCastTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_SHAPE, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->proxy = a->proxy;
		q->translation = a->translation;
	}
}

// CollideMover dispatcher

static bool b2RecReplayPlaneTrampoline( b2ShapeId id, const b2PlaneResult* plane, void* ctx )
{
	b2RecReplayQueryCtx* rc = ctx;
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return true;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation ||
		 b2RecVec2Differs( plane->plane.normal, h->plane.plane.normal ) ||
		 b2RecF32Differs( plane->plane.offset, h->plane.plane.offset ) || b2RecVec2Differs( plane->point, h->plane.point ) ||
		 plane->hit != h->plane.hit )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnB;
}

static void b2RecDispatch_QueryCollideMover( const b2RecArgs_QueryCollideMover* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].plane = b2RecR_PLANERESULT( rdr );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	// CollideMover has no TREESTATS tail (returns void)
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CollideMover( rdr->replayWorldId, a->origin, &a->mover, a->filter, b2RecReplayPlaneTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_COLLIDE_MOVER, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->mover = a->mover;
	}
}

// CastRayClosest dispatcher

static void b2RecDispatch_QueryCastRayClosest( const b2RecArgs_QueryCastRayClosest* a, b2RecReader* rdr )
{
	b2RayResult rec = b2RecR_RAYRESULT( rdr );
	if ( !rdr->ok )
		return;
	b2RayResult got = b2World_CastRayClosest( rdr->replayWorldId, a->origin, a->translation, a->filter );
	if ( got.hit != rec.hit ||
		 ( got.hit && ( got.shapeId.index1 != rec.shapeId.index1 || got.shapeId.generation != rec.shapeId.generation ||
						b2RecVec2Differs( b2SubPos( got.point, rec.point ), b2Vec2_zero ) ||
						b2RecVec2Differs( got.normal, rec.normal ) || b2RecF32Differs( got.fraction, rec.fraction ) ) ) )
	{
		rdr->diverged = true;
	}
	if ( rdr->owner )
	{
		// Stash the closest result as a single pooled hit so the shared draw loop renders its point
		b2RecRecordedHit h = { 0 };
		h.id = b2RecMakeShapeId( rdr, rec.shapeId );
		h.point = rec.point;
		h.normal = rec.normal;
		h.fraction = rec.fraction;
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_RAY_CLOSEST, &h, rec.hit ? 1 : 0 );
		q->filter = a->filter;
		q->origin = a->origin;
		q->translation = a->translation;
	}
}

// CastMover dispatcher

static void b2RecDispatch_QueryCastMover( const b2RecArgs_QueryCastMover* a, b2RecReader* rdr )
{
	float rec = b2RecR_F32( rdr );
	if ( !rdr->ok )
		return;
	float got = b2World_CastMover( rdr->replayWorldId, a->origin, &a->mover, a->translation, a->filter );
	if ( b2RecF32Differs( got, rec ) )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_MOVER, NULL, 0 );
		q->filter = a->filter;
		q->origin = a->origin;
		q->mover = a->mover;
		q->translation = a->translation;
		q->castFraction = rec;
	}
}

// ShapeTestPoint dispatcher

static void b2RecDispatch_ShapeTestPoint( const b2RecArgs_ShapeTestPoint* a, b2RecReader* rdr )
{
	bool rec = b2RecR_BOOL( rdr );
	if ( !rdr->ok )
		return;
	b2ShapeId id = b2RecMakeShapeId( rdr, a->shape );
	bool got = b2Shape_TestPoint( id, a->point );
	if ( got != rec )
		rdr->diverged = true;
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_SHAPE_TEST_POINT, NULL, 0 );
		q->shape = id;
		q->origin = a->point;
		q->boolResult = rec;
	}
}

// ShapeRayCast dispatcher

static void b2RecDispatch_ShapeRayCast( const b2RecArgs_ShapeRayCast* a, b2RecReader* rdr )
{
	b2WorldCastOutput rec = b2RecR_WORLDCASTOUTPUT( rdr );
	if ( !rdr->ok )
		return;
	b2ShapeId id = b2RecMakeShapeId( rdr, a->shape );
	b2WorldCastOutput got = b2Shape_RayCast( id, a->origin, a->translation );
	if ( got.hit != rec.hit || ( got.hit && ( b2RecVec2Differs( got.normal, rec.normal ) ||
											  b2RecVec2Differs( b2SubPos( got.point, rec.point ), b2Vec2_zero ) ||
											  b2RecF32Differs( got.fraction, rec.fraction ) ) ) )
	{
		rdr->diverged = true;
	}
	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_SHAPE_RAY_CAST, NULL, 0 );
		q->shape = id;
		// The ray starts at the origin
		q->origin = a->origin;
		q->translation = a->translation;
		q->castOut = rec;
	}
}

static void b2RecDispatch_StateHash( const b2RecArgs_StateHash* a, b2RecReader* rdr )
{
	b2World* world = b2GetWorldFromId( rdr->replayWorldId );
	uint64_t computed = b2HashWorldState( world );
	if ( computed != a->hash )
	{
		printf( "b2ReplayFile: StateHash mismatch (recorded=0x%llX, computed=0x%llX)\n", (unsigned long long)a->hash,
				(unsigned long long)computed );
		// Non-fatal: reading continues so a viewer can show where divergence begins
		rdr->diverged = true;
	}
}

static void b2RecDispatch_RecordingBounds( const b2RecArgs_RecordingBounds* a, b2RecReader* rdr )
{
	// Primary resolve is the open-time scan, this keeps the value right if it ever moves earlier
	rdr->owner->bounds = a->bounds;
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
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	case op:                                                                                                                     \
	{                                                                                                                            \
		b2RecArgs_##Name a;                                                                                                      \
		memset( &a, 0, sizeof( a ) );                                                                                            \
		__VA_ARGS__ b2RecDispatch_##Name( &a, rdr );                                                                             \
		break;                                                                                                                   \
	}
#include "recording_ops.inl"
#undef B2_REC_OP
#undef ARG
		default:
			printf( "b2ReplayFile: unknown opcode 0x%02X, skipping %u bytes\n", opcode, payloadSize );
			// payloadStart is in bounds, so size - payloadStart is the bytes left to skip over
			if ( payloadSize > (uint32_t)( rdr->size - payloadStart ) )
			{
				rdr->ok = false;
			}
			else
			{
				rdr->cursor = payloadStart + (int)payloadSize;
			}
			break;
	}

	return (int)opcode;
}

// Walk the records once without dispatching to count steps and read the first step's tuning.
// The framing is opcode u8 + payload u24 + payload, so we can skip records blind.
static void b2RecScanFile( b2RecPlayer* player )
{
	const uint8_t* data = player->data;
	int size = player->size;
	int cursor = player->headerEnd;
	int frameCount = 0;
	bool gotStep = false;

	while ( cursor + 4 <= size )
	{
		uint8_t opcode = data[cursor];
		uint32_t payloadSize =
			(uint32_t)data[cursor + 1] | ( (uint32_t)data[cursor + 2] << 8 ) | ( (uint32_t)data[cursor + 3] << 16 );
		int payloadStart = cursor + 4;
		if ( payloadStart + (int)payloadSize > size )
		{
			break;
		}

		if ( opcode == 0x80 ) // Step: [u32 world][f32 dt][i32 subStepCount]
		{
			frameCount += 1;
			if ( gotStep == false && payloadSize >= 12 )
			{
				uint32_t dtBits = (uint32_t)data[payloadStart + 4] | ( (uint32_t)data[payloadStart + 5] << 8 ) |
								  ( (uint32_t)data[payloadStart + 6] << 16 ) | ( (uint32_t)data[payloadStart + 7] << 24 );
				memcpy( &player->recordedDt, &dtBits, 4 );
				player->recordedSubStepCount =
					(int)( (uint32_t)data[payloadStart + 8] | ( (uint32_t)data[payloadStart + 9] << 8 ) |
						   ( (uint32_t)data[payloadStart + 10] << 16 ) | ( (uint32_t)data[payloadStart + 11] << 24 ) );
				gotStep = true;
			}
		}
		else if ( opcode == 0xF2 && payloadSize >= 16 ) // RecordingBounds: [f32 lo.x][lo.y][hi.x][hi.y]
		{
			// Little-endian floats match b2AABB layout, header already rejected big-endian files
			memcpy( &player->bounds, data + payloadStart, 16 );
		}

		cursor = payloadStart + (int)payloadSize;
	}

	player->frameCount = frameCount;
}

b2RecPlayer* b2RecPlayer_Create( const void* data, int size, int workerCount )
{
	if ( data == NULL || size < 32 )
	{
		printf( "b2RecPlayer_Create: recording too small\n" );
		return NULL;
	}

	// Validate the header before copying anything
	b2RecHeader hdr;
	memcpy( &hdr, data, sizeof( b2RecHeader ) );

	if ( hdr.magic != B2_REC_MAGIC )
	{
		printf( "b2RecPlayer_Create: bad magic (got 0x%08X)\n", hdr.magic );
		return NULL;
	}

	if ( hdr.versionMajor != B2_REC_VERSION_MAJOR || hdr.versionMinor != B2_REC_VERSION_MINOR )
	{
		printf( "b2RecPlayer_Create: version mismatch (file=%u.%u, runtime=%u.%u)\n", hdr.versionMajor, hdr.versionMinor,
				B2_REC_VERSION_MAJOR, B2_REC_VERSION_MINOR );
		return NULL;
	}

	if ( hdr.pointerWidth != (uint8_t)sizeof( void* ) )
	{
		printf( "b2RecPlayer_Create: pointer width mismatch (file=%u, runtime=%u)\n", hdr.pointerWidth,
				(unsigned)sizeof( void* ) );
		return NULL;
	}

	if ( hdr.bigEndian != 0 )
	{
		printf( "b2RecPlayer_Create: big-endian recording not supported\n" );
		return NULL;
	}

	// Every recording is snapshot-seeded: the blob sits between the header and the op stream
	if ( hdr.snapshotSize == 0 || hdr.snapshotSize > (uint64_t)( size - 32 ) )
	{
		printf( "b2RecPlayer_Create: missing or oversized snapshot\n" );
		return NULL;
	}
	int headerEnd = 32 + (int)hdr.snapshotSize;

	// Own a private copy of the bytes so the caller can free its buffer right after this call
	uint8_t* copy = b2Alloc( size );
	memcpy( copy, data, (size_t)size );

	b2RecPlayer* player = b2Alloc( (int)sizeof( b2RecPlayer ) );
	player->data = copy;
	player->size = size;
	player->headerEnd = headerEnd;
	player->lengthScale = hdr.lengthScale;
	player->previousLengthScale = b2GetLengthUnitsPerMeter();
	player->frame = 0;
	player->frameCount = 0;
	player->recordedWorkerCount = 0;
	player->recordedDt = 0.0f;
	player->recordedSubStepCount = 0;
	player->bounds = (b2AABB){ 0 };
	player->divergeFrame = -1;
	player->atEnd = false;
	player->rdr.data = copy;
	player->rdr.size = size;
	player->rdr.cursor = headerEnd; // past header and the snapshot blob
	player->rdr.replayWorldId = b2_nullWorldId;
	player->rdr.workerCount = workerCount;
	player->rdr.ok = true;
	player->rdr.diverged = false;
	player->rdr.chainPoints = NULL;
	player->rdr.chainPointCap = 0;
	player->rdr.chainMaterials = NULL;
	player->rdr.chainMaterialCap = 0;
	player->rdr.hits = NULL;
	player->rdr.hitCap = 0;
	player->rdr.owner = player;
	player->frameQueries = NULL;
	player->frameQueryCount = 0;
	player->frameQueryCap = 0;
	player->frameHits = NULL;
	player->frameHitCount = 0;
	player->frameHitCap = 0;
	player->bodyIds = NULL;
	player->bodyIdCount = 0;
	player->bodyIdCap = 0;
	player->frame0Image = NULL;
	player->frame0Size = 0;
	player->frame0BodyIds = NULL;
	player->frame0BodyIdCount = 0;
	player->keyframes = NULL;
	player->keyframeCount = 0;
	player->keyframeCapacity = 0;
	player->keyframeBudget = B2_REC_KEYFRAME_BUDGET_DEFAULT;
	player->keyframeBytes = 0;
	player->keyframeMinInterval = B2_REC_KEYFRAME_INTERVAL_DEFAULT;
	player->keyframeInterval = B2_REC_KEYFRAME_INTERVAL_DEFAULT;
	player->lastKeyframeFrame = 0;

	// Override the global length scale with the recording's so replay reproduces the same constants.
	// This is global engine state and affects the caller's other worlds, so the previous value was
	// captured above and is restored in b2RecPlayer_Destroy.
	if ( hdr.lengthScale > 0.0f )
	{
		b2SetLengthUnitsPerMeter( hdr.lengthScale );
	}

	// Count steps and read the first step's tuning so the viewer can show length and hz up front
	b2RecScanFile( player );

	// Deserialize the seed snapshot to stand up the replay world. The op stream that follows is the
	// hook log. The blob doubles as the frame-0 restore image, owned by the copy we hold.
	player->rdr.replayWorldId = b2CreateWorldFromSnapshot( copy + 32, (int)hdr.snapshotSize, workerCount );
	if ( b2World_IsValid( player->rdr.replayWorldId ) == false )
	{
		printf( "b2RecPlayer_Create: snapshot deserialize failed\n" );
		b2RecPlayer_Destroy( player );
		return NULL;
	}
	player->recordedWorkerCount = workerCount;
	player->frame0Image = copy + 32;
	player->frame0Size = (int)hdr.snapshotSize;

	// The seed snapshot holds the bodies present when recording began; only post-snapshot creates
	// reach the tracker, so seed the outliner list directly from the restored world
	b2RecSeedBodyIds( player );

	// Stash the frame-0 list so a restart or backward scrub rolls the outliner back to it
	player->frame0BodyIdCount = player->bodyIdCount;
	if ( player->bodyIdCount > 0 )
	{
		player->frame0BodyIds = b2Alloc( player->bodyIdCount * (int)sizeof( b2BodyId ) );
		memcpy( player->frame0BodyIds, player->bodyIds, player->bodyIdCount * (int)sizeof( b2BodyId ) );
	}

	return player;
}

// Free a keyframe's heap. image is freed at its allocation size, which over-allocates the logical
// image, so the free size matches the alloc.
static void b2FreeKeyframe( b2RecKeyframe* kf )
{
	b2Free( kf->image, kf->imageCapacity );
	if ( kf->bodyIds != NULL )
	{
		b2Free( kf->bodyIds, kf->bodyIdCount * (int)sizeof( b2BodyId ) );
	}
}

// Capture a restore point for the just-completed frame. rdr.cursor already sits at the next frame's
// Step, so this records the exact resume position next to a full world image plus the outliner and
// divergence state forward stepping would otherwise have to rebuild.
static void b2RecCaptureKeyframe( b2RecPlayer* player )
{
	// Serialize into a buffer the keyframe takes ownership of, so there is no second full-size alloc
	// and copy. The buffer over-allocates, so the budget and free track its capacity, not its size.
	b2World* world = b2GetWorldFromId( player->rdr.replayWorldId );
	b2RecBuffer buf = { 0 };
	b2SerializeWorld( world, &buf );

	size_t bodyBytes = (size_t)player->bodyIdCount * sizeof( b2BodyId );
	size_t newBytes = (size_t)buf.capacity + bodyBytes;

	// Make room under the budget: doubling the spacing drops the off-grid keyframes, roughly halving
	// the bytes, until the new keyframe fits or only it remains. The budget is soft in the corner
	// where a single snapshot already exceeds it.
	while ( player->keyframeCount > 0 && player->keyframeBytes + newBytes > player->keyframeBudget )
	{
		player->keyframeInterval *= 2;
		int kept = 0;
		size_t keptBytes = 0;
		for ( int i = 0; i < player->keyframeCount; ++i )
		{
			b2RecKeyframe* kf = &player->keyframes[i];
			if ( kf->frame % player->keyframeInterval == 0 )
			{
				player->keyframes[kept] = *kf;
				keptBytes += (size_t)kf->imageCapacity + (size_t)kf->bodyIdCount * sizeof( b2BodyId );
				kept += 1;
			}
			else
			{
				b2FreeKeyframe( kf );
			}
		}
		bool progress = kept < player->keyframeCount;
		player->keyframeCount = kept;
		player->keyframeBytes = keptBytes;
		if ( progress == false )
		{
			break;
		}
	}

	b2RecGrow( (void**)&player->keyframes, &player->keyframeCapacity, player->keyframeCount + 1, player->keyframeCount,
			   (int)sizeof( b2RecKeyframe ) );

	b2RecKeyframe* kf = &player->keyframes[player->keyframeCount];
	// Hand the serialized buffer to the keyframe rather than copying it into an exact-size block
	kf->image = buf.data;
	kf->imageSize = buf.size;
	kf->imageCapacity = buf.capacity;

	kf->frame = player->frame;
	kf->cursor = player->rdr.cursor;
	kf->divergeFrame = player->divergeFrame;
	kf->diverged = player->rdr.diverged;
	kf->bodyIdCount = player->bodyIdCount;
	kf->bodyIds = NULL;
	if ( bodyBytes > 0 )
	{
		kf->bodyIds = b2Alloc( bodyBytes );
		memcpy( kf->bodyIds, player->bodyIds, (size_t)bodyBytes );
	}

	player->keyframeBytes += newBytes;
	player->keyframeCount += 1;
	player->lastKeyframeFrame = player->frame;
}

// Restore the world and player state from a keyframe, so a backward seek resumes from it instead of
// frame 0. Mirrors b2RecPlayer_Restart but targets a mid-stream image. b2World_Restore is in place,
// so the replay world id stays stable.
static void b2RecPlayerRestoreKeyframe( b2RecPlayer* player, const b2RecKeyframe* kf )
{
	if ( b2World_Restore( player->rdr.replayWorldId, kf->image, kf->imageSize ) == false )
	{
		player->rdr.ok = false;
		return;
	}
	player->rdr.cursor = kf->cursor;
	player->rdr.ok = true;
	player->rdr.diverged = kf->diverged;
	player->frame = kf->frame;
	player->divergeFrame = kf->divergeFrame;
	player->atEnd = false;

	b2RecGrow( (void**)&player->bodyIds, &player->bodyIdCap, kf->bodyIdCount, 0, (int)sizeof( b2BodyId ) );
	player->bodyIdCount = kf->bodyIdCount;
	if ( kf->bodyIdCount > 0 )
	{
		memcpy( player->bodyIds, kf->bodyIds, kf->bodyIdCount * (int)sizeof( b2BodyId ) );
	}
}

bool b2RecPlayer_StepFrame( b2RecPlayer* player )
{
	if ( player->atEnd )
	{
		return false;
	}

	// Reset per-frame query store before dispatching new records
	player->frameQueryCount = 0;
	player->frameHitCount = 0;

	// Run this frame's Step, then consume the records that trail it (StateHash, queries, any
	// between-frame mutators) up to the next Step. The queries and hash for a frame are recorded
	// after its Step, so grouping them with that Step keeps them paired with the world state they
	// were computed against. Stopping before the next Step is what advances exactly one frame.
	bool stepped = false;
	for ( ;; )
	{
		// Peek the next opcode without consuming it. The next frame's Step ends this frame.
		if ( player->rdr.cursor >= player->rdr.size || player->rdr.ok == false )
		{
			player->atEnd = true;
			return stepped;
		}
		if ( stepped && player->rdr.data[player->rdr.cursor] == 0x80 )
		{
			// Capture a keyframe at the interval. The guard skips frames already covered, so
			// re-stepping a gap during a backward seek never re-captures.
			if ( player->frame > player->lastKeyframeFrame && player->frame % player->keyframeInterval == 0 )
			{
				b2RecCaptureKeyframe( player );
			}
			return true;
		}

		int opcode = b2RecDispatchOne( player );
		if ( opcode < 0 )
		{
			player->atEnd = true;
			return stepped;
		}
		if ( opcode == 0x80 ) // Step
		{
			player->frame += 1;
			stepped = true;
		}
		// Latch the first frame that diverged for the timeline marker
		if ( player->divergeFrame < 0 && player->rdr.diverged )
		{
			player->divergeFrame = player->frame;
		}
	}
}

void b2RecPlayer_Restart( b2RecPlayer* player )
{
	// Restore the frame-0 image in place so the replay world id stays stable across a restart or
	// backward scrub. Stepping resumes at the first Step, which rebuilds the body
	// list deterministically.
	if ( b2World_Restore( player->rdr.replayWorldId, player->frame0Image, player->frame0Size ) == false )
	{
		player->rdr.ok = false;
		return;
	}
	// Stepping resumes at the first Step, which sits right after the header and snapshot blob
	player->rdr.cursor = player->headerEnd;
	player->rdr.ok = true;
	player->rdr.diverged = false;
	player->frame = 0;
	player->divergeFrame = -1;
	player->atEnd = false;

	// Frame 0 is the pre-step snapshot, so it has no recorded queries. Clear the per-frame store so
	// the last stepped frame's queries do not linger on a load or a backward scrub to the start.
	player->frameQueryCount = 0;
	player->frameHitCount = 0;

	// Roll the outliner body list back to its frame-0 contents
	player->bodyIdCount = player->frame0BodyIdCount;
	if ( player->frame0BodyIdCount > 0 )
	{
		memcpy( player->bodyIds, player->frame0BodyIds, player->frame0BodyIdCount * (int)sizeof( b2BodyId ) );
	}
}

b2WorldId b2RecPlayer_GetWorldId( const b2RecPlayer* player )
{
	return player != NULL ? player->rdr.replayWorldId : b2_nullWorldId;
}

int b2RecPlayer_GetFrame( const b2RecPlayer* player )
{
	return player != NULL ? player->frame : 0;
}

void b2RecPlayer_SeekFrame( b2RecPlayer* player, int targetFrame )
{
	if ( player == NULL )
	{
		return;
	}

	if ( targetFrame < 0 )
	{
		targetFrame = 0;
	}

	// Resume from the nearest keyframe strictly below the target when it beats the current cursor.
	// A backward seek must restore since the cursor cannot rewind. A forward seek restores only when
	// a keyframe sits ahead of the cursor, capping a long forward fling at one keyframe interval of
	// replay instead of every intervening frame. Strictly below so the step loop still runs the
	// target frame and regenerates its per-frame query store, body list, and divergence latch
	// exactly as a plain forward replay would.
	const b2RecKeyframe* best = NULL;
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		const b2RecKeyframe* kf = &player->keyframes[i];
		if ( kf->frame < targetFrame && ( best == NULL || kf->frame > best->frame ) )
		{
			best = kf;
		}
	}

	if ( targetFrame < player->frame )
	{
		if ( best != NULL )
		{
			b2RecPlayerRestoreKeyframe( player, best );
		}
		else
		{
			b2RecPlayer_Restart( player );
		}
	}
	else if ( best != NULL && best->frame > player->frame )
	{
		b2RecPlayerRestoreKeyframe( player, best );
	}

	while ( player->frame < targetFrame && b2RecPlayer_StepFrame( player ) )
	{
	}
}

b2RecPlayerInfo b2RecPlayer_GetInfo( const b2RecPlayer* player )
{
	b2RecPlayerInfo info = { 0 };
	if ( player != NULL )
	{
		info.frameCount = player->frameCount;
		info.workerCount = player->recordedWorkerCount;
		info.timeStep = player->recordedDt;
		info.subStepCount = player->recordedSubStepCount;
		info.lengthScale = player->lengthScale;
		info.bounds = player->bounds;
	}
	return info;
}

bool b2RecPlayer_IsAtEnd( const b2RecPlayer* player )
{
	return player != NULL ? player->atEnd : true;
}

bool b2RecPlayer_HasDiverged( const b2RecPlayer* player )
{
	return player != NULL ? player->rdr.diverged : false;
}

int b2RecPlayer_GetDivergeFrame( const b2RecPlayer* player )
{
	return player != NULL ? player->divergeFrame : -1;
}

void b2RecPlayer_SetKeyframePolicy( b2RecPlayer* player, size_t budgetBytes, int minIntervalFrames )
{
	if ( player == NULL )
	{
		return;
	}
	if ( budgetBytes > 0 )
	{
		player->keyframeBudget = budgetBytes;
	}
	if ( minIntervalFrames > 0 )
	{
		player->keyframeMinInterval = minIntervalFrames;
	}

	// Drop the ring so it repopulates under the new policy on the next replay
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		b2FreeKeyframe( &player->keyframes[i] );
	}
	player->keyframeCount = 0;
	player->keyframeBytes = 0;
	player->keyframeInterval = player->keyframeMinInterval;
	player->lastKeyframeFrame = 0;
}

size_t b2RecPlayer_GetKeyframeBudget( const b2RecPlayer* player )
{
	return player != NULL ? player->keyframeBudget : 0;
}

int b2RecPlayer_GetKeyframeMinInterval( const b2RecPlayer* player )
{
	return player != NULL ? player->keyframeMinInterval : 0;
}

int b2RecPlayer_GetKeyframeInterval( const b2RecPlayer* player )
{
	return player != NULL ? player->keyframeInterval : 0;
}

size_t b2RecPlayer_GetKeyframeBytes( const b2RecPlayer* player )
{
	return player != NULL ? player->keyframeBytes : 0;
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
	if ( player->rdr.hits != NULL )
	{
		b2Free( player->rdr.hits, player->rdr.hitCap * (int)sizeof( b2RecRecordedHit ) );
	}
	if ( player->frameQueries != NULL )
	{
		b2Free( player->frameQueries, player->frameQueryCap * (int)sizeof( b2RecDrawQuery ) );
	}
	if ( player->frameHits != NULL )
	{
		b2Free( player->frameHits, player->frameHitCap * (int)sizeof( b2RecRecordedHit ) );
	}
	if ( player->bodyIds != NULL )
	{
		b2Free( player->bodyIds, player->bodyIdCap * (int)sizeof( b2BodyId ) );
	}
	// frame0Image points into the owned data copy, freed above, so it is not freed here
	if ( player->frame0BodyIds != NULL )
	{
		b2Free( player->frame0BodyIds, player->frame0BodyIdCount * (int)sizeof( b2BodyId ) );
	}
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		b2FreeKeyframe( &player->keyframes[i] );
	}
	if ( player->keyframes != NULL )
	{
		b2Free( player->keyframes, (size_t)player->keyframeCapacity * sizeof( b2RecKeyframe ) );
	}

	// Restore the global length scale.
	b2SetLengthUnitsPerMeter( player->previousLengthScale );

	b2Free( player, (int)sizeof( b2RecPlayer ) );
}

// Highlight each reported overlap shape by its AABB. Skip any destroyed since the query,
// per the b2Shape_GetAABB contract that overlap results may contain stale shapes.
static void b2RecDrawHitAABBs( const b2RecPlayer* player, const b2RecDrawQuery* q, b2DebugDraw* draw )
{
	if ( draw->DrawPolygonFcn == NULL )
	{
		return;
	}
	for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
	{
		b2ShapeId id = player->frameHits[hi].id;
		if ( b2Shape_IsValid( id ) == false )
		{
			continue;
		}
		b2AABB b = b2Shape_GetAABB( id );
		b2Vec2 lower = b.lowerBound;
		b2Vec2 upper = b.upperBound;
		b2Vec2 vs[4] = { lower, { upper.x, lower.y }, upper, { lower.x, upper.y } };
		draw->DrawPolygonFcn( b2WorldTransform_identity, vs, 4, b2_colorMagenta, draw->context );
	}
}

void b2RecPlayer_DrawFrameQueries( b2RecPlayer* player, b2DebugDraw* draw, int queryIndex )
{
	if ( player == NULL || draw == NULL )
		return;

	// queryIndex < 0 draws all queries, otherwise just the one selected in the viewer
	for ( int qi = 0; qi < player->frameQueryCount; ++qi )
	{
		if ( queryIndex >= 0 && qi != queryIndex )
		{
			continue;
		}

		const b2RecDrawQuery* q = &player->frameQueries[qi];

		switch ( q->kind )
		{
			case B2_RECQ_CAST_RAY:
			case B2_RECQ_CAST_RAY_CLOSEST:
			{
				// Ray origin to endpoint
				b2Pos origin = q->origin;
				b2Pos end = b2OffsetPos( origin, q->translation );
				if ( draw->DrawLineFcn )
				{
					draw->DrawLineFcn( origin, end, b2_colorYellow, draw->context );
				}
				// Per-hit point + short normal
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					b2Pos point = h->point;
					if ( draw->DrawPointFcn )
					{
						draw->DrawPointFcn( point, 4.0f, b2_colorYellow, draw->context );
					}
					if ( draw->DrawLineFcn )
					{
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->normal ) );
						draw->DrawLineFcn( point, np, b2_colorLightYellow, draw->context );
					}
				}
				break;
			}
			case B2_RECQ_CAST_SHAPE:
			{
				// Shape cast: draw per-hit points along the swept path
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					b2Pos point = h->point;
					if ( draw->DrawPointFcn )
					{
						draw->DrawPointFcn( point, 4.0f, b2_colorSkyBlue, draw->context );
					}
					if ( draw->DrawLineFcn )
					{
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->normal ) );
						draw->DrawLineFcn( point, np, b2_colorLightSkyBlue, draw->context );
					}
				}
				break;
			}
			case B2_RECQ_CAST_MOVER:
			{
				// The mover capsule is relative to the query origin
				if ( draw->DrawSolidCapsuleFcn )
				{
					b2Pos c1 = b2OffsetPos( q->origin, q->mover.center1 );
					b2Pos c2 = b2OffsetPos( q->origin, q->mover.center2 );
					draw->DrawSolidCapsuleFcn( c1, c2, q->mover.radius, b2_colorLightSkyBlue, draw->context );
				}
				break;
			}
			case B2_RECQ_OVERLAP_AABB:
			{
				// The query box is relative to the query origin
				b2Vec2 lower = q->aabb.lowerBound;
				b2Vec2 upper = q->aabb.upperBound;
				b2Vec2 vs[4] = { lower, { upper.x, lower.y }, upper, { lower.x, upper.y } };
				if ( draw->DrawPolygonFcn )
				{
					draw->DrawPolygonFcn( (b2WorldTransform){ q->origin, b2Rot_identity }, vs, 4, b2_colorLimeGreen, draw->context );
				}
				b2RecDrawHitAABBs( player, q, draw );
				break;
			}
			case B2_RECQ_OVERLAP_SHAPE:
			{
				// The proxy points are relative to the query origin
				if ( q->proxy.count == 1 )
				{
					if ( draw->DrawCircleFcn )
					{
						draw->DrawCircleFcn( b2OffsetPos( q->origin, q->proxy.points[0] ), q->proxy.radius, b2_colorLimeGreen,
											 draw->context );
					}
				}
				else if ( q->proxy.count >= 2 && draw->DrawPolygonFcn )
				{
					draw->DrawPolygonFcn( (b2WorldTransform){ q->origin, b2Rot_identity }, q->proxy.points, q->proxy.count,
										  b2_colorLimeGreen,
										  draw->context );
				}
				b2RecDrawHitAABBs( player, q, draw );
				break;
			}
			case B2_RECQ_COLLIDE_MOVER:
			{
				// The mover capsule and the collision planes are relative to the query origin
				if ( draw->DrawSolidCapsuleFcn )
				{
					b2Pos c1 = b2OffsetPos( q->origin, q->mover.center1 );
					b2Pos c2 = b2OffsetPos( q->origin, q->mover.center2 );
					draw->DrawSolidCapsuleFcn( c1, c2, q->mover.radius, b2_colorTan, draw->context );
				}
				// Per-hit plane point and normal
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					if ( h->plane.hit && draw->DrawLineFcn )
					{
						b2Pos point = b2OffsetPos( q->origin, h->plane.point );
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->plane.plane.normal ) );
						draw->DrawLineFcn( point, np, b2_colorOrange, draw->context );
					}
				}
				break;
			}
			case B2_RECQ_SHAPE_TEST_POINT:
			{
				b2HexColor c = q->boolResult ? b2_colorAqua : b2_colorRed;
				if ( draw->DrawPointFcn )
				{
					draw->DrawPointFcn( q->origin, 6.0f, c, draw->context );
				}
				break;
			}
			case B2_RECQ_SHAPE_RAY_CAST:
			{
				b2Pos origin = q->origin;
				b2Pos end = b2OffsetPos( origin, q->translation );
				if ( draw->DrawLineFcn )
				{
					draw->DrawLineFcn( origin, end, b2_colorViolet, draw->context );
				}
				if ( q->castOut.hit && draw->DrawPointFcn )
				{
					draw->DrawPointFcn( q->castOut.point, 4.0f, b2_colorViolet, draw->context );
				}
				break;
			}
			default:
				break;
		}
	}
}

// Public query inspection. The internal b2RecQueryKind values match the public b2RecQueryType, so
// the kind copies across as a plain cast. Pin the first and last kinds to catch enum drift.
_Static_assert( b2_recQueryOverlapAABB == 0 && B2_RECQ_OVERLAP_AABB == 0, "query type enum drift" );
_Static_assert( b2_recQueryShapeRayCast == 8 && B2_RECQ_SHAPE_RAY_CAST == 8, "query type enum drift" );

int b2RecPlayer_GetFrameQueryCount( const b2RecPlayer* player )
{
	return player != NULL ? player->frameQueryCount : 0;
}

b2RecQueryInfo b2RecPlayer_GetFrameQuery( const b2RecPlayer* player, int index )
{
	b2RecQueryInfo info = { 0 };
	if ( player == NULL || index < 0 || index >= player->frameQueryCount )
	{
		return info;
	}

	const b2RecDrawQuery* q = &player->frameQueries[index];
	info.type = (b2RecQueryType)q->kind;
	info.filter = q->filter;
	info.aabb = q->aabb;
	info.origin = q->origin;
	info.translation = q->translation;
	info.shape = q->shape;
	info.hitCount = q->hitCount;
	return info;
}

b2RecQueryHit b2RecPlayer_GetFrameQueryHit( const b2RecPlayer* player, int queryIndex, int hitIndex )
{
	b2RecQueryHit hit = { 0 };
	if ( player == NULL || queryIndex < 0 || queryIndex >= player->frameQueryCount )
	{
		return hit;
	}

	const b2RecDrawQuery* q = &player->frameQueries[queryIndex];
	if ( hitIndex < 0 || hitIndex >= q->hitCount )
	{
		return hit;
	}

	const b2RecRecordedHit* h = &player->frameHits[q->hitStart + hitIndex];
	hit.shape = h->id;
	hit.point = h->point;
	hit.normal = h->normal;
	hit.fraction = h->fraction;
	return hit;
}

int b2RecPlayer_GetBodyCount( const b2RecPlayer* player )
{
	return player != NULL ? player->bodyIdCount : 0;
}

b2BodyId b2RecPlayer_GetBodyId( const b2RecPlayer* player, int index )
{
	if ( player == NULL || index < 0 || index >= player->bodyIdCount )
	{
		return b2_nullBodyId;
	}
	return player->bodyIds[index];
}

bool b2ValidateReplay( const void* data, int size, int workerCount )
{
	b2RecPlayer* player = b2RecPlayer_Create( data, size, workerCount );
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
