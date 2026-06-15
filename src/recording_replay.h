// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2RecPlayer b2RecPlayer;

// A single recorded callback hit, used both as reader scratch and as the per-frame draw store
typedef struct b2RecRecordedHit
{
	b2ShapeId id;
	b2Pos point;
	b2Vec2 normal;
	float fraction;
	b2PlaneResult plane;
	float userReturnF;
	bool userReturnB;
} b2RecRecordedHit;

// Per-frame draw record for one query call
typedef enum b2RecQueryKind
{
	B2_RECQ_OVERLAP_AABB,
	B2_RECQ_OVERLAP_SHAPE,
	B2_RECQ_CAST_RAY,
	B2_RECQ_CAST_SHAPE,
	B2_RECQ_COLLIDE_MOVER,
	B2_RECQ_CAST_RAY_CLOSEST,
	B2_RECQ_CAST_MOVER,
	B2_RECQ_SHAPE_TEST_POINT,
	B2_RECQ_SHAPE_RAY_CAST
} b2RecQueryKind;

typedef struct b2RecDrawQuery
{
	int kind;
	b2QueryFilter filter;
	b2AABB aabb;
	b2ShapeProxy proxy;
	b2Capsule mover;
	b2Pos origin;
	b2Vec2 translation;
	bool boolResult;
	float castFraction;
	b2WorldCastOutput castOut;
	b2ShapeId shape;
	int hitStart;
	int hitCount;
} b2RecDrawQuery;

// Reader state threaded through the replay loop and all dispatch functions
typedef struct b2RecReader
{
	const uint8_t* data;
	int size;
	int cursor;
	b2WorldId replayWorldId; // world created during replay; valid after CreateWorld record
	int workerCount;		 // 0 = use recorded count
	bool ok;				 // false on read overrun or id mismatch, a fatal stop
	bool diverged;			 // a StateHash failed to reproduce, non-fatal so a viewer can keep playing

	// Scratch for variable-length defs (chain points/materials), grown on demand and
	// freed with the player. Cloned by the create call so only valid during one dispatch.
	b2Vec2* chainPoints;
	int chainPointCap;
	b2SurfaceMaterial* chainMaterials;
	int chainMaterialCap;

	// Scratch for recorded query hits; grown on demand, freed with the player
	b2RecRecordedHit* hits;
	int hitCap;

	b2RecPlayer* owner; // player that owns this reader
} b2RecReader;

// A restore point captured during forward replay, so a backward seek can re-simulate only the gap
// from the nearest keyframe instead of from frame 0. The image is a full b2SerializeWorld blob.
typedef struct b2RecKeyframe
{
	uint8_t* image; // serialized world at the end of this frame
	int imageSize;
	int imageCapacity; // allocation size of image, which over-allocates, so the free size matches
	int frame;		   // frame this restores to, a post-step boundary
	int cursor;		   // op-stream offset where the next frame resumes
	b2BodyId* bodyIds; // outliner list as of this frame
	int bodyIdCount;
	int divergeFrame; // divergence latches as of this frame, so a seek reports the linear path's state
	bool diverged;
} b2RecKeyframe;

// Incremental player. Owns a private copy of the recording bytes and drives replay one step at a time.
struct b2RecPlayer
{
	uint8_t* data; // recording bytes, a private copy owned here
	int size;
	int headerEnd;			   // first payload offset
	float lengthScale;		   // length scale used in the recording
	float previousLengthScale; // global length scale before this player overrode it, restored on destroy
	int frame;				   // steps dispatched so far
	int frameCount;			   // total recorded steps, counted once at open
	int recordedWorkerCount;   // worker count the replay world runs at
	float recordedDt;		   // dt of the first recorded step
	int recordedSubStepCount;  // sub-steps of the first recorded step
	b2AABB bounds;			   // accumulated world bounds, resolved by the open-time scan
	int divergeFrame;		   // first step that diverged, -1 until then
	bool atEnd;				   // a StepFrame ran out of records without reaching a step
	b2RecReader rdr;		   // cursor and replay world, threaded into every dispatcher

	// Per-frame query store, reset at the top of each StepFrame
	b2RecDrawQuery* frameQueries;
	int frameQueryCount;
	int frameQueryCap;
	b2RecRecordedHit* frameHits;
	int frameHitCount;
	int frameHitCap;

	// Live bodies in creation order, tracked from create/destroy ops to drive the viewer outliner.
	// Destroyed slots hold b2_nullBodyId so ordinals stay stable. Rebuilt deterministically on replay.
	b2BodyId* bodyIds;
	int bodyIdCount;
	int bodyIdCap;

	// Frame-0 image used to restart in place, so the replay world id stays stable across a
	// restart or backward scrub. Points into the seed snapshot blob inside the owned copy.
	const uint8_t* frame0Image;
	int frame0Size;
	b2BodyId* frame0BodyIds;
	int frame0BodyIdCount;

	// Keyframe ring for fast backward seeks. Captured in increasing-frame order as the replay plays
	// forward. The spacing doubles and the off-grid keyframes are evicted once the memory budget is
	// hit, so memory stays bounded and seek cost grows only once a recording outgrows the budget.
	b2RecKeyframe* keyframes;
	int keyframeCount;
	int keyframeCapacity;
	size_t keyframeBudget;	 // memory cap in bytes for the kept snapshots
	size_t keyframeBytes;	 // running total of kept snapshot + body-list bytes
	int keyframeMinInterval; // finest spacing in frames
	int keyframeInterval;	 // current spacing, a power-of-two multiple of the min, doubles on eviction
	int lastKeyframeFrame;	 // highest frame captured, guards against re-capture while back-stepping
};

// Read primitives

uint8_t b2RecR_U8( b2RecReader* rdr );
uint16_t b2RecR_U16( b2RecReader* rdr );
uint32_t b2RecR_U24( b2RecReader* rdr );
uint32_t b2RecR_U32( b2RecReader* rdr );
uint64_t b2RecR_U64( b2RecReader* rdr );
int32_t b2RecR_I32( b2RecReader* rdr );
float b2RecR_F32( b2RecReader* rdr );
bool b2RecR_BOOL( b2RecReader* rdr );
b2Vec2 b2RecR_VEC2( b2RecReader* rdr );
b2Rot b2RecR_ROT( b2RecReader* rdr );
b2Transform b2RecR_XF( b2RecReader* rdr );
double b2RecR_F64( b2RecReader* rdr );
b2Pos b2RecR_POSITION( b2RecReader* rdr );
b2WorldTransform b2RecR_WORLDXF( b2RecReader* rdr );
b2WorldId b2RecR_WORLDID( b2RecReader* rdr );
b2BodyId b2RecR_BODYID( b2RecReader* rdr );
b2ShapeId b2RecR_SHAPEID( b2RecReader* rdr );
b2ChainId b2RecR_CHAINID( b2RecReader* rdr );
b2JointId b2RecR_JOINTID( b2RecReader* rdr );
b2Circle b2RecR_CIRCLE( b2RecReader* rdr );
b2Capsule b2RecR_CAPSULE( b2RecReader* rdr );
b2Segment b2RecR_SEGMENT( b2RecReader* rdr );
b2Polygon b2RecR_POLYGON( b2RecReader* rdr );
b2ChainSegment b2RecR_CHAINSEG( b2RecReader* rdr );
b2Filter b2RecR_FILTER( b2RecReader* rdr );
b2SurfaceMaterial b2RecR_MATERIAL( b2RecReader* rdr );
b2MassData b2RecR_MASSDATA( b2RecReader* rdr );
b2MotionLocks b2RecR_LOCKS( b2RecReader* rdr );
const char* b2RecR_STR( b2RecReader* rdr );
b2ExplosionDef b2RecR_EXPLOSIONDEF( b2RecReader* rdr );
b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr );
b2ShapeDef b2RecR_SHAPEDEF( b2RecReader* rdr );
b2ChainDef b2RecR_CHAINDEF( b2RecReader* rdr );
b2DistanceJointDef b2RecR_DISTANCEJOINTDEF( b2RecReader* rdr );
b2MotorJointDef b2RecR_MOTORJOINTDEF( b2RecReader* rdr );
b2FilterJointDef b2RecR_FILTERJOINTDEF( b2RecReader* rdr );
b2PrismaticJointDef b2RecR_PRISMATICJOINTDEF( b2RecReader* rdr );
b2RevoluteJointDef b2RecR_REVOLUTEJOINTDEF( b2RecReader* rdr );
b2WeldJointDef b2RecR_WELDJOINTDEF( b2RecReader* rdr );
b2WheelJointDef b2RecR_WHEELJOINTDEF( b2RecReader* rdr );
b2AABB b2RecR_AABB( b2RecReader* rdr );
b2QueryFilter b2RecR_QUERYFILTER( b2RecReader* rdr );
b2ShapeProxy b2RecR_SHAPEPROXY( b2RecReader* rdr );
b2WorldCastOutput b2RecR_WORLDCASTOUTPUT( b2RecReader* rdr );
b2RayResult b2RecR_RAYRESULT( b2RecReader* rdr );
b2PlaneResult b2RecR_PLANERESULT( b2RecReader* rdr );
b2TreeStats b2RecR_TREESTATS( b2RecReader* rdr );

// Grow the reader's hit scratch to at least n entries
void b2RecEnsureHits( b2RecReader* rdr, int n );
