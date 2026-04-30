// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "bitset.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "container.h"
#include "id_pool.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"

#include "box2d/types.h"

b2DeclareArray( b2BodyMoveEvent );
b2DeclareArray( b2ContactBeginTouchEvent );
b2DeclareArray( b2ContactEndTouchEvent );
b2DeclareArray( b2ContactHitEvent );
b2DeclareArray( b2JointEvent );
b2DeclareArray( b2SensorBeginTouchEvent );
b2DeclareArray( b2SensorEndTouchEvent );
b2DeclareArray( b2TaskContext );

// Per thread task storage
typedef struct b2TaskContext
{
	// Collect per thread sensor continuous hit events.
	b2Array(b2SensorHit) sensorHits;

	// These bits align with the contact id capacity and signal a change in contact status
	b2BitSet contactStateBitSet;

	// These bits align with the contact id capacity and signal a hit event.
	b2BitSet hitEventBitSet;

	// Fast-path flag: true when this worker set at least one bit in hitEventBitSet this step.
	bool hasHitEvents;

	// These bits align with the joint id capacity and signal a change in contact status
	b2BitSet jointStateBitSet;

	// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
	// that is very large when there are many static shapes.
	b2BitSet enlargedSimBitSet;

	// Used to put islands to sleep
	b2BitSet awakeIslandBitSet;

	// Per worker split island candidate
	float splitSleepTime;
	int splitIslandId;

	// Number of contacts recycled this step (collide pass).
	int recycledContactCount;

} b2TaskContext;

// The world struct manages all physics entities, dynamic simulation,  and asynchronous queries.
// The world also contains efficient memory management facilities.
typedef struct b2World
{
	b2Stack stack;
	b2BroadPhase broadPhase;
	b2ConstraintGraph constraintGraph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2Body.
	b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	b2Array( b2Body ) bodies;

	// Provides free list for solver sets.
	b2IdPool solverSetIdPool;

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	b2Array( b2SolverSet ) solverSets;

	// Used to create stable ids for joints
	b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	b2Array( b2Joint ) joints;

	// Used to create stable ids for contacts
	b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	b2Array( b2Contact ) contacts;

	// Used to create stable ids for islands
	b2IdPool islandIdPool;

	// Persistent islands
	b2Array( b2Island ) islands;

	b2IdPool shapeIdPool;
	b2IdPool chainIdPool;

	// These are sparse arrays that point into the pools above
	b2Array( b2Shape ) shapes;
	b2Array( b2ChainShape ) chainShapes;

	// This is a dense array of sensor data.
	b2Array( b2Sensor ) sensors;

	// Per thread storage
	b2Array( b2TaskContext ) taskContexts;
	b2Array( b2SensorTaskContext ) sensorTaskContexts;

	b2Array( b2BodyMoveEvent ) bodyMoveEvents;
	b2Array( b2SensorBeginTouchEvent ) sensorBeginEvents;
	b2Array( b2ContactBeginTouchEvent ) contactBeginEvents;

	// End events are double buffered so that the user doesn't need to flush events
	b2Array( b2SensorEndTouchEvent ) sensorEndEvents[2];
	b2Array( b2ContactEndTouchEvent ) contactEndEvents[2];
	int endEventArrayIndex;

	b2Array( b2ContactHitEvent ) contactHitEvents;
	b2Array( b2JointEvent ) jointEvents;

	// todo consider deferred waking and impulses to make it possible
	// to apply forces and impulses from multiple threads
	// impulses must be deferred because sleeping bodies have no velocity state
	// Problems:
	// - multiple forces applied to the same body from multiple threads
	// Deferred wake
	// b2BitSet bodyWakeSet;
	// b2ImpulseArray deferredImpulses;

	// Used to track debug draw
	b2BitSet debugBodySet;
	b2BitSet debugJointSet;
	b2BitSet debugContactSet;
	b2BitSet debugIslandSet;

	// Id that is incremented every time step
	uint64_t stepIndex;

	// Identify islands for splitting as follows:
	// - I want to split islands so smaller islands can sleep
	// - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
	//   if it has removed constraints
	// - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
	// - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
	// - if no bodies want to sleep then there is no reason to perform island splitting
	int splitIslandId;

	b2Vec2 gravity;
	float hitEventThreshold;
	float restitutionThreshold;
	float maxLinearSpeed;
	float contactSpeed;
	float contactHertz;
	float contactDampingRatio;
	float contactRecycleDistance;

	b2FrictionCallback* frictionCallback;
	b2RestitutionCallback* restitutionCallback;

	uint16_t generation;

	b2Profile profile;

	b2Capacity maxCapacity;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b2CustomFilterFcn* customFilterFcn;
	void* customFilterContext;

	int workerCount;
	b2EnqueueTaskCallback* enqueueTaskFcn;
	b2FinishTaskCallback* finishTaskFcn;
	void* userTaskContext;
	void* userTreeTask;

	struct b2Scheduler* scheduler;

	void* userData;

	// Remember type step used for reporting forces and torques
	// inverse sub-step
	float inv_h;

	// inverse full-step
	float inv_dt;

	int activeTaskCount;
	int taskCount;

	uint16_t worldId;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
	bool enableContactSoftening;
	bool enableContinuous;
	bool enableSpeculative;
	bool inUse;
} b2World;

b2World* b2GetWorldFromId( b2WorldId id );
b2World* b2GetWorld( int index );
b2World* b2GetWorldLocked( int index );

void b2ValidateConnectivity( b2World* world );
void b2ValidateSolverSets( b2World* world );
void b2ValidateContacts( b2World* world );
