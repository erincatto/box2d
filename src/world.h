// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "array.h"
#include "bitset.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "id_pool.h"
#include "stack_allocator.h"

#include "box2d/types.h"

typedef struct b2ContactSim b2ContactSim;

enum b2SetType
{
	b2_staticSet = 0,
	b2_disabledSet = 1,
	b2_awakeSet = 2,
	b2_firstSleepingSet = 3,
};

// Per thread task storage
typedef struct b2TaskContext
{
	// These bits align with the b2ConstraintGraph::contactBlocks and signal a change in contact status
	b2BitSet contactStateBitSet;

	// Used to track bodies with shapes that have enlarged AABBs. This avoids having a bit array
	// that is very large when there are many static shapes.
	b2BitSet enlargedSimBitSet;

	// Used to put islands to sleep
	b2BitSet awakeIslandBitSet;

	// Per worker split island candidate
	float splitSleepTime;
	int splitIslandId;

} b2TaskContext;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
typedef struct b2World
{
	b2StackAllocator stackAllocator;
	b2BroadPhase broadPhase;
	b2ConstraintGraph constraintGraph;

	// The body id pool is used to allocate and recycle body ids. Body ids
	// provide a stable identifier for users, but incur caches misses when used
	// to access body data. Aligns with b2Body.
	b2IdPool bodyIdPool;

	// This is a sparse array that maps body ids to the body data
	// stored in solver sets. As sims move within a set or across set.
	// Indices come from id pool.
	b2BodyArray bodies;

	// Provides free list for solver sets.
	b2IdPool solverSetIdPool;

	// Solvers sets allow sims to be stored in contiguous arrays. The first
	// set is all static sims. The second set is active sims. The third set is disabled
	// sims. The remaining sets are sleeping islands.
	b2SolverSetArray solverSets;

	// Used to create stable ids for joints
	b2IdPool jointIdPool;

	// This is a sparse array that maps joint ids to the joint data stored in the constraint graph
	// or in the solver sets.
	b2JointArray joints;

	// Used to create stable ids for contacts
	b2IdPool contactIdPool;

	// This is a sparse array that maps contact ids to the contact data stored in the constraint graph
	// or in the solver sets.
	b2ContactArray contacts;

	// Used to create stable ids for islands
	b2IdPool islandIdPool;

	// This is a sparse array that maps island ids to the island data stored in the solver sets.
	b2IslandArray islands;

	b2IdPool shapeIdPool;
	b2IdPool chainIdPool;

	// These are sparse arrays that point into the pools above
	b2ShapeArray shapes;
	b2ChainShapeArray chainShapes;

	// Per thread storage
	b2TaskContextArray taskContexts;

	b2BodyMoveEventArray bodyMoveEvents;
	b2SensorBeginTouchEventArray sensorBeginEvents;
	b2ContactBeginTouchEventArray contactBeginEvents;

	// End events are double buffered so that the user doesn't need to flush events
	b2SensorEndTouchEventArray sensorEndEvents[2];
	b2ContactEndTouchEventArray contactEndEvents[2];
	int endEventArrayIndex;

	b2ContactHitEventArray contactHitEvents;

	// Used to track debug draw
	b2BitSet debugBodySet;
	b2BitSet debugJointSet;
	b2BitSet debugContactSet;

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
	float maxLinearVelocity;
	float contactPushoutVelocity;
	float contactHertz;
	float contactDampingRatio;
	float jointHertz;
	float jointDampingRatio;

	b2MixingRule frictionMixingRule;
	b2MixingRule restitutionMixingRule;

	uint16_t revision;

	b2Profile profile;

	b2PreSolveFcn* preSolveFcn;
	void* preSolveContext;

	b2CustomFilterFcn* customFilterFcn;
	void* customFilterContext;

	int workerCount;
	b2EnqueueTaskCallback* enqueueTaskFcn;
	b2FinishTaskCallback* finishTaskFcn;
	void* userTaskContext;
	void* userTreeTask;

	void* userData;

	// Remember type step used for reporting forces and torques
	float inv_h;

	int activeTaskCount;
	int taskCount;

	uint16_t worldId;

	bool enableSleep;
	bool locked;
	bool enableWarmStarting;
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

B2_ARRAY_INLINE( b2BodyMoveEvent, b2BodyMoveEvent );
B2_ARRAY_INLINE( b2ContactBeginTouchEvent, b2ContactBeginTouchEvent );
B2_ARRAY_INLINE( b2ContactEndTouchEvent, b2ContactEndTouchEvent );
B2_ARRAY_INLINE( b2ContactHitEvent, b2ContactHitEvent );
B2_ARRAY_INLINE( b2SensorBeginTouchEvent, b2SensorBeginTouchEvent );
B2_ARRAY_INLINE( b2SensorEndTouchEvent, b2SensorEndTouchEvent );
B2_ARRAY_INLINE( b2TaskContext, b2TaskContext );
