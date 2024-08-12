// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef struct b2BodySim b2BodySim;
typedef struct b2BodyState b2BodyState;
typedef struct b2ContactSim b2ContactSim;
typedef struct b2IslandSim b2IslandSim;
typedef struct b2JointSim b2JointSim;

typedef struct b2BodySimArray
{
	b2BodySim* data;
	int count;
	int capacity;
} b2BodySimArray;

typedef struct b2BodyStateArray
{
	b2BodyState* data;
	int count;
	int capacity;
} b2BodyStateArray;

typedef struct b2ContactArray
{
	b2ContactSim* data;
	int count;
	int capacity;
} b2ContactArray;

typedef struct b2IslandArray
{
	b2IslandSim* data;
	int count;
	int capacity;
} b2IslandArray;

typedef struct b2JointArray
{
	b2JointSim* data;
	int count;
	int capacity;
} b2JointArray;

// These provide a way to create an array with a specified capacity. If the capacity is not
// known, you may use zero initialization.
b2BodySimArray b2CreateBodySimArray( int capacity );
b2BodyStateArray b2CreateBodyStateArray( int capacity );
b2ContactArray b2CreateContactArray( int capacity );
b2IslandArray b2CreateIslandArray( int capacity );
b2JointArray b2CreateJointArray( int capacity );

void b2DestroyBodySimArray( b2BodySimArray* array );
void b2DestroyBodyStateArray( b2BodyStateArray* array );
void b2DestroyContactArray( b2ContactArray* array );
void b2DestroyIslandArray( b2IslandArray* array );
void b2DestroyJointArray( b2JointArray* array );

b2BodySim* b2AddBodySim( b2BodySimArray* array );
b2BodyState* b2AddBodyState( b2BodyStateArray* array );
b2ContactSim* b2AddContact( b2ContactArray* array );
b2IslandSim* b2AddIsland( b2IslandArray* array );
b2JointSim* b2AddJoint( b2JointArray* array );

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
// todo have these return the id directly?
int b2RemoveBodySim( b2BodySimArray* array, int index );
int b2RemoveBodyState( b2BodyStateArray* array, int index );
int b2RemoveContact( b2ContactArray* array, int index );
int b2RemoveIsland( b2IslandArray* array, int index );
int b2RemoveJoint( b2JointArray* array, int index );
