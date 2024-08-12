// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "block_array.h"

#include "allocate.h"
#include "body.h"
#include "contact.h"
#include "core.h"
#include "island.h"
#include "joint.h"

#include <string.h>

#define B2_INITIAL_CAPACITY 16

b2BodySimArray b2CreateBodySimArray( int capacity )
{
	if ( capacity > 0 )
	{
		return ( b2BodySimArray ){ b2Alloc( capacity * sizeof( b2BodySim ) ), 0, capacity };
	}

	return ( b2BodySimArray ){ 0 };
}

b2BodyStateArray b2CreateBodyStateArray( int capacity )
{
	if ( capacity > 0 )
	{
		return ( b2BodyStateArray ){ b2Alloc( capacity * sizeof( b2BodyState ) ), 0, capacity };
	}

	return ( b2BodyStateArray ){ 0 };
}

b2ContactArray b2CreateContactArray( int capacity )
{
	if ( capacity > 0 )
	{
		return ( b2ContactArray ){ b2Alloc( capacity * sizeof( b2ContactSim ) ), 0, capacity };
	}

	return ( b2ContactArray ){ 0 };
}

b2JointArray b2CreateJointArray( int capacity )
{
	if ( capacity > 0 )
	{
		return ( b2JointArray ){ b2Alloc( capacity * sizeof( b2JointSim ) ), 0, capacity };
	}

	return ( b2JointArray ){ 0 };
}

b2IslandArray b2CreateIslandArray( int capacity )
{
	if ( capacity > 0 )
	{
		return ( b2IslandArray ){ b2Alloc( capacity * sizeof( b2IslandSim ) ), 0, capacity };
	}

	return ( b2IslandArray ){ 0 };
}

void b2DestroyBodySimArray( b2BodySimArray* array )
{
	b2Free( array->data, array->capacity * sizeof( b2BodySim ) );
}

void b2DestroyBodyStateArray( b2BodyStateArray* array )
{
	b2Free( array->data, array->capacity * sizeof( b2BodyState ) );
}

void b2DestroyContactArray( b2ContactArray* array )
{
	b2Free( array->data, array->capacity * sizeof( b2ContactSim ) );
}

void b2DestroyJointArray( b2JointArray* array )
{
	b2Free( array->data, array->capacity * sizeof( b2JointSim ) );
}

void b2DestroyIslandArray( b2IslandArray* array )
{
	b2Free( array->data, array->capacity * sizeof( b2IslandSim ) );
}

b2BodySim* b2AddBodySim( b2BodySimArray* array )
{
	int elementSize = sizeof( b2BodySim );
	if ( array->capacity == 0 )
	{
		B2_ASSERT( array->count == 0 );
		array->data = b2Alloc( B2_INITIAL_CAPACITY * elementSize );
		array->capacity = B2_INITIAL_CAPACITY;
	}
	else if ( array->count == array->capacity )
	{
		int newCapacity = 2 * array->capacity;
		b2BodySim* newElements = b2Alloc( newCapacity * elementSize );
		memcpy( newElements, array->data, array->capacity * elementSize );
		b2Free( array->data, array->capacity * elementSize );
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2BodySim* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2BodyState* b2AddBodyState( b2BodyStateArray* array )
{
	int elementSize = sizeof( b2BodyState );
	if ( array->capacity == 0 )
	{
		B2_ASSERT( array->count == 0 );
		array->data = b2Alloc( B2_INITIAL_CAPACITY * elementSize );
		array->capacity = B2_INITIAL_CAPACITY;
	}
	else if ( array->count == array->capacity )
	{
		int newCapacity = 2 * array->capacity;
		b2BodyState* newElements = b2Alloc( newCapacity * elementSize );
		memcpy( newElements, array->data, array->capacity * elementSize );
		b2Free( array->data, array->capacity * elementSize );
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2BodyState* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2ContactSim* b2AddContact( b2ContactArray* array )
{
	int elementSize = sizeof( b2ContactSim );
	if ( array->capacity == 0 )
	{
		B2_ASSERT( array->count == 0 );
		array->data = b2Alloc( B2_INITIAL_CAPACITY * elementSize );
		array->capacity = B2_INITIAL_CAPACITY;
	}
	else if ( array->count == array->capacity )
	{
		int newCapacity = 2 * array->capacity;
		b2ContactSim* newElements = b2Alloc( newCapacity * elementSize );
		memcpy( newElements, array->data, array->capacity * elementSize );
		b2Free( array->data, array->capacity * elementSize );
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2ContactSim* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2JointSim* b2AddJoint( b2JointArray* array )
{
	int elementSize = sizeof( b2JointSim );
	if ( array->capacity == 0 )
	{
		B2_ASSERT( array->count == 0 );
		array->data = b2Alloc( B2_INITIAL_CAPACITY * elementSize );
		array->capacity = B2_INITIAL_CAPACITY;
	}
	else if ( array->count == array->capacity )
	{
		int newCapacity = 2 * array->capacity;
		b2JointSim* newElements = b2Alloc( newCapacity * elementSize );
		memcpy( newElements, array->data, array->capacity * elementSize );
		b2Free( array->data, array->capacity * elementSize );
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2JointSim* element = array->data + array->count;
	array->count += 1;
	return element;
}

b2IslandSim* b2AddIsland( b2IslandArray* array )
{
	int elementSize = sizeof( b2IslandSim );
	if ( array->capacity == 0 )
	{
		B2_ASSERT( array->count == 0 );
		array->data = b2Alloc( B2_INITIAL_CAPACITY * elementSize );
		array->capacity = B2_INITIAL_CAPACITY;
	}
	else if ( array->count == array->capacity )
	{
		int newCapacity = 2 * array->capacity;
		b2IslandSim* newElements = b2Alloc( newCapacity * elementSize );
		memcpy( newElements, array->data, array->capacity * elementSize );
		b2Free( array->data, array->capacity * elementSize );
		array->data = newElements;
		array->capacity = newCapacity;
	}

	b2IslandSim* element = array->data + array->count;
	element->islandId = B2_NULL_INDEX;
	array->count += 1;
	return element;
}

// Returns the index of the element moved into the empty slot (or B2_NULL_INDEX)
int b2RemoveBodySim( b2BodySimArray* array, int index )
{
	B2_ASSERT( 0 <= index && index < array->count );
	if ( index < array->count - 1 )
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveBodyState( b2BodyStateArray* array, int index )
{
	B2_ASSERT( 0 <= index && index < array->count );
	if ( index < array->count - 1 )
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveContact( b2ContactArray* array, int index )
{
	B2_ASSERT( 0 <= index && index < array->count );
	if ( index < array->count - 1 )
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveJoint( b2JointArray* array, int index )
{
	B2_ASSERT( 0 <= index && index < array->count );
	if ( index < array->count - 1 )
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}

int b2RemoveIsland( b2IslandArray* array, int index )
{
	B2_ASSERT( 0 <= index && index < array->count );
	if ( index < array->count - 1 )
	{
		int removed = array->count - 1;
		array->data[index] = array->data[removed];
		array->count -= 1;
		return removed;
	}

	array->count -= 1;
	return B2_NULL_INDEX;
}
