// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constraint_graph.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "contact_solver.h"
#include "joint.h"
#include "physics_world.h"
#include "solver_set.h"

#include <string.h>

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.
// todo should be possible to branch on the scatters to avoid writing to kinematic bodies

// This is used for debugging by making all constraints be assigned to overflow.
#define B2_FORCE_OVERFLOW 0

void b2CreateGraph( b2ConstraintGraph* graph, int bodyCapacity )
{
	_Static_assert( B2_GRAPH_COLOR_COUNT >= 2, "must have at least two constraint graph colors" );
	_Static_assert( B2_OVERFLOW_INDEX == B2_GRAPH_COLOR_COUNT - 1, "bad over flow index" );
	_Static_assert( B2_DYNAMIC_COLOR_COUNT >= 2, "need more dynamic colors" );

	*graph = (b2ConstraintGraph){ 0 };

	bodyCapacity = b2MaxInt( bodyCapacity, 8 );

	// Initialize graph color bit set.
	// No bitset for overflow color.
	for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
	{
		b2GraphColor* color = graph->colors + i;
		color->bodySet = b2CreateBitSet( bodyCapacity );
		b2SetBitCountAndClear( &color->bodySet, bodyCapacity );
	}
}

void b2DestroyGraph( b2ConstraintGraph* graph )
{
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* color = graph->colors + i;

		// The bit set should never be used on the overflow color
		B2_ASSERT( i != B2_OVERFLOW_INDEX || color->bodySet.bits == NULL );

		b2DestroyBitSet( &color->bodySet );

		b2ContactSimArray_Destroy( &color->contactSims );
		b2JointSimArray_Destroy( &color->jointSims );

		if ( i == B2_OVERFLOW_INDEX )
		{
			b2Free( color->overflowConstraints, color->wideContactCapacity * sizeof( b2ContactConstraint ) );
		}
		else
		{
			b2Free( color->wideConstraints, color->wideContactCapacity * sizeof( b2ContactConstraintWide ) );
		}
	}
}

// Contacts are always created as non-touching. They get moved into the constraint
// graph once they are found to be touching.
void b2AddContactToGraph( b2World* world, b2ContactSim* contactSim, b2Contact* contact )
{
	B2_ASSERT( contactSim->manifold.pointCount > 0 );
	B2_ASSERT( contactSim->simFlags & b2_simTouchingFlag );
	B2_ASSERT( contact->flags & b2_contactTouchingFlag );

	b2ConstraintGraph* graph = &world->constraintGraph;
	int colorIndex = B2_OVERFLOW_INDEX;

	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	b2BodyType typeA = bodyA->type;
	b2BodyType typeB = bodyB->type;
	B2_ASSERT( typeA == b2_dynamicBody || typeB == b2_dynamicBody );

#if B2_FORCE_OVERFLOW == 0
	if ( typeA == b2_dynamicBody && typeB == b2_dynamicBody )
	{
		// Dynamic constraint colors cannot encroach on colors reserved for static constraints
		for ( int i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdA ) || b2GetBit( &color->bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdA );
			b2SetBitGrow( &color->bodySet, bodyIdB );
			colorIndex = i;
			break;
		}
	}
	else if ( typeA == b2_dynamicBody )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdA ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdA );
			colorIndex = i;
			break;
		}
	}
	else if ( typeB == b2_dynamicBody )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdB );
			colorIndex = i;
			break;
		}
	}
#endif

	b2GraphColor* color = graph->colors + colorIndex;
	contact->colorIndex = colorIndex;
	contact->localIndex = color->contactSims.count;

	b2ContactSim* newContact = b2ContactSimArray_Add( &color->contactSims );
	memcpy( newContact, contactSim, sizeof( b2ContactSim ) );

	// todo perhaps skip this if the contact is already awake

	if ( typeA == b2_staticBody )
	{
		newContact->bodySimIndexA = B2_NULL_INDEX;
		newContact->invMassA = 0.0f;
		newContact->invIA = 0.0f;
	}
	else
	{
		B2_ASSERT( bodyA->setIndex == b2_awakeSet );
		b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );

		int localIndex = bodyA->localIndex;
		newContact->bodySimIndexA = localIndex;

		b2BodySim* bodySimA = b2BodySimArray_Get( &awakeSet->bodySims, localIndex );
		newContact->invMassA = bodySimA->invMass;
		newContact->invIA = bodySimA->invInertia;
	}

	if ( typeB == b2_staticBody )
	{
		newContact->bodySimIndexB = B2_NULL_INDEX;
		newContact->invMassB = 0.0f;
		newContact->invIB = 0.0f;
	}
	else
	{
		B2_ASSERT( bodyB->setIndex == b2_awakeSet );
		b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );

		int localIndex = bodyB->localIndex;
		newContact->bodySimIndexB = localIndex;

		b2BodySim* bodySimB = b2BodySimArray_Get( &awakeSet->bodySims, localIndex );
		newContact->invMassB = bodySimB->invMass;
		newContact->invIB = bodySimB->invInertia;
	}

	// Allocate a contact constraint
	if ( colorIndex == B2_OVERFLOW_INDEX )
	{
		if ( color->contactCount == color->wideContactCapacity )
		{
			int oldCapacity = color->wideContactCapacity;
			int newCapacity = b2MaxInt( 8, 2 * oldCapacity );
			int elementSize = (int)sizeof( b2ContactConstraint );
			color->overflowConstraints =
				b2GrowAllocZeroInit( color->overflowConstraints, oldCapacity * elementSize, newCapacity * elementSize );
			color->wideContactCapacity = newCapacity;
		}

		color->overflowConstraints[color->contactCount].contactIndex = contact->contactId + 1;
	}
	else
	{
		// Initial space
		if ( color->wideContactCapacity == 0 )
		{
			int capacity = 8;
			color->wideContactCapacity = capacity;
			color->wideConstraints = b2AllocZeroInit( 8 * (int)sizeof( b2ContactConstraintWide ) );
			color->contactCount = 0;
		}

		// Did the lane roll over to the next wide constraint?
		if ( color->contactCount > 0 && color->contactCount % B2_SIMD_WIDTH == 0 )
		{
			// Grow if needed
			int capacity = color->wideContactCapacity;
			if ( color->contactCount == B2_SIMD_WIDTH * capacity )
			{
				int newCapacity = b2MaxInt( 8, 2 * capacity );
				int elementSize = (int)sizeof( b2ContactConstraintWide );
				color->wideConstraints =
					b2GrowAllocZeroInit( color->wideConstraints, capacity * elementSize, newCapacity * elementSize );
				color->wideContactCapacity = newCapacity;
			}
		}

		int wideIndex = color->contactCount / B2_SIMD_WIDTH;
		int laneIndex = color->contactCount & ( B2_SIMD_WIDTH - 1 );
		B2_VALIDATE( color->wideConstraints[wideIndex].contactIndex[laneIndex] == 0 );
		color->wideConstraints[wideIndex].contactIndex[laneIndex] = contact->contactId + 1;

#if B2_ENABLE_VALIDATION
		{
			int index = 0;
			for ( int i = 0; i < color->wideContactCapacity; ++i )
			{
				b2ContactConstraintWide* c = color->wideConstraints + i;
				for ( int j = 0; j < B2_SIMD_WIDTH; ++j )
				{
					if ( index < color->contactCount + 1 )
					{
						B2_VALIDATE( c->contactIndex[j] > 0 );
					}
					else
					{
						B2_VALIDATE( c->contactIndex[j] == 0 );
					}

					index += 1;
				}
			}
		}
#endif
	}

	b2ContactId id = {
		.index1 = newContact->contactId + 1,
		.world0 = world->worldId,
		.padding = 0,
		.generation = contact->generation,
	};

	b2ContactIdArray_Push( &world->pendingContacts, id );

	color->contactCount += 1;
}

int b2RemoveContactConstraint( b2World* world, b2Contact* contact )
{
	int colorIndex = contact->colorIndex;
	int localIndex = contact->localIndex;
	b2GraphColor* color = world->constraintGraph.colors + colorIndex;
	int constraintIndex = localIndex;
	B2_ASSERT( 0 <= constraintIndex && constraintIndex < color->contactCount );
	int result = B2_NULL_INDEX;

	if ( colorIndex == B2_OVERFLOW_INDEX )
	{
		int lastConstraintIndex = color->contactCount - 1;

		// If this is not the last constraint, then swap the last constraint into this slot.
		if ( constraintIndex != lastConstraintIndex )
		{
			b2ContactConstraint* source = color->overflowConstraints + lastConstraintIndex;
			b2ContactConstraint* target = color->overflowConstraints + constraintIndex;

			memcpy( target, source, sizeof( b2ContactConstraint ) );

			source->contactIndex = 0;
			source->indexA = 0;
			source->indexB = 0;

			int movedContactIndex = target->contactIndex - 1;
			b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactIndex );
			movedContact->localIndex = constraintIndex;
			result = lastConstraintIndex;
		}
	}
	else
	{
		int wideConstraintIndex = constraintIndex / B2_SIMD_WIDTH;
		int laneIndex = constraintIndex & ( B2_SIMD_WIDTH - 1 );

		int lastWideConstraintIndex = ( color->contactCount - 1 ) / B2_SIMD_WIDTH;
		int lastLaneIndex = ( color->contactCount - 1 ) & ( B2_SIMD_WIDTH - 1 );

		B2_ASSERT( B2_SIMD_WIDTH * lastWideConstraintIndex + lastLaneIndex == color->contactCount - 1 );

		b2ContactConstraintWide* source = color->wideConstraints + lastWideConstraintIndex;
		int sourceLane = lastLaneIndex;

		// If this is not the last constraint, then swap the last constraint into this slot.
		if ( constraintIndex != color->contactCount - 1 )
		{
			int targetLane = laneIndex;
			b2ContactConstraintWide* target = color->wideConstraints + wideConstraintIndex;

			// Copy lane by shifting the constraint start address by lane offset
			static_assert( sizeof( b2ContactConstraintWide ) % (B2_SIMD_WIDTH * sizeof( float )) == 0 );
			int floatCount = sizeof( b2ContactConstraintWide ) / (B2_SIMD_WIDTH * sizeof(float));
			float* restrict sourceFloats = ((float*)source) + sourceLane;
			float* restrict targetFloats = ((float*)target) + targetLane;
			for ( int i = 0; i < B2_SIMD_WIDTH * floatCount; i += B2_SIMD_WIDTH )
			{
				targetFloats[i] = sourceFloats[i];
			}

			int movedContactIndex = target->contactIndex[targetLane] - 1;
			b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactIndex );
			movedContact->localIndex = constraintIndex;
			result = color->contactCount - 1;
		}

		source->contactIndex[sourceLane] = 0;
		source->indexA[sourceLane] = 0;
		source->indexB[sourceLane] = 0;

#if B2_ENABLE_VALIDATION
		{
			int index = 0;
			for ( int i = 0; i < color->wideContactCapacity; ++i )
			{
				b2ContactConstraintWide* c = color->wideConstraints + i;
				for ( int j = 0; j < B2_SIMD_WIDTH; ++j )
				{
					if ( index < color->contactCount - 1 )
					{
						B2_VALIDATE( c->contactIndex[j] > 0 );
					}
					else
					{
						B2_VALIDATE( c->contactIndex[j] == 0 );
					}

					index += 1;
				}
			}
		}
#endif
	}

	color->contactCount -= 1;
	return result;
}

void b2RemoveContactFromGraph( b2World* world, int bodyIdA, int bodyIdB, b2Contact* contact )
{
	int colorIndex = contact->colorIndex;
	int localIndex = contact->localIndex;
	B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
	b2GraphColor* color = world->constraintGraph.colors + colorIndex;
	B2_ASSERT( color->contactCount > 0 );

	int constraintMovedIndex = b2RemoveContactConstraint( world, contact );

	if ( colorIndex != B2_OVERFLOW_INDEX )
	{
		// This might clear a bit for a kinematic or static body, but this has no effect
		b2ClearBit( &color->bodySet, bodyIdA );
		b2ClearBit( &color->bodySet, bodyIdB );
	}

	int movedIndex = b2ContactSimArray_RemoveSwap( &color->contactSims, localIndex );
	(void)movedIndex;
	(void)constraintMovedIndex;
	B2_ASSERT( movedIndex == constraintMovedIndex );

#if 0
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix index on swapped contact
		b2ContactSim* movedContactSim = color->contactSims.data + localIndex;

		// Fix moved contact
		int movedId = movedContactSim->contactId;
		b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedId );
		B2_ASSERT( movedContact->setIndex == b2_awakeSet );
		B2_ASSERT( movedContact->colorIndex == colorIndex );
		B2_ASSERT( movedContact->localIndex == movedIndex );
		movedContact->localIndex = localIndex;
	}
#endif
}

// Notice that a joint cannot share the same color as a contact between the same two bodies. This means I can solve contacts and
// joints in parallel with each other within each color.
static int b2AssignJointColor( b2ConstraintGraph* graph, int bodyIdA, int bodyIdB, b2BodyType typeA, b2BodyType typeB )
{
	B2_ASSERT( typeA == b2_dynamicBody || typeB == b2_dynamicBody );

#if B2_FORCE_OVERFLOW == 0
	if ( typeA == b2_dynamicBody && typeB == b2_dynamicBody )
	{
		// Dynamic constraint colors cannot encroach on colors reserved for static constraints
		for ( int i = 0; i < B2_DYNAMIC_COLOR_COUNT; ++i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdA ) || b2GetBit( &color->bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdA );
			b2SetBitGrow( &color->bodySet, bodyIdB );
			return i;
		}
	}
	else if ( typeA == b2_dynamicBody )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdA ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdA );
			return i;
		}
	}
	else if ( typeB == b2_dynamicBody )
	{
		// Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		for ( int i = B2_OVERFLOW_INDEX - 1; i >= 1; --i )
		{
			b2GraphColor* color = graph->colors + i;
			if ( b2GetBit( &color->bodySet, bodyIdB ) )
			{
				continue;
			}

			b2SetBitGrow( &color->bodySet, bodyIdB );
			return i;
		}
	}
#else
	B2_UNUSED( graph, bodyIdA, bodyIdB );
#endif

	return B2_OVERFLOW_INDEX;
}

b2JointSim* b2CreateJointInGraph( b2World* world, b2Joint* joint )
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	int bodyIdA = joint->edges[0].bodyId;
	int bodyIdB = joint->edges[1].bodyId;
	b2Body* bodyA = b2BodyArray_Get( &world->bodies, bodyIdA );
	b2Body* bodyB = b2BodyArray_Get( &world->bodies, bodyIdB );

	int colorIndex = b2AssignJointColor( graph, bodyIdA, bodyIdB, bodyA->type, bodyB->type );

	b2JointSim* jointSim = b2JointSimArray_Add( &graph->colors[colorIndex].jointSims );
	memset( jointSim, 0, sizeof( b2JointSim ) );

	joint->colorIndex = colorIndex;
	joint->localIndex = graph->colors[colorIndex].jointSims.count - 1;
	return jointSim;
}

void b2AddJointToGraph( b2World* world, b2JointSim* jointSim, b2Joint* joint )
{
	b2JointSim* jointDst = b2CreateJointInGraph( world, joint );
	memcpy( jointDst, jointSim, sizeof( b2JointSim ) );
}

void b2RemoveJointFromGraph( b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
	b2GraphColor* color = graph->colors + colorIndex;

	if ( colorIndex != B2_OVERFLOW_INDEX )
	{
		// May clear static bodies, no effect
		b2ClearBit( &color->bodySet, bodyIdA );
		b2ClearBit( &color->bodySet, bodyIdB );
	}

	int movedIndex = b2JointSimArray_RemoveSwap( &color->jointSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved joint
		b2JointSim* movedJointSim = color->jointSims.data + localIndex;
		int movedId = movedJointSim->jointId;
		b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedId );
		B2_ASSERT( movedJoint->setIndex == b2_awakeSet );
		B2_ASSERT( movedJoint->colorIndex == colorIndex );
		B2_ASSERT( movedJoint->localIndex == movedIndex );
		movedJoint->localIndex = localIndex;
	}
}

b2HexColor b2_graphColors[B2_GRAPH_COLOR_COUNT] = {
	b2_colorRed,	b2_colorOrange, b2_colorYellow,	   b2_colorGreen,	  b2_colorCyan,		b2_colorBlue,
	b2_colorViolet, b2_colorPink,	b2_colorChocolate, b2_colorGoldenRod, b2_colorCoral,	b2_colorRosyBrown,
	b2_colorAqua,	b2_colorPeru,	b2_colorLime,	   b2_colorGold,	  b2_colorPlum,		b2_colorSnow,
	b2_colorTeal,	b2_colorKhaki,	b2_colorSalmon,	   b2_colorPeachPuff, b2_colorHoneyDew, b2_colorBlack,
};
