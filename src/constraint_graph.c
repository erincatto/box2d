// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "constraint_graph.h"

#include "array.h"
#include "bitset.h"
#include "body.h"
#include "contact.h"
#include "joint.h"
#include "solver_set.h"
#include "world.h"

#include <string.h>

// Solver using graph coloring. Islands are only used for sleep.
// High-Performance Physical Simulations on Next-Generation Architecture with Many Cores
// http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf

// Kinematic bodies have to be treated like dynamic bodies in graph coloring. Unlike static bodies, we cannot use a dummy solver
// body for kinematic bodies. We cannot access a kinematic body from multiple threads efficiently because the SIMD solver body
// scatter would write to the same kinematic body from multiple threads. Even if these writes don't modify the body, they will
// cause horrible cache stalls. To make this feasible I would need a way to block these writes.

// This is used for debugging by making all constraints be assigned to overflow.
#define B2_FORCE_OVERFLOW 0

_Static_assert( B2_GRAPH_COLOR_COUNT == 12, "graph color count assumed to be 12" );

void b2CreateGraph( b2ConstraintGraph* graph, int bodyCapacity )
{
	_Static_assert( B2_GRAPH_COLOR_COUNT >= 2, "must have at least two constraint graph colors" );
	_Static_assert( B2_OVERFLOW_INDEX == B2_GRAPH_COLOR_COUNT - 1, "bad over flow index" );

	*graph = ( b2ConstraintGraph ){ 0 };

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
	}
}

// Contacts are always created as non-touching. They get cloned into the constraint
// graph once they are found to be touching.
// todo maybe kinematic bodies should not go into graph
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
	bool staticA = bodyA->setIndex == b2_staticSet;
	bool staticB = bodyB->setIndex == b2_staticSet;
	B2_ASSERT( staticA == false || staticB == false );

#if B2_FORCE_OVERFLOW == 0
	if ( staticA == false && staticB == false )
	{
		for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
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
	else if ( staticA == false )
	{
		// No static contacts in color 0
		for ( int i = 1; i < B2_OVERFLOW_INDEX; ++i )
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
	else if ( staticB == false )
	{
		// No static contacts in color 0
		for ( int i = 1; i < B2_OVERFLOW_INDEX; ++i )
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

	if ( staticA )
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

	if ( staticB )
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
}

void b2RemoveContactFromGraph( b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex )
{
	b2ConstraintGraph* graph = &world->constraintGraph;

	B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
	b2GraphColor* color = graph->colors + colorIndex;

	if ( colorIndex != B2_OVERFLOW_INDEX )
	{
		// might clear a bit for a static body, but this has no effect
		b2ClearBit( &color->bodySet, bodyIdA );
		b2ClearBit( &color->bodySet, bodyIdB );
	}

	int movedIndex = b2ContactSimArray_RemoveSwap( &color->contactSims, localIndex );
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
}

static int b2AssignJointColor( b2ConstraintGraph* graph, int bodyIdA, int bodyIdB, bool staticA, bool staticB )
{
	B2_ASSERT( staticA == false || staticB == false );

#if B2_FORCE_OVERFLOW == 0
	if ( staticA == false && staticB == false )
	{
		for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
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
	else if ( staticA == false )
	{
		for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
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
	else if ( staticB == false )
	{
		for ( int i = 0; i < B2_OVERFLOW_INDEX; ++i )
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
	B2_UNUSED( graph, bodyIdA, bodyIdB, staticA, staticB );
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
	bool staticA = bodyA->setIndex == b2_staticSet;
	bool staticB = bodyB->setIndex == b2_staticSet;

	int colorIndex = b2AssignJointColor( graph, bodyIdA, bodyIdB, staticA, staticB );

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
