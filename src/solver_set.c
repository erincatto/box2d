// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "core.h"
#include "island.h"
#include "joint.h"
#include "world.h"

#include <string.h>

B2_ARRAY_SOURCE( b2SolverSet, b2SolverSet )

void b2DestroySolverSet( b2World* world, int setIndex )
{
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
	b2BodySimArray_Destroy( &set->bodySims );
	b2BodyStateArray_Destroy( &set->bodyStates );
	b2ContactSimArray_Destroy( &set->contactSims );
	b2JointSimArray_Destroy( &set->jointSims );
	b2IslandSimArray_Destroy( &set->islandSims );
	b2FreeId( &world->solverSetIdPool, setIndex );
	*set = ( b2SolverSet ){ 0 };
	set->setIndex = B2_NULL_INDEX;
}

// Wake a solver set. Does not merge islands.
// Contacts can be in several places:
// 1. non-touching contacts in the disabled set
// 2. non-touching contacts already in the awake set
// 3. touching contacts in the sleeping set
// This handles contact types 1 and 3. Type 2 doesn't need any action.
void b2WakeSolverSet( b2World* world, int setIndex )
{
	B2_ASSERT( setIndex >= b2_firstSleepingSet );
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setIndex );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );

	b2Body* bodies = world->bodies.data;

	int bodyCount = set->bodySims.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2BodySim* simSrc = set->bodySims.data + i;

		b2Body* body = bodies + simSrc->bodyId;
		B2_ASSERT( body->setIndex == setIndex );
		body->setIndex = b2_awakeSet;
		body->localIndex = awakeSet->bodySims.count;

		// Reset sleep timer
		body->sleepTime = 0.0f;

		b2BodySim* simDst = b2BodySimArray_Add( &awakeSet->bodySims );
		memcpy( simDst, simSrc, sizeof( b2BodySim ) );

		b2BodyState* state = b2BodyStateArray_Add( &awakeSet->bodyStates );
		*state = b2_identityBodyState;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b2_disabledSet )
			{
				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;
			b2ContactSim* contactSim = b2ContactSimArray_Get( &disabledSet->contactSims, localIndex );

			B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 && contactSim->manifold.pointCount == 0 );

			contact->setIndex = b2_awakeSet;
			contact->localIndex = awakeSet->contactSims.count;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Add( &awakeSet->contactSims );
			memcpy( awakeContactSim, contactSim, sizeof( b2ContactSim ) );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( &disabledSet->contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = disabledSet->contactSims.data + localIndex;
				b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
				B2_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = set->contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSim = set->contactSims.data + i;
			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSim->contactId );
			B2_ASSERT( contact->flags & b2_contactTouchingFlag );
			B2_ASSERT( contactSim->simFlags & b2_simTouchingFlag );
			B2_ASSERT( contactSim->manifold.pointCount > 0 );
			B2_ASSERT( contact->setIndex == setIndex );
			b2AddContactToGraph( world, contactSim, contact );
			contact->setIndex = b2_awakeSet;
		}
	}

	// transfer joints from sleeping set to awake set
	{
		int jointCount = set->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSim = set->jointSims.data + i;
			b2Joint* joint = b2JointArray_Get( &world->joints, jointSim->jointId );
			B2_ASSERT( joint->setIndex == setIndex );
			b2AddJointToGraph( world, jointSim, joint );
			joint->setIndex = b2_awakeSet;
		}
	}

	// transfer island from sleeping set to awake set
	// Usually a sleeping set has only one island, but it is possible
	// that joints are created between sleeping islands and they
	// are moved to the same sleeping set.
	{
		int islandCount = set->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set->islandSims.data + i;
			b2Island* island = b2IslandArray_Get( &world->islands, islandSrc->islandId );
			island->setIndex = b2_awakeSet;
			island->localIndex = awakeSet->islandSims.count;
			b2IslandSim* islandDst = b2IslandSimArray_Add( &awakeSet->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet( world, setIndex );

	b2ValidateSolverSets( world );
}

void b2TrySleepIsland( b2World* world, int islandId )
{
	b2Island* island = b2IslandArray_Get( &world->islands, islandId );
	B2_ASSERT( island->setIndex == b2_awakeSet );

	// cannot put an island to sleep while it has a pending split
	if ( island->constraintRemoveCount > 0 )
	{
		return;
	}

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	int sleepSetId = b2AllocId( &world->solverSetIdPool );
	if ( sleepSetId == world->solverSets.count )
	{
		b2SolverSet set = { 0 };
		set.setIndex = B2_NULL_INDEX;
		b2SolverSetArray_Push( &world->solverSets, set );
	}

	b2SolverSet* sleepSet = b2SolverSetArray_Get( &world->solverSets, sleepSetId );
	*sleepSet = ( b2SolverSet ){ 0 };

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, b2_awakeSet );
	B2_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count );

	sleepSet->setIndex = sleepSetId;
	sleepSet->bodySims = b2BodySimArray_Create( island->bodyCount );
	sleepSet->contactSims = b2ContactSimArray_Create( island->contactCount );
	sleepSet->jointSims = b2JointSimArray_Create( island->jointCount );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSets, b2_disabledSet );
		int bodyId = island->headBody;
		while ( bodyId != B2_NULL_INDEX )
		{
			b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body->bodyMoveIndex != B2_NULL_INDEX )
			{
				b2BodyMoveEvent* moveEvent = b2BodyMoveEventArray_Get( &world->bodyMoveEvents, body->bodyMoveIndex );
				B2_ASSERT( moveEvent->bodyId.index1 - 1 == bodyId );
				B2_ASSERT( moveEvent->bodyId.generation == body->generation );
				moveEvent->fellAsleep = true;
				body->bodyMoveIndex = B2_NULL_INDEX;
			}

			int awakeBodyIndex = body->localIndex;
			b2BodySim* awakeSim = b2BodySimArray_Get( &awakeSet->bodySims, awakeBodyIndex );

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->bodySims.count;
			b2BodySim* sleepBodySim = b2BodySimArray_Add( &sleepSet->bodySims );
			memcpy( sleepBodySim, awakeSim, sizeof( b2BodySim ) );

			int movedIndex = b2BodySimArray_RemoveSwap( &awakeSet->bodySims, awakeBodyIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix local index on moved element
				b2BodySim* movedSim = awakeSet->bodySims.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b2Body* movedBody = b2BodyArray_Get( &world->bodies, movedId );
				B2_ASSERT( movedBody->localIndex == movedIndex );
				movedBody->localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			b2BodyStateArray_RemoveSwap( &awakeSet->bodyStates, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == b2_disabledSet );
				contactKey = contact->edges[edgeIndex].nextKey;

				if ( contact->setIndex == b2_disabledSet )
				{
					// already moved to disabled set by another body in the island
					continue;
				}

				if ( contact->colorIndex != B2_NULL_INDEX )
				{
					// contact is touching and will be moved separately
					B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) != 0 );
					continue;
				}

				// the other body may still be awake, it still may go to sleep and then it will be responsible
				// for moving this contact to the disabled set.
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = b2BodyArray_Get( &world->bodies, otherBodyId );
				if ( otherBody->setIndex == b2_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				b2ContactSim* contactSim = b2ContactSimArray_Get( &awakeSet->contactSims, localIndex );

				B2_ASSERT( contactSim->manifold.pointCount == 0 );
				B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 );

				// move the non-touching contact to the disabled set
				contact->setIndex = b2_disabledSet;
				contact->localIndex = disabledSet->contactSims.count;
				b2ContactSim* disabledContactSim = b2ContactSimArray_Add( &disabledSet->contactSims );
				memcpy( disabledContactSim, contactSim, sizeof( b2ContactSim ) );

				int movedLocalIndex = b2ContactSimArray_RemoveSwap( &awakeSet->contactSims, localIndex );
				if ( movedLocalIndex != B2_NULL_INDEX )
				{
					// fix moved element
					b2ContactSim* movedContactSim = awakeSet->contactSims.data + localIndex;
					b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
					B2_ASSERT( movedContact->localIndex == movedLocalIndex );
					movedContact->localIndex = localIndex;
				}
			}

			bodyId = body->islandNext;
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		int contactId = island->headContact;
		while ( contactId != B2_NULL_INDEX )
		{
			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );
			B2_ASSERT( contact->islandId == islandId );
			int colorIndex = contact->colorIndex;
			B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			// Remove bodies from graph coloring associated with this constraint
			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color->bodySet, contact->edges[0].bodyId );
				b2ClearBit( &color->bodySet, contact->edges[1].bodyId );
			}

			int localIndex = contact->localIndex;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Get( &color->contactSims, localIndex );

			int sleepContactIndex = sleepSet->contactSims.count;
			b2ContactSim* sleepContactSim = b2ContactSimArray_Add( &sleepSet->contactSims );
			memcpy( sleepContactSim, awakeContactSim, sizeof( b2ContactSim ) );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( &color->contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = color->contactSims.data + localIndex;
				b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
				B2_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}

			contact->setIndex = sleepSetId;
			contact->colorIndex = B2_NULL_INDEX;
			contact->localIndex = sleepContactIndex;

			contactId = contact->islandNext;
		}
	}

	// move joints
	// this shuffles joints in the awake set
	{
		int jointId = island->headJoint;
		while ( jointId != B2_NULL_INDEX )
		{
			b2Joint* joint = b2JointArray_Get( &world->joints, jointId );
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->islandId == islandId );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			b2JointSim* awakeJointSim = b2JointSimArray_Get( &color->jointSims, localIndex );

			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color->bodySet, joint->edges[0].bodyId );
				b2ClearBit( &color->bodySet, joint->edges[1].bodyId );
			}

			int sleepJointIndex = sleepSet->jointSims.count;
			b2JointSim* sleepJointSim = b2JointSimArray_Add( &sleepSet->jointSims );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b2JointSim ) );

			int movedIndex = b2JointSimArray_RemoveSwap( &color->jointSims, localIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2JointSim* movedJointSim = color->jointSims.data + localIndex;
				int movedId = movedJointSim->jointId;
				b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedId );
				B2_ASSERT( movedJoint->localIndex == movedIndex );
				movedJoint->localIndex = localIndex;
			}

			joint->setIndex = sleepSetId;
			joint->colorIndex = B2_NULL_INDEX;
			joint->localIndex = sleepJointIndex;

			jointId = joint->islandNext;
		}
	}

	// move island struct
	{
		B2_ASSERT( island->setIndex == b2_awakeSet );

		int islandIndex = island->localIndex;
		b2IslandSim* sleepIsland = b2IslandSimArray_Add( &sleepSet->islandSims );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2IslandSimArray_RemoveSwap( &awakeSet->islandSims, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = awakeSet->islandSims.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2Island* movedIsland = b2IslandArray_Get( &world->islands, movedIslandId );
			B2_ASSERT( movedIsland->localIndex == movedIslandIndex );
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	b2ValidateSolverSets( world );
}

// This is called when joints are created between sets. I want to allow the sets
// to continue sleeping if both are asleep. Otherwise one set is waked.
// Islands will get merge when the set is waked.
void b2MergeSolverSets( b2World* world, int setId1, int setId2 )
{
	B2_ASSERT( setId1 >= b2_firstSleepingSet );
	B2_ASSERT( setId2 >= b2_firstSleepingSet );
	b2SolverSet* set1 = b2SolverSetArray_Get( &world->solverSets, setId1 );
	b2SolverSet* set2 = b2SolverSetArray_Get( &world->solverSets, setId2 );

	// Move the fewest number of bodies
	if ( set1->bodySims.count < set2->bodySims.count )
	{
		b2SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// transfer bodies
	{
		b2Body* bodies = world->bodies.data;
		int bodyCount = set2->bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2BodySim* simSrc = set2->bodySims.data + i;

			b2Body* body = bodies + simSrc->bodyId;
			B2_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->bodySims.count;

			b2BodySim* simDst = b2BodySimArray_Add( &set1->bodySims );
			memcpy( simDst, simSrc, sizeof( b2BodySim ) );
		}
	}

	// transfer contacts
	{
		int contactCount = set2->contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSrc = set2->contactSims.data + i;

			b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSrc->contactId );
			B2_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactSims.count;

			b2ContactSim* contactDst = b2ContactSimArray_Add( &set1->contactSims );
			memcpy( contactDst, contactSrc, sizeof( b2ContactSim ) );
		}
	}

	// transfer joints
	{
		int jointCount = set2->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSrc = set2->jointSims.data + i;

			b2Joint* joint = b2JointArray_Get( &world->joints, jointSrc->jointId );
			B2_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointSims.count;

			b2JointSim* jointDst = b2JointSimArray_Add( &set1->jointSims );
			memcpy( jointDst, jointSrc, sizeof( b2JointSim ) );
		}
	}

	// transfer islands
	{
		int islandCount = set2->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2->islandSims.data + i;
			int islandId = islandSrc->islandId;

			b2Island* island = b2IslandArray_Get( &world->islands, islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandSims.count;

			b2IslandSim* islandDst = b2IslandSimArray_Add( &set1->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the merged set
	b2DestroySolverSet( world, setId2 );

	b2ValidateSolverSets( world );
}

void b2TransferBody( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Body* body )
{
	B2_ASSERT( targetSet != sourceSet );

	int sourceIndex = body->localIndex;
	b2BodySim* sourceSim = b2BodySimArray_Get( &sourceSet->bodySims, sourceIndex );

	int targetIndex = targetSet->bodySims.count;
	b2BodySim* targetSim = b2BodySimArray_Add( &targetSet->bodySims );
	memcpy( targetSim, sourceSim, sizeof( b2BodySim ) );

	// Remove body sim from solver set that owns it
	int movedIndex = b2BodySimArray_RemoveSwap( &sourceSet->bodySims, sourceIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved body index
		b2BodySim* movedSim = sourceSet->bodySims.data + sourceIndex;
		int movedId = movedSim->bodyId;
		b2Body* movedBody = b2BodyArray_Get( &world->bodies, movedId );
		B2_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = sourceIndex;
	}

	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2BodyStateArray_RemoveSwap( &sourceSet->bodyStates, sourceIndex );
	}
	else if ( targetSet->setIndex == b2_awakeSet )
	{
		b2BodyState* state = b2BodyStateArray_Add( &targetSet->bodyStates );
		*state = b2_identityBodyState;
	}

	body->setIndex = targetSet->setIndex;
	body->localIndex = targetIndex;
}

void b2TransferJoint( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Joint* joint )
{
	B2_ASSERT( targetSet != sourceSet );

	int localIndex = joint->localIndex;
	int colorIndex = joint->colorIndex;

	// Retrieve source.
	b2JointSim* sourceSim;
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;

		sourceSim = b2JointSimArray_Get( &color->jointSims, localIndex );
	}
	else
	{
		B2_ASSERT( colorIndex == B2_NULL_INDEX );
		sourceSim = b2JointSimArray_Get( &sourceSet->jointSims, localIndex );
	}

	// Create target and copy. Fix joint.
	if ( targetSet->setIndex == b2_awakeSet )
	{
		b2AddJointToGraph( world, sourceSim, joint );
		joint->setIndex = b2_awakeSet;
	}
	else
	{
		joint->setIndex = targetSet->setIndex;
		joint->localIndex = targetSet->jointSims.count;
		joint->colorIndex = B2_NULL_INDEX;

		b2JointSim* targetSim = b2JointSimArray_Add( &targetSet->jointSims );
		memcpy( targetSim, sourceSim, sizeof( b2JointSim ) );
	}

	// Destroy source.
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b2JointSimArray_RemoveSwap( &sourceSet->jointSims, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// fix swapped element
			b2JointSim* movedJointSim = sourceSet->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b2Joint* movedJoint = b2JointArray_Get( &world->joints, movedId );
			movedJoint->localIndex = localIndex;
		}
	}
}
