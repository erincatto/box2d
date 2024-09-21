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

B2_ARRAY_SOURCE( b2SolverSet, b2SolverSet );

void b2DestroySolverSet( b2World* world, int setIndex )
{
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSetArray, setIndex );
	b2BodySimArray_Destroy( &set->simsNew );
	b2BodyStateArray_Destroy( &set->statesNew );
	b2ContactSimArray_Destroy( &set->contactsNew );
	b2JointSimArray_Destroy( &set->jointsNew );
	b2IslandSimArray_Destroy( &set->islandsNew );
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
	b2SolverSet* set = b2SolverSetArray_Get( &world->solverSetArray, setIndex );
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSetArray, b2_awakeSet );
	b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSetArray, b2_disabledSet );

	b2Body* bodies = world->bodyArrayNew.data;

	int bodyCount = set->simsNew.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2BodySim* simSrc = set->simsNew.data + i;

		b2Body* body = bodies + simSrc->bodyId;
		B2_ASSERT( body->setIndex == setIndex );
		body->setIndex = b2_awakeSet;
		body->localIndex = awakeSet->simsNew.count;

		// Reset sleep timer
		body->sleepTime = 0.0f;

		b2BodySim* simDst = b2BodySimArray_Add( &awakeSet->simsNew );
		memcpy( simDst, simSrc, sizeof( b2BodySim ) );

		b2BodyState* state = b2BodyStateArray_Add( &awakeSet->statesNew );
		*state = b2_identityBodyState;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2Contact* contact = b2ContactArray_Get( &world->contactArray, contactId );

			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b2_disabledSet )
			{
				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;
			b2ContactSim* contactSim = b2ContactSimArray_Get( &disabledSet->contactsNew, localIndex );

			B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 && contactSim->manifold.pointCount == 0 );

			contact->setIndex = b2_awakeSet;
			contact->localIndex = awakeSet->contactsNew.count;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Add( &awakeSet->contactsNew );
			memcpy( awakeContactSim, contactSim, sizeof( b2ContactSim ) );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( &disabledSet->contactsNew, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = disabledSet->contactsNew.data + localIndex;
				b2Contact* movedContact = b2ContactArray_Get( &world->contactArray, movedContactSim->contactId );
				B2_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = set->contactsNew.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSim = set->contactsNew.data + i;
			b2Contact* contact = b2ContactArray_Get( &world->contactArray, contactSim->contactId );
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
		int jointCount = set->jointsNew.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSim = set->jointsNew.data + i;
			b2Joint* joint = b2JointArray_Get( &world->jointArray, +jointSim->jointId );
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
		int islandCount = set->islandsNew.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set->islandsNew.data + i;
			b2Island* island = b2IslandArray_Get( &world->islandArray, islandSrc->islandId );
			island->setIndex = b2_awakeSet;
			island->localIndex = awakeSet->islandsNew.count;
			b2IslandSim* islandDst = b2IslandSimArray_Add( &awakeSet->islandsNew );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet( world, setIndex );

	b2ValidateSolverSets( world );
}

void b2TrySleepIsland( b2World* world, int islandId )
{
	b2Island* island = b2IslandArray_Get( &world->islandArray, islandId );
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
	if ( sleepSetId == world->solverSetArray.count )
	{
		b2SolverSet set = { 0 };
		set.setIndex = B2_NULL_INDEX;
		b2SolverSetArray_Push( &world->solverSetArray, set );
	}

	b2SolverSet* sleepSet = b2SolverSetArray_Get( &world->solverSetArray, sleepSetId );
	*sleepSet = ( b2SolverSet ){ 0 };

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSetArray, b2_awakeSet );
	B2_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandsNew.count );

	sleepSet->setIndex = sleepSetId;
	sleepSet->simsNew = b2BodySimArray_Create( island->bodyCount );
	sleepSet->contactsNew = b2ContactSimArray_Create( island->contactCount );
	sleepSet->jointsNew = b2JointSimArray_Create( island->jointCount );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = b2SolverSetArray_Get( &world->solverSetArray, b2_disabledSet );
		int bodyId = island->headBody;
		while ( bodyId != B2_NULL_INDEX )
		{
			b2Body* body = b2BodyArray_Get( &world->bodyArrayNew, bodyId );
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body->bodyMoveIndex != B2_NULL_INDEX )
			{
				b2BodyMoveEvent* moveEvent = b2BodyMoveEventArray_Get( &world->bodyMoveEventArray, body->bodyMoveIndex );
				B2_ASSERT( moveEvent->bodyId.index1 - 1 == bodyId );
				B2_ASSERT( moveEvent->bodyId.revision == body->revision );
				moveEvent->fellAsleep = true;
				body->bodyMoveIndex = B2_NULL_INDEX;
			}

			int awakeBodyIndex = body->localIndex;
			b2BodySim* awakeSim = b2BodySimArray_Get( &awakeSet->simsNew, awakeBodyIndex );

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->simsNew.count;
			b2BodySim* sleepBodySim = b2BodySimArray_Add( &sleepSet->simsNew );
			memcpy( sleepBodySim, awakeSim, sizeof( b2BodySim ) );

			int movedIndex = b2BodySimArray_RemoveSwap( &awakeSet->simsNew, awakeBodyIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix local index on moved element
				b2BodySim* movedSim = awakeSet->simsNew.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b2Body* movedBody = b2BodyArray_Get( &world->bodyArrayNew, movedId );
				B2_ASSERT( movedBody->localIndex == movedIndex );
				movedBody->localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			b2BodyStateArray_RemoveSwap( &awakeSet->statesNew, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2ContactArray_Get( &world->contactArray, contactId );

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
				b2Body* otherBody = b2BodyArray_Get( &world->bodyArrayNew, otherBodyId );
				if ( otherBody->setIndex == b2_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				b2ContactSim* contactSim = b2ContactSimArray_Get( &awakeSet->contactsNew, localIndex );

				B2_ASSERT( contactSim->manifold.pointCount == 0 );
				B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 || ( contact->flags & b2_contactSensorFlag ) != 0 );

				// move the non-touching contact to the disabled set
				contact->setIndex = b2_disabledSet;
				contact->localIndex = disabledSet->contactsNew.count;
				b2ContactSim* disabledContactSim = b2ContactSimArray_Add( &disabledSet->contactsNew );
				memcpy( disabledContactSim, contactSim, sizeof( b2ContactSim ) );

				int movedLocalIndex = b2ContactSimArray_RemoveSwap( &awakeSet->contactsNew, localIndex );
				if ( movedLocalIndex != B2_NULL_INDEX )
				{
					// fix moved element
					b2ContactSim* movedContactSim = awakeSet->contactsNew.data + localIndex;
					b2Contact* movedContact = b2ContactArray_Get( &world->contactArray, movedContactSim->contactId );
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
			b2Contact* contact = b2ContactArray_Get( &world->contactArray, contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );
			B2_ASSERT( contact->islandId == islandId );
			int colorIndex = contact->colorIndex;
			B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			// Remove bodies from graph coloring associated with this constraint
			if ( colorIndex != b2_overflowIndex )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color->bodySet, contact->edges[0].bodyId );
				b2ClearBit( &color->bodySet, contact->edges[1].bodyId );
			}

			int localIndex = contact->localIndex;
			b2ContactSim* awakeContactSim = b2ContactSimArray_Get( &color->contactSims, localIndex );

			int sleepContactIndex = sleepSet->contactsNew.count;
			b2ContactSim* sleepContactSim = b2ContactSimArray_Add( &sleepSet->contactsNew );
			memcpy( sleepContactSim, awakeContactSim, sizeof( b2ContactSim ) );

			int movedLocalIndex = b2ContactSimArray_RemoveSwap( &color->contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = color->contactSims.data + localIndex;
				b2Contact* movedContact = b2ContactArray_Get( &world->contactArray, movedContactSim->contactId );
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
			b2Joint* joint = b2JointArray_Get( &world->jointArray, jointId );
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->islandId == islandId );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			b2JointSim* awakeJointSim = b2JointSimArray_Get( &color->jointSims, localIndex );

			if ( colorIndex != b2_overflowIndex )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color->bodySet, joint->edges[0].bodyId );
				b2ClearBit( &color->bodySet, joint->edges[1].bodyId );
			}

			int sleepJointIndex = sleepSet->jointsNew.count;
			b2JointSim* sleepJointSim = b2JointSimArray_Add( &sleepSet->jointsNew );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b2JointSim ) );

			int movedIndex = b2JointSimArray_RemoveSwap( &color->jointSims, localIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2JointSim* movedJointSim = color->jointSims.data + localIndex;
				int movedId = movedJointSim->jointId;
				b2Joint* movedJoint = b2JointArray_Get( &world->jointArray, movedId );
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
		b2IslandSim* sleepIsland = b2IslandSimArray_Add( &sleepSet->islandsNew );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2IslandSimArray_RemoveSwap( &awakeSet->islandsNew, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = awakeSet->islandsNew.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2Island* movedIsland = b2IslandArray_Get( &world->islandArray, movedIslandId );
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
	b2SolverSet* set1 = b2SolverSetArray_Get( &world->solverSetArray, setId1 );
	b2SolverSet* set2 = b2SolverSetArray_Get( &world->solverSetArray, setId2 );

	// Move the fewest number of bodies
	if ( set1->simsNew.count < set2->simsNew.count )
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
		b2Body* bodies = world->bodyArrayNew.data;
		int bodyCount = set2->simsNew.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2BodySim* simSrc = set2->simsNew.data + i;

			b2Body* body = bodies + simSrc->bodyId;
			B2_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->simsNew.count;

			b2BodySim* simDst = b2BodySimArray_Add( &set1->simsNew );
			memcpy( simDst, simSrc, sizeof( b2BodySim ) );
		}
	}

	// transfer contacts
	{
		int contactCount = set2->contactsNew.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSrc = set2->contactsNew.data + i;

			b2Contact* contact = b2ContactArray_Get( &world->contactArray, contactSrc->contactId );
			B2_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactsNew.count;

			b2ContactSim* contactDst = b2ContactSimArray_Add( &set1->contactsNew );
			memcpy( contactDst, contactSrc, sizeof( b2ContactSim ) );
		}
	}

	// transfer joints
	{
		int jointCount = set2->jointsNew.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSrc = set2->jointsNew.data + i;

			b2Joint* joint = b2JointArray_Get( &world->jointArray, jointSrc->jointId );
			B2_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointsNew.count;

			b2JointSim* jointDst = b2JointSimArray_Add( &set1->jointsNew );
			memcpy( jointDst, jointSrc, sizeof( b2JointSim ) );
		}
	}

	// transfer islands
	{
		int islandCount = set2->islandsNew.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2->islandsNew.data + i;
			int islandId = islandSrc->islandId;

			b2Island* island = b2IslandArray_Get( &world->islandArray, islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandsNew.count;

			b2IslandSim* islandDst = b2IslandSimArray_Add( &set1->islandsNew );
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
	b2BodySim* sourceSim = b2BodySimArray_Get( &sourceSet->simsNew, sourceIndex );

	int targetIndex = targetSet->simsNew.count;
	b2BodySim* targetSim = b2BodySimArray_Add( &targetSet->simsNew );
	memcpy( targetSim, sourceSim, sizeof( b2BodySim ) );

	// Remove body sim from solver set that owns it
	int movedIndex = b2BodySimArray_RemoveSwap( &sourceSet->simsNew, sourceIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved body index
		b2BodySim* movedSim = sourceSet->simsNew.data + sourceIndex;
		int movedId = movedSim->bodyId;
		b2Body* movedBody = b2BodyArray_Get( &world->bodyArrayNew, movedId );
		B2_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = sourceIndex;
	}

	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2BodyStateArray_RemoveSwap( &sourceSet->statesNew, sourceIndex );
	}
	else if ( targetSet->setIndex == b2_awakeSet )
	{
		b2BodyState* state = b2BodyStateArray_Add( &targetSet->statesNew );
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
		B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;

		sourceSim = b2JointSimArray_Get( &color->jointSims, localIndex );
	}
	else
	{
		B2_ASSERT( colorIndex == B2_NULL_INDEX );
		sourceSim = b2JointSimArray_Get( &sourceSet->jointsNew, +localIndex );
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
		joint->localIndex = targetSet->jointsNew.count;
		joint->colorIndex = B2_NULL_INDEX;

		b2JointSim* targetSim = b2JointSimArray_Add( &targetSet->jointsNew );
		memcpy( targetSim, sourceSim, sizeof( b2JointSim ) );
	}

	// Destroy source.
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b2JointSimArray_RemoveSwap( &sourceSet->jointsNew, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// fix swapped element
			b2JointSim* movedJointSim = sourceSet->jointsNew.data + localIndex;
			int movedId = movedJointSim->jointId;
			b2Joint* movedJoint = b2JointArray_Get( &world->jointArray, movedId );
			movedJoint->localIndex = localIndex;
		}
	}
}
