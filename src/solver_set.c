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

void b2DestroySolverSet( b2World* world, int setIndex )
{
	b2SolverSet* set = world->solverSetArray + setIndex;
	b2DestroyBodySimArray( &set->sims );
	b2DestroyBodyStateArray( &set->states );
	b2DestroyContactArray( &set->contacts );
	b2DestroyJointArray( &set->joints );
	b2DestroyIslandArray( &set->islands );
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
	b2CheckIndex( world->solverSetArray, setIndex );
	b2SolverSet* set = world->solverSetArray + setIndex;
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;
	b2SolverSet* disabledSet = world->solverSetArray + b2_disabledSet;

	b2Body* bodies = world->bodyArray;
	b2Contact* contacts = world->contactArray;

	int bodyCount = set->sims.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2BodySim* simSrc = set->sims.data + i;

		b2Body* body = bodies + simSrc->bodyId;
		B2_ASSERT( body->setIndex == setIndex );
		body->setIndex = b2_awakeSet;
		body->localIndex = awakeSet->sims.count;

		// Reset sleep timer
		body->sleepTime = 0.0f;

		b2BodySim* simDst = b2AddBodySim( &awakeSet->sims );
		memcpy( simDst, simSrc, sizeof( b2BodySim ) );

		b2BodyState* state = b2AddBodyState( &awakeSet->states );
		*state = b2_identityBodyState;

		// move non-touching contacts from disabled set to awake set
		int contactKey = body->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			int edgeIndex = contactKey & 1;
			int contactId = contactKey >> 1;

			b2CheckIndex( contacts, contactId );
			b2Contact* contact = contacts + contactId;

			contactKey = contact->edges[edgeIndex].nextKey;

			if ( contact->setIndex != b2_disabledSet )
			{
				B2_ASSERT( contact->setIndex == b2_awakeSet || contact->setIndex == setIndex );
				continue;
			}

			int localIndex = contact->localIndex;
			B2_ASSERT( 0 <= localIndex && localIndex < disabledSet->contacts.count );
			b2ContactSim* contactSim = disabledSet->contacts.data + localIndex;

			B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 && contactSim->manifold.pointCount == 0 );

			contact->setIndex = b2_awakeSet;
			contact->localIndex = awakeSet->contacts.count;
			b2ContactSim* awakeContactSim = b2AddContact( &awakeSet->contacts );
			memcpy( awakeContactSim, contactSim, sizeof( b2ContactSim ) );

			int movedLocalIndex = b2RemoveContact( &disabledSet->contacts, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContact = disabledSet->contacts.data + localIndex;
				int movedId = movedContact->contactId;
				b2CheckIndex( contacts, movedId );
				B2_ASSERT( contacts[movedId].localIndex == movedLocalIndex );
				contacts[movedId].localIndex = localIndex;
			}
		}
	}

	// transfer touching contacts from sleeping set to contact graph
	{
		int contactCount = set->contacts.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSim = set->contacts.data + i;
			b2Contact* contact = contacts + contactSim->contactId;
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
		b2Joint* joints = world->jointArray;
		int jointCount = set->joints.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSim = set->joints.data + i;
			b2Joint* joint = joints + jointSim->jointId;
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
		b2Island* islands = world->islandArray;
		int islandCount = set->islands.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set->islands.data + i;
			b2CheckIndex( islands, islandSrc->islandId );
			b2Island* island = islands + islandSrc->islandId;
			island->setIndex = b2_awakeSet;
			island->localIndex = awakeSet->islands.count;
			b2IslandSim* islandDst = b2AddIsland( &awakeSet->islands );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// destroy the sleeping set
	b2DestroySolverSet( world, setIndex );

	b2ValidateSolverSets( world );
}

void b2TrySleepIsland( b2World* world, int islandId )
{
	b2CheckIndex( world->islandArray, islandId );
	b2Island* island = world->islandArray + islandId;
	B2_ASSERT( island->setIndex == b2_awakeSet );

	// cannot put an island to sleep while it has a pending split
	if ( island->constraintRemoveCount > 0 )
	{
		return;
	}

	b2BodyMoveEvent* moveEvents = world->bodyMoveEventArray;

	// island is sleeping
	// - create new sleeping solver set
	// - move island to sleeping solver set
	// - identify non-touching contacts that should move to sleeping solver set or disabled set
	// - remove old island
	// - fix island
	int sleepSetId = b2AllocId( &world->solverSetIdPool );
	if ( sleepSetId == b2Array( world->solverSetArray ).count )
	{
		b2SolverSet set = { 0 };
		set.setIndex = B2_NULL_INDEX;
		b2Array_Push( world->solverSetArray, set );
	}

	b2SolverSet* sleepSet = world->solverSetArray + sleepSetId;
	*sleepSet = ( b2SolverSet ){ 0 };

	// grab awake set after creating the sleep set because the solver set array may have been resized
	b2SolverSet* awakeSet = world->solverSetArray + b2_awakeSet;
	B2_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islands.count );

	sleepSet->setIndex = sleepSetId;
	sleepSet->sims = b2CreateBodySimArray( island->bodyCount );
	sleepSet->contacts = b2CreateContactArray( island->contactCount );
	sleepSet->joints = b2CreateJointArray( island->jointCount );

	// move awake bodies to sleeping set
	// this shuffles around bodies in the awake set
	{
		b2SolverSet* disabledSet = world->solverSetArray + b2_disabledSet;
		b2Body* bodies = world->bodyArray;
		b2Contact* contacts = world->contactArray;
		int bodyId = island->headBody;
		while ( bodyId != B2_NULL_INDEX )
		{
			b2CheckIndex( bodies, bodyId );
			b2Body* body = bodies + bodyId;
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );

			// Update the body move event to indicate this body fell asleep
			// It could happen the body is forced asleep before it ever moves.
			if ( body->bodyMoveIndex != B2_NULL_INDEX )
			{
				b2CheckIndex( moveEvents, body->bodyMoveIndex );
				B2_ASSERT( moveEvents[body->bodyMoveIndex].bodyId.index1 - 1 == bodyId );
				B2_ASSERT( moveEvents[body->bodyMoveIndex].bodyId.revision == body->revision );
				moveEvents[body->bodyMoveIndex].fellAsleep = true;
				body->bodyMoveIndex = B2_NULL_INDEX;
			}

			int awakeBodyIndex = body->localIndex;
			B2_ASSERT( 0 <= awakeBodyIndex && awakeBodyIndex < awakeSet->sims.count );

			b2BodySim* awakeSim = awakeSet->sims.data + awakeBodyIndex;

			// move body sim to sleep set
			int sleepBodyIndex = sleepSet->sims.count;
			b2BodySim* sleepBodySim = b2AddBodySim( &sleepSet->sims );
			memcpy( sleepBodySim, awakeSim, sizeof( b2BodySim ) );

			int movedIndex = b2RemoveBodySim( &awakeSet->sims, awakeBodyIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix local index on moved element
				b2BodySim* movedSim = awakeSet->sims.data + awakeBodyIndex;
				int movedId = movedSim->bodyId;
				b2CheckIndex( bodies, movedId );
				b2Body* movedBody = bodies + movedId;
				B2_ASSERT( movedBody->localIndex == movedIndex );
				movedBody->localIndex = awakeBodyIndex;
			}

			// destroy state, no need to clone
			b2RemoveBodyState( &awakeSet->states, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// Move non-touching contacts to the disabled set.
			// Non-touching contacts may exist between sleeping islands and there is no clear ownership.
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2CheckIndex( contacts, contactId );
				b2Contact* contact = contacts + contactId;

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
				b2CheckIndex( bodies, otherBodyId );
				b2Body* otherBody = bodies + otherBodyId;
				if ( otherBody->setIndex == b2_awakeSet )
				{
					continue;
				}

				int localIndex = contact->localIndex;
				B2_ASSERT( 0 <= localIndex && localIndex < awakeSet->contacts.count );
				b2ContactSim* contactSim = awakeSet->contacts.data + localIndex;

				B2_ASSERT( contactSim->manifold.pointCount == 0 );
				B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 || ( contact->flags & b2_contactSensorFlag ) != 0 );

				// move the non-touching contact to the disabled set
				contact->setIndex = b2_disabledSet;
				contact->localIndex = disabledSet->contacts.count;
				b2ContactSim* disabledContactSim = b2AddContact( &disabledSet->contacts );
				memcpy( disabledContactSim, contactSim, sizeof( b2ContactSim ) );

				int movedContactIndex = b2RemoveContact( &awakeSet->contacts, localIndex );
				if ( movedContactIndex != B2_NULL_INDEX )
				{
					// fix moved element
					b2ContactSim* movedContactSim = awakeSet->contacts.data + localIndex;
					int movedId = movedContactSim->contactId;
					b2CheckIndex( contacts, movedId );
					B2_ASSERT( contacts[movedId].localIndex == movedContactIndex );
					contacts[movedId].localIndex = localIndex;
				}
			}

			bodyId = body->islandNext;
		}
	}

	// move touching contacts
	// this shuffles contacts in the awake set
	{
		b2Contact* contacts = world->contactArray;
		int contactId = island->headContact;
		while ( contactId != B2_NULL_INDEX )
		{
			b2CheckIndex( contacts, contactId );
			b2Contact* contact = contacts + contactId;
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

			int awakeContactIndex = contact->localIndex;
			B2_ASSERT( 0 <= awakeContactIndex && awakeContactIndex < color->contacts.count );
			b2ContactSim* awakeContactSim = color->contacts.data + awakeContactIndex;

			int sleepContactIndex = sleepSet->contacts.count;
			b2ContactSim* sleepContactSim = b2AddContact( &sleepSet->contacts );
			memcpy( sleepContactSim, awakeContactSim, sizeof( b2ContactSim ) );

			int movedIndex = b2RemoveContact( &color->contacts, awakeContactIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2ContactSim* movedContactSim = color->contacts.data + awakeContactIndex;
				int movedId = movedContactSim->contactId;
				b2CheckIndex( contacts, movedId );
				b2Contact* movedContact = contacts + movedId;
				B2_ASSERT( movedContact->localIndex == movedIndex );
				movedContact->localIndex = awakeContactIndex;
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
		b2Joint* joints = world->jointArray;
		int jointId = island->headJoint;
		while ( jointId != B2_NULL_INDEX )
		{
			b2CheckIndex( joints, jointId );
			b2Joint* joint = joints + jointId;
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->islandId == islandId );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B2_ASSERT( 0 <= colorIndex && colorIndex < b2_graphColorCount );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			B2_ASSERT( 0 <= localIndex && localIndex < color->joints.count );
			b2JointSim* awakeJointSim = color->joints.data + localIndex;

			if ( colorIndex != b2_overflowIndex )
			{
				// might clear a bit for a static body, but this has no effect
				b2ClearBit( &color->bodySet, joint->edges[0].bodyId );
				b2ClearBit( &color->bodySet, joint->edges[1].bodyId );
			}

			int sleepJointIndex = sleepSet->joints.count;
			b2JointSim* sleepJointSim = b2AddJoint( &sleepSet->joints );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b2JointSim ) );

			int movedIndex = b2RemoveJoint( &color->joints, localIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// fix moved element
				b2JointSim* movedJointSim = color->joints.data + localIndex;
				int movedId = movedJointSim->jointId;
				b2CheckIndex( joints, movedId );
				b2Joint* movedJoint = joints + movedId;
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
		b2IslandSim* sleepIsland = b2AddIsland( &sleepSet->islands );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2RemoveIsland( &awakeSet->islands, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// fix index on moved element
			b2IslandSim* movedIslandSim = awakeSet->islands.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2CheckIndex( world->islandArray, movedIslandId );
			b2Island* movedIsland = world->islandArray + movedIslandId;
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
	b2CheckIndex( world->solverSetArray, setId1 );
	b2CheckIndex( world->solverSetArray, setId2 );
	b2SolverSet* set1 = world->solverSetArray + setId1;
	b2SolverSet* set2 = world->solverSetArray + setId2;

	// Move the fewest number of bodies
	if ( set1->sims.count < set2->sims.count )
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
		b2Body* bodies = world->bodyArray;
		int bodyCount = set2->sims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2BodySim* simSrc = set2->sims.data + i;

			b2Body* body = bodies + simSrc->bodyId;
			B2_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->sims.count;

			b2BodySim* simDst = b2AddBodySim( &set1->sims );
			memcpy( simDst, simSrc, sizeof( b2BodySim ) );
		}
	}

	// transfer contacts
	{
		b2Contact* contacts = world->contactArray;
		int contactCount = set2->contacts.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSrc = set2->contacts.data + i;

			b2Contact* contact = contacts + contactSrc->contactId;
			B2_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contacts.count;

			b2ContactSim* contactDst = b2AddContact( &set1->contacts );
			memcpy( contactDst, contactSrc, sizeof( b2ContactSim ) );
		}
	}

	// transfer joints
	{
		b2Joint* joints = world->jointArray;
		int jointCount = set2->joints.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSrc = set2->joints.data + i;

			b2Joint* joint = joints + jointSrc->jointId;
			B2_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->joints.count;

			b2JointSim* jointDst = b2AddJoint( &set1->joints );
			memcpy( jointDst, jointSrc, sizeof( b2JointSim ) );
		}
	}

	// transfer islands
	{
		b2Island* islands = world->islandArray;
		int islandCount = set2->islands.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2->islands.data + i;
			int islandId = islandSrc->islandId;

			b2CheckIndex( islands, islandId );
			b2Island* island = islands + islandId;
			island->setIndex = setId1;
			island->localIndex = set1->islands.count;

			b2IslandSim* islandDst = b2AddIsland( &set1->islands );
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
	B2_ASSERT( 0 <= sourceIndex && sourceIndex <= sourceSet->sims.count );
	b2BodySim* sourceSim = sourceSet->sims.data + sourceIndex;

	int targetIndex = targetSet->sims.count;
	b2BodySim* targetSim = b2AddBodySim( &targetSet->sims );
	memcpy( targetSim, sourceSim, sizeof( b2BodySim ) );

	// Remove body sim from solver set that owns it
	int movedIndex = b2RemoveBodySim( &sourceSet->sims, sourceIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// Fix moved body index
		b2BodySim* movedSim = sourceSet->sims.data + sourceIndex;
		int movedId = movedSim->bodyId;
		b2Body* movedBody = world->bodyArray + movedId;
		B2_ASSERT( movedBody->localIndex == movedIndex );
		movedBody->localIndex = sourceIndex;
	}

	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2RemoveBodyState( &sourceSet->states, sourceIndex );
	}
	else if ( targetSet->setIndex == b2_awakeSet )
	{
		b2BodyState* state = b2AddBodyState( &targetSet->states );
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

		B2_ASSERT( 0 <= localIndex && localIndex < color->joints.count );
		sourceSim = color->joints.data + localIndex;
	}
	else
	{
		B2_ASSERT( colorIndex == B2_NULL_INDEX );
		B2_ASSERT( 0 <= localIndex && localIndex < sourceSet->joints.count );
		sourceSim = sourceSet->joints.data + localIndex;
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
		joint->localIndex = targetSet->joints.count;
		joint->colorIndex = B2_NULL_INDEX;

		b2JointSim* targetSim = b2AddJoint( &targetSet->joints );
		memcpy( targetSim, sourceSim, sizeof( b2JointSim ) );
	}

	// Destroy source.
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b2RemoveJoint( &sourceSet->joints, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// fix swapped element
			b2JointSim* movedJointSim = sourceSet->joints.data + localIndex;
			int movedId = movedJointSim->jointId;
			b2CheckIndex( world->jointArray, movedId );
			b2Joint* movedJoint = world->jointArray + movedId;
			movedJoint->localIndex = localIndex;
		}
	}
}
