// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometric_mover.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>
#include <float.h>

struct CastResult
{
	b2Pos point;
	b2Vec2 normal;
	b2BodyId bodyId;
	float fraction;
	bool hit;
};

static float CastCallback( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	CastResult* result = (CastResult*)context;
	result->point = point;
	result->normal = normal;
	result->bodyId = b2Shape_GetBody( shapeId );
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

static bool PlaneResultFcn( b2ShapeId shapeId, const b2PlaneResult* planeResult, void* context )
{
	assert( planeResult->hit == true );

	GeometricMover* self = static_cast<GeometricMover*>( context );
	float maxPush = FLT_MAX;
	bool clipVelocity = true;
	MoverShapeUserData* userData = static_cast<MoverShapeUserData*>( (void*)b2Shape_GetUserData( shapeId ) );
	if ( userData != nullptr )
	{
		maxPush = userData->maxPush;
		clipVelocity = userData->clipVelocity;
	}

	if ( self->m_planeCount < GeometricMover::m_planeCapacity )
	{
		assert( b2IsValidPlane( planeResult->plane ) );
		self->m_planes[self->m_planeCount] = { planeResult->plane, maxPush, 0.0f, clipVelocity };

		// The plane point is relative to the origin passed to the collide call
		self->m_planeExtras[self->m_planeCount] = {
			.point = b2OffsetPos( self->m_position, planeResult->point ),
			.shapeId = shapeId,
		};
		self->m_planeCount += 1;
	}

	return true;
}

GeometricMover::GeometricMover()
{
	m_jumpSpeed = 10.0f;
	m_maxSpeed = 6.0f;
	m_minSpeed = 0.1f;
	m_stopSpeed = 3.0f;
	m_accelerate = 20.0f;
	m_airSteer = 0.2f;
	m_friction = 8.0f;
	m_gravity = 30.0f;
	m_pogoRestLength = 0.9f;
	m_pogoHertz = 5.0f;
	m_pogoDampingRatio = 0.8f;
	m_pogoGroundForce = 50.0f;
	m_pushBodies = true;

	m_capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };

	m_filter = b2DefaultQueryFilter();

	m_worldId = b2_nullWorldId;
	m_position = b2Pos_zero;
	m_velocity = b2Vec2_zero;
	m_pogoVelocity = 0.0f;
	m_minGroundNormalY = 0.7f;
	m_onGround = false;

	m_planeCount = 0;
	m_totalIterations = 0;

	m_pogoOrigin = b2Pos_zero;
	m_pogoTranslation = b2Vec2_zero;
	m_pogoFraction = 1.0f;
	m_pogoHit = false;
}

void GeometricMover::Create( b2WorldId worldId, const GeometricMoverDef* def )
{
	m_capsule = def->capsule;
	m_filter = def->filter;
	m_jumpSpeed = def->jumpSpeed;
	m_maxSpeed = def->maxSpeed;
	m_minSpeed = def->minSpeed;
	m_stopSpeed = def->stopSpeed;
	m_accelerate = def->accelerate;
	m_airSteer = def->airSteer;
	m_friction = def->friction;
	m_gravity = def->gravity;
	m_pogoRestLength = def->pogoRestLength;
	m_pogoHertz = def->pogoHertz;
	m_pogoDampingRatio = def->pogoDampingRatio;
	m_minGroundNormalY = 0.7f;

	m_worldId = worldId;
	m_position = def->position;
	m_velocity = b2Vec2_zero;
	m_pogoVelocity = 0.0f;
	m_onGround = false;
	m_planeCount = 0;
	m_totalIterations = 0;
	m_pogoFraction = 1.0f;
	m_pogoHit = false;
}

bool GeometricMover::Jump()
{
	if ( m_onGround == false )
	{
		return false;
	}

	m_velocity.y = m_jumpSpeed;
	m_onGround = false;
	return true;
}

// Solve a normal constraint between the mover and each touched rigid body. This is the
// same velocity constraint the solver uses for a contact, except the mover is treated as
// having infinite mass, so all of the impulse lands on the body.
void GeometricMover::PushBodies()
{
	for ( int i = 0; i < m_planeCount; ++i )
	{
		b2BodyId bodyId = b2Shape_GetBody( m_planeExtras[i].shapeId );
		if ( b2Body_GetType( bodyId ) != b2_dynamicBody )
		{
			continue;
		}

		// Plane normals point at the mover
		b2Pos point = m_planeExtras[i].point;
		b2Vec2 normal = -m_planes[i].plane.normal;

		float invMassA = 0.0f;

		float massB = b2Body_GetMass( bodyId );
		float invMassB = massB > 0.0f ? 1.0f / massB : 0.0f;

		float inertiaB = b2Body_GetRotationalInertia( bodyId );
		float invIB = inertiaB > 0.0f ? 1.0f / inertiaB : 0.0f;

		b2Pos centerB = b2Body_GetWorldCenter( bodyId );
		b2Vec2 rB = point - centerB;

		float rnB = b2Cross( rB, normal );
		float kNormal = invMassA + invMassB + invIB * rnB * rnB;
		float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

		b2Vec2 vB = b2Body_GetLinearVelocity( bodyId );
		float omegaB = b2Body_GetAngularVelocity( bodyId );
		b2Vec2 vrB = vB + b2CrossSV( omegaB, rB );
		float vn = b2Dot( vrB - m_velocity, normal );

		// Push only, a separating body is left alone
		float impulse = b2MaxFloat( -normalMass * vn, 0.0f );

		b2Vec2 P = impulse * normal;
		m_velocity = b2MulSub( m_velocity, invMassA, P );

		b2Body_ApplyLinearImpulse( bodyId, P, point, true );
	}
}

// Movement follows the Quake ground and air model:
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
void GeometricMover::Update( float timeStep, float throttle )
{
	// Friction
	float speed = b2Length( m_velocity );
	if ( speed < m_minSpeed )
	{
		m_velocity.x = 0.0f;
		m_velocity.y = 0.0f;
	}
	else if ( m_onGround )
	{
		// Linear damping above stopSpeed and fixed reduction below stopSpeed
		float control = speed < m_stopSpeed ? m_stopSpeed : speed;

		// friction has units of 1/time
		float drop = control * m_friction * timeStep;
		float newSpeed = b2MaxFloat( 0.0f, speed - drop );
		m_velocity *= newSpeed / speed;
	}

	b2Vec2 desiredVelocity = { m_maxSpeed * throttle, 0.0f };
	float desiredSpeed;
	b2Vec2 desiredDirection = b2GetLengthAndNormalize( &desiredSpeed, desiredVelocity );

	if ( desiredSpeed > m_maxSpeed )
	{
		desiredSpeed = m_maxSpeed;
	}

	// Accelerate
	float currentSpeed = b2Dot( m_velocity, desiredDirection );
	float addSpeed = desiredSpeed - currentSpeed;
	if ( addSpeed > 0.0f )
	{
		float steer = m_onGround ? 1.0f : m_airSteer;
		float accelSpeed = steer * m_accelerate * m_maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m_velocity += accelSpeed * desiredDirection;
	}

	// The probe reaches past the rest length so the spring can be stretched
	float rayLength = m_onGround ? 2.0f * m_pogoRestLength : m_pogoRestLength;
	b2Vec2 translation = { 0.0f, -rayLength };
	b2Pos origin = m_position + m_capsule.center1;

	CastResult castResult = {};
	b2World_CastRay( m_worldId, origin, translation, m_filter, CastCallback, &castResult );

	m_pogoOrigin = origin;
	m_pogoTranslation = translation;
	m_pogoFraction = castResult.hit ? castResult.fraction : 1.0f;
	m_pogoHit = castResult.hit;

	// Avoid snapping to ground if still going up
	bool haveGround = castResult.hit && castResult.normal.y > m_minGroundNormalY;

	if ( m_onGround == false )
	{
		m_onGround = haveGround && m_velocity.y <= 0.01f;
	}
	else
	{
		m_onGround = haveGround;
	}

	if ( m_onGround == false )
	{
		m_velocity.y -= m_gravity * timeStep;
		m_pogoVelocity = 0.0f;
	}
	else
	{
		m_velocity.y = 0.0f;

		float pogoCurrentLength = castResult.fraction * rayLength;

		float offset = pogoCurrentLength - m_pogoRestLength;
		m_pogoVelocity = b2SpringDamper( m_pogoHertz, m_pogoDampingRatio, offset, m_pogoVelocity, timeStep );

		b2Body_ApplyForce( castResult.bodyId, { 0.0f, -m_pogoGroundForce }, castResult.point, true );
	}

	b2Pos target = m_position + timeStep * m_velocity + timeStep * m_pogoVelocity * b2Vec2{ 0.0f, 1.0f };

	m_totalIterations = 0;
	float tolerance = 0.01f;

	// Don't cast against other movers.
	b2QueryFilter castFilter = {
		.categoryBits = m_filter.categoryBits,
		.maskBits = ( m_filter.maskBits & ~m_filter.categoryBits ),
	};

	for ( int iteration = 0; iteration < 5; ++iteration )
	{
		m_planeCount = 0;

		b2World_CollideMover( m_worldId, m_position, &m_capsule, m_filter, PlaneResultFcn, this );
		b2PlaneSolverResult result = b2SolvePlanes( target - m_position, m_planes, m_planeCount );

		m_totalIterations += result.iterationCount;

		float fraction = b2World_CastMover( m_worldId, m_position, &m_capsule, result.delta, castFilter );

		b2Vec2 delta = fraction * result.delta;
		m_position = m_position + delta;

		if ( b2LengthSquared( delta ) < tolerance * tolerance )
		{
			break;
		}
	}

	if ( m_pushBodies )
	{
		PushBodies();
	}

	// Stop pushing into surfaces so speed can't accumulate against a wall
	m_velocity = b2ClipVector( m_velocity, m_planes, m_planeCount );
}
