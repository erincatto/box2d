// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_mover.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

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

DynamicMover::DynamicMover()
{
	m_jumpSpeed = 5.0f;
	m_maxSpeed = 6.0f;
	m_minSpeed = 0.1f;
	m_stopSpeed = 3.0f;
	m_accelerate = 20.0f;
	m_airSteer = 0.2f;
	m_friction = 8.0f;
	m_gravityScale = 1.5f;
	m_maxGroundForce = 70.0f;
	m_maxAirForce = 20.0f;
	m_pogoRestLength = 0.9f;
	m_pogoHertz = 5.0f;
	m_pogoDampingRatio = 0.8f;
	m_pogoCompressionScale = 100.0f;
	m_pogoTensionScale = 100.0f;
	m_minGroundNormalY = 0.7f;

	m_capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };

	m_filter = b2DefaultFilter();
	m_worldId = b2_nullWorldId;
	m_groundId = b2_nullBodyId;
	m_moverId = b2_nullBodyId;
	m_moverJointId = b2_nullJointId;
	m_pogoJointId = b2_nullJointId;

	m_velocity = b2Vec2_zero;
	m_onGround = false;
	m_walkable = false;
	m_jumping = false;
	m_jumpTicks = 0;

	m_pogoImpulse = 0.0f;
	m_pogoVelocity = 0.0f;
	m_pogoLength = 0.0f;

	m_pogoOrigin = b2Pos_zero;
	m_pogoTranslation = b2Vec2_zero;
	m_pogoFraction = 1.0f;
	m_pogoHit = false;

	m_castResult = {};
}

void DynamicMover::Create( b2WorldId worldId, const DynamicMoverDef* def )
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
	m_gravityScale = def->gravityScale;
	m_maxGroundForce = def->maxGroundForce;
	m_maxAirForce = def->maxAirForce;
	m_pogoRestLength = def->pogoRestLength;
	m_pogoHertz = def->pogoHertz;
	m_pogoDampingRatio = def->pogoDampingRatio;
	m_pogoCompressionScale = def->pogoCompressionScale;
	m_pogoTensionScale = def->pogoTensionScale;
	m_minGroundNormalY = 0.7f;

	m_worldId = worldId;
	m_velocity = b2Vec2_zero;
	m_onGround = false;
	m_walkable = false;
	m_jumping = false;
	m_pogoImpulse = 0.0f;
	m_pogoVelocity = 0.0f;
	m_pogoLength = 0.0f;
	m_pogoJointId = b2_nullJointId;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	m_groundId = b2CreateBody( worldId, &bodyDef );

	// Position is the center of the capsule.
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = def->position;
	bodyDef.gravityScale = m_gravityScale;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.enableSleep = false;
	bodyDef.name = "mover";

	m_moverId = b2CreateBody( worldId, &bodyDef );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = def->density;

	// The mover joint and the pogo joint handle all the movement, surface friction would fight them
	shapeDef.material.friction = 0.0f;
	shapeDef.filter = def->filter;
	shapeDef.enablePreSolveEvents = def->enablePreSolveEvents;

	b2CreateCapsuleShape( m_moverId, &shapeDef, &m_capsule );

	b2MoverJointDef moverDef = b2DefaultMoverJointDef();
	moverDef.linearVelocity = m_velocity;
	moverDef.maxVelocityForce = { m_maxGroundForce, 0.0f };
	moverDef.base.bodyIdA = m_groundId;
	moverDef.base.bodyIdB = m_moverId;
	moverDef.base.collideConnected = true;

	m_moverJointId = b2CreateMoverJoint( worldId, &moverDef );
}

void DynamicMover::Destroy()
{
	if ( B2_IS_NON_NULL( m_moverId ) )
	{
		b2DestroyBody( m_moverId );
	}

	if ( B2_IS_NON_NULL( m_groundId ) )
	{
		b2DestroyBody( m_groundId );
	}

	m_groundId = b2_nullBodyId;
	m_moverId = b2_nullBodyId;

	// Joints are implicitly destroyed.
	m_moverJointId = b2_nullJointId;
	m_pogoJointId = b2_nullJointId;
}

void DynamicMover::SetGravityScale( float gravityScale )
{
	m_gravityScale = gravityScale;

	if ( B2_IS_NON_NULL( m_moverId ) )
	{
		b2Body_SetGravityScale( m_moverId, gravityScale );
	}
}

bool DynamicMover::Jump()
{
	if ( m_onGround == false || m_walkable == false )
	{
		return false;
	}

	float surfaceVelocity = 0.0f;
	if (b2Body_IsValid(m_castResult.bodyId))
	{
		b2Vec2 v = b2Body_GetWorldPointVelocity( m_castResult.bodyId, m_castResult.point );
		surfaceVelocity = v.y;
	}

	float mass = b2Body_GetMass( m_moverId );
	float vy = b2Body_GetLinearVelocity( m_moverId ).y;

	// Remove the pogo constraint velocity but add the surface velocity.
	float dv = b2MaxFloat( 0.0f, m_jumpSpeed - vy ) + surfaceVelocity;
	b2Body_ApplyLinearImpulseToCenter( m_moverId, { 0.0f, mass * dv }, true );

	// This removes the pogo step down on the next update.
	m_onGround = false;
	m_walkable = false;
	m_jumping = true;
	m_jumpTicks = 0;
	return true;
}

// Movement follows the Quake ground and air model:
// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390

// onGround - controls friction, mover max force, and pogo step-down extension
// walkable - controls air steer
// jumping - blocks the pogo from pulling down
void DynamicMover::Update( float timeStep, float throttle )
{
	// Reach further while grounded so the spring can find the ground over a step
	float stepDownLength = m_pogoRestLength;
	float rayLength = m_onGround ? m_pogoRestLength + stepDownLength : m_pogoRestLength;
	b2Vec2 translation = { 0.0f, -rayLength };

	b2Pos position = b2Body_GetPosition( m_moverId );
	b2Pos origin = position + m_capsule.center1;

	b2QueryFilter queryFilter = {
		.categoryBits = m_filter.categoryBits,
		.maskBits = m_filter.maskBits,
	};

	m_castResult = {};
	b2World_CastRay( m_worldId, origin, translation, queryFilter, CastCallback, &m_castResult );

	m_pogoOrigin = origin;
	m_pogoTranslation = translation;
	m_pogoFraction = m_castResult.hit ? m_castResult.fraction : 1.0f;
	m_pogoHit = m_castResult.hit;

	// Should jumping end?
	if ( m_jumping )
	{
		m_jumpTicks += 1;

		// Jump ticks allow time for the jump to leave the ground. Otherwise jumping ends
		// when the character approaches the current ground.
		if ( m_jumpTicks > 2 && m_castResult.hit && b2Dot( m_velocity, m_castResult.normal ) <= 0.0f )
		{
			m_jumping = false;
		}
	}

	if ( m_castResult.hit )
	{
		// The cast can still hit while jumping.
		m_onGround = m_jumping == false;

		// Is the ground too steep to walk?
		m_walkable = m_castResult.normal.y > m_minGroundNormalY;
	}
	else
	{
		m_onGround = false;
		m_walkable = false;
	}

		m_velocity = b2Body_GetLinearVelocity( m_moverId );

	// Friction
	if ( m_onGround )
	{
		float speed = b2Length( m_velocity );
		if ( speed < m_minSpeed )
		{
			m_velocity.x = 0.0f;
		}
		else
		{
			// Linear damping above stopSpeed and fixed reduction below stopSpeed
			float control = speed < m_stopSpeed ? m_stopSpeed : speed;

			// friction has units of 1/time
			float drop = control * m_friction * timeStep;
			float newSpeed = b2MaxFloat( 0.0f, speed - drop );
			m_velocity.x *= newSpeed / speed;
		}
	}

	// Mover force
	if ( m_onGround )
	{
		float maxForce = m_walkable ? m_maxGroundForce : 0.0f;
		b2MoverJoint_SetMaxVelocityForce( m_moverJointId, { maxForce, 0.0f } );
	}
	else
	{
		b2MoverJoint_SetMaxVelocityForce( m_moverJointId, { m_maxAirForce, 0.0f } );
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
		float steer = m_walkable ? 1.0f : m_airSteer;
		float accelSpeed = steer * m_accelerate * m_maxSpeed * timeStep;
		if ( accelSpeed > addSpeed )
		{
			accelSpeed = addSpeed;
		}

		m_velocity.x += accelSpeed * desiredDirection.x;
	}

	// The pogo joint is rebuilt every solve because it may land on a different body
	if ( b2Joint_IsValid( m_pogoJointId ) )
	{
		m_pogoLength = b2PogoJoint_GetLength( m_pogoJointId );
		m_pogoImpulse = b2PogoJoint_GetImpulse( m_pogoJointId );
		m_pogoVelocity = b2PogoJoint_GetVelocity( m_pogoJointId );

		b2DestroyJoint( m_pogoJointId );
		m_pogoJointId = b2_nullJointId;
	}

	if ( m_castResult.hit == true )
	{
		float moverMass = b2Body_GetMass( m_moverId );
		float moverWeight = m_gravityScale * b2Length( b2World_GetGravity( m_worldId ) ) * moverMass;

		b2PogoJointDef pogoDef = b2DefaultPogoJointDef();
		pogoDef.base.localFrameA.p = b2Body_GetLocalPoint( m_castResult.bodyId, m_castResult.point );
		pogoDef.base.localFrameB.p = m_capsule.center1;
		pogoDef.normal = m_castResult.normal;
		pogoDef.base.bodyIdA = m_castResult.bodyId;
		pogoDef.base.bodyIdB = m_moverId;
		pogoDef.base.collideConnected = true;
		pogoDef.restLength = m_pogoRestLength;
		pogoDef.hertz = m_pogoHertz;
		pogoDef.dampingRatio = m_pogoDampingRatio;
		pogoDef.maxCompressionForce = m_pogoCompressionScale * moverWeight;

		// Warm start from the joint that was just destroyed
		pogoDef.impulse = m_pogoImpulse;
		pogoDef.velocity = m_pogoVelocity;

		if ( m_jumping )
		{
			// Don't allow the pogo to pull down
			pogoDef.maxTensionForce = 0.0f;
		}
		else
		{
			// The pogo can pull down at a multiple of the gravity force.
			pogoDef.maxTensionForce = m_pogoTensionScale * moverWeight;
		}

		m_pogoJointId = b2CreatePogoJoint( m_worldId, &pogoDef );
	}
	else
	{
		m_pogoImpulse = 0.0f;
		m_pogoVelocity = 0.0f;
	}

	b2MoverJoint_SetLinearVelocity( m_moverJointId, m_velocity );
}
