// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/types.h"

struct DynamicMoverDef
{
	b2Pos position = {};
	b2Capsule capsule = {};
	float density = 1.0f;
	b2Filter filter = b2DefaultFilter();
	float jumpSpeed = 7.0f;
	float maxSpeed = 6.0f;
	float minSpeed = 0.1f;
	float stopSpeed = 3.0f;
	float accelerate = 20.0f;
	float airSteer = 0.5f;
	float friction = 8.0f;
	float gravityScale = 1.5f;
	float maxGroundForce = 70.0f;
	float maxAirForce = 20.0f;
	float pogoRestLength = 0.9f;
	float pogoHertz = 5.0f;
	float pogoDampingRatio = 0.8f;
	float pogoCompressionScale = 100.0f;
	float pogoTensionScale = 100.0f;
	bool enablePreSolveEvents = false;
};

struct DynamicMoverCastResult
{
	b2Pos point;
	b2Vec2 normal;
	b2BodyId bodyId;
	float fraction;
	bool hit;
};

// Dynamic character mover. The capsule is a rigid body with a locked rotation, so it
// collides and is pushed like everything else in the world. A mover joint drives it to
// the target velocity and a pogo joint holds it above the ground so it can climb
// steps without touching the capsule.
//
// The dynamic mover should be vendored into your project so you can alter it to meet
// your game's specific needs.
class DynamicMover
{
public:
	DynamicMover();

	void Create( b2WorldId worldId, const DynamicMoverDef* def );
	void Destroy();

	// Advance the mover. Throttle is the horizontal input on [-1, 1].
	void Update( float timeStep, float throttle );

	// Jump if grounded. Returns false when airborne.
	bool Jump();

	// Gravity scale is used to size the pogo forces, so change it here.
	void SetGravityScale( float gravityScale );

	float m_jumpSpeed;
	float m_maxSpeed;
	float m_minSpeed;
	float m_stopSpeed;
	float m_accelerate;

	// Fraction of the ground acceleration available in the air.
	float m_airSteer;

	// Ground friction in units of 1/time.
	float m_friction;

	// A character usually wants to fall faster than the rest of the world.
	float m_gravityScale;

	// Force budget the mover joint has for reaching the target velocity. The air value
	// is small so the character keeps its momentum while jumping.
	float m_maxGroundForce;
	float m_maxAirForce;

	float m_pogoRestLength;
	float m_pogoHertz;
	float m_pogoDampingRatio;

	// Pogo force limits as a multiple of the mover weight.
	float m_pogoCompressionScale;
	float m_pogoTensionScale;

	// Surfaces steeper than this are not ground.
	float m_minGroundNormalY;

	b2Capsule m_capsule;
	b2Filter m_filter;

	b2WorldId m_worldId;
	b2BodyId m_groundId;
	b2BodyId m_moverId;
	b2JointId m_moverJointId;
	b2JointId m_pogoJointId;

	DynamicMoverCastResult m_castResult;

	b2Vec2 m_velocity;
	bool m_onGround;
	bool m_walkable;
	bool m_jumping;
	int m_jumpTicks;

	// Pogo joint state, carried across the joint being recreated each solve.
	float m_pogoImpulse;
	float m_pogoVelocity;
	float m_pogoLength;

	// Ground probe from the last solve, kept for debug drawing. The probe ends at
	// the origin plus the fraction of the translation.
	b2Pos m_pogoOrigin;
	b2Vec2 m_pogoTranslation;
	float m_pogoFraction;
	bool m_pogoHit;
};
