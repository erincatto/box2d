// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/types.h"

struct GeometricMoverDef
{
	b2Pos position = {};
	b2Capsule capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };
	b2QueryFilter filter = b2DefaultQueryFilter();
	float jumpSpeed = 10.0f;
	float maxSpeed = 6.0f;
	float minSpeed = 0.1f;
	float stopSpeed = 3.0f;
	float accelerate = 20.0f;
	float airSteer = 0.2f;
	float friction = 8.0f;
	float gravity = 30.0f;
	float pogoRestLength = 0.9f;
	float pogoHertz = 5.0f;
	float pogoDampingRatio = 0.8f;
	float pogoCompressionScale = 100.0f;
	float pogoTensionScale = 100.0f;
};

// Optional shape user data read by GeometryMover when it gathers collision planes.
// Shapes without user data are treated as rigid and fully clipping.
struct MoverShapeUserData
{
	// Distance limit on the push this surface may apply to the mover. FLT_MAX is rigid.
	// Use a small value for surfaces that must not shove the mover, such as an elevator
	// that would otherwise push it through the floor.
	float maxPush;

	// False keeps the velocity along this surface, which is what soft collision needs.
	bool clipVelocity;
};

// Extra data kept alongside each collision plane so the mover can push rigid bodies.
struct PlaneExtra
{
	b2Pos point;
	b2ShapeId shapeId;
};

// Geometric character mover. The capsule is moved by casting
// and by solving collision planes, so the game keeps full control of the motion.
//
// A pogo spring holds the capsule above the ground so it can move up steps.
class GeometricMover
{
public:
	GeometricMover();

	// Place the mover. The capsule and tuning may be changed at any time.
	void Create( b2WorldId worldId, const GeometricMoverDef* def );

	// Advance the mover. Throttle is the horizontal input on [-1, 1].
	void Update( float timeStep, float throttle );

	// Jump if grounded. Returns false when airborne.
	bool Jump();

	static constexpr int m_planeCapacity = 8;

	float m_jumpSpeed;
	float m_maxSpeed;
	float m_minSpeed;
	float m_stopSpeed;
	float m_accelerate;

	// Fraction of the ground acceleration available in the air.
	float m_airSteer;

	// Ground friction in units of 1/time.
	float m_friction;

	// The mover is not simulated so it needs its own gravity.
	float m_gravity;

	float m_pogoRestLength;
	float m_pogoHertz;
	float m_pogoDampingRatio;

	// Downward force applied to whatever the pogo stick is resting on.
	float m_pogoGroundForce;

	// Surfaces steeper than this are not ground.
	float m_minGroundNormalY;

	// Push touched rigid bodies out of the way by solving a normal constraint against
	// each collision plane. Without this the mover slides along a crate instead of
	// shoving it.
	bool m_pushBodies;

	// The capsule is relative to the mover position.
	b2Capsule m_capsule;

	b2QueryFilter m_filter;

	b2WorldId m_worldId;

	// The center of the capsule.
	b2Pos m_position;

	b2Vec2 m_velocity;
	float m_pogoVelocity;
	bool m_onGround;

	// Planes from the last solve, kept for debug drawing.
	b2CollisionPlane m_planes[m_planeCapacity] = {};
	PlaneExtra m_planeExtras[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;

	b2Pos m_pogoOrigin;
	b2Vec2 m_pogoTranslation;
	float m_pogoFraction;
	bool m_pogoHit;

private:
	void PushBodies();
};
