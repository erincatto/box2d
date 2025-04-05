// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

struct ShapeUserData
{
	float maxPush;
	bool clipVelocity;
};

enum CollisionBits : uint64_t
{
	StaticBit = 0x0001,
	MoverBit = 0x0002,
	DynamicBit = 0x0004,

	AllBits = ~0u,
};

struct CastResult
{
	b2Vec2 point;
	float fraction;
	bool hit;
};

static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	CastResult* result = (CastResult*)context;
	result->point = point;
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

class Mover : public Sample
{
public:
	explicit Mover( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 20.0f, 9.0f };
			g_camera.m_zoom = 10.0f;
		}

		m_transform = { { 2.0f, 8.0f }, b2Rot_identity };

		m_velocity = { 0.0f, 0.0f };
		m_capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			const char* path =
				"M 2.6458333,201.08333 H 293.68751 l -10e-6,-55.5625 h -2.64584 l 2e-5,52.91667 h -23.8125 l -39.68751,-13.22917 "
				"h -26.45833 l -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v -2.64583 h -2.64583 v "
				"-2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v -2.64584 h -5.291667 v "
				"-2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 H 34.395833 L "
				"23.8125,185.20833 H 15.875 L 5.2916669,187.85416 V 153.45833 H 2.6458333 v 47.625";

			b2Vec2 points[64];

			b2Vec2 offset = { -50.0f, -200.0f };
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = count;
			chainDef.isLoop = true;

			b2CreateChain( groundId, &chainDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 32.0f, 4.0f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.05f;
			m_friendlyShape.clipVelocity = false;

			shapeDef.filter = { MoverBit, AllBits, 0 };
			shapeDef.userData = &m_friendlyShape;
			b2BodyId body = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( body, &shapeDef, &m_capsule );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 7.0f, 7.0f };
			b2BodyId body = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };
			b2Circle circle = { b2Vec2_zero, 0.5f };
			b2CreateCircleShape( body, &shapeDef, &circle );
		}

		m_totalIterations = 0;
		m_pogoVelocity = 0.0f;
		m_onGround = false;

		m_deltaX = 0.0f;
		m_deltaY = 0.0f;
		m_planeCount = 0;
	}

	// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
	void SolveMove( float timeStep, float throttle )
	{
		// Friction
		float speed = b2Length( m_velocity );
		if ( speed < m_minSpeed )
		{
			m_velocity.x = 0.0f;
			m_velocity.y = 0.0f;
		}
		else
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

		if ( m_onGround )
		{
			m_velocity.y = 0.0f;
		}

		// Accelerate
		float currentSpeed = b2Dot( m_velocity, desiredDirection );
		float addSpeed = desiredSpeed - currentSpeed;
		if ( addSpeed > 0.0f )
		{
			float accelSpeed = m_accelerate * m_maxSpeed * timeStep;
			if ( accelSpeed > addSpeed )
			{
				accelSpeed = addSpeed;
			}

			m_velocity += accelSpeed * desiredDirection;
		}

		m_velocity.y -= m_gravity * timeStep;

		float pogoRestLength = 3.0f * m_capsule.radius;
		float rayLength = pogoRestLength + m_capsule.radius;
		b2Circle circle = { b2TransformPoint( m_transform, m_capsule.center1 ), 0.5f * m_capsule.radius };
		b2Vec2 translation = { 0.0f, -rayLength + circle.radius };
		b2QueryFilter skipTeamFilter = { 1, ~2u };
		CastResult result = {};
		b2World_CastCircle( m_worldId, &circle, translation, skipTeamFilter, CastCallback, &result );

		if ( result.hit == false )
		{
			m_onGround = false;
			m_pogoVelocity = 0.0f;

			g_draw.DrawSegment( circle.center, circle.center + translation, b2_colorGray );
			g_draw.DrawCircle( circle.center + translation, circle.radius, b2_colorGray );
		}
		else
		{
			m_onGround = true;
			float pogoCurrentLength = result.fraction * rayLength;

			float zeta = 0.7f;
			float hertz = 6.0f;
			float omega = 2.0f * B2_PI * hertz;
			float omegaH = omega * timeStep;

			m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - pogoRestLength ) ) /
							 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );

			b2Vec2 delta = result.fraction * translation;
			g_draw.DrawSegment( circle.center, circle.center + delta, b2_colorPlum );
			g_draw.DrawCircle( circle.center + delta, circle.radius, b2_colorPlum );
		}

		b2Vec2 target = m_transform.p + timeStep * m_velocity + timeStep * m_pogoVelocity * b2Vec2{ 0.0f, 1.0f };

		// Movers collide with every thing
		b2QueryFilter collideFilter = { MoverBit, AllBits };

		// Movers don't sweep against other movers, allows for soft collision
		b2QueryFilter castFilter = { MoverBit, StaticBit | DynamicBit };

		m_totalIterations = 0;
		float tolerance = 0.01f;

		for ( int iteration = 0; iteration < 5; ++iteration )
		{
			m_planeCount = 0;

			b2Capsule mover;
			mover.center1 = b2TransformPoint( m_transform, m_capsule.center1 );
			mover.center2 = b2TransformPoint( m_transform, m_capsule.center2 );
			mover.radius = m_capsule.radius;

			b2World_CollideMover( m_worldId, &mover, collideFilter, PlaneResultFcn, this );
			b2PlaneSolverResult result = b2SolvePlanes( target, m_planes, m_planeCount );

			m_totalIterations += result.iterationCount;

			b2Vec2 moverTranslation = result.position - m_transform.p;

			float fraction = b2World_CastMover( m_worldId, &mover, moverTranslation, castFilter );

			b2Vec2 delta = fraction * moverTranslation;
			m_transform.p += delta;

			if ( b2LengthSquared( delta ) < tolerance * tolerance )
			{
				break;
			}
		}

		m_velocity = b2ClipVector( m_velocity, m_planes, m_planeCount );
	}

#if 0
	void UpdateGui() override
	{
		ImGui::SetNextWindowPos( ImVec2( 10.0f, 600.0f ) );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, 80.0f ) );
		ImGui::Begin( "Mover", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );


		ImGui::End();
	}
#endif

	static bool PlaneResultFcn( b2ShapeId shapeId, const b2PlaneResult* planeResult, void* context )
	{
		assert( planeResult->hit == true );

		Mover* self = static_cast<Mover*>( context );
		float maxPush = FLT_MAX;
		bool clipVelocity = true;
		ShapeUserData* userData = static_cast<ShapeUserData*>( (void*)b2Shape_GetUserData( shapeId ) );
		if ( userData != nullptr )
		{
			maxPush = userData->maxPush;
			clipVelocity = userData->clipVelocity;
		}

		if ( self->m_planeCount < m_planeCapacity )
		{
			assert( b2IsValidPlane( planeResult->plane ) );
			self->m_planes[self->m_planeCount] = { planeResult->plane, maxPush, 0.0f, clipVelocity };
			self->m_planeCount += 1;
		}

		return true;
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float throttle = 0.0f;

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) )
		{
			throttle -= 1.0f;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) )
		{
			throttle += 1.0f;
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_SPACE ) && m_onGround == true )
		{
			m_velocity.y = m_jumpSpeed;
			m_onGround = false;
		}

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		// throttle = { 0.0f, 0.0f, -1.0f };

		SolveMove( timeStep, throttle );

		int count = m_planeCount;
		for ( int i = 0; i < count; ++i )
		{
			b2Plane plane = m_planes[i].plane;
			b2Vec2 p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
			b2Vec2 p2 = p1 + 0.1f * plane.normal;
			g_draw.DrawPoint( p1, 5.0f, b2_colorYellow );
			g_draw.DrawSegment( p1, p2, b2_colorYellow );
		}

		b2Vec2 p1 = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Vec2 p2 = b2TransformPoint( m_transform, m_capsule.center2 );
		g_draw.DrawSolidCapsule( p1, p2, m_capsule.radius, b2_colorOrange );
		g_draw.DrawSegment( m_transform.p, m_transform.p + m_velocity, b2_colorPurple );

		b2Vec2 p = m_transform.p;
		DrawTextLine( "position %.2f %.2f", p.x, p.y );
		DrawTextLine( "velocity %.2f %.2f", m_velocity.x, m_velocity.y );
		DrawTextLine( "iterations %d", m_totalIterations );
		DrawTextLine( "deltaX = %d, deltaY = %d", m_deltaX, m_deltaY );

		g_camera.m_center.x = m_transform.p.x;
	}

	static Sample* Create( Settings& settings )
	{
		return new Mover( settings );
	}

	static constexpr int m_planeCapacity = 8;
	static constexpr float m_jumpSpeed = 10.0f;
	static constexpr float m_maxSpeed = 4.0f;
	static constexpr float m_minSpeed = 0.01f;
	static constexpr float m_stopSpeed = 1.0f;
	static constexpr float m_accelerate = 20.0f;
	static constexpr float m_friction = 2.0f;
	static constexpr float m_gravity = 15.0f;

	b2Transform m_transform;
	b2Vec2 m_velocity;
	b2Capsule m_capsule;
	ShapeUserData m_friendlyShape;
	b2CollisionPlane m_planes[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	bool m_onGround;

	int m_deltaX;
	int m_deltaY;
};

static int sampleMover = RegisterSample( "Character", "Mover", Mover::Create );
