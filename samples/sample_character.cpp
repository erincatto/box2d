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

class Mover : public Sample
{
public:
	explicit Mover( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 2.0f, 20.0f };
			g_camera.m_zoom = 21.0f;
		}

		m_transform = { { 7.5f, 5.75f}, b2Rot_identity };

		m_velocity = { 0.0f, 0.0f };
		m_capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -6.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[36];
			points[0] = { -20.58325, 14.54175 };
			points[1] = { -21.90625, 15.8645 };
			points[2] = { -24.552, 17.1875 };
			points[3] = { -27.198, 11.89575 };
			points[4] = { -29.84375, 15.8645 };
			points[5] = { -29.84375, 21.15625 };
			points[6] = { -25.875, 23.802 };
			points[7] = { -20.58325, 25.125 };
			points[8] = { -25.875, 29.09375 };
			points[9] = { -20.58325, 31.7395 };
			points[10] = { -11.0089998, 23.2290001 };
			points[11] = { -8.67700005, 21.15625 };
			points[12] = { -6.03125, 21.15625 };
			points[13] = { -7.35424995, 29.09375 };
			points[14] = { -3.38549995, 29.09375 };
			points[15] = { 1.90625, 30.41675 };
			points[16] = { 5.875, 17.1875 };
			points[17] = { 11.16675, 25.125 };
			points[18] = { 9.84375, 29.09375 };
			points[19] = { 13.8125, 31.7395 };
			points[20] = { 21.75, 30.41675 };
			points[21] = { 28.3644981, 26.448 };
			points[22] = { 25.71875, 18.5105 };
			points[23] = { 24.3957481, 13.21875 };
			points[24] = { 17.78125, 11.89575 };
			points[25] = { 15.1355, 7.92700005 };
			points[26] = { 5.875, 9.25 };
			points[27] = { 1.90625, 11.89575 };
			points[28] = { -2.25, 11.89575 };
			points[29] = { -3.25, 9.9375 };
			points[30] = { -4.70825005, 9.25 };
			points[31] = { -8.67700005, 9.25 };
			points[32] = { -11.323, 11.89575 };
			points[33] = { -13.96875, 11.89575 };
			points[34] = { -15.29175, 14.54175 };
			points[35] = { -19.2605, 14.54175 };

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 36;
			chainDef.isLoop = true;
			chainDef.filter = { StaticBit, AllBits, 0 };

			b2CreateChain( groundId, &chainDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 7.4f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.02f;
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
		b2Vec2 rayOrigin = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Vec2 rayTranslation = { 0.0f, -rayLength };
		b2QueryFilter skipTeamFilter = { 1, ~2u };
		b2RayResult rayResult = b2World_CastRayClosest( m_worldId, rayOrigin, rayTranslation, skipTeamFilter );

		if ( rayResult.hit == false )
		{
			m_onGround = false;
			m_pogoVelocity = 0.0f;

			g_draw.DrawSegment( rayOrigin, rayOrigin + rayTranslation, b2_colorGray );
		}
		else
		{
			m_onGround = true;
			float pogoCurrentLength = rayResult.fraction * rayLength;

			float zeta = 0.7f;
			float hertz = 6.0f;
			float omega = 2.0f * B2_PI * hertz;
			float omegaH = omega * timeStep;

			m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - pogoRestLength ) ) /
							 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );
			g_draw.DrawSegment(rayOrigin, rayResult.point, b2_colorGreen );
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
		g_draw.DrawSolidCapsule(p1, p2, m_capsule.radius, b2_colorOrange );
		g_draw.DrawSegment( m_transform.p, m_transform.p + m_velocity, b2_colorPurple );

		b2Vec2 p = m_transform.p;
		DrawTextLine( "position %.2f %.2f", p.x, p.y );
		DrawTextLine( "velocity %.2f %.2f", m_velocity.x, m_velocity.y );
		DrawTextLine( "iterations %d", m_totalIterations );
		DrawTextLine( "deltaX = %d, deltaY = %d", m_deltaX, m_deltaY );
	}

	static Sample* Create( Settings& settings )
	{
		return new Mover( settings );
	}

	static constexpr int m_planeCapacity = 8;
	static constexpr float m_jumpSpeed = 10.0f;
	static constexpr float m_maxSpeed = 6.0f;
	static constexpr float m_minSpeed = 0.01f;
	static constexpr float m_stopSpeed = 1.0f;
	static constexpr float m_accelerate = 30.0f;
	static constexpr float m_friction = 4.0f;
	static constexpr float m_gravity = 20.0f;

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
