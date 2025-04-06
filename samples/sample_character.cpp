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

enum PogoShape
{
	PogoPoint,
	PogoCircle,
	PogoBox
};

struct CastResult
{
	b2Vec2 point;
	b2BodyId bodyId;
	float fraction;
	bool hit;
};

static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	CastResult* result = (CastResult*)context;
	result->point = point;
	result->bodyId = b2Shape_GetBody( shapeId );
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

		settings.drawJoints = false;
		m_transform = { { 2.0f, 8.0f }, b2Rot_identity };
		m_velocity = { 0.0f, 0.0f };
		m_capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };

		b2BodyId groundId1;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			groundId1 = b2CreateBody( m_worldId, &bodyDef );

			const char* path =
				"M 2.6458333,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 "
				"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v "
				"-2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v "
				"-2.64584 h -5.291667 v -2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 H "
				"34.395833 L 23.8125,185.20833 H 15.875 L 5.2916669,187.85416 V 153.45833 H 2.6458333 v 47.625";

			b2Vec2 points[64];

			b2Vec2 offset = { -50.0f, -200.0f };
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = count;
			chainDef.isLoop = true;

			b2CreateChain( groundId1, &chainDef );
		}

		b2BodyId groundId2;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 98.0f, 0.0f };
			groundId2 = b2CreateBody( m_worldId, &bodyDef );

			const char* path =
				"M 2.6458333,201.08333 H 293.68751 l 0,-23.8125 h -23.8125 l 21.16667,21.16667 h -23.8125 l -39.68751,-13.22917 "
				"-26.45833,7.9375 -23.8125,2.64583 h -13.22917 l -0.0575,2.64584 h -5.29166 v -2.64583 l -7.86855,-1e-5 "
				"-0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l -2.64584,2.64583 -2.58891,-2.64584 h -13.28609 v "
				"-2.64583 h -2.64583 v -2.64584 l -5.29167,1e-5 v -2.64583 h -2.64583 v -2.64583 l -5.29167,-1e-5 v -2.64583 h "
				"-2.64583 v -2.64584 h -5.291667 v -2.64583 H 92.60417 V 174.625 h -5.291667 v -2.64584 l -34.395835,1e-5 "
				"-7.9375,-2.64584 -7.9375,-2.64583 -5.291667,-5.29167 H 21.166667 L 13.229167,158.75 5.2916668,153.45833 H "
				"2.6458334 l -10e-8,47.625";

			b2Vec2 points[64];

			b2Vec2 offset = { 0.0f, -200.0f };
			float scale = 0.2f;

			int count = ParsePath( path, offset, points, 64, scale, false );

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = count;
			chainDef.isLoop = true;

			b2CreateChain( groundId2, &chainDef );
		}

		{
			b2Polygon box = b2MakeBox( 0.5f, 0.125f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.maxMotorTorque = 10.0f;
			jointDef.enableMotor = true;
			jointDef.hertz = 3.0f;
			jointDef.dampingRatio = 0.8f;
			jointDef.enableSpring = true;

			float xBase = 48.7f;
			float yBase = 9.2f;
			int count = 50;
			b2BodyId prevBodyId = groundId1;
			for ( int i = 0; i < count; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { xBase + 0.5f + 1.0f * i, yBase };
				bodyDef.angularDamping = 0.2f;
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );

				b2Vec2 pivot = { xBase + 1.0f * i, yBase };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Vec2 pivot = { xBase + 1.0f * count, yBase };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = groundId2;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 32.0f, 4.0f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.025f;
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
		m_lockCamera = true;
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

		if ( m_onGround )
		{
			m_velocity.y = 0.0f;
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

		m_velocity.y -= m_gravity * timeStep;

		float pogoRestLength = 3.0f * m_capsule.radius;
		float rayLength = pogoRestLength + m_capsule.radius;
		b2Vec2 origin = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Circle circle = { origin, 0.5f * m_capsule.radius };
		float boxHalfWidth = 0.75f * m_capsule.radius;
		float boxHalfHeight = 0.05f * m_capsule.radius;
		b2Polygon box = b2MakeOffsetBox( boxHalfWidth, boxHalfHeight, origin, b2Rot_identity );
		b2Vec2 translation;
		b2QueryFilter skipTeamFilter = { 1, ~2u };
		CastResult result = {};

		if ( m_pogoShape == PogoPoint )
		{
			translation = { 0.0f, -rayLength };
			b2World_CastRay( m_worldId, origin, translation, skipTeamFilter, CastCallback, &result );
		}
		else if ( m_pogoShape == PogoCircle )
		{
			translation = { 0.0f, -rayLength + circle.radius };
			b2World_CastCircle( m_worldId, &circle, translation, skipTeamFilter, CastCallback, &result );
		}
		else
		{
			translation = { 0.0f, -rayLength + boxHalfHeight };
			b2World_CastPolygon( m_worldId, &box, translation, skipTeamFilter, CastCallback, &result );
		}

		if ( result.hit == false )
		{
			m_onGround = false;
			m_pogoVelocity = 0.0f;

			b2Vec2 delta = translation;
			g_draw.DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == PogoPoint )
			{
				g_draw.DrawPoint( origin + delta, 10.0f, b2_colorGray );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				g_draw.DrawCircle( origin + delta, circle.radius, b2_colorGray );
			}
			else
			{
				b2Transform xf = { delta, b2Rot_identity };
				g_draw.DrawSolidPolygon( xf, box.vertices, box.count, 0.0f, b2_colorGray );
			}
		}
		else
		{
			m_onGround = true;
			float pogoCurrentLength = result.fraction * rayLength;

			float zeta = m_pogoDampingRatio;
			float hertz = m_pogoHertz;
			float omega = 2.0f * B2_PI * hertz;
			float omegaH = omega * timeStep;

			m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - pogoRestLength ) ) /
							 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );

			b2Vec2 delta = result.fraction * translation;
			g_draw.DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == PogoPoint )
			{
				g_draw.DrawPoint( origin + delta, 10.0f, b2_colorPlum );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				g_draw.DrawCircle( origin + delta, circle.radius, b2_colorPlum );
			}
			else
			{
				b2Transform xf = { delta, b2Rot_identity };
				g_draw.DrawSolidPolygon( xf, box.vertices, box.count, 0.0f, b2_colorPlum );
			}

			b2Body_ApplyForce( result.bodyId, { 0.0f, -50.0f }, result.point, true );
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

	void UpdateGui() override
	{
		float height = 350.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 25.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 340.0f, height ) );

		ImGui::Begin( "Mover", nullptr, 0 );

		ImGui::PushItemWidth( 240.0f );

		ImGui::SliderFloat( "Jump Speed", &m_jumpSpeed, 0.0f, 40.0f, "%.0f" );
		ImGui::SliderFloat( "Min Speed", &m_minSpeed, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Max Speed", &m_maxSpeed, 0.0f, 20.0f, "%.0f" );
		ImGui::SliderFloat( "Stop Speed", &m_stopSpeed, 0.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Accelerate", &m_accelerate, 0.0f, 100.0f, "%.0f" );
		ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Gravity", &m_gravity, 0.0f, 100.0f, "%.1f" );
		ImGui::SliderFloat( "Air Steer", &m_airSteer, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Pogo Hertz", &m_pogoHertz, 0.0f, 30.0f, "%.0f" );
		ImGui::SliderFloat( "Pogo Damping", &m_pogoDampingRatio, 0.0f, 4.0f, "%.1f" );

		ImGui::PopItemWidth();

		ImGui::Separator();

		ImGui::Text( "Pogo Shape" );
		ImGui::RadioButton( "Point", &m_pogoShape, PogoPoint );
		ImGui::SameLine();
		ImGui::RadioButton( "Circle", &m_pogoShape, PogoCircle );
		ImGui::SameLine();
		ImGui::RadioButton( "Box", &m_pogoShape, PogoBox );

		ImGui::Checkbox( "Lock Camera", &m_lockCamera );

		ImGui::End();
	}

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

		if ( m_lockCamera )
		{
			g_camera.m_center.x = m_transform.p.x;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Mover( settings );
	}

	static constexpr int m_planeCapacity = 8;
	float m_jumpSpeed = 10.0f;
	float m_maxSpeed = 6.0f;
	float m_minSpeed = 0.1f;
	float m_stopSpeed = 3.0f;
	float m_accelerate = 20.0f;
	float m_airSteer = 0.2f;
	float m_friction = 8.0f;
	float m_gravity = 30.0f;
	float m_pogoHertz = 5.0f;
	float m_pogoDampingRatio = 0.8f;

	int m_pogoShape = PogoBox;
	b2Transform m_transform;
	b2Vec2 m_velocity;
	b2Capsule m_capsule;
	ShapeUserData m_friendlyShape;
	b2CollisionPlane m_planes[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	bool m_onGround;
	bool m_lockCamera;
};

static int sampleMover = RegisterSample( "Character", "Mover", Mover::Create );
