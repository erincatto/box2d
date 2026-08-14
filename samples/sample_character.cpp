// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "dynamic_mover.h"
#include "geometric_mover.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

enum CollisionBits : uint64_t
{
	StaticBit = 0x0001,
	MoverBit = 0x0002,
	DynamicBit = 0x0004,

	AllBits = ~0u,
};

// The ground probe is a point, a circle, or a segment, all carried by the proxy.
static void DrawPogo( Draw* draw, b2Pos origin, b2Vec2 translation, float fraction, bool hit )
{
	b2HexColor color = hit ? b2_colorPlum : b2_colorGray;
	b2Vec2 delta = fraction * translation;

	DrawLine( draw, origin, origin + delta, b2_colorGray );
	DrawPoint( draw, origin + delta, 10.0f, color );
}

class GeometryMoverSample : public Sample
{
public:
	explicit GeometryMoverSample( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->center = { 20.0f, 9.0f };
			m_camera->zoom = 10.0f;
		}

		context->debugDraw.drawJoints = false;

		m_mover.m_collideFilter = { MoverBit, StaticBit | DynamicBit | MoverBit };

		// Movers don't sweep against other movers, allows for soft collision
		m_mover.m_castFilter = { MoverBit, StaticBit | DynamicBit };
		m_mover.m_pogoFilter = { MoverBit, StaticBit | DynamicBit };
		m_mover.Create( m_worldId, { 2.0f, 8.0f } );

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

				b2Pos pivot = { xBase + 1.0f * i, yBase };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = bodyId;
				jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Pos pivot = { xBase + 1.0f * count, yBase };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = groundId2;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 32.0f, 4.5f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.025f;
			m_friendlyShape.clipVelocity = false;

			shapeDef.filter = { MoverBit, AllBits, 0 };
			shapeDef.userData = &m_friendlyShape;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId, &shapeDef, &m_mover.m_capsule );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 7.0f, 7.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			b2Circle circle = { b2Vec2_zero, 0.4f };
			m_ballId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { m_elevatorBase.x, m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = b2CreateBody( m_worldId, &bodyDef );

			// The elevator must not push the mover through the floor
			m_elevatorShape = {
				.maxPush = 0.1f,
				.clipVelocity = true,
			};
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };
			shapeDef.userData = &m_elevatorShape;

			b2Polygon box = b2MakeBox( 2.0f, 0.1f );
			b2CreatePolygonShape( m_elevatorId, &shapeDef, &box );
		}

		m_jumpReleased = true;
		m_lockCamera = true;
		m_time = 0.0f;
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Jump Speed", &m_mover.m_jumpSpeed, 0.0f, 40.0f, "%.0f" );
		ImGui::SliderFloat( "Min Speed", &m_mover.m_minSpeed, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Max Speed", &m_mover.m_maxSpeed, 0.0f, 20.0f, "%.0f" );
		ImGui::SliderFloat( "Stop Speed", &m_mover.m_stopSpeed, 0.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Accelerate", &m_mover.m_accelerate, 0.0f, 100.0f, "%.0f" );
		ImGui::SliderFloat( "Friction", &m_mover.m_friction, 0.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Gravity", &m_mover.m_gravity, 0.0f, 100.0f, "%.1f" );
		ImGui::SliderFloat( "Air Steer", &m_mover.m_airSteer, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Pogo Hertz", &m_mover.m_pogoHertz, 0.0f, 30.0f, "%.0f" );
		ImGui::SliderFloat( "Pogo Damping", &m_mover.m_pogoDampingRatio, 0.0f, 4.0f, "%.1f" );

		ImGui::Separator();

		ImGui::Checkbox( "Push Bodies", &m_mover.m_pushBodies );
		ImGui::Checkbox( "Lock Camera", &m_lockCamera );

		return true;
	}

	static bool Kick( b2ShapeId shapeId, void* context )
	{
		GeometryMoverSample* self = (GeometryMoverSample*)context;
		b2BodyId bodyId = b2Shape_GetBody( shapeId );
		b2BodyType type = b2Body_GetType( bodyId );

		if ( type != b2_dynamicBody )
		{
			return true;
		}

		b2Pos center = b2Body_GetWorldCenter( bodyId );
		b2Vec2 direction = b2Normalize( center - self->m_mover.m_position );
		b2Vec2 impulse = b2Vec2{ 2.0f * direction.x, 2.0f };
		b2Body_ApplyLinearImpulseToCenter( bodyId, impulse, true );

		return true;
	}

	void Step() override
	{
		DrawScreenTextLine( "left/right/jump = A/D/W" );

		bool pause = false;
		if ( m_context->pause )
		{
			pause = m_context->singleStep != true;
		}

		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( pause )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f )
		{
			b2Pos point = {
				.x = m_elevatorBase.x,
				.y = m_elevatorAmplitude * cosf( 1.0f * m_time + B2_PI ) + m_elevatorBase.y,
			};

			bool wake = true;
			b2Body_SetTargetTransform( m_elevatorId, { point, b2Rot_identity }, timeStep, wake );
		}

		m_time += timeStep;

		Sample::Step();

		if ( pause == false )
		{
			float throttle = 0.0f;

			if ( glfwGetKey( m_context->window, GLFW_KEY_A ) )
			{
				throttle -= 1.0f;
			}

			if ( glfwGetKey( m_context->window, GLFW_KEY_D ) )
			{
				throttle += 1.0f;
			}

			if ( glfwGetKey( m_context->window, GLFW_KEY_W ) )
			{
				if ( m_jumpReleased && m_mover.Jump() )
				{
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			m_mover.Solve( timeStep, throttle );
		}

		DrawPogo( m_draw, m_mover.m_pogoOrigin, m_mover.m_pogoTranslation, m_mover.m_pogoFraction, m_mover.m_pogoHit );

		DrawTransform( m_draw, { m_mover.m_position, b2Rot_identity }, 0.25f );

		b2Capsule capsule = m_mover.m_capsule;

		int count = m_mover.m_planeCount;
		for ( int i = 0; i < count; ++i )
		{
			b2Plane plane = m_mover.m_planes[i].plane;
			b2Pos p1 = m_mover.m_position + ( plane.offset - capsule.radius ) * plane.normal;
			b2Pos p2 = p1 + 0.1f * plane.normal;
			DrawPoint( m_draw, p1, 5.0f, b2_colorYellow );
			DrawLine( m_draw, p1, p2, b2_colorYellow );
		}

		b2Pos p1 = m_mover.m_position + capsule.center1;
		b2Pos p2 = m_mover.m_position + capsule.center2;

		b2HexColor color = m_mover.m_onGround ? b2_colorOrange : b2_colorAquamarine;
		DrawCapsule( m_draw, p1, p2, capsule.radius, color );
		DrawLine( m_draw, m_mover.m_position, m_mover.m_position + m_mover.m_velocity, b2_colorPurple );

		b2Pos p = m_mover.m_position;
		DrawScreenTextLine( "position %.2f %.2f", p.x, p.y );
		DrawScreenTextLine( "velocity %.2f %.2f", m_mover.m_velocity.x, m_mover.m_velocity.y );
		DrawScreenTextLine( "iterations %d", m_mover.m_totalIterations );

		if ( m_lockCamera )
		{
			m_camera->center.x = m_mover.m_position.x;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new GeometryMoverSample( context );
	}

	static constexpr b2Vec2 m_elevatorBase = { 112.0f, 10.0f };
	static constexpr float m_elevatorAmplitude = 4.0f;

	GeometryMover m_mover;
	b2BodyId m_elevatorId;
	b2ShapeId m_ballId;
	MoverShapeUserData m_friendlyShape;
	MoverShapeUserData m_elevatorShape;
	float m_time;
	bool m_jumpReleased;
	bool m_lockCamera;
};

static int sampleGeometricMover = RegisterSample( "Character", "Geometric Mover", GeometryMoverSample::Create );

class DynamicMoverSample : public Sample
{
public:
	explicit DynamicMoverSample( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->center = { 20.0f, 9.0f };
			m_camera->zoom = 10.0f;
		}

		context->debugDraw.drawJoints = false;

		// This is the mover the player controls
		{
			DynamicMoverDef def;
			def.position = { 0.0f, 8.0f };
			def.filter.categoryBits = MoverBit;
			def.capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.3f };
			m_mover.m_pogoFilter = { MoverBit, StaticBit | DynamicBit };
			m_mover.Create( m_worldId, &def );
		}

		m_elevatorId = b2_nullBodyId;

		b2BodyId groundId1;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.name = "svg";
			bodyDef.position = { 0.0f, 0.0f };
			groundId1 = b2CreateBody( m_worldId, &bodyDef );

#if 0
			const char* path = "M -34.4,201.08 H 293.69 V 34.4 L -21.17,198.44 H -31.75 l -0,-92.6 h -2.65 l 0,95.25";
#else
			const char* path =
				"M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 "
				"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 -15.875,2e-5 v -3.96875 l -17.19792,2e-5 v 2.64582 l "
				"-17.19791,0 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 "
				"h -3.96875 v -1.32292 h -3.96875 v -1.32291 l -3.968754,0 v -1.32292 h -3.96875 v -1.32292 h -3.968751 v "
				"-1.32291 h -3.96875 v -1.32292 h -3.96875 v -1.32292 h -3.96875 v -1.32291 h -3.96875 v -1.32292 l -3.96875,0 v "
				"-1.32292 h -3.968751 v -1.32291 h -3.96875 l -2e-6,-1.32294 H 52.916669 L 39.6875,177.27083 h -5.291667 l "
				"-7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25";
#endif

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
				"M 2.6458333,201.08333 H 399.52085 v -23.8125 h -23.8125 l 21.16667,18.52084 -232.83335,-1e-5 -0.0575,2.64584 h "
				"-5.29166 v -2.64583 l -7.86855,-1e-5 -0.0114,-2.64583 h -2.64583 l -2.64583,2.64584 h -7.9375 l "
				"-2.64584,2.64583 -2.58891,-2.64584 h -13.28609 v -2.64583 h -2.64583 v -2.64584 l -5.29167,1e-5 v -2.64583 h "
				"-2.64583 v -2.64583 l -5.29167,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -5.291667 v -2.64583 H 92.60417 V "
				"174.625 h -5.291667 v -2.64584 l -34.395835,1e-5 -7.9375,-2.64584 -7.9375,-2.64583 -5.291667,-5.29167 H "
				"21.166667 L 13.229167,158.75 5.2916668,153.45833 H 2.6458334 l -10e-8,47.625";

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

#if 1
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

				b2Pos pivot = { xBase + 1.0f * i, yBase };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = bodyId;
				jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Pos pivot = { xBase + 1.0f * count, yBase };
			jointDef.base.bodyIdA = prevBodyId;
			jointDef.base.bodyIdB = groundId2;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		for ( int i = 0; i < 5; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 7.0f, 7.0f + 0.5f * i };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			b2Circle circle = { b2Vec2_zero, 0.25f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { m_elevatorBase.x, m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };

			b2Polygon box = b2MakeBox( 2.0f, 0.1f );
			b2CreatePolygonShape( m_elevatorId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position.x = 140.0f;
			float a = 0.25f;
			b2Polygon square = b2MakeSquare( a );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DynamicBit, AllBits, 0 };
			for ( int i = 0; i < 10; ++i )
			{
				bodyDef.position.y = ( 2.0f * i + 1.0f ) * a + 1.0f;
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );
			}
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 160.0f, 5.0f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId body = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.1f, 4.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( body, &shapeDef, &box );

			b2Pos pivot = bodyDef.position + b2Vec2{ 0.0f, 4.0f };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = groundId2;
			jointDef.base.bodyIdB = body;
			jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 1.0f;
			jointDef.maxMotorTorque = 500.0f;

			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}
#endif

		m_jumpReleased = true;
		m_lockCamera = true;
		m_time = 0.0f;
	}

	bool DrawControls() override
	{
		ImGui::SliderFloat( "Jump Speed", &m_mover.m_jumpSpeed, 0.0f, 40.0f, "%.0f" );
		ImGui::SliderFloat( "Min Speed", &m_mover.m_minSpeed, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Max Speed", &m_mover.m_maxSpeed, 0.0f, 20.0f, "%.0f" );
		ImGui::SliderFloat( "Stop Speed", &m_mover.m_stopSpeed, 0.0f, 10.0f, "%.1f" );
		ImGui::SliderFloat( "Accelerate", &m_mover.m_accelerate, 0.0f, 100.0f, "%.0f" );
		ImGui::SliderFloat( "Friction", &m_mover.m_friction, 0.0f, 10.0f, "%.1f" );

		float gravityScale = m_mover.m_gravityScale;
		if ( ImGui::SliderFloat( "Gravity Scale", &gravityScale, 0.0f, 4.0f, "%.1f" ) )
		{
			m_mover.SetGravityScale( gravityScale );
		}

		ImGui::SliderFloat( "Air Steer", &m_mover.m_airSteer, 0.0f, 1.0f, "%.2f" );
		ImGui::SliderFloat( "Pogo Hertz", &m_mover.m_pogoHertz, 0.0f, 30.0f, "%.0f" );
		ImGui::SliderFloat( "Pogo Damping", &m_mover.m_pogoDampingRatio, 0.0f, 4.0f, "%.1f" );

		ImGui::Separator();

		ImGui::Checkbox( "Lock Camera", &m_lockCamera );

		return true;
	}

	void Step() override
	{
		DrawScreenTextLine( "left/right/jump = A/D/W" );

		bool pause = false;
		if ( m_context->pause )
		{
			pause = m_context->singleStep != true;
		}

		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( pause )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f && B2_IS_NON_NULL( m_elevatorId ) )
		{
			b2Pos point = {
				.x = m_elevatorBase.x,
				.y = m_elevatorAmplitude * cosf( 1.0f * m_time + B2_PI ) + m_elevatorBase.y,
			};

			bool wake = true;
			b2Body_SetTargetTransform( m_elevatorId, { point, b2Rot_identity }, timeStep, wake );
		}

		m_time += timeStep;

		Sample::Step();

		if ( pause == false )
		{
			float throttle = 0.0f;

			if ( glfwGetKey( m_context->window, GLFW_KEY_A ) )
			{
				throttle -= 1.0f;
			}

			if ( glfwGetKey( m_context->window, GLFW_KEY_D ) )
			{
				throttle += 1.0f;
			}

			if ( glfwGetKey( m_context->window, GLFW_KEY_W ) )
			{
				if ( m_jumpReleased && m_mover.Jump() )
				{
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			m_mover.Solve( timeStep, throttle );
		}

		DrawPogo( m_draw, m_mover.m_pogoOrigin, m_mover.m_pogoTranslation, m_mover.m_pogoFraction, m_mover.m_pogoHit );

		b2Pos position = b2Body_GetPosition( m_mover.m_moverId );

		b2HexColor color = m_mover.m_onGround ? b2_colorOrange : b2_colorAquamarine;
		DrawLine( m_draw, position, position + m_mover.m_velocity, color );

		DrawScreenTextLine( "position %.2f %.2f", position.x, position.y );
		DrawScreenTextLine( "velocity %.2f %.2f", m_mover.m_velocity.x, m_mover.m_velocity.y );
		DrawScreenTextLine( "pogo: impulse %.3f, velocity %.3f", m_mover.m_pogoImpulse, m_mover.m_pogoVelocity );
		DrawScreenTextLine( "pogo: length = %.2f/%.2f", m_mover.m_pogoLength, m_mover.m_pogoRestLength );
		DrawScreenTextLine( "on ground: %d", (int)m_mover.m_onGround );

		if ( m_lockCamera )
		{
			m_camera->center.x = position.x;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new DynamicMoverSample( context );
	}

	static constexpr b2Vec2 m_elevatorBase = { 112.0f, 10.0f };
	static constexpr float m_elevatorAmplitude = 4.0f;

	DynamicMover m_mover;
	b2BodyId m_elevatorId;
	float m_time;
	bool m_jumpReleased;
	bool m_lockCamera;
};

static int sampleDynamicMover = RegisterSample( "Character", "Dynamic Mover", DynamicMoverSample::Create );
