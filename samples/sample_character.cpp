// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"

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
	DebrisBit = 0x0008,

	AllBits = ~0u,
};

enum PogoShape
{
	PogoPoint,
	PogoCircle,
	PogoSegment
};

struct CastResult
{
	b2Vec2 point;
	b2Vec2 normal;
	b2BodyId bodyId;
	float fraction;
	bool hit;
};

static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
{
	CastResult* result = (CastResult*)context;
	result->point = point;
	result->normal = normal;
	result->bodyId = b2Shape_GetBody( shapeId );
	result->fraction = fraction;
	result->hit = true;
	return fraction;
}

class Mover : public Sample
{
public:
	explicit Mover( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->m_center = { 20.0f, 9.0f };
			m_camera->m_zoom = 10.0f;
		}

		context->drawJoints = false;
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
			bodyDef.position = { 32.0f, 4.5f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.025f;
			m_friendlyShape.clipVelocity = false;

			shapeDef.filter = { MoverBit, AllBits, 0 };
			shapeDef.userData = &m_friendlyShape;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId, &shapeDef, &m_capsule );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 7.0f, 7.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DebrisBit, AllBits, 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			b2Circle circle = { b2Vec2_zero, 0.3f };
			m_ballId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { m_elevatorBase.x, m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = b2CreateBody( m_worldId, &bodyDef );

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

		m_totalIterations = 0;
		m_pogoVelocity = 0.0f;
		m_onGround = false;
		m_jumpReleased = true;
		m_lockCamera = true;
		m_planeCount = 0;
		m_time = 0.0f;
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
		b2Vec2 segmentOffset = { 0.75f * m_capsule.radius, 0.0f };
		b2Segment segment = {
			.point1 = origin - segmentOffset,
			.point2 = origin + segmentOffset,
		};

		b2ShapeProxy proxy = {};
		b2Vec2 translation;
		b2QueryFilter pogoFilter = { MoverBit, StaticBit | DynamicBit };
		CastResult castResult = {};

		if ( m_pogoShape == PogoPoint )
		{
			proxy = b2MakeProxy( &origin, 1, 0.0f );
			translation = { 0.0f, -rayLength };
		}
		else if ( m_pogoShape == PogoCircle )
		{
			proxy = b2MakeProxy( &origin, 1, circle.radius );
			translation = { 0.0f, -rayLength + circle.radius };
		}
		else
		{
			proxy = b2MakeProxy( &segment.point1, 2, 0.0f );
			translation = { 0.0f, -rayLength };
		}

		b2World_CastShape( m_worldId, &proxy, translation, pogoFilter, CastCallback, &castResult );

		// Avoid snapping to ground if still going up
		if ( m_onGround == false )
		{
			m_onGround = castResult.hit && m_velocity.y <= 0.01f;
		}
		else
		{
			m_onGround = castResult.hit;
		}

		if ( castResult.hit == false )
		{
			m_pogoVelocity = 0.0f;

			b2Vec2 delta = translation;
			m_draw->DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == PogoPoint )
			{
				m_draw->DrawPoint( origin + delta, 10.0f, b2_colorGray );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				m_draw->DrawCircle( origin + delta, circle.radius, b2_colorGray );
			}
			else
			{
				m_draw->DrawSegment( segment.point1 + delta, segment.point2 + delta, b2_colorGray );
			}
		}
		else
		{
			float pogoCurrentLength = castResult.fraction * rayLength;

			float offset = pogoCurrentLength - pogoRestLength;
			m_pogoVelocity = b2SpringDamper( m_pogoHertz, m_pogoDampingRatio, offset, m_pogoVelocity, timeStep );

			b2Vec2 delta = castResult.fraction * translation;
			m_draw->DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == PogoPoint )
			{
				m_draw->DrawPoint( origin + delta, 10.0f, b2_colorPlum );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				m_draw->DrawCircle( origin + delta, circle.radius, b2_colorPlum );
			}
			else
			{
				m_draw->DrawSegment( segment.point1 + delta, segment.point2 + delta, b2_colorPlum );
			}

			b2Body_ApplyForce( castResult.bodyId, { 0.0f, -50.0f }, castResult.point, true );
		}

		b2Vec2 target = m_transform.p + timeStep * m_velocity + timeStep * m_pogoVelocity * b2Vec2{ 0.0f, 1.0f };

		// Mover overlap filter
		b2QueryFilter collideFilter = { MoverBit, StaticBit | DynamicBit | MoverBit };

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
			b2PlaneSolverResult result = b2SolvePlanes( target - m_transform.p, m_planes, m_planeCount );

			m_totalIterations += result.iterationCount;

			float fraction = b2World_CastMover( m_worldId, &mover, result.translation, castFilter );

			b2Vec2 delta = fraction * result.translation;
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
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_camera->m_height - height - 25.0f ), ImGuiCond_Once );
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
		ImGui::RadioButton( "Segment", &m_pogoShape, PogoSegment );

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

	static bool Kick( b2ShapeId shapeId, void* context )
	{
		Mover* self = (Mover*)context;
		b2BodyId bodyId = b2Shape_GetBody( shapeId );
		b2BodyType type = b2Body_GetType( bodyId );

		if ( type != b2_dynamicBody )
		{
			return true;
		}

		b2Vec2 center = b2Body_GetWorldCenterOfMass( bodyId );
		b2Vec2 direction = b2Normalize( center - self->m_transform.p );
		b2Vec2 impulse = b2Vec2{ 2.0f * direction.x, 2.0f };
		b2Body_ApplyLinearImpulseToCenter( bodyId, impulse, true );

		return true;
	}

	void Keyboard( int key ) override
	{
		if ( key == 'K' )
		{
			b2Vec2 point = b2TransformPoint( m_transform, { 0.0f, m_capsule.center1.y - 3.0f * m_capsule.radius } );
			b2Circle circle = { point, 0.5f };
			b2ShapeProxy proxy = b2MakeProxy( &circle.center, 1, circle.radius );
			b2QueryFilter filter = { MoverBit, DebrisBit };
			b2World_OverlapShape( m_worldId, &proxy, filter, Kick, this );
			m_draw->DrawCircle( circle.center, circle.radius, b2_colorGoldenRod );
		}

		Sample::Keyboard( key );
	}

	void Step() override
	{
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
			b2Vec2 point = {
				.x = m_elevatorBase.x,
				.y = m_elevatorAmplitude * cosf( 1.0f * m_time + B2_PI ) + m_elevatorBase.y,
			};

			b2Body_SetTargetTransform( m_elevatorId, { point, b2Rot_identity }, timeStep );
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

			if ( glfwGetKey( m_context->window, GLFW_KEY_SPACE ) )
			{
				if ( m_onGround == true && m_jumpReleased )
				{
					m_velocity.y = m_jumpSpeed;
					m_onGround = false;
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			SolveMove( timeStep, throttle );
		}

		int count = m_planeCount;
		for ( int i = 0; i < count; ++i )
		{
			b2Plane plane = m_planes[i].plane;
			b2Vec2 p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
			b2Vec2 p2 = p1 + 0.1f * plane.normal;
			m_draw->DrawPoint( p1, 5.0f, b2_colorYellow );
			m_draw->DrawSegment( p1, p2, b2_colorYellow );
		}

		b2Vec2 p1 = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Vec2 p2 = b2TransformPoint( m_transform, m_capsule.center2 );

		b2HexColor color = m_onGround ? b2_colorOrange : b2_colorAquamarine;
		m_draw->DrawSolidCapsule( p1, p2, m_capsule.radius, color );
		m_draw->DrawSegment( m_transform.p, m_transform.p + m_velocity, b2_colorPurple );

		b2Vec2 p = m_transform.p;
		DrawTextLine( "position %.2f %.2f", p.x, p.y );
		DrawTextLine( "velocity %.2f %.2f", m_velocity.x, m_velocity.y );
		DrawTextLine( "iterations %d", m_totalIterations );

		if ( m_lockCamera )
		{
			m_camera->m_center.x = m_transform.p.x;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Mover( context );
	}

	static constexpr int m_planeCapacity = 8;
	static constexpr b2Vec2 m_elevatorBase = { 112.0f, 10.0f };
	static constexpr float m_elevatorAmplitude = 4.0f;

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

	int m_pogoShape = PogoSegment;
	b2Transform m_transform;
	b2Vec2 m_velocity;
	b2Capsule m_capsule;
	b2BodyId m_elevatorId;
	b2ShapeId m_ballId;
	ShapeUserData m_friendlyShape;
	ShapeUserData m_elevatorShape;
	b2CollisionPlane m_planes[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	float m_time;
	bool m_onGround;
	bool m_jumpReleased;
	bool m_lockCamera;
};

static int sampleMover = RegisterSample( "Character", "Mover", Mover::Create );

// todo mover experiment wip
#if 0
enum MoverMode
{
	e_walkingMode,
	e_fallingMode,
	e_modeCount,
};

class Mover2 : public Sample
{
public:
	explicit Mover( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 20.0f, 9.0f };
			m_context->camera.m_zoom = 10.0f;
		}

		settings.drawJoints = false;
		m_transform = { { 2.0f, 8.0f }, b2Rot_identity };
		m_velocity = { 0.0f, 0.0f };
		m_capsule = { { 0.0f, -0.5f * m_capsuleInteriorLength }, { 0.0f, 0.5f * m_capsuleInteriorLength }, m_capsuleRadius };

		b2BodyId groundId1;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			groundId1 = b2CreateBody( m_worldId, &bodyDef );

			const char* path =
				"M -34.395834,201.08333 H 293.68751 v -47.625 h -2.64584 l -10.58333,7.9375 -13.22916,7.9375 -13.24648,5.29167 "
				"-31.73269,7.9375 -21.16667,2.64583 -23.8125,10.58333 H 142.875 v -5.29167 h -5.29166 v 5.29167 H 119.0625 v "
				"-2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 H 84.666668 v -2.64583 h -5.291666 v "
				"-2.64584 h -5.291667 v -2.64583 H 68.791668 V 174.625 h -5.291666 v -2.64584 H 52.916669 L 39.6875,177.27083 h "
				"-5.291667 l -7.937499,5.29167 H 15.875001 l -47.625002,-50.27083 v -26.45834 h -2.645834 l 10e-7,95.25";

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
			bodyDef.position = { 32.0f, 4.5f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_friendlyShape.maxPush = 0.025f;
			m_friendlyShape.clipVelocity = false;

			shapeDef.filter = { MoverBit, AllBits, 0 };
			shapeDef.userData = &m_friendlyShape;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId, &shapeDef, &m_capsule );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 7.0f, 7.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter = { DebrisBit, AllBits, 0 };
			shapeDef.material.restitution = 0.7f;
			shapeDef.material.rollingResistance = 0.2f;

			b2Circle circle = { b2Vec2_zero, 0.3f };
			m_ballId = b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { m_elevatorBase.x, m_elevatorBase.y - m_elevatorAmplitude };
			m_elevatorId = b2CreateBody( m_worldId, &bodyDef );

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

		m_floorResult = {};
		m_floorSeparation = 0.0f;
		m_mode = MoverMode::e_fallingMode;
		m_nonWalkablePosition = m_transform.p;
		m_noWalkTime = 0.0f;
		m_groundNormal = b2Vec2_zero;
		m_totalIterations = 0;
		m_pogoVelocity = 0.0f;
		m_canWalk = false;
		m_jumpReleased = true;
		m_lockCamera = true;
		m_planeCount = 0;
		m_time = 0.0f;
	}

	void UpdateFloorData(float timeStep)
	{
		// This cast extension keeps the character glued to the ground when walking down slopes and steps.
		float castExtension = m_stepDownHeight;

		// The extension increases with velocity while on ground.
		if (m_mode == MoverMode::e_walkingMode)
		{
			castExtension += b2Length( m_velocity ) * timeStep;
		}

		float castLength = m_pogoRestLength + m_capsule.radius + castExtension;

		DrawTextLine( "extension = %.3f", castExtension );

		// Start cast from the center of the bottom capsule sphere.
		b2Vec2 origin = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Vec2 translation = { 0.0, -castLength };

		b2QueryFilter pogoFilter = { MoverBit, StaticBit | DynamicBit };
		m_floorResult = {};

		b2World_CastRay( m_worldId, origin, translation, pogoFilter, CastCallback, &m_floorResult );

		if ( m_floorResult.hit == false || m_floorResult.normal.y < m_walkableGroundY )
		{
			m_floorResult = {};

			// Try again with a circle cast
			b2ShapeProxy proxy = b2MakeProxy( &origin, 1, 0.9f * m_capsuleRadius );
			b2World_CastShape( m_worldId, &proxy, translation, pogoFilter, CastCallback, &m_floorResult );
		}

		if ( m_floorResult.hit == true)
		{
			float contactHeight = m_floorResult.point.y;
			float distanceToGround = m_transform.p.y - contactHeight;

			if (m_velocity.y > 0.0f)
			{
				// todo why bother trying to find the surface if we just reset it here?
				m_floorResult = {};
				m_floorSeparation = 1.0f;
				m_canWalk = false;
			}
			else
			{
				m_floorSeparation = distanceToGround;
				m_canWalk = false;

				if (m_floorResult.hit && m_floorResult.normal.y >= m_walkableGroundY)
				{
					if (distanceToGround <= castExtension)
					{
						if (m_mode == MoverMode::e_walkingMode)
						{
							float currentPogoLength = m_pogoRestLength + m_pogoOffset;
							float bottomOfCapsule = m_transform.p.y + currentPogoLength;
							float bottomOfCapsuleAtRest = contactHeight + m_pogoRestLength;
							float prevOffset = m_pogoOffset;
							m_pogoOffset = bottomOfCapsule - bottomOfCapsuleAtRest;
							m_visualOffset += m_pogoOffset - prevOffset;
						}

						m_canWalk = true;
					}
					else
					{
						m_floorResult = {};
					}
				}
			}
		}
	}

	void SetMode(MoverMode mode)
	{
		if (mode == m_mode)
		{
			return;
		}

		m_canJump = false;

		if (m_modePicked[mode] == true)
		{
			return;
		}

		if (mode == e_fallingMode)
		{
			m_timeInAir = 0.0f;
		}
		else if (mode == e_walkingMode)
		{
			if (m_mode == e_fallingMode)
			{
				m_landed = true;
				m_landingSpeed = -m_velocity.y;
			}

			m_velocity.y = 0.0f;
		}

		m_mode = mode;
	}

	void ComputeVelocity( float timeStep, bool keepNormalVelocity )
	{
#if 0
		bool walkable = m_groundNormal.y > 0.71f;

		// Friction
		float speed = b2Length( m_velocity );

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

		float noWalkSteer = 0.0f;
		if ( m_onGround )
		{
			if ( walkable )
			{
				m_velocity.y = 0.0f;
				noWalkSteer = m_airSteer;
				m_noWalkTime = 0.0f;
			}
			else
			{
				if ( m_noWalkTime == 0.0f )
				{
					m_nonWalkablePosition = m_transform.p;
				}

				m_velocity = m_velocity - b2Dot( m_velocity, m_groundNormal ) * m_groundNormal;
				// m_noWalkSpeed = m_airSteer * b2MaxFloat( 0.0f, 1.0f - 0.25f * speed / b2MaxFloat(m_stopSpeed, 1.0f) );

				float noWalkDistance = b2Distance( m_transform.p, m_nonWalkablePosition );
				noWalkSteer = m_airSteer;
				m_noWalkTime += timeStep;
				// if ( noWalkDistance / m_noWalkTime < 0.5f * m_airSteer * m_maxSpeed )
				//{
				//	noWalkSteer = m_airSteer + 0.5f * m_noWalkTime * ( 1.0f - m_airSteer );
				//	noWalkSteer = b2ClampFloat( noWalkSteer, m_airSteer, 1.0f );
				// }
			}
		}
		else
		{
			m_noWalkTime = 0.0f;
		}

		// Accelerate
		float currentSpeed = b2Dot( m_velocity, desiredDirection );
		float addSpeed = desiredSpeed - currentSpeed;
		if ( addSpeed > 0.0f )
		{
			float steer;
			if ( m_onGround )
			{
				steer = walkable ? 1.0f : noWalkSteer;
			}
			else
			{
				steer = m_airSteer;
			}

			DrawTextLine( "steer = %.2f", steer );

			float accelSpeed = steer * m_accelerate * m_maxSpeed * timeStep;
			if ( accelSpeed > addSpeed )
			{
				accelSpeed = addSpeed;
			}

			m_velocity += accelSpeed * desiredDirection;
		}
#else
		// todo keepNormalVelocity always true
		b2Vec2 v = m_velocity;
		float vy = v.y;
		v.y = 0.0f;

		// ConfigAccel = m_accelerate
		// ConfigDecelBase =
		// ConfigDecelPerSpeed
		// ConfigDecelStartAtSpeed

		bool inAir = m_mode == e_fallingMode;
		float acceleration = inAir ? m_airSteer * m_accelerate : m_accelerate;

		// m/s^2
		float decelBase = inAir ? 0.0f : 2.0f; // todo

		// m/s^2 / m/s = 1/s
		// same as friction in quake?
		float decelPerSpeed = inAir ? 1.0f : 5.0f; // todo

		// m/s
		float decelStartAtSpeed = inAir ? 4.25f : 0.0f; // todo

		float speed = b2Length( v );

		// m/s^2
		float perSpeedDecel = decelPerSpeed * b2MaxFloat( speed - decelStartAtSpeed, 0.0f );

		// m/s^2
		float decelTotal = decelBase + perSpeedDecel;

		// m/s
		float decel = decelTotal * timeStep;

		// m/s
		float decelCapped = b2MinFloat( decel, speed );

		float accel = acceleration * m_maxSpeed * timeStep;

		v -= decelCapped * b2Normalize( v );

		b2Vec2 throttle = { m_throttle, 0.0f };
		b2Vec2 control = m_maxSpeed * throttle;
		b2Vec2 toTarget = control - v;
		float toTargetDist = b2Length( toTarget );

		if ( b2Length( control ) > b2Length(v))
		{
			float accelFraction = b2ClampFloat( accel / toTargetDist, 0.0f, 1.0f );
			v += accelFraction * toTarget;
		}
#endif
	}

	void UpdateFalling(float timeStep)
	{
		float vy = m_velocity.y;
		ComputeVelocity(timeStep, true );
		m_velocity.y = vy;

		m_velocity.y -= m_gravity * timeStep;

		m_timeInAir += timeStep;

		m_floorSeparation = 0.0f;

		if ( b2Dot(m_velocity, m_floorResult.normal) < 0.0f && m_floorResult.normal.y >= m_walkableGroundY)
		{
			float currentPogoLength = m_pogoRestLength + m_pogoOffset;
			float bottomOfCapsule = m_transform.p.y + currentPogoLength;
			float bottomOfCapsuleAtRest = m_floorResult.point.y + m_pogoRestLength;
			float prevOffset = m_pogoOffset;
			m_pogoOffset = bottomOfCapsule - bottomOfCapsuleAtRest;
			m_visualOffset += m_pogoOffset - prevOffset;

			SetMode( e_walkingMode );
		}
	}

	void UpdateWalking( float timeStep )
	{
		if (m_floorResult.hit == false || m_floorResult.normal.y < m_walkableGroundY)
		{
			SetMode( e_fallingMode );
			return;
		}

		if (m_floorResult.normal.y >= m_walkableGroundY)
		{
			// todo unsafe teleport
			m_transform.p.y = m_floorResult.point.y;
		}

		ComputeVelocity(timeStep, true);
		m_velocity.y = 0.0f;

		if ( m_canJump && glfwGetKey( m_context->window, GLFW_KEY_SPACE ) )
		{
			m_velocity.y = m_jumpSpeed;
			SetMode( e_fallingMode );
		}
	}

	// https://github.com/id-Software/Quake/blob/master/QW/client/pmove.c#L390
	void SolveMove( float timeStep, float throttle )
	{
		bool walkable = m_groundNormal.y > 0.71f;

		// Friction
		float speed = b2Length( m_velocity );
		if ( speed < m_minSpeed )
		{
			m_velocity.x = 0.0f;
			m_velocity.y = 0.0f;
		}
		else if ( m_onGround && walkable )
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

		float noWalkSteer = 0.0f;
		if ( m_onGround )
		{
			if ( walkable )
			{
				m_velocity.y = 0.0f;
				noWalkSteer = m_airSteer;
				m_noWalkTime = 0.0f;
			}
			else
			{
				if (m_noWalkTime == 0.0f)
				{
					m_nonWalkablePosition = m_transform.p;
				}

				m_velocity = m_velocity - b2Dot( m_velocity, m_groundNormal ) * m_groundNormal;
				//m_noWalkSpeed = m_airSteer * b2MaxFloat( 0.0f, 1.0f - 0.25f * speed / b2MaxFloat(m_stopSpeed, 1.0f) );

				float noWalkDistance = b2Distance( m_transform.p, m_nonWalkablePosition );
				noWalkSteer = m_airSteer;
				m_noWalkTime += timeStep;
				//if ( noWalkDistance / m_noWalkTime < 0.5f * m_airSteer * m_maxSpeed )
				//{
				//	noWalkSteer = m_airSteer + 0.5f * m_noWalkTime * ( 1.0f - m_airSteer );
				//	noWalkSteer = b2ClampFloat( noWalkSteer, m_airSteer, 1.0f );
				//}
			}
		}
		else
		{
			m_noWalkTime = 0.0f;
		}

		// Accelerate
		float currentSpeed = b2Dot( m_velocity, desiredDirection );
		float addSpeed = desiredSpeed - currentSpeed;
		if ( addSpeed > 0.0f )
		{
			float steer;
			if ( m_onGround )
			{
				steer = walkable ? 1.0f : noWalkSteer;
			}
			else
			{
				steer = m_airSteer;
			}

			DrawTextLine( "steer = %.2f", steer );

			float accelSpeed = steer * m_accelerate * m_maxSpeed * timeStep;
			if ( accelSpeed > addSpeed )
			{
				accelSpeed = addSpeed;
			}

			m_velocity += accelSpeed * desiredDirection;
		}

		m_velocity.y -= m_gravity * timeStep;

		// This ray extension keeps you glued to the ground when walking down slopes.
		// The extension increases with velocity.
		float rayExtension = m_stepDownHeight;
		rayExtension += m_onGround ? b2Length( m_velocity ) * timeStep : 0.0f;

		float rayLength = m_pogoRestLength + m_capsule.radius + rayExtension;

		DrawTextLine( "extension = %.3f", rayExtension );

		b2Vec2 origin = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Circle circle = { origin, 0.9f * m_capsule.radius };
		b2Vec2 segmentOffset = { 0.9f * m_capsule.radius, 0.0f };
		b2Segment segment = {
			.point1 = origin - segmentOffset,
			.point2 = origin + segmentOffset,
		};

		b2ShapeProxy proxy = {};
		b2Vec2 translation;
		b2QueryFilter pogoFilter = { MoverBit, StaticBit | DynamicBit };
		CastResult castResult = {};

		if ( m_pogoShape == e_pogoPoint )
		{
			proxy = b2MakeProxy( &origin, 1, 0.0f );
			translation = { 0.0f, -rayLength };
		}
		else if ( m_pogoShape == PogoCircle )
		{
			proxy = b2MakeProxy( &origin, 1, circle.radius );
			translation = { 0.0f, -rayLength + circle.radius };
		}
		else
		{
			proxy = b2MakeProxy( &segment.point1, 2, 0.0f );
			translation = { 0.0f, -rayLength };
		}

		b2World_CastShape( m_worldId, &proxy, translation, pogoFilter, CastCallback, &castResult );

		// Avoid snapping to ground if still going up
		if ( m_onGround == false )
		{
			m_onGround = castResult.hit && m_velocity.y <= 0.01f;
		}
		else
		{
			m_onGround = castResult.hit;
		}

		if ( castResult.hit == false )
		{
			m_pogoVelocity = 0.0f;
			m_groundNormal = b2Vec2_zero;

			b2Vec2 delta = translation;
			m_context->draw.DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == e_pogoPoint )
			{
				m_context->draw.DrawPoint( origin + delta, 10.0f, b2_colorGray );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				m_context->draw.DrawCircle( origin + delta, circle.radius, b2_colorGray );
			}
			else
			{
				m_context->draw.DrawSegment( segment.point1 + delta, segment.point2 + delta, b2_colorGray );
			}
		}
		else
		{
			m_groundNormal = castResult.normal;

			float pogoCurrentLength = castResult.fraction * rayLength - m_capsuleRadius;

			float zeta = m_pogoDampingRatio;
			float hertz = m_pogoHertz;
			float omega = 2.0f * B2_PI * hertz;
			float omegaH = omega * timeStep;

			m_pogoVelocity = ( m_pogoVelocity - omega * omegaH * ( pogoCurrentLength - m_pogoRestLength ) ) /
							 ( 1.0f + 2.0f * zeta * omegaH + omegaH * omegaH );

			b2Vec2 delta = castResult.fraction * translation;
			m_context->draw.DrawSegment( origin, origin + delta, b2_colorGray );

			if ( m_pogoShape == e_pogoPoint )
			{
				m_context->draw.DrawPoint( origin + delta, 10.0f, b2_colorPlum );
			}
			else if ( m_pogoShape == PogoCircle )
			{
				m_context->draw.DrawCircle( origin + delta, circle.radius, b2_colorPlum );
			}
			else
			{
				m_context->draw.DrawSegment( segment.point1 + delta, segment.point2 + delta, b2_colorPlum );
			}

			b2Body_ApplyForce( castResult.bodyId, { 0.0f, -50.0f }, castResult.point, true );
		}

		b2Vec2 target = m_transform.p + timeStep * m_velocity + timeStep * m_pogoVelocity * b2Vec2{ 0.0f, 1.0f };

		// Mover overlap filter
		b2QueryFilter collideFilter = { MoverBit, StaticBit | DynamicBit | MoverBit };

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
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 25.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 340.0f, height ) );

		ImGui::Begin( "Mover", nullptr, 0 );

		ImGui::PushItemWidth( 240.0f );

		ImGui::SliderFloat( "Jump Speed", &m_jumpSpeed, 0.0f, 20.0f, "%.0f" );
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

	static bool Kick( b2ShapeId shapeId, void* context )
	{
		Mover* self = (Mover*)context;
		b2BodyId bodyId = b2Shape_GetBody( shapeId );
		b2BodyType type = b2Body_GetType( bodyId );

		if ( type != b2_dynamicBody )
		{
			return true;
		}

		b2Vec2 center = b2Body_GetWorldCenterOfMass( bodyId );
		b2Vec2 direction = b2Normalize( center - self->m_transform.p );
		b2Vec2 impulse = b2Vec2{ 2.0f * direction.x, 2.0f };
		b2Body_ApplyLinearImpulseToCenter( bodyId, impulse, true );

		return true;
	}

	void Keyboard( int key ) override
	{
		if ( key == 'K' )
		{
			b2Vec2 point = b2TransformPoint( m_transform, { 0.0f, m_capsule.center1.y - m_capsuleRadius } );
			b2Circle circle = { point, 0.5f };
			b2ShapeProxy proxy = b2MakeProxy( &circle.center, 1, circle.radius );
			b2QueryFilter filter = { MoverBit, DebrisBit };
			b2World_OverlapShape( m_worldId, &proxy, filter, Kick, this );
			m_context->draw.DrawCircle( circle.center, circle.radius, b2_colorGoldenRod );
		}

		Sample::Keyboard( key );
	}

	void Step() override
	{
		bool pause = false;
		if ( settings.pause )
		{
			pause = settings.singleStep != true;
		}

		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;
		if ( pause )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f )
		{
			b2Vec2 point = {
				.x = m_elevatorBase.x,
				.y = m_elevatorAmplitude * cosf( 1.0f * m_time + B2_PI ) + m_elevatorBase.y,
			};

			b2Body_SetTargetTransform( m_elevatorId, { point, b2Rot_identity }, timeStep );
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

			if ( glfwGetKey( m_context->window, GLFW_KEY_SPACE ) )
			{
				bool walkable = m_groundNormal.y > 0.71f;
				if ( m_onGround == true && walkable && m_jumpReleased )
				{
					m_velocity.y = m_jumpSpeed;
					m_onGround = false;
					m_jumpReleased = false;
				}
			}
			else
			{
				m_jumpReleased = true;
			}

			SolveMove( timeStep, throttle );
		}

		int count = m_planeCount;
		for ( int i = 0; i < count; ++i )
		{
			b2Plane plane = m_planes[i].plane;
			b2Vec2 p1 = m_transform.p + ( plane.offset - m_capsule.radius ) * plane.normal;
			b2Vec2 p2 = p1 + 0.1f * plane.normal;
			m_context->draw.DrawPoint( p1, 5.0f, b2_colorYellow );
			m_context->draw.DrawSegment( p1, p2, b2_colorYellow );
		}

		b2Vec2 p1 = b2TransformPoint( m_transform, m_capsule.center1 );
		b2Vec2 p2 = b2TransformPoint( m_transform, m_capsule.center2 );

		b2HexColor color = m_onGround ? b2_colorOrange : b2_colorAquamarine;
		m_context->draw.DrawSolidCapsule( p1, p2, m_capsule.radius, color );
		m_context->draw.DrawSegment( m_transform.p, m_transform.p + m_velocity, b2_colorPurple );

		b2Vec2 p = m_transform.p;
		DrawTextLine( "position %.2f %.2f", p.x, p.y );
		DrawTextLine( "velocity %.2f %.2f", m_velocity.x, m_velocity.y );
		DrawTextLine( "iterations %d", m_totalIterations );

		if ( m_lockCamera )
		{
			m_context->camera.m_center.x = m_transform.p.x;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Mover( context );
	}

	static constexpr int m_planeCapacity = 8;
	static constexpr b2Vec2 m_elevatorBase = { 112.0f, 10.0f };
	static constexpr float m_elevatorAmplitude = 4.0f;
	static constexpr float m_walkableGroundY = 0.706f;

	float m_pogoRestLength = 0.5f;
	float m_capsuleRadius = 0.4f;
	float m_capsuleInteriorLength = 0.5f;
	float m_jumpSpeed = 5.0f;
	float m_maxSpeed = 6.0f;
	float m_stopSpeed = 2.0f;
	float m_accelerate = 11.0f;
	float m_airSteer = 0.2f;
	float m_friction = 8.0f;
	float m_gravity = 15.0f;
	float m_pogoHertz = 5.0f;
	float m_pogoDampingRatio = 0.8f;
	float m_stepDownHeight = 0.5f;

	CastResult m_floorResult;
	float m_floorSeparation;
	float m_pogoSpeed = 0.0f;
	float m_pogoOffset = 0.0f;
	float m_visualOffset = 0.0f;

	// The mover transform origin is at the bottom of the pogo stick. On the floor if walking.
	b2Transform m_transform;

	MoverMode m_mode;
	b2Vec2 m_velocity;
	b2Vec2 m_groundNormal;
	b2Vec2 m_nonWalkablePosition;
	b2Capsule m_capsule;
	b2BodyId m_elevatorId;
	b2ShapeId m_ballId;
	ShapeUserData m_friendlyShape;
	ShapeUserData m_elevatorShape;
	b2CollisionPlane m_planes[m_planeCapacity] = {};
	bool m_modePicked[MoverMode::e_modeCount];
	int m_planeCount;
	int m_totalIterations;
	float m_timeInAir;
	float m_throttle;
	bool m_canJump;
	bool m_landed;
	float m_landingSpeed;
	float m_pogoVelocity;
	float m_time;
	float m_noWalkTime;
	bool m_canWalk;
	bool m_jumpReleased;
	bool m_lockCamera;
};

static int sampleMover = RegisterSample( "Character", "Mover", Mover2::Create );
#endif
