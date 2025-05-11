// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"

#include <imgui.h>

class BodyType : public Sample
{
public:
	explicit BodyType( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.8f, 6.4f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
		}

		m_type = b2_dynamicBody;
		m_isEnabled = true;

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Define attachment
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -2.0f, 3.0f };
			m_attachmentId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.5f, 2.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( m_attachmentId, &shapeDef, &box );
		}

		// Define second attachment
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 3.0f, 3.0f };
			m_secondAttachmentId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.5f, 2.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( m_secondAttachmentId, &shapeDef, &box );
		}

		// Define platform
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { -4.0f, 5.0f };
			m_platformId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 0.5f, 4.0f, { 4.0f, 0.0f }, b2MakeRot( 0.5f * B2_PI ) );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;
			b2CreatePolygonShape( m_platformId, &shapeDef, &box );

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
			b2Vec2 pivot = { -2.0f, 5.0f };
			revoluteDef.bodyIdA = m_attachmentId;
			revoluteDef.bodyIdB = m_platformId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.localAnchorB = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			pivot = { 3.0f, 5.0f };
			revoluteDef.bodyIdA = m_secondAttachmentId;
			revoluteDef.bodyIdB = m_platformId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint( m_secondAttachmentId, pivot );
			revoluteDef.localAnchorB = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
			b2Vec2 anchor = { 0.0f, 5.0f };
			prismaticDef.bodyIdA = groundId;
			prismaticDef.bodyIdB = m_platformId;
			prismaticDef.localAnchorA = b2Body_GetLocalPoint( groundId, anchor );
			prismaticDef.localAnchorB = b2Body_GetLocalPoint( m_platformId, anchor );
			prismaticDef.localAxisA = { 1.0f, 0.0f };
			prismaticDef.maxMotorForce = 1000.0f;
			prismaticDef.motorSpeed = 0.0f;
			prismaticDef.enableMotor = true;
			prismaticDef.lowerTranslation = -10.0f;
			prismaticDef.upperTranslation = 10.0f;
			prismaticDef.enableLimit = true;

			b2CreatePrismaticJoint( m_worldId, &prismaticDef );

			m_speed = 3.0f;
		}

		// Create a payload
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -3.0f, 8.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.75f, 0.75f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;

			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// Create a second payload
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 2.0f, 8.0f };
			m_secondPayloadId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.75f, 0.75f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;

			b2CreatePolygonShape( m_secondPayloadId, &shapeDef, &box );
		}

		// Create a separate body on the ground
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { 8.0f, 0.2f };
			m_touchingBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, 0.25f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;

			b2CreateCapsuleShape( m_touchingBodyId, &shapeDef, &capsule );
		}

		// Create a separate floating body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { -8.0f, 12.0f };
			bodyDef.gravityScale = 0.0f;
			m_floatingBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle circle = { { 0.0f, 0.5f }, 0.25f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;

			b2CreateCircleShape( m_floatingBodyId, &shapeDef, &circle );
		}
	}

	void UpdateGui() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );
		ImGui::Begin( "Body Type", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::RadioButton( "Static", m_type == b2_staticBody ) )
		{
			m_type = b2_staticBody;
			b2Body_SetType( m_platformId, b2_staticBody );
			b2Body_SetType( m_secondAttachmentId, b2_staticBody );
			b2Body_SetType( m_secondPayloadId, b2_staticBody );
			b2Body_SetType( m_touchingBodyId, b2_staticBody );
			b2Body_SetType( m_floatingBodyId, b2_staticBody );
		}

		if ( ImGui::RadioButton( "Kinematic", m_type == b2_kinematicBody ) )
		{
			m_type = b2_kinematicBody;
			b2Body_SetType( m_platformId, b2_kinematicBody );
			b2Body_SetLinearVelocity( m_platformId, { -m_speed, 0.0f } );
			b2Body_SetAngularVelocity( m_platformId, 0.0f );
			b2Body_SetType( m_secondAttachmentId, b2_kinematicBody );
			b2Body_SetType( m_secondPayloadId, b2_kinematicBody );
			b2Body_SetType( m_touchingBodyId, b2_kinematicBody );
			b2Body_SetType( m_floatingBodyId, b2_kinematicBody );
		}

		if ( ImGui::RadioButton( "Dynamic", m_type == b2_dynamicBody ) )
		{
			m_type = b2_dynamicBody;
			b2Body_SetType( m_platformId, b2_dynamicBody );
			b2Body_SetType( m_secondAttachmentId, b2_dynamicBody );
			b2Body_SetType( m_secondPayloadId, b2_dynamicBody );
			b2Body_SetType( m_touchingBodyId, b2_dynamicBody );
			b2Body_SetType( m_floatingBodyId, b2_dynamicBody );
		}

		if ( ImGui::Checkbox( "Enable", &m_isEnabled ) )
		{
			if ( m_isEnabled )
			{
				b2Body_Enable( m_platformId );
				b2Body_Enable( m_secondAttachmentId );
				b2Body_Enable( m_secondPayloadId );
				b2Body_Enable( m_touchingBodyId );
				b2Body_Enable( m_floatingBodyId );

				if ( m_type == b2_kinematicBody )
				{
					b2Body_SetLinearVelocity( m_platformId, { -m_speed, 0.0f } );
					b2Body_SetAngularVelocity( m_platformId, 0.0f );
				}
			}
			else
			{
				b2Body_Disable( m_platformId );
				b2Body_Disable( m_secondAttachmentId );
				b2Body_Disable( m_secondPayloadId );
				b2Body_Disable( m_touchingBodyId );
				b2Body_Disable( m_floatingBodyId );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		// Drive the kinematic body.
		if ( m_type == b2_kinematicBody )
		{
			b2Vec2 p = b2Body_GetPosition( m_platformId );
			b2Vec2 v = b2Body_GetLinearVelocity( m_platformId );

			if ( ( p.x < -14.0f && v.x < 0.0f ) || ( p.x > 6.0f && v.x > 0.0f ) )
			{
				v.x = -v.x;
				b2Body_SetLinearVelocity( m_platformId, v );
			}
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BodyType( context );
	}

	b2BodyId m_attachmentId;
	b2BodyId m_secondAttachmentId;
	b2BodyId m_platformId;
	b2BodyId m_secondPayloadId;
	b2BodyId m_touchingBodyId;
	b2BodyId m_floatingBodyId;
	b2BodyType m_type;
	float m_speed;
	bool m_isEnabled;
};

static int sampleBodyType = RegisterSample( "Bodies", "Body Type", BodyType::Create );

float FrictionCallback( float, int, float, int )
{
	return 0.1f;
}

float RestitutionCallback( float, int, float, int )
{
	return 1.0f;
}

class Weeble : public Sample
{
public:
	explicit Weeble( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 2.3f, 10.0f };
			m_context->camera.m_zoom = 25.0f * 0.5f;
		}

		// Test friction and restitution callbacks
		b2World_SetFrictionCallback( m_worldId, FrictionCallback );
		b2World_SetRestitutionCallback( m_worldId, RestitutionCallback );

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Build weeble
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 3.0f };
			bodyDef.rotation = b2MakeRot( 0.25f * B2_PI );
			m_weebleId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 1.0f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateCapsuleShape( m_weebleId, &shapeDef, &capsule );

			float mass = b2Body_GetMass( m_weebleId );
			float inertiaTensor = b2Body_GetRotationalInertia( m_weebleId );

			float offset = 1.5f;

			// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
			inertiaTensor += mass * offset * offset;

			b2MassData massData = { mass, { 0.0f, -offset }, inertiaTensor };
			b2Body_SetMassData( m_weebleId, massData );
		}

		m_explosionPosition = { 0.0f, 0.0f };
		m_explosionRadius = 2.0f;
		m_explosionMagnitude = 8.0f;
	}

	void UpdateGui() override
	{
		float height = 120.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );
		ImGui::Begin( "Weeble", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );
		if ( ImGui::Button( "Teleport" ) )
		{
			b2Body_SetTransform( m_weebleId, { 0.0f, 5.0f }, b2MakeRot( 0.95 * B2_PI ) );
		}

		if ( ImGui::Button( "Explode" ) )
		{
			b2ExplosionDef def = b2DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = m_explosionRadius;
			def.falloff = 0.1f;
			def.impulsePerLength = m_explosionMagnitude;
			b2World_Explode( m_worldId, &def );
		}
		ImGui::PushItemWidth( 100.0f );

		ImGui::SliderFloat( "Magnitude", &m_explosionMagnitude, -100.0f, 100.0f, "%.1f" );

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		m_context->draw.DrawCircle( m_explosionPosition, m_explosionRadius, b2_colorAzure );

		// This shows how to get the velocity of a point on a body
		b2Vec2 localPoint = { 0.0f, 2.0f };
		b2Vec2 worldPoint = b2Body_GetWorldPoint( m_weebleId, localPoint );

		b2Vec2 v1 = b2Body_GetLocalPointVelocity( m_weebleId, localPoint );
		b2Vec2 v2 = b2Body_GetWorldPointVelocity( m_weebleId, worldPoint );

		b2Vec2 offset = { 0.05f, 0.0f };
		m_context->draw.DrawSegment( worldPoint, worldPoint + v1, b2_colorRed );
		m_context->draw.DrawSegment( worldPoint + offset, worldPoint + v2 + offset, b2_colorGreen );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Weeble( context );
	}

	b2BodyId m_weebleId;
	b2Vec2 m_explosionPosition;
	float m_explosionRadius;
	float m_explosionMagnitude;
};

static int sampleWeeble = RegisterSample( "Bodies", "Weeble", Weeble::Create );

class Sleep : public Sample
{
public:
	explicit Sleep( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 3.0f, 50.0f };
			m_context->camera.m_zoom = 25.0f * 2.2f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.enableSensorEvents = true;
			m_groundShapeId = b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Sleeping body with sensors
		for ( int i = 0; i < 2; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -4.0f, 3.0f + 2.0f * i };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, 1.0f }, { 1.0f, 1.0f }, 0.75f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			capsule.radius = 1.0f;
			m_sensorIds[i] = b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
			m_sensorTouching[i] = false;
		}

		// Sleeping body but sleep is disabled
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 3.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = false;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle circle = { { 1.0f, 1.0f }, 1.0f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		// Awake body and sleep is disabled
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 5.0f, 3.0f };
			bodyDef.isAwake = true;
			bodyDef.enableSleep = false;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 1.0f, 1.0f, { 0.0f, 1.0f }, b2MakeRot( 0.25f * B2_PI ) );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// A sleeping body to test waking on collision
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 5.0f, 1.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeSquare( 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// A long pendulum
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 100.0f };
			bodyDef.angularDamping = 0.5f;
			bodyDef.sleepThreshold = 0.05f;
			m_pendulumId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, 0.0f }, { 90.0f, 0.0f }, 0.25f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateCapsuleShape( m_pendulumId, &shapeDef, &capsule );

			b2Vec2 pivot = bodyDef.position;
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_pendulumId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// A sleeping body to test waking on contact destroyed
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -10.0f, 1.0f };
			bodyDef.isAwake = false;
			bodyDef.enableSleep = true;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeSquare( 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		m_staticBodyId = b2_nullBodyId;
	}

	void ToggleInvoker()
	{
		if ( B2_IS_NULL( m_staticBodyId ) )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -10.5f, 3.0f };
			m_staticBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 2.0f, 0.1f, { 0.0f, 0.0f }, b2MakeRot( 0.25f * B2_PI ) );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.invokeContactCreation = true;
			b2CreatePolygonShape( m_staticBodyId, &shapeDef, &box );
		}
		else
		{
			b2DestroyBody( m_staticBodyId );
			m_staticBodyId = b2_nullBodyId;
		}
	}

	void UpdateGui() override
	{
		float height = 160.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );
		ImGui::Begin( "Sleep", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 120.0f );

		ImGui::Text( "Pendulum Tuning" );

		float sleepVelocity = b2Body_GetSleepThreshold( m_pendulumId );
		if ( ImGui::SliderFloat( "sleep velocity", &sleepVelocity, 0.0f, 1.0f, "%.2f" ) )
		{
			b2Body_SetSleepThreshold( m_pendulumId, sleepVelocity );
			b2Body_SetAwake( m_pendulumId, true );
		}

		float angularDamping = b2Body_GetAngularDamping( m_pendulumId );
		if ( ImGui::SliderFloat( "angular damping", &angularDamping, 0.0f, 2.0f, "%.2f" ) )
		{
			b2Body_SetAngularDamping( m_pendulumId, angularDamping );
		}

		ImGui::PopItemWidth();

		ImGui::Separator();

		if ( B2_IS_NULL( m_staticBodyId ) )
		{
			if ( ImGui::Button( "Create" ) )
			{
				ToggleInvoker();
			}
		}
		else
		{
			if ( ImGui::Button( "Destroy" ) )
			{
				ToggleInvoker();
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		// Detect sensors touching the ground
		b2SensorEvents sensorEvents = b2World_GetSensorEvents( m_worldId );

		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = sensorEvents.beginEvents + i;
			if ( B2_ID_EQUALS( event->visitorShapeId, m_groundShapeId ) )
			{
				if ( B2_ID_EQUALS( event->sensorShapeId, m_sensorIds[0] ) )
				{
					m_sensorTouching[0] = true;
				}
				else if ( B2_ID_EQUALS( event->sensorShapeId, m_sensorIds[1] ) )
				{
					m_sensorTouching[1] = true;
				}
			}
		}

		for ( int i = 0; i < sensorEvents.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = sensorEvents.endEvents + i;
			if ( B2_ID_EQUALS( event->visitorShapeId, m_groundShapeId ) )
			{
				if ( B2_ID_EQUALS( event->sensorShapeId, m_sensorIds[0] ) )
				{
					m_sensorTouching[0] = false;
				}
				else if ( B2_ID_EQUALS( event->sensorShapeId, m_sensorIds[1] ) )
				{
					m_sensorTouching[1] = false;
				}
			}
		}

		for ( int i = 0; i < 2; ++i )
		{
			DrawTextLine( "sensor touch %d = %s", i, m_sensorTouching[i] ? "true" : "false" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Sleep( context );
	}

	b2BodyId m_pendulumId;
	b2BodyId m_staticBodyId;
	b2ShapeId m_groundShapeId;
	b2ShapeId m_sensorIds[2];
	bool m_sensorTouching[2];
};

static int sampleSleep = RegisterSample( "Bodies", "Sleep", Sleep::Create );

class BadBody : public Sample
{
public:
	explicit BadBody( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 2.3f, 10.0f };
			m_context->camera.m_zoom = 25.0f * 0.5f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Build a bad body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 3.0f };
			bodyDef.angularVelocity = 0.5f;
			bodyDef.rotation = b2MakeRot( 0.25f * B2_PI );

			m_badBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 1.0f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// density set to zero intentionally to create a bad body
			shapeDef.density = 0.0f;
			b2CreateCapsuleShape( m_badBodyId, &shapeDef, &capsule );
		}

		// Build a normal body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 2.0f, 3.0f };
			bodyDef.rotation = b2MakeRot( 0.25f * B2_PI );

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 1.0f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}
	}

	void Step() override
	{
		Sample::Step();

		DrawTextLine("A bad body is a dynamic body with no mass and behaves like a kinematic body." );
		DrawTextLine( "Bad bodies are considered invalid and a user bug. Behavior is not guaranteed." );

		// For science
		b2Body_ApplyForceToCenter( m_badBodyId, { 0.0f, 10.0f }, true );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BadBody( context );
	}

	b2BodyId m_badBodyId;
};

static int sampleBadBody = RegisterSample( "Bodies", "Bad", BadBody::Create );

// This shows how to set the initial angular velocity to get a specific movement.
class Pivot : public Sample
{
public:
	explicit Pivot( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.8f, 6.4f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Create a separate body on the ground
		{
			b2Vec2 v = { 5.0f, 0.0f };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 3.0f };
			bodyDef.gravityScale = 1.0f;
			bodyDef.linearVelocity = v;

			m_bodyId = b2CreateBody( m_worldId, &bodyDef );

			m_lever = 3.0f;
			b2Vec2 r = { 0.0f, -m_lever };

			float omega = b2Cross( v, r ) / b2Dot( r, r );
			b2Body_SetAngularVelocity( m_bodyId, omega );

			b2Polygon box = b2MakeBox( 0.1f, m_lever );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void Step() override
	{
		Sample::Step();

		b2Vec2 v = b2Body_GetLinearVelocity( m_bodyId );
		float omega = b2Body_GetAngularVelocity( m_bodyId );
		b2Vec2 r = b2Body_GetWorldVector( m_bodyId, { 0.0f, -m_lever } );

		b2Vec2 vp = v + b2CrossSV( omega, r );
		DrawTextLine( "pivot velocity = (%g, %g)", vp.x, vp.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Pivot( context );
	}

	b2BodyId m_bodyId;
	float m_lever;
};

static int samplePivot = RegisterSample( "Bodies", "Pivot", Pivot::Create );

// This shows how to drive a kinematic body to reach a target
class Kinematic : public Sample
{
public:
	explicit Kinematic( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 4.0f;
		}

		m_amplitude = 2.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position.x = 2.0f * m_amplitude;

			m_bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.1f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}

		m_time = 0.0f;
	}

	void Step() override
	{
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( m_context->pause && m_context->singleStep == false )
		{
			timeStep = 0.0f;
		}

		if ( timeStep > 0.0f )
		{
			b2Vec2 point = {
				.x = 2.0f * m_amplitude * cosf( m_time ),
				.y = m_amplitude * sinf( 2.0f * m_time ),
			};
			b2Rot rotation = b2MakeRot( 2.0f * m_time );

			b2Vec2 axis = b2RotateVector( rotation, { 0.0f, 1.0f } );
			m_context->draw.DrawSegment( point - 0.5f * axis, point + 0.5f * axis, b2_colorPlum );
			m_context->draw.DrawPoint( point, 10.0f, b2_colorPlum );

			b2Body_SetTargetTransform( m_bodyId, { point, rotation }, timeStep );
		}

		Sample::Step();

		m_time += timeStep;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Kinematic( context );
	}

	b2BodyId m_bodyId;
	float m_amplitude;
	float m_time;
};

static int sampleKinematic = RegisterSample( "Bodies", "Kinematic", Kinematic::Create );
