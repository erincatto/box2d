// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"

#include <imgui.h>

class BodyType : public Sample
{
public:
	explicit BodyType( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.8f, 6.4f };
			g_camera.m_zoom = 25.0f * 0.4f;
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
			shapeDef.friction = 0.6f;
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
			shapeDef.friction = 0.6f;
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
			shapeDef.friction = 0.6f;
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
			shapeDef.friction = 0.6f;
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
			shapeDef.friction = 0.6f;
			shapeDef.density = 2.0f;

			b2CreateCircleShape( m_floatingBodyId, &shapeDef, &circle );
		}
	}

	void UpdateUI() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
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

	void Step( Settings& settings ) override
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

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new BodyType( settings );
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

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on chain shapes.
class Character : public Sample
{
public:
	explicit Character( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { -2.0f, 7.0f };
			g_camera.m_zoom = 25.0f * 0.4f;
		}

		// Ground body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment1 = { { -8.0f, 1.0f }, { -6.0f, 1.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment1 );

			b2Segment segment2 = { { -6.0f, 1.0f }, { -4.0f, 1.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment2 );

			b2Segment segment3 = { { -4.0f, 1.0f }, { -2.0f, 1.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment3 );
		}

		// Chain shape
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.rotation = b2MakeRot( 0.25f * B2_PI );
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[4] = { { 8.0f, 7.0f }, { 7.0f, 8.0f }, { 6.0f, 8.0f }, { 5.0f, 7.0f } };
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 4;
			chainDef.isLoop = true;

			b2CreateChain( groundId, &chainDef );
		}

		// Square tiles. This shows that adjacency shapes may have non-smooth collision. Box2D has no solution
		// to this problem.
		// todo_erin try this: https://briansemrau.github.io/dealing-with-ghost-collisions/
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 1.0f, 1.0f, { 4.0f, 3.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.0f, 1.0f, { 6.0f, 3.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.0f, 1.0f, { 8.0f, 3.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		// Square made from a chain loop. Collision should be smooth.
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[4] = { { -1.0f, 3.0 }, { 1.0f, 3.0f }, { 1.0f, 5.0f }, { -1.0f, 5.0 } };
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 4;
			chainDef.isLoop = true;
			b2CreateChain( groundId, &chainDef );
		}

		// Chain loop. Collision should be smooth.
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -10.0f, 4.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[10] = { { 0.0f, 0.0f }, { 6.0f, 0.0f },  { 6.0f, 2.0f },	{ 4.0f, 1.0f },	 { 2.0f, 2.0f },
								  { 0.0f, 2.0f }, { -2.0f, 2.0f }, { -4.0f, 3.0f }, { -6.0f, 2.0f }, { -6.0f, 0.0f } };
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 10;
			chainDef.isLoop = true;
			b2CreateChain( groundId, &chainDef );
		}

		// Circle character
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -7.0f, 6.0f };
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_circleCharacterId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle circle = { { 0.0f, 0.0f }, 0.25f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2CreateCircleShape( m_circleCharacterId, &shapeDef, &circle );
		}

		// Capsule character
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 3.0f, 5.0f };
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_capsuleCharacterId = b2CreateBody( m_worldId, &bodyDef );

			b2Capsule capsule = { { 0.0f, 0.25f }, { 0.0f, 0.75f }, 0.25f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2CreateCapsuleShape( m_capsuleCharacterId, &shapeDef, &capsule );
		}

		// Square character
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -3.0f, 8.0f };
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.enableSleep = false;

			m_boxCharacterId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.4f, 0.4f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.friction = 0.2f;
			b2CreatePolygonShape( m_boxCharacterId, &shapeDef, &box );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawString( 5, m_textLine, "This tests various character collision shapes." );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "Limitation: square and hexagon can snag on aligned boxes." );
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new Character( settings );
	}

	b2BodyId m_circleCharacterId;
	b2BodyId m_capsuleCharacterId;
	b2BodyId m_boxCharacterId;
};

static int sampleCharacter = RegisterSample( "Bodies", "Character", Character::Create );

class Weeble : public Sample
{
public:
	explicit Weeble( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 2.3f, 10.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
		}

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
			shapeDef.density = 1.0f;
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

	void UpdateUI() override
	{
		float height = 120.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
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

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawCircle( m_explosionPosition, m_explosionRadius, b2_colorAzure );
	}

	static Sample* Create( Settings& settings )
	{
		return new Weeble( settings );
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
	explicit Sleep( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 3.0f, 50.0f };
			g_camera.m_zoom = 25.0f * 2.2f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
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
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
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

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

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
			g_draw.DrawString( 5, m_textLine, "sensor touch %d = %s", i, m_sensorTouching[i] ? "true" : "false" );
			m_textLine += m_textIncrement;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Sleep( settings );
	}

	b2BodyId m_pendulumId;
	b2ShapeId m_groundShapeId;
	b2ShapeId m_sensorIds[2];
	bool m_sensorTouching[2];
};

static int sampleSleep = RegisterSample( "Bodies", "Sleep", Sleep::Create );

class BadBody : public Sample
{
public:
	explicit BadBody( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 2.3f, 10.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
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

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawString( 5, m_textLine, "A bad body is a dynamic body with no mass and behaves like a kinematic body." );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "Bad bodies are considered invalid and a user bug. Behavior is not guaranteed." );
		m_textLine += m_textIncrement;

		// For science
		b2Body_ApplyForceToCenter( m_badBodyId, { 0.0f, 10.0f }, true );
	}

	static Sample* Create( Settings& settings )
	{
		return new BadBody( settings );
	}

	b2BodyId m_badBodyId;
};

static int sampleBadBody = RegisterSample( "Bodies", "Bad", BadBody::Create );

class Pivot : public Sample
{
public:
	explicit Pivot( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.8f, 6.4f };
			g_camera.m_zoom = 25.0f * 0.4f;
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
			bodyDef.gravityScale = 0.0f;
			bodyDef.linearVelocity = v;

			m_bodyId = b2CreateBody( m_worldId, &bodyDef );

			m_lever = 3.0f;
			b2Vec2 r = { 0.0f, -m_lever };

			float omega = b2Cross(v, r) / b2Dot(r, r);
			b2Body_SetAngularVelocity( m_bodyId, omega );

			b2Polygon box = b2MakeBox( 0.1f, m_lever );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		b2Vec2 v = b2Body_GetLinearVelocity( m_bodyId );
		float omega = b2Body_GetAngularVelocity( m_bodyId );
		b2Vec2 r = b2Body_GetWorldVector( m_bodyId, { 0.0f, -m_lever } );

		b2Vec2 vp = v + b2CrossSV( omega, r );
		g_draw.DrawString( 5, m_textLine, "pivot velocity = (%g, %g)", vp.x, vp.y);
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new Pivot( settings );
	}

	b2BodyId m_bodyId;
	float m_lever;
};

static int samplePivot = RegisterSample( "Bodies", "Pivot", Pivot::Create );
