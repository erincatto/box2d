// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "human.h"
#include "random.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <vector>

class BounceHouse : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	struct HitEvent
	{
		b2Vec2 point;
		float speed;
		int stepIndex;
	};

	explicit BounceHouse( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 0.45f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		{
			b2Segment segment = { { -10.0f, -10.0f }, { 10.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2Segment segment = { { 10.0f, -10.0f }, { 10.0f, 10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2Segment segment = { { 10.0f, 10.0f }, { -10.0f, 10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2Segment segment = { { -10.0f, 10.0f }, { -10.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_shapeType = e_boxShape;
		m_bodyId = b2_nullBodyId;
		m_enableHitEvents = true;

		memset( m_hitEvents, 0, sizeof( m_hitEvents ) );

		Launch();
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2DestroyBody( m_bodyId );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearVelocity = { 10.0f, 20.0f };
		bodyDef.position = { 0.0f, 0.0f };
		bodyDef.gravityScale = 0.0f;

		// Circle shapes centered on the body can spin fast without risk of tunnelling.
		bodyDef.allowFastRotation = m_shapeType == e_circleShape;

		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.restitution = 1.2f;
		shapeDef.friction = 0.3f;
		shapeDef.enableHitEvents = m_enableHitEvents;

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0 }, 0.25f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float h = 0.1f;
			b2Polygon box = b2MakeBox( 20.0f * h, h );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Bounce House", nullptr, ImGuiWindowFlags_NoResize );

		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = int( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = ShapeType( shapeType );
			Launch();
		}

		if ( ImGui::Checkbox( "hit events", &m_enableHitEvents ) )
		{
			b2Body_EnableHitEvents( m_bodyId, m_enableHitEvents );
		}

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			b2ContactHitEvent* event = events.hitEvents + i;

			HitEvent* e = m_hitEvents + 0;
			for ( int j = 1; j < 4; ++j )
			{
				if ( m_hitEvents[j].stepIndex < e->stepIndex )
				{
					e = m_hitEvents + j;
				}
			}

			e->point = event->point;
			e->speed = event->approachSpeed;
			e->stepIndex = m_stepCount;
		}

		for ( int i = 0; i < 4; ++i )
		{
			HitEvent* e = m_hitEvents + i;
			if ( e->stepIndex > 0 && m_stepCount <= e->stepIndex + 30 )
			{
				g_draw.DrawCircle( e->point, 0.1f, b2_colorOrangeRed );
				g_draw.DrawString( e->point, "%.1f", e->speed );
			}
		}

		if ( m_stepCount == 1000 )
		{
			m_stepCount += 0;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new BounceHouse( settings );
	}

	HitEvent m_hitEvents[4];
	b2BodyId m_bodyId;
	ShapeType m_shapeType;
	bool m_enableHitEvents;
};

static int sampleBounceHouse = RegisterSample( "Continuous", "Bounce House", BounceHouse::Create );

class FastChain : public Sample
{
public:
	explicit FastChain( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 0.35f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -6.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Vec2 points[4] = { { -10.0f, -2.0f }, { 10.0f, -2.0f }, { 10.0f, 1.0f }, { -10.0f, 1.0f } };

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = 4;
		chainDef.isLoop = true;

		b2CreateChain( groundId, &chainDef );

		m_bodyId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2DestroyBody( m_bodyId );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.linearVelocity = { 0.0f, -200.0f };
		bodyDef.position = { 0.0f, 10.0f };
		bodyDef.gravityScale = 1.0f;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
	}

	void UpdateUI() override
	{
		float height = 70.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Fast Chain", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new FastChain( settings );
	}

	b2BodyId m_bodyId;
};

static int sampleFastChainHouse = RegisterSample( "Continuous", "Fast Chain", FastChain::Create );

class SkinnyBox : public Sample
{
public:
	explicit SkinnyBox( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 1.0f, 5.0f };
			g_camera.m_zoom = 25.0f * 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.9f;
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			b2Polygon box = b2MakeOffsetBox( 0.1f, 1.0f, { 0.0f, 1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_autoTest = false;
		m_bullet = false;
		m_capsule = false;

		m_bodyId = b2_nullBodyId;
		m_bulletId = b2_nullBodyId;

		Launch();
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2DestroyBody( m_bodyId );
		}

		if ( B2_IS_NON_NULL( m_bulletId ) )
		{
			b2DestroyBody( m_bulletId );
		}

		m_angularVelocity = RandomFloatRange( -50.0f, 50.0f );
		// m_angularVelocity = -30.6695766f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 8.0f };
		bodyDef.angularVelocity = m_angularVelocity;
		bodyDef.linearVelocity = { 0.0f, -100.0f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.9f;

		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		if ( m_capsule )
		{
			b2Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 0.1f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			b2Polygon polygon = b2MakeBox( 2.0f, 0.05f );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &polygon );
		}

		if ( m_bullet )
		{
			b2Polygon polygon = b2MakeBox( 0.25f, 0.25f );
			m_x = RandomFloatRange( -1.0f, 1.0f );
			bodyDef.position = { m_x, 10.0f };
			bodyDef.linearVelocity = { 0.0f, -50.0f };
			m_bulletId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bulletId, &shapeDef, &polygon );
		}
	}

	void UpdateUI() override
	{
		float height = 110.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 140.0f, height ) );

		ImGui::Begin( "Skinny Box", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::Checkbox( "Capsule", &m_capsule );

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::Checkbox( "Auto Test", &m_autoTest );

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if ( m_autoTest && m_stepCount % 60 == 0 )
		{
			Launch();
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new SkinnyBox( settings );
	}

	b2BodyId m_bodyId, m_bulletId;
	float m_angularVelocity;
	float m_x;
	bool m_capsule;
	bool m_autoTest;
	bool m_bullet;
};

static int sampleSkinnyBox = RegisterSample( "Continuous", "Skinny Box", SkinnyBox::Create );

// This sample shows ghost bumps
class GhostBumps : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	explicit GhostBumps( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 1.5f, 16.0f };
			g_camera.m_zoom = 25.0f * 0.8f;
		}

		m_groundId = b2_nullBodyId;
		m_bodyId = b2_nullBodyId;
		m_shapeId = b2_nullShapeId;
		m_shapeType = e_circleShape;
		m_round = 0.0f;
		m_friction = 0.2f;
		m_bevel = 0.0f;
		m_useChain = true;

		CreateScene();
		Launch();
	}

	void CreateScene()
	{
		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
		}

		m_shapeId = b2_nullShapeId;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		float m = 1.0f / sqrt( 2.0f );
		float mm = 2.0f * ( sqrt( 2.0f ) - 1.0f );
		float hx = 4.0f, hy = 0.25f;

		if ( m_useChain )
		{
			b2Vec2 points[20];
			points[0] = { -3.0f * hx, hy };
			points[1] = b2Add( points[0], { -2.0f * hx * m, 2.0f * hx * m } );
			points[2] = b2Add( points[1], { -2.0f * hx * m, 2.0f * hx * m } );
			points[3] = b2Add( points[2], { -2.0f * hx * m, 2.0f * hx * m } );
			points[4] = b2Add( points[3], { -2.0f * hy * m, -2.0f * hy * m } );
			points[5] = b2Add( points[4], { 2.0f * hx * m, -2.0f * hx * m } );
			points[6] = b2Add( points[5], { 2.0f * hx * m, -2.0f * hx * m } );
			points[7] =
				b2Add( points[6], { 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ), -2.0f * hx * m - 2.0f * hy * ( 1.0f - m ) } );
			points[8] = b2Add( points[7], { 2.0f * hx + hy * mm, 0.0f } );
			points[9] = b2Add( points[8], { 2.0f * hx, 0.0f } );
			points[10] = b2Add( points[9], { 2.0f * hx + hy * mm, 0.0f } );
			points[11] =
				b2Add( points[10], { 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ), 2.0f * hx * m + 2.0f * hy * ( 1.0f - m ) } );
			points[12] = b2Add( points[11], { 2.0f * hx * m, 2.0f * hx * m } );
			points[13] = b2Add( points[12], { 2.0f * hx * m, 2.0f * hx * m } );
			points[14] = b2Add( points[13], { -2.0f * hy * m, 2.0f * hy * m } );
			points[15] = b2Add( points[14], { -2.0f * hx * m, -2.0f * hx * m } );
			points[16] = b2Add( points[15], { -2.0f * hx * m, -2.0f * hx * m } );
			points[17] = b2Add( points[16], { -2.0f * hx * m, -2.0f * hx * m } );
			points[18] = b2Add( points[17], { -2.0f * hx, 0.0f } );
			points[19] = b2Add( points[18], { -2.0f * hx, 0.0f } );

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 20;
			chainDef.isLoop = true;
			chainDef.friction = m_friction;

			b2CreateChain( m_groundId, &chainDef );
		}
		else
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = m_friction;

			b2Hull hull = { 0 };

			if ( m_bevel > 0.0f )
			{
				float hb = m_bevel;
				b2Vec2 vs[8] = { { hx + hb, hy - 0.05f },	{ hx, hy },	  { -hx, hy }, { -hx - hb, hy - 0.05f },
								 { -hx - hb, -hy + 0.05f }, { -hx, -hy }, { hx, -hy }, { hx + hb, -hy + 0.05f } };
				hull = b2ComputeHull( vs, 8 );
			}
			else
			{
				b2Vec2 vs[4] = { { hx, hy }, { -hx, hy }, { -hx, -hy }, { hx, -hy } };
				hull = b2ComputeHull( vs, 4 );
			}

			b2Transform transform;
			float x, y;

			// Left slope
			x = -3.0f * hx - m * hx - m * hy;
			y = hy + m * hx - m * hy;
			transform.q = b2MakeRot( -0.25f * B2_PI );

			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x -= 2.0f * m * hx;
				y += 2.0f * m * hx;
			}

			x = -2.0f * hx;
			y = 0.0f;
			transform.q = b2MakeRot( 0.0f );

			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * hx;
			}

			x = 3.0f * hx + m * hx + m * hy;
			y = hy + m * hx - m * hy;
			transform.q = b2MakeRot( 0.25f * B2_PI );

			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
			{
				transform.p = { x, y };
				b2Polygon polygon = b2MakeOffsetPolygon( &hull, transform.p, transform.q );
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
				x += 2.0f * m * hx;
				y += 2.0f * m * hx;
			}
		}
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2DestroyBody( m_bodyId );
			m_shapeId = b2_nullShapeId;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { -28.0f, 18.0f };
		bodyDef.linearVelocity = { 0.0f, 0.0f };
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = m_friction;

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			m_shapeId = b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0 }, 0.25f };
			m_shapeId = b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float h = 0.5f - m_round;
			b2Polygon box = b2MakeRoundedBox( h, 2.0f * h, m_round );
			m_shapeId = b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void UpdateUI() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

		ImGui::Begin( "Ghost Bumps", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		if ( ImGui::Checkbox( "Chain", &m_useChain ) )
		{
			CreateScene();
		}

		if ( m_useChain == false )
		{
			if ( ImGui::SliderFloat( "Bevel", &m_bevel, 0.0f, 1.0f, "%.2f" ) )
			{
				CreateScene();
			}
		}

		{
			const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
			int shapeType = int( m_shapeType );
			ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
			m_shapeType = ShapeType( shapeType );
		}

		if ( m_shapeType == e_boxShape )
		{
			ImGui::SliderFloat( "Round", &m_round, 0.0f, 0.4f, "%.1f" );
		}

		if ( ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.1f" ) )
		{
			if ( B2_IS_NON_NULL( m_shapeId ) )
			{
				b2Shape_SetFriction( m_shapeId, m_friction );
			}

			CreateScene();
		}

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new GhostBumps( settings );
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyId;
	b2ShapeId m_shapeId;
	ShapeType m_shapeType;
	float m_round;
	float m_friction;
	float m_bevel;
	bool m_useChain;
};

static int sampleGhostCollision = RegisterSample( "Continuous", "Ghost Bumps", GhostBumps::Create );

// Speculative collision failure case suggested by Dirk Gregorius. This uses
// a simple fallback scheme to prevent tunneling.
class SpeculativeFallback : public Sample
{
public:
	explicit SpeculativeFallback( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 1.0f, 5.0f };
			g_camera.m_zoom = 25.0f * 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			b2Vec2 points[5] = { { -2.0f, 4.0f }, { 2.0f, 4.0f }, { 2.0f, 4.1f }, { -0.5f, 4.2f }, { -2.0f, 4.2f } };
			b2Hull hull = b2ComputeHull( points, 5 );
			b2Polygon poly = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &poly );
		}

		// Fast moving skinny box. Also testing a large shape offset.
		{
			float offset = 8.0f;
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { offset, 12.0f };
			bodyDef.linearVelocity = { 0.0f, -100.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 2.0f, 0.05f, { -offset, 0.0f }, b2MakeRot( B2_PI ) );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new SpeculativeFallback( settings );
	}
};

static int sampleSpeculativeFallback = RegisterSample( "Continuous", "Speculative Fallback", SpeculativeFallback::Create );

// This shows that while Box2D uses speculative collision, it does not lead to speculative ghost collisions at small distances
class SpeculativeGhost : public Sample
{
public:
	explicit SpeculativeGhost( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 1.75f };
			g_camera.m_zoom = 2.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			b2Polygon box = b2MakeOffsetBox( 1.0f, 0.1f, { 0.0f, 0.9f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			// The speculative distance is 0.02 meters, so this avoid it
			bodyDef.position = { 0.015f, 2.515f };
			bodyDef.linearVelocity = { 0.1f * 1.25f * settings.hertz, -0.1f * 1.25f * settings.hertz };
			bodyDef.gravityScale = 0.0f;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeSquare( 0.25f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new SpeculativeGhost( settings );
	}
};

static int sampleSpeculativeGhost = RegisterSample( "Continuous", "Speculative Ghost", SpeculativeGhost::Create );

// This shows that Box2D does not have pixel perfect collision.
class PixelImperfect : public Sample
{
public:
	explicit PixelImperfect( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 7.0f, 5.0f };
			g_camera.m_zoom = 6.0f;
		}

		float pixelsPerMeter = 30.f;

		{
			b2BodyDef block4BodyDef = b2DefaultBodyDef();
			block4BodyDef.type = b2_staticBody;
			block4BodyDef.position = { 175.f / pixelsPerMeter, 150.f / pixelsPerMeter };
			b2BodyId block4BodyId = b2CreateBody( m_worldId, &block4BodyDef );
			b2Polygon block4Shape = b2MakeBox( 20.f / pixelsPerMeter, 10.f / pixelsPerMeter );
			b2ShapeDef block4ShapeDef = b2DefaultShapeDef();
			block4ShapeDef.friction = 0.f;
			b2CreatePolygonShape( block4BodyId, &block4ShapeDef, &block4Shape );
		}

		{
			b2BodyDef ballBodyDef = b2DefaultBodyDef();
			ballBodyDef.type = b2_dynamicBody;
			ballBodyDef.position = { 200.0f / pixelsPerMeter, 275.f / pixelsPerMeter };
			ballBodyDef.gravityScale = 0.0f;

			m_ballId = b2CreateBody( m_worldId, &ballBodyDef );
			// Ball shape
			//b2Polygon ballShape = b2MakeBox( 5.f / pixelsPerMeter, 5.f / pixelsPerMeter );
			b2Polygon ballShape = b2MakeRoundedBox( 4.0f / pixelsPerMeter, 4.0f / pixelsPerMeter, 0.9f / pixelsPerMeter );
			b2ShapeDef ballShapeDef = b2DefaultShapeDef();
			ballShapeDef.friction = 0.f;
			//ballShapeDef.restitution = 1.f;
			b2CreatePolygonShape( m_ballId, &ballShapeDef, &ballShape );
			b2Body_SetLinearVelocity( m_ballId, { 0.f, -5.0f } );
			b2Body_SetFixedRotation( m_ballId, true );
		}
	}

	void Step( Settings& settings ) override
	{
		b2ContactData data;
		b2Body_GetContactData( m_ballId, &data, 1 );

		b2Vec2 p = b2Body_GetPosition( m_ballId );
		b2Vec2 v = b2Body_GetLinearVelocity( m_ballId );
		g_draw.DrawString( 5, m_textLine, "p.x = %.9f, v.y = %.9f", p.x, v.y );
		m_textLine += m_textIncrement;

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new PixelImperfect( settings );
	}

	b2BodyId m_ballId;
};

static int samplePixelImperfect = RegisterSample( "Continuous", "Pixel Imperfect", PixelImperfect::Create );

class RestitutionThreshold : public Sample
{
public:
	explicit RestitutionThreshold( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 7.0f, 5.0f };
			g_camera.m_zoom = 6.0f;
		}

		float pixelsPerMeter = 30.f;

		// With the default threshold the ball will not bounce.
		b2World_SetRestitutionThreshold( m_worldId, 0.1f );

		{
			b2BodyDef block0BodyDef = b2DefaultBodyDef();
			block0BodyDef.type = b2_staticBody;
			block0BodyDef.position = { 205.f / pixelsPerMeter, 120.f / pixelsPerMeter };
			block0BodyDef.rotation = b2MakeRot( 70.f * 3.14f / 180.f );
			b2BodyId block0BodyId = b2CreateBody( m_worldId, &block0BodyDef );
			b2Polygon block0Shape = b2MakeBox( 50.f / pixelsPerMeter, 5.f / pixelsPerMeter );
			b2ShapeDef block0ShapeDef = b2DefaultShapeDef();
			block0ShapeDef.friction = 0.f;
			b2CreatePolygonShape( block0BodyId, &block0ShapeDef, &block0Shape );
		}

		{
			// Make a ball
			b2BodyDef ballBodyDef = b2DefaultBodyDef();
			ballBodyDef.type = b2_dynamicBody;
			ballBodyDef.position = { 200.f / pixelsPerMeter, 250.f / pixelsPerMeter };
			m_ballId = b2CreateBody( m_worldId, &ballBodyDef );

			b2Circle ballShape = {};
			ballShape.radius = 5.f / pixelsPerMeter;
			b2ShapeDef ballShapeDef = b2DefaultShapeDef();
			ballShapeDef.friction = 0.f;
			ballShapeDef.restitution = 1.f;
			b2CreateCircleShape( m_ballId, &ballShapeDef, &ballShape );

			b2Body_SetLinearVelocity( m_ballId, { 0.f, -2.9f } ); // Initial velocity
			b2Body_SetFixedRotation( m_ballId, true );						// Do not rotate a ball
		}
	}

	void Step( Settings& settings ) override
	{
		b2ContactData data;
		b2Body_GetContactData( m_ballId, &data, 1 );

		b2Vec2 p = b2Body_GetPosition( m_ballId );
		b2Vec2 v = b2Body_GetLinearVelocity( m_ballId );
		g_draw.DrawString( 5, m_textLine, "p.x = %.9f, v.y = %.9f", p.x, v.y );
		m_textLine += m_textIncrement;

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new RestitutionThreshold( settings );
	}

	b2BodyId m_ballId;
};

static int sampleRestitutionThreshold = RegisterSample( "Continuous", "Restitution Threshold", RestitutionThreshold::Create );

class Drop : public Sample
{
public:
	explicit Drop( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 1.5f };
			g_camera.m_zoom = 3.0f;
			settings.enableSleep = false;
			settings.drawJoints = false;
		}

#if 0
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float w = 0.25f;
			int count = 40;

			float x = -0.5f * count * w;
			float h = 0.05f;
			for ( int j = 0; j <= count; ++j )
			{
				b2Polygon box = b2MakeOffsetBox( w, h, { x, -h }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				x += w;
			}
		}
#endif

		m_human = {};
		m_frameSkip = 0;
		m_frameCount = 0;
		m_continuous = true;
		m_speculative = true;

		Scene1();
	}

	void Clear()
	{
		for ( int i = 0; i < m_bodyIds.size(); ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
		}

		m_bodyIds.clear();

		if ( m_human.isSpawned )
		{
			DestroyHuman( &m_human );
		}
	}

	void CreateGround1()
	{
		for ( int i = 0; i < m_groundIds.size(); ++i )
		{
			b2DestroyBody( m_groundIds[i] );
		}
		m_groundIds.clear();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float w = 0.25f;
		int count = 40;
		b2Segment segment = { { -0.5f * count * w, 0.0f }, { 0.5f * count * w, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		m_groundIds.push_back( groundId );
	}

	void CreateGround2()
	{
		for ( int i = 0; i < m_groundIds.size(); ++i )
		{
			b2DestroyBody( m_groundIds[i] );
		}
		m_groundIds.clear();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float w = 0.25f;
		int count = 40;

		float x = -0.5f * count * w;
		float h = 0.05f;
		for ( int j = 0; j <= count; ++j )
		{
			b2Polygon box = b2MakeOffsetBox( 0.5f * w, h, { x, 0.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
			x += w;
		}

		m_groundIds.push_back( groundId );
	}

	void CreateGround3()
	{
		for ( int i = 0; i < m_groundIds.size(); ++i )
		{
			b2DestroyBody( m_groundIds[i] );
		}
		m_groundIds.clear();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float w = 0.25f;
		int count = 40;
		b2Segment segment = { { -0.5f * count * w, 0.0f }, { 0.5f * count * w, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		segment = { { 3.0f, 0.0f }, { 3.0f, 8.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		m_groundIds.push_back( groundId );
	}

	// ball
	void Scene1()
	{
		Clear();
		CreateGround2();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f };
		bodyDef.linearVelocity = { 0.0f, -100.0f };

		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Circle circle = { { 0.0f, 0.0f }, 0.125f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );

		m_bodyIds.push_back( bodyId );
		m_frameCount = 1;
	}

	// ruler
	void Scene2()
	{
		Clear();
		CreateGround1();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 4.0f };
		bodyDef.rotation = b2MakeRot( 0.5f * B2_PI );
		bodyDef.linearVelocity = { 0.0f, 0.0f };
		bodyDef.angularVelocity = -0.5f;

		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeBox( 0.75f, 0.01f );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );

		m_bodyIds.push_back( bodyId );
		m_frameCount = 1;
	}

	// ragdoll
	void Scene3()
	{
		Clear();
		CreateGround2();

		float jointFrictionTorque = 0.03f;
		float jointHertz = 1.0f;
		float jointDampingRatio = 0.5f;

		CreateHuman( &m_human, m_worldId, { 0.0f, 40.0f }, 1.0f, jointFrictionTorque, jointHertz, jointDampingRatio, 1, nullptr,
					 true );

		m_frameCount = 1;
	}

	void Scene4()
	{
		Clear();
		CreateGround3();

		float a = 0.25f;
		b2Polygon box = b2MakeSquare( a );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float offset = 0.01f;

		for ( int i = 0; i < 5; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			float shift = ( i % 2 == 0 ? -offset : offset );
			bodyDef.position = { 2.5f + shift, a + 2.0f * a * i };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			m_bodyIds.push_back( bodyId );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		b2Circle circle = { { 0.0f, 0.0f }, 0.125f };
		shapeDef.density = 4.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -7.7f, 1.9f };
			bodyDef.linearVelocity = { 200.0f, 0.0f };
			bodyDef.isBullet = true;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
			m_bodyIds.push_back( bodyId );
		}

		m_frameCount = 1;
	}

	void Keyboard( int key ) override
	{
		switch ( key )
		{
			case GLFW_KEY_1:
				Scene1();
				break;

			case GLFW_KEY_2:
				Scene2();
				break;

			case GLFW_KEY_3:
				Scene3();
				break;

			case GLFW_KEY_4:
				Scene4();
				break;

			case GLFW_KEY_C:
				Clear();
				m_continuous = !m_continuous;
				break;

			case GLFW_KEY_V:
				Clear();
				m_speculative = !m_speculative;
				b2World_EnableSpeculative( m_worldId, m_speculative );
				break;

			case GLFW_KEY_S:
				m_frameSkip = m_frameSkip > 0 ? 0 : 60;
				break;

			default:
				Sample::Keyboard( key );
				break;
		}
	}

	void Step( Settings& settings ) override
	{
#if 0
		ImGui::SetNextWindowPos( ImVec2( 0.0f, 0.0f ) );
		ImGui::SetNextWindowSize( ImVec2( float( g_camera.m_width ), float( g_camera.m_height ) ) );
		ImGui::SetNextWindowBgAlpha( 0.0f );
		ImGui::Begin( "DropBackground", nullptr,
					  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
						  ImGuiWindowFlags_NoScrollbar );

		ImDrawList* drawList = ImGui::GetWindowDrawList();

		const char* ContinuousText = m_continuous && m_speculative ? "Continuous ON" : "Continuous OFF";
		drawList->AddText( g_draw.m_largeFont, g_draw.m_largeFont->FontSize, { 40.0f, 40.0f }, IM_COL32_WHITE, ContinuousText );

		if ( m_frameSkip > 0 )
		{
			drawList->AddText( g_draw.m_mediumFont, g_draw.m_mediumFont->FontSize, { 40.0f, 40.0f + 64.0f + 20.0f },
							   IM_COL32( 200, 200, 200, 255 ), "Slow Time" );
		}

		ImGui::End();
#endif

		//if (m_frameCount == 165)
		//{
		//	settings.pause = true;
		//	m_frameSkip = 30;
		//}

		settings.enableContinuous = m_continuous;

		if ( ( m_frameSkip == 0 || m_frameCount % m_frameSkip == 0 ) && settings.pause == false )
		{
			Sample::Step( settings );
		}
		else
		{
			bool pause = settings.pause;
			settings.pause = true;
			Sample::Step( settings );
			settings.pause = pause;
		}

		m_frameCount += 1;
	}

	static Sample* Create( Settings& settings )
	{
		return new Drop( settings );
	}

	std::vector<b2BodyId> m_groundIds;
	std::vector<b2BodyId> m_bodyIds;
	Human m_human;
	int m_frameSkip;
	int m_frameCount;
	bool m_continuous;
	bool m_speculative;
};

static int sampleDrop = RegisterSample( "Continuous", "Drop", Drop::Create );

// This shows a fast moving body that uses continuous collision versus static and dynamic bodies.
// This is achieved by setting the ball body as a *bullet*.
class Pinball : public Sample
{
public:
	explicit Pinball( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 9.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
		}

		settings.drawJoints = false;

		// Ground body
		b2BodyId groundId = {};
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 vs[5] = { { -8.0f, 6.0f }, { -8.0f, 20.0f }, { 8.0f, 20.0f }, { 8.0f, 6.0f }, { 0.0f, -2.0f } };

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = vs;
			chainDef.count = 5;
			chainDef.isLoop = true;
			b2CreateChain( groundId, &chainDef );
		}

		// Flippers
		{
			b2Vec2 p1 = { -2.0f, 0.0f }, p2 = { 2.0f, 0.0f };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.enableSleep = false;

			bodyDef.position = p1;
			b2BodyId leftFlipperId = b2CreateBody( m_worldId, &bodyDef );

			bodyDef.position = p2;
			b2BodyId rightFlipperId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 1.75f, 0.2f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2CreatePolygonShape( leftFlipperId, &shapeDef, &box );
			b2CreatePolygonShape( rightFlipperId, &shapeDef, &box );

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.localAnchorB = b2Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 1000.0f;
			jointDef.enableLimit = true;

			jointDef.motorSpeed = 0.0f;
			jointDef.localAnchorA = p1;
			jointDef.bodyIdB = leftFlipperId;
			jointDef.lowerAngle = -30.0f * B2_PI / 180.0f;
			jointDef.upperAngle = 5.0f * B2_PI / 180.0f;
			m_leftJointId = b2CreateRevoluteJoint( m_worldId, &jointDef );

			jointDef.motorSpeed = 0.0f;
			jointDef.localAnchorA = p2;
			jointDef.bodyIdB = rightFlipperId;
			jointDef.lowerAngle = -5.0f * B2_PI / 180.0f;
			jointDef.upperAngle = 30.0f * B2_PI / 180.0f;
			m_rightJointId = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Spinners
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -4.0f, 17.0f };

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box1 = b2MakeBox( 1.5f, 0.125f );
			b2Polygon box2 = b2MakeBox( 0.125f, 1.5f );

			b2CreatePolygonShape( bodyId, &shapeDef, &box1 );
			b2CreatePolygonShape( bodyId, &shapeDef, &box2 );

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = bodyDef.position;
			jointDef.localAnchorB = b2Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 0.1f;
			b2CreateRevoluteJoint( m_worldId, &jointDef );

			bodyDef.position = { 4.0f, 8.0f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box1 );
			b2CreatePolygonShape( bodyId, &shapeDef, &box2 );
			jointDef.localAnchorA = bodyDef.position;
			jointDef.bodyIdB = bodyId;
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Bumpers
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -4.0f, 8.0f };

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.restitution = 1.5f;

			b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			bodyDef.position = { 4.0f, 17.0f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		// Ball
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 1.0f, 15.0f };
			bodyDef.type = b2_dynamicBody;
			bodyDef.isBullet = true;

			m_ballId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Circle circle = { { 0.0f, 0.0f }, 0.2f };
			b2CreateCircleShape( m_ballId, &shapeDef, &circle );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_SPACE ) == GLFW_PRESS )
		{
			b2RevoluteJoint_SetMotorSpeed( m_leftJointId, 20.0f );
			b2RevoluteJoint_SetMotorSpeed( m_rightJointId, -20.0f );
		}
		else
		{
			b2RevoluteJoint_SetMotorSpeed( m_leftJointId, -10.0f );
			b2RevoluteJoint_SetMotorSpeed( m_rightJointId, 10.0f );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Pinball( settings );
	}

	b2JointId m_leftJointId;
	b2JointId m_rightJointId;
	b2BodyId m_ballId;
};

static int samplePinball = RegisterSample( "Continuous", "Pinball", Pinball::Create );
