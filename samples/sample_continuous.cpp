// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "human.h"
#include "sample.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <float.h>
#include <imgui.h>
#include <vector>

// extern "C" int b2_toiHitCount;

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
		b2Pos point;
		float speed;
		int stepIndex;
	};

	explicit BounceHouse( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 25.0f * 0.45f;
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

		m_shapeType = e_circleShape;
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
		bodyDef.isBullet = true;

		// Circle shapes centered on the body can spin fast without risk of tunnelling.
		bodyDef.allowFastRotation = m_shapeType == e_circleShape;

		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = m_enableHitEvents;

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f };
			b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float h = 0.1f;
			b2Polygon box = b2MakeBox( 20.0f * h, h );
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = int( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = ShapeType( shapeType );
			Launch();
		}
		ImGui::PopItemWidth();

		if ( ImGui::Checkbox( "hit events", &m_enableHitEvents ) )
		{
			b2Body_EnableHitEvents( m_bodyId, m_enableHitEvents );
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

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
				DrawCircle( m_draw, e->point, 0.1f, b2_colorOrangeRed );
				DrawString( m_draw, m_camera, e->point, b2_colorWhite, "%.1f", e->speed );
			}
		}

		if ( m_stepCount == 1000 )
		{
			m_stepCount += 0;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BounceHouse( context );
	}

	HitEvent m_hitEvents[4];
	b2BodyId m_bodyId;
	ShapeType m_shapeType;
	bool m_enableHitEvents;
};

static int sampleBounceHouse = RegisterSample( "Continuous", "Bounce House", BounceHouse::Create );

class BounceHumans : public Sample
{
public:
	explicit BounceHumans( SampleContext* context )
		: Sample( context )
	{
		m_context->camera.center = { 0.0f, 0.0f };
		m_context->camera.zoom = 12.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = 1.3f;
		shapeDef.material.friction = 0.1f;

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

		b2Circle circle = { { 0.0f, 0.0f }, 2.0f };
		shapeDef.material.restitution = 2.0f;
		b2CreateCircleShape( groundId, &shapeDef, &circle );
	}

	void Step() override
	{
		if ( m_humanCount < 5 && m_countDown <= 0.0f )
		{
			float jointFrictionTorque = 0.0f;
			float jointHertz = 1.0f;
			float jointDampingRatio = 0.1f;

			CreateHuman( m_humans + m_humanCount, m_worldId, { 0.0f, 5.0f }, 1.0f, jointFrictionTorque, jointHertz,
						 jointDampingRatio, 1, nullptr, true );
			// Human_SetVelocity( m_humans + m_humanCount, { 10.0f - 5.0f * m_humanCount, -20.0f + 5.0f * m_humanCount } );

			m_countDown = 2.0f;
			m_humanCount += 1;
		}

		float timeStep = 1.0f / 60.0f;
		b2CosSin cs1 = b2ComputeCosSin( 0.5f * m_time );
		b2CosSin cs2 = b2ComputeCosSin( m_time );
		float gravity = 10.0f;
		b2Vec2 gravityVec = { gravity * cs1.sine, gravity * cs2.cosine };
		DrawLine( m_draw, b2Pos_zero, b2ToPos( b2Vec2{ 3.0f * cs1.sine, 3.0f * cs2.cosine } ), b2_colorWhite );
		m_time += timeStep;
		m_countDown -= timeStep;
		b2World_SetGravity( m_worldId, gravityVec );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BounceHumans( context );
	}

	Human m_humans[5] = {};
	int m_humanCount = 0;
	float m_countDown = 0.0f;
	float m_time = 0.0f;
};

static int sampleBounceHumans = RegisterSample( "Continuous", "Bounce Humans", BounceHumans::Create );

class ChainDrop : public Sample
{
public:
	explicit ChainDrop( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 25.0f * 0.35f;
		}

		//
		// b2World_SetContactTuning( m_worldId, 30.0f, 1.0f, 100.0f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -6.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Vec2 points[4] = { { -10.0f, -2.0f }, { 10.0f, -2.0f }, { 10.0f, 1.0f }, { -10.0f, 1.0f } };

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 4;
		chainDef.isLoop = true;

		b2CreateChain( groundId, &chainDef );

		m_bodyId = b2_nullBodyId;
		m_yOffset = -0.1f;
		m_speed = -42.0f;

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
		bodyDef.linearVelocity = { 0.0f, m_speed };
		bodyDef.position = { 0.0f, 10.0f + m_yOffset };
		bodyDef.rotation = b2MakeRot( 0.5f * B2_PI );
		bodyDef.motionLocks.angularZ = true;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		m_shapeId = b2CreateCircleShape( m_bodyId, &shapeDef, &circle );

		// b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f };
		// m_shapeId = b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );

		// float h = 0.5f;
		// b2Polygon box = b2MakeBox( h, h );
		// m_shapeId = b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		ImGui::SliderFloat( "Speed", &m_speed, -100.0f, 0.0f, "%.0f" );
		ImGui::SliderFloat( "Y Offset", &m_yOffset, -1.0f, 1.0f, "%.1f" );
		ImGui::PopItemWidth();

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new ChainDrop( context );
	}

	b2BodyId m_bodyId;
	b2ShapeId m_shapeId;
	float m_yOffset;
	float m_speed;
};

static int sampleChainDrop = RegisterSample( "Continuous", "Chain Drop", ChainDrop::Create );

class ChainSlide : public Sample
{
public:
	explicit ChainSlide( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 10.0f };
			m_context->camera.zoom = 15.0f;
		}

		// b2_toiHitCount = 0;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			constexpr int count = 80;
			b2Vec2 points[count];

			float w = 2.0f;
			float h = 1.0f;
			float x = 20.0f, y = 0.0f;
			for ( int i = 0; i < 20; ++i )
			{
				points[i] = { x, y };
				x -= w;
			}

			for ( int i = 20; i < 40; ++i )
			{
				points[i] = { x, y };
				y += h;
			}

			for ( int i = 40; i < 60; ++i )
			{
				points[i] = { x, y };
				x += w;
			}

			for ( int i = 60; i < 80; ++i )
			{
				points[i] = { x, y };
				y -= h;
			}

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = count;
			chainDef.isLoop = true;

			b2CreateChain( groundId, &chainDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.linearVelocity = { 100.0f, 0.0f };
			bodyDef.position = { -19.5f, 0.0f + 0.5f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.0f;
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		// DrawTextLine( "toi hits = %d", b2_toiHitCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new ChainSlide( context );
	}
};

static int sampleChainSlide = RegisterSample( "Continuous", "Chain Slide", ChainSlide::Create );

class SegmentSlide : public Sample
{
public:
	explicit SegmentSlide( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 10.0f };
			m_context->camera.zoom = 15.0f;
		}

		// b2_toiHitCount = 0;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			segment = { { 40.0f, 0.0f }, { 40.0f, 10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.linearVelocity = { 100.0f, 0.0f };
			bodyDef.position = { -20.0f, 0.7f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			// shapeDef.friction = 0.0f;
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		// DrawTextLine("toi hits = %d", b2_toiHitCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SegmentSlide( context );
	}
};

static int sampleSegmentSlide = RegisterSample( "Continuous", "Segment Slide", SegmentSlide::Create );

class SkinnyBox : public Sample
{
public:
	explicit SkinnyBox( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.0f, 5.0f };
			m_context->camera.zoom = 25.0f * 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.9f;
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
		shapeDef.material.friction = 0.9f;

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

	bool DrawControls() override
	{
		ImGui::Checkbox( "Capsule", &m_capsule );

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::Checkbox( "Auto Test", &m_autoTest );

		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_autoTest && m_stepCount % 60 == 0 )
		{
			Launch();
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new SkinnyBox( context );
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

	explicit GhostBumps( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.5f, 16.0f };
			m_context->camera.zoom = 25.0f * 0.8f;
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

			b2SurfaceMaterial material = {};
			material.friction = m_friction;

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.pointCount = 20;
			chainDef.isLoop = true;
			chainDef.materials = &material;
			chainDef.materialCount = 1;

			b2CreateChain( m_groundId, &chainDef );
		}
		else
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = m_friction;

			b2Hull hull = {};

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
		shapeDef.material.friction = m_friction;

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			m_shapeId = b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.25f };
			m_shapeId = b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float h = 0.5f - m_round;
			b2Polygon box = b2MakeRoundedBox( h, 2.0f * h, m_round );
			m_shapeId = b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

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

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new GhostBumps( context );
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
	explicit SpeculativeFallback( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 1.0f, 5.0f };
			m_context->camera.zoom = 25.0f * 0.25f;
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

	static Sample* Create( SampleContext* context )
	{
		return new SpeculativeFallback( context );
	}
};

static int sampleSpeculativeFallback = RegisterSample( "Continuous", "Speculative Fallback", SpeculativeFallback::Create );

class SpeculativeSliver : public Sample
{
public:
	explicit SpeculativeSliver( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 2.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 12.0f };
			bodyDef.linearVelocity = { 0.0f, -100.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Vec2 points[3] = { { -2.0f, 0.0f }, { -1.0f, 0.0f }, { 2.0f, 0.5f } };
			b2Hull hull = b2ComputeHull( points, 3 );
			b2Polygon poly = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &poly );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpeculativeSliver( context );
	}
};

static int sampleSpeculativeSliver = RegisterSample( "Continuous", "Speculative Sliver", SpeculativeSliver::Create );

// This shows that while Box2D uses speculative collision, it does not lead to speculative ghost collisions at small distances
class SpeculativeGhost : public Sample
{
public:
	explicit SpeculativeGhost( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 2.0f;
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
			bodyDef.linearVelocity = { 0.1f * 1.25f * m_context->hertz, -0.1f * 1.25f * m_context->hertz };
			bodyDef.gravityScale = 0.0f;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeSquare( 0.25f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new SpeculativeGhost( context );
	}
};

static int sampleSpeculativeGhost = RegisterSample( "Continuous", "Speculative Ghost", SpeculativeGhost::Create );

// This shows that Box2D does not have pixel perfect collision.
class PixelImperfect : public Sample
{
public:
	explicit PixelImperfect( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 7.0f, 5.0f };
			m_context->camera.zoom = 6.0f;
		}

		float pixelsPerMeter = 30.f;

		{
			b2BodyDef block4BodyDef = b2DefaultBodyDef();
			block4BodyDef.type = b2_staticBody;
			block4BodyDef.position = { 175.f / pixelsPerMeter, 150.f / pixelsPerMeter };
			b2BodyId block4BodyId = b2CreateBody( m_worldId, &block4BodyDef );
			b2Polygon block4Shape = b2MakeBox( 20.f / pixelsPerMeter, 10.f / pixelsPerMeter );
			b2ShapeDef block4ShapeDef = b2DefaultShapeDef();
			block4ShapeDef.material.friction = 0.f;
			b2CreatePolygonShape( block4BodyId, &block4ShapeDef, &block4Shape );
		}

		{
			b2BodyDef ballBodyDef = b2DefaultBodyDef();
			ballBodyDef.type = b2_dynamicBody;
			ballBodyDef.position = { 200.0f / pixelsPerMeter, 275.f / pixelsPerMeter };
			ballBodyDef.gravityScale = 0.0f;

			m_ballId = b2CreateBody( m_worldId, &ballBodyDef );
			// Ball shape
			// b2Polygon ballShape = b2MakeBox( 5.f / pixelsPerMeter, 5.f / pixelsPerMeter );
			b2Polygon ballShape = b2MakeRoundedBox( 4.0f / pixelsPerMeter, 4.0f / pixelsPerMeter, 0.9f / pixelsPerMeter );
			b2ShapeDef ballShapeDef = b2DefaultShapeDef();
			ballShapeDef.material.friction = 0.f;
			// ballShapeDef.restitution = 1.f;
			b2CreatePolygonShape( m_ballId, &ballShapeDef, &ballShape );
			b2Body_SetLinearVelocity( m_ballId, { 0.f, -5.0f } );
			b2Body_SetMotionLocks( m_ballId, { false, false, true } );
		}
	}

	void Step() override
	{
		b2ContactData data;
		b2Body_GetContactData( m_ballId, &data, 1 );

		b2Pos p = b2Body_GetPosition( m_ballId );
		b2Vec2 v = b2Body_GetLinearVelocity( m_ballId );
		DrawScreenTextLine( "p.x = %.9f, v.y = %.9f", p.x, v.y );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new PixelImperfect( context );
	}

	b2BodyId m_ballId;
};

static int samplePixelImperfect = RegisterSample( "Continuous", "Pixel Imperfect", PixelImperfect::Create );

class RestitutionThreshold : public Sample
{
public:
	explicit RestitutionThreshold( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 7.0f, 5.0f };
			m_context->camera.zoom = 6.0f;
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
			block0ShapeDef.material.friction = 0.f;
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
			ballShapeDef.material.friction = 0.f;
			ballShapeDef.material.restitution = 1.f;
			b2CreateCircleShape( m_ballId, &ballShapeDef, &ballShape );

			b2Body_SetLinearVelocity( m_ballId, { 0.f, -2.9f } );	   // Initial velocity
			b2Body_SetMotionLocks( m_ballId, { false, false, true } ); // Do not rotate a ball
		}
	}

	void Step() override
	{
		b2ContactData data;
		b2Body_GetContactData( m_ballId, &data, 1 );

		b2Pos p = b2Body_GetPosition( m_ballId );
		b2Vec2 v = b2Body_GetLinearVelocity( m_ballId );
		DrawScreenTextLine( "p.x = %.9f, v.y = %.9f", p.x, v.y );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new RestitutionThreshold( context );
	}

	b2BodyId m_ballId;
};

static int sampleRestitutionThreshold = RegisterSample( "Continuous", "Restitution Threshold", RestitutionThreshold::Create );

class Drop : public Sample
{
public:
	explicit Drop( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.5f };
			m_context->camera.zoom = 3.0f;
			m_context->enableSleep = false;
			m_context->debugDraw.drawJoints = false;
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

	void Keyboard( int key, int action, int mods ) override
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

			case GLFW_KEY_S:
				m_frameSkip = m_frameSkip > 0 ? 0 : 60;
				break;

			default:
				Sample::Keyboard( key, action, mods );
				break;
		}
	}

	void Step() override
	{
#if 0
		ImGui::SetNextWindowPos( ImVec2( 0.0f, 0.0f ) );
		ImGui::SetNextWindowSize( ImVec2( float( m_context->camera.width ), float( m_camera->height ) ) );
		ImGui::SetNextWindowBgAlpha( 0.0f );
		ImGui::Begin( "DropBackground", nullptr,
					  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
						  ImGuiWindowFlags_NoScrollbar );

		ImDrawList* drawList = ImGui::GetWindowDrawList();

		const char* ContinuousText = m_continuous && m_speculative ? "Continuous ON" : "Continuous OFF";
		drawList->AddText( m_largeFont, m_largeFont->FontSize, { 40.0f, 40.0f }, IM_COL32_WHITE, ContinuousText );

		if ( m_frameSkip > 0 )
		{
			drawList->AddText( m_mediumFont, m_mediumFont->FontSize, { 40.0f, 40.0f + 64.0f + 20.0f },
							   IM_COL32( 200, 200, 200, 255 ), "Slow Time" );
		}

		ImGui::End();
#endif

		// if (m_frameCount == 165)
		//{
		//	settings.pause = true;
		//	m_frameSkip = 30;
		// }

		m_context->enableContinuous = m_continuous;

		if ( ( m_frameSkip == 0 || m_frameCount % m_frameSkip == 0 ) && m_context->pause == false )
		{
			Sample::Step();
		}
		else
		{
			bool pause = m_context->pause;
			m_context->pause = true;
			Sample::Step();
			m_context->pause = pause;
		}

		m_frameCount += 1;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Drop( context );
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
	explicit Pinball( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 9.0f };
			m_context->camera.zoom = 25.0f * 0.5f;
		}

		m_context->debugDraw.drawJoints = false;

		// Ground body
		b2BodyId groundId = {};
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 vs[5] = { { -8.0f, 6.0f }, { -8.0f, 20.0f }, { 8.0f, 20.0f }, { 8.0f, 6.0f }, { 0.0f, -2.0f } };

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = vs;
			chainDef.pointCount = 5;
			chainDef.isLoop = true;
			b2CreateChain( groundId, &chainDef );
		}

		// Flippers
		{
			b2Pos p1 = { -2.0f, 0.0f }, p2 = { 2.0f, 0.0f };

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
			jointDef.base.bodyIdA = groundId;
			jointDef.base.localFrameB.p = b2Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 1000.0f;
			jointDef.enableLimit = true;

			jointDef.motorSpeed = 0.0f;
			jointDef.base.localFrameA.p = b2ToVec2( p1 );
			jointDef.base.bodyIdB = leftFlipperId;
			jointDef.lowerAngle = -30.0f * B2_PI / 180.0f;
			jointDef.upperAngle = 5.0f * B2_PI / 180.0f;
			m_leftJointId = b2CreateRevoluteJoint( m_worldId, &jointDef );

			jointDef.motorSpeed = 0.0f;
			jointDef.base.localFrameA.p = b2ToVec2( p2 );
			jointDef.base.bodyIdB = rightFlipperId;
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
			jointDef.base.bodyIdA = groundId;
			jointDef.base.bodyIdB = bodyId;
			jointDef.base.localFrameA.p = b2ToVec2( bodyDef.position );
			jointDef.base.localFrameB.p = b2Vec2_zero;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 0.1f;
			b2CreateRevoluteJoint( m_worldId, &jointDef );

			bodyDef.position = { 4.0f, 8.0f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box1 );
			b2CreatePolygonShape( bodyId, &shapeDef, &box2 );
			jointDef.base.localFrameA.p = b2ToVec2( bodyDef.position );
			jointDef.base.bodyIdB = bodyId;
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Bumpers
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -4.0f, 8.0f };

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.restitution = 1.5f;

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

	bool DrawControls() override
	{
		ImGui::Text( "Flipper: press A" );
		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			b2RevoluteJoint_SetTargetMotorSpeed( m_leftJointId, 20.0f );
			b2RevoluteJoint_SetTargetMotorSpeed( m_rightJointId, -20.0f );
		}
		else
		{
			b2RevoluteJoint_SetTargetMotorSpeed( m_leftJointId, -10.0f );
			b2RevoluteJoint_SetTargetMotorSpeed( m_rightJointId, 10.0f );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Pinball( context );
	}

	b2JointId m_leftJointId;
	b2JointId m_rightJointId;
	b2BodyId m_ballId;
};

static int samplePinball = RegisterSample( "Continuous", "Pinball", Pinball::Create );

// This shows the importance of secondary collisions in continuous physics.
// This also shows a difficult setup for the solver with an acute angle.
class Wedge : public Sample
{
public:
	explicit Wedge( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.5f };
			m_context->camera.zoom = 6.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -4.0f, 8.0f }, { 0.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
			segment = { { 0.0f, 0.0f }, { 0.0f, 8.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -0.45f, 10.75f };
			bodyDef.linearVelocity = { 0.0f, -200.0f };

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle circle = {};
			circle.radius = 0.3f;
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.2f;
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Wedge( context );
	}
};

static int sampleWedge = RegisterSample( "Continuous", "Wedge", Wedge::Create );

// This shows how adjusting the CCD safety factor can engage continuous collision at
// lower speeds to avoid the slight overlap that happens with discrete collision detection.
// The best way to run this sample is:
// 1. press pause (P)
// 2. restart (R) or press the Drop button
// 3. then single step (.)
// Then look at the metrics and overlap. The color of the shape gets darker when it is using
// continuous collision detection.
class SafetyFactor : public Sample
{
public:
	explicit SafetyFactor( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 3.0f };
			m_context->camera.zoom = 6.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( 20.0f, 1.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_bodyId = b2_nullBodyId;
		m_extent = 0.5f;
		m_height = 0.3f;
		m_safetyFactor = 0.1f;
		m_overlap = 0.0f;

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
		bodyDef.position = { 0.0f, m_height + m_extent };
		bodyDef.safetyFactor = m_safetyFactor;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeSquare( m_extent );
		b2CreatePolygonShape( m_bodyId, &shapeDef, &box );

		m_overlap = 0.0f;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep )
		{
			// Contacts are computed at the beginning of the step, so the manifold holds the initial separation.
			b2ContactData data;
			int count = b2Body_GetContactData( m_bodyId, &data, 1 );
			if ( count == 1 )
			{
				for ( int j = 0; j < data.manifold.pointCount; ++j )
				{
					m_overlap = b2MaxFloat( m_overlap, -data.manifold.points[j].separation );
				}
			}
		}

		DrawLine( m_draw, b2ToPos( b2Vec2{ -3.0f, -m_overlap } ), b2ToPos( b2Vec2{ 3.0f, -m_overlap } ), b2_colorRed );

		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		float motion = timeStep * b2Length( b2Body_GetLinearVelocity( m_bodyId ) );

		DrawScreenTextLine( "actual movement = %.3f m", motion );
		DrawScreenTextLine( "fast movement = %.3f m", m_safetyFactor * m_extent );
		DrawScreenTextLine( "overlap = %.4f m", m_overlap );
	}

	bool DrawControls() override
	{
		bool changed = false;

		ImGui::PushItemWidth( 10.0f * ImGui::GetFontSize() );
		changed |= ImGui::SliderFloat( "Height", &m_height, 0.0f, 2.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Safety", &m_safetyFactor, 0.0f, 1.0f, "%.2f" );
		ImGui::PopItemWidth();

		if ( ImGui::Button( "Drop" ) || changed )
		{
			Launch();
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new SafetyFactor( context );
	}

	b2BodyId m_bodyId;
	float m_extent;
	float m_height;
	float m_safetyFactor;
	float m_overlap;
};

static int sampleSafetyFactor = RegisterSample( "Continuous", "Safety Factor", SafetyFactor::Create );

// Ghost collision scenes from the ghostbusters fork (github.com/mache102/ghostbusters), rebuilt
// from static polygons. The terrain is either the polygons themselves or the chain loops that
// b2ComputeChains makes from them. Hop, sink and kick are measured against the travel surface. A
// snag is a step that loses more than 10% of the tangential speed.
class PolygonChains : public Sample
{
public:
	enum Scene
	{
		e_slidingBox,
		e_misalignedTiles,
		e_tiltedBox,
		e_fastLanding,
		e_marble,
		e_slopeTiles,
		e_slopeTriangles,
		e_tileWall,
		e_cliffPit,
		e_wallSlide,
		e_vehicleCrest,
		e_smallGaps,
		e_parabolaCrates,
		e_sceneCount
	};

	enum Terrain
	{
		e_polygons,
		e_chains,
	};

	struct SceneInfo
	{
		const char* name;
		const char* summary;
		const char* knobName;
		float speed;
		float gravityScale;
		float knob;
		float knobMax;
	};

	static constexpr SceneInfo m_scenes[e_sceneCount] = {
		{ "Sliding Box", "Locked box sliding over box tiles, gravity scale loads the contact", nullptr, 10.0f, 1.0f, 0.0f, 0.0f },
		{ "Misaligned Tiles", "Odd tiles raised by a small step, a step below the weld tolerance merges", "step height", 10.0f,
		  1.0f, 0.001f, 0.05f },
		{ "Tilted Box", "Locked box tilted onto one corner, point face contacts", "tilt (deg)", 10.0f, 1.0f, 10.0f, 45.0f },
		{ "Fast Landing", "Fast diagonal landing onto 0.5 m tiles", nullptr, 40.0f, 1.0f, 0.0f, 0.0f },
		{ "Marble", "Fast marble rolling over 1 m tiles", nullptr, 25.0f, 2.0f, 0.0f, 0.0f },
		{ "Slope Tiles", "Free box sliding down a 30 degree slope of slanted tiles", nullptr, 5.0f, 1.0f, 0.0f, 0.0f },
		{ "Slope Triangles", "Triangle tiles touching only at corners, chains warn and still snag", nullptr, 5.0f, 1.0f, 0.0f,
		  0.0f },
		{ "Tile Wall", "Box pushed into a wall of 0.5 m tiles, it should stop at x = 11.5", nullptr, 10.0f, 1.0f, 0.0f, 0.0f },
		{ "Cliff Pit", "Floor ends at a pit of stacked tiles, the box should end up in the pit", nullptr, 10.0f, 1.0f, 0.0f,
		  0.0f },
		{ "Wall Slide", "Box pushed into a tiled wall slides down into the corner", nullptr, 2.0f, 1.0f, 0.0f, 0.0f },
		{ "Vehicle Crest", "Vehicle over a crest onto a slope of quads, it should reach the lower floor", nullptr, 8.0f, 1.0f,
		  0.0f, 0.0f },
		{ "Small Gaps", "Tiles separated by small gaps, a gap below the weld tolerance merges", "gap", 10.0f, 1.0f, 0.05f,
		  0.2f },
		{ "Parabola Crates", "Ball rolling over unevenly cut static slices on a curved floor", nullptr, 6.0f, 1.0f, 0.0f, 0.0f },
	};

	struct PlayerDef
	{
		b2Vec2 position;
		b2Vec2 velocity;
		float angularVelocity;
		float friction;
		float restitution;
		bool lockRotation;
		bool isBullet;
	};

	explicit PolygonChains( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.zoom = 8.0f;
		}

		m_groundId = b2_nullBodyId;
		m_playerId = b2_nullBodyId;
		m_scene = e_slidingBox;
		m_terrain = e_chains;
		m_follow = true;
		m_weldTolerance = B2_LINEAR_SLOP;
		SetSceneDefaults();
		Rebuild();
	}

	void SetSceneDefaults()
	{
		const SceneInfo& info = m_scenes[m_scene];
		m_speed = info.speed;
		m_gravityScale = info.gravityScale;
		m_knob = info.knob;
	}

	void AddBox( float x0, float y0, float x1, float y1 )
	{
		b2Vec2 center = { 0.5f * ( x0 + x1 ), 0.5f * ( y0 + y1 ) };
		m_polygons.push_back( b2MakeOffsetBox( 0.5f * ( x1 - x0 ), 0.5f * ( y1 - y0 ), center, b2Rot_identity ) );
	}

	void AddTiledBlock( float x0, float y0, float x1, float y1, int nx, int ny )
	{
		float w = ( x1 - x0 ) / nx;
		float h = ( y1 - y0 ) / ny;
		for ( int i = 0; i < nx; ++i )
		{
			for ( int j = 0; j < ny; ++j )
			{
				AddBox( x0 + i * w, y0 + j * h, x0 + ( i + 1 ) * w, y0 + ( j + 1 ) * h );
			}
		}
	}

	// Floor tiles half a meter thick, odd tiles raised by the step
	void AddFloor( float x0, int count, float width, float topY, float step )
	{
		for ( int i = 0; i < count; ++i )
		{
			float y = ( i & 1 ) ? topY + step : topY;
			AddBox( x0 + i * width, y - 0.5f, x0 + ( i + 1 ) * width, y );
		}
	}

	void AddQuad( b2Vec2 a, b2Vec2 b, b2Vec2 c, b2Vec2 d )
	{
		b2Vec2 points[4] = { a, b, c, d };
		b2Hull hull = b2ComputeHull( points, 4 );
		m_polygons.push_back( b2MakePolygon( &hull, 0.0f ) );
	}

	PlayerDef MakePlayerDef( b2Vec2 position )
	{
		PlayerDef def = {};
		def.position = position;
		def.velocity = { m_speed, 0.0f };
		def.lockRotation = true;
		return def;
	}

	b2ShapeDef CreatePlayerBody( const PlayerDef& def )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2ToPos( def.position );
		bodyDef.linearVelocity = def.velocity;
		bodyDef.angularVelocity = def.angularVelocity;
		bodyDef.gravityScale = m_gravityScale;
		bodyDef.motionLocks.angularZ = def.lockRotation;
		bodyDef.isBullet = def.isBullet;
		bodyDef.enableSleep = false;
		m_playerId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = def.friction;
		shapeDef.material.restitution = def.restitution;
		return shapeDef;
	}

	void CreatePlayerPolygon( const PlayerDef& def, const b2Polygon& polygon )
	{
		b2ShapeDef shapeDef = CreatePlayerBody( def );
		b2CreatePolygonShape( m_playerId, &shapeDef, &polygon );
	}

	void CreatePlayerCircle( const PlayerDef& def, float radius )
	{
		b2ShapeDef shapeDef = CreatePlayerBody( def );
		b2Circle circle = { b2Vec2_zero, radius };
		b2CreateCircleShape( m_playerId, &shapeDef, &circle );
	}

	// The travel surface runs from p1 to p2 with the outside on the left. Progress is zero at p1.
	void SetFlatSurface( b2Vec2 p1, b2Vec2 p2, float restHeight, float measureStart, float pathLength )
	{
		m_surface = { p1, p2 };
		m_restHeight = restHeight;
		m_measureStart = measureStart;
		m_pathLength = pathLength;
	}

	void BuildScene()
	{
		switch ( m_scene )
		{
			case e_slidingBox:
			case e_misalignedTiles:
			{
				float step = m_scene == e_misalignedTiles ? m_knob : 0.0f;
				AddFloor( -2.0f, 24, 2.0f, 0.0f, step );
				PlayerDef def = MakePlayerDef( { 0.0f, 0.5f } );
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );
				SetFlatSurface( { 0.0f, 0.0f }, { 46.0f, 0.0f }, 0.5f, 0.5f, 40.0f );
			}
			break;

			case e_tiltedBox:
			{
				AddFloor( -2.0f, 24, 2.0f, 0.0f, 0.0f );
				float h = 0.5f;
				b2Rot q = b2MakeRot( m_knob * B2_PI / 180.0f );

				// Lowest corner touches the floor
				float restHeight = h * ( b2AbsFloat( q.s ) + q.c );
				PlayerDef def = MakePlayerDef( { 0.0f, restHeight } );
				CreatePlayerPolygon( def, b2MakeOffsetBox( h, h, b2Vec2_zero, q ) );
				SetFlatSurface( { 0.0f, 0.0f }, { 46.0f, 0.0f }, restHeight, 0.5f, 40.0f );
			}
			break;

			case e_fastLanding:
			{
				AddFloor( -2.0f, 400, 0.5f, 0.0f, 0.0f );
				PlayerDef def = MakePlayerDef( { 0.0f, 3.0f } );
				def.velocity = { m_speed, -15.0f };
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );

				// Landing takes about 0.17 seconds
				SetFlatSurface( { 0.0f, 0.0f }, { 198.0f, 0.0f }, 0.5f, 0.3f * m_speed, b2MinFloat( 3.0f * m_speed, 180.0f ) );
			}
			break;

			case e_marble:
			{
				m_friction = 0.01f;
				AddTiledBlock( -4.0f, -1.0f, 16.0f, 0.0f, 20, 1 );

				float radius = 0.2f;
				PlayerDef def = MakePlayerDef( { -3.5f, radius } );
				def.lockRotation = false;
				def.isBullet = true;
				def.friction = 0.01f;
				def.restitution = 0.1f;
				CreatePlayerCircle( def, radius );
				SetFlatSurface( { 0.0f, 0.0f }, { 16.0f, 0.0f }, radius, -3.0f, 14.0f );
			}
			break;

			case e_slopeTiles:
			case e_slopeTriangles:
			{
				float slope = 30.0f * B2_PI / 180.0f;
				b2Vec2 down = { cosf( slope ), -sinf( slope ) };
				b2Vec2 up = b2LeftPerp( down );

				b2Vec2 points[31];
				for ( int i = 0; i <= 30; ++i )
				{
					points[i] = b2MulSV( (float)i, down );
				}

				// Slanted slabs with vertical cuts share whole edges. Triangles touch at one corner, which
				// leaves the step face below each corner exposed to the slope.
				b2Vec2 thickness = { 0.0f, 0.5f };
				for ( int i = 0; i < 30; ++i )
				{
					if ( m_scene == e_slopeTiles )
					{
						AddQuad( points[i], points[i + 1], b2Sub( points[i + 1], thickness ), b2Sub( points[i], thickness ) );
					}
					else
					{
						b2Vec2 corner = { points[i].x, points[i + 1].y };
						b2Vec2 triangle[3] = { points[i], points[i + 1], corner };
						b2Hull hull = b2ComputeHull( triangle, 3 );
						m_polygons.push_back( b2MakePolygon( &hull, 0.0f ) );
					}
				}

				// Low friction lets a caught corner tip the box over
				float h = 0.5f;
				PlayerDef def = MakePlayerDef( b2MulAdd( down, h, up ) );
				def.velocity = b2MulSV( m_speed, down );
				def.lockRotation = false;
				def.friction = 0.1f;
				CreatePlayerPolygon( def, b2MakeOffsetBox( h, h, b2Vec2_zero, b2MakeRot( -slope ) ) );
				SetFlatSurface( points[0], points[30], h, 1.5f, 26.0f );
			}
			break;

			case e_tileWall:
			{
				AddFloor( -2.0f, 12, 2.0f, 0.0f, 0.0f );
				AddTiledBlock( 12.0f, 0.0f, 13.5f, 1.0f, 3, 4 );
				PlayerDef def = MakePlayerDef( { 0.0f, 0.5f } );
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );
				m_push = { 20.0f, 0.0f };
				SetFlatSurface( { 0.0f, 0.0f }, { 22.0f, 0.0f }, 0.5f, 0.5f, 14.0f );
			}
			break;

			case e_cliffPit:
			{
				AddFloor( -2.0f, 6, 2.0f, 0.0f, 0.0f );
				AddTiledBlock( 8.0f, -3.5f, 10.0f, -0.5f, 1, 6 );
				AddTiledBlock( 10.0f, -3.5f, 14.0f, -3.0f, 2, 1 );
				AddTiledBlock( 14.0f, -3.5f, 16.0f, -0.5f, 1, 6 );
				AddFloor( 14.0f, 8, 2.0f, 0.0f, 0.0f );
				PlayerDef def = MakePlayerDef( { 0.0f, 0.5f } );
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );
				m_measure = false;
			}
			break;

			case e_wallSlide:
			{
				AddTiledBlock( 0.0f, 0.0f, 1.0f, 10.0f, 1, 20 );
				AddTiledBlock( -1.0f, -0.5f, 9.0f, 0.0f, 5, 1 );
				PlayerDef def = MakePlayerDef( { 1.5f, 9.0f } );
				def.velocity = { 0.0f, -m_speed };
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );
				m_push = { -20.0f, 0.0f };

				// Progress runs down the wall face
				SetFlatSurface( { 1.0f, 9.0f }, { 1.0f, 0.0f }, 0.5f, 0.3f, 7.5f );
			}
			break;

			case e_vehicleCrest:
			{
				float slope = 20.0f * B2_PI / 180.0f;
				b2Vec2 crest = { 10.0f, 0.0f };
				b2Vec2 foot = { 10.0f + 2.0f / tanf( slope ), -2.0f };

				AddFloor( -4.0f, 7, 2.0f, 0.0f, 0.0f );
				for ( int i = 0; i < 3; ++i )
				{
					b2Vec2 a = b2Lerp( crest, foot, i / 3.0f );
					b2Vec2 b = b2Lerp( crest, foot, ( i + 1 ) / 3.0f );
					AddQuad( a, b, { b.x, -2.5f }, { a.x, -2.5f } );
				}
				AddFloor( foot.x, 16, 2.0f, -2.0f, 0.0f );

				float radius = 0.4f;
				PlayerDef def = MakePlayerDef( { 0.0f, radius } );
				def.lockRotation = false;
				b2ShapeDef shapeDef = CreatePlayerBody( def );
				b2Circle rear = { { -1.5f, 0.0f }, radius };
				b2Circle front = { { 1.5f, 0.0f }, radius };
				b2CreateCircleShape( m_playerId, &shapeDef, &rear );
				b2CreateCircleShape( m_playerId, &shapeDef, &front );

				shapeDef.density = 0.2f;
				b2Polygon chassis = b2MakeOffsetBox( 2.0f, 0.3f, { 0.0f, 0.9f }, b2Rot_identity );
				b2CreatePolygonShape( m_playerId, &shapeDef, &chassis );
				m_measure = false;
			}
			break;

			case e_smallGaps:
			{
				float x = -2.0f;
				for ( int i = 0; i < 20; ++i )
				{
					AddBox( x, -0.5f, x + 2.0f, 0.0f );
					x += 2.0f + m_knob;
				}

				PlayerDef def = MakePlayerDef( { 0.0f, 0.5f } );
				CreatePlayerPolygon( def, b2MakeBox( 0.5f, 0.5f ) );
				SetFlatSurface( { 0.0f, 0.0f }, { 40.0f, 0.0f }, 0.5f, 0.5f, 30.0f );
			}
			break;

			case e_parabolaCrates:
				BuildParabolaCrates();
				break;

			default:
				break;
		}
	}

	// Slices of the band above y = x^2 / 8 with uneven widths and slanted cuts, so neighbors share
	// a sloped face
	void BuildParabolaCrates()
	{
		constexpr int crateCount = 14;
		constexpr float widths[crateCount] = { 1.0f, 1.3f, 0.8f, 1.15f, 0.9f, 1.25f, 0.85f,
											   1.1f, 1.35f, 0.95f, 1.2f, 0.8f, 1.05f, 1.3f };
		constexpr float skews[crateCount + 1] = { 0.06f,  -0.08f, 0.1f,	 -0.05f, 0.09f, -0.1f, 0.04f, 0.12f,
												  -0.07f, 0.08f,  -0.09f, 0.05f, -0.06f, 0.1f,  -0.04f };
		constexpr float thickness = 0.4f;
		constexpr float x0 = -3.5f;
		constexpr float span = 7.0f;

		auto floorHeight = []( float x ) { return 0.125f * x * x; };

		float widthSum = 0.0f;
		for ( float width : widths )
		{
			widthSum += width;
		}

		b2Vec2 bottom[crateCount + 1];
		b2Vec2 top[crateCount + 1];
		float xb = x0;
		for ( int i = 0; i <= crateCount; ++i )
		{
			float xt = xb + skews[i];
			bottom[i] = { xb, floorHeight( xb ) };
			top[i] = { xt, floorHeight( xt ) + thickness };
			if ( i < crateCount )
			{
				xb += widths[i] * span / widthSum;
			}
		}

		m_friction = 1.0f;
		for ( int i = 0; i < crateCount; ++i )
		{
			AddQuad( bottom[i], bottom[i + 1], top[i + 1], top[i] );
		}

		// Rim walls keep the ball in the bowl
		m_polygons.push_back( b2MakeOffsetBox( 0.1f, 1.0f, { x0 - 0.2f, floorHeight( x0 ) + 1.0f }, b2Rot_identity ) );
		m_polygons.push_back( b2MakeOffsetBox( 0.1f, 1.0f, { -x0 + 0.2f, floorHeight( x0 ) + 1.0f }, b2Rot_identity ) );

		float radius = 0.25f;
		b2Vec2 tangent = b2Normalize( b2Sub( top[1], top[0] ) );
		PlayerDef def = MakePlayerDef( b2MulAdd( b2Lerp( top[0], top[1], 0.5f ), radius, b2LeftPerp( tangent ) ) );
		def.velocity = b2MulSV( m_speed, tangent );
		def.angularVelocity = -m_speed / radius;
		def.lockRotation = false;
		def.friction = 0.6f;
		CreatePlayerCircle( def, radius );

		m_surface.assign( top, top + crateCount + 1 );
		float arcLength = 0.0f;
		for ( int i = 1; i < crateCount; ++i )
		{
			arcLength += b2Distance( top[i - 1], top[i] );
		}
		m_restHeight = radius;
		m_measureStart = 0.5f;
		m_pathLength = arcLength;
	}

	void CreateTerrain()
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		if ( m_terrain == e_polygons )
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = m_friction;
			for ( const b2Polygon& polygon : m_polygons )
			{
				b2CreatePolygonShape( m_groundId, &shapeDef, &polygon );
			}
			return;
		}

		b2ChainResult result = b2ComputeChains( m_polygons.data(), (int)m_polygons.size(), m_weldTolerance );

		b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
		material.friction = m_friction;

		for ( int i = 0; i < result.chainCount; ++i )
		{
			const b2ChainData& chain = result.chains[i];
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = chain.points;
			chainDef.pointCount = chain.count;
			chainDef.isLoop = true;
			chainDef.materials = &material;
			chainDef.materialCount = 1;
			b2CreateChain( m_groundId, &chainDef );

			m_pointCount += chain.count;
		}

		m_chainCount = result.chainCount;
		m_chainStatus = result.status;
		b2FreeChains( &result );
	}

	void Rebuild()
	{
		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
			m_groundId = b2_nullBodyId;
		}

		if ( B2_IS_NON_NULL( m_playerId ) )
		{
			b2DestroyBody( m_playerId );
			m_playerId = b2_nullBodyId;
		}

		m_polygons.clear();
		m_surface.clear();
		m_chainCount = 0;
		m_pointCount = 0;
		m_chainStatus = b2_chainStatusSuccess;
		m_friction = 0.0f;
		m_push = b2Vec2_zero;
		m_measure = true;
		m_restHeight = 0.0f;
		m_measureStart = 0.0f;
		m_pathLength = 0.0f;

		m_progress = 0.0f;
		m_maxHop = 0.0f;
		m_maxSink = 0.0f;
		m_maxKick = 0.0f;
		m_snagCount = 0;
		m_lastTangentSpeed = 0.0f;

		BuildScene();
		CreateTerrain();
		FollowPlayer();
	}

	void FollowPlayer()
	{
		if ( m_follow )
		{
			b2Pos p = b2Body_GetPosition( m_playerId );
			m_context->camera.center = { p.x, p.y + 1.0f };
		}
	}

	void UpdateMetrics()
	{
		b2Vec2 p = b2ToVec2( b2Body_GetPosition( m_playerId ) );
		b2Vec2 v = b2Body_GetLinearVelocity( m_playerId );

		int bestIndex = 0;
		float bestDistance = FLT_MAX;
		float bestProgress = 0.0f;
		float arcLength = 0.0f;
		int segmentCount = (int)m_surface.size() - 1;
		for ( int i = 0; i < segmentCount; ++i )
		{
			b2Vec2 a = m_surface[i];
			b2Vec2 d = b2Sub( m_surface[i + 1], a );
			float length = b2Length( d );
			float s = b2Dot( b2Sub( p, a ), d ) / ( length * length );
			float distance = b2Distance( p, b2MulAdd( a, b2ClampFloat( s, 0.0f, 1.0f ), d ) );
			if ( distance < bestDistance )
			{
				bestIndex = i;
				bestDistance = distance;
				bestProgress = arcLength + s * length;
			}
			arcLength += length;
		}

		b2Vec2 a = m_surface[bestIndex];
		b2Vec2 tangent = b2Normalize( b2Sub( m_surface[bestIndex + 1], a ) );
		b2Vec2 normal = b2LeftPerp( tangent );
		float height = b2Dot( b2Sub( p, a ), normal );
		float tangentSpeed = b2Dot( v, tangent );
		m_progress = bestProgress;

		if ( m_measureStart <= m_progress && m_progress <= m_pathLength )
		{
			m_maxHop = b2MaxFloat( m_maxHop, height - m_restHeight );
			m_maxSink = b2MaxFloat( m_maxSink, m_restHeight - height );
			m_maxKick = b2MaxFloat( m_maxKick, b2Dot( v, normal ) );
			// Slow climbs lose 10% per step without any ghost
			if ( m_lastTangentSpeed > 1.0f && tangentSpeed < 0.9f * m_lastTangentSpeed )
			{
				m_snagCount += 1;
			}
		}

		m_lastTangentSpeed = tangentSpeed;
	}

	bool DrawControls() override
	{
		const SceneInfo& info = m_scenes[m_scene];
		bool changed = false;

		const char* sceneNames[e_sceneCount];
		for ( int i = 0; i < e_sceneCount; ++i )
		{
			sceneNames[i] = m_scenes[i].name;
		}

		ImGui::PushItemWidth( 10.0f * ImGui::GetFontSize() );
		if ( ImGui::Combo( "Scene", &m_scene, sceneNames, e_sceneCount ) )
		{
			SetSceneDefaults();
			changed = true;
		}

		const char* terrainNames[] = { "Polygons", "Chains" };
		changed |= ImGui::Combo( "Terrain", &m_terrain, terrainNames, IM_ARRAYSIZE( terrainNames ) );
		if ( m_terrain == e_chains )
		{
			changed |= ImGui::SliderFloat( "Weld", &m_weldTolerance, B2_LINEAR_SLOP, 0.25f, "%.4f",
										   ImGuiSliderFlags_Logarithmic );
		}
		changed |= ImGui::SliderFloat( "Speed", &m_speed, 0.0f, 100.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Gravity", &m_gravityScale, 1.0f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic );
		if ( info.knobName != nullptr )
		{
			changed |= ImGui::SliderFloat( info.knobName, &m_knob, 0.0f, info.knobMax, "%.4f" );
		}
		ImGui::PopItemWidth();

		ImGui::Checkbox( "Follow", &m_follow );

		if ( ImGui::Button( "Relaunch" ) || changed )
		{
			Rebuild();
		}

		ImGui::SameLine();
		if ( ImGui::Button( "Defaults" ) )
		{
			SetSceneDefaults();
			Rebuild();
		}

		return true;
	}

	void Step() override
	{
		if ( m_push.x != 0.0f || m_push.y != 0.0f )
		{
			float mass = b2Body_GetMass( m_playerId );
			b2Body_ApplyForceToCenter( m_playerId, b2MulSV( mass, m_push ), true );
		}

		Sample::Step();

		if ( m_didStep && m_measure )
		{
			UpdateMetrics();
		}

		FollowPlayer();

		DrawScreenTextLine( "%s", m_scenes[m_scene].summary );
		if ( m_terrain == e_chains )
		{
			DrawScreenTextLine( "%d polygons became %d chain loops with %d points", (int)m_polygons.size(), m_chainCount,
								m_pointCount );

			if ( m_chainStatus == b2_chainStatusCornerTouch )
			{
				DrawScreenTextLine( "status: polygons touch at a corner" );
			}
			else if ( m_chainStatus == b2_chainStatusOverlap )
			{
				DrawScreenTextLine( "status: polygons overlap" );
			}
		}
		else
		{
			DrawScreenTextLine( "%d polygons", (int)m_polygons.size() );
		}

		if ( m_measure )
		{
			DrawScreenTextLine( "progress %.2f of %.2f", m_progress, m_pathLength );
			DrawScreenTextLine( "max hop %.4f, max sink %.4f, max kick %.3f, snags %d", m_maxHop, m_maxSink, m_maxKick,
								m_snagCount );
		}

		b2Pos p = b2Body_GetPosition( m_playerId );
		DrawScreenTextLine( "position (%.2f, %.2f)", (float)p.x, (float)p.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new PolygonChains( context );
	}

	b2BodyId m_groundId;
	b2BodyId m_playerId;
	std::vector<b2Polygon> m_polygons;
	std::vector<b2Vec2> m_surface;
	int m_chainCount;
	int m_pointCount;
	b2ChainStatus m_chainStatus;

	int m_scene;
	int m_terrain;
	float m_speed;
	float m_gravityScale;
	float m_knob;
	bool m_follow;
	float m_weldTolerance;

	float m_friction;
	b2Vec2 m_push;
	bool m_measure;
	float m_restHeight;
	float m_measureStart;
	float m_pathLength;

	float m_progress;
	float m_maxHop;
	float m_maxSink;
	float m_maxKick;
	int m_snagCount;
	float m_lastTangentSpeed;
};

static int samplePolygonChains = RegisterSample( "Continuous", "Polygon Chains", PolygonChains::Create );
