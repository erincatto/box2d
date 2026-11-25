#include "sample.h"

#include "box2d/box2d.h"

#include <imgui.h>

// This simulations stresses the solver by putting a light mass between two bodies on a prismatic joint with a stiff spring.
// This can be made stable by increasing the size of the middle circle and/or increasing the number of sub-steps.
// todo try applying the impulse at the mid point to avoid breaking Newton's 3rd law
class UnstablePrismaticJoints : public Sample
{
public:
	explicit UnstablePrismaticJoints( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 32.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -100.0f, 0.0f }, { 100.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyId centerId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { 0, 3 };
			centerId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };

			// Note: this will crash due to divergence (inf/nan) with a radius of 0.1
			// circle.radius = 0.1f;
			circle.radius = 0.5f;

			b2CreateCircleShape( centerId, &sd, &circle );
		}

		b2PrismaticJointDef jd = b2DefaultPrismaticJointDef();
		jd.enableSpring = true;
		jd.hertz = 10.0f;
		jd.dampingRatio = 2.0f;

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { -3.5, 3 };

			b2BodyId leftId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };
			circle.radius = 2.0f;
			b2CreateCircleShape( leftId, &sd, &circle );

			jd.base.bodyIdA = centerId;
			jd.base.bodyIdB = leftId;
			jd.targetTranslation = -3.0f;
			b2CreatePrismaticJoint( m_worldId, &jd );
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { 3.5, 3 };
			b2BodyId rightId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };
			circle.radius = 2.0f;

			b2CreateCircleShape( rightId, &sd, &circle );

			jd.base.bodyIdA = centerId;
			jd.base.bodyIdB = rightId;
			jd.targetTranslation = 3.0f;
			b2CreatePrismaticJoint( m_worldId, &jd );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new UnstablePrismaticJoints( context );
	}
};

static int sampleUnstablePrismaticJoints =
	RegisterSample( "Issues", "Unstable Prismatic Joints", UnstablePrismaticJoints::Create );

// This configuration has stability problems.
class UnstableWindmill : public Sample
{
public:
	explicit UnstableWindmill( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 32.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -100.0f, -10.0f }, { 100.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bdef = b2DefaultBodyDef();
		bdef.gravityScale = 0.0f;
		bdef.type = b2_dynamicBody;
		b2ShapeDef sdef = b2DefaultShapeDef();
		sdef.material = b2DefaultSurfaceMaterial();
		sdef.material.friction = 0.1f;

		// center
		bdef.position = { 10, 10 };
		b2BodyId center = b2CreateBody( m_worldId, &bdef );
		b2Circle circle = { .center = { 0, 0 }, .radius = 5 };
		b2CreateCircleShape( center, &sdef, &circle );

		// rotors
		b2WeldJointDef wjdef = b2DefaultWeldJointDef();

		// This simulation can be stabilized by using a lower constraint stiffness
		//wjdef.base.constraintHertz = 30.0f;
		wjdef.base.bodyIdA = center;

		b2Polygon polygon;

		bdef.position = { 10, 0 };
		b2BodyId body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 4, 5 ) ) );
		wjdef.base.localFrameA = { .p = { 0, -5 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 0, 5 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 20, 10 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 5, 4 ) ) );
		wjdef.base.localFrameA = { .p = { 5, 0 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { -5, 0 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 10, 20 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 4, 5 ) ) );
		wjdef.base.localFrameA = { .p = { 0, 5 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 0, -5 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 0, 10 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 5, 4 ) ) );
		wjdef.base.localFrameA = { .p = { -5, 0 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 5, 0 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );
	}

	static Sample* Create( SampleContext* context )
	{
		return new UnstableWindmill( context );
	}
};

static int sampleUnstableWindmill = RegisterSample( "Issues", "Unstable Windmill", UnstableWindmill::Create );

// From Box3D issue #219: a small bullet circle fired into a thin static plate at high speed must stay on
// the approach side. A soft contact only removes part of the approach speed in the first sub-step, which
// may carry the circle through the plate. The left plate is a polygon, the right plate a chain loop. A
// circle drawn red has crossed its plate.
class BulletThinPlate : public Sample
{
public:
	static constexpr float m_plateY = 1.0f;
	static constexpr float m_plateHalfThickness = 0.0125f;
	static constexpr float m_plateHalfLength = 5.0f;
	static constexpr float m_dropHeight = 2.0f;
	static constexpr int m_fireInterval = 60;

	explicit BulletThinPlate( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, m_plateY };
			m_context->camera.zoom = 8.0f;
		}

		m_plateX[0] = -6.0f;
		m_plateX[1] = 6.0f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.name = "polygon plate";
		bodyDef.position = { m_plateX[0], m_plateY };
		b2BodyId polygonPlateId = b2CreateBody( m_worldId, &bodyDef );

		// Top face at the body origin
		b2Polygon box = b2MakeOffsetBox( m_plateHalfLength, m_plateHalfThickness, { 0.0f, -m_plateHalfThickness }, b2Rot_identity );
		b2CreatePolygonShape( polygonPlateId, &shapeDef, &box );

		bodyDef.name = "chain plate";
		bodyDef.position = { m_plateX[1], m_plateY };
		b2BodyId chainPlateId = b2CreateBody( m_worldId, &bodyDef );

		float h = m_plateHalfLength;
		float t = 2.0f * m_plateHalfThickness;
		b2Vec2 points[4] = { { -h, -t }, { h, -t }, { h, 0.0f }, { -h, 0.0f } };

		b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
		material.friction = 0.0f;

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.pointCount = 4;
		chainDef.materials = &material;
		chainDef.materialCount = 1;
		chainDef.isLoop = true;
		b2CreateChain( chainPlateId, &chainDef );

		for ( int i = 0; i < 2; ++i )
		{
			m_bulletIds[i] = b2_nullBodyId;
			m_shotCount[i] = 0;
			m_crossCount[i] = 0;
			m_crossed[i] = false;
		}

		Fire();
	}

	void Fire()
	{
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.enableSleep = false;
		bodyDef.linearVelocity = { m_lateralSpeed, -m_speed };

		b2Circle circle = { b2Vec2_zero, m_radius };

		for ( int i = 0; i < 2; ++i )
		{
			if ( B2_IS_NON_NULL( m_bulletIds[i] ) )
			{
				b2DestroyBody( m_bulletIds[i] );
			}

			bodyDef.position = { m_plateX[i] - m_plateHalfLength + 1.0f, m_plateY + m_dropHeight };
			m_bulletIds[i] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_bulletIds[i], &shapeDef, &circle );

			m_shotCount[i] += 1;
			m_crossed[i] = false;
		}

		m_firedRadius = m_radius;
		m_stepsSinceFire = 0;
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		ImGui::SliderFloat( "Radius", &m_radius, 0.005f, 0.1f, "%.3f m" );
		ImGui::SliderFloat( "Speed", &m_speed, 10.0f, 1000.0f, "%.0f m/s" );
		ImGui::SliderFloat( "Lateral", &m_lateralSpeed, 0.0f, 100.0f, "%.0f m/s" );
		ImGui::Checkbox( "Auto Fire", &m_autoFire );

		if ( ImGui::Button( "Fire" ) )
		{
			Fire();
		}

		ImGui::SameLine();
		if ( ImGui::Button( "Clear Counts" ) )
		{
			for ( int i = 0; i < 2; ++i )
			{
				m_shotCount[i] = 0;
				m_crossCount[i] = 0;
			}
		}

		ImGui::PopItemWidth();
		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_didStep )
		{
			float plateBottom = m_plateY - 2.0f * m_plateHalfThickness;
			for ( int i = 0; i < 2; ++i )
			{
				b2Pos p = b2Body_GetPosition( m_bulletIds[i] );

				// A circle that slid off the far end falls past the plate legitimately
				float x = (float)p.x - m_plateX[i];
				float y = (float)p.y;
				if ( m_crossed[i] == false && x < m_plateHalfLength && y + m_firedRadius < plateBottom )
				{
					m_crossed[i] = true;
					m_crossCount[i] += 1;
				}
			}

			m_stepsSinceFire += 1;
			if ( m_autoFire && m_stepsSinceFire >= m_fireInterval )
			{
				Fire();
			}
		}

		const char* names[2] = { "polygon", "chain" };
		for ( int i = 0; i < 2; ++i )
		{
			b2Pos p = b2Body_GetPosition( m_bulletIds[i] );
			DrawPoint( m_draw, p, 10.0f, m_crossed[i] ? b2_colorRed : b2_colorLime );

			DrawScreenTextLine( "%s plate: %s, crossed %d of %d shots", names[i], m_crossed[i] ? "CROSSED" : "ok",
								m_crossCount[i], m_shotCount[i] );
		}

		float substepTime = 1.0f / ( m_context->hertz * m_context->subStepCount );
		DrawScreenTextLine( "plate thickness %.3f m, travel per sub-step %.3f m", 2.0f * m_plateHalfThickness,
							m_speed * substepTime );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BulletThinPlate( context );
	}

	b2BodyId m_bulletIds[2];
	float m_plateX[2];
	int m_shotCount[2];
	int m_crossCount[2];
	bool m_crossed[2];
	int m_stepsSinceFire = 0;
	float m_firedRadius = 0.01f;
	float m_radius = 0.01f;
	float m_speed = 300.0f;
	float m_lateralSpeed = 30.0f;
	bool m_autoFire = true;
};

static int sampleBulletThinPlate = RegisterSample( "Issues", "Bullet Thin Plate", BulletThinPlate::Create );
