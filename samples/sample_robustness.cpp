// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <stdlib.h>

// Pyramid with heavy box on top
class HighMassRatio1 : public Sample
{
public:
	explicit HighMassRatio1( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 3.0f, 14.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		float extent = 1.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 50.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			b2Polygon box = b2MakeBox( extent, extent );
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			for ( int j = 0; j < 3; ++j )
			{
				int count = 10;
				float offset = -20.0f * extent + 2.0f * ( count + 1.0f ) * extent * j;
				float y = extent;
				while ( count > 0 )
				{
					for ( int i = 0; i < count; ++i )
					{
						float coeff = i - 0.5f * count;

						float yy = count == 1 ? y + 2.0f : y;
						bodyDef.position = { 2.0f * coeff * extent + offset, yy };
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

						shapeDef.density = count == 1 ? ( j + 1.0f ) * 100.0f : 1.0f;
						b2CreatePolygonShape( bodyId, &shapeDef, &box );
					}

					--count;
					y += 2.0f * extent;
				}
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new HighMassRatio1( context );
	}
};

static int sampleIndex1 = RegisterSample( "Robustness", "HighMassRatio1", HighMassRatio1::Create );

// Big box on small boxes
class HighMassRatio2 : public Sample
{
public:
	explicit HighMassRatio2( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 16.5f };
			m_context->camera.m_zoom = 25.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 50.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float extent = 1.0f;
			b2Polygon smallBox = b2MakeBox( 0.5f * extent, 0.5f * extent );
			b2Polygon bigBox = b2MakeBox( 10.0f * extent, 10.0f * extent );

			{
				bodyDef.position = { -9.0f * extent, 0.5f * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &smallBox );
			}

			{
				bodyDef.position = { 9.0f * extent, 0.5f * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &smallBox );
			}

			{
				bodyDef.position = { 0.0f, ( 10.0f + 16.0f ) * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &bigBox );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new HighMassRatio2( context );
	}
};

static int sampleIndex2 = RegisterSample( "Robustness", "HighMassRatio2", HighMassRatio2::Create );

// Big box on small triangles
class HighMassRatio3 : public Sample
{
public:
	explicit HighMassRatio3( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 16.5f };
			m_context->camera.m_zoom = 25.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 50.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float extent = 1.0f;
			b2Vec2 points[3] = { { -0.5f * extent, 0.0f }, { 0.5f * extent, 0.0f }, { 0.0f, 1.0f * extent } };
			b2Hull hull = b2ComputeHull( points, 3 );
			b2Polygon smallTriangle = b2MakePolygon( &hull, 0.0f );
			b2Polygon bigBox = b2MakeBox( 10.0f * extent, 10.0f * extent );

			{
				bodyDef.position = { -9.0f * extent, 0.5f * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &smallTriangle );
			}

			{
				bodyDef.position = { 9.0f * extent, 0.5f * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &smallTriangle );
			}

			{
				bodyDef.position = { 0.0f, ( 10.0f + 4.0f ) * extent };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &bigBox );
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new HighMassRatio3( context );
	}
};

static int sampleIndex3 = RegisterSample( "Robustness", "HighMassRatio3", HighMassRatio3::Create );

class OverlapRecovery : public Sample
{
public:
	explicit OverlapRecovery( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 2.5f };
			m_context->camera.m_zoom = 3.75f;
		}

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_baseCount = 4;
		m_overlap = 0.25f;
		m_extent = 0.5f;
		m_pushOut = 3.0f;
		m_hertz = 30.0f;
		m_dampingRatio = 10.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		float groundWidth = 40.0f;
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		b2Segment segment = { { -groundWidth, 0.0f }, { groundWidth, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		CreateScene();
	}

	~OverlapRecovery() override
	{
		free( m_bodyIds );
	}

	void CreateScene()
	{
		for ( int32_t i = 0; i < m_bodyCount; ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
		}

		b2World_SetContactTuning( m_worldId, m_hertz, m_dampingRatio, m_pushOut );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2Polygon box = b2MakeBox( m_extent, m_extent );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		m_bodyCount = m_baseCount * ( m_baseCount + 1 ) / 2;
		m_bodyIds = (b2BodyId*)realloc( m_bodyIds, m_bodyCount * sizeof( b2BodyId ) );

		int32_t bodyIndex = 0;
		float fraction = 1.0f - m_overlap;
		float y = m_extent;
		for ( int32_t i = 0; i < m_baseCount; ++i )
		{
			float x = fraction * m_extent * ( i - m_baseCount );
			for ( int32_t j = i; j < m_baseCount; ++j )
			{
				bodyDef.position = { x, y };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				b2CreatePolygonShape( bodyId, &shapeDef, &box );

				m_bodyIds[bodyIndex++] = bodyId;

				x += 2.0f * fraction * m_extent;
			}

			y += 2.0f * fraction * m_extent;
		}

		assert( bodyIndex == m_bodyCount );
	}

	void UpdateGui() override
	{
		float height = 210.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 220.0f, height ) );

		ImGui::Begin( "Overlap Recovery", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Extent", &m_extent, 0.1f, 1.0f, "%.1f" );
		changed = changed || ImGui::SliderInt( "Base Count", &m_baseCount, 1, 10 );
		changed = changed || ImGui::SliderFloat( "Overlap", &m_overlap, 0.0f, 1.0f, "%.2f" );
		changed = changed || ImGui::SliderFloat( "Speed", &m_pushOut, 0.0f, 10.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio", &m_dampingRatio, 0.0f, 20.0f, "%.1f" );
		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new OverlapRecovery( context );
	}

	b2BodyId* m_bodyIds;
	int32_t m_bodyCount;
	int32_t m_baseCount;
	float m_overlap;
	float m_extent;
	float m_pushOut;
	float m_hertz;
	float m_dampingRatio;
};

static int sampleIndex4 = RegisterSample( "Robustness", "Overlap Recovery", OverlapRecovery::Create );

class TinyPyramid : public Sample
{
public:
	explicit TinyPyramid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.8f };
			m_context->camera.m_zoom = 1.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 5.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			m_extent = 0.025f;
			int baseCount = 30;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Polygon box = b2MakeSquare( m_extent );

			for ( int i = 0; i < baseCount; ++i )
			{
				float y = ( 2.0f * i + 1.0f ) * m_extent;

				for ( int j = i; j < baseCount; ++j )
				{
					float x = ( i + 1.0f ) * m_extent + 2.0f * ( j - i ) * m_extent - baseCount * m_extent;
					bodyDef.position = { x, y };

					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
					b2CreatePolygonShape( bodyId, &shapeDef, &box );
				}
			}
		}
	}

	void Step() override
	{
		DrawTextLine( "%.1fcm squares", 200.0f * m_extent );
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new TinyPyramid( context );
	}

	float m_extent;
};

static int sampleTinyPyramid = RegisterSample( "Robustness", "Tiny Pyramid", TinyPyramid::Create );

// High gravity and high mass ratio
class Cart : public Sample
{
public:
	explicit Cart( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 1.0f };
			m_context->camera.m_zoom = 1.5f;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon groundBox = b2MakeBox( 20.0f, 1.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &groundBox );
		}

		b2World_SetGravity( m_worldId, { 0, -22 } );

		m_contactHertz = 30.0f;
		m_contactDampingRatio = 10.0f;
		m_contactSpeed = 3.0f;
		b2World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );

		m_jointHertz = 60.0f;
		m_jointDampingRatio = 1.0f;

		m_chassisId = {};
		m_wheelId1 = {};
		m_wheelId2 = {};

		CreateScene();
	}

	void CreateScene()
	{
		if (B2_IS_NON_NULL(m_chassisId))
		{
			b2DestroyBody( m_chassisId );
		}

		if (B2_IS_NON_NULL(m_wheelId1))
		{
			b2DestroyBody( m_wheelId1 );
		}

		if (B2_IS_NON_NULL(m_wheelId2))
		{
			b2DestroyBody( m_wheelId2 );
		}

		float yBase = 2.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, yBase };
		m_chassisId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 100.0f;

		b2Polygon box = b2MakeOffsetBox( 0.5f, 0.25f, { 0.0f, 0.25f }, b2Rot_identity );
		b2CreatePolygonShape( m_chassisId, &shapeDef, &box );

		shapeDef = b2DefaultShapeDef();
		shapeDef.material.rollingResistance = 0.02f;
		shapeDef.density = 10.0f;

		b2Circle circle = { b2Vec2_zero, 0.1f };
		bodyDef.position = { -0.4f, yBase - 0.15f };
		m_wheelId1 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCircleShape( m_wheelId1, &shapeDef, &circle );

		bodyDef.position = { 0.4f, yBase - 0.15f };
		m_wheelId2 = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCircleShape( m_wheelId2, &shapeDef, &circle );

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_chassisId;
		jointDef.bodyIdB = m_wheelId1;
		jointDef.localAnchorA = { -0.4f, -0.15f };
		jointDef.localAnchorB = { 0.0f, 0.0f };

		m_jointId1 = b2CreateRevoluteJoint( m_worldId, &jointDef );
		b2Joint_SetConstraintTuning( m_jointId1, m_jointHertz, m_jointDampingRatio );

		jointDef.bodyIdA = m_chassisId;
		jointDef.bodyIdB = m_wheelId2;
		jointDef.localAnchorA = { 0.4f, -0.15f };
		jointDef.localAnchorB = { 0.0f, 0.0f };

		m_jointId2 = b2CreateRevoluteJoint( m_worldId, &jointDef );
		b2Joint_SetConstraintTuning( m_jointId2, m_jointHertz, m_jointDampingRatio );
	}

	void UpdateGui() override
	{
		float height = 240.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 320.0f, height ) );

		ImGui::Begin( "Cart", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 200.0f );

		bool changed = false;
		ImGui::Text( "Contact" );
		changed = changed || ImGui::SliderFloat( "Hertz##contact", &m_contactHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio##contact", &m_contactDampingRatio, 0.0f, 1000.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Speed", &m_contactSpeed, 0.0f, 5.0f, "%.1f" );

		if ( changed )
		{
			b2World_SetContactTuning( m_worldId, m_contactHertz, m_contactDampingRatio, m_contactSpeed );
			CreateScene();
		}

		ImGui::Separator();

		changed = false;
		ImGui::Text( "Joint" );
		changed = changed || ImGui::SliderFloat( "Hertz##joint", &m_jointHertz, 0.0f, 240.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio##joint", &m_jointDampingRatio, 0.0f, 1000.0f, "%.f" );

		ImGui::Separator();

		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			b2Joint_SetConstraintTuning( m_jointId1, m_jointHertz, m_jointDampingRatio );
			b2Joint_SetConstraintTuning( m_jointId2, m_jointHertz, m_jointDampingRatio );
			CreateScene();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Cart( context );
	}

	b2BodyId m_chassisId;
	b2BodyId m_wheelId1;
	b2BodyId m_wheelId2;
	b2JointId m_jointId1;
	b2JointId m_jointId2;

	float m_contactHertz;
	float m_contactDampingRatio;
	float m_contactSpeed;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleCart = RegisterSample( "Robustness", "Cart", Cart::Create );
