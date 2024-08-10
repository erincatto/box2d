// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Pyramid with heavy box on top
class HighMassRatio1 : public Sample
{
public:
	explicit HighMassRatio1( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 3.0f, 14.0f };
			g_camera.m_zoom = 25.0f;
		}

		float extent = 1.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 50.0f, 1.0f, { 0.0f, -1.0f }, 0.0f );
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

	static Sample* Create( Settings& settings )
	{
		return new HighMassRatio1( settings );
	}
};

static int sampleIndex1 = RegisterSample( "Robustness", "HighMassRatio1", HighMassRatio1::Create );

// Big box on small boxes
class HighMassRatio2 : public Sample
{
public:
	explicit HighMassRatio2( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 16.5f };
			g_camera.m_zoom = 25.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 50.0f, 1.0f, { 0.0f, -1.0f }, 0.0f );
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

	static Sample* Create( Settings& settings )
	{
		return new HighMassRatio2( settings );
	}
};

static int sampleIndex2 = RegisterSample( "Robustness", "HighMassRatio2", HighMassRatio2::Create );

class OverlapRecovery : public Sample
{
public:
	explicit OverlapRecovery( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 2.5f };
			g_camera.m_zoom = 25.0f * 0.15f;
		}

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_baseCount = 4;
		m_overlap = 0.25f;
		m_extent = 0.5f;
		m_pushout = 3.0f;
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

		b2World_SetContactTuning( m_worldId, m_hertz, m_dampingRatio, m_pushout );

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

	void UpdateUI() override
	{
		float height = 210.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 220.0f, height ) );

		ImGui::Begin( "Overlap Recovery", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		bool changed = false;
		changed = changed || ImGui::SliderFloat( "Extent", &m_extent, 0.1f, 1.0f, "%.1f" );
		changed = changed || ImGui::SliderInt( "Base Count", &m_baseCount, 1, 10 );
		changed = changed || ImGui::SliderFloat( "Overlap", &m_overlap, 0.0f, 1.0f, "%.2f" );
		changed = changed || ImGui::SliderFloat( "Pushout", &m_pushout, 0.0f, 10.0f, "%.1f" );
		changed = changed || ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 120.0f, "%.f" );
		changed = changed || ImGui::SliderFloat( "Damping Ratio", &m_dampingRatio, 0.0f, 20.0f, "%.1f" );
		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new OverlapRecovery( settings );
	}

	b2BodyId* m_bodyIds;
	int32_t m_bodyCount;
	int32_t m_baseCount;
	float m_overlap;
	float m_extent;
	float m_pushout;
	float m_hertz;
	float m_dampingRatio;
};

static int sampleIndex4 = RegisterSample( "Robustness", "Overlap Recovery", OverlapRecovery::Create );
