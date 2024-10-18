// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <vector>

class SingleBox : public Sample
{
public:
	explicit SingleBox( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 2.5f };
			g_camera.m_zoom = 3.5f;
		}

		float extent = 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		float groundWidth = 66.0f * extent;
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.5f;

		b2Segment segment = { { -0.5f * 2.0f * groundWidth, 0.0f }, { 0.5f * 2.0f * groundWidth, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );
		bodyDef.type = b2_dynamicBody;

		b2Polygon box = b2MakeBox( extent, extent );
		bodyDef.position = { 0.0f, 4.0f };
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
		b2CreatePolygonShape( bodyId, &shapeDef, &box );
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		// g_draw.DrawCircle({0.0f, 2.0f}, 1.0f, b2_colorWhite);
	}

	static Sample* Create( Settings& settings )
	{
		return new SingleBox( settings );
	}
};

static int sampleSingleBox = RegisterSample( "Stacking", "Single Box", SingleBox::Create );

class TiltedStack : public Sample
{
public:
	enum
	{
		e_columns = 10,
		e_rows = 10,
	};

	explicit TiltedStack( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 7.5f, 7.5f };
			g_camera.m_zoom = 20.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 1000.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		for ( int i = 0; i < e_rows * e_columns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		b2Polygon box = b2MakeRoundedBox( 0.45f, 0.45f, 0.05f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.3f;

		float offset = 0.2f;
		float dx = 5.0f;
		float xroot = -0.5f * dx * ( e_columns - 1.0f );

		for ( int j = 0; j < e_columns; ++j )
		{
			float x = xroot + j * dx;

			for ( int i = 0; i < e_rows; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;

				int n = j * e_rows + i;

				bodyDef.position = { x + offset * i, 0.5f + 1.0f * i };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				m_bodies[n] = bodyId;

				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new TiltedStack( settings );
	}

	b2BodyId m_bodies[e_rows * e_columns];
};

static int sampleTiltedStack = RegisterSample( "Stacking", "Tilted Stack", TiltedStack::Create );

class VerticalStack : public Sample
{
public:
	enum
	{
		e_maxColumns = 10,
		e_maxRows = 15,
		e_maxBullets = 8
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	explicit VerticalStack( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { -7.0f, 9.0f };
			g_camera.m_zoom = 14.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 100.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			b2Segment segment = { { 10.0f, 1.0f }, { 10.0f, 21.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		for ( int i = 0; i < e_maxBullets; ++i )
		{
			m_bullets[i] = b2_nullBodyId;
		}

		m_shapeType = e_boxShape;
		m_rowCount = e_maxRows;
		m_columnCount = 5;
		m_bulletCount = 1;
		m_bulletType = e_circleShape;

		CreateStacks();
	}

	void CreateStacks()
	{
		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = { 0 };
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		// b2Polygon box = b2MakeRoundedBox(0.45f, 0.45f, 0.05f);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.3f;

		float offset;

		if ( m_shapeType == e_circleShape )
		{
			offset = 0.0f;
		}
		else
		{
			offset = 0.01f;
		}

		float dx = -3.0f;
		float xroot = 8.0f;

		for ( int j = 0; j < m_columnCount; ++j )
		{
			float x = xroot + j * dx;

			for ( int i = 0; i < m_rowCount; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;

				int n = j * m_rowCount + i;

				float shift = ( i % 2 == 0 ? -offset : offset );
				bodyDef.position = { x + shift, 0.5f + 1.0f * i };
				// bodyDef.position = {x + shift, 1.0f + 1.51f * i};
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				m_bodies[n] = bodyId;

				if ( m_shapeType == e_circleShape )
				{
					b2CreateCircleShape( bodyId, &shapeDef, &circle );
				}
				else
				{
					b2CreatePolygonShape( bodyId, &shapeDef, &box );
				}
			}
		}
	}

	void DestroyBody()
	{
		for ( int j = 0; j < m_columnCount; ++j )
		{
			for ( int i = 0; i < m_rowCount; ++i )
			{
				int n = j * m_rowCount + i;

				if ( B2_IS_NON_NULL( m_bodies[n] ) )
				{
					b2DestroyBody( m_bodies[n] );
					m_bodies[n] = b2_nullBodyId;
					break;
				}
			}
		}
	}

	void DestroyBullets()
	{
		for ( int i = 0; i < e_maxBullets; ++i )
		{
			b2BodyId bullet = m_bullets[i];

			if ( B2_IS_NON_NULL( bullet ) )
			{
				b2DestroyBody( bullet );
				m_bullets[i] = b2_nullBodyId;
			}
		}
	}

	void FireBullets()
	{
		b2Circle circle = { { 0.0f, 0.0f }, 0.25f };
		b2Polygon box = b2MakeBox( 0.25f, 0.25f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 4.0f;

		for ( int i = 0; i < m_bulletCount; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -25.0f - i, 6.0f };
			float speed = RandomFloat( 200.0f, 300.0f );
			bodyDef.linearVelocity = { speed, 0.0f };
			bodyDef.isBullet = true;

			b2BodyId bullet = b2CreateBody( m_worldId, &bodyDef );

			if ( m_bulletType == e_boxShape )
			{
				b2CreatePolygonShape( bullet, &shapeDef, &box );
			}
			else
			{
				b2CreateCircleShape( bullet, &shapeDef, &circle );
			}
			assert( B2_IS_NULL( m_bullets[i] ) );
			m_bullets[i] = bullet;
		}
	}

	void UpdateUI() override
	{
		float height = 230.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Vertical Stack", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 120.0f );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		changed = changed || ImGui::SliderInt( "Rows", &m_rowCount, 1, e_maxRows );
		changed = changed || ImGui::SliderInt( "Columns", &m_columnCount, 1, e_maxColumns );

		ImGui::SliderInt( "Bullets", &m_bulletCount, 1, e_maxBullets );

		int bulletType = int( m_bulletType );
		ImGui::Combo( "Bullet Shape", &bulletType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_bulletType = ShapeType( bulletType );

		ImGui::PopItemWidth();

		if ( ImGui::Button( "Fire Bullets" ) || glfwGetKey( g_mainWindow, GLFW_KEY_B ) == GLFW_PRESS )
		{
			DestroyBullets();
			FireBullets();
		}

		if ( ImGui::Button( "Destroy Body" ) )
		{
			DestroyBody();
		}

		changed = changed || ImGui::Button( "Reset Stack" );

		if ( changed )
		{
			DestroyBullets();
			CreateStacks();
		}

		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new VerticalStack( settings );
	}

	b2BodyId m_bullets[e_maxBullets];
	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	int m_columnCount;
	int m_rowCount;
	int m_bulletCount;
	ShapeType m_shapeType;
	ShapeType m_bulletType;
};

static int sampleVerticalStack = RegisterSample( "Stacking", "Vertical Stack", VerticalStack::Create );

// A simple circle stack that also shows how to collect hit events
class CircleStack : public Sample
{
public:

	struct Event
	{
		int indexA, indexB;
	};

	explicit CircleStack( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 5.0f };
			g_camera.m_zoom = 6.0f;
		}

		int shapeIndex = 0;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.userData = reinterpret_cast<void*>( intptr_t( shapeIndex ) );
			shapeIndex += 1;

			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2World_SetGravity( m_worldId, { 0.0f, -20.0f } );
		b2World_SetContactTuning( m_worldId, 0.25f * 360.0f, 10.0f, 3.0f );

		b2Circle circle = {};
		circle.radius = 0.25f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableHitEvents = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		float y = 0.5f;

		for ( int i = 0; i < 8; ++i )
		{
			bodyDef.position.y = y;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			shapeDef.userData = reinterpret_cast<void*>( intptr_t( shapeIndex ) );
			shapeIndex += 1;
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			y += 2.0f;
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < events.hitCount; ++i )
		{
			b2ContactHitEvent* event = events.hitEvents + i;

			void* userDataA = b2Shape_GetUserData( event->shapeIdA );
			void* userDataB = b2Shape_GetUserData( event->shapeIdB );
			int indexA = static_cast<int>( reinterpret_cast<intptr_t>( userDataA ) );
			int indexB = static_cast<int>( reinterpret_cast<intptr_t>( userDataB ) );

			g_draw.DrawPoint( event->point, 10.0f, b2_colorWhite );

			m_events.push_back( { indexA, indexB } );
		}

		int eventCount = m_events.size();
		for (int i = 0; i < eventCount; ++i)
		{
			g_draw.DrawString( 5, m_textLine, "%d, %d", m_events[i].indexA, m_events[i].indexB );
			m_textLine += m_textIncrement;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new CircleStack( settings );
	}

	std::vector<Event> m_events;
};

static int sampleCircleStack = RegisterSample( "Stacking", "Circle Stack", CircleStack::Create );

class Cliff : public Sample
{
public:
	explicit Cliff( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_zoom = 25.0f * 0.5f;
			g_camera.m_center = { 0.0f, 5.0f };
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 100.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity  );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			b2Segment segment = { { -14.0f, 4.0f }, { -8.0f, 4.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			box = b2MakeOffsetBox( 3.0f, 0.5f, { 0.0f, 4.0f }, b2Rot_identity  );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			b2Capsule capsule = { { 8.5f, 4.0f }, { 13.5f, 4.0f }, 0.5f };
			b2CreateCapsuleShape( groundId, &shapeDef, &capsule );
		}

		m_flip = false;

		for ( int i = 0; i < 9; ++i )
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		CreateBodies();
	}

	void CreateBodies()
	{
		for ( int i = 0; i < 9; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		float sign = m_flip ? -1.0f : 1.0f;

		b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2Polygon square = b2MakeSquare( 0.5f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.01f;
			bodyDef.linearVelocity = { 2.0f * sign, 0.0f };

			float offset = m_flip ? -4.0f : 0.0f;

			bodyDef.position = { -9.0f + offset, 4.25f };
			m_bodyIds[0] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( m_bodyIds[0], &shapeDef, &capsule );

			bodyDef.position = { 2.0f + offset, 4.75f };
			m_bodyIds[1] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( m_bodyIds[1], &shapeDef, &capsule );

			bodyDef.position = { 13.0f + offset, 4.75f };
			m_bodyIds[2] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( m_bodyIds[2], &shapeDef, &capsule );
		}

		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.01f;
			bodyDef.linearVelocity = { 2.5f * sign, 0.0f };

			bodyDef.position = { -11.0f, 4.5f };
			m_bodyIds[3] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[3], &shapeDef, &square );

			bodyDef.position = { 0.0f, 5.0f };
			m_bodyIds[4] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[4], &shapeDef, &square );

			bodyDef.position = { 11.0f, 5.0f };
			m_bodyIds[5] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[5], &shapeDef, &square );
		}

		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.2f;
			bodyDef.linearVelocity = { 1.5f * sign, 0.0f };

			float offset = m_flip ? 4.0f : 0.0f;

			bodyDef.position = { -13.0f + offset, 4.5f };
			m_bodyIds[6] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_bodyIds[6], &shapeDef, &circle );

			bodyDef.position = { -2.0f + offset, 5.0f };
			m_bodyIds[7] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_bodyIds[7], &shapeDef, &circle );

			bodyDef.position = { 9.0f + offset, 5.0f };
			m_bodyIds[8] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_bodyIds[8], &shapeDef, &circle );
		}
	}

	void UpdateUI() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 160.0f, height ) );

		ImGui::Begin( "Cliff", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Flip" ) )
		{
			m_flip = !m_flip;
			CreateBodies();
		}

		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new Cliff( settings );
	}

	b2BodyId m_bodyIds[9];
	bool m_flip;
};

static int sampleCliff = RegisterSample( "Stacking", "Cliff", Cliff::Create );

class Arch : public Sample
{
public:
	explicit Arch( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 8.0f };
			g_camera.m_zoom = 25.0f * 0.35f;
		}

		b2Vec2 ps1[9] = { { 16.0f, 0.0f },
						  { 14.93803712795643f, 5.133601056842984f },
						  { 13.79871746027416f, 10.24928069555078f },
						  { 12.56252963284711f, 15.34107019122473f },
						  { 11.20040987372525f, 20.39856541571217f },
						  { 9.66521217819836f, 25.40369899225096f },
						  { 7.87179930638133f, 30.3179337000085f },
						  { 5.635199558196225f, 35.03820717801641f },
						  { 2.405937953536585f, 39.09554102558315f } };

		b2Vec2 ps2[9] = { { 24.0f, 0.0f },
						  { 22.33619528222415f, 6.02299846205841f },
						  { 20.54936888969905f, 12.00964361211476f },
						  { 18.60854610798073f, 17.9470321677465f },
						  { 16.46769273811807f, 23.81367936585418f },
						  { 14.05325025774858f, 29.57079353071012f },
						  { 11.23551045834022f, 35.13775818285372f },
						  { 7.752568160730571f, 40.30450679009583f },
						  { 3.016931552701656f, 44.28891593799322f } };

		float scale = 0.25f;
		for ( int i = 0; i < 9; ++i )
		{
			ps1[i] = b2MulSV( scale, ps1[i] );
			ps2[i] = b2MulSV( scale, ps2[i] );
		}

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.6f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2Segment segment = { { -100.0f, 0.0f }, { 100.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		for ( int i = 0; i < 8; ++i )
		{
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2Vec2 ps[4] = { ps1[i], ps2[i], ps2[i + 1], ps1[i + 1] };
			b2Hull hull = b2ComputeHull( ps, 4 );
			b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		}

		for ( int i = 0; i < 8; ++i )
		{
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2Vec2 ps[4] = { { -ps2[i].x, ps2[i].y },
							 { -ps1[i].x, ps1[i].y },
							 { -ps1[i + 1].x, ps1[i + 1].y },
							 { -ps2[i + 1].x, ps2[i + 1].y } };
			b2Hull hull = b2ComputeHull( ps, 4 );
			b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		}

		{
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2Vec2 ps[4] = { ps1[8], ps2[8], { -ps2[8].x, ps2[8].y }, { -ps1[8].x, ps1[8].y } };
			b2Hull hull = b2ComputeHull( ps, 4 );
			b2Polygon polygon = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		}

		for ( int i = 0; i < 4; ++i )
		{
			b2Polygon box = b2MakeBox( 2.0f, 0.5f );
			bodyDef.position = { 0.0f, 0.5f + ps2[8].y + 1.0f * i };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Arch( settings );
	}
};

static int sampleArch = RegisterSample( "Stacking", "Arch", Arch::Create );

class DoubleDomino : public Sample
{
public:
	explicit DoubleDomino( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 4.0f };
			g_camera.m_zoom = 25.0f * 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 100.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		b2Polygon box = b2MakeBox( 0.125f, 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.6f;
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		int count = 15;
		float x = -0.5f * count;
		for ( int i = 0; i < count; ++i )
		{
			bodyDef.position = { x, 0.5f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
			if ( i == 0 )
			{
				b2Body_ApplyLinearImpulse( bodyId, b2Vec2{ 0.2f, 0.0f }, b2Vec2{ x, 1.0f }, true );
			}

			x += 1.0f;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new DoubleDomino( settings );
	}
};

static int sampleDoubleDomino = RegisterSample( "Stacking", "Double Domino", DoubleDomino::Create );

class Confined : public Sample
{
public:
	enum
	{
		e_gridCount = 25,
		e_maxCount = e_gridCount * e_gridCount
	};

	explicit Confined( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 10.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Capsule capsule;
			capsule = { { -10.5f, 0.0f }, { 10.5f, 0.0f }, 0.5f };
			b2CreateCapsuleShape( groundId, &shapeDef, &capsule );
			capsule = { { -10.5f, 0.0f }, { -10.5f, 20.5f }, 0.5f };
			b2CreateCapsuleShape( groundId, &shapeDef, &capsule );
			capsule = { { 10.5f, 0.0f }, { 10.5f, 20.5f }, 0.5f };
			b2CreateCapsuleShape( groundId, &shapeDef, &capsule );
			capsule = { { -10.5f, 20.5f }, { 10.5f, 20.5f }, 0.5f };
			b2CreateCapsuleShape( groundId, &shapeDef, &capsule );
		}

		m_row = 0;
		m_column = 0;
		m_count = 0;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };

		while ( m_count < e_maxCount )
		{
			m_row = 0;
			for ( int i = 0; i < e_gridCount; ++i )
			{
				float x = -8.75f + m_column * 18.0f / e_gridCount;
				float y = 1.5f + m_row * 18.0f / e_gridCount;

				bodyDef.position = { x, y };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCircleShape( bodyId, &shapeDef, &circle );

				m_count += 1;
				m_row += 1;
			}
			m_column += 1;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Confined( settings );
	}

	int m_row;
	int m_column;
	int m_count;
};

static int sampleConfined = RegisterSample( "Stacking", "Confined", Confined::Create );

// From PEEL
class CardHouse : public Sample
{
public:
	explicit CardHouse( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.75f, 0.9f };
			g_camera.m_zoom = 25.0f * 0.05f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -2.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.7f;

		b2Polygon groundBox = b2MakeBox( 40.0f, 2.0f );
		b2CreatePolygonShape( groundId, &shapeDef, &groundBox );

		float cardHeight = 0.2f;
		float cardThickness = 0.001f;

		float angle0 = 25.0f * b2_pi / 180.0f;
		float angle1 = -25.0f * b2_pi / 180.0f;
		float angle2 = 0.5f * b2_pi;

		b2Polygon cardBox = b2MakeBox( cardThickness, cardHeight );
		bodyDef.type = b2_dynamicBody;

		int Nb = 5;
		float z0 = 0.0f;
		float y = cardHeight - 0.02f;
		while ( Nb )
		{
			float z = z0;
			for ( int i = 0; i < Nb; i++ )
			{
				if ( i != Nb - 1 )
				{
					bodyDef.position = { z + 0.25f, y + cardHeight - 0.015f };
					bodyDef.rotation = b2MakeRot( angle2 );
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
					b2CreatePolygonShape( bodyId, &shapeDef, &cardBox );
				}

				bodyDef.position = { z, y };
				bodyDef.rotation = b2MakeRot( angle1 );
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &cardBox );

				z += 0.175f;

				bodyDef.position = { z, y };
				bodyDef.rotation = b2MakeRot( angle0 );
				bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &cardBox );

				z += 0.175f;
			}
			y += cardHeight * 2.0f - 0.03f;
			z0 += 0.175f;
			Nb--;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new CardHouse( settings );
	}
};

static int sampleCardHouse = RegisterSample( "Stacking", "Card House", CardHouse::Create );
