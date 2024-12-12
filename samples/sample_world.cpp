// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "car.h"
#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class LargeWorld : public Sample
{
public:
	explicit LargeWorld( Settings& settings )
		: Sample( settings )
	{
		m_period = 40.0f;
		float omega = 2.0 * B2_PI / m_period;
		m_cycleCount = g_sampleDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)( m_cycleCount * m_period / m_gridSize );

		float xStart = -0.5f * ( m_cycleCount * m_period );

		m_viewPosition = { xStart, 15.0f };

		if ( settings.restart == false )
		{
			g_camera.m_center = m_viewPosition;
			g_camera.m_zoom = 25.0f * 1.0f;
			settings.drawJoints = false;
			settings.useCameraBounds = true;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// Setting this to false significantly reduces the cost of creating
			// static bodies and shapes.
			shapeDef.invokeContactCreation = false;

			float height = 4.0f;
			float xBody = xStart;
			float xShape = xStart;

			b2BodyId groundId;

			for ( int i = 0; i < m_gridCount; ++i )
			{
				// Create a new body regularly so that shapes are not too far from the body origin.
				// Most algorithms in Box2D work in local coordinates, but contact points are computed
				// relative to the body origin.
				// This makes a noticeable improvement in stability far from the origin.
				if ( i % 10 == 0 )
				{
					bodyDef.position.x = xBody;
					groundId = b2CreateBody( m_worldId, &bodyDef );
					xShape = 0.0f;
				}

				float y = 0.0f;

				int ycount = (int)roundf( height * cosf( omega * xBody ) ) + 12;

				for ( int j = 0; j < ycount; ++j )
				{
					b2Polygon square = b2MakeOffsetBox( 0.4f * m_gridSize, 0.4f * m_gridSize,  { xShape, y }, b2Rot_identity  );
					square.radius = 0.1f;
					b2CreatePolygonShape( groundId, &shapeDef, &square );

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
		int donutIndex = 0;
		for ( int cycleIndex = 0; cycleIndex < m_cycleCount; ++cycleIndex )
		{
			float xbase = ( 0.5f + cycleIndex ) * m_period + xStart;

			int remainder = cycleIndex % 3;
			if ( remainder == 0 )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { xbase - 3.0f, 10.0f };

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2Polygon box = b2MakeBox( 0.3f, 0.2f );

				for ( int i = 0; i < 10; ++i )
				{
					bodyDef.position.y = 10.0f;
					for ( int j = 0; j < 5; ++j )
					{
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
						b2CreatePolygonShape( bodyId, &shapeDef, &box );
						bodyDef.position.y += 0.5f;
					}
					bodyDef.position.x += 0.6f;
				}
			}
			else if ( remainder == 1 )
			{
				b2Vec2 position = { xbase - 2.0f, 10.0f };
				for ( int i = 0; i < 5; ++i )
				{
					Human human = {};
					CreateHuman(&human, m_worldId, position, 1.5f, 0.05f, 0.0f, 0.0f, humanIndex + 1, NULL, false );
					humanIndex += 1;
					position.x += 1.0f;
				}
			}
			else
			{
				b2Vec2 position = { xbase - 4.0f, 12.0f };

				for ( int i = 0; i < 5; ++i )
				{
					Donut donut;
					donut.Spawn( m_worldId, position, 0.75f, 0, NULL );
					donutIndex += 1;
					position.x += 2.0f;
				}
			}
		}

		m_car.Spawn( m_worldId, { xStart + 20.0f, 40.0f }, 10.0f, 2.0f, 0.7f, 2000.0f, nullptr );

		m_cycleIndex = 0;
		m_speed = 0.0f;
		m_explosionPosition = { ( 0.5f + m_cycleIndex ) * m_period + xStart, 7.0f };
		m_explode = true;
		m_followCar = false;
	}

	void UpdateUI() override
	{
		float height = 160.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Large World", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "speed", &m_speed, -400.0f, 400.0f, "%.0f" );
		if ( ImGui::Button( "stop" ) )
		{
			m_speed = 0.0f;
		}

		ImGui::Checkbox( "explode", &m_explode );
		ImGui::Checkbox( "follow car", &m_followCar );

		ImGui::Text( "world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f );
		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		float span = 0.5f * ( m_period * m_cycleCount );
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

		if ( settings.pause )
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = b2ClampFloat( m_viewPosition.x, -span, span );

		if ( m_speed != 0.0f )
		{
			g_camera.m_center = m_viewPosition;
		}

		if ( m_followCar )
		{
			g_camera.m_center.x = b2Body_GetPosition( m_car.m_chassisId ).x;
		}

		float radius = 2.0f;
		if ( ( m_stepCount & 0x1 ) == 0x1 && m_explode )
		{
			m_explosionPosition.x = ( 0.5f + m_cycleIndex ) * m_period - span;

			b2ExplosionDef def = b2DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = radius;
			def.falloff = 0.1f;
			def.impulsePerLength = 1.0f;
			b2World_Explode( m_worldId, &def );

			m_cycleIndex = ( m_cycleIndex + 1 ) % m_cycleCount;
		}

		if ( m_explode )
		{
			g_draw.DrawCircle( m_explosionPosition, radius, b2_colorAzure );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 20.0f );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_car.SetSpeed( -5.0f );
		}

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new LargeWorld( settings );
	}

	Car m_car;
	b2Vec2 m_viewPosition;
	float m_period;
	int m_cycleCount;
	int m_cycleIndex;
	float m_gridCount;
	float m_gridSize;
	float m_speed;

	b2Vec2 m_explosionPosition;
	bool m_explode;
	bool m_followCar;
};

static int sampleLargeWorld = RegisterSample( "World", "Large World", LargeWorld::Create );
