// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "car.h"
#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class TileWorld : public Sample
{
public:
	explicit TileWorld( SampleContext* context )
		: Sample( context )
	{
		m_period = 40.0f;
		float omega = 2.0f * B2_PI / m_period;
		m_cycleCount = m_isDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)( m_cycleCount * m_period / m_gridSize );

		float xStart = -0.5f * ( m_cycleCount * m_period );

		m_viewPosition = { xStart, 15.0f };

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2MakePosition( m_viewPosition );
			m_context->camera.zoom = 25.0f * 1.0f;
			m_context->debugDraw.drawJoints = false;
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
					b2Polygon square = b2MakeOffsetBox( 0.4f * m_gridSize, 0.4f * m_gridSize, { xShape, y }, b2Rot_identity );
					square.radius = 0.1f;
					b2CreatePolygonShape( groundId, &shapeDef, &square );

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
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
				b2Position position = { xbase - 2.0f, 10.0f };
				for ( int i = 0; i < 5; ++i )
				{
					Human human = {};
					CreateHuman( &human, m_worldId, position, 1.5f, 0.05f, 0.0f, 0.0f, humanIndex + 1, nullptr, false );
					humanIndex += 1;
					position.x += 1.0f;
				}
			}
			else
			{
				b2Position position = { xbase - 4.0f, 12.0f };

				for ( int i = 0; i < 5; ++i )
				{
					Donut donut;
					donut.Create( m_worldId, position, 0.75f, 0, false, nullptr );
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

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		ImGui::SliderFloat( "speed", &m_speed, -400.0f, 400.0f, "%.0f" );
		ImGui::PopItemWidth();
		if ( ImGui::Button( "stop" ) )
		{
			m_speed = 0.0f;
		}

		ImGui::Checkbox( "explode", &m_explode );
		ImGui::Checkbox( "follow car", &m_followCar );

		ImGui::Text( "world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f );

		return true;
	}

	void Step() override
	{
		float span = 0.5f * ( m_period * m_cycleCount );
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		if ( m_context->pause )
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = b2ClampFloat( m_viewPosition.x, -span, span );

		if ( m_speed != 0.0f )
		{
			m_context->camera.center = b2MakePosition( m_viewPosition );
		}

		if ( m_followCar )
		{
			m_context->camera.center.x = b2Body_GetPosition( m_car.m_chassisId ).x;
		}

		float radius = 2.0f;
		if ( ( m_stepCount & 0x1 ) == 0x1 && m_explode )
		{
			m_explosionPosition.x = ( 0.5f + m_cycleIndex ) * m_period - span;

			b2ExplosionDef def = b2DefaultExplosionDef();
			def.position = b2MakePosition( m_explosionPosition );
			def.radius = radius;
			def.falloff = 0.1f;
			def.impulsePerLength = 1.0f;
			b2World_Explode( m_worldId, &def );

			m_cycleIndex = ( m_cycleIndex + 1 ) % m_cycleCount;
		}

		if ( m_explode )
		{
			DrawCircle( m_draw, m_explosionPosition, radius, b2_colorAzure );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 20.0f );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_car.SetSpeed( -5.0f );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new TileWorld( context );
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

static int sampleTileWorld = RegisterSample( "World", "Tiles", TileWorld::Create );

#ifdef BOX2D_DOUBLE_PRECISION

// A pyramid built far from the origin to exercise double precision world positions. The contact
// solver runs in delta space, so the stack settles the same as one at the origin while the body
// transforms hold their full double coordinate. Record it (R) and open it in the Replay Viewer:
// the recorded doubles survive the snapshot and the motion reproduces with no divergence.
class FarPyramid : public Sample
{
public:
	explicit FarPyramid( SampleContext* context )
		: Sample( context )
	{
		// 1e7 is exactly representable in float, so integer box offsets stay exact and a run here
		// can be compared against one at the origin.
		b2Position origin = b2MakePosition( { 10.0e6f, 0.0f } );

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2OffsetPosition( origin, { 0.0f, 12.0f } );
			m_context->camera.zoom = 17.0f;
		}

		float h = 0.25f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = origin;
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		b2Polygon box = b2MakeBox( h, h );
		bodyDef.type = b2_dynamicBody;

		int baseCount = 50;
		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * h;
			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * h + 2.0f * ( j - i ) * h - h * baseCount;
				bodyDef.position = b2OffsetPosition( origin, { x, y } );
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		b2Position c = m_context->camera.center;
		DrawScreenTextLine( "view center (%.1f, %.1f) m from world origin", c.x, c.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarPyramid( context );
	}
};

static int sampleFarPyramid = RegisterSample( "World", "Far Pyramid", FarPyramid::Create );

class FarRagdolls : public Sample
{
public:
	explicit FarRagdolls( SampleContext* context )
		: Sample( context )
	{
		b2Position origin = b2MakePosition( { 10.0e6f, 0.0f } );

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2OffsetPosition( origin, { 0.0f, 6.0f } );
			m_context->camera.zoom = 10.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = origin;
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		float w = 6.0f;
		float h = 12.0f;
		b2Segment floor = { { -w, 0.0f }, { w, 0.0f } };
		b2Segment leftWall = { { -w, 0.0f }, { -w, h } };
		b2Segment rightWall = { { w, 0.0f }, { w, h } };
		b2CreateSegmentShape( groundId, &shapeDef, &floor );
		b2CreateSegmentShape( groundId, &shapeDef, &leftWall );
		b2CreateSegmentShape( groundId, &shapeDef, &rightWall );

		float scale = 1.0f;
		int index = 0;
		for ( int i = 0; i < e_rowCount; ++i )
		{
			for ( int j = 0; j < e_columnCount; ++j )
			{
				float x = 2.4f * scale * ( j - 0.5f * ( e_columnCount - 1 ) ) + RandomFloatRange( -0.3f, 0.3f );
				float y = 2.0f + 2.2f * scale * i;
				b2Position p = b2OffsetPosition( origin, { x, y } );
				CreateHuman( m_humans + index, m_worldId, p, scale, 0.05f, 1.0f, 0.5f, index + 1, nullptr, false );
				index += 1;
			}
		}

		m_humanCount = index;
	}

	void Step() override
	{
		Sample::Step();

		b2Position c = m_context->camera.center;
		DrawScreenTextLine( "%d ragdolls piled %.0f m from the world origin", m_humanCount, c.x );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarRagdolls( context );
	}

	static constexpr int e_columnCount = 5;
	static constexpr int e_rowCount = 5;
	Human m_humans[e_columnCount * e_rowCount] = {};
	int m_humanCount = 0;
};

static int sampleFarRagdolls = RegisterSample( "World", "Far Ragdolls", FarRagdolls::Create );

#endif
