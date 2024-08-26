// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// step = 295, hash = 0x4a4d465c
class FallingHinges : public Sample
{
public:
	enum
	{
		e_columns = 4,
		e_rows = 30,
	};

	explicit FallingHinges( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 7.5f };
			g_camera.m_zoom = 10.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 20.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		for ( int i = 0; i < e_rows * e_columns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		float h = 0.25f;
		float r = 0.1f * h;
		b2Polygon box = b2MakeRoundedBox( h - r, h - r, r );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.3f;

		float offset = 0.4f * h;
		float dx = 10.0f * h;
		float xroot = -0.5f * dx * ( e_columns - 1.0f );

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.1f * b2_pi;
		jointDef.upperAngle = 0.2f * b2_pi;
		jointDef.enableSpring = true;
		jointDef.hertz = 0.5f;
		jointDef.dampingRatio = 0.5f;
		jointDef.localAnchorA = { h, h };
		jointDef.localAnchorB = { offset, -h };
		jointDef.drawSize = 0.1f;

		int bodyIndex = 0;
		int bodyCount = e_rows * e_columns;

		for ( int j = 0; j < e_columns; ++j )
		{
			float x = xroot + j * dx;

			b2BodyId prevBodyId = b2_nullBodyId;

			for ( int i = 0; i < e_rows; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;

				bodyDef.position = { x + offset * i, h + 2.0f * h * i };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				if ((i & 1) == 0)
				{
					prevBodyId = bodyId;
				}
				else
				{
					jointDef.bodyIdA = prevBodyId;
					jointDef.bodyIdB = bodyId;
					b2CreateRevoluteJoint( m_worldId, &jointDef );
					prevBodyId = b2_nullBodyId;
				}

				b2CreatePolygonShape( bodyId, &shapeDef, &box );

				assert( bodyIndex < bodyCount );
				m_bodies[bodyIndex] = bodyId;

				bodyIndex += 1;
			}
		}

		m_hash = 0;
		m_sleepStep = -1;
	}

	void Step(Settings& settings) override
	{
		Sample::Step( settings );

		if (m_hash == 0)
		{
			bool sleeping = true;
			int bodyCount = e_rows * e_columns;
			for (int i = 0; i < bodyCount; ++i)
			{
				if ( b2Body_IsAwake( m_bodies[i] ) == true )
				{
					sleeping = false;
					break;
				}
			}

			if (sleeping == true)
			{
				m_hash = B2_HASH_INIT;
				for ( int i = 0; i < bodyCount; ++i )
				{
					b2Transform xf = b2Body_GetTransform( m_bodies[i] );
					m_hash = b2Hash( m_hash, reinterpret_cast<uint8_t*>( &xf ), sizeof( b2Transform ) );
				}
			
				m_sleepStep = m_stepCount - 1;
				printf( "step = %d, hash = 0x%08x\n", m_sleepStep, m_hash );
			}
		}

		g_draw.DrawString( 5, m_textLine, "sleep step = %d, hash = 0x%08x", m_sleepStep, m_hash );
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new FallingHinges( settings );
	}

	b2BodyId m_bodies[e_rows * e_columns];
	uint32_t m_hash;
	int m_sleepStep;
};

static int sampleFallingHinges = RegisterSample( "Determinism", "Falling Hinges", FallingHinges::Create );
