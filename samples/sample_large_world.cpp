// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <imgui.h>

// A pyramid built far from the origin to exercise double precision world positions. The contact
// solver runs in delta space, so the stack settles the same as one at the origin while the body
// transforms hold their full double coordinate. Record it (R) and open it in the Replay Viewer:
// the recorded doubles survive the snapshot and the motion reproduces with no divergence.
class LargeWorld : public Sample
{
public:
	explicit LargeWorld( SampleContext* context )
		: Sample( context )
	{
		// 1e7 is exactly representable in float, so integer box offsets stay exact and a run here
		// can be compared against one at the origin.
		b2Position origin = b2MakePosition( { 1.0e7f, 0.0f } );

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2OffsetPosition( origin, {0.0f, 5.0f} );
			m_context->camera.zoom = 14.0f;
		}

		float h = 0.5f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = origin;
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		b2Polygon box = b2MakeBox( h, h );
		bodyDef.type = b2_dynamicBody;

		int baseCount = 14;
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
		return new LargeWorld( context );
	}
};

static int sampleLargeWorld = RegisterSample( "World", "Large World", LargeWorld::Create );
