// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "sample.h"

#include "box2d/math_functions.h"

#include <stdio.h>

// This sample provides a visual representation of the cross platform determinism unit test.
// The scenario is designed to produce a chaotic result engaging:
// - continuous collision
// - joint limits (approximate atan2)
// - b2MakeRot (approximate sin/cos)
// Once all the bodies go to sleep the step counter and transform hash is emitted which
// can then be transferred to the unit test and tested in GitHub build actions.
// See CrossPlatformTest in the unit tests.
class FallingHinges : public Sample
{
public:

	explicit FallingHinges( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
	}

	~FallingHinges() override
	{
		DestroyFallingHinges( &m_data );
	}

	static bool OverlapFcn( b2ShapeId, void* )
	{
		return true;
	}

	static float CastFcn( b2ShapeId, b2Vec2, b2Vec2, float fraction, void* )
	{
		return 1.0f;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_context->pause == false && m_done == false )
		{
			m_done = UpdateFallingHinges( m_worldId, &m_data );

			// Issue a few queries each step so the Replay viewer has something to draw
			b2QueryFilter filter = b2DefaultQueryFilter();
			b2AABB scanBox = { { 5.0f, 1.0f }, { 7.0f, 2.5f } };
			b2World_OverlapAABB( m_worldId, scanBox, filter, OverlapFcn, nullptr );

			b2Vec2 origin = { 0.0f, 12.0f };
			b2Vec2 translation = { 0.0f, -14.0f };
			b2World_CastRayClosest( m_worldId, origin, translation, filter );

			origin = { -10.0f, 2.0f };
			translation = { 20.0f, 0.0f };
			b2World_CastRay( m_worldId, origin, translation, filter, CastFcn, nullptr );

			if ( m_done )
			{
				printf( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash );

				FinishRecording();
			}
		}
		else
		{
			DrawScreenTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FallingHinges( context );
	}

	FallingHingeData m_data;
	bool m_done;
};

static int sampleFallingHinges = RegisterSample( "Determinism", "Falling Hinges", FallingHinges::Create );
