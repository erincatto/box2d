// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "sample.h"

#include "box2d/math_functions.h"

#include <stdio.h>
#include <stdlib.h>

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

	static float CastFcn( b2ShapeId, b2Pos, b2Vec2, float fraction, void* )
	{
		return 1.0f;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_context->pause == false && m_done == false )
		{
			m_done = UpdateFallingHinges( m_worldId, &m_data );

			b2QueryFilter filter = b2DefaultQueryFilter();
			b2AABB scanBox = { { 5.0f, 1.0f }, { 7.0f, 2.5f } };
			b2World_OverlapAABB( m_worldId, b2Pos_zero, scanBox, filter, OverlapFcn, nullptr );

			b2Pos origin = { 0.0f, 12.0f };
			b2Vec2 translation = { 0.0f, -14.0f };

			if ( m_stepCount == 30 )
			{
				b2World_CastRayClosest( m_worldId, origin, translation, filter );
			}

			if ( m_stepCount < 5 || 100 < m_stepCount )
			{
				origin = { -10.0f, 2.0f };
				translation = { 20.0f, 0.0f };
				b2World_CastRay( m_worldId, origin, translation, filter, CastFcn, nullptr );
			}

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

class SnapShot : public Sample
{
public:
	explicit SnapShot( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
		m_image = nullptr;
		m_size = 0;
	}

	~SnapShot() override
	{
		if ( m_image != nullptr )
		{
			free( m_image );
		}

		DestroyFallingHinges( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == 50 )
		{
			m_size = b2World_Snapshot( m_worldId, nullptr, 0 );
			m_image = (uint8_t*)malloc( m_size );
			b2World_Snapshot( m_worldId, m_image, m_size );
		}
		else if ( m_stepCount == 150 && m_image != nullptr )
		{
			b2World_Restore( m_worldId, m_image, m_size );
		}

		if ( m_context->pause == false && m_done == false )
		{
			m_done = UpdateFallingHinges( m_worldId, &m_data );

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
		return new SnapShot( context );
	}

	FallingHingeData m_data;
	uint8_t* m_image;
	int m_size;
	bool m_done;
};

static int sampleSnapShot = RegisterSample( "Determinism", "SnapShot", SnapShot::Create );
