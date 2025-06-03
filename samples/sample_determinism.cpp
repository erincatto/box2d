// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "sample.h"

#include "box2d/math_functions.h"

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
			m_context->camera.m_center = { 0.0f, 7.5f };
			m_context->camera.m_zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
	}

	~FallingHinges() override
	{
		DestroyFallingHinges( &m_data );
	}

	void Step() override
	{
		Sample::Step();

		if (m_context->pause == false && m_done == false)
		{
			m_done = UpdateFallingHinges( m_worldId, &m_data );
		}
		else
		{
			DrawTextLine( "sleep step = %d, hash = 0x%08x", m_data.sleepStep, m_data.hash );
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
