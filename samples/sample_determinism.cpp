// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
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

	explicit FallingHinges( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 7.5f };
			g_camera.m_zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
	}

	~FallingHinges() override
	{
		DestroyFallingHinges( &m_data );
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if (m_done == false)
		{
			m_done = UpdateFallingHinges( m_worldId, &m_data );
		}
		else
		{
			g_draw.DrawString( 5, m_textLine, "sleep step = %d, hash = 0x%08x", m_data.sleepStep, m_data.hash );
			m_textLine += m_textIncrement;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new FallingHinges( settings );
	}

	FallingHingeData m_data;
	bool m_done;
};

static int sampleFallingHinges = RegisterSample( "Determinism", "Falling Hinges", FallingHinges::Create );
