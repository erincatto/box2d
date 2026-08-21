// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "sample.h"

#include "box2d/math_functions.h"

#include <float.h>
#include <imgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

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

// Records a snapshot every few steps of a full run, then rolls the world back to any of them.
// Resimulating from a rollback point has to reach the same sleep step and transform hash as the
// original run. This demonstrates rollback determinism.
class Rollback : public Sample
{
public:
	explicit Rollback( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
		}

		m_data = CreateFallingHinges( m_worldId );
		m_referenceSleepStep = 0;
		m_referenceHash = 0;
		m_selected = 0;
		m_rollbackStep = -1;
		m_result = e_running;
		m_done = false;

		Capture();
	}

	~Rollback() override
	{
		for ( Snapshot& snapshot : m_snapshots )
		{
			free( snapshot.image );
		}

		DestroyFallingHinges( &m_data );
	}

	void Capture()
	{
		Snapshot snapshot;
		snapshot.step = m_stepCount;
		snapshot.size = b2World_Snapshot( m_worldId, nullptr, 0 );
		snapshot.image = (uint8_t*)malloc( snapshot.size );
		b2World_Snapshot( m_worldId, snapshot.image, snapshot.size );

		m_snapshots.push_back( snapshot );
	}

	void RollBack( int index )
	{
		const Snapshot& snapshot = m_snapshots[index];
		b2World_Restore( m_worldId, snapshot.image, snapshot.size );

		m_data.stepCount = snapshot.step;
		m_data.sleepStep = -1;
		m_data.hash = 0;

		m_rollbackStep = snapshot.step;
		m_result = e_running;
		m_done = false;
	}

	bool DrawControls() override
	{
		if ( m_referenceHash == 0 )
		{
			ImGui::Text( "reference run: step %d", m_stepCount );
			return true;
		}

		ImGui::Text( "reference sleep step %d", m_referenceSleepStep );
		ImGui::Text( "reference hash 0x%08X", m_referenceHash );

		int totalSize = 0;
		for ( const Snapshot& snapshot : m_snapshots )
		{
			totalSize += snapshot.size;
		}
		ImGui::TextDisabled( "%d snapshots, %.0f kB", int( m_snapshots.size() ), totalSize / 1024.0f );

		if ( ImGui::BeginListBox( "##snapshots", { -FLT_MIN, 8.0f * ImGui::GetTextLineHeightWithSpacing() } ) )
		{
			for ( int i = 0; i < int( m_snapshots.size() ); ++i )
			{
				char label[32];
				snprintf( label, sizeof( label ), "step %d, %d kB", m_snapshots[i].step, m_snapshots[i].size / 1024 );

				if ( ImGui::Selectable( label, i == m_selected ) )
				{
					m_selected = i;
				}
			}

			ImGui::EndListBox();
		}

		if ( ImGui::Button( "Roll Back", { 8.0f * ImGui::GetFontSize(), 0.0f } ) )
		{
			RollBack( m_selected );
		}

		if ( m_result == e_running )
		{
			ImGui::Text( "resimulating from step %d", m_rollbackStep );
		}
		else if ( m_result == e_match )
		{
			ImGui::TextColored( ImVec4( 0.4f, 0.8f, 0.4f, 1.0f ), "match: sleep step %d", m_data.sleepStep );
			ImGui::TextColored( ImVec4( 0.4f, 0.8f, 0.4f, 1.0f ), "match: hash 0x%08X", m_data.hash );
		}
		else
		{
			ImGui::TextColored( ImVec4( 0.9f, 0.3f, 0.3f, 1.0f ), "MISMATCH: sleep step %d", m_data.sleepStep );
			ImGui::TextColored( ImVec4( 0.9f, 0.3f, 0.3f, 1.0f ), "MISMATCH: hash 0x%08X", m_data.hash );
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		if ( m_context->pause == false && m_done == false )
		{
			// Only the reference run lays down snapshots. A rollback resims through the same steps
			// and would otherwise pile up duplicates.
			if ( m_referenceHash == 0 && m_stepCount % e_snapshotInterval == 0 )
			{
				Capture();
			}

			m_done = UpdateFallingHinges( m_worldId, &m_data );

			if ( m_done )
			{
				if ( m_referenceHash == 0 )
				{
					m_referenceSleepStep = m_data.sleepStep;
					m_referenceHash = m_data.hash;

					FinishRecording();
				}

				m_result = m_data.sleepStep == m_referenceSleepStep && m_data.hash == m_referenceHash ? e_match : e_mismatch;

				printf( "sleep step = %d, hash = 0x%08X%s\n", m_data.sleepStep, m_data.hash,
						m_result == e_match ? "" : " MISMATCH" );
			}
		}

		if ( m_referenceHash == 0 )
		{
			DrawScreenTextLine( "reference run" );
		}
		else if ( m_result == e_running )
		{
			DrawScreenTextLine( "rolled back to step %d", m_rollbackStep );
		}
		else
		{
			DrawScreenTextLine( "sleep step = %d, hash = 0x%08X%s", m_data.sleepStep, m_data.hash,
								m_result == e_match ? "" : " MISMATCH" );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new Rollback( context );
	}

	struct Snapshot
	{
		uint8_t* image;
		int size;
		int step;
	};

	enum
	{
		e_snapshotInterval = 20
	};

	enum Result
	{
		e_running,
		e_match,
		e_mismatch
	};

	std::vector<Snapshot> m_snapshots;
	FallingHingeData m_data;
	int m_referenceSleepStep;
	uint32_t m_referenceHash;
	int m_selected;
	int m_rollbackStep;
	Result m_result;
	bool m_done;
};

static int sampleRollback = RegisterSample( "Determinism", "Rollback", Rollback::Create );
