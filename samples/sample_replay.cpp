// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"

#include <imgui.h>
#include <stdio.h>

// Produces a recording file so the Replay File sample has something to load. Runs a small scene
// with recording enabled at world creation.
class MakeRecording : public Sample
{
public:
	explicit MakeRecording( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
		}

		// Recreate the base world with recording enabled
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2DestroyWorld( m_worldId );
			m_worldId = b2_nullWorldId;
		}

		b2WorldDef worldDef = b2DefaultWorldDef();
		worldDef.workerCount = m_context->workerCount;
		worldDef.enableSleep = m_context->enableSleep;
		worldDef.recordingPath = m_path;
		m_worldId = b2CreateWorld( &worldDef );

		m_data = CreateFallingHinges( m_worldId );
		m_done = false;
	}

	~MakeRecording() override
	{
		DestroyFallingHinges( &m_data );
	}

	static bool OverlapCounter( b2ShapeId, void* )
	{
		return true;
	}

	static float AllHitsCast( b2ShapeId, b2Vec2, b2Vec2, float fraction, void* )
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
			b2AABB scanBox = { { -5.0f, -2.0f }, { 5.0f, 4.0f } };
			b2World_OverlapAABB( m_worldId, scanBox, filter, OverlapCounter, nullptr );
			b2Vec2 origin = { 0.0f, 12.0f };
			b2Vec2 translation = { 0.0f, -14.0f };
			b2World_CastRayClosest( m_worldId, origin, translation, filter );

			origin.x = 5.0f;
			b2World_CastRay( m_worldId, origin, translation, filter, AllHitsCast, nullptr );

			if ( m_done )
			{
				printf( "sleep step = %d, hash = 0x%08X\n", m_data.sleepStep, m_data.hash );

				b2World_StopRecording( m_worldId );
			}
		}
		else
		{
			DrawScreenTextLine( "sleep step = %d, hash = 0x%08X", m_data.sleepStep, m_data.hash );
		}
	}

	bool DrawControls() override
	{
		ImGui::TextWrapped( "Recording to \"%s\".", m_path );
		return true;
	}

	// Block mouse interaction
	void MouseDown( b2Vec2, int, int ) override
	{
	}
	void MouseUp( b2Vec2, int ) override
	{
	}
	void MouseMove( b2Vec2 ) override
	{
	}

	static Sample* Create( SampleContext* context )
	{
		return new MakeRecording( context );
	}

	FallingHingeData m_data;
	bool m_done;

	const char* m_path = "recording.b2rec";
};

static int sampleMakeRecording = RegisterSample( "Replay", "Make Recording", MakeRecording::Create );

// Plays back a recording by re-running the engine one step at a time and drawing the
// replayed world. Stepping is driven by the recorded inputs, not by b2World_Step, so the motion
// reproduces the original session exactly. Pause, single step, and restart use the shared sample
// controls. Mouse picking is disabled because dragging a body would mutate the replayed world
// and diverge it from the recording.
class ReplayFile : public Sample
{
public:
	explicit ReplayFile( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
		}

		// The base ctor created an empty world we do not need. The player owns the world we draw,
		// and the base dtor frees m_worldId, so it must not point at the player's world yet.
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2DestroyWorld( m_worldId );
			m_worldId = b2_nullWorldId;
		}

		// The timeline scrubber lives in the diagnostics drawer, so open it for the replay
		m_prevShowMetrics = m_context->showMetrics;
		m_context->showMetrics = true;
		m_selectTimelineTab = true;

		snprintf( m_path, sizeof( m_path ), "%s", "recording.b2rec" );
		m_status[0] = '\0';
		m_player = nullptr;
		OpenPlayer();
	}

	~ReplayFile() override
	{
		ClosePlayer();

		m_context->showMetrics = m_prevShowMetrics;
		m_worldId = b2_nullWorldId;
	}

	void ClosePlayer()
	{
		if ( m_player != nullptr )
		{
			b2RecPlayer_Destroy( m_player );
			m_player = nullptr;
		}
		m_worldId = b2_nullWorldId;
		m_buildMismatch = false;
	}

	void OpenPlayer()
	{
		ClosePlayer();

		// Replay workers of 0 uses the recorded count, otherwise force a different count
		// to spot-check cross-thread determinism.
		m_player = b2RecPlayer_Create( m_path, m_replayWorkers );
		m_frameAccum = 0.0f;
		if ( m_player != nullptr )
		{
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_info = b2RecPlayer_GetInfo( m_player );

			// Flag a file made by a different engine build. 0 on either side is unstamped.
			m_recHash = b2RecPlayer_GetBuildHash( m_player );
			m_runHash = b2GetBuildHash();
			m_buildMismatch = m_recHash != 0 && m_runHash != 0 && m_recHash != m_runHash;
			snprintf( m_status, sizeof( m_status ), "loaded (build %08x)", m_recHash );
		}
		else
		{
			m_info = b2RecPlayerInfo{};
			snprintf( m_status, sizeof( m_status ), "failed to open file" );
		}
	}

	// Advance one recorded step and keep the world pointer current
	void AdvanceOne()
	{
		b2RecPlayer_StepFrame( m_player );
		m_worldId = b2RecPlayer_GetWorldId( m_player );
	}

	void Step() override
	{
		if ( m_player == nullptr )
		{
			DrawScreenTextLine( "%s", m_status );
			return;
		}

		if ( m_context->pause && m_context->singleStep )
		{
			m_context->singleStep = false;
			if ( b2RecPlayer_IsAtEnd( m_player ) == false )
			{
				AdvanceOne();
			}
			m_frameAccum = 0.0f;
		}
		else if ( m_context->pause == false )
		{
			// Speed scales how many recorded steps pass per display frame. Below 1 advances
			// only every few frames, above 1 advances several.
			m_frameAccum += m_speed;
			while ( m_frameAccum >= 1.0f )
			{
				m_frameAccum -= 1.0f;
				if ( b2RecPlayer_IsAtEnd( m_player ) )
				{
					if ( m_loop )
					{
						b2RecPlayer_Restart( m_player );
						m_worldId = b2RecPlayer_GetWorldId( m_player );
					}
					else
					{
						m_frameAccum = 0.0f;
						break;
					}
				}
				AdvanceOne();
			}
		}

		// Keep the base panel "step N" line tracking the replay frame
		m_stepCount = b2RecPlayer_GetFrame( m_player );

		m_context->debugDraw.drawingBounds = GetViewBounds( &m_context->camera );
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2World_Draw( m_worldId, &m_context->debugDraw );
			b2RecPlayer_DrawFrameQueries( m_player, &m_context->debugDraw );
		}

		DrawScreenTextLine( "frame %d / %d%s", b2RecPlayer_GetFrame( m_player ), m_info.frameCount,
							b2RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );

		if ( b2RecPlayer_HasDiverged( m_player ) )
		{
			DrawScreenTextLine( "****DIVERGED****" );
		}

		if ( m_buildMismatch )
		{
			DrawScreenTextLine( "build mismatch: file %08x, engine %08x", m_recHash, m_runHash );
		}

		if ( m_context->pause )
		{
			DrawScreenTextLine( "****PAUSED****" );
		}
	}

	// Shared transport row used by both the right panel and the timeline tab
	void DrawTransport()
	{
		if ( m_player == nullptr )
		{
			return;
		}

		int frame = b2RecPlayer_GetFrame( m_player );

		if ( ImGui::Button( "|<" ) )
		{
			b2RecPlayer_SeekFrame( m_player, 0 );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
		}
		ImGui::SameLine();
		if ( ImGui::Button( "<" ) )
		{
			b2RecPlayer_SeekFrame( m_player, frame - 1 );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();
		if ( ImGui::Button( m_context->pause ? "Play " : "Pause" ) )
		{
			m_context->pause = !m_context->pause;
		}
		ImGui::SameLine();
		if ( ImGui::Button( ">" ) )
		{
			b2RecPlayer_SeekFrame( m_player, frame + 1 );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();
		if ( ImGui::Button( ">|" ) )
		{
			b2RecPlayer_SeekFrame( m_player, m_info.frameCount );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
		}
	}

	// A replay re-runs recorded inputs, so the live solver sliders would do nothing
	bool HasSolverControls() const override
	{
		return false;
	}

	// The right panel is otherwise free for a future details view and outliner. The one
	// control here reopens the drawer and jumps to the timeline if it was closed.
	bool DrawControls() override
	{
		if ( ImGui::Button( "Show Timeline" ) )
		{
			m_context->showMetrics = true;
			m_selectTimelineTab = true;
		}
		return false;
	}

	// All replay controls live in the diagnostics drawer tab.
	void DrawMetricsTab() override
	{
		ImGuiTabItemFlags tabFlags = 0;
		if ( m_selectTimelineTab )
		{
			tabFlags |= ImGuiTabItemFlags_SetSelected;
			m_selectTimelineTab = false;
		}

		if ( ImGui::BeginTabItem( "Timeline", nullptr, tabFlags ) == false )
		{
			return;
		}

		float fontSize = ImGui::GetFontSize();

		// File row, always available so a recording can be loaded even when none is open
		ImGui::PushItemWidth( 18.0f * fontSize );
		ImGui::InputText( "File", m_path, sizeof( m_path ) );
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if ( ImGui::Button( "Load" ) )
		{
			OpenPlayer();
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Restart" ) && m_player != nullptr )
		{
			b2RecPlayer_Restart( m_player );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
		}
		ImGui::SameLine();
		ImGui::TextUnformatted( m_status );

		if ( m_player == nullptr )
		{
			ImGui::EndTabItem();
			return;
		}

		// Transport row: buttons, speed, loop, replay worker count
		DrawTransport();
		ImGui::SameLine();

		const char* speedNames[] = { "0.25x", "0.5x", "1x", "2x", "4x" };
		const float speedValues[] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
		int speedIndex = 2;
		for ( int i = 0; i < 5; ++i )
		{
			if ( m_speed == speedValues[i] )
			{
				speedIndex = i;
			}
		}
		ImGui::PushItemWidth( 5.0f * fontSize );
		if ( ImGui::Combo( "Speed", &speedIndex, speedNames, 5 ) )
		{
			m_speed = speedValues[speedIndex];
		}
		ImGui::PopItemWidth();
		ImGui::SameLine();
		ImGui::Checkbox( "Loop", &m_loop );
		ImGui::SameLine();

		// Replaying at a different worker count is a visual cross-thread determinism check.
		// 0 means use the recorded count. Re-open on release so the player is not rebuilt mid-drag.
		ImGui::PushItemWidth( 6.0f * fontSize );
		ImGui::SliderInt( "Workers", &m_replayWorkers, 0, B2_MAX_WORKERS );
		ImGui::PopItemWidth();
		bool reopen = ImGui::IsItemDeactivatedAfterEdit();
		ImGui::SameLine();
		ImGui::TextDisabled( "(rec %d)", m_info.workerCount );

		// Scrubber: full width, seeks both directions
		int scrub = b2RecPlayer_GetFrame( m_player );
		ImGui::PushItemWidth( -1.0f );
		if ( ImGui::SliderInt( "##frame", &scrub, 0, m_info.frameCount ) )
		{
			b2RecPlayer_SeekFrame( m_player, scrub );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccum = 0.0f;
			m_context->pause = true;
		}
		ImGui::PopItemWidth();

		// Mark where the replay first diverged on the scrubber track
		int divergeFrame = b2RecPlayer_GetDivergeFrame( m_player );
		if ( divergeFrame >= 0 && m_info.frameCount > 0 )
		{
			ImVec2 lo = ImGui::GetItemRectMin();
			ImVec2 hi = ImGui::GetItemRectMax();
			float t = (float)divergeFrame / (float)m_info.frameCount;
			float x = lo.x + t * ( hi.x - lo.x );
			ImGui::GetWindowDrawList()->AddLine( ImVec2( x, lo.y ), ImVec2( x, hi.y ), IM_COL32( 220, 60, 60, 255 ), 2.0f );
		}

		// Info row: recording metadata, live counts, divergence
		ImGui::Text( "frames %d", m_info.frameCount );
		if ( m_info.timeStep > 0.0f )
		{
			ImGui::SameLine();
			ImGui::Text( "   %.0f hz, %d sub-steps", 1.0f / m_info.timeStep, m_info.subStepCount );
		}
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2Counters c = b2World_GetCounters( m_worldId );
			ImGui::SameLine();
			ImGui::Text( "   bodies %d  shapes %d  contacts %d  joints %d", c.bodyCount, c.shapeCount, c.contactCount,
						 c.jointCount );
		}
		if ( divergeFrame >= 0 )
		{
			ImGui::SameLine();
			ImGui::TextColored( ImVec4( 0.85f, 0.30f, 0.30f, 1.0f ), "   diverged at frame %d", divergeFrame );
		}

		// Re-open last so the player is not torn down mid-draw
		if ( reopen )
		{
			OpenPlayer();
		}

		ImGui::EndTabItem();
	}

	// Block mouse interaction.
	void MouseDown( b2Vec2, int, int ) override
	{
	}
	void MouseUp( b2Vec2, int ) override
	{
	}
	void MouseMove( b2Vec2 ) override
	{
	}

	static Sample* Create( SampleContext* context )
	{
		return new ReplayFile( context );
	}

	b2RecPlayer* m_player;
	char m_path[256];
	char m_status[32];
	uint32_t m_recHash = 0;
	uint32_t m_runHash = 0;
	bool m_buildMismatch = false;

	b2RecPlayerInfo m_info = {};
	float m_speed = 1.0f;
	float m_frameAccum = 0.0f;
	int m_replayWorkers = 0;
	bool m_loop = false;
	bool m_selectTimelineTab = true;
	bool m_prevShowMetrics = false;
};

static int sampleReplayFile = RegisterSample( "Replay", "Replay File", ReplayFile::Create );
