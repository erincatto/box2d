// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"

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
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 25.0f;
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

		b2BodyDef groundDef = b2DefaultBodyDef();
		groundDef.position = { 0.0f, -10.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &groundDef );
		b2ShapeDef groundShapeDef = b2DefaultShapeDef();
		b2Circle groundCircle = { { 0.0f, 0.0f }, 10.0f };
		b2CreateCircleShape( groundId, &groundShapeDef, &groundCircle );

		for ( int i = 0; i < 24; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -1.75f + 0.5f * ( i % 8 ), 6.0f + 0.7f * ( i / 8 ) };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2Circle circle = { { 0.0f, 0.0f }, 0.22f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
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
			m_context->camera.center = { 0.0f, 0.0f };
			m_context->camera.zoom = 25.0f;
		}

		// The base ctor created an empty world we do not need. The player owns the world we draw,
		// and the base dtor frees m_worldId, so it must not point at the player's world yet.
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2DestroyWorld( m_worldId );
			m_worldId = b2_nullWorldId;
		}

		snprintf( m_path, sizeof( m_path ), "%s", "recording.b2rec" );
		m_status[0] = '\0';
		m_player = nullptr;
		OpenPlayer();
	}

	~ReplayFile() override
	{
		ClosePlayer();

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
	}

	void OpenPlayer()
	{
		ClosePlayer();

		// A worker count of 0 uses the recorded count.
		m_player = b2RecPlayer_Create( m_path, 0 );
		if ( m_player != nullptr )
		{
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			snprintf( m_status, sizeof( m_status ), "loaded file" );
		}
		else
		{
			snprintf( m_status, sizeof( m_status ), "failed to open file" );
		}
	}

	void Step() override
	{
		if ( m_player == nullptr )
		{
			DrawScreenTextLine( "%s", m_status );
			return;
		}

		bool advance = m_context->pause == false;
		if ( m_context->pause && m_context->singleStep )
		{
			m_context->singleStep = false;
			advance = true;
		}

		if ( advance && b2RecPlayer_IsAtEnd( m_player ) == false )
		{
			b2RecPlayer_StepFrame( m_player );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
		}

		m_context->debugDraw.drawingBounds = GetViewBounds( &m_context->camera );
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2World_Draw( m_worldId, &m_context->debugDraw );
		}

		DrawScreenTextLine( "frame %d%s", b2RecPlayer_GetFrame( m_player ),
		                    b2RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );

		if ( b2RecPlayer_HasDiverged( m_player ) )
		{
			DrawScreenTextLine( "****DIVERGED****" );
		}
		
		if ( m_context->pause )
		{
			DrawScreenTextLine( "****PAUSED****" );
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 18.0f * ImGui::GetFontSize() );
		ImGui::InputText( "File", m_path, sizeof( m_path ) );
		ImGui::PopItemWidth();

		if ( ImGui::Button( "Load" ) )
		{
			OpenPlayer();
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Restart" ) && m_player != nullptr )
		{
			b2RecPlayer_Restart( m_player );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
		}

		ImGui::TextUnformatted( m_status );
		return true;
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
};

static int sampleReplayFile = RegisterSample( "Replay", "Replay File", ReplayFile::Create );
