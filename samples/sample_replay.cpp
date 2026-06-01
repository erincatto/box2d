// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "determinism.h"
#include "draw.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"

#include <GLFW/glfw3.h>
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
			b2AABB scanBox = { { 5.0f, 1.0f }, { 7.0f, 2.5f } };
			b2World_OverlapAABB( m_worldId, scanBox, filter, OverlapCounter, nullptr );

			b2Vec2 origin = { 0.0f, 12.0f };
			b2Vec2 translation = { 0.0f, -14.0f };
			b2World_CastRayClosest( m_worldId, origin, translation, filter );

			origin = {-10.0f, 2.0f};
			translation = {20.0f, 0.0f};
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

// Names for the inspector readouts
static const char* ReplayBodyTypeName( b2BodyType type )
{
	switch ( type )
	{
		case b2_staticBody: return "static";
		case b2_kinematicBody: return "kinematic";
		case b2_dynamicBody: return "dynamic";
		default: return "?";
	}
}

static const char* ReplayShapeTypeName( b2ShapeType type )
{
	switch ( type )
	{
		case b2_circleShape: return "circle";
		case b2_capsuleShape: return "capsule";
		case b2_segmentShape: return "segment";
		case b2_polygonShape: return "polygon";
		case b2_chainSegmentShape: return "chain segment";
		default: return "?";
	}
}

static const char* ReplayJointTypeName( b2JointType type )
{
	switch ( type )
	{
		case b2_distanceJoint: return "distance";
		case b2_filterJoint: return "filter";
		case b2_motorJoint: return "motor";
		case b2_prismaticJoint: return "prismatic";
		case b2_revoluteJoint: return "revolute";
		case b2_weldJoint: return "weld";
		case b2_wheelJoint: return "wheel";
		default: return "?";
	}
}

static const char* ReplayQueryTypeName( b2RecQueryType type )
{
	switch ( type )
	{
		case b2_recQueryOverlapAABB: return "overlap AABB";
		case b2_recQueryOverlapShape: return "overlap shape";
		case b2_recQueryCastRay: return "cast ray";
		case b2_recQueryCastShape: return "cast shape";
		case b2_recQueryCollideMover: return "collide mover";
		case b2_recQueryCastRayClosest: return "cast ray closest";
		case b2_recQueryCastMover: return "cast mover";
		case b2_recQueryShapeTestPoint: return "shape test point";
		case b2_recQueryShapeRayCast: return "shape ray cast";
		default: return "?";
	}
}

// Pick the first shape whose area contains the click point
struct ReplayPickContext
{
	b2Vec2 point;
	b2ShapeId shape;
};

static bool ReplayPickCallback( b2ShapeId shapeId, void* context )
{
	ReplayPickContext* pick = static_cast<ReplayPickContext*>( context );
	if ( b2Shape_TestPoint( shapeId, pick->point ) )
	{
		pick->shape = shapeId;
		return false;
	}
	return true;
}

// Plays back a recording by re-running the engine one step at a time and drawing the
// replayed world. Stepping is driven by the recorded inputs, not by b2World_Step, so the motion
// reproduces the original session exactly. Pause, single step, and restart use the shared sample
// controls. Mouse picking is disabled because dragging a body would mutate the replayed world
// and diverge it from the recording.
class ReplayFile : public Sample
{
public:
	// The player owns the world we draw, so skip the base world. m_worldId stays null until
	// OpenPlayer adopts the player's world.
	explicit ReplayFile( SampleContext* context )
		: Sample( context, false )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 7.5f };
			m_context->camera.zoom = 10.0f;
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
		m_selKind = SelNone;
		m_selBodyOrdinal = -1;
		m_selSlot = -1;
		m_selQuery = -1;
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
			if ( m_selKind == SelQuery )
			{
				b2RecPlayer_DrawFrameQueries( m_player, &m_context->debugDraw, m_selQuery );
			}
			DrawSelectionHighlight();
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

		DrawInspectorPanel();
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

	// The inspector lives in the wide left panel. This right-panel control just reopens the
	// diagnostics drawer and jumps to the timeline if it was closed.
	bool DrawControls() override
	{
		if ( ImGui::Button( "Show Timeline" ) )
		{
			m_context->showMetrics = true;
			m_selectTimelineTab = true;
		}
		return false;
	}

	// Selection resolution. The selection is stored as creation ordinals so it survives a backward
	// scrub that rebuilds the world. Each frame the ordinal is mapped back to a live id, or to null
	// when that object does not exist at the current frame.
	b2BodyId SelectedBody() const
	{
		if ( m_selBodyOrdinal < 0 )
		{
			return b2_nullBodyId;
		}
		return b2RecPlayer_GetBodyId( m_player, m_selBodyOrdinal );
	}

	b2ShapeId SelectedShape() const
	{
		b2BodyId body = SelectedBody();
		if ( m_selKind != SelShape || b2Body_IsValid( body ) == false )
		{
			return b2_nullShapeId;
		}
		b2ShapeId shapes[32];
		int n = b2Body_GetShapes( body, shapes, 32 );
		return ( m_selSlot >= 0 && m_selSlot < n ) ? shapes[m_selSlot] : b2_nullShapeId;
	}

	b2JointId SelectedJoint() const
	{
		b2BodyId body = SelectedBody();
		if ( m_selKind != SelJoint || b2Body_IsValid( body ) == false )
		{
			return b2_nullJointId;
		}
		b2JointId joints[16];
		int n = b2Body_GetJoints( body, joints, 16 );
		return ( m_selSlot >= 0 && m_selSlot < n ) ? joints[m_selSlot] : b2_nullJointId;
	}

	int FindBodyOrdinal( b2BodyId body ) const
	{
		int count = b2RecPlayer_GetBodyCount( m_player );
		for ( int i = 0; i < count; ++i )
		{
			if ( B2_ID_EQUALS( b2RecPlayer_GetBodyId( m_player, i ), body ) )
			{
				return i;
			}
		}
		return -1;
	}

	// Map a picked shape back to its body ordinal and shape slot. A null shape clears the selection.
	void SelectShape( b2ShapeId shape )
	{
		if ( B2_IS_NULL( shape ) )
		{
			m_selKind = SelNone;
			return;
		}
		b2BodyId body = b2Shape_GetBody( shape );
		int ordinal = FindBodyOrdinal( body );
		if ( ordinal < 0 )
		{
			m_selKind = SelNone;
			return;
		}
		b2ShapeId shapes[32];
		int n = b2Body_GetShapes( body, shapes, 32 );
		int slot = -1;
		for ( int i = 0; i < n; ++i )
		{
			if ( B2_ID_EQUALS( shapes[i], shape ) )
			{
				slot = i;
				break;
			}
		}
		m_selKind = SelShape;
		m_selBodyOrdinal = ordinal;
		m_selSlot = slot;
		m_revealSelection = true; // expand and scroll the tree to the picked shape next draw
	}

	// Draw a body's live contact points and normals, the most useful solver readout
	void DrawBodyContacts( b2BodyId body )
	{
		Draw* draw = m_context->draw;
		b2ContactData contacts[64];
		int capacity = b2Body_GetContactCapacity( body );
		if ( capacity > 64 )
		{
			capacity = 64;
		}
		int count = b2Body_GetContactData( body, contacts, capacity );
		for ( int i = 0; i < count; ++i )
		{
			b2Vec2 originA = b2Body_GetPosition( b2Shape_GetBody( contacts[i].shapeIdA ) );
			const b2Manifold* m = &contacts[i].manifold;
			for ( int j = 0; j < m->pointCount; ++j )
			{
				b2Vec2 point = b2Add( originA, m->points[j].anchorA );
				DrawPoint( draw, point, 6.0f, b2_colorOrange );
				DrawLine( draw, point, b2MulAdd( point, 0.3f, m->normal ), b2_colorOrange );
			}
		}
	}

	// Highlight the current selection without touching the world. Queries are already drawn by
	// b2RecPlayer_DrawFrameQueries, so they need nothing here.
	void DrawSelectionHighlight()
	{
		Draw* draw = m_context->draw;

		if ( m_selKind == SelShape )
		{
			b2ShapeId shape = SelectedShape();
			if ( b2Shape_IsValid( shape ) == false )
			{
				return;
			}
			b2BodyId body = b2Shape_GetBody( shape );
			DrawBounds( draw, b2Shape_GetAABB( shape ), b2_colorYellow );
			DrawTransform( draw, b2Body_GetTransform( body ), 0.5f );
			DrawPoint( draw, b2Body_GetWorldCenterOfMass( body ), 8.0f, b2_colorYellow );
			DrawBodyContacts( body );
		}
		else if ( m_selKind == SelBody )
		{
			b2BodyId body = SelectedBody();
			if ( b2Body_IsValid( body ) == false )
			{
				return;
			}
			DrawBounds( draw, b2Body_ComputeAABB( body ), b2_colorYellow );
			DrawTransform( draw, b2Body_GetTransform( body ), 0.5f );
			DrawPoint( draw, b2Body_GetWorldCenterOfMass( body ), 8.0f, b2_colorYellow );
			DrawBodyContacts( body );
		}
		else if ( m_selKind == SelJoint )
		{
			b2JointId joint = SelectedJoint();
			if ( b2Joint_IsValid( joint ) == false )
			{
				return;
			}
			b2BodyId a = b2Joint_GetBodyA( joint );
			b2BodyId b = b2Joint_GetBodyB( joint );
			if ( b2Body_IsValid( a ) )
			{
				DrawPoint( draw, b2Body_GetWorldCenterOfMass( a ), 8.0f, b2_colorMagenta );
			}
			if ( b2Body_IsValid( b ) )
			{
				DrawPoint( draw, b2Body_GetWorldCenterOfMass( b ), 8.0f, b2_colorMagenta );
			}
		}
	}

	// Wide left panel: an outliner tree of the scene on top, the selected item's full detail below.
	// Its own window, so it is not bound by the fixed-width right Info panel. Opened from Step, which
	// runs inside the imgui frame.
	void DrawInspectorPanel()
	{
		if ( m_player == nullptr )
		{
			return;
		}

		float fontSize = ImGui::GetFontSize();
		float menuBarHeight = ImGui::GetFrameHeight();
		float drawerHeight = 16.0f * fontSize; // matches the diagnostics drawer in sample.cpp
		float top = menuBarHeight + 0.5f * fontSize;
		// Stop above the timeline drawer, which this sample keeps open
		float bottom = m_context->showMetrics ? m_context->camera.height - drawerHeight - fontSize
											   : m_context->camera.height - 0.5f * fontSize;

		ImGui::SetNextWindowPos( { 0.5f * fontSize, top } );
		ImGui::SetNextWindowSize( { 22.0f * fontSize, bottom - top } );
		ImGui::Begin( "Inspector", nullptr,
					  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
						  ImGuiWindowFlags_NoTitleBar );

		ImGui::TextColored( ImVec4( 0.9f, 0.6f, 0.2f, 1.0f ), "Outline" );
		float avail = ImGui::GetContentRegionAvail().y;
		ImGui::BeginChild( "tree", ImVec2( 0.0f, 0.55f * avail ) );
		DrawOutlineTree();
		ImGui::EndChild();

		ImGui::Separator();
		ImGui::TextColored( ImVec4( 0.9f, 0.6f, 0.2f, 1.0f ), "Detail" );
		ImGui::BeginChild( "detail" );
		DrawDetail();
		ImGui::EndChild();

		ImGui::End();
	}

	// The scene tree: bodies (creation order), each expandable to its shapes and joints, plus the
	// current frame's queries. Clicking a row selects it; clicking a body arrow expands it.
	void DrawOutlineTree()
	{
		// A viewport pick asks the tree to reveal its target once: expand the owning body and scroll to
		// the row. Consumed at the end so it never fights the user's own expand/collapse.
		bool reveal = m_revealSelection;

		int count = b2RecPlayer_GetBodyCount( m_player );
		for ( int ord = 0; ord < count; ++ord )
		{
			b2BodyId body = b2RecPlayer_GetBodyId( m_player, ord );
			if ( B2_IS_NULL( body ) || b2Body_IsValid( body ) == false )
			{
				continue;
			}

			bool ownsSelection = m_selBodyOrdinal == ord &&
								 ( m_selKind == SelBody || m_selKind == SelShape || m_selKind == SelJoint );

			const char* name = b2Body_GetName( body );
			char label[64];
			snprintf( label, sizeof( label ), "Body %d  %s###b%d", ord,
					  ( name != nullptr && name[0] != '\0' ) ? name : ReplayBodyTypeName( b2Body_GetType( body ) ), ord );

			ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
			if ( m_selKind == SelBody && m_selBodyOrdinal == ord )
			{
				flags |= ImGuiTreeNodeFlags_Selected;
			}
			// Reveal a picked shape or joint by expanding its body
			if ( reveal && ownsSelection && m_selKind != SelBody )
			{
				ImGui::SetNextItemOpen( true );
			}
			bool open = ImGui::TreeNodeEx( label, flags );
			if ( reveal && ownsSelection && m_selKind == SelBody )
			{
				ImGui::SetScrollHereY( 0.5f );
			}
			if ( ImGui::IsItemClicked() && ImGui::IsItemToggledOpen() == false )
			{
				m_selKind = SelBody;
				m_selBodyOrdinal = ord;
				m_selSlot = -1;
			}
			if ( open == false )
			{
				continue;
			}

			b2ShapeId shapes[32];
			int sn = b2Body_GetShapes( body, shapes, 32 );
			for ( int s = 0; s < sn; ++s )
			{
				char sl[64];
				snprintf( sl, sizeof( sl ), "Shape %d  %s###b%ds%d", s, ReplayShapeTypeName( b2Shape_GetType( shapes[s] ) ),
						  ord, s );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				if ( m_selKind == SelShape && m_selBodyOrdinal == ord && m_selSlot == s )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::TreeNodeEx( sl, lf );
				if ( reveal && m_selKind == SelShape && m_selBodyOrdinal == ord && m_selSlot == s )
				{
					ImGui::SetScrollHereY( 0.5f );
				}
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelShape;
					m_selBodyOrdinal = ord;
					m_selSlot = s;
				}
			}

			b2JointId joints[16];
			int jn = b2Body_GetJoints( body, joints, 16 );
			for ( int j = 0; j < jn; ++j )
			{
				char jl[64];
				snprintf( jl, sizeof( jl ), "%s joint###b%dj%d", ReplayJointTypeName( b2Joint_GetType( joints[j] ) ), ord, j );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				if ( m_selKind == SelJoint && m_selBodyOrdinal == ord && m_selSlot == j )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::TreeNodeEx( jl, lf );
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelJoint;
					m_selBodyOrdinal = ord;
					m_selSlot = j;
				}
			}

			ImGui::TreePop();
		}

		int qn = b2RecPlayer_GetFrameQueryCount( m_player );
		char ql[32];
		snprintf( ql, sizeof( ql ), "Queries (%d)###queries", qn );
		if ( ImGui::TreeNodeEx( ql, ImGuiTreeNodeFlags_SpanAvailWidth ) )
		{
			for ( int i = 0; i < qn; ++i )
			{
				b2RecQueryInfo q = b2RecPlayer_GetFrameQuery( m_player, i );
				char qi[64];
				snprintf( qi, sizeof( qi ), "%s  (%d)###q%d", ReplayQueryTypeName( q.type ), q.hitCount, i );
				ImGuiTreeNodeFlags lf =
					ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_NoTreePushOnOpen;
				if ( m_selKind == SelQuery && m_selQuery == i )
				{
					lf |= ImGuiTreeNodeFlags_Selected;
				}
				ImGui::TreeNodeEx( qi, lf );
				if ( ImGui::IsItemClicked() )
				{
					m_selKind = SelQuery;
					m_selQuery = i;
				}
			}
			ImGui::TreePop();
		}

		m_revealSelection = false;
	}

	// Detail pane for the current selection. Full width, so 64-bit hex fits without clipping.
	void DrawDetail()
	{
		if ( m_selKind == SelNone )
		{
			ImGui::TextWrapped( "Click a node, or a shape in the view." );
			if ( B2_IS_NON_NULL( m_worldId ) )
			{
				b2Vec2 g = b2World_GetGravity( m_worldId );
				b2Counters c = b2World_GetCounters( m_worldId );
				ImGui::Text( "gravity (%.2f, %.2f)", g.x, g.y );
				ImGui::Text( "bodies %d  shapes %d", c.bodyCount, c.shapeCount );
				ImGui::Text( "contacts %d  joints %d", c.contactCount, c.jointCount );
			}
			return;
		}

		if ( m_selKind == SelQuery )
		{
			DrawQueryDetail();
			return;
		}

		b2BodyId body = SelectedBody();
		if ( b2Body_IsValid( body ) == false )
		{
			ImGui::TextDisabled( "Not present at this frame." );
			return;
		}

		DrawBodyDetail( body );
		if ( m_selKind == SelShape )
		{
			b2ShapeId shape = SelectedShape();
			if ( b2Shape_IsValid( shape ) )
			{
				DrawShapeDetail( shape );
			}
		}
		else if ( m_selKind == SelJoint )
		{
			b2JointId joint = SelectedJoint();
			if ( b2Joint_IsValid( joint ) )
			{
				DrawJointDetail( joint );
			}
		}
		DrawContactDetail( body );
	}

	void DrawBodyDetail( b2BodyId body )
	{
		if ( ImGui::CollapsingHeader( "Body", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		const char* name = b2Body_GetName( body );
		b2Transform xf = b2Body_GetTransform( body );
		b2Vec2 v = b2Body_GetLinearVelocity( body );

		ImGui::Text( "id      %d", body.index1 );
		ImGui::Text( "name    %s", ( name != nullptr && name[0] != '\0' ) ? name : "(none)" );
		ImGui::Text( "type    %s", ReplayBodyTypeName( b2Body_GetType( body ) ) );
		ImGui::Text( "pos     (%.3f, %.3f)", xf.p.x, xf.p.y );
		ImGui::Text( "angle   %.1f deg", b2Rot_GetAngle( xf.q ) * 57.2957795f );
		ImGui::Text( "vel     (%.3f, %.3f)", v.x, v.y );
		ImGui::Text( "omega   %.3f rad/s", b2Body_GetAngularVelocity( body ) );
		ImGui::Text( "mass    %.4g kg", b2Body_GetMass( body ) );
		ImGui::Text( "inertia %.4g", b2Body_GetRotationalInertia( body ) );
		ImGui::Text( "awake   %s", b2Body_IsAwake( body ) ? "yes" : "no" );
		ImGui::Text( "enabled %s", b2Body_IsEnabled( body ) ? "yes" : "no" );
		ImGui::Text( "bullet  %s", b2Body_IsBullet( body ) ? "yes" : "no" );
		ImGui::Text( "gravity scale %.2f", b2Body_GetGravityScale( body ) );
		ImGui::Text( "shapes %d  joints %d", b2Body_GetShapeCount( body ), b2Body_GetJointCount( body ) );
	}

	void DrawShapeDetail( b2ShapeId shape )
	{
		if ( ImGui::CollapsingHeader( "Shape", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		ImGui::Text( "type     %s", ReplayShapeTypeName( b2Shape_GetType( shape ) ) );
		b2Filter f = b2Shape_GetFilter( shape );
		ImGui::Text( "category 0x%016llx", (unsigned long long)f.categoryBits );
		ImGui::Text( "mask     0x%016llx", (unsigned long long)f.maskBits );
		ImGui::Text( "group    %d", f.groupIndex );
		ImGui::Text( "density  %.3g", b2Shape_GetDensity( shape ) );
		ImGui::Text( "friction %.3g", b2Shape_GetFriction( shape ) );
		ImGui::Text( "restitution %.3g", b2Shape_GetRestitution( shape ) );
		ImGui::Text( "sensor   %s", b2Shape_IsSensor( shape ) ? "yes" : "no" );
		b2SurfaceMaterial mat = b2Shape_GetSurfaceMaterial( shape );
		ImGui::Text( "custom color 0x%06x", (unsigned)mat.customColor );
		b2AABB aabb = b2Shape_GetAABB( shape );
		ImGui::Text( "aabb (%.2f, %.2f)-(%.2f, %.2f)", aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x,
					 aabb.upperBound.y );
	}

	void DrawContactDetail( b2BodyId body )
	{
		b2ContactData contacts[64];
		int capacity = b2Body_GetContactCapacity( body );
		if ( capacity > 64 )
		{
			capacity = 64;
		}
		int count = b2Body_GetContactData( body, contacts, capacity );

		char header[32];
		snprintf( header, sizeof( header ), "Contacts (%d)###contacts", count );
		if ( ImGui::CollapsingHeader( header ) == false )
		{
			return;
		}

		for ( int i = 0; i < count; ++i )
		{
			const b2Manifold* m = &contacts[i].manifold;
			ImGui::Text( "shapes %d / %d   normal (%.2f, %.2f)   points %d", contacts[i].shapeIdA.index1,
						 contacts[i].shapeIdB.index1, m->normal.x, m->normal.y, m->pointCount );
			for ( int j = 0; j < m->pointCount; ++j )
			{
				const b2ManifoldPoint* mp = &m->points[j];
				ImGui::Text( "  sep %.4f  Pn %.3g  Pt %.3g", mp->separation, mp->normalImpulse, mp->tangentImpulse );
			}
		}
	}

	void DrawJointDetail( b2JointId joint )
	{
		if ( ImGui::CollapsingHeader( "Joint", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		b2JointType type = b2Joint_GetType( joint );
		ImGui::Text( "type     %s", ReplayJointTypeName( type ) );
		ImGui::Text( "body A   %d", b2Joint_GetBodyA( joint ).index1 );
		ImGui::Text( "body B   %d", b2Joint_GetBodyB( joint ).index1 );
		ImGui::Text( "collide  %s", b2Joint_GetCollideConnected( joint ) ? "yes" : "no" );
		ImGui::Text( "force    %.3g", b2Length( b2Joint_GetConstraintForce( joint ) ) );
		ImGui::Text( "torque   %.3g", b2Joint_GetConstraintTorque( joint ) );

		switch ( type )
		{
			case b2_revoluteJoint:
				ImGui::Text( "angle    %.1f deg", b2RevoluteJoint_GetAngle( joint ) * 57.2957795f );
				break;
			case b2_prismaticJoint:
				ImGui::Text( "translation %.3f", b2PrismaticJoint_GetTranslation( joint ) );
				break;
			case b2_distanceJoint:
				ImGui::Text( "length   %.3f", b2DistanceJoint_GetCurrentLength( joint ) );
				break;
			default:
				break;
		}
	}

	void DrawQueryDetail()
	{
		int count = b2RecPlayer_GetFrameQueryCount( m_player );
		if ( m_selQuery < 0 || m_selQuery >= count )
		{
			ImGui::TextDisabled( "Query not present at this frame." );
			return;
		}

		b2RecQueryInfo q = b2RecPlayer_GetFrameQuery( m_player, m_selQuery );
		if ( ImGui::CollapsingHeader( "Query", ImGuiTreeNodeFlags_DefaultOpen ) == false )
		{
			return;
		}

		ImGui::Text( "type     %s", ReplayQueryTypeName( q.type ) );
		bool shapeLocal = q.type == b2_recQueryShapeTestPoint || q.type == b2_recQueryShapeRayCast;
		if ( shapeLocal == false )
		{
			ImGui::Text( "category 0x%016llx", (unsigned long long)q.filter.categoryBits );
			ImGui::Text( "mask     0x%016llx", (unsigned long long)q.filter.maskBits );
		}
		else
		{
			ImGui::Text( "shape    %d", q.shape.index1 );
		}
		ImGui::Text( "hits     %d", q.hitCount );

		// Hits as one wrapped id list, so a 50-hit query stays compact
		char line[256];
		int len = 0;
		for ( int h = 0; h < q.hitCount && len < (int)sizeof( line ) - 12; ++h )
		{
			b2RecQueryHit hit = b2RecPlayer_GetFrameQueryHit( m_player, m_selQuery, h );
			len += snprintf( line + len, sizeof( line ) - len, "%d ", hit.shape.index1 );
		}
		if ( q.hitCount > 0 )
		{
			ImGui::TextWrapped( "hit shapes: %s", line );
		}
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

	// Left click selects a shape to inspect. Picking only reads the world, it never creates the drag
	// joint the base sample does, so the replay is not mutated. Dragging stays disabled.
	void MouseDown( b2Vec2 p, int button, int ) override
	{
		if ( button != GLFW_MOUSE_BUTTON_1 || B2_IS_NULL( m_worldId ) )
		{
			return;
		}

		b2Vec2 d = { 0.001f, 0.001f };
		b2AABB box = { b2Sub( p, d ), b2Add( p, d ) };
		ReplayPickContext pick = { p, b2_nullShapeId };
		b2World_OverlapAABB( m_worldId, box, b2DefaultQueryFilter(), ReplayPickCallback, &pick );

		// A miss clears the selection
		SelectShape( pick.shape );
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

	// Inspector selection, keyed by stable creation ordinals so it survives a backward scrub. Resolved
	// to live ids each frame from the player's body tracking; out of range means "not at this frame".
	enum SelKind
	{
		SelNone,
		SelBody,
		SelShape,
		SelJoint,
		SelQuery
	};
	SelKind m_selKind = SelNone;
	int m_selBodyOrdinal = -1; // index into the player's tracked body list
	int m_selSlot = -1;        // shape or joint slot within that body
	int m_selQuery = -1;       // query index, only meaningful for the current frame
	bool m_revealSelection = false; // one-shot request to expand and scroll the tree to a viewport pick
};

static int sampleReplayFile = RegisterSample( "Replay", "Replay File", ReplayFile::Create );
