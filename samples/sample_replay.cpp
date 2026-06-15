// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"

#include <GLFW/glfw3.h>
#include <float.h>
#include <imgui.h>
#include <limits.h>
#include <stdio.h>

// Names for the inspector readouts
static const char* ReplayBodyTypeName( b2BodyType type )
{
	switch ( type )
	{
		case b2_staticBody:
			return "static";
		case b2_kinematicBody:
			return "kinematic";
		case b2_dynamicBody:
			return "dynamic";
		default:
			return "?";
	}
}

static const char* ReplayShapeTypeName( b2ShapeType type )
{
	switch ( type )
	{
		case b2_circleShape:
			return "circle";
		case b2_capsuleShape:
			return "capsule";
		case b2_segmentShape:
			return "segment";
		case b2_polygonShape:
			return "polygon";
		case b2_chainSegmentShape:
			return "chain segment";
		default:
			return "?";
	}
}

static const char* ReplayJointTypeName( b2JointType type )
{
	switch ( type )
	{
		case b2_distanceJoint:
			return "distance";
		case b2_filterJoint:
			return "filter";
		case b2_motorJoint:
			return "motor";
		case b2_prismaticJoint:
			return "prismatic";
		case b2_revoluteJoint:
			return "revolute";
		case b2_weldJoint:
			return "weld";
		case b2_wheelJoint:
			return "wheel";
		default:
			return "?";
	}
}

static const char* ReplayQueryTypeName( b2RecQueryType type )
{
	switch ( type )
	{
		case b2_recQueryOverlapAABB:
			return "overlap AABB";
		case b2_recQueryOverlapShape:
			return "overlap shape";
		case b2_recQueryCastRay:
			return "cast ray";
		case b2_recQueryCastShape:
			return "cast shape";
		case b2_recQueryCollideMover:
			return "collide mover";
		case b2_recQueryCastRayClosest:
			return "cast ray closest";
		case b2_recQueryCastMover:
			return "cast mover";
		case b2_recQueryShapeTestPoint:
			return "shape test point";
		case b2_recQueryShapeRayCast:
			return "shape ray cast";
		default:
			return "?";
	}
}

// Pick the first shape whose area contains the click point
struct ReplayPickContext
{
	b2Position point;
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
class ReplayViewer : public Sample
{
public:
	// The player owns the world we draw, so skip the base world. m_worldId stays null until
	// CreatePlayer adopts the player's world.
	explicit ReplayViewer( SampleContext* context )
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
		m_context->pause = true;
		m_selectTimelineTab = true;

		snprintf( m_path, sizeof( m_path ), "%s", m_context->replayFile );
		m_status[0] = '\0';
		m_player = nullptr;

		// A fresh open gathers the keyframe policy through the Load popup, then pre-generates every
		// keyframe. A restart reuses the persisted policy and fills the ring lazily so R stays quick.
		if ( m_context->restart == false )
		{
			if ( strlen( m_path ) > 0 )
			{
				m_requestLoadPopup = true;
			}
			else
			{
				snprintf( m_status, sizeof( m_status ), "Open recording from Replay menu" );
			}
		}
		else
		{
			CreatePlayer();
		}
	}

	~ReplayViewer() override
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
		m_selKind = SelNone;
		m_selBodyOrdinal = -1;
		m_selSlot = -1;
		m_selQuery = -1;
	}

	void CreatePlayer()
	{
		ClosePlayer();

		if ( strlen(m_path) == 0 )
		{
			snprintf( m_status, sizeof( m_status ), "Open recording from Replay menu" );
			return;
		}

		// Load the file into a recording buffer, then hand its bytes to the player. The player
		// copies them, so the buffer is freed right away. Replay workers of 0 uses the serial
		// fallback, otherwise force a different count to spot-check cross-thread determinism.
		b2Recording* recording = b2LoadRecordingFromFile( m_path );
		if ( recording != nullptr )
		{
			const uint8_t* data = b2Recording_GetData( recording );
			int byteCount = b2Recording_GetSize( recording );

			// Use a large worker count so key frame generation is fast
			m_context->workerCount = b2MinInt( 8, GetNumberOfCores() / 2 );
			m_player = b2RecPlayer_Create( data, byteCount, m_context->workerCount );
		}
		else
		{
			m_player = nullptr;
		}

		b2DestroyRecording( recording );
		m_frameAccumulator = 0.0f;
		if ( m_player != nullptr )
		{
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_info = b2RecPlayer_GetInfo( m_player );

			// Apply the persisted keyframe policy before any stepping captures keyframes. A freshly
			// created player starts at the engine defaults, so the ring rebuilds under our spacing.
			size_t bytes = (size_t)m_context->replayKeyframeBudgetMB * 1024 * 1024;
			b2RecPlayer_SetKeyframePolicy( m_player, bytes, m_context->replayKeyframeMinInterval );

			snprintf( m_status, sizeof( m_status ), "loaded" );

			if ( m_context->restart == false )
			{
				// Frame the whole recorded motion. Older recordings lack stored bounds, so fall
				// back to the live frame-0 bounds when the recorded extents are empty.
				b2AABB bounds = m_info.bounds;
				b2Vec2 extents = b2AABB_Extents( bounds );
				if ( extents.x <= 0.0f && extents.y <= 0.0f )
				{
					bounds = b2World_GetBounds( m_worldId );
				}
				FocusOnBounds( &m_context->camera, bounds );
				m_context->camera.zoom *= 1.5f;
			}
		}
		else
		{
			m_info = b2RecPlayerInfo{};
			snprintf( m_status, sizeof( m_status ), "failed to open file" );
		}
	}

	// Modal shown after the Replay menu picks a file: choose the keyframe budget and min interval,
	// then Load creates the player and pre-generates the whole ring behind a progress bar. Drawn at
	// the root ID stack from Step, like the sample picker.
	void DrawLoadPopup()
	{
		const char* popupId = "Load Replay";

		if ( m_requestLoadPopup )
		{
			m_requestLoadPopup = false;
			m_popupBudgetMB = m_context->replayKeyframeBudgetMB;
			m_popupMinInterval = m_context->replayKeyframeMinInterval;
			ImGui::OpenPopup( popupId );
		}

		float fontSize = ImGui::GetFontSize();
		ImGui::SetNextWindowPos( { m_context->camera.width * 0.5f, m_context->camera.height * 0.35f }, ImGuiCond_Appearing,
								 { 0.5f, 0.5f } );
		ImGui::SetNextWindowSize( { 26.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

		if ( ImGui::BeginPopupModal( popupId, nullptr, ImGuiWindowFlags_AlwaysAutoResize ) == false )
		{
			return;
		}

		// Show just the file name, paths run long
		const char* slash = strrchr( m_path, '\\' );
		ImGui::TextDisabled( "File:" );
		ImGui::SameLine();
		ImGui::TextUnformatted( slash != nullptr ? slash + 1 : m_path );
		ImGui::Separator();

		if ( m_generating )
		{
			// Step forward in wall-clock slices so the bar animates. Forward stepping captures
			// keyframes at the interval; a restart then returns to frame 0 with the ring kept.
			uint64_t ticks = b2GetTicks();
			while ( b2RecPlayer_IsAtEnd( m_player ) == false && b2GetMilliseconds( ticks ) < 12.0f )
			{
				b2RecPlayer_StepFrame( m_player );
			}

			int frame = b2RecPlayer_GetFrame( m_player );
			int total = m_info.frameCount > 0 ? m_info.frameCount : 1;
			float frac = frame >= total ? 1.0f : (float)frame / (float)total;
			char overlay[32];
			snprintf( overlay, sizeof( overlay ), "%d / %d", frame, m_info.frameCount );
			ImGui::TextUnformatted( "Generating keyframes" );
			ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), overlay );

			if ( b2RecPlayer_IsAtEnd( m_player ) )
			{
				b2RecPlayer_Restart( m_player );
				m_worldId = b2RecPlayer_GetWorldId( m_player );
				m_generating = false;
				m_context->pause = true;
				ImGui::CloseCurrentPopup();
			}

			ImGui::EndPopup();
			return;
		}

		ImGui::PushItemWidth( 10.0f * fontSize );
		ImGui::SliderInt( "Memory budget (MB)", &m_popupBudgetMB, 128, 4096 );
		ImGui::SliderInt( "Min sample interval", &m_popupMinInterval, 8, 60 );
		ImGui::PopItemWidth();

		// Surface a failed open inline so Load can be retried
		if ( m_status[0] != '\0' && strcmp( m_status, "loaded" ) != 0 )
		{
			ImGui::TextColored( ImVec4( 0.85f, 0.30f, 0.30f, 1.0f ), "%s", m_status );
		}

		ImGui::Separator();
		if ( ImGui::Button( "Load" ) )
		{
			// Commit the choices so they persist, build the player under that policy, then start
			// pre-generation. An empty recording has nothing to generate.
			m_context->replayKeyframeBudgetMB = m_popupBudgetMB;
			m_context->replayKeyframeMinInterval = m_popupMinInterval;
			CreatePlayer();
			if ( m_player != nullptr )
			{
				m_generating = m_info.frameCount > 0;
				if ( m_generating == false )
				{
					ImGui::CloseCurrentPopup();
				}
			}
		}
		ImGui::SameLine();
		if ( ImGui::Button( "Cancel" ) )
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	// Advance one recorded step and keep the world pointer current
	void AdvanceOne()
	{
		b2RecPlayer_StepFrame( m_player );
		m_worldId = b2RecPlayer_GetWorldId( m_player );
	}

	void Step() override
	{
		DrawLoadPopup();

		// While the ring builds the world is mid-fast-forward, so hold off drawing it. The popup
		// owns the screen and shows the progress bar until generation finishes.
		if ( m_generating )
		{
			m_stepCount = b2RecPlayer_GetFrame( m_player );
			return;
		}

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
			m_frameAccumulator = 0.0f;
		}
		else if ( m_context->pause == false )
		{
			// Speed scales how many recorded steps pass per display frame. Below 1 advances
			// only every few frames, above 1 advances several.
			m_frameAccumulator += m_speed;
			while ( m_frameAccumulator >= 1.0f )
			{
				m_frameAccumulator -= 1.0f;
				if ( b2RecPlayer_IsAtEnd( m_player ) )
				{
					if ( m_loop )
					{
						b2RecPlayer_Restart( m_player );
						m_worldId = b2RecPlayer_GetWorldId( m_player );
					}
					else
					{
						m_frameAccumulator = 0.0f;
						break;
					}
				}
				AdvanceOne();
			}
		}

		// Keep the base panel "step N" line tracking the replay frame
		m_stepCount = b2RecPlayer_GetFrame( m_player );

		m_context->debugDraw.drawingBounds = GetViewBounds( &m_context->camera );
#if defined( BOX2D_DOUBLE_PRECISION )
		// Draw relative to the camera so callbacks, recorded queries, and ad-hoc draws land near the origin.
		m_context->debugDraw.origin = m_context->camera.center;
		SetDrawOrigin( m_context->draw, m_context->camera.center );
#endif
		if ( B2_IS_NON_NULL( m_worldId ) )
		{
			b2World_Draw( m_worldId, &m_context->debugDraw );
			if ( m_selKind == SelQuery )
			{
				b2RecPlayer_DrawFrameQueries( m_player, &m_context->debugDraw, m_selQuery );
			}
			DrawSelectionHighlight();
		}

		if ( m_context->showUI )
		{
			DrawInspectorPanel();
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
			m_frameAccumulator = 0.0f;
		}
		ImGui::SameLine();
		if ( ImGui::Button( "<" ) )
		{
			b2RecPlayer_SeekFrame( m_player, frame - 1 );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();

		if ( m_context->pause )
		{
			ImGui::PushStyleColor( ImGuiCol_Button, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.6f, 0.6f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.7f, 0.7f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV( 2.0f / 7.0f, 0.8f, 0.8f ) );
			if ( ImGui::Button( "Play " ) )
			{
				m_context->pause = false;
			}
			ImGui::PopStyleColor( 3 );
		}
		else
		{
			ImGui::PushStyleColor( ImGuiCol_Button, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.6f, 0.6f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.7f, 0.7f ) );
			ImGui::PushStyleColor( ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV( 1.0f / 7.0f, 0.8f, 0.8f ) );
			if ( ImGui::Button( "Pause" ) )
			{
				m_context->pause = true;
			}
			ImGui::PopStyleColor( 3 );
		}

		ImGui::SameLine();
		if ( ImGui::Button( ">" ) )
		{
			b2RecPlayer_SeekFrame( m_player, frame + 1 );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
			m_context->pause = true;
		}
		ImGui::SameLine();
		if ( ImGui::Button( ">|" ) )
		{
			b2RecPlayer_SeekFrame( m_player, m_info.frameCount );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
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

		if ( b2RecPlayer_HasDiverged( m_player ) )
		{
			ImGui::TextColored( MakeColor( b2_colorRed ), "****DIVERGED****" );
		}

		ImGui::TextDisabled( "Frame %d / %d%s", b2RecPlayer_GetFrame( m_player ), m_info.frameCount,
							 b2RecPlayer_IsAtEnd( m_player ) ? "  (end)" : "" );

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
			b2Position originA = b2Body_GetPosition( b2Shape_GetBody( contacts[i].shapeIdA ) );
			const b2Manifold* m = &contacts[i].manifold;
			for ( int j = 0; j < m->pointCount; ++j )
			{
				b2Position point = b2OffsetPosition( originA, m->points[j].anchorA );
				DrawWorldPoint( draw, point, 6.0f, b2_colorOrange );
				DrawWorldLine( draw, point, point + b2MulSV( 0.3f, m->normal ), b2_colorOrange );
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
			DrawWorldBounds( draw, b2Shape_GetAABB( shape ), b2_colorYellow );
			DrawWorldTransform( draw, b2Body_GetTransform( body ), 0.5f );
			DrawWorldPoint( draw, b2Body_GetWorldCenter( body ), 8.0f, b2_colorYellow );
			DrawBodyContacts( body );
		}
		else if ( m_selKind == SelBody )
		{
			b2BodyId body = SelectedBody();
			if ( b2Body_IsValid( body ) == false )
			{
				return;
			}
			DrawWorldBounds( draw, b2Body_ComputeAABB( body ), b2_colorYellow );
			DrawWorldTransform( draw, b2Body_GetTransform( body ), 0.5f );
			DrawWorldPoint( draw, b2Body_GetWorldCenter( body ), 8.0f, b2_colorYellow );
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
				DrawWorldPoint( draw, b2Body_GetWorldCenter( a ), 8.0f, b2_colorMagenta );
			}
			if ( b2Body_IsValid( b ) )
			{
				DrawWorldPoint( draw, b2Body_GetWorldCenter( b ), 8.0f, b2_colorMagenta );
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

			bool ownsSelection =
				m_selBodyOrdinal == ord && ( m_selKind == SelBody || m_selKind == SelShape || m_selKind == SelJoint );

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
				snprintf( sl, sizeof( sl ), "Shape %d  %s###b%ds%d", s, ReplayShapeTypeName( b2Shape_GetType( shapes[s] ) ), ord,
						  s );
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
		b2WorldTransform xf = b2Body_GetTransform( body );
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

		ImGui::Text( "id      %d", shape.index1 );
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
			ImGui::Text( "shapes %d / %d", contacts[i].shapeIdA.index1, contacts[i].shapeIdB.index1 );
			ImGui::Text( "normal (%.2f, %.2f)", m->normal.x, m->normal.y );
			ImGui::Text( "points %d", m->pointCount );
			for ( int j = 0; j < m->pointCount; ++j )
			{
				const b2ManifoldPoint* mp = &m->points[j];
				ImGui::Text( "  sep %.3f  Pn %.2g", mp->separation, mp->normalImpulse );
			}

			ImGui::Separator();
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
		ImGui::Text( "File: %s", m_path );
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
		if (ImGui::SliderInt( "Workers", &m_context->workerCount, 0, B2_MAX_WORKERS ))
		{
			if (B2_IS_NON_NULL(m_worldId))
			{
				b2World_SetWorkerCount( m_worldId, m_context->workerCount );
			}
		}
		ImGui::PopItemWidth();

		// Live ring state: the effective spacing widens as the ring evicts under the budget, and the
		// memory held grows as keyframes accumulate. Budget and min interval are chosen in the Load
		// popup and persisted, so there is no live slider here.
		ImGui::TextDisabled( "keyframe spacing %d frames, %.1f MB", b2RecPlayer_GetKeyframeInterval( m_player ),
							 (double)b2RecPlayer_GetKeyframeBytes( m_player ) / ( 1024.0 * 1024.0 ) );

		// Scrubber: full width, seeks both directions
		int scrub = b2RecPlayer_GetFrame( m_player );
		ImGui::PushItemWidth( -1.0f );
		if ( ImGui::SliderInt( "##frame", &scrub, 0, m_info.frameCount ) )
		{
			b2RecPlayer_SeekFrame( m_player, scrub );
			m_worldId = b2RecPlayer_GetWorldId( m_player );
			m_frameAccumulator = 0.0f;
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

		ImGui::EndTabItem();
	}

	// Left click selects a shape to inspect. Picking only reads the world, it never creates the drag
	// joint the base sample does, so the replay is not mutated. Dragging stays disabled.
	void MouseDown( b2Position p, int button, int ) override
	{
		if ( button != GLFW_MOUSE_BUTTON_1 || B2_IS_NULL( m_worldId ) )
		{
			return;
		}

		// A tiny box around the click point, exact at any distance with the click as the origin
		b2Vec2 d = { 0.001f, 0.001f };
		b2AABB box = { b2Neg( d ), d };
		ReplayPickContext pick = { p, b2_nullShapeId };
		b2World_OverlapAABB( m_worldId, p, box, b2DefaultQueryFilter(), ReplayPickCallback, &pick );

		// A miss clears the selection
		SelectShape( pick.shape );
	}

	void MouseUp( b2Position, int ) override
	{
	}

	void MouseMove( b2Position ) override
	{
	}

	static Sample* Create( SampleContext* context )
	{
		return new ReplayViewer( context );
	}

	b2RecPlayer* m_player;
	char m_path[256];
	char m_status[64];

	b2RecPlayerInfo m_info = {};
	float m_speed = 1.0f;
	float m_frameAccumulator = 0.0f;
	bool m_loop = false;
	bool m_selectTimelineTab = true;
	bool m_prevShowMetrics = false;

	// Load popup state. A fresh open configures the keyframe policy here, then the popup switches to
	// a progress bar while every keyframe is generated up front. Temporaries hold the in-popup edits
	// so Cancel leaves the persisted settings untouched.
	bool m_requestLoadPopup = false;
	bool m_generating = false;
	int m_popupBudgetMB = 512;
	int m_popupMinInterval = 16;

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
	int m_selBodyOrdinal = -1;		// index into the player's tracked body list
	int m_selSlot = -1;				// shape or joint slot within that body
	int m_selQuery = -1;			// query index, only meaningful for the current frame
	bool m_revealSelection = false; // one-shot request to expand and scroll the tree to a viewport pick
};

static int sampleReplayViewer = RegisterReplay( "Replay", "Viewer", ReplayViewer::Create );
