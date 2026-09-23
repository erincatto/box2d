// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample.h"

#include "draw.h"
#include "imgui.h"
#include "implot.h"
#include "utils.h"

// consider using https://github.com/skeeto/pdjson
#include "jsmn.h"

#include "box2d/box2d.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <ctype.h>
#include <nfd.h>
#include <stdio.h>
#include <stdlib.h>

#define INFO_PANEL_WIDTH 16.0f

static const char* fileName = "settings.ini";

// Load a file. You must free the character array.
static bool ReadFile( char*& data, int& size, const char* filename )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		return false;
	}

	fseek( file, 0, SEEK_END );
	size = (int)ftell( file );
	fseek( file, 0, SEEK_SET );

	if ( size == 0 )
	{
		return false;
	}

	data = (char*)malloc( size + 1 );
	size_t count = fread( data, size, 1, file );
	(void)count;
	fclose( file );
	data[size] = 0;

	return true;
}

void SampleContext::Save()
{
	FILE* file = fopen( fileName, "w" );
	fprintf( file, "{\n" );
	fprintf( file, "  \"sampleIndex\": %d,\n", sampleIndex );
	fprintf( file, "  \"newUser\": %d,\n", false );
	fprintf( file, "  \"drawShapes\": %s,\n", debugDraw.drawShapes ? "true" : "false" );
	fprintf( file, "  \"showMetrics\": %s,\n", showMetrics ? "true" : "false" );
	fprintf( file, "  \"showProfile\": %s,\n", showProfile ? "true" : "false" );
	fprintf( file, "  \"replayKeyframeBudgetMB\": %d,\n", replayKeyframeBudgetMB );
	fprintf( file, "  \"replayKeyframeMinInterval\": %d\n", replayKeyframeMinInterval );
	fprintf( file, "}\n" );
	fclose( file );
}

static int jsoneq( const char* json, jsmntok_t* tok, const char* s )
{
	if ( tok->type == JSMN_STRING && (int)strlen( s ) == tok->end - tok->start &&
		 strncmp( json + tok->start, s, tok->end - tok->start ) == 0 )
	{
		return 0;
	}
	return -1;
}

void DrawPolygonFcn( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawPolygon( sampleContext->draw, transform, vertices, vertexCount, color );
}

void DrawSolidPolygonFcn( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
						  void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawSolidPolygon( sampleContext->draw, transform, vertices, vertexCount, radius, color );
}

void DrawCircleFcn( b2Pos center, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawCircle( sampleContext->draw, center, radius, color );
}

void DrawSolidCircleFcn( b2WorldTransform transform, b2Vec2 center, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawSolidCircle( sampleContext->draw, transform, center, radius, color );
}

void DrawSolidCapsuleFcn( b2Pos p1, b2Pos p2, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawCapsule( sampleContext->draw, p1, p2, radius, color );
}

void DrawLineFcn( b2Pos p1, b2Pos p2, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawLine( sampleContext->draw, p1, p2, color );
}

void DrawTransformFcn( b2WorldTransform transform, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawTransform( sampleContext->draw, transform, 1.0f );
}

void DrawPointFcn( b2Pos p, float size, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawPoint( sampleContext->draw, p, size, color );
}

void DrawStringFcn( b2Pos p, const char* s, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawString( sampleContext->draw, &sampleContext->camera, p, color, "%s", s );
}

void DrawBoundsFcn( b2AABB aabb, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawBounds( sampleContext->draw, aabb, color );
}

#define MAX_TOKENS 32

void SampleContext::Load()
{
	camera = GetDefaultCamera();
	debugDraw = b2DefaultDebugDraw();
	debugDraw.DrawPolygonFcn = DrawPolygonFcn;
	debugDraw.DrawSolidPolygonFcn = DrawSolidPolygonFcn;
	debugDraw.DrawCircleFcn = DrawCircleFcn;
	debugDraw.DrawSolidCircleFcn = DrawSolidCircleFcn;
	debugDraw.DrawSolidCapsuleFcn = DrawSolidCapsuleFcn;
	debugDraw.DrawLineFcn = DrawLineFcn;
	debugDraw.DrawTransformFcn = DrawTransformFcn;
	debugDraw.DrawPointFcn = DrawPointFcn;
	debugDraw.DrawStringFcn = DrawStringFcn;
	debugDraw.DrawBoundsFcn = DrawBoundsFcn;

	debugDraw.context = this;

	recycleDistance = B2_CONTACT_RECYCLE_DISTANCE;

	char* data = nullptr;
	int size = 0;
	bool found = ReadFile( data, size, fileName );
	if ( found == false )
	{
		return;
	}

	newUser = false;

	jsmn_parser parser;
	jsmntok_t tokens[MAX_TOKENS];

	jsmn_init( &parser );

	// js - pointer to JSON string
	// tokens - an array of tokens available
	// 10 - number of tokens available
	int tokenCount = jsmn_parse( &parser, data, size, tokens, MAX_TOKENS );
	char buffer[32];

	for ( int i = 0; i < tokenCount; ++i )
	{
		if ( jsoneq( data, &tokens[i], "sampleIndex" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			char* dummy;
			sampleIndex = (int)strtol( buffer, &dummy, 10 );
		}
		else if ( jsoneq( data, &tokens[i], "drawShapes" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				debugDraw.drawShapes = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				debugDraw.drawShapes = false;
			}
		}
		else if ( jsoneq( data, &tokens[i], "showMetrics" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				showMetrics = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				showMetrics = false;
			}
		}
		else if ( jsoneq( data, &tokens[i], "showProfile" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				showProfile = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				showProfile = false;
			}
		}
		else if ( jsoneq( data, &tokens[i], "replayKeyframeBudgetMB" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			char* dummy;
			replayKeyframeBudgetMB = (int)strtol( buffer, &dummy, 10 );
		}
		else if ( jsoneq( data, &tokens[i], "replayKeyframeMinInterval" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			char* dummy;
			replayKeyframeMinInterval = (int)strtol( buffer, &dummy, 10 );
		}
	}

	free( data );
}

static void TestMathCpp()
{
	b2Vec2 a = { 1.0f, 2.0f };
	b2Vec2 b = { 3.0f, 4.0f };

	b2Vec2 c = a;
	c += b;
	c -= b;
	c *= 2.0f;
	c = -a;
	c = c + b;
	c = c - a;
	c = 2.0f * a;
	c = a * 2.0f;

	if ( b == a )
	{
		c = a;
	}

	if ( b != a )
	{
		c = b;
	}

	c += c;
}

Sample::Sample( SampleContext* context, bool createWorld )
{
	m_context = context;
	m_camera = &context->camera;
	m_draw = context->draw;

	m_worldId = b2_nullWorldId;

	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;
	m_didStep = false;
	m_screenTextX = 5.0f;
	m_screenTextY = 0.0f;

	m_mouseBodyId = b2_nullBodyId;
	m_mousePoint = {};
	m_mouseForceScale = 100.0f;

	memset( m_profiles, 0, sizeof( m_profiles ) );
	m_currentProfileIndex = 0;
	m_profileReadIndex = 0;
	m_profileWriteIndex = 0;

	g_randomSeed = RAND_SEED;

	m_recording = nullptr;
	m_recordStartStep = 0;

	if ( createWorld )
	{
		CreateWorld();
	}
	TestMathCpp();
}

Sample::~Sample()
{
	if ( B2_IS_NON_NULL( m_worldId ) )
	{
		FinishRecording();
		b2DestroyWorld( m_worldId );
	}
}

void Sample::StartRecording()
{
	if ( m_recording != nullptr )
	{
		return;
	}

	uint64_t ticks = b2GetTicks();

	// Snapshots the live world as the seed, so recording can begin at any step boundary
	m_recording = b2CreateRecording( 0 );
	b2World_StartRecording( m_worldId, m_recording );
	m_recordStartStep = m_stepCount;

	float ms = b2GetMilliseconds( ticks );
	printf( "b2World_StartRecording took : %g ms", ms );
}

void Sample::FinishRecording()
{
	if ( m_recording == nullptr )
	{
		return;
	}

	b2World_StopRecording( m_worldId );
	if ( b2SaveRecordingToFile( m_recording, m_context->recordingFile ) )
	{
		snprintf( m_context->savedRecordingFile, sizeof( m_context->savedRecordingFile ), "%s", m_context->recordingFile );
	}
	b2DestroyRecording( m_recording );
	m_recording = nullptr;
}

void Sample::CreateWorld()
{
	if ( B2_IS_NON_NULL( m_worldId ) )
	{
		FinishRecording();
		b2DestroyWorld( m_worldId );
		m_worldId = b2_nullWorldId;
	}

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = m_context->workerCount;
	worldDef.userTaskContext = this;
	worldDef.enableSleep = m_context->enableSleep;
	worldDef.capacity = m_context->capacity;
	m_worldId = b2CreateWorld( &worldDef );

	b2World_SetContactRecycleDistance( m_worldId, m_context->recycleDistance );
}

void Sample::ResetText()
{
	float fontSize = ImGui::GetFontSize();
	if ( m_context->showUI )
	{
		m_screenTextY = ImGui::GetFrameHeight() + 1.5f * fontSize;
	}
	else
	{
		m_screenTextY = 3.0f * fontSize;
	}

	m_screenTextX = 5.0f;
	if ( IsProfileVisible() )
	{
		m_screenTextX += GetProfilePanelWidth() + 0.5f * fontSize;
	}
}

struct QueryContext
{
	b2Pos point;
	b2BodyId bodyId = b2_nullBodyId;
};

bool QueryCallback( b2ShapeId shapeId, void* context )
{
	QueryContext* queryContext = static_cast<QueryContext*>( context );

	b2BodyId bodyId = b2Shape_GetBody( shapeId );
	b2BodyType bodyType = b2Body_GetType( bodyId );
	if ( bodyType != b2_dynamicBody )
	{
		// continue query
		return true;
	}

	bool overlap = b2Shape_TestPoint( shapeId, queryContext->point );
	if ( overlap )
	{
		// found shape
		queryContext->bodyId = bodyId;
		return false;
	}

	return true;
}

void Sample::MouseDown( b2Pos p, int button, int mod )
{
	if ( B2_IS_NON_NULL( m_mouseJointId ) )
	{
		return;
	}

	if ( button == GLFW_MOUSE_BUTTON_1 )
	{
		// A tiny box around the click point, exact at any distance with the click as the origin
		b2Vec2 d = { 0.001f, 0.001f };
		b2AABB box = { b2Neg( d ), d };

		m_mousePoint = p;

		// Query the world for overlapping shapes.
		QueryContext queryContext = { p, b2_nullBodyId };
		b2World_OverlapAABB( m_worldId, p, box, b2DefaultQueryFilter(), QueryCallback, &queryContext );

		if ( B2_IS_NON_NULL( queryContext.bodyId ) )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = m_mousePoint;
			bodyDef.enableSleep = false;
			m_mouseBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2MotorJointDef jointDef = b2DefaultMotorJointDef();
			jointDef.base.bodyIdA = m_mouseBodyId;
			jointDef.base.bodyIdB = queryContext.bodyId;
			jointDef.base.localFrameB.p = b2Body_GetLocalPoint( queryContext.bodyId, p );
			jointDef.linearHertz = 7.5f;
			jointDef.linearDampingRatio = 1.0f;

			b2MassData massData = b2Body_GetMassData( queryContext.bodyId );
			float g = b2Length( b2World_GetGravity( m_worldId ) );
			float mg = massData.mass * g;

			jointDef.maxSpringForce = m_mouseForceScale * mg;

			if ( massData.mass > 0.0f )
			{
				// This acts like angular friction
				float lever = sqrtf( massData.rotationalInertia / massData.mass );
				jointDef.maxVelocityTorque = 0.25f * lever * mg;
			}

			m_mouseJointId = b2CreateMotorJoint( m_worldId, &jointDef );
		}
	}
}

void Sample::MouseUp( b2Pos p, int button )
{
	if ( B2_IS_NON_NULL( m_mouseJointId ) && button == GLFW_MOUSE_BUTTON_1 )
	{
		b2DestroyJoint( m_mouseJointId );
		m_mouseJointId = b2_nullJointId;

		b2DestroyBody( m_mouseBodyId );
		m_mouseBodyId = b2_nullBodyId;
	}
}

void Sample::MouseMove( b2Pos p )
{
	if ( b2Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = b2_nullJointId;
	}

	m_mousePoint = p;
}

void Sample::DrawScreenTextLine( const char* text, ... )
{
	char buffer[256];
	va_list arg;
	va_start( arg, text );
	vsnprintf( buffer, sizeof( buffer ), text, arg );
	va_end( arg );
	buffer[sizeof( buffer ) - 1] = 0;
	DrawScreenString( m_draw, m_screenTextX, m_screenTextY, b2_colorWhite, "%s", buffer );
	m_screenTextY += 1.5f * ImGui::GetFontSize();
}

float Sample::InfoPanelWidthEm() const
{
	return INFO_PANEL_WIDTH;
}

void Sample::FocusHome()
{
	m_context->camera.center = m_context->homeCenter;
	m_context->camera.zoom = m_context->homeZoom;
}

void Sample::ResetProfile()
{
	// Keeps the elapsed recording length intact across a profile reset
	m_recordStartStep -= m_stepCount;
	m_stepCount = 0;
	memset( m_profiles, 0, sizeof( m_profiles ) );
	m_currentProfileIndex = 0;
	m_profileReadIndex = 0;
	m_profileWriteIndex = 0;
}

void Sample::Step()
{
	m_didStep = false;

	float timeStep = 0.0f;
	if ( m_context->pause == false || m_context->singleStep > 0 )
	{
		timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		m_context->singleStep = b2MaxInt( 0, m_context->singleStep - 1 );
	}

	if ( B2_IS_NON_NULL( m_mouseJointId ) && b2Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = b2_nullJointId;

		if ( B2_IS_NON_NULL( m_mouseBodyId ) )
		{
			b2DestroyBody( m_mouseBodyId );
			m_mouseBodyId = b2_nullBodyId;
		}
	}

	if ( B2_IS_NON_NULL( m_mouseBodyId ) && timeStep > 0.0f )
	{
		bool wake = true;
		b2Body_SetTargetTransform( m_mouseBodyId, { m_mousePoint, b2Rot_identity }, timeStep, wake );
	}

	m_context->debugDraw.drawingBounds = GetViewBounds( &m_context->camera );

	b2World_EnableSleeping( m_worldId, m_context->enableSleep );
	b2World_EnableWarmStarting( m_worldId, m_context->enableWarmStarting );
	b2World_EnableContinuous( m_worldId, m_context->enableContinuous );
	b2World_SetRestitutionIterations( m_worldId, m_context->restitutionIterations );
	b2World_EnableRestitutionPropagation( m_worldId, m_context->enableRestitutionPropagation );

	for ( int i = 0; i < 1; ++i )
	{
		b2World_Step( m_worldId, timeStep, m_context->subStepCount );
	}

	b2World_Draw( m_worldId, &m_context->debugDraw );

	if ( timeStep > 0.0f )
	{
		m_stepCount += 1;
		m_didStep = true;

		if ( m_profileWriteIndex == m_profileCapacity + m_profileReadIndex )
		{
			m_profileReadIndex += 1;
		}

		m_currentProfileIndex = static_cast<int>( m_profileWriteIndex & ( m_profileCapacity - 1 ) );
		m_profiles[m_currentProfileIndex] = b2World_GetProfile( m_worldId );

		m_profileWriteIndex += 1;
	}
}

struct RowDef
{
	const char* name;
	int indent;
	ImU32 color;
};

float AddSegment( ImDrawList* dl, float availWidth, float t, float stepNow, ImU32 col, float x, ImVec2 cursor, float barHeight )
{
	float w = availWidth * ( t / stepNow );
	if ( w > 0.0f )
	{
		dl->AddRectFilled( ImVec2( x, cursor.y ), ImVec2( x + w, cursor.y + barHeight ), col );
		x += w;
	}

	return x;
}

#define PROFILE_SECTION_WIDTH 8.0f
#define PROFILE_INDENT_WIDTH 0.75f
#define PROFILE_VALUE_WIDTH 3.0f
#define PROFILE_BAR_WIDTH 4.0f
#define PROFILE_COLUMN_COUNT 5

// Shared by the profile panel and the frame time plot so the two views read alike
static const ImU32 s_colorStep = IM_COL32( 230, 230, 230, 255 );
static const ImU32 s_colorPairs = IM_COL32( 102, 153, 255, 255 );
static const ImU32 s_colorCollide = IM_COL32( 255, 140, 51, 255 );
static const ImU32 s_colorSolve = IM_COL32( 102, 204, 102, 255 );
static const ImU32 s_colorSolveChild = IM_COL32( 150, 190, 150, 255 );
static const ImU32 s_colorConstraintChild = IM_COL32( 110, 200, 190, 255 );
static const ImU32 s_colorSensors = IM_COL32( 200, 120, 220, 255 );
static const ImU32 s_colorOther = IM_COL32( 140, 140, 140, 255 );

// Well apart from both row background shades so the track reads the same on every row
static const ImU32 s_colorTrack = IM_COL32( 70, 70, 74, 255 );

bool Sample::IsProfileVisible() const
{
	return m_context->showUI && m_context->showProfile && HasProfile();
}

float Sample::GetProfilePanelWidth() const
{
	if ( IsProfileVisible() == false )
	{
		return 0.0f;
	}

	const ImGuiStyle& style = ImGui::GetStyle();
	float fontSize = ImGui::GetFontSize();
	float width = PROFILE_SECTION_WIDTH + 3.0f * PROFILE_VALUE_WIDTH + PROFILE_BAR_WIDTH;
	return width * fontSize + 2.0f * PROFILE_COLUMN_COUNT * style.CellPadding.x + 2.0f * style.WindowPadding.x;
}

void Sample::DrawProfile()
{
	if ( IsProfileVisible() == false )
	{
		return;
	}

	float fontSize = ImGui::GetFontSize();
	float menuBarHeight = ImGui::GetFrameHeight();
	float panelWidth = GetProfilePanelWidth();

	ImGui::SetNextWindowPos( { 0.5f * fontSize, menuBarHeight + 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { panelWidth, m_camera->height - menuBarHeight - fontSize } );

	ImGui::Begin( "Profile", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	const int count = static_cast<int>( m_profileWriteIndex - m_profileReadIndex );

	constexpr int kRowCount = 21;
	float histories[kRowCount][m_profileCapacity];
	float totals[kRowCount] = {};
	for ( int i = 0; i < count; ++i )
	{
		int idx = static_cast<int>( ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 ) );
		const b2Profile& p = m_profiles[idx];
		histories[0][i] = p.step;
		histories[1][i] = p.pairs;
		histories[2][i] = p.collide;
		histories[3][i] = p.solve;
		histories[4][i] = p.solverSetup;
		histories[5][i] = p.constraints;
		histories[6][i] = p.prepareConstraints;
		histories[7][i] = p.integrateVelocities;
		histories[8][i] = p.warmStart;
		histories[9][i] = p.solveImpulses;
		histories[10][i] = p.integratePositions;
		histories[11][i] = p.relaxImpulses;
		histories[12][i] = p.storeImpulses;
		histories[13][i] = p.splitIslands;
		histories[14][i] = p.transforms;
		histories[15][i] = p.jointEvents;
		histories[16][i] = p.hitEvents;
		histories[17][i] = p.refit;
		histories[18][i] = p.sleepIslands;
		histories[19][i] = p.bullets;
		histories[20][i] = p.sensors;
		for ( int j = 0; j < kRowCount; ++j )
		{
			totals[j] += histories[j][i];
		}
	}

	// "now" smoothed over the last few frames so bars don't jitter visibly.
	constexpr int kNowWindow = 10;
	float now[kRowCount] = {};
	{
		int n = count < kNowWindow ? count : kNowWindow;
		if ( n > 0 )
		{
			float inv = 1.0f / n;
			for ( int r = 0; r < kRowCount; ++r )
			{
				float sum = 0.0f;
				for ( int i = count - n; i < count; ++i )
				{
					sum += histories[r][i];
				}
				now[r] = sum * inv;
			}
		}
	}

	float avg[kRowCount] = {};
	if ( count > 0 )
	{
		float scale = 1.0f / count;
		for ( int i = 0; i < kRowCount; ++i )
		{
			avg[i] = scale * totals[i];
		}
	}

	float rowMax[kRowCount] = {};
	for ( int r = 0; r < kRowCount; ++r )
	{
		for ( int i = 0; i < count; ++i )
		{
			if ( histories[r][i] > rowMax[r] )
			{
				rowMax[r] = histories[r][i];
			}
		}
	}

	const RowDef rows[kRowCount] = {
		{ "step", 0, s_colorStep },			 { "pairs", 0, s_colorPairs },
		{ "collide", 0, s_colorCollide },	 { "solve", 0, s_colorSolve },
		{ "setup", 1, s_colorSolveChild },	 { "constraints", 1, s_colorSolveChild },
		{ "prepare", 2, s_colorConstraintChild }, { "velocities", 2, s_colorConstraintChild },
		{ "warm start", 2, s_colorConstraintChild }, { "bias", 2, s_colorConstraintChild },
		{ "positions", 2, s_colorConstraintChild }, { "relax", 2, s_colorConstraintChild },
		{ "store", 2, s_colorConstraintChild },	 { "split", 2, s_colorConstraintChild },
		{ "transforms", 1, s_colorSolveChild }, { "joint events", 1, s_colorSolveChild },
		{ "hit events", 1, s_colorSolveChild }, { "refit BVH", 1, s_colorSolveChild },
		{ "sleep", 1, s_colorSolveChild },	 { "bullets", 1, s_colorSolveChild },
		{ "sensors", 0, s_colorSensors },
	};

	float stepNow = b2MaxFloat( now[0], 0.001f );

	if ( ImGui::Button( "Reset" ) )
	{
		ResetProfile();
	}
	ImGui::SameLine();
	ImGui::Text( " step %.2f ms", now[0] );

	// Flame strip: step subdivided by top-level children.
	{
		float pairsT = now[1];
		float collideT = now[2];
		float solveT = now[3];
		float sensorsT = now[20];
		float otherT = b2MaxFloat( stepNow - pairsT - collideT - solveT - sensorsT, 0.0f );

		float availWidth = ImGui::GetContentRegionAvail().x;
		float barHeight = 1.5f * fontSize;
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 cursor = ImGui::GetCursorScreenPos();
		float x = cursor.x;

		x = AddSegment( dl, availWidth, pairsT, stepNow, s_colorPairs, x, cursor, barHeight );
		x = AddSegment( dl, availWidth, collideT, stepNow, s_colorCollide, x, cursor, barHeight );
		x = AddSegment( dl, availWidth, solveT, stepNow, s_colorSolve, x, cursor, barHeight );
		x = AddSegment( dl, availWidth, sensorsT, stepNow, s_colorSensors, x, cursor, barHeight );
		x = AddSegment( dl, availWidth, otherT, stepNow, s_colorOther, x, cursor, barHeight );

		ImGui::Dummy( ImVec2( availWidth, barHeight ) );
	}

	const ImGuiTableFlags tableFlags =
		ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;

	if ( ImGui::BeginTable( "profile", PROFILE_COLUMN_COUNT, tableFlags ) )
	{
		ImGui::TableSetupColumn( "section", ImGuiTableColumnFlags_WidthFixed, PROFILE_SECTION_WIDTH * fontSize );
		ImGui::TableSetupColumn( "now", ImGuiTableColumnFlags_WidthFixed, PROFILE_VALUE_WIDTH * fontSize );
		ImGui::TableSetupColumn( "avg", ImGuiTableColumnFlags_WidthFixed, PROFILE_VALUE_WIDTH * fontSize );
		ImGui::TableSetupColumn( "max", ImGuiTableColumnFlags_WidthFixed, PROFILE_VALUE_WIDTH * fontSize );
		ImGui::TableSetupColumn( "%", ImGuiTableColumnFlags_WidthFixed, PROFILE_BAR_WIDTH * fontSize );
		ImGui::TableHeadersRow();

		for ( int r = 0; r < kRowCount; ++r )
		{
			const RowDef& d = rows[r];

			ImGui::TableNextRow();

			ImGui::TableNextColumn();
			float indent = d.indent * PROFILE_INDENT_WIDTH * fontSize;
			ImGui::SetCursorPosX( ImGui::GetCursorPosX() + indent );
			ImGui::PushStyleColor( ImGuiCol_Text, d.color );
			ImGui::TextUnformatted( d.name );
			ImGui::PopStyleColor();

			// Dim rows that would print as zero so the costly sections stand out
			bool idle = avg[r] < 0.005f;
			if ( idle )
			{
				ImGui::PushStyleColor( ImGuiCol_Text, ImGui::GetStyleColorVec4( ImGuiCol_TextDisabled ) );
			}
			ImGui::TableNextColumn();
			ImGui::Text( "%6.2f", now[r] );
			ImGui::TableNextColumn();
			ImGui::Text( "%6.2f", avg[r] );
			ImGui::TableNextColumn();
			ImGui::Text( "%6.2f", rowMax[r] );
			if ( idle )
			{
				ImGui::PopStyleColor();
			}

			ImGui::TableNextColumn();

			// The step itself is measured against the frame budget to show headroom
			bool isBudget = r == 0 && m_context->hertz > 0.0f;
			float reference = isBudget ? 1000.0f / m_context->hertz : stepNow;
			float frac = b2ClampFloat( now[r] / reference, 0.0f, 1.0f );
			// Text line height instead of a framed progress bar keeps the rows tight
			float barWidth = ImGui::GetContentRegionAvail().x;
			float barHeight = ImGui::GetTextLineHeight();
			ImVec2 p = ImGui::GetCursorScreenPos();
			ImDrawList* drawList = ImGui::GetWindowDrawList();
			drawList->AddRectFilled( p, ImVec2( p.x + barWidth, p.y + barHeight ), s_colorTrack );
			drawList->AddRectFilled( p, ImVec2( p.x + frac * barWidth, p.y + barHeight ), d.color );
			ImGui::Dummy( ImVec2( barWidth, barHeight ) );
			if ( isBudget )
			{
				ImGui::SetItemTooltip( "%.0f%% of %.1f ms frame", 100.0f * now[r] / reference, reference );
			}
			else
			{
				ImGui::SetItemTooltip( "%.0f%% of step", 100.0f * now[r] / reference );
			}
		}
		ImGui::EndTable();
	}

	ImGui::End();
}

void Sample::DrawMetrics()
{
	if ( m_context->showMetrics == false )
	{
		return;
	}

	float fontSize = ImGui::GetFontSize();
	float menuWidth = InfoPanelWidthEm() * fontSize;
	float drawerHeight = 16.0f * fontSize;
	float drawerX = 0.5f * fontSize;
	if ( IsProfileVisible() )
	{
		drawerX += GetProfilePanelWidth() + 0.5f * fontSize;
	}
	float drawerWidth = m_camera->width - menuWidth - fontSize - drawerX;

	ImGui::SetNextWindowPos( { drawerX, m_camera->height - drawerHeight - 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { drawerWidth, drawerHeight } );

	ImGui::Begin( "Metrics", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	if ( ImGui::BeginTabBar( "MetricsTabs", ImGuiTabBarFlags_None ) )
	{
		if ( HasProfile() && ImGui::BeginTabItem( "Frame Time" ) )
		{
			float maxValue = 0.0f;
			float times[m_profileCapacity];
			float stepTimes[m_profileCapacity];
			float pairsTimes[m_profileCapacity];
			float collideTimes[m_profileCapacity];
			float solveTimes[m_profileCapacity];
			float sensorsTimes[m_profileCapacity];
			int count = static_cast<int>( m_profileWriteIndex - m_profileReadIndex );
			for ( int i = 0; i < count; ++i )
			{
				int index = ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 );
				times[i] = i / 60.0f;
				stepTimes[i] = m_profiles[index].step;
				pairsTimes[i] = m_profiles[index].pairs;
				collideTimes[i] = m_profiles[index].collide;
				solveTimes[i] = m_profiles[index].solve;
				sensorsTimes[i] = m_profiles[index].sensors;
				maxValue = b2MaxFloat( stepTimes[i], maxValue );
			}

			// ImPlot picks a 5 second major interval at this width, leaving one bright line among the half seconds
			ImPlot::PushStyleVar( ImPlotStyleVar_MinorAlpha, 1.0f );
			ImPlot::PushStyleColor( ImPlotCol_AxisGrid, IM_COL32( 255, 255, 255, 15 ) );

			ImVec2 plotSize = ImGui::GetContentRegionAvail();
			if ( ImPlot::BeginPlot( "Profile", plotSize, ImPlotFlags_NoTitle ) )
			{
				ImPlot::SetupAxes( nullptr, "ms" );
				ImPlot::SetupAxesLimits( 0, m_profileCapacity / 60.0, 0.0, maxValue, ImPlotCond_Always );
				ImPlot::SetNextLineStyle( ImGui::ColorConvertU32ToFloat4( s_colorStep ) );
				ImPlot::PlotLine( "step", times, stepTimes, count );
				ImPlot::SetNextLineStyle( ImGui::ColorConvertU32ToFloat4( s_colorPairs ) );
				ImPlot::PlotLine( "pairs", times, pairsTimes, count );
				ImPlot::SetNextLineStyle( ImGui::ColorConvertU32ToFloat4( s_colorCollide ) );
				ImPlot::PlotLine( "collide", times, collideTimes, count );
				ImPlot::SetNextLineStyle( ImGui::ColorConvertU32ToFloat4( s_colorSolve ) );
				ImPlot::PlotLine( "solve", times, solveTimes, count );
				ImPlot::SetNextLineStyle( ImGui::ColorConvertU32ToFloat4( s_colorSensors ) );
				ImPlot::PlotLine( "sensors", times, sensorsTimes, count );
				ImPlot::EndPlot();
			}

			ImPlot::PopStyleColor();
			ImPlot::PopStyleVar();

			ImGui::EndTabItem();
		}

		if ( ImGui::BeginTabItem( "Counters" ) )
		{
			if ( b2World_IsValid( m_worldId ) == false )
			{
				ImGui::TextDisabled( "No world" );
			}
			else
			{
				b2Counters s = b2World_GetCounters( m_worldId );
				b2Capacity c = b2World_GetMaxCapacity( m_worldId );
				constexpr int colorCount = sizeof( s.colorCounts ) / sizeof( s.colorCounts[0] );
				const int overflowIndex = colorCount - 1;

				if ( ImGui::BeginTable( "counters_layout", 3, ImGuiTableFlags_SizingFixedFit ) )
				{
					ImGui::TableSetupColumn( "world", ImGuiTableColumnFlags_WidthFixed, 14.0f * fontSize );
					ImGui::TableSetupColumn( "memory", ImGuiTableColumnFlags_WidthFixed, 11.0f * fontSize );
					ImGui::TableSetupColumn( "solver", ImGuiTableColumnFlags_WidthStretch );
					ImGui::TableNextRow();
					ImGui::TableNextColumn();

					ImGui::Text( "bodies   %d / %d", s.bodyCount, c.staticBodyCount + c.dynamicBodyCount );
					ImGui::Text( "shapes   %d / %d", s.shapeCount, c.staticShapeCount + c.dynamicShapeCount );
					ImGui::Text( "contacts %d / %d", s.contactCount, c.contactCount );
					ImGui::Text( "joints   %d", s.jointCount );
					ImGui::Text( "islands  %d", s.islandCount );
					ImGui::Text( "tasks    %d", s.taskCount );

					ImGui::TableNextColumn();

					ImGui::Text( "static tree  %d", s.staticTreeHeight );
					ImGui::Text( "movable tree %d", s.treeHeight );
					ImGui::Text( "alloc %lld K", (long long)( s.byteCount / 1024 ) );
					ImGui::Text( "stack %d K", s.stackUsed / 1024 );

					ImGui::TableNextColumn();

					int totalCount = 0;
					int normalCount = 0;
					for ( int i = 0; i < colorCount; ++i )
					{
						totalCount += s.colorCounts[i];
						if ( i != overflowIndex )
						{
							normalCount += s.colorCounts[i];
						}
					}
					int overflowCount = s.colorCounts[overflowIndex];

					ImGui::Text( "%d constraints across %d colors", totalCount, colorCount - 1 );

					float availWidth = ImGui::GetContentRegionAvail().x;
					float barHeight = 2.0f * fontSize;
					ImDrawList* dl = ImGui::GetWindowDrawList();

					ImVec2 cursor = ImGui::GetCursorScreenPos();
					dl->AddRectFilled( cursor, ImVec2( cursor.x + availWidth, cursor.y + barHeight ), IM_COL32( 40, 40, 40, 255 ) );
					if ( normalCount > 0 )
					{
						float x = cursor.x;
						const float invTotal = 1.0f / (float)normalCount;
						for ( int i = 0; i < overflowIndex; ++i )
						{
							int cnt = s.colorCounts[i];
							if ( cnt == 0 )
							{
								continue;
							}
							float segW = availWidth * cnt * invTotal;
							uint32_t hex = static_cast<uint32_t>( b2GetGraphColor( i ) );
							ImU32 col = IM_COL32( ( hex >> 16 ) & 0xFF, ( hex >> 8 ) & 0xFF, hex & 0xFF, 255 );
							dl->AddRectFilled( ImVec2( x, cursor.y ), ImVec2( x + segW, cursor.y + barHeight ), col );
							x += segW;
						}
					}
					ImGui::Dummy( ImVec2( availWidth, barHeight ) );

					ImGui::Spacing();
					float overflowFrac = totalCount > 0 ? (float)overflowCount / (float)totalCount : 0.0f;
					char overflowOverlay[32];
					snprintf( overflowOverlay, sizeof( overflowOverlay ), "overflow %d", overflowCount );
					ImGui::PushStyleColor( ImGuiCol_PlotHistogram, IM_COL32( 220, 60, 60, 255 ) );
					ImGui::ProgressBar( overflowFrac, ImVec2( -FLT_MIN, 0.0f ), overflowOverlay );
					ImGui::PopStyleColor();

					float recycledFrac =
						s.awakeContactCount > 0
							? b2ClampFloat( (float)s.recycledContactCount / (float)s.awakeContactCount, 0.0f, 1.0f )
							: 0.0f;
					char recycledOverlay[48];
					snprintf( recycledOverlay, sizeof( recycledOverlay ), "recycled %d / %d", s.recycledContactCount,
							  s.awakeContactCount );
					ImGui::ProgressBar( recycledFrac, ImVec2( -FLT_MIN, 0.0f ), recycledOverlay );

					ImGui::EndTable();
				}
			}

			ImGui::EndTabItem();
		}

		DrawMetricsTab();

		ImGui::EndTabBar();
	}

	ImGui::End();
}

void Sample::DrawHud( float frameTime )
{
	const SampleEntry& entry = g_sampleEntries[m_context->sampleIndex];
	float fontSize = ImGui::GetFontSize();

	DrawScreenString( m_context->draw, 5.0f, 1.5f * fontSize, b2_colorYellow, "%s : %s", entry.category, entry.name );
	DrawScreenString( m_context->draw, 5.0f, 3.0f * fontSize, b2_colorForestGreen, "Press TAB to show UI" );
	m_screenTextY += 1.5f * fontSize;

	if ( m_context->pause )
	{
		DrawScreenString( m_context->draw, 5.0f, 4.5f * fontSize, b2_colorRed, "****PAUSED****" );
		m_screenTextY += 1.5f * fontSize;
	}

	DrawScreenString( m_context->draw, 5.0f, m_camera->height - 0.5f * fontSize, b2_colorSeaGreen, "%.1f ms  step %d",
					  1000.0f * frameTime, m_stepCount );
}

// Parse an SVG path element with only straight lines. Removes the loop point.
// Example:
// "M 47.625004,185.20833 H 161.39585 l 29.10417,-2.64583 26.45834,-7.9375 26.45833,-13.22917 23.81251,-21.16666 h "
// "13.22916 v 44.97916 H 592.66669 V 0 h 21.16671 v 206.375 l -566.208398,-1e-5 z"
int Sample::ParsePath( const char* svgPath, b2Vec2 offset, b2Vec2* points, int capacity, float scale, bool reverseOrder )
{
	int pointCount = 0;
	b2Vec2 currentPoint = {};
	const char* ptr = svgPath;
	char command = *ptr;

	while ( *ptr != '\0' )
	{
		if ( isdigit( *ptr ) == 0 && *ptr != '-' )
		{
			// note: command can be implicitly repeated
			command = *ptr;

			if ( command == 'M' || command == 'L' || command == 'H' || command == 'V' || command == 'm' || command == 'l' ||
				 command == 'h' || command == 'v' )
			{
				ptr += 2; // Skip the command character and space
			}

			if ( command == 'z' )
			{
				break;
			}
		}

		assert( isdigit( *ptr ) != 0 || *ptr == '-' );

		float x = 0.0f, y = 0.0f;
		switch ( command )
		{
			case 'M':
			case 'L':
				if ( sscanf( ptr, "%f,%f", &x, &y ) == 2 )
				{
					currentPoint.x = x;
					currentPoint.y = y;
				}
				else
				{
					assert( false );
				}
				break;
			case 'H':
				if ( sscanf( ptr, "%f", &x ) == 1 )
				{
					currentPoint.x = x;
				}
				else
				{
					assert( false );
				}
				break;
			case 'V':
				if ( sscanf( ptr, "%f", &y ) == 1 )
				{
					currentPoint.y = y;
				}
				else
				{
					assert( false );
				}
				break;
			case 'm':
			case 'l':
				if ( sscanf( ptr, "%f,%f", &x, &y ) == 2 )
				{
					currentPoint.x += x;
					currentPoint.y += y;
				}
				else
				{
					assert( false );
				}
				break;
			case 'h':
				if ( sscanf( ptr, "%f", &x ) == 1 )
				{
					currentPoint.x += x;
				}
				else
				{
					assert( false );
				}
				break;
			case 'v':
				if ( sscanf( ptr, "%f", &y ) == 1 )
				{
					currentPoint.y += y;
				}
				else
				{
					assert( false );
				}
				break;

			default:
				assert( false );
				break;
		}

		points[pointCount] = { scale * ( currentPoint.x + offset.x ), -scale * ( currentPoint.y + offset.y ) };
		pointCount += 1;
		if ( pointCount == capacity )
		{
			break;
		}

		// Move to the next space or end of string
		while ( *ptr != '\0' && isspace( *ptr ) == 0 )
		{
			ptr++;
		}

		// Skip contiguous spaces
		while ( isspace( *ptr ) )
		{
			ptr++;
		}

		ptr += 0;
	}

	if ( pointCount == 0 )
	{
		return 0;
	}

	if ( reverseOrder )
	{
		// todo
	}

	// Remove the loop point for chain shapes.
	if ( pointCount > 2 && b2Distance( points[0], points[pointCount - 1] ) <= B2_LINEAR_SLOP)
	{
		pointCount -= 1;
	}

	return pointCount;
}

// Case-insensitive subsequence match. Returns >=0 score on match, -1 on no match.
// Empty needle returns 0 so an empty query lets all samples through with a neutral score.
static int FuzzyScore( const char* needle, const char* haystack )
{
	if ( needle == nullptr || needle[0] == '\0' )
	{
		return 0;
	}

	int score = 0;
	int hi = 0;
	int prevMatchHi = -2;

	for ( int ni = 0; needle[ni] != '\0'; ++ni )
	{
		int nc = tolower( (unsigned char)needle[ni] );
		while ( haystack[hi] != '\0' && tolower( (unsigned char)haystack[hi] ) != nc )
		{
			++hi;
		}
		if ( haystack[hi] == '\0' )
		{
			return -1;
		}

		int bonus = 1;
		if ( hi == 0 )
		{
			bonus += 10; // prefix match
		}
		else if ( !isalnum( (unsigned char)haystack[hi - 1] ) )
		{
			bonus += 5; // word-start (after _, space, etc.)
		}
		if ( hi == prevMatchHi + 1 )
		{
			bonus += 3; // contiguous run
		}

		score += bonus;
		prevMatchHi = hi;
		++hi;
	}

	return score;
}

SampleEntry g_sampleEntries[MAX_SAMPLES] = {};
int g_sampleCount = 0;
int g_replayIndex = -1;

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn )
{
	int index = g_sampleCount;
	if ( index < MAX_SAMPLES )
	{
		g_sampleEntries[index] = { category, name, fcn, nullptr };
		++g_sampleCount;
		return index;
	}

	return -1;
}

int RegisterSampleWithCapacity( const char* category, const char* name, SampleCreateFcn* fcn, SampleCapacityFcn* capacityFcn )
{
	int index = g_sampleCount;
	if ( index < MAX_SAMPLES )
	{
		g_sampleEntries[index] = { category, name, fcn, capacityFcn };
		++g_sampleCount;
		return index;
	}

	return -1;
}

int RegisterReplay( const char* category, const char* name, SampleCreateFcn* fcn )
{
	int index = g_sampleCount;
	if ( index < MAX_SAMPLES )
	{
		g_sampleEntries[index] = { category, name, fcn, nullptr };
		g_replayIndex = index;
		++g_sampleCount;
		return index;
	}

	return -1;
}

void SelectSample( SampleContext* context, int selection, bool restart )
{
	if ( restart == false )
	{
		ResetView( &context->camera );
		context->sampleIndex = selection;
		context->pause = false;
		context->subStepCount = 4;
		context->restitutionIterations = 2;
		context->enableRestitutionPropagation = false;
		context->debugDraw.drawJoints = true;
	}

	// Steps queued in a sample that never consumes them must not play out in the next one
	context->singleStep = 0;

	delete context->sample;
	context->sample = nullptr;
	if ( g_sampleEntries[context->sampleIndex].capacityFcn != nullptr )
	{
		context->capacity = g_sampleEntries[context->sampleIndex].capacityFcn();
	}
	else
	{
		context->capacity = b2DefaultWorldDef().capacity;
	}
	context->restart = restart;
	context->sample = g_sampleEntries[context->sampleIndex].createFcn( context );
	context->restart = false;

	// A restart keeps the camera where it was, so the original starting view stays home
	if ( restart == false )
	{
		context->homeCenter = context->camera.center;
		context->homeZoom = context->camera.zoom;
	}
}

static void DrawRow( const char* key, const char* desc )
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex( 0 );
	ImGui::TextUnformatted( key );
	ImGui::TableSetColumnIndex( 1 );
	ImGui::TextUnformatted( desc );
}

static void DrawMenuBar( SampleContext* context )
{
	float fontSize = ImGui::GetFontSize();

	if ( ImGui::BeginMainMenuBar() )
	{
		if ( ImGui::BeginMenu( "Sim" ) )
		{
			ImGui::MenuItem( "Pause", "P", &context->pause );
			if ( ImGui::MenuItem( "Single Step", "." ) )
			{
				context->singleStep += 1;
			}
			if ( ImGui::MenuItem( "Restart", "R" ) )
			{
				SelectSample( context, context->sampleIndex, true );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Previous Sample", "[" ) )
			{
				int selection = context->sampleIndex - 1;
				if ( selection < 0 )
				{
					selection = g_sampleCount - 1;
				}
				SelectSample( context, selection, false );
			}
			if ( ImGui::MenuItem( "Next Sample", "]" ) )
			{
				int selection = context->sampleIndex + 1;
				if ( selection == g_sampleCount )
				{
					selection = 0;
				}
				SelectSample( context, selection, false );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Reset Profile" ) )
			{
				context->sample->ResetProfile();
			}
			if ( ImGui::MenuItem( "Dump Mem Stats" ) )
			{
				b2World_DumpMemoryStats( context->sample->m_worldId );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Quit", "Ctrl+Q" ) )
			{
				glfwSetWindowShouldClose( context->window, GL_TRUE );
			}
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "View" ) )
		{
			if ( ImGui::MenuItem( "Hide UI", "Tab" ) )
			{
				context->showUI = false;
			}
			if ( ImGui::MenuItem( "Reset Camera", "Home" ) )
			{
				context->sample->FocusHome();
			}
			ImGui::Separator();
			ImGui::MenuItem( "Shapes", nullptr, &context->debugDraw.drawShapes );
			ImGui::MenuItem( "Chain Normals", nullptr, &context->debugDraw.drawChainNormals );
			ImGui::MenuItem( "Joints", nullptr, &context->debugDraw.drawJoints );
			ImGui::MenuItem( "Joint Extras", nullptr, &context->debugDraw.drawJointExtras );
			ImGui::MenuItem( "Bounds", nullptr, &context->debugDraw.drawBounds );
			ImGui::MenuItem( "Mass", nullptr, &context->debugDraw.drawMass );
			ImGui::MenuItem( "Body Names", nullptr, &context->debugDraw.drawBodyNames );
			ImGui::MenuItem( "Graph Colors", nullptr, &context->debugDraw.drawGraphColors );
			ImGui::MenuItem( "Islands", nullptr, &context->debugDraw.drawIslands );
			ImGui::Separator();
			ImGui::MenuItem( "Contact Points", nullptr, &context->debugDraw.drawContacts );
			ImGui::MenuItem( "Contact Normals", nullptr, &context->debugDraw.drawContactNormals );
			ImGui::MenuItem( "Contact Features", nullptr, &context->debugDraw.drawContactFeatures );
			ImGui::MenuItem( "Contact Forces", nullptr, &context->debugDraw.drawContactForces );
			ImGui::MenuItem( "Friction Forces", nullptr, &context->debugDraw.drawFrictionForces );
			if ( ImGui::BeginMenu( "Anchor" ) )
			{
				if ( ImGui::MenuItem( "Anchor A", nullptr, context->debugDraw.drawAnchorA ) )
				{
					context->debugDraw.drawAnchorA = true;
				}
				if ( ImGui::MenuItem( "Anchor B", nullptr, !context->debugDraw.drawAnchorA ) )
				{
					context->debugDraw.drawAnchorA = false;
				}
				ImGui::EndMenu();
			}
			ImGui::Separator();
			ImGui::MenuItem( "Metrics", "M", &context->showMetrics );
			ImGui::MenuItem( "Profile", "I", &context->showProfile, context->sample->HasProfile() );
			ImGui::Separator();
			if ( ImGui::BeginMenu( "Scale" ) )
			{
				ImGui::PushItemWidth( 6.0f * fontSize );
				ImGui::InputFloat( "Joint", &context->debugDraw.jointScale );
				ImGui::InputFloat( "Force", &context->debugDraw.forceScale );
				ImGui::PopItemWidth();
				ImGui::EndMenu();
			}
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "Samples" ) )
		{
			int i = 0;
			while ( i < g_sampleCount )
			{
				const char* category = g_sampleEntries[i].category;
				if ( ImGui::BeginMenu( category ) )
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
					{
						bool selected = ( i == context->sampleIndex );
						if ( ImGui::MenuItem( g_sampleEntries[i].name, nullptr, selected ) )
						{
							SelectSample( context, i, false );
						}
						++i;
					}
					ImGui::EndMenu();
				}
				else
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
					{
						++i;
					}
				}
			}
			ImGui::EndMenu();
		}

		// Only present once the replay viewer is registered. Open pops a native picker, then
		// hands the chosen file to the viewer through replayFile.
		if ( g_replayIndex >= 0 && ImGui::BeginMenu( "Replay" ) )
		{
			if ( ImGui::MenuItem( "Open..." ) )
			{
				NFD_Init();
				nfdu8char_t* outPath = nullptr;
				nfdu8filteritem_t filter[1] = { { "Box2D recording", "b2rec" } };
				if ( NFD_OpenDialogU8( &outPath, filter, 1, nullptr ) == NFD_OKAY )
				{
					snprintf( context->replayFile, sizeof( context->replayFile ), "%s", outPath );
					NFD_FreePathU8( outPath );
					SelectSample( context, g_replayIndex, false );
				}
				NFD_Quit();
			}
			ImGui::EndMenu();
		}

		static bool showAbout = false;
		if ( ImGui::BeginMenu( "Help" ) )
		{
			ImGui::MenuItem( "Controls", "?", &context->showControls );
			ImGui::MenuItem( "About", nullptr, &showAbout );
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();

		// Draw a dark border along the bottom of the menu
		{
			float menuBarBottom = ImGui::GetFrameHeight();
			ImU32 borderColor = ImGui::GetColorU32( ImGuiCol_Border );
			ImVec2 displaySize = ImGui::GetIO().DisplaySize;
			ImGui::GetForegroundDrawList()->AddLine( ImVec2( 0.0f, menuBarBottom ), ImVec2( displaySize.x, menuBarBottom ),
													 borderColor, 1.0f );
		}

		if ( context->showControls )
		{
			ImGui::SetNextWindowPos( { context->camera.width * 0.5f, context->camera.height * 0.5f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 24.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

			if ( ImGui::Begin( "Controls", &context->showControls, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				ImGui::SeparatorText( "Keyboard" );
				if ( ImGui::BeginTable( "keys", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Tab", "Show / hide UI" );
					DrawRow( "M", "Show / hide metrics" );
					DrawRow( "I", "Show / hide profile" );
					DrawRow( "P", "Pause / resume" );
					DrawRow( ".", "Single step (Shift: 5)" );
					DrawRow( ",", "Step back, replay only (Shift: 5)" );
					DrawRow( "R", "Restart sample" );
					DrawRow( "[  ]", "Previous / next sample" );
					DrawRow( "Ctrl+O", "Open sample picker" );
					DrawRow( "Arrows", "Pan camera" );
					DrawRow( "Z  X", "Zoom out / in" );
					DrawRow( "Home", "Reset camera" );
					DrawRow( "?", "Show / hide controls" );
					DrawRow( "Esc", "Cancel / close" );
					DrawRow( "Ctrl+Q", "Quit" );
					ImGui::EndTable();
				}

				ImGui::SeparatorText( "Mouse" );
				if ( ImGui::BeginTable( "mouse", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Left drag", "Move bodies (mouse joint)" );
					DrawRow( "Right drag", "Pan camera" );
					DrawRow( "Scroll wheel", "Zoom" );
					ImGui::EndTable();
				}
			}
			ImGui::End();
		}

		if ( showAbout )
		{
			ImGui::SetNextWindowPos( { context->camera.width * 0.5f, context->camera.height * 0.5f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 22.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

			if ( ImGui::Begin( "About", &showAbout, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				b2Version version = b2GetVersion();
				ImGui::Text( "Box2D %d.%d.%d", version.major, version.minor, version.revision );
				ImGui::Spacing();
				ImGui::TextLinkOpenURL( "box2d.org", "https://box2d.org/" );
				ImGui::TextLinkOpenURL( "github.com/erincatto/box2d", "https://github.com/erincatto/box2d" );
			}
			ImGui::End();
		}
	}
}

struct Scored
{
	int idx;
	int score;
};

void Rebuild( const char* q, int* outFiltered, int* outCount )
{
	static Scored scored[MAX_SAMPLES];
	int n = 0;
	for ( int i = 0; i < g_sampleCount; ++i )
	{
		int nameScore = FuzzyScore( q, g_sampleEntries[i].name );
		int catScore = FuzzyScore( q, g_sampleEntries[i].category );
		int best = -1;
		if ( nameScore >= 0 )
		{
			best = nameScore * 2; // name matches outweigh category-only matches
		}
		if ( catScore >= 0 && catScore > best )
		{
			best = catScore;
		}
		if ( best < 0 )
		{
			continue;
		}
		scored[n].idx = i;
		scored[n].score = best;
		n += 1;
	}

	// Stable insertion sort by score desc. Equal scores keep registry order
	// (which main.cpp sorts by category then name).
	for ( int i = 1; i < n; ++i )
	{
		Scored tmp = scored[i];
		int j = i - 1;
		while ( j >= 0 && scored[j].score < tmp.score )
		{
			scored[j + 1] = scored[j];
			--j;
		}
		scored[j + 1] = tmp;
	}

	for ( int i = 0; i < n; ++i )
	{
		outFiltered[i] = scored[i].idx;
	}
	*outCount = n;
}

void DrawSamplePicker( SampleContext* context )
{
	float fontSize = ImGui::GetFontSize();

	// Fuzzy sample picker (Ctrl+O). Opens a transient popup; type to filter by
	// name or category, Up/Down to navigate, Enter to select, Esc / click-outside to dismiss.
	static char query[64] = {};
	static char prevQuery[64] = {};
	static int highlight = 0;
	static int prevHighlight = 0;
	static int filtered[MAX_SAMPLES];
	static int filteredCount = 0;
	static bool justOpened = false;
	static bool forceScroll = false;

	if ( context->openSamplePicker )
	{
		ImGui::OpenPopup( "##sample_picker" );
		context->openSamplePicker = false;
		query[0] = '\0';
		prevQuery[0] = '\0';
		highlight = 0;
		prevHighlight = 0;
		Rebuild( query, filtered, &filteredCount );
		justOpened = true;
		forceScroll = true;
	}

	ImGui::SetNextWindowPos( { context->camera.width * 0.5f, context->camera.height * 0.35f }, ImGuiCond_Appearing,
							 { 0.5f, 0.5f } );
	ImGui::SetNextWindowSize( { 32.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

	if ( ImGui::BeginPopup( "##sample_picker",
							ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings ) )
	{
		if ( justOpened )
		{
			ImGui::SetKeyboardFocusHere();
			justOpened = false;
		}

		ImGui::PushItemWidth( -1.0f );
		ImGui::InputTextWithHint( "##q", "Search by name or category...", query, sizeof( query ) );
		ImGui::PopItemWidth();

		if ( strcmp( query, prevQuery ) != 0 )
		{
			Rebuild( query, filtered, &filteredCount );
			strncpy( prevQuery, query, sizeof( prevQuery ) );
			prevQuery[sizeof( prevQuery ) - 1] = '\0';
			highlight = 0;
			forceScroll = true;
		}

		if ( filteredCount > 0 )
		{
			if ( ImGui::IsKeyPressed( ImGuiKey_DownArrow, true ) )
			{
				highlight = ( highlight + 1 ) % filteredCount;
			}
			if ( ImGui::IsKeyPressed( ImGuiKey_UpArrow, true ) )
			{
				highlight = ( highlight + filteredCount - 1 ) % filteredCount;
			}
		}
		bool commit = ImGui::IsKeyPressed( ImGuiKey_Enter, false ) || ImGui::IsKeyPressed( ImGuiKey_KeypadEnter, false );

		ImGui::BeginChild( "##results", { 0.0f, 14.0f * fontSize }, ImGuiChildFlags_Borders );
		for ( int row = 0; row < filteredCount; ++row )
		{
			int i = filtered[row];
			char label[160];
			snprintf( label, sizeof( label ), "%s  >  %s", g_sampleEntries[i].category, g_sampleEntries[i].name );
			bool sel = ( row == highlight );
			if ( ImGui::Selectable( label, sel ) )
			{
				highlight = row;
				commit = true;
			}
			if ( sel && ( forceScroll || highlight != prevHighlight ) )
			{
				ImGui::SetScrollHereY();
			}
		}
		ImGui::EndChild();
		prevHighlight = highlight;
		forceScroll = false;

		if ( commit && filteredCount > 0 )
		{
			SelectSample( context, filtered[highlight], false );
			ImGui::CloseCurrentPopup();
		}

		// The active search field eats the first Esc, so dismiss here instead of
		// relying on the popup's default nav close.
		if ( ImGui::IsKeyPressed( ImGuiKey_Escape, false ) )
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

// A dot after the widget that reveals help on hover. Unlike a tooltip on the widget itself,
// it never covers the value being edited, and it is quieter than a "(?)" on every row.
static void HelpMarker( const char* text )
{
	ImGui::SameLine( 0.0f, ImGui::GetStyle().ItemInnerSpacing.x );
	float width = ImGui::GetFontSize();
	float height = ImGui::GetFrameHeight();
	ImVec2 pos = ImGui::GetCursorScreenPos();
	ImGui::Dummy( { width, height } );
	bool hovered = ImGui::IsItemHovered();
	ImU32 color = ImGui::GetColorU32( ImGuiCol_CheckMark, hovered ? 1.0f : 0.6f );
	ImVec2 center = { pos.x + 0.5f * width, pos.y + 0.5f * height };
	ImGui::GetWindowDrawList()->AddCircleFilled( center, 0.2f * width, color );
	ImGui::SetItemTooltip( "%s", text );
}

static void DrawInfoPanel( SampleContext* context, float frameTime )
{
	const SampleEntry& entry = g_sampleEntries[context->sampleIndex];
	float fontSize = ImGui::GetFontSize();
	float menuWidth = context->sample->InfoPanelWidthEm() * fontSize;
	float menuBarHeight = ImGui::GetFrameHeight();

	ImGui::SetNextWindowPos( { context->camera.width - menuWidth - 0.5f * fontSize, menuBarHeight + 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { menuWidth, context->camera.height - menuBarHeight - fontSize } );

	ImGui::Begin( "Info", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	ImGui::TextColored( MakeColor( b2_colorGoldenRod ), "%s", entry.name );
	ImGui::TextColored( MakeColor( b2_colorLightGray ), "%s", entry.category );
	ImGui::Separator();

	if ( context->pause )
	{
		ImGui::TextColored( MakeColor( b2_colorRed ), "PAUSED" );
		ImGui::SameLine();
		ImGui::TextDisabled( "(P)" );
		ImGui::Separator();
	}

	ImGui::TextColored( MakeColor( b2_colorSeaGreen ), "frame %.1f ms", 1000.0f * frameTime );
	ImGui::TextColored( MakeColor( b2_colorSeaGreen ), "step %d", context->sample->m_stepCount );
	ImGui::Separator();
	ImGui::TextColored( MakeColor( b2_colorSeaGreen ), "cam (%.1f, %.1f)", context->camera.center.x, context->camera.center.y );
	ImGui::TextColored( MakeColor( b2_colorSeaGreen ), "zoom %.2f", context->camera.zoom );

	ImGui::Separator();

	ImGui::PushItemWidth( 6.0f * fontSize );
	if ( context->sample->DrawControls() )
	{
		ImGui::Separator();
	}
	ImGui::PopItemWidth();

	if ( context->sample->HasSolverControls() && ImGui::CollapsingHeader( "Solver", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		ImGui::PushItemWidth( 6.0f * fontSize );

		ImGui::SliderInt( "Sub-steps##Solver", &context->subStepCount, 1, 32 );
		HelpMarker( "The solver breaks the full step into several sub-steps.\nMore sub-steps usually lead to more accurate results." );

		ImGui::SliderInt( "Bounce Iters##Solver", &context->restitutionIterations, 0, 8 );
		HelpMarker( "Iterations for the restitution solver." );

		ImGui::SliderFloat( "Hertz##Solver", &context->hertz, 5.0f, 240.0f, "%.0f Hz" );
		HelpMarker( "The number of world steps per second." );

		if ( ImGui::SliderInt( "Workers##Solver", &context->workerCount, 1, B2_MAX_WORKERS ) )
		{
			context->workerCount = b2ClampInt( context->workerCount, 1, B2_MAX_WORKERS );
			SelectSample( context, context->sampleIndex, true );
		}
		HelpMarker( "The number worker threads used by the world step." );

		float recyclingCentimeters = 100.0f * context->recycleDistance;
		if ( ImGui::SliderFloat( "Recycle##Solver", &recyclingCentimeters, 0.0f, 10.0f, "%.1f cm" ) )
		{
			context->recycleDistance = 0.01f * recyclingCentimeters;
			b2World_SetContactRecycleDistance( context->sample->m_worldId, context->recycleDistance );
		}
		HelpMarker( "The contact recycling distance tolerance.\nSet to zero to disable recycling." );

		ImGui::PopItemWidth();

		ImGui::Checkbox( "Sleep##Solver", &context->enableSleep );
		HelpMarker( "Allow bodies to sleep, reducing simulation CPU cost." );

		ImGui::Checkbox( "Warm Starting##Solver", &context->enableWarmStarting );
		HelpMarker( "Enable solver warm starting which usually improves stacking stability." );

		ImGui::Checkbox( "Continuous##Solver", &context->enableContinuous );
		HelpMarker( "Enable continuous collision detection." );

		ImGui::Checkbox( "Bounce Propagation##Solver", &context->enableRestitutionPropagation );
		HelpMarker( "Enable restitution solver propagation across all touching contacts points" );
	}

	if ( context->sample->HasSolverControls() && ImGui::CollapsingHeader( "Recording", ImGuiTreeNodeFlags_DefaultOpen ) )
	{
		ImGui::PushItemWidth( 9.0f * fontSize );
		ImGui::InputText( "File##Recording", context->recordingFile, sizeof( context->recordingFile ) );
		ImGui::PopItemWidth();

		if ( context->sample->m_recording == nullptr )
		{
			// Restart to a clean world then snapshot it at step 0, a whole session capture
			if ( ImGui::Button( "Record (restart)##Recording" ) )
			{
				SelectSample( context, context->sampleIndex, true );
				context->sample->StartRecording();
			}

			// Snapshot the running world and log from here, the mid simulation case
			if ( ImGui::Button( "Record Now##Recording" ) )
			{
				context->sample->StartRecording();
			}

			if ( g_replayIndex >= 0 && context->savedRecordingFile[0] != 0 )
			{
				if ( ImGui::Button( "Play##Recording" ) )
				{
					snprintf( context->replayFile, sizeof( context->replayFile ), "%s", context->savedRecordingFile );
					SelectSample( context, g_replayIndex, false );
				}
				ImGui::SetItemTooltip( "Open %s in the replay viewer", context->savedRecordingFile );
			}
		}
		else
		{
			if ( ImGui::Button( "Stop##Recording" ) )
			{
				context->sample->FinishRecording();
			}
		}

		if ( context->sample->m_recording != nullptr )
		{
			float kilobytes = b2Recording_GetSize( context->sample->m_recording ) / 1024.0f;
			int steps = context->sample->m_stepCount - context->sample->m_recordStartStep;
			if ( kilobytes < 1024.0f )
			{
				ImGui::TextColored( MakeColor( b2_colorRed ), "recording %d steps, %.0f KB", steps, kilobytes );
			}
			else
			{
				ImGui::TextColored( MakeColor( b2_colorRed ), "recording %d steps, %.1f MB", steps, kilobytes / 1024.0f );
			}
		}
	}

	ImGui::End();
}

// Entry point for all UI drawing. This should not be a member of Sample because
// this can delete the current sample.
void DrawUI( SampleContext* context, float frameTime )
{
	DrawMenuBar( context );
	DrawSamplePicker( context );
	DrawInfoPanel( context, frameTime );
	context->sample->DrawProfile();
	context->sample->DrawMetrics();
}
