// SPDX-FileCopyrightText: 2023 Erin Catto
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
#include <stdio.h>
#include <stdlib.h>

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
	fprintf( file, "  \"drawShapes\": %s,\n", debugDraw.drawShapes ? "true" : "false" );
	fprintf( file, "  \"drawJoints\": %s,\n", debugDraw.drawJoints ? "true" : "false" );
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

void DrawPolygonFcn( const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawPolygon( sampleContext->draw, vertices, vertexCount, color );
}

void DrawSolidPolygonFcn( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
						  void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawSolidPolygon( sampleContext->draw, transform, vertices, vertexCount, radius, color );
}

void DrawCircleFcn( b2Vec2 center, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawCircle( sampleContext->draw, center, radius, color );
}

void DrawSolidCircleFcn( b2Transform transform, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawSolidCircle( sampleContext->draw, transform, radius, color );
}

void DrawSolidCapsuleFcn( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawSolidCapsule( sampleContext->draw, p1, p2, radius, color );
}

void DrawLineFcn( b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawLine( sampleContext->draw, p1, p2, color );
}

void DrawTransformFcn( b2Transform transform, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawTransform( sampleContext->draw, transform, 1.0f );
}

void DrawPointFcn( b2Vec2 p, float size, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawPoint( sampleContext->draw, p, size, color );
}

void DrawStringFcn( b2Vec2 p, const char* s, b2HexColor color, void* context )
{
	SampleContext* sampleContext = static_cast<SampleContext*>( context );
	DrawWorldString( sampleContext->draw, &sampleContext->camera, p, color, s );
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
	debugDraw.context = this;

	recycleDistance = B2_CONTACT_RECYCLE_DISTANCE;

	char* data = nullptr;
	int size = 0;
	bool found = ReadFile( data, size, fileName );
	if ( found == false )
	{
		return;
	}

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
		else if ( jsoneq( data, &tokens[i], "drawJoints" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				debugDraw.drawJoints = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				debugDraw.drawJoints = false;
			}
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

Sample::Sample( SampleContext* context )
{
	m_context = context;
	m_camera = &context->camera;
	m_draw = context->draw;

	m_worldId = b2_nullWorldId;

	m_textIncrement = 26;
	m_textLine = m_textIncrement;
	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;
	m_didStep = false;

	m_mouseBodyId = b2_nullBodyId;
	m_mousePoint = {};
	m_mouseForceScale = 100.0f;

	memset( m_profiles, 0, sizeof( m_profiles ) );
	m_currentProfileIndex = 0;
	m_profileReadIndex = 0;
	m_profileWriteIndex = 0;

	g_randomSeed = RAND_SEED;

	CreateWorld();
	TestMathCpp();
}

Sample::~Sample()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	b2DestroyWorld( m_worldId );
}

void Sample::CreateWorld()
{
	if ( B2_IS_NON_NULL( m_worldId ) )
	{
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
	m_textLine = m_textIncrement;
}

struct QueryContext
{
	b2Vec2 point;
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

void Sample::MouseDown( b2Vec2 p, int button, int mod )
{
	if ( B2_IS_NON_NULL( m_mouseJointId ) )
	{
		return;
	}

	if ( button == GLFW_MOUSE_BUTTON_1 )
	{
		// Make a small box.
		b2AABB box;
		b2Vec2 d = { 0.001f, 0.001f };
		box.lowerBound = b2Sub( p, d );
		box.upperBound = b2Add( p, d );

		m_mousePoint = p;

		// Query the world for overlapping shapes.
		QueryContext queryContext = { p, b2_nullBodyId };
		b2World_OverlapAABB( m_worldId, box, b2DefaultQueryFilter(), QueryCallback, &queryContext );

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

void Sample::MouseUp( b2Vec2 p, int button )
{
	if ( B2_IS_NON_NULL( m_mouseJointId ) && button == GLFW_MOUSE_BUTTON_1 )
	{
		b2DestroyJoint( m_mouseJointId, true );
		m_mouseJointId = b2_nullJointId;

		b2DestroyBody( m_mouseBodyId );
		m_mouseBodyId = b2_nullBodyId;
	}
}

void Sample::MouseMove( b2Vec2 p )
{
	if ( b2Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = b2_nullJointId;
	}

	m_mousePoint = p;
}

void Sample::DrawColoredTextLine( b2HexColor color, const char* text, ... )
{
	if ( m_context->showUI == false )
	{
		return;
	}

	char buffer[256];
	va_list arg;
	va_start( arg, text );
	vsnprintf( buffer, 256, text, arg );
	va_end( arg );
	buffer[255] = 0;
	DrawScreenString( m_draw, 5, m_textLine, color, buffer );
	m_textLine += m_textIncrement;
}

void Sample::DrawTextLine( const char* text, ... )
{
	if ( m_context->showUI == false )
	{
		return;
	}

	char buffer[256];
	va_list arg;
	va_start( arg, text );
	vsnprintf( buffer, 256, text, arg );
	va_end( arg );
	buffer[255] = 0;
	DrawScreenString( m_draw, 5, m_textLine, b2_colorWhite, buffer );
	m_textLine += m_textIncrement;
}

void Sample::ResetProfile()
{
	m_stepCount = 0;
}

void Sample::Step()
{
	m_didStep = false;

	float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

	if ( m_context->pause )
	{
		if ( m_context->singleStep )
		{
			m_context->singleStep = false;
		}
		else
		{
			timeStep = 0.0f;
		}

		if ( m_context->showUI )
		{
			DrawTextLine( "****PAUSED****" );
			m_textLine += m_textIncrement;
		}
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

void Sample::UpdateGui()
{
	float fontSize = ImGui::GetFontSize();

	if ( m_context->drawProfile )
	{
		ImGui::SetNextWindowPos( { fontSize, 8.0f * fontSize }, ImGuiCond_FirstUseEver );
		ImGui::Begin( "Profile (ms)", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize );

		const int count = static_cast<int>( m_profileWriteIndex - m_profileReadIndex );

		// Unroll ring buffer into per-field histories.
		constexpr int kRowCount = 22;
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
			histories[12][i] = p.applyRestitution;
			histories[13][i] = p.storeImpulses;
			histories[14][i] = p.splitIslands;
			histories[15][i] = p.transforms;
			histories[16][i] = p.jointEvents;
			histories[17][i] = p.hitEvents;
			histories[18][i] = p.refit;
			histories[19][i] = p.sleepIslands;
			histories[20][i] = p.bullets;
			histories[21][i] = p.sensors;

			totals[0] += p.step;
			totals[1] += p.pairs;
			totals[2] += p.collide;
			totals[3] += p.solve;
			totals[4] += p.solverSetup;
			totals[5] += p.constraints;
			totals[6] += p.prepareConstraints;
			totals[7] += p.integrateVelocities;
			totals[8] += p.warmStart;
			totals[9] += p.solveImpulses;
			totals[10] += p.integratePositions;
			totals[11] += p.relaxImpulses;
			totals[12] += p.applyRestitution;
			totals[13] += p.storeImpulses;
			totals[14] += p.splitIslands;
			totals[15] += p.transforms;
			totals[16] += p.jointEvents;
			totals[17] += p.hitEvents;
			totals[18] += p.refit;
			totals[19] += p.sleepIslands;
			totals[20] += p.bullets;
			totals[21] += p.sensors;
		}

		const b2Profile& cur = m_profiles[m_currentProfileIndex];
		const float now[kRowCount] = {
			cur.step,
			cur.pairs,
			cur.collide,
			cur.solve,
			cur.solverSetup,
			cur.constraints,
			cur.prepareConstraints,
			cur.integrateVelocities,
			cur.warmStart,
			cur.solveImpulses,
			cur.integratePositions,
			cur.relaxImpulses,
			cur.applyRestitution,
			cur.storeImpulses,
			cur.splitIslands,
			cur.transforms,
			cur.jointEvents,
			cur.hitEvents,
			cur.refit,
			cur.sleepIslands,
			cur.bullets,
			cur.sensors,
		};

		// Rolling average
		float avg[kRowCount] = {};
		if ( count > 0 )
		{
			float scale = 1.0f / count;
			for ( int i = 0; i < kRowCount; ++i )
			{
				avg[i] = scale * totals[i];
			}
		}

		// Match Frame Time chart's first three colors so rows read with the line plot.
		const ImU32 colorStep = IM_COL32( 102, 153, 255, 255 );
		const ImU32 colorCollide = IM_COL32( 255, 140, 51, 255 );
		const ImU32 colorSolve = IM_COL32( 102, 204, 102, 255 );
		const ImU32 colorDefault = IM_COL32( 220, 220, 220, 255 );

		struct RowDef
		{
			const char* name;
			int indent;
			ImU32 color;
		};
		const RowDef rows[kRowCount] = {
			{ "step", 0, colorStep },			{ "pairs", 0, colorDefault },		 { "collide", 0, colorCollide },
			{ "solve", 0, colorSolve },			{ "setup", 1, colorDefault },		 { "constraints", 1, colorDefault },
			{ "prepare", 2, colorDefault },		{ "velocities", 2, colorDefault },	 { "warm start", 2, colorDefault },
			{ "bias", 2, colorDefault },		{ "positions", 2, colorDefault },	 { "relax", 2, colorDefault },
			{ "restitution", 2, colorDefault }, { "store", 2, colorDefault },		 { "split islands", 2, colorDefault },
			{ "transforms", 1, colorDefault },	{ "joint events", 1, colorDefault }, { "hit events", 1, colorDefault },
			{ "refit BVH", 1, colorDefault },	{ "sleep", 1, colorDefault },		 { "bullets", 1, colorDefault },
			{ "sensors", 0, colorDefault },
		};

		// Derive parent/child links from the indent levels so we can collapse subtrees.
		int parents[kRowCount];
		bool hasChildren[kRowCount] = {};
		{
			int stack[8];
			int stackSize = 0;
			for ( int i = 0; i < kRowCount; ++i )
			{
				while ( stackSize > 0 && rows[stack[stackSize - 1]].indent >= rows[i].indent )
				{
					--stackSize;
				}
				parents[i] = stackSize > 0 ? stack[stackSize - 1] : -1;
				stack[stackSize++] = i;
				if ( parents[i] >= 0 )
				{
					hasChildren[parents[i]] = true;
				}
			}
		}

		static bool s_rowOpen[kRowCount];
		static bool s_showPlots = false;

		// Bars are drawn relative to the step row so the proportions are visually consistent.
		const float stepNow = b2MaxFloat( cur.step, 0.001f );

		if ( ImGui::Button( "Reset" ) )
		{
			ResetProfile();
		}
		ImGui::SameLine();
		ImGui::Checkbox( "Show plots", &s_showPlots );

		const ImGuiTableFlags tableFlags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;

		const int colCount = s_showPlots ? 6 : 5;
		if ( ImGui::BeginTable( "profile", colCount, tableFlags ) )
		{
			ImGui::TableSetupColumn( "section", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize );
			ImGui::TableSetupColumn( "now", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "avg", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "max", ImGuiTableColumnFlags_WidthFixed, 3.0f * fontSize );
			ImGui::TableSetupColumn( "% step", ImGuiTableColumnFlags_WidthFixed, 8.0f * fontSize );
			if ( s_showPlots )
			{
				ImGui::TableSetupColumn( "history", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize );
			}
			ImGui::TableHeadersRow();

			const float rowHeight = 1.5f * fontSize;

			for ( int r = 0; r < kRowCount; ++r )
			{
				bool visible = true;
				for ( int p = parents[r]; p >= 0; p = parents[p] )
				{
					if ( !s_rowOpen[p] )
					{
						visible = false;
						break;
					}
				}
				if ( !visible )
				{
					continue;
				}

				const RowDef& d = rows[r];
				const float* hist = histories[r];

				// Rolling max from live history, replacing the old session-sticky max.
				float rmax = 0.0f;
				for ( int i = 0; i < count; ++i )
				{
					rmax = b2MaxFloat( rmax, hist[i] );
				}

				ImGui::TableNextRow();

				ImGui::TableNextColumn();
				if ( d.indent > 0 )
				{
					ImGui::Indent( d.indent * fontSize );
				}
				if ( hasChildren[r] )
				{
					ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
											   ImGuiTreeNodeFlags_NoTreePushOnOpen;
					ImGui::PushStyleColor( ImGuiCol_Text, d.color );
					s_rowOpen[r] = ImGui::TreeNodeEx( d.name, flags );
					ImGui::PopStyleColor();
				}
				else
				{
					float leafIndent = ImGui::GetTreeNodeToLabelSpacing();
					ImGui::Indent( leafIndent );
					ImGui::PushStyleColor( ImGuiCol_Text, d.color );
					ImGui::TextUnformatted( d.name );
					ImGui::PopStyleColor();
					ImGui::Unindent( leafIndent );
				}
				if ( d.indent > 0 )
				{
					ImGui::Unindent( d.indent * fontSize );
				}

				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", now[r] );
				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", avg[r] );
				ImGui::TableNextColumn();
				ImGui::Text( "%6.2f", rmax );

				ImGui::TableNextColumn();
				float frac = b2ClampFloat( now[r] / stepNow, 0.0f, 1.0f );
				ImGui::PushStyleColor( ImGuiCol_PlotHistogram, d.color );
				ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), "" );
				ImGui::PopStyleColor();

				if ( s_showPlots )
				{
					ImGui::TableNextColumn();
					if ( count > 1 )
					{
						char id[16];
						snprintf( id, sizeof( id ), "##h%d", r );
						ImGui::PushStyleColor( ImGuiCol_PlotLines, d.color );
						ImGui::PlotLines( id, hist, count, 0, nullptr, 0.0f, rmax * 1.05f + 0.001f,
										  ImVec2( -FLT_MIN, rowHeight ) );
						ImGui::PopStyleColor();
					}
				}
			}
			ImGui::EndTable();
		}

		ImGui::End();
	}

	if ( m_context->drawCounters )
	{
		b2Counters s = b2World_GetCounters( m_worldId );
		constexpr int colorCount = sizeof( s.colorCounts ) / sizeof( s.colorCounts[0] );
		const int overflowIndex = colorCount - 1;

		// Bars are scaled to the largest non-overflow color so the distribution shape reads clearly;
		// overflow gets its own bar against the same scale, with a red tint to flag coupling problems.
		int totalCount = 0;
		int maxCount = 1;
		for ( int i = 0; i < colorCount; ++i )
		{
			totalCount += s.colorCounts[i];
			if ( i != overflowIndex && s.colorCounts[i] > maxCount )
			{
				maxCount = s.colorCounts[i];
			}
		}

		ImGui::SetNextWindowPos( { fontSize, 8.0f * fontSize }, ImGuiCond_FirstUseEver );
		ImGui::Begin( "Counters", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize );

		ImGui::Text( "bodies/shapes/contacts/joints = %d/%d/%d/%d", s.bodyCount, s.shapeCount, s.contactCount, s.jointCount );
		{
			float frac = s.awakeContactCount > 0
							 ? b2ClampFloat( (float)s.recycledContactCount / (float)s.awakeContactCount, 0.0f, 1.0f )
							 : 0.0f;

			char overlay[32];
			snprintf( overlay, sizeof( overlay ), "%d / %d", s.recycledContactCount, s.awakeContactCount );

			ImGui::TextUnformatted( "recycled contacts" );
			ImGui::SameLine();
			ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), overlay );
		}
		ImGui::Text( "islands/tasks = %d/%d", s.islandCount, s.taskCount );
		ImGui::Text( "tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight );
		ImGui::Text( "stack allocator size = %d K", s.stackUsed / 1024 );
		ImGui::Text( "total allocation = %d K", s.byteCount / 1024 );

		ImGui::Separator();
		b2Capacity c = b2World_GetMaxCapacity( m_worldId );
		ImGui::Text( "max capacities" );
		ImGui::BulletText( "static shapes/bodies = %d/%d", c.staticShapeCount, c.staticBodyCount );
		ImGui::BulletText( "dynamic shapes/bodies = %d/%d", c.dynamicShapeCount, c.dynamicBodyCount );
		ImGui::BulletText( "contacts = %d", c.contactCount );

		ImGui::Separator();
		ImGui::Text( "%d constraints across %d colors", totalCount, colorCount );

		const ImGuiTableFlags tableFlags = ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit;
		if ( ImGui::BeginTable( "graphColors", 3, tableFlags ) )
		{
			ImGui::TableSetupColumn( "color", ImGuiTableColumnFlags_WidthFixed, 3.5f * fontSize );
			ImGui::TableSetupColumn( "count", ImGuiTableColumnFlags_WidthFixed, 5.0f * fontSize );
			ImGui::TableSetupColumn( "share", ImGuiTableColumnFlags_WidthFixed, 16.0f * fontSize );
			ImGui::TableHeadersRow();

			const float invMax = 1.0f / static_cast<float>( maxCount );

			for ( int i = 0; i < colorCount; ++i )
			{
				int count = s.colorCounts[i];
				bool isOverflow = ( i == overflowIndex );

				// Skip empty slots, but always show overflow — a non-zero overflow row is the signal we care about.
				if ( count == 0 && !isOverflow )
				{
					continue;
				}

				uint32_t hex = static_cast<uint32_t>( b2GetGraphColor( i ) );
				ImU32 swatch = IM_COL32( ( hex >> 16 ) & 0xFF, ( hex >> 8 ) & 0xFF, hex & 0xFF, 255 );
				ImU32 barColor = isOverflow ? IM_COL32( 220, 60, 60, 255 ) : swatch;

				ImGui::TableNextRow();

				ImGui::TableNextColumn();
				if ( isOverflow )
				{
					ImGui::PushStyleColor( ImGuiCol_Text, IM_COL32( 220, 60, 60, 255 ) );
					ImGui::TextUnformatted( "over" );
					ImGui::PopStyleColor();
				}
				else
				{
					ImGui::PushStyleColor( ImGuiCol_Text, swatch );
					ImGui::Text( "%d", i );
					ImGui::PopStyleColor();
				}

				ImGui::TableNextColumn();
				ImGui::Text( "%d", count );

				ImGui::TableNextColumn();
				float frac = b2ClampFloat( count * invMax, 0.0f, 1.0f );
				ImGui::PushStyleColor( ImGuiCol_PlotHistogram, barColor );
				ImGui::ProgressBar( frac, ImVec2( -FLT_MIN, 0.0f ), "" );
				ImGui::PopStyleColor();
			}
			ImGui::EndTable();
		}

		ImGui::End();
	}

	if ( m_context->frameTime )
	{
		float frameTimeHeight = 30.0f * fontSize;
		float frameTimeWidth = 50.0f * fontSize;

		ImGui::SetNextWindowPos( { 3.0f * fontSize, 3.0f * fontSize }, ImGuiCond_FirstUseEver );
		ImGui::SetNextWindowSize( { frameTimeWidth, frameTimeHeight }, ImGuiCond_FirstUseEver );

		ImGui::Begin( "Frame Time", nullptr, ImGuiWindowFlags_NoCollapse );

		ImGui::PushItemWidth( ImGui::GetWindowWidth() - 2.0f * fontSize );

		float maxValue = 0.0f;
		float times[m_profileCapacity];
		float stepTimes[m_profileCapacity];
		float collideTimes[m_profileCapacity];
		float solveTimes[m_profileCapacity];
		int count = static_cast<int>( m_profileWriteIndex - m_profileReadIndex );
		for ( int i = 0; i < count; ++i )
		{
			int index = ( m_profileReadIndex + i ) & ( m_profileCapacity - 1 );
			times[i] = i / 60.0f;
			stepTimes[i] = m_profiles[index].step;
			collideTimes[i] = m_profiles[index].collide;
			solveTimes[i] = m_profiles[index].solve;
			maxValue = b2MaxFloat( stepTimes[i], maxValue );
		}

		// This is the pixel size, not the range.
		ImVec2 plotSize = { -1, 22.0f * fontSize };
		if ( ImPlot::BeginPlot( "Profile", plotSize, ImPlotFlags_NoTitle ) )
		{
			ImPlot::SetupAxes( "t", "ms" );
			ImPlot::SetupAxesLimits( 0, m_profileCapacity / 60.0, 0.0, maxValue, ImPlotCond_Always );
			ImPlot::PlotLine( "step", times, stepTimes, count );
			ImPlot::PlotLine( "collide", times, collideTimes, count );
			ImPlot::PlotLine( "solve", times, solveTimes, count );
			ImPlot::EndPlot();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}
}

void Sample::ShiftOrigin( b2Vec2 newOrigin )
{
	// m_world->ShiftOrigin(newOrigin);
}

// Parse an SVG path element with only straight lines. Example:
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
	}
	return pointCount;
}

SampleEntry g_sampleEntries[MAX_SAMPLES] = {};
int g_sampleCount = 0;

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

void SelectSample( SampleContext* context, int selection, bool restart )
{
	if ( restart == false )
	{
		ResetView( &context->camera );
		context->sampleIndex = selection;
		context->subStepCount = 4;
		context->debugDraw.drawJoints = true;
	}

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
}

void UpdateSampleUI( SampleContext* context )
{
	int maxWorkers = B2_MAX_WORKERS;
	b2WorldId worldId = context->sample->m_worldId;

	float fontSize = ImGui::GetFontSize();
	float menuWidth = 13.0f * fontSize;
	if ( context->showUI )
	{
		ImGui::SetNextWindowPos( { context->camera.width - menuWidth - 0.5f * fontSize, 0.5f * fontSize } );
		ImGui::SetNextWindowSize( { menuWidth, context->camera.height - fontSize } );

		ImGui::Begin( "Tools", &context->showUI,
					  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse );

		if ( ImGui::BeginTabBar( "ControlTabs", ImGuiTabBarFlags_None ) )
		{
			if ( ImGui::BeginTabItem( "Controls" ) )
			{
				ImGui::PushItemWidth( 100.0f );
				ImGui::SliderInt( "Sub-steps", &context->subStepCount, 1, 32 );
				ImGui::SliderFloat( "Hertz", &context->hertz, 5.0f, 240.0f, "%.0f hz" );

				if ( ImGui::SliderInt( "Workers", &context->workerCount, 1, maxWorkers ) )
				{
					context->workerCount = b2ClampInt( context->workerCount, 1, maxWorkers );
					SelectSample( context, context->sampleIndex, true );
				}
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Checkbox( "Sleep", &context->enableSleep );
				ImGui::Checkbox( "Warm Starting", &context->enableWarmStarting );
				ImGui::Checkbox( "Continuous", &context->enableContinuous );

				ImGui::PushItemWidth( 100.0f );
				float recyclingCentimeters = 100.0f * context->recycleDistance;
				if ( ImGui::SliderFloat( "Recycle", &recyclingCentimeters, 0.0f, 10.0f, "%.1f cm" ) )
				{
					context->recycleDistance = 0.01f * recyclingCentimeters;
					b2World_SetContactRecycleDistance( worldId, context->recycleDistance );
				}
				ImGui::PopItemWidth();

				ImGui::Separator();

				ImGui::Checkbox( "Shapes", &context->debugDraw.drawShapes );
				ImGui::Checkbox( "Joints", &context->debugDraw.drawJoints );
				ImGui::Checkbox( "Joint Extras", &context->debugDraw.drawJointExtras );
				ImGui::Checkbox( "Bounds", &context->debugDraw.drawBounds );
				ImGui::Checkbox( "Mass", &context->debugDraw.drawMass );
				ImGui::Checkbox( "Body Names", &context->debugDraw.drawBodyNames );
				ImGui::Checkbox( "Graph Colors", &context->debugDraw.drawGraphColors );
				ImGui::Checkbox( "Islands", &context->debugDraw.drawIslands );
				ImGui::Checkbox( "Counters", &context->drawCounters );
				ImGui::Checkbox( "Profile", &context->drawProfile );
				ImGui::Checkbox( "Frame Time", &context->frameTime );

				ImGui::Separator();

				ImGui::Checkbox( "Contact Points", &context->debugDraw.drawContacts );

				if ( ImGui::RadioButton( "Anchor A", context->debugDraw.drawAnchorA == true ) )
				{
					context->debugDraw.drawAnchorA = true;
				}
				ImGui::SameLine();
				if ( ImGui::RadioButton( "Anchor B", context->debugDraw.drawAnchorA == false ) )
				{
					context->debugDraw.drawAnchorA = false;
				}
				ImGui::Checkbox( "Contact Normals", &context->debugDraw.drawContactNormals );
				ImGui::Checkbox( "Contact Features", &context->debugDraw.drawContactFeatures );
				ImGui::Checkbox( "Contact Forces", &context->debugDraw.drawContactForces );
				ImGui::Checkbox( "Friction Forces", &context->debugDraw.drawFrictionForces );

				ImGui::Separator();

				ImGui::PushItemWidth( 80.0f );
				ImGui::InputFloat( "Joint Scale", &context->debugDraw.jointScale );
				ImGui::InputFloat( "Force Scale", &context->debugDraw.forceScale );
				ImGui::PopItemWidth();

				ImVec2 button_sz = ImVec2( -1, 0 );
				if ( ImGui::Button( "Pause (P)", button_sz ) )
				{
					context->pause = !context->pause;
				}

				if ( ImGui::Button( "Single Step (O)", button_sz ) )
				{
					context->singleStep = !context->singleStep;
				}

				if ( ImGui::Button( "Dump Mem Stats", button_sz ) )
				{
					b2World_DumpMemoryStats( context->sample->m_worldId );
				}

				if ( ImGui::Button( "Reset Profile", button_sz ) )
				{
					context->sample->ResetProfile();
				}

				if ( ImGui::Button( "Restart (R)", button_sz ) )
				{
					SelectSample( context, context->sampleIndex, true );
				}

				if ( ImGui::Button( "Quit", button_sz ) )
				{
					glfwSetWindowShouldClose( context->window, GL_TRUE );
				}

				ImGui::EndTabItem();
			}

			ImGuiTreeNodeFlags leafNodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
			leafNodeFlags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

			ImGuiTreeNodeFlags nodeFlags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

			if ( ImGui::BeginTabItem( "Samples" ) )
			{
				int categoryIndex = 0;
				const char* category = g_sampleEntries[categoryIndex].category;
				int i = 0;
				while ( i < g_sampleCount )
				{
					bool categorySelected = strcmp( category, g_sampleEntries[context->sampleIndex].category ) == 0;
					ImGuiTreeNodeFlags nodeSelectionFlags = categorySelected ? ImGuiTreeNodeFlags_Selected : 0;
					bool nodeOpen = ImGui::TreeNodeEx( category, nodeFlags | nodeSelectionFlags );

					if ( nodeOpen )
					{
						while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
						{
							ImGuiTreeNodeFlags selectionFlags = 0;
							if ( context->sampleIndex == i )
							{
								selectionFlags = ImGuiTreeNodeFlags_Selected;
							}
							ImGui::TreeNodeEx( (void*)(intptr_t)i, leafNodeFlags | selectionFlags, "%s",
											   g_sampleEntries[i].name );
							if ( ImGui::IsItemClicked() )
							{
								SelectSample( context, i, false );
							}
							++i;
						}
						ImGui::TreePop();
					}
					else
					{
						while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
						{
							++i;
						}
					}

					if ( i < g_sampleCount )
					{
						category = g_sampleEntries[i].category;
						categoryIndex = i;
					}
				}
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}

		ImGui::End();

		context->sample->UpdateGui();
	}
}
