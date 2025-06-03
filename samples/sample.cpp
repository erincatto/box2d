// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample.h"

#include "TaskScheduler.h"
#include "draw.h"
#include "imgui.h"
#include "random.h"

// consider using https://github.com/skeeto/pdjson
#include "jsmn.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <ctype.h>
#include <stdio.h>

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
	fprintf( file, "  \"drawShapes\": %s,\n", drawShapes ? "true" : "false" );
	fprintf( file, "  \"drawJoints\": %s,\n", drawJoints ? "true" : "false" );
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

#define MAX_TOKENS 32

void SampleContext::Load()
{
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
				drawShapes = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				drawShapes = false;
			}
		}
		else if ( jsoneq( data, &tokens[i], "drawJoints" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				drawJoints = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				drawJoints = false;
			}
		}
	}

	free( data );
}

class SampleTask : public enki::ITaskSet
{
public:
	SampleTask() = default;

	void ExecuteRange( enki::TaskSetPartition range, uint32_t threadIndex ) override
	{
		m_task( range.start, range.end, threadIndex, m_taskContext );
	}

	b2TaskCallback* m_task = nullptr;
	void* m_taskContext = nullptr;
};

static void* EnqueueTask( b2TaskCallback* task, int32_t itemCount, int32_t minRange, void* taskContext, void* userContext )
{
	Sample* sample = static_cast<Sample*>( userContext );
	if ( sample->m_taskCount < Sample::m_maxTasks )
	{
		SampleTask& sampleTask = sample->m_tasks[sample->m_taskCount];
		sampleTask.m_SetSize = itemCount;
		sampleTask.m_MinRange = minRange;
		sampleTask.m_task = task;
		sampleTask.m_taskContext = taskContext;
		sample->m_scheduler->AddTaskSetToPipe( &sampleTask );
		++sample->m_taskCount;
		return &sampleTask;
	}
	else
	{
		// This is not fatal but the maxTasks should be increased
		assert( false );
		task( 0, itemCount, 0, taskContext );
		return nullptr;
	}
}

static void FinishTask( void* taskPtr, void* userContext )
{
	if ( taskPtr != nullptr )
	{
		SampleTask* sampleTask = static_cast<SampleTask*>( taskPtr );
		Sample* sample = static_cast<Sample*>( userContext );
		sample->m_scheduler->WaitforTask( sampleTask );
	}
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
	m_draw = &context->draw;

	m_scheduler = new enki::TaskScheduler;
	m_scheduler->Initialize( m_context->workerCount );

	m_tasks = new SampleTask[m_maxTasks];
	m_taskCount = 0;

	m_threadCount = 1 + m_context->workerCount;

	m_worldId = b2_nullWorldId;

	m_textLine = 30;
	m_textIncrement = 22;
	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;

	m_groundBodyId = b2_nullBodyId;

	m_maxProfile = {};
	m_totalProfile = {};

	g_randomSeed = RAND_SEED;

	CreateWorld();
	TestMathCpp();
}

Sample::~Sample()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	b2DestroyWorld( m_worldId );

	delete m_scheduler;
	delete[] m_tasks;
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
	worldDef.enqueueTask = EnqueueTask;
	worldDef.finishTask = FinishTask;
	worldDef.userTaskContext = this;
	worldDef.enableSleep = m_context->enableSleep;
	m_worldId = b2CreateWorld( &worldDef );
}

void Sample::DrawTitle( const char* string )
{
	m_context->draw.DrawString( 5, 5, string );
	m_textLine = int( 26.0f );
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

		// Query the world for overlapping shapes.
		QueryContext queryContext = { p, b2_nullBodyId };
		b2World_OverlapAABB( m_worldId, box, b2DefaultQueryFilter(), QueryCallback, &queryContext );

		if ( B2_IS_NON_NULL( queryContext.bodyId ) )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			m_groundBodyId = b2CreateBody( m_worldId, &bodyDef );

			b2MouseJointDef mouseDef = b2DefaultMouseJointDef();
			mouseDef.bodyIdA = m_groundBodyId;
			mouseDef.bodyIdB = queryContext.bodyId;
			mouseDef.target = p;
			mouseDef.hertz = 10.0f;
			mouseDef.dampingRatio = 0.7f;
			mouseDef.maxForce = 1000.0f * b2Body_GetMass( queryContext.bodyId ) * b2Length(b2World_GetGravity(m_worldId));
			m_mouseJointId = b2CreateMouseJoint( m_worldId, &mouseDef );

			b2Body_SetAwake( queryContext.bodyId, true );
		}
	}
}

void Sample::MouseUp( b2Vec2 p, int button )
{
	if ( b2Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = b2_nullJointId;
	}

	if ( B2_IS_NON_NULL( m_mouseJointId ) && button == GLFW_MOUSE_BUTTON_1 )
	{
		b2DestroyJoint( m_mouseJointId );
		m_mouseJointId = b2_nullJointId;

		b2DestroyBody( m_groundBodyId );
		m_groundBodyId = b2_nullBodyId;
	}
}

void Sample::MouseMove( b2Vec2 p )
{
	if ( b2Joint_IsValid( m_mouseJointId ) == false )
	{
		// The world or attached body was destroyed.
		m_mouseJointId = b2_nullJointId;
	}

	if ( B2_IS_NON_NULL( m_mouseJointId ) )
	{
		b2MouseJoint_SetTarget( m_mouseJointId, p );
		b2BodyId bodyIdB = b2Joint_GetBodyB( m_mouseJointId );
		b2Body_SetAwake( bodyIdB, true );
	}
}

void Sample::DrawTextLine( const char* text, ... )
{
	va_list arg;
	va_start( arg, text );
	ImGui::Begin( "Overlay", nullptr,
				  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
					  ImGuiWindowFlags_NoScrollbar );
	ImGui::PushFont( m_context->draw.m_regularFont );
	ImGui::SetCursorPos( ImVec2( 5.0f, float( m_textLine ) ) );
	ImGui::TextColoredV( ImColor( 230, 153, 153, 255 ), text, arg );
	ImGui::PopFont();
	ImGui::End();
	va_end( arg );

	m_textLine += m_textIncrement;
}

void Sample::ResetProfile()
{
	m_totalProfile = {};
	m_maxProfile = {};
	m_stepCount = 0;
}

void Sample::Step(  )
{
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

		if ( m_context->draw.m_showUI )
		{
			DrawTextLine( "****PAUSED****" );
			m_textLine += m_textIncrement;
		}
	}

	m_context->draw.m_debugDraw.drawingBounds = m_context->camera.GetViewBounds();
	m_context->draw.m_debugDraw.useDrawingBounds = m_context->useCameraBounds;

	// todo testing
	// b2Transform t1 = {m_context->draw.m_debugDraw.drawingBounds.lowerBound, b2Rot_identity};
	// b2Transform t2 = {m_context->draw.m_debugDraw.drawingBounds.upperBound, b2Rot_identity};
	// m_context->draw.DrawSolidCircle(t1, b2Vec2_zero, 1.0f, {1.0f, 0.0f, 0.0f, 1.0f});
	// m_context->draw.DrawSolidCircle(t2, b2Vec2_zero, 1.0f, {1.0f, 0.0f, 0.0f, 1.0f});

	m_context->draw.m_debugDraw.drawShapes = m_context->drawShapes;
	m_context->draw.m_debugDraw.drawJoints = m_context->drawJoints;
	m_context->draw.m_debugDraw.drawJointExtras = m_context->drawJointExtras;
	m_context->draw.m_debugDraw.drawBounds = m_context->drawBounds;
	m_context->draw.m_debugDraw.drawMass = m_context->drawMass;
	m_context->draw.m_debugDraw.drawBodyNames = m_context->drawBodyNames;
	m_context->draw.m_debugDraw.drawContacts = m_context->drawContactPoints;
	m_context->draw.m_debugDraw.drawGraphColors = m_context->drawGraphColors;
	m_context->draw.m_debugDraw.drawContactNormals = m_context->drawContactNormals;
	m_context->draw.m_debugDraw.drawContactImpulses = m_context->drawContactImpulses;
	m_context->draw.m_debugDraw.drawContactFeatures = m_context->drawContactFeatures;
	m_context->draw.m_debugDraw.drawFrictionImpulses = m_context->drawFrictionImpulses;
	m_context->draw.m_debugDraw.drawIslands = m_context->drawIslands;

	b2World_EnableSleeping( m_worldId, m_context->enableSleep );
	b2World_EnableWarmStarting( m_worldId, m_context->enableWarmStarting );
	b2World_EnableContinuous( m_worldId, m_context->enableContinuous );

	for ( int i = 0; i < 1; ++i )
	{
		b2World_Step( m_worldId, timeStep, m_context->subStepCount );
		m_taskCount = 0;
	}

	b2World_Draw( m_worldId, &m_context->draw.m_debugDraw );

	if ( timeStep > 0.0f )
	{
		++m_stepCount;
	}

	if ( m_context->drawCounters )
	{
		b2Counters s = b2World_GetCounters( m_worldId );

		DrawTextLine( "bodies/shapes/contacts/joints = %d/%d/%d/%d", s.bodyCount, s.shapeCount, s.contactCount, s.jointCount );
		DrawTextLine( "islands/tasks = %d/%d", s.islandCount, s.taskCount );
		DrawTextLine( "tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight );

		int totalCount = 0;
		char buffer[256] = { 0 };
		int colorCount = sizeof( s.colorCounts ) / sizeof( s.colorCounts[0] );

		// todo fix this
		int offset = snprintf( buffer, 256, "colors: " );
		for ( int i = 0; i < colorCount; ++i )
		{
			offset += snprintf( buffer + offset, 256 - offset, "%d/", s.colorCounts[i] );
			totalCount += s.colorCounts[i];
		}
		snprintf( buffer + offset, 256 - offset, "[%d]", totalCount );
		DrawTextLine( buffer );
		DrawTextLine( "stack allocator size = %d K", s.stackUsed / 1024 );
		DrawTextLine( "total allocation = %d K", s.byteCount / 1024 );
	}

	// Track maximum profile times
	{
		b2Profile p = b2World_GetProfile( m_worldId );
		m_maxProfile.step = b2MaxFloat( m_maxProfile.step, p.step );
		m_maxProfile.pairs = b2MaxFloat( m_maxProfile.pairs, p.pairs );
		m_maxProfile.collide = b2MaxFloat( m_maxProfile.collide, p.collide );
		m_maxProfile.solve = b2MaxFloat( m_maxProfile.solve, p.solve );
		m_maxProfile.mergeIslands = b2MaxFloat( m_maxProfile.mergeIslands, p.mergeIslands );
		m_maxProfile.prepareStages = b2MaxFloat( m_maxProfile.prepareStages, p.prepareStages );
		m_maxProfile.solveConstraints = b2MaxFloat( m_maxProfile.solveConstraints, p.solveConstraints );
		m_maxProfile.prepareConstraints = b2MaxFloat( m_maxProfile.prepareConstraints, p.prepareConstraints );
		m_maxProfile.integrateVelocities = b2MaxFloat( m_maxProfile.integrateVelocities, p.integrateVelocities );
		m_maxProfile.warmStart = b2MaxFloat( m_maxProfile.warmStart, p.warmStart );
		m_maxProfile.solveImpulses = b2MaxFloat( m_maxProfile.solveImpulses, p.solveImpulses );
		m_maxProfile.integratePositions = b2MaxFloat( m_maxProfile.integratePositions, p.integratePositions );
		m_maxProfile.relaxImpulses = b2MaxFloat( m_maxProfile.relaxImpulses, p.relaxImpulses );
		m_maxProfile.applyRestitution = b2MaxFloat( m_maxProfile.applyRestitution, p.applyRestitution );
		m_maxProfile.storeImpulses = b2MaxFloat( m_maxProfile.storeImpulses, p.storeImpulses );
		m_maxProfile.transforms = b2MaxFloat( m_maxProfile.transforms, p.transforms );
		m_maxProfile.splitIslands = b2MaxFloat( m_maxProfile.splitIslands, p.splitIslands );
		m_maxProfile.hitEvents = b2MaxFloat( m_maxProfile.hitEvents, p.hitEvents );
		m_maxProfile.refit = b2MaxFloat( m_maxProfile.refit, p.refit );
		m_maxProfile.bullets = b2MaxFloat( m_maxProfile.bullets, p.bullets );
		m_maxProfile.sleepIslands = b2MaxFloat( m_maxProfile.sleepIslands, p.sleepIslands );
		m_maxProfile.sensors = b2MaxFloat( m_maxProfile.sensors, p.sensors );

		m_totalProfile.step += p.step;
		m_totalProfile.pairs += p.pairs;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.mergeIslands += p.mergeIslands;
		m_totalProfile.prepareStages += p.prepareStages;
		m_totalProfile.solveConstraints += p.solveConstraints;
		m_totalProfile.prepareConstraints += p.prepareConstraints;
		m_totalProfile.integrateVelocities += p.integrateVelocities;
		m_totalProfile.warmStart += p.warmStart;
		m_totalProfile.solveImpulses += p.solveImpulses;
		m_totalProfile.integratePositions += p.integratePositions;
		m_totalProfile.relaxImpulses += p.relaxImpulses;
		m_totalProfile.applyRestitution += p.applyRestitution;
		m_totalProfile.storeImpulses += p.storeImpulses;
		m_totalProfile.transforms += p.transforms;
		m_totalProfile.splitIslands += p.splitIslands;
		m_totalProfile.hitEvents += p.hitEvents;
		m_totalProfile.refit += p.refit;
		m_totalProfile.bullets += p.bullets;
		m_totalProfile.sleepIslands += p.sleepIslands;
		m_totalProfile.sensors += p.sensors;
	}

	if ( m_context->drawProfile )
	{
		b2Profile p = b2World_GetProfile( m_worldId );

		b2Profile aveProfile = {};
		if ( m_stepCount > 0 )
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.pairs = scale * m_totalProfile.pairs;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.mergeIslands = scale * m_totalProfile.mergeIslands;
			aveProfile.prepareStages = scale * m_totalProfile.prepareStages;
			aveProfile.solveConstraints = scale * m_totalProfile.solveConstraints;
			aveProfile.prepareConstraints = scale * m_totalProfile.prepareConstraints;
			aveProfile.integrateVelocities = scale * m_totalProfile.integrateVelocities;
			aveProfile.warmStart = scale * m_totalProfile.warmStart;
			aveProfile.solveImpulses = scale * m_totalProfile.solveImpulses;
			aveProfile.integratePositions = scale * m_totalProfile.integratePositions;
			aveProfile.relaxImpulses = scale * m_totalProfile.relaxImpulses;
			aveProfile.applyRestitution = scale * m_totalProfile.applyRestitution;
			aveProfile.storeImpulses = scale * m_totalProfile.storeImpulses;
			aveProfile.transforms = scale * m_totalProfile.transforms;
			aveProfile.splitIslands = scale * m_totalProfile.splitIslands;
			aveProfile.hitEvents = scale * m_totalProfile.hitEvents;
			aveProfile.refit = scale * m_totalProfile.refit;
			aveProfile.bullets = scale * m_totalProfile.bullets;
			aveProfile.sleepIslands = scale * m_totalProfile.sleepIslands;
			aveProfile.sensors = scale * m_totalProfile.sensors;
		}

		DrawTextLine( "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step );
		DrawTextLine( "pairs [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.pairs, aveProfile.pairs, m_maxProfile.pairs );
		DrawTextLine( "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide );
		DrawTextLine( "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve );
		DrawTextLine( "> merge islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.mergeIslands, aveProfile.mergeIslands,
					  m_maxProfile.mergeIslands );
		DrawTextLine( "> prepare tasks [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.prepareStages, aveProfile.prepareStages,
					  m_maxProfile.prepareStages );
		DrawTextLine( "> solve constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveConstraints, aveProfile.solveConstraints,
					  m_maxProfile.solveConstraints );
		DrawTextLine( ">> prepare constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.prepareConstraints,
					  aveProfile.prepareConstraints, m_maxProfile.prepareConstraints );
		DrawTextLine( ">> integrate velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.integrateVelocities,
					  aveProfile.integrateVelocities, m_maxProfile.integrateVelocities );
		DrawTextLine( ">> warm start [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.warmStart, aveProfile.warmStart,
					  m_maxProfile.warmStart );
		DrawTextLine( ">> solve impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveImpulses, aveProfile.solveImpulses,
					  m_maxProfile.solveImpulses );
		DrawTextLine( ">> integrate positions [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.integratePositions,
					  aveProfile.integratePositions, m_maxProfile.integratePositions );
		DrawTextLine( ">> relax impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.relaxImpulses, aveProfile.relaxImpulses,
					  m_maxProfile.relaxImpulses );
		DrawTextLine( ">> apply restitution [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.applyRestitution, aveProfile.applyRestitution,
					  m_maxProfile.applyRestitution );
		DrawTextLine( ">> store impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.storeImpulses, aveProfile.storeImpulses,
					  m_maxProfile.storeImpulses );
		DrawTextLine( ">> split islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.splitIslands, aveProfile.splitIslands,
					  m_maxProfile.splitIslands );
		DrawTextLine( "> update transforms [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.transforms, aveProfile.transforms,
					  m_maxProfile.transforms );
		DrawTextLine( "> hit events [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.hitEvents, aveProfile.hitEvents,
					  m_maxProfile.hitEvents );
		DrawTextLine( "> refit BVH [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.refit, aveProfile.refit, m_maxProfile.refit );
		DrawTextLine( "> sleep islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.sleepIslands, aveProfile.sleepIslands,
					  m_maxProfile.sleepIslands );
		DrawTextLine( "> bullets [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.bullets, aveProfile.bullets, m_maxProfile.bullets );
		DrawTextLine( "sensors [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.sensors, aveProfile.sensors, m_maxProfile.sensors );
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
		g_sampleEntries[index] = { category, name, fcn };
		++g_sampleCount;
		return index;
	}

	return -1;
}
