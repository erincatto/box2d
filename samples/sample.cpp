// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "sample.h"

#include "TaskScheduler.h"
#include "draw.h"
#include "imgui.h"
#include "random.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <stdio.h>
#include <ctype.h>

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

Sample::Sample( Settings& settings )
{
	m_scheduler = new enki::TaskScheduler;
	m_scheduler->Initialize( settings.workerCount );

	m_tasks = new SampleTask[m_maxTasks];
	m_taskCount = 0;

	m_threadCount = 1 + settings.workerCount;

	m_worldId = b2_nullWorldId;

	m_textLine = 30;
	m_textIncrement = 22;
	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;

	m_groundBodyId = b2_nullBodyId;

	m_maxProfile = {};
	m_totalProfile = {};

	g_seed = RAND_SEED;

	m_settings = &settings;

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
	worldDef.workerCount = m_settings->workerCount;
	worldDef.enqueueTask = EnqueueTask;
	worldDef.finishTask = FinishTask;
	worldDef.userTaskContext = this;
	worldDef.enableSleep = m_settings->enableSleep;

	m_worldId = b2CreateWorld( &worldDef );
}

void Sample::DrawTitle( const char* string )
{
	g_draw.DrawString( 5, 5, string );
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
			mouseDef.hertz = 5.0f;
			mouseDef.dampingRatio = 0.7f;
			mouseDef.maxForce = 1000.0f * b2Body_GetMass( queryContext.bodyId );
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
	ImGui::PushFont( g_draw.m_regularFont );
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

void Sample::Step( Settings& settings )
{
	float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : 0.0f;

	if ( settings.pause )
	{
		if ( settings.singleStep )
		{
			settings.singleStep = false;
		}
		else
		{
			timeStep = 0.0f;
		}

		if ( g_draw.m_showUI )
		{
			g_draw.DrawString( 5, m_textLine, "****PAUSED****" );
			m_textLine += m_textIncrement;
		}
	}

	g_draw.m_debugDraw.drawingBounds = g_camera.GetViewBounds();
	g_draw.m_debugDraw.useDrawingBounds = settings.useCameraBounds;

	// todo testing
	// b2Transform t1 = {g_draw.m_debugDraw.drawingBounds.lowerBound, b2Rot_identity};
	// b2Transform t2 = {g_draw.m_debugDraw.drawingBounds.upperBound, b2Rot_identity};
	// g_draw.DrawSolidCircle(t1, b2Vec2_zero, 1.0f, {1.0f, 0.0f, 0.0f, 1.0f});
	// g_draw.DrawSolidCircle(t2, b2Vec2_zero, 1.0f, {1.0f, 0.0f, 0.0f, 1.0f});

	g_draw.m_debugDraw.drawShapes = settings.drawShapes;
	g_draw.m_debugDraw.drawJoints = settings.drawJoints;
	g_draw.m_debugDraw.drawJointExtras = settings.drawJointExtras;
	g_draw.m_debugDraw.drawAABBs = settings.drawAABBs;
	g_draw.m_debugDraw.drawMass = settings.drawMass;
	g_draw.m_debugDraw.drawBodyNames = settings.drawBodyNames;
	g_draw.m_debugDraw.drawContacts = settings.drawContactPoints;
	g_draw.m_debugDraw.drawGraphColors = settings.drawGraphColors;
	g_draw.m_debugDraw.drawContactNormals = settings.drawContactNormals;
	g_draw.m_debugDraw.drawContactImpulses = settings.drawContactImpulses;
	g_draw.m_debugDraw.drawFrictionImpulses = settings.drawFrictionImpulses;

	b2World_EnableSleeping( m_worldId, settings.enableSleep );
	b2World_EnableWarmStarting( m_worldId, settings.enableWarmStarting );
	b2World_EnableContinuous( m_worldId, settings.enableContinuous );

	for ( int i = 0; i < 1; ++i )
	{
		b2World_Step( m_worldId, timeStep, settings.subStepCount );
		m_taskCount = 0;
	}

	b2World_Draw( m_worldId, &g_draw.m_debugDraw );

	if ( timeStep > 0.0f )
	{
		++m_stepCount;
	}

	if ( settings.drawCounters )
	{
		b2Counters s = b2World_GetCounters( m_worldId );

		g_draw.DrawString( 5, m_textLine, "bodies/shapes/contacts/joints = %d/%d/%d/%d", s.bodyCount, s.shapeCount,
						   s.contactCount, s.jointCount );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "islands/tasks = %d/%d", s.islandCount, s.taskCount );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "tree height static/movable = %d/%d", s.staticTreeHeight, s.treeHeight );
		m_textLine += m_textIncrement;

		int totalCount = 0;
		char buffer[256] = { 0 };
		static_assert( std::size( s.colorCounts ) == 12 );

		// todo fix this
		int offset = snprintf( buffer, 256, "colors: " );
		for ( int i = 0; i < 12; ++i )
		{
			offset += snprintf( buffer + offset, 256 - offset, "%d/", s.colorCounts[i] );
			totalCount += s.colorCounts[i];
		}
		snprintf( buffer + offset, 256 - offset, "[%d]", totalCount );
		g_draw.DrawString( 5, m_textLine, buffer );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "stack allocator size = %d K", s.stackUsed / 1024 );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "total allocation = %d K", s.byteCount / 1024 );
		m_textLine += m_textIncrement;
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

	if ( settings.drawProfile )
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
