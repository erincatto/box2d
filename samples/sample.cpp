// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sample.h"

#include "draw.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <stdio.h>
#include <string.h>

static void* EnqueueTask( b2TaskCallback* task, int32_t itemCount, int32_t minRange, void* taskContext, void* userContext )
{
	Sample* sample = static_cast<Sample*>( userContext );
	if ( sample->m_taskCount < maxTasks )
	{
		SampleTask& sampleTask = sample->m_tasks[sample->m_taskCount];
		sampleTask.m_SetSize = itemCount;
		sampleTask.m_MinRange = minRange;
		sampleTask.m_task = task;
		sampleTask.m_taskContext = taskContext;
		sample->m_scheduler.AddTaskSetToPipe( &sampleTask );
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
		sample->m_scheduler.WaitforTask( sampleTask );
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
	m_scheduler.Initialize( settings.workerCount );
	m_taskCount = 0;

	m_threadCount = 1 + settings.workerCount;

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = settings.workerCount;
	worldDef.enqueueTask = EnqueueTask;
	worldDef.finishTask = FinishTask;
	worldDef.userTaskContext = this;
	worldDef.enableSleep = settings.enableSleep;

	m_worldId = b2CreateWorld( &worldDef );
	m_textLine = 30;
	m_textIncrement = 18;
	m_mouseJointId = b2_nullJointId;

	m_stepCount = 0;

	m_groundBodyId = b2_nullBodyId;

	m_maxProfile = {};
	m_totalProfile = {};

	TestMathCpp();
}

Sample::~Sample()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	b2DestroyWorld( m_worldId );
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

		g_draw.DrawString( 5, m_textLine, "****PAUSED****" );
		m_textLine += m_textIncrement;
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
		m_maxProfile.buildIslands = b2MaxFloat( m_maxProfile.buildIslands, p.buildIslands );
		m_maxProfile.solveConstraints = b2MaxFloat( m_maxProfile.solveConstraints, p.solveConstraints );
		m_maxProfile.prepareTasks = b2MaxFloat( m_maxProfile.prepareTasks, p.prepareTasks );
		m_maxProfile.solverTasks = b2MaxFloat( m_maxProfile.solverTasks, p.solverTasks );
		m_maxProfile.prepareConstraints = b2MaxFloat( m_maxProfile.prepareConstraints, p.prepareConstraints );
		m_maxProfile.integrateVelocities = b2MaxFloat( m_maxProfile.integrateVelocities, p.integrateVelocities );
		m_maxProfile.warmStart = b2MaxFloat( m_maxProfile.warmStart, p.warmStart );
		m_maxProfile.solveVelocities = b2MaxFloat( m_maxProfile.solveVelocities, p.solveVelocities );
		m_maxProfile.integratePositions = b2MaxFloat( m_maxProfile.integratePositions, p.integratePositions );
		m_maxProfile.relaxVelocities = b2MaxFloat( m_maxProfile.relaxVelocities, p.relaxVelocities );
		m_maxProfile.applyRestitution = b2MaxFloat( m_maxProfile.applyRestitution, p.applyRestitution );
		m_maxProfile.storeImpulses = b2MaxFloat( m_maxProfile.storeImpulses, p.storeImpulses );
		m_maxProfile.finalizeBodies = b2MaxFloat( m_maxProfile.finalizeBodies, p.finalizeBodies );
		m_maxProfile.sleepIslands = b2MaxFloat( m_maxProfile.sleepIslands, p.sleepIslands );
		m_maxProfile.splitIslands = b2MaxFloat( m_maxProfile.splitIslands, p.splitIslands );
		m_maxProfile.hitEvents = b2MaxFloat( m_maxProfile.hitEvents, p.hitEvents );
		m_maxProfile.broadphase = b2MaxFloat( m_maxProfile.broadphase, p.broadphase );
		m_maxProfile.continuous = b2MaxFloat( m_maxProfile.continuous, p.continuous );

		m_totalProfile.step += p.step;
		m_totalProfile.pairs += p.pairs;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.buildIslands += p.buildIslands;
		m_totalProfile.solveConstraints += p.solveConstraints;
		m_totalProfile.prepareTasks += p.prepareTasks;
		m_totalProfile.solverTasks += p.solverTasks;
		m_totalProfile.prepareConstraints += p.prepareConstraints;
		m_totalProfile.integrateVelocities += p.integrateVelocities;
		m_totalProfile.warmStart += p.warmStart;
		m_totalProfile.solveVelocities += p.solveVelocities;
		m_totalProfile.integratePositions += p.integratePositions;
		m_totalProfile.relaxVelocities += p.relaxVelocities;
		m_totalProfile.applyRestitution += p.applyRestitution;
		m_totalProfile.storeImpulses += p.storeImpulses;
		m_totalProfile.finalizeBodies += p.finalizeBodies;
		m_totalProfile.sleepIslands += p.sleepIslands;
		m_totalProfile.splitIslands += p.splitIslands;
		m_totalProfile.hitEvents += p.hitEvents;
		m_totalProfile.broadphase += p.broadphase;
		m_totalProfile.continuous += p.continuous;
	}

	if ( settings.drawProfile )
	{
		b2Profile p = b2World_GetProfile( m_worldId );

		b2Profile aveProfile;
		memset( &aveProfile, 0, sizeof( b2Profile ) );
		if ( m_stepCount > 0 )
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.pairs = scale * m_totalProfile.pairs;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.buildIslands = scale * m_totalProfile.buildIslands;
			aveProfile.solveConstraints = scale * m_totalProfile.solveConstraints;
			aveProfile.prepareTasks = scale * m_totalProfile.prepareTasks;
			aveProfile.solverTasks = scale * m_totalProfile.solverTasks;
			aveProfile.prepareConstraints = scale * m_totalProfile.prepareConstraints;
			aveProfile.integrateVelocities = scale * m_totalProfile.integrateVelocities;
			aveProfile.warmStart = scale * m_totalProfile.warmStart;
			aveProfile.solveVelocities = scale * m_totalProfile.solveVelocities;
			aveProfile.integratePositions = scale * m_totalProfile.integratePositions;
			aveProfile.relaxVelocities = scale * m_totalProfile.relaxVelocities;
			aveProfile.applyRestitution = scale * m_totalProfile.applyRestitution;
			aveProfile.storeImpulses = scale * m_totalProfile.storeImpulses;
			aveProfile.finalizeBodies = scale * m_totalProfile.finalizeBodies;
			aveProfile.sleepIslands = scale * m_totalProfile.sleepIslands;
			aveProfile.splitIslands = scale * m_totalProfile.splitIslands;
			aveProfile.hitEvents = scale * m_totalProfile.hitEvents;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
			aveProfile.continuous = scale * m_totalProfile.continuous;
		}

		g_draw.DrawString( 5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step,
						   m_maxProfile.step );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "pairs [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.pairs, aveProfile.pairs,
						   m_maxProfile.pairs );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide,
						   m_maxProfile.collide );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve,
						   m_maxProfile.solve );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "builds island [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.buildIslands,
						   aveProfile.buildIslands, m_maxProfile.buildIslands );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "solve constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveConstraints,
						   aveProfile.solveConstraints, m_maxProfile.solveConstraints );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "prepare tasks [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.prepareTasks,
						   aveProfile.prepareTasks, m_maxProfile.prepareTasks );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "solver tasks [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solverTasks,
						   aveProfile.solverTasks, m_maxProfile.solverTasks );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "prepare constraints [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.prepareConstraints,
						   aveProfile.prepareConstraints, m_maxProfile.prepareConstraints );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "integrate velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.integrateVelocities,
						   aveProfile.integrateVelocities, m_maxProfile.integrateVelocities );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "warm start [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.warmStart, aveProfile.warmStart,
						   m_maxProfile.warmStart );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "solve velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocities,
						   aveProfile.solveVelocities, m_maxProfile.solveVelocities );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "integrate positions [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.integratePositions,
						   aveProfile.integratePositions, m_maxProfile.integratePositions );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "relax velocities [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.relaxVelocities,
						   aveProfile.relaxVelocities, m_maxProfile.relaxVelocities );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "apply restitution [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.applyRestitution,
						   aveProfile.applyRestitution, m_maxProfile.applyRestitution );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "store impulses [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.storeImpulses,
						   aveProfile.storeImpulses, m_maxProfile.storeImpulses );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "finalize bodies [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.finalizeBodies,
						   aveProfile.finalizeBodies, m_maxProfile.finalizeBodies );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "sleep islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.sleepIslands,
						   aveProfile.sleepIslands, m_maxProfile.sleepIslands );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "split islands [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.splitIslands,
						   aveProfile.splitIslands, m_maxProfile.splitIslands );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "hit events [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.hitEvents, aveProfile.hitEvents,
						   m_maxProfile.hitEvents );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase,
						   m_maxProfile.broadphase );
		m_textLine += m_textIncrement;
		g_draw.DrawString( 5, m_textLine, "continuous collision [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.continuous,
						   aveProfile.continuous, m_maxProfile.continuous );
		m_textLine += m_textIncrement;
	}
}

void Sample::ShiftOrigin( b2Vec2 newOrigin )
{
	// m_world->ShiftOrigin(newOrigin);
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

b2Polygon RandomPolygon( float extent )
{
	b2Vec2 points[b2_maxPolygonVertices];
	int count = 3 + rand() % 6;
	for ( int i = 0; i < count; ++i )
	{
		points[i] = RandomVec2( -extent, extent );
	}

	b2Hull hull = b2ComputeHull( points, count );
	if ( hull.count > 0 )
	{
		return b2MakePolygon( &hull, 0.0f );
	}

	return b2MakeSquare( extent );
}
