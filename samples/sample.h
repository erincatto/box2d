// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"
#include "box2d/types.h"

// todo this include is slow
#include "TaskScheduler.h"

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

struct Settings;

#ifdef NDEBUG
constexpr bool g_sampleDebug = false;
#else
constexpr bool g_sampleDebug = true;
#endif

constexpr int32_t k_maxContactPoints = 12 * 2048;

struct ContactPoint
{
	b2ShapeId shapeIdA;
	b2ShapeId shapeIdB;
	b2Vec2 normal;
	b2Vec2 position;
	bool persisted;
	float normalImpulse;
	float tangentImpulse;
	float separation;
	int32_t constraintIndex;
	int32_t color;
};

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

constexpr int32_t maxTasks = 64;
constexpr int32_t maxThreads = 64;

class Sample
{
public:
	explicit Sample( Settings& settings );
	virtual ~Sample();

	void DrawTitle( const char* string );
	virtual void Step( Settings& settings );
	virtual void UpdateUI()
	{
	}
	virtual void Keyboard( int )
	{
	}
	virtual void MouseDown( b2Vec2 p, int button, int mod );
	virtual void MouseUp( b2Vec2 p, int button );
	virtual void MouseMove( b2Vec2 p );

	void ResetProfile();
	void ShiftOrigin( b2Vec2 newOrigin );

	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	enki::TaskScheduler m_scheduler;
	SampleTask m_tasks[maxTasks];
	int32_t m_taskCount;
	int m_threadCount;

	b2BodyId m_groundBodyId;

	// DestructionListener m_destructionListener;
	int32_t m_textLine;
	b2WorldId m_worldId;
	b2JointId m_mouseJointId;
	int32_t m_stepCount;
	int32_t m_textIncrement;
	b2Profile m_maxProfile;
	b2Profile m_totalProfile;
};

typedef Sample* SampleCreateFcn( Settings& settings );

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn );

struct SampleEntry
{
	const char* category;
	const char* name;
	SampleCreateFcn* createFcn;
};

#define MAX_SAMPLES 256
extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;
