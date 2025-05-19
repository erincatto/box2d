// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/id.h"
#include "box2d/types.h"
#include "draw.h"

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

namespace enki
{
class TaskScheduler;
};

struct SampleContext
{
	void Save();
	void Load();

	struct GLFWwindow* window = nullptr;
	Camera camera;
	Draw draw;
	float hertz = 60.0f;
	int subStepCount = 4;
	int workerCount = 1;
	bool restart = false;
	bool pause = false;
	bool singleStep = false;
	bool useCameraBounds = false;
	bool drawJointExtras = false;
	bool drawBounds = false;
	bool drawMass = false;
	bool drawBodyNames = false;
	bool drawContactPoints = false;
	bool drawContactNormals = false;
	bool drawContactImpulses = false;
	bool drawContactFeatures = false;
	bool drawFrictionImpulses = false;
	bool drawIslands = false;
	bool drawGraphColors = false;
	bool drawCounters = false;
	bool drawProfile = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSleep = true;

	// These are persisted
	int sampleIndex = 0;
	bool drawShapes = true;
	bool drawJoints = true;
};

class Sample
{
public:
	explicit Sample( SampleContext* context );
	virtual ~Sample();

	void CreateWorld( );

	void DrawTitle( const char* string );
	virtual void Step( );
	virtual void UpdateGui()
	{
	}
	virtual void Keyboard( int )
	{
	}
	virtual void MouseDown( b2Vec2 p, int button, int mod );
	virtual void MouseUp( b2Vec2 p, int button );
	virtual void MouseMove( b2Vec2 p );

	void DrawTextLine( const char* text, ... );
	void ResetProfile();
	void ShiftOrigin( b2Vec2 newOrigin );

	static int ParsePath( const char* svgPath, b2Vec2 offset, b2Vec2* points, int capacity, float scale, bool reverseOrder );

	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	static constexpr int m_maxTasks = 64;
	static constexpr int m_maxThreads = 64;

#ifdef NDEBUG
	static constexpr bool m_isDebug = false;
#else
	static constexpr bool m_isDebug = true;
#endif

	SampleContext* m_context;
	Camera* m_camera;
	Draw* m_draw;

	enki::TaskScheduler* m_scheduler;
	class SampleTask* m_tasks;
	int m_taskCount;
	int m_threadCount;

	b2BodyId m_groundBodyId;

	b2WorldId m_worldId;
	b2JointId m_mouseJointId;
	int m_stepCount;
	b2Profile m_maxProfile;
	b2Profile m_totalProfile;

private:
	int m_textLine;
	int m_textIncrement;
};

typedef Sample* SampleCreateFcn( SampleContext* context );

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
