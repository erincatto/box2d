// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "draw.h"

#include "box2d/id.h"
#include "box2d/types.h"

#define ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

struct ImFont;

struct SampleContext
{
	void Save();
	void Load();

	struct GLFWwindow* window = nullptr;
	Camera camera;
	Draw* draw;
	class Sample* sample = nullptr;
	b2Capacity capacity;
	b2DebugDraw debugDraw;
	float uiScale = 1.0f;
	float hertz = 60.0f;
	float recycleDistance = 0.05f;
	int subStepCount = 4;
	int workerCount = 1;
	bool restart = false;
	bool pause = false;
	bool singleStep = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSleep = true;
	bool showUI = true;

	// Diagnostics drawer visibility. D toggles.
	bool showDiagnostics = false;

	// Set by Ctrl+O; consumed by UpdateSampleUI to open the fuzzy sample picker.
	bool openSamplePicker = false;

	// These are persisted
	int sampleIndex = 0;
};

class Sample
{
public:
	explicit Sample( SampleContext* context );
	virtual ~Sample();

	void CreateWorld();

	void ResetText();
	virtual void Step();

	virtual void UpdateGui();
	virtual void BuildSamplePanel()
	{
	}
	virtual void Keyboard( int )
	{
	}
	virtual void MouseDown( b2Vec2 p, int button, int mod );
	virtual void MouseUp( b2Vec2 p, int button );
	virtual void MouseMove( b2Vec2 p );

	void DrawTextLine( const char* text, ... );
	void DrawColoredTextLine( b2HexColor color, const char* text, ... );
	void DrawScreenTextLine( const char* text, ... );
	void DrawColoredScreenTextLine( b2HexColor color, const char* text, ... );
	void ResetProfile();
	void ShiftOrigin( b2Vec2 newOrigin );

	static int ParsePath( const char* svgPath, b2Vec2 offset, b2Vec2* points, int capacity, float scale, bool reverseOrder );

	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	static constexpr int m_maxTasks = 512;
	static constexpr int m_maxThreads = 64;
	static constexpr int m_profileCapacity = 512;
	static constexpr int m_maxHudLines = 64;

	struct HudLine
	{
		b2HexColor color;
		char text[256];
	};

#ifdef NDEBUG
	static constexpr bool m_isDebug = false;
#else
	static constexpr bool m_isDebug = true;
#endif

	SampleContext* m_context;
	Camera* m_camera;
	Draw* m_draw;

	b2BodyId m_mouseBodyId;

	b2WorldId m_worldId;
	b2JointId m_mouseJointId;
	b2Vec2 m_mousePoint;
	float m_mouseForceScale;
	int m_stepCount;

	HudLine m_hudLines[m_maxHudLines];
	int m_hudLineCount;
	float m_screenTextY;

	b2Profile m_profiles[m_profileCapacity];
	int m_currentProfileIndex;
	uint64_t m_profileReadIndex;
	uint64_t m_profileWriteIndex;

	bool m_didStep;
};

typedef Sample* SampleCreateFcn( SampleContext* context );
typedef b2Capacity SampleCapacityFcn( void );

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn );
int RegisterSampleWithCapacity( const char* category, const char* name, SampleCreateFcn* fcn, SampleCapacityFcn* capacityFcn );
void SelectSample( SampleContext* context, int selection, bool restart );
void UpdateSampleUI( SampleContext* context );

struct SampleEntry
{
	const char* category;
	const char* name;
	SampleCreateFcn* createFcn;
	SampleCapacityFcn* capacityFcn;
};

#define MAX_SAMPLES 256
extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;
