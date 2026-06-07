// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "draw.h"
#include "imgui.h"

#include "box2d/box2d.h"

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
	bool showMetrics = false;

	// Set by Ctrl+O; consumed by UpdateSampleUI to open the fuzzy sample picker.
	bool openSamplePicker = false;

	// Path the active recording is saved to when recording stops.
	char recordingFile[256] = "recording.b2rec";

	// Path the Replay menu hands to the viewer. Kept apart from the record path so opening a
	// file to view never moves where the next recording lands.
	char replayFile[256] = "";

	// These are persisted
	int sampleIndex = 0;
	bool newUser = true;
};

class Sample
{
public:
	// createWorld false lets a subclass that supplies its own world (e.g. the replay viewer)
	// skip the throwaway world the base would otherwise build and immediately discard
	explicit Sample( SampleContext* context, bool createWorld = true );
	virtual ~Sample();

	void CreateWorld();

	// Snapshot the live world and begin recording into a host-owned buffer
	void StartRecording();

	// Stop the active recording if any, save it to context->recordingFile, and free it
	void FinishRecording();

	void ResetText();
	virtual void Step();

	virtual bool DrawControls()
	{
		return false;
	}

	// Allow solver controls to be hidden by a sample.
	virtual bool HasSolverControls() const
	{
		return true;
	}

	// Allow a sample to add extra tabs to the metrics window.
	virtual void DrawMetricsTab()
	{
	}

	virtual void Keyboard( int )
	{
	}
	virtual void MouseDown( b2Vec2 p, int button, int mod );
	virtual void MouseUp( b2Vec2 p, int button );
	virtual void MouseMove( b2Vec2 p );

	void DrawMetrics();
	void DrawHud( float frameTime );
	void DrawScreenTextLine( const char* text, ... );
	void ResetProfile();

	static int ParsePath( const char* svgPath, b2Vec2 offset, b2Vec2* points, int capacity, float scale, bool reverseOrder );

	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	static constexpr int m_maxTasks = 512;
	static constexpr int m_maxThreads = 64;
	static constexpr int m_profileCapacity = 512;

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
	b2Recording* m_recording; // active recording buffer, owned here; NULL when not recording
	int m_recordStartStep;	  // step the active recording began at, for the UI indicator
	b2JointId m_mouseJointId;
	b2Vec2 m_mousePoint;
	float m_mouseForceScale;
	int m_stepCount;
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
int RegisterReplay( const char* category, const char* name, SampleCreateFcn* fcn );
void SelectSample( SampleContext* context, int selection, bool restart );
void DrawUI( SampleContext* context, float frameTime );

struct SampleEntry
{
	const char* category;
	const char* name;
	SampleCreateFcn* createFcn;
	SampleCapacityFcn* capacityFcn;
};

inline ImVec4 MakeColor( b2HexColor hexColor )
{
	ImU32 color = IM_COL32( ( hexColor >> 16 ) & 0xFF, ( hexColor >> 8 ) & 0xFF, hexColor & 0xFF, 255 );
	return ImGui::ColorConvertU32ToFloat4( color );
}

#define MAX_SAMPLES 256
extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int g_sampleCount;
extern int g_replayIndex;
