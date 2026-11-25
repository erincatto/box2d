// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/collision.h"
#include "box2d/id.h"
#include "box2d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Wave Drop scene, shared by the Continuous / Wave Drop sample and the headless wave soak. A 2D port
// of Box3D's Mesh Drop. A generation drops a grid of small bodies with random velocities on a wave shaped
// chain loop and steps until they sleep. A body that falls out of the world tunneled. The depth tracker
// measures how far each body sinks into the wave, which flags contact trouble far more often than tunnels
// do. The grid is stacked in height, so the top rows land faster than the bottom rows. The rough wave has
// bumps smaller than the bodies, so a body touches many chain segments with different normals.

typedef enum WaveDropShape
{
	e_waveDropBox,
	e_waveDropCapsule,
	e_waveDropRounded,
	e_waveDropCircle,
	e_waveDropShapeCount
} WaveDropShape;

// Bodies per axis. A debug build drops a smaller patch at the same spacing so the samples app stays
// responsive. The soak tool runs in release and keeps the full grid.
#define WAVE_DROP_GRID_COUNT 32
#define WAVE_DROP_BODY_COUNT ( WAVE_DROP_GRID_COUNT * WAVE_DROP_GRID_COUNT )
#define WAVE_DROP_DEPTH_LEVEL_COUNT 4

// Most wave cells, reached by the rough wave
#define WAVE_DROP_MAX_WAVE_COUNT 640

extern const char* g_waveDropShapeNames[e_waveDropShapeCount];

// Sink depth thresholds in linear slop
extern const float g_waveDropDepthLevels[WAVE_DROP_DEPTH_LEVEL_COUNT];

typedef struct WaveDropSettings
{
	WaveDropShape shape;
	float amplitude;

	// Fine bumps instead of the broad wave. The amplitude should be a few centimeters.
	bool rough;

	// Let the bodies collide with each other. Off isolates chain collision.
	bool collide;

	// Create only this grid index, or -1 for all. Without body collision a body moves the same alone.
	int soloIndex;
} WaveDropSettings;

WaveDropSettings GetDefaultWaveDropSettings( void );

float GetWaveDropDefaultAmplitude( bool rough );

// Hashes the base before adding the generation, so nearby base seeds share no generations
uint32_t GetWaveDropSeed( uint32_t baseSeed, int generation );

// Steps a generation may take to fall asleep
int GetWaveDropStepLimit( WaveDropShape shape );

// A body below this height tunneled
float GetWaveDropTunnelHeight( void );

// Creates a static body with the wave chain and two walls
b2BodyId CreateWaveDropGround( b2WorldId worldId, WaveDropSettings settings );

// Creates the bodies of one generation in grid order. Bodies left out by the solo index are null. The
// body user data encodes the grid index, see GetWaveDropBodyIndex.
void CreateWaveDropBodies( b2WorldId worldId, WaveDropSettings settings, uint32_t seed, b2BodyId* bodies );

// Grid index from body user data, or -1 if the body is not part of the drop
int GetWaveDropBodyIndex( void* userData );

typedef struct WaveDropDepth
{
	// Wave height at each chain vertex
	float heights[WAVE_DROP_MAX_WAVE_COUNT + 1];
	int waveCount;
	float cellWidth;

	// Collision points in the body frame, rounded by the radius
	b2Vec2 probes[B2_MAX_POLYGON_VERTICES];
	int probeCount;
	float radius;
	float boundingRadius;
	float amplitude;

	// Deepest sink of each body so far, positive when inside the wave
	float bodyDepths[WAVE_DROP_BODY_COUNT];

	float maxDepth;
	int maxIndex;
	int maxStep;
	int stepCount;
} WaveDropDepth;

// Starts tracking a new generation
void ResetWaveDropDepth( WaveDropDepth* depth, WaveDropSettings settings );

// Call after each world step
void UpdateWaveDropDepth( b2WorldId worldId, WaveDropDepth* depth );

// Number of bodies that sank deeper than each depth level
void GetWaveDropDepthCounts( const WaveDropDepth* depth, int counts[WAVE_DROP_DEPTH_LEVEL_COUNT] );

#ifdef __cplusplus
}
#endif
