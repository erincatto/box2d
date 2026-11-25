// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "wave_drop.h"

#include "box2d/box2d.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#define WAVE_COUNT 40
#define WAVE_CELL_WIDTH 1.0f
#define WAVE_ROW_FREQUENCY 0.1f
#define WAVE_COLUMN_FREQUENCY 0.2f
#define WAVE_AMPLITUDE 0.5f

// Bumps about 20 cm across, close to the curvature of the bodies, on cells a few times smaller. The cell
// width is a power of two so the chain vertices land where the depth tracker computes them.
#define ROUGH_WAVE_COUNT 640
#define ROUGH_CELL_WIDTH 0.03125f
#define ROUGH_ROW_FREQUENCY 4.7f
#define ROUGH_COLUMN_FREQUENCY 5.9f
#define ROUGH_AMPLITUDE 0.015f

#define DROP_SPACING 0.5f
#define DROP_HEIGHT 5.0f
#define BODY_RADIUS 0.05f
#define CAPSULE_HALF_LENGTH 0.2f

// The bottom of the chain loop. A body that crosses the wave passes out through the one sided bottom.
#define GROUND_DEPTH 1.0f

const char* g_waveDropShapeNames[e_waveDropShapeCount] = { "box", "capsule", "rounded", "circle" };

const float g_waveDropDepthLevels[WAVE_DROP_DEPTH_LEVEL_COUNT] = { 4.0f, 8.0f, 16.0f, 32.0f };

// Same XorShift32 and mapping as the samples, but with local state so worlds can run on many threads
static float RandomFloatRange( uint32_t* state, float lo, float hi )
{
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;

	int i = (int)( x % 32768 );
	float r = (float)( i & 32767 ) / 32767.0f;
	return ( hi - lo ) * r + lo;
}

static uint32_t HashSeed( uint32_t x )
{
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x != 0 ? x : 1;
}

typedef struct WaveGrid
{
	int count;
	float cellWidth;
	float rowFrequency;
	float columnFrequency;
} WaveGrid;

static WaveGrid GetWaveGrid( bool rough )
{
	_Static_assert( ROUGH_WAVE_COUNT <= WAVE_DROP_MAX_WAVE_COUNT && WAVE_COUNT <= WAVE_DROP_MAX_WAVE_COUNT,
					"wave count exceeds the depth tracker table" );

	if ( rough )
	{
		return (WaveGrid){ ROUGH_WAVE_COUNT, ROUGH_CELL_WIDTH, ROUGH_ROW_FREQUENCY, ROUGH_COLUMN_FREQUENCY };
	}

	return (WaveGrid){ WAVE_COUNT, WAVE_CELL_WIDTH, WAVE_ROW_FREQUENCY, WAVE_COLUMN_FREQUENCY };
}

// The Box3D mesh wave is a product of two sines across the grid. This is that surface along its diagonal.
static float GetWaveHeight( WaveGrid grid, float amplitude, int index )
{
	float omegaA = 2.0f * B2_PI * grid.rowFrequency * grid.cellWidth;
	float omegaB = 2.0f * B2_PI * grid.columnFrequency * grid.cellWidth;
	return amplitude * b2ComputeCosSin( omegaA * index ).sine * b2ComputeCosSin( omegaB * index ).sine;
}

static b2Polygon MakeDropBox( void )
{
	// The soft contact solver cannot always resolve a thinner box at 4 sub-steps, and its size gets
	// close to the time of impact tolerances
	return b2MakeBox( 0.02f, 0.2f );
}

static b2Polygon MakeDropRounded( void )
{
	return b2MakeRoundedBox( 0.03f, 0.17f, 0.02f );
}

WaveDropSettings GetDefaultWaveDropSettings( void )
{
	WaveDropSettings settings = {
		.shape = e_waveDropBox,
		.amplitude = WAVE_AMPLITUDE,
		.rough = false,
		.collide = false,
		.soloIndex = -1,
	};
	return settings;
}

float GetWaveDropDefaultAmplitude( bool rough )
{
	return rough ? ROUGH_AMPLITUDE : WAVE_AMPLITUDE;
}

uint32_t GetWaveDropSeed( uint32_t baseSeed, int generation )
{
	return HashSeed( HashSeed( baseSeed ) + (uint32_t)generation );
}

int GetWaveDropStepLimit( WaveDropShape shape )
{
	return shape == e_waveDropCapsule ? 3000 : 1000;
}

float GetWaveDropTunnelHeight( void )
{
	return -2.0f;
}

b2BodyId CreateWaveDropGround( b2WorldId worldId, WaveDropSettings settings )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );

	WaveGrid grid = GetWaveGrid( settings.rough );
	float extent = 0.5f * grid.count * grid.cellWidth;

	// Counter clockwise so the one sided segments face out. The wave runs right to left.
	int pointCount = grid.count + 3;
	b2Vec2* points = malloc( pointCount * sizeof( b2Vec2 ) );
	for ( int i = 0; i <= grid.count; ++i )
	{
		int index = grid.count - i;
		points[i] = (b2Vec2){ -extent + index * grid.cellWidth, GetWaveHeight( grid, settings.amplitude, index ) };
	}
	points[grid.count + 1] = (b2Vec2){ -extent, -GROUND_DEPTH };
	points[grid.count + 2] = (b2Vec2){ extent, -GROUND_DEPTH };

	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();

	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = points;
	chainDef.pointCount = pointCount;
	chainDef.materials = &material;
	chainDef.materialCount = 1;
	chainDef.filter.categoryBits = 1;
	chainDef.isLoop = true;
	b2CreateChain( groundId, &chainDef );

	free( points );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.filter.categoryBits = 1;

	float halfHeight = 1.0f;
	for ( int i = 0; i < 2; ++i )
	{
		b2Vec2 center = { i == 0 ? -extent : extent, halfHeight };
		b2Polygon wall = b2MakeOffsetBox( 0.1f, halfHeight, center, b2Rot_identity );
		b2CreatePolygonShape( groundId, &shapeDef, &wall );
	}

	return groundId;
}

void CreateWaveDropBodies( b2WorldId worldId, WaveDropSettings settings, uint32_t seed, b2BodyId* bodies )
{
	b2Polygon box = MakeDropBox();
	b2Polygon rounded = MakeDropRounded();
	b2Capsule capsule = { { 0.0f, -CAPSULE_HALF_LENGTH }, { 0.0f, CAPSULE_HALF_LENGTH }, BODY_RADIUS };
	b2Circle circle = { b2Vec2_zero, BODY_RADIUS };

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	bool round = settings.shape == e_waveDropCapsule || settings.shape == e_waveDropCircle;
	shapeDef.material.rollingResistance = round ? 0.4f : 0.1f;

	if ( settings.collide == false )
	{
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = 1;
	}

	uint32_t state = seed;
	int index = 0;
	for ( int i = 0; i < WAVE_DROP_GRID_COUNT; ++i )
	{
		for ( int j = 0; j < WAVE_DROP_GRID_COUNT; ++j )
		{
			// Draw for every body so a solo body gets the same velocity it has in the full drop
			b2Vec2 linearVelocity;
			linearVelocity.x = RandomFloatRange( &state, -1.0f, 1.0f );
			linearVelocity.y = RandomFloatRange( &state, -1.0f, 1.0f );
			float angularVelocity = RandomFloatRange( &state, -5.0f, 5.0f );

			if ( settings.soloIndex >= 0 && settings.soloIndex != index )
			{
				bodies[index] = b2_nullBodyId;
				index += 1;
				continue;
			}

			bodyDef.position = (b2Pos){ DROP_SPACING * ( i - 0.5f * WAVE_DROP_GRID_COUNT ), DROP_HEIGHT + DROP_SPACING * j };
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			bodyDef.userData = (void*)(intptr_t)( index + 1 );
			b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );

			switch ( settings.shape )
			{
				case e_waveDropBox:
					b2CreatePolygonShape( bodyId, &shapeDef, &box );
					break;
				case e_waveDropCapsule:
					b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
					break;
				case e_waveDropRounded:
					b2CreatePolygonShape( bodyId, &shapeDef, &rounded );
					break;
				default:
					b2CreateCircleShape( bodyId, &shapeDef, &circle );
					break;
			}

			bodies[index] = bodyId;
			index += 1;
		}
	}
}

int GetWaveDropBodyIndex( void* userData )
{
	intptr_t value = (intptr_t)userData;
	return 1 <= value && value <= WAVE_DROP_BODY_COUNT ? (int)value - 1 : -1;
}

void ResetWaveDropDepth( WaveDropDepth* depth, WaveDropSettings settings )
{
	WaveGrid grid = GetWaveGrid( settings.rough );
	for ( int i = 0; i <= grid.count; ++i )
	{
		depth->heights[i] = GetWaveHeight( grid, settings.amplitude, i );
	}

	depth->waveCount = grid.count;
	depth->cellWidth = grid.cellWidth;
	depth->amplitude = fabsf( settings.amplitude );
	depth->probeCount = 0;
	depth->radius = 0.0f;

	switch ( settings.shape )
	{
		case e_waveDropBox:
		case e_waveDropRounded:
		{
			b2Polygon polygon = settings.shape == e_waveDropBox ? MakeDropBox() : MakeDropRounded();
			for ( int i = 0; i < polygon.count; ++i )
			{
				depth->probes[i] = polygon.vertices[i];
			}
			depth->probeCount = polygon.count;
			depth->radius = polygon.radius;
		}
		break;

		case e_waveDropCapsule:
			depth->probes[0] = (b2Vec2){ 0.0f, -CAPSULE_HALF_LENGTH };
			depth->probes[1] = (b2Vec2){ 0.0f, CAPSULE_HALF_LENGTH };
			depth->probeCount = 2;
			depth->radius = BODY_RADIUS;
			break;

		default:
			depth->probes[0] = b2Vec2_zero;
			depth->probeCount = 1;
			depth->radius = BODY_RADIUS;
			break;
	}

	float maxLength = 0.0f;
	for ( int i = 0; i < depth->probeCount; ++i )
	{
		maxLength = fmaxf( maxLength, b2Length( depth->probes[i] ) );
	}
	depth->boundingRadius = maxLength + depth->radius;

	for ( int i = 0; i < WAVE_DROP_BODY_COUNT; ++i )
	{
		depth->bodyDepths[i] = -FLT_MAX;
	}

	depth->maxDepth = -FLT_MAX;
	depth->maxIndex = -1;
	depth->maxStep = 0;
	depth->stepCount = 0;
}

// Signed distance to the line of the wave segment under the point. Exact away from the joints, where it
// reads a little deep on a crest and shallow in a valley.
static float GetWaveDistance( const WaveDropDepth* depth, b2Vec2 p )
{
	float w = depth->cellWidth;
	float origin = -0.5f * depth->waveCount * w;
	int ix = b2ClampInt( (int)floorf( ( p.x - origin ) / w ), 0, depth->waveCount - 1 );

	b2Vec2 a = { origin + ix * w, depth->heights[ix] };
	b2Vec2 b = { origin + ( ix + 1 ) * w, depth->heights[ix + 1] };
	b2Vec2 normal = b2Normalize( b2LeftPerp( b2Sub( b, a ) ) );
	return b2Dot( normal, b2Sub( p, a ) );
}

void UpdateWaveDropDepth( b2WorldId worldId, WaveDropDepth* depth )
{
	depth->stepCount += 1;

	b2BodyEvents events = b2World_GetBodyEvents( worldId );
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b2BodyMoveEvent* event = events.moveEvents + i;
		int index = GetWaveDropBodyIndex( event->userData );
		if ( index < 0 )
		{
			continue;
		}

		b2WorldTransform transform = event->transform;
		if ( (float)transform.p.y - depth->boundingRadius > depth->amplitude )
		{
			continue;
		}

		float sink = -FLT_MAX;
		for ( int j = 0; j < depth->probeCount; ++j )
		{
			b2Pos point = b2TransformWorldPoint( transform, depth->probes[j] );
			b2Vec2 p = { (float)point.x, (float)point.y };
			sink = fmaxf( sink, depth->radius - GetWaveDistance( depth, p ) );
		}

		if ( sink > depth->bodyDepths[index] )
		{
			depth->bodyDepths[index] = sink;
		}

		if ( sink > depth->maxDepth )
		{
			depth->maxDepth = sink;
			depth->maxIndex = index;
			depth->maxStep = depth->stepCount;
		}
	}
}

void GetWaveDropDepthCounts( const WaveDropDepth* depth, int counts[WAVE_DROP_DEPTH_LEVEL_COUNT] )
{
	for ( int level = 0; level < WAVE_DROP_DEPTH_LEVEL_COUNT; ++level )
	{
		float threshold = g_waveDropDepthLevels[level] * B2_LINEAR_SLOP;
		int count = 0;
		for ( int i = 0; i < WAVE_DROP_BODY_COUNT; ++i )
		{
			count += depth->bodyDepths[i] > threshold ? 1 : 0;
		}
		counts[level] = count;
	}
}
