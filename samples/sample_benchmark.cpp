// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"
#include "draw.h"
#include "human.h"
#include "random.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <imgui.h>
#include <limits.h>
#include <set>
#include <stdint.h>
#include <vector>

#if defined( _MSC_VER )
#include <intrin.h>
#define GET_CYCLES __rdtsc()
#else
#define GET_CYCLES b2GetTicks()
#endif

inline bool operator<( b2BodyId a, b2BodyId b )
{
	uint64_t ua = b2StoreBodyId( a );
	uint64_t ub = b2StoreBodyId( b );
	return ua < ub;
}

// these are not accessible in some build types
// extern "C" int b2_toiCalls;
// extern "C" int b2_toiHitCount;

// Note: resetting the scene is non-deterministic because the world uses freelists
class BenchmarkBarrel : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_mixShape,
		e_compoundShape,
		e_humanShape,
	};

	enum
	{
		e_maxColumns = 26,
		e_maxRows = 150,
	};

	explicit BenchmarkBarrel( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 8.0f, 53.0f };
			m_context->camera.m_zoom = 25.0f * 2.35f;
		}

		m_context->drawJoints = false;

		{
			float gridSize = 1.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				x += gridSize;
			}

			y = gridSize;
			x = -40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				y += gridSize;
			}

			y = gridSize;
			x = 40.0f * gridSize;
			for ( int i = 0; i < 100; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				y += gridSize;
			}

			b2Segment segment = { { -800.0f, -80.0f }, { 800.0f, -80.f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		memset( m_humans, 0, sizeof( m_humans ) );

		m_shapeType = e_compoundShape;

		CreateScene();
	}

	void CreateScene()
	{
		g_randomSeed = 42;

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}

			if ( m_humans[i].isSpawned )
			{
				DestroyHuman( m_humans + i );
			}
		}

		m_columnCount = m_isDebug ? 10 : e_maxColumns;
		m_rowCount = m_isDebug ? 40 : e_maxRows;

		if ( m_shapeType == e_compoundShape )
		{
			if constexpr ( m_isDebug == false )
			{
				m_columnCount = 20;
			}
		}
		else if ( m_shapeType == e_humanShape )
		{
			if constexpr ( m_isDebug )
			{
				m_rowCount = 5;
				m_columnCount = 10;
			}
			else
			{
				m_rowCount = 30;
			}
		}

		float rad = 0.5f;

		float shift = 1.15f;
		float centerx = shift * m_columnCount / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		// todo eliminate this once rolling resistance is added
		if ( m_shapeType == e_mixShape )
		{
			bodyDef.angularDamping = 0.3f;
		}

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		b2Capsule capsule = { { 0.0f, -0.25f }, { 0.0f, 0.25f }, rad };
		b2Circle circle = { { 0.0f, 0.0f }, rad };

		b2Vec2 points[3] = { { -0.1f, -0.5f }, { 0.1f, -0.5f }, { 0.0f, 0.5f } };
		b2Hull wedgeHull = b2ComputeHull( points, 3 );
		b2Polygon wedge = b2MakePolygon( &wedgeHull, 0.0f );

		b2Vec2 vertices[3];
		vertices[0] = { -1.0f, 0.0f };
		vertices[1] = { 0.5f, 1.0f };
		vertices[2] = { 0.0f, 2.0f };
		b2Hull hull = b2ComputeHull( vertices, 3 );
		b2Polygon left = b2MakePolygon( &hull, 0.0f );

		vertices[0] = { 1.0f, 0.0f };
		vertices[1] = { -0.5f, 1.0f };
		vertices[2] = { 0.0f, 2.0f };
		hull = b2ComputeHull( vertices, 3 );
		b2Polygon right = b2MakePolygon( &hull, 0.0f );

		// b2Polygon top = b2MakeOffsetBox(0.8f, 0.2f, {0.0f, 0.8f}, 0.0f);
		// b2Polygon leftLeg = b2MakeOffsetBox(0.2f, 0.5f, {-0.6f, 0.5f}, 0.0f);
		// b2Polygon rightLeg = b2MakeOffsetBox(0.2f, 0.5f, {0.6f, 0.5f}, 0.0f);

		float side = -0.1f;
		float extray = 0.5f;

		if ( m_shapeType == e_compoundShape )
		{
			extray = 0.25f;
			side = 0.25f;
			shift = 2.0f;
			centerx = shift * m_columnCount / 2.0f - 1.0f;
		}
		else if ( m_shapeType == e_humanShape )
		{
			extray = 0.5f;
			side = 0.55f;
			shift = 2.5f;
			centerx = shift * m_columnCount / 2.0f;
		}

		int index = 0;
		float yStart = m_shapeType == e_humanShape ? 2.0f : 100.0f;

		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < m_rowCount; ++j )
			{
				float y = j * ( shift + extray ) + centery + yStart;

				bodyDef.position = { x + side, y };
				side = -side;

				if ( m_shapeType == e_circleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					circle.radius = RandomFloatRange( 0.25f, 0.75f );
					shapeDef.material.rollingResistance = 0.2f;
					b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
				}
				else if ( m_shapeType == e_capsuleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					capsule.radius = RandomFloatRange( 0.25f, 0.5f );
					float length = RandomFloatRange( 0.25f, 1.0f );
					capsule.center1 = { 0.0f, -0.5f * length };
					capsule.center2 = { 0.0f, 0.5f * length };
					shapeDef.material.rollingResistance = 0.2f;
					b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
				}
				else if ( m_shapeType == e_mixShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );

					int mod = index % 3;
					if ( mod == 0 )
					{
						circle.radius = RandomFloatRange( 0.25f, 0.75f );
						b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
					}
					else if ( mod == 1 )
					{
						capsule.radius = RandomFloatRange( 0.25f, 0.5f );
						float length = RandomFloatRange( 0.25f, 1.0f );
						capsule.center1 = { 0.0f, -0.5f * length };
						capsule.center2 = { 0.0f, 0.5f * length };
						b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
					}
					else if ( mod == 2 )
					{
						float width = RandomFloatRange( 0.1f, 0.5f );
						float height = RandomFloatRange( 0.5f, 0.75f );
						b2Polygon box = b2MakeBox( width, height );

						// Don't put a function call into a macro.
						float value = RandomFloatRange( -1.0f, 1.0f );
						box.radius = 0.25f * b2MaxFloat( 0.0f, value );
						b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );
					}
					else
					{
						wedge.radius = RandomFloatRange( 0.1f, 0.25f );
						b2CreatePolygonShape( m_bodies[index], &shapeDef, &wedge );
					}
				}
				else if ( m_shapeType == e_compoundShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );

					b2CreatePolygonShape( m_bodies[index], &shapeDef, &left );
					b2CreatePolygonShape( m_bodies[index], &shapeDef, &right );
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &top);
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &leftLeg);
					// b2CreatePolygonShape(m_bodies[index], &shapeDef, &rightLeg);
				}
				else if ( m_shapeType == e_humanShape )
				{
					float scale = 3.5f;
					float jointFriction = 0.05f;
					float jointHertz = 5.0f;
					float jointDamping = 0.5f;
					CreateHuman( m_humans + index, m_worldId, bodyDef.position, scale, jointFriction, jointHertz, jointDamping,
								 index + 1, nullptr, false );
				}

				index += 1;
			}
		}
	}

	void UpdateGui() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 220.0f, height ) );
		ImGui::Begin( "Benchmark: Barrel", nullptr, ImGuiWindowFlags_NoResize );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Capsule", "Mix", "Compound", "Human" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkBarrel( context );
	}

	b2BodyId m_bodies[e_maxRows * e_maxColumns];
	Human m_humans[e_maxRows * e_maxColumns];
	int m_columnCount;
	int m_rowCount;

	ShapeType m_shapeType;
};

static int benchmarkBarrel = RegisterSample( "Benchmark", "Barrel", BenchmarkBarrel::Create );

class BenchmarkTumbler : public Sample
{
public:
	explicit BenchmarkTumbler( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 1.5f, 10.0f };
			m_context->camera.m_zoom = 25.0f * 0.6f;
		}

		CreateTumbler( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkTumbler( context );
	}
};

static int benchmarkTumbler = RegisterSample( "Benchmark", "Tumbler", BenchmarkTumbler::Create );

// todo try removing kinematics from graph coloring
class BenchmarkManyTumblers : public Sample
{
public:
	explicit BenchmarkManyTumblers( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 1.0f, -5.5 };
			m_context->camera.m_zoom = 25.0f * 3.4f;
			m_context->drawJoints = false;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		m_rowCount = m_isDebug ? 2 : 19;
		m_columnCount = m_isDebug ? 2 : 19;

		m_tumblerIds = nullptr;
		m_positions = nullptr;
		m_tumblerCount = 0;

		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		m_angularSpeed = 25.0f;

		CreateScene();
	}

	~BenchmarkManyTumblers() override
	{
		free( m_tumblerIds );
		free( m_positions );
		free( m_bodyIds );
	}

	void CreateTumbler( b2Vec2 position, int index )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.position = { position.x, position.y };
		bodyDef.angularVelocity = ( B2_PI / 180.0f ) * m_angularSpeed;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
		m_tumblerIds[index] = bodyId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { 2.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { -2.0f, 0.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, 2.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, -2.0f }, b2Rot_identity );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
			}
		}

		for ( int i = 0; i < m_tumblerCount; ++i )
		{
			b2DestroyBody( m_tumblerIds[i] );
		}

		free( m_tumblerIds );
		free( m_positions );

		m_tumblerCount = m_rowCount * m_columnCount;
		m_tumblerIds = static_cast<b2BodyId*>( malloc( m_tumblerCount * sizeof( b2BodyId ) ) );
		m_positions = static_cast<b2Vec2*>( malloc( m_tumblerCount * sizeof( b2Vec2 ) ) );

		int index = 0;
		float x = -4.0f * m_rowCount;
		for ( int i = 0; i < m_rowCount; ++i )
		{
			float y = -4.0f * m_columnCount;
			for ( int j = 0; j < m_columnCount; ++j )
			{
				m_positions[index] = { x, y };
				CreateTumbler( m_positions[index], index );
				++index;
				y += 8.0f;
			}

			x += 8.0f;
		}

		free( m_bodyIds );

		int bodiesPerTumbler = m_isDebug ? 8 : 50;
		m_bodyCount = bodiesPerTumbler * m_tumblerCount;

		m_bodyIds = static_cast<b2BodyId*>( malloc( m_bodyCount * sizeof( b2BodyId ) ) );

		memset( m_bodyIds, 0, m_bodyCount * sizeof( b2BodyId ) );
		m_bodyIndex = 0;
	}

	void UpdateGui() override
	{
		float height = 110.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );
		ImGui::Begin( "Benchmark: Many Tumblers", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		bool changed = false;
		changed = changed || ImGui::SliderInt( "Row Count", &m_rowCount, 1, 32 );
		changed = changed || ImGui::SliderInt( "Column Count", &m_columnCount, 1, 32 );

		if ( changed )
		{
			CreateScene();
		}

		if ( ImGui::SliderFloat( "Speed", &m_angularSpeed, 0.0f, 100.0f, "%.f" ) )
		{
			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				b2Body_SetAngularVelocity( m_tumblerIds[i], ( B2_PI / 180.0f ) * m_angularSpeed );
				b2Body_SetAwake( m_tumblerIds[i], true );
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		if ( m_bodyIndex < m_bodyCount && ( m_stepCount & 0x7 ) == 0 )
		{
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Capsule capsule = { { -0.1f, 0.0f }, { 0.1f, 0.0f }, 0.075f };

			for ( int i = 0; i < m_tumblerCount; ++i )
			{
				assert( m_bodyIndex < m_bodyCount );

				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = m_positions[i];
				m_bodyIds[m_bodyIndex] = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( m_bodyIds[m_bodyIndex], &shapeDef, &capsule );

				m_bodyIndex += 1;
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkManyTumblers( context );
	}

	b2BodyId m_groundId;

	int m_rowCount;
	int m_columnCount;

	b2BodyId* m_tumblerIds;
	b2Vec2* m_positions;
	int m_tumblerCount;

	b2BodyId* m_bodyIds;
	int m_bodyCount;
	int m_bodyIndex;

	float m_angularSpeed;
};

static int benchmarkManyTumblers = RegisterSample( "Benchmark", "Many Tumblers", BenchmarkManyTumblers::Create );

class BenchmarkLargePyramid : public Sample
{
public:
	explicit BenchmarkLargePyramid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 50.0f };
			m_context->camera.m_zoom = 25.0f * 2.2f;
			m_context->enableSleep = false;
		}

		CreateLargePyramid( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkLargePyramid( context );
	}
};

static int benchmarkLargePyramid = RegisterSample( "Benchmark", "Large Pyramid", BenchmarkLargePyramid::Create );

class BenchmarkManyPyramids : public Sample
{
public:
	explicit BenchmarkManyPyramids( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 16.0f, 110.0f };
			m_context->camera.m_zoom = 25.0f * 5.0f;
			m_context->enableSleep = false;
		}

		CreateManyPyramids( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkManyPyramids( context );
	}
};

static int benchmarkManyPyramids = RegisterSample( "Benchmark", "Many Pyramids", BenchmarkManyPyramids::Create );

class BenchmarkCreateDestroy : public Sample
{
public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * ( e_maxBaseCount + 1 ) / 2
	};

	explicit BenchmarkCreateDestroy( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 50.0f };
			m_context->camera.m_zoom = 25.0f * 2.2f;
		}

		float groundSize = 100.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( groundSize, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		m_baseCount = m_isDebug ? 40 : 100;
		m_iterations = m_isDebug ? 1 : 10;
		m_bodyCount = 0;
	}

	void CreateScene()
	{
		uint64_t ticks = b2GetTicks();

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}
		}

		m_destroyTime += b2GetMillisecondsAndReset( &ticks );

		int count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		float h = 0.5f;
		b2Polygon box = b2MakeRoundedBox( h, h, 0.0f );

		int index = 0;

		for ( int i = 0; i < count; ++i )
		{
			float y = i * shift + centery;

			for ( int j = i; j < count; ++j )
			{
				float x = 0.5f * i * shift + ( j - i ) * shift - centerx;
				bodyDef.position = { x, y };

				assert( index < e_maxBodyCount );
				m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );

				index += 1;
			}
		}

		m_createTime += b2GetMilliseconds( ticks );

		m_bodyCount = index;

		b2World_Step( m_worldId, 1.0f / 60.0f, 4 );
	}

	void Step() override
	{
		m_createTime = 0.0f;
		m_destroyTime = 0.0f;

		for ( int i = 0; i < m_iterations; ++i )
		{
			CreateScene();
		}

		DrawTextLine( "total: create = %g ms, destroy = %g ms", m_createTime, m_destroyTime );

		float createPerBody = 1000.0f * m_createTime / m_iterations / m_bodyCount;
		float destroyPerBody = 1000.0f * m_destroyTime / m_iterations / m_bodyCount;
		DrawTextLine( "body: create = %g us, destroy = %g us", createPerBody, destroyPerBody );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCreateDestroy( context );
	}

	float m_createTime;
	float m_destroyTime;
	b2BodyId m_bodies[e_maxBodyCount];
	int m_bodyCount;
	int m_baseCount;
	int m_iterations;
};

static int benchmarkCreateDestroy = RegisterSample( "Benchmark", "CreateDestroy", BenchmarkCreateDestroy::Create );

class BenchmarkSleep : public Sample
{
public:
	enum
	{
		e_maxBaseCount = 100,
		e_maxBodyCount = e_maxBaseCount * ( e_maxBaseCount + 1 ) / 2
	};

	explicit BenchmarkSleep( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 50.0f };
			m_context->camera.m_zoom = 25.0f * 2.2f;
		}

		float groundSize = 100.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( groundSize, 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_baseCount = m_isDebug ? 40 : 100;
		m_iterations = m_isDebug ? 1 : 41;
		m_bodyCount = 0;
		m_awake = false;

		m_wakeTotal = 0.0f;
		m_wakeCount = 0;

		m_sleepTotal = 0.0f;
		m_sleepCount = 0;

		CreateScene();
	}

	void CreateScene()
	{
		for ( int i = 0; i < e_maxBodyCount; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}
		}

		int count = m_baseCount;
		float rad = 0.5f;
		float shift = rad * 2.0f;
		float centerx = shift * count / 2.0f;
		float centery = shift / 2.0f + 1.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.5f;

		float h = 0.5f;
		b2Polygon box = b2MakeRoundedBox( h, h, 0.0f );

		int index = 0;

		for ( int i = 0; i < count; ++i )
		{
			float y = i * shift + centery;

			for ( int j = i; j < count; ++j )
			{
				float x = 0.5f * i * shift + ( j - i ) * shift - centerx;
				bodyDef.position = { x, y };

				assert( index < e_maxBodyCount );
				m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );

				index += 1;
			}
		}

		m_bodyCount = index;
	}

	void Step() override
	{
		uint64_t ticks = b2GetTicks();

		for ( int i = 0; i < m_iterations; ++i )
		{
			b2Body_SetAwake( m_bodies[0], m_awake );
			if ( m_awake )
			{
				m_wakeTotal += b2GetMillisecondsAndReset( &ticks );
				m_wakeCount += 1;
			}
			else
			{
				m_sleepTotal += b2GetMillisecondsAndReset( &ticks );
				m_sleepCount += 1;
			}
			m_awake = !m_awake;
		}

		if ( m_wakeCount > 0 )
		{
			DrawTextLine( "wake ave = %g ms", m_wakeTotal / m_wakeCount );
		}

		if ( m_sleepCount > 0 )
		{
			DrawTextLine( "sleep ave = %g ms", m_sleepTotal / m_sleepCount );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSleep( context );
	}

	b2BodyId m_bodies[e_maxBodyCount];
	int m_bodyCount;
	int m_baseCount;
	int m_iterations;
	float m_wakeTotal;
	float m_sleepTotal;
	int m_wakeCount;
	int m_sleepCount;
	bool m_awake;
};

static int benchmarkSleep = RegisterSample( "Benchmark", "Sleep", BenchmarkSleep::Create );

class BenchmarkJointGrid : public Sample
{
public:
	explicit BenchmarkJointGrid( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 60.0f, -57.0f };
			m_context->camera.m_zoom = 25.0f * 2.5f;
			m_context->enableSleep = false;
		}

		CreateJointGrid( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkJointGrid( context );
	}
};

static int benchmarkJointGridIndex = RegisterSample( "Benchmark", "Joint Grid", BenchmarkJointGrid::Create );

class BenchmarkSmash : public Sample
{
public:
	explicit BenchmarkSmash( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 60.0f, 6.0f };
			m_context->camera.m_zoom = 25.0f * 1.6f;
		}

		CreateSmash( m_worldId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSmash( context );
	}
};

static int sampleSmash = RegisterSample( "Benchmark", "Smash", BenchmarkSmash::Create );

class BenchmarkCompound : public Sample
{
public:
	explicit BenchmarkCompound( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 18.0f, 115.0f };
			m_context->camera.m_zoom = 25.0f * 5.5f;
		}

		float grid = 1.0f;
#ifdef NDEBUG
		int height = 200;
		int width = 200;
#else
		int height = 100;
		int width = 100;
#endif
		{

			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = grid * j;
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
					b2CreatePolygonShape( groundId, &shapeDef, &square );
				}
			}

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = -grid * j;
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
					b2CreatePolygonShape( groundId, &shapeDef, &square );
				}
			}
		}

		{
#ifdef NDEBUG
			int span = 20;
			int count = 5;
#else
			int span = 5;
			int count = 5;
#endif

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			// defer mass properties to avoid n-squared mass computations
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.updateBodyMass = false;

			for ( int m = 0; m < count; ++m )
			{
				float ybody = ( 100.0f + m * span ) * grid;

				for ( int n = 0; n < count; ++n )
				{
					float xbody = -0.5f * grid * count * span + n * span * grid;
					bodyDef.position = { xbody, ybody };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

					for ( int i = 0; i < span; ++i )
					{
						float y = i * grid;
						for ( int j = 0; j < span; ++j )
						{
							float x = j * grid;
							b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
							b2CreatePolygonShape( bodyId, &shapeDef, &square );
						}
					}

					// All shapes have been added so I can efficiently compute the mass properties.
					b2Body_ApplyMassFromShapes( bodyId );
				}
			}
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCompound( context );
	}
};

static int sampleCompound = RegisterSample( "Benchmark", "Compound", BenchmarkCompound::Create );

class BenchmarkKinematic : public Sample
{
public:
	explicit BenchmarkKinematic( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 150.0f;
		}

		float grid = 1.0f;

#ifdef NDEBUG
		int span = 100;
#else
		int span = 20;
#endif

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_kinematicBody;
		bodyDef.angularVelocity = 1.0f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		shapeDef.filter.maskBits = 2;

		// defer mass properties to avoid n-squared mass computations
		shapeDef.updateBodyMass = false;

		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		for ( int i = -span; i < span; ++i )
		{
			float y = i * grid;
			for ( int j = -span; j < span; ++j )
			{
				float x = j * grid;
				b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );
			}
		}

		// All shapes have been added so I can efficiently compute the mass properties.
		b2Body_ApplyMassFromShapes( bodyId );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkKinematic( context );
	}
};

static int sampleKinematic = RegisterSample( "Benchmark", "Kinematic", BenchmarkKinematic::Create );

enum QueryType
{
	e_rayCast,
	e_circleCast,
	e_overlap,
};

class BenchmarkCast : public Sample
{
public:
	explicit BenchmarkCast( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 500.0f, 500.0f };
			m_context->camera.m_zoom = 25.0f * 21.0f;
			// settings.drawShapes = m_isDebug;
		}

		m_queryType = e_circleCast;
		m_ratio = 5.0f;
		m_grid = 1.0f;
		m_fill = 0.1f;
		m_rowCount = m_isDebug ? 100 : 1000;
		m_columnCount = m_isDebug ? 100 : 1000;
		m_minTime = 1e6f;
		m_drawIndex = 0;
		m_topDown = false;
		m_buildTime = 0.0f;
		m_radius = 0.1f;

		g_randomSeed = 1234;
		int sampleCount = m_isDebug ? 100 : 10000;
		m_origins.resize( sampleCount );
		m_translations.resize( sampleCount );
		float extent = m_rowCount * m_grid;

		// Pre-compute rays to avoid randomizer overhead
		for ( int i = 0; i < sampleCount; ++i )
		{
			b2Vec2 rayStart = RandomVec2( 0.0f, extent );
			b2Vec2 rayEnd = RandomVec2( 0.0f, extent );

			m_origins[i] = rayStart;
			m_translations[i] = rayEnd - rayStart;
		}

		BuildScene();
	}

	void BuildScene()
	{
		g_randomSeed = 1234;
		b2DestroyWorld( m_worldId );
		b2WorldDef worldDef = b2DefaultWorldDef();
		m_worldId = b2CreateWorld( &worldDef );

		uint64_t ticks = b2GetTicks();

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = 0.0f;

		for ( int i = 0; i < m_rowCount; ++i )
		{
			float x = 0.0f;

			for ( int j = 0; j < m_columnCount; ++j )
			{
				float fillTest = RandomFloatRange( 0.0f, 1.0f );
				if ( fillTest <= m_fill )
				{
					bodyDef.position = { x, y };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

					float ratio = RandomFloatRange( 1.0f, m_ratio );
					float halfWidth = RandomFloatRange( 0.05f, 0.25f );

					b2Polygon box;
					if ( RandomFloat() > 0.0f )
					{
						box = b2MakeBox( ratio * halfWidth, halfWidth );
					}
					else
					{
						box = b2MakeBox( halfWidth, ratio * halfWidth );
					}

					int category = RandomIntRange( 0, 2 );
					shapeDef.filter.categoryBits = 1 << category;
					if ( category == 0 )
					{
						shapeDef.material.customColor = b2_colorBox2DBlue;
					}
					else if ( category == 1 )
					{
						shapeDef.material.customColor = b2_colorBox2DYellow;
					}
					else
					{
						shapeDef.material.customColor = b2_colorBox2DGreen;
					}

					b2CreatePolygonShape( bodyId, &shapeDef, &box );
				}

				x += m_grid;
			}

			y += m_grid;
		}

		if ( m_topDown )
		{
			b2World_RebuildStaticTree( m_worldId );
		}

		m_buildTime = b2GetMilliseconds( ticks );
		m_minTime = 1e6f;
	}

	void UpdateGui() override
	{
		float height = 240.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Cast", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 100.0f );

		bool changed = false;

		const char* queryTypes[] = { "Ray", "Circle", "Overlap" };
		int queryType = int( m_queryType );
		if ( ImGui::Combo( "Query", &queryType, queryTypes, IM_ARRAYSIZE( queryTypes ) ) )
		{
			m_queryType = QueryType( queryType );
			if ( m_queryType == e_overlap )
			{
				m_radius = 5.0f;
			}
			else
			{
				m_radius = 0.1f;
			}

			changed = true;
		}

		if ( ImGui::SliderInt( "rows", &m_rowCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderInt( "columns", &m_columnCount, 0, 1000, "%d" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "fill", &m_fill, 0.0f, 1.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "grid", &m_grid, 0.5f, 2.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::SliderFloat( "ratio", &m_ratio, 1.0f, 10.0f, "%.2f" ) )
		{
			changed = true;
		}

		if ( ImGui::Checkbox( "top down", &m_topDown ) )
		{
			changed = true;
		}

		if ( ImGui::Button( "Draw Next" ) )
		{
			m_drawIndex = ( m_drawIndex + 1 ) % m_origins.size();
		}

		ImGui::PopItemWidth();
		ImGui::End();

		if ( changed )
		{
			BuildScene();
		}
	}

	struct CastResult
	{
		b2Vec2 point;
		float fraction;
		bool hit;
	};

	static float CastCallback( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context )
	{
		CastResult* result = (CastResult*)context;
		result->point = point;
		result->fraction = fraction;
		result->hit = true;
		return fraction;
	}

	struct OverlapResult
	{
		b2Vec2 points[32];
		int count;
	};

	static bool OverlapCallback( b2ShapeId shapeId, void* context )
	{
		OverlapResult* result = (OverlapResult*)context;
		if ( result->count < 32 )
		{
			b2AABB aabb = b2Shape_GetAABB( shapeId );
			result->points[result->count] = b2AABB_Center( aabb );
			result->count += 1;
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		b2QueryFilter filter = b2DefaultQueryFilter();
		filter.maskBits = 1;
		int hitCount = 0;
		int nodeVisits = 0;
		int leafVisits = 0;
		float ms = 0.0f;
		int sampleCount = (int)m_origins.size();

		if ( m_queryType == e_rayCast )
		{
			uint64_t ticks = b2GetTicks();

			b2RayResult drawResult = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2Vec2 origin = m_origins[i];
				b2Vec2 translation = m_translations[i];

				b2RayResult result = b2World_CastRayClosest( m_worldId, origin, translation, filter );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += result.nodeVisits;
				leafVisits += result.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 p1 = m_origins[m_drawIndex];
			b2Vec2 p2 = p1 + m_translations[m_drawIndex];
			m_context->draw.DrawSegment( p1, p2, b2_colorWhite );
			m_context->draw.DrawPoint( p1, 5.0f, b2_colorGreen );
			m_context->draw.DrawPoint( p2, 5.0f, b2_colorRed );
			if ( drawResult.hit )
			{
				m_context->draw.DrawPoint( drawResult.point, 5.0f, b2_colorWhite );
			}
		}
		else if ( m_queryType == e_circleCast )
		{
			uint64_t ticks = b2GetTicks();

			CastResult drawResult = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2ShapeProxy proxy = b2MakeProxy( &m_origins[i], 1, m_radius );
				b2Vec2 translation = m_translations[i];

				CastResult result;
				b2TreeStats traversalResult = b2World_CastShape( m_worldId, &proxy, translation, filter, CastCallback, &result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.hit ? 1 : 0;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 p1 = m_origins[m_drawIndex];
			b2Vec2 p2 = p1 + m_translations[m_drawIndex];
			m_context->draw.DrawSegment( p1, p2, b2_colorWhite );
			m_context->draw.DrawPoint( p1, 5.0f, b2_colorGreen );
			m_context->draw.DrawPoint( p2, 5.0f, b2_colorRed );
			if ( drawResult.hit )
			{
				b2Vec2 t = b2Lerp( p1, p2, drawResult.fraction );
				m_context->draw.DrawCircle( t, m_radius, b2_colorWhite );
				m_context->draw.DrawPoint( drawResult.point, 5.0f, b2_colorWhite );
			}
		}
		else if ( m_queryType == e_overlap )
		{
			uint64_t ticks = b2GetTicks();

			OverlapResult drawResult = {};
			b2Vec2 extent = { m_radius, m_radius };
			OverlapResult result = {};

			for ( int i = 0; i < sampleCount; ++i )
			{
				b2Vec2 origin = m_origins[i];
				b2AABB aabb = { origin - extent, origin + extent };

				result.count = 0;
				b2TreeStats traversalResult = b2World_OverlapAABB( m_worldId, aabb, filter, OverlapCallback, &result );

				if ( i == m_drawIndex )
				{
					drawResult = result;
				}

				nodeVisits += traversalResult.nodeVisits;
				leafVisits += traversalResult.leafVisits;
				hitCount += result.count;
			}

			ms = b2GetMilliseconds( ticks );

			m_minTime = b2MinFloat( m_minTime, ms );

			b2Vec2 origin = m_origins[m_drawIndex];
			b2AABB aabb = { origin - extent, origin + extent };

			m_context->draw.DrawAABB( aabb, b2_colorWhite );

			for ( int i = 0; i < drawResult.count; ++i )
			{
				m_context->draw.DrawPoint( drawResult.points[i], 5.0f, b2_colorHotPink );
			}
		}

		DrawTextLine( "build time ms = %g", m_buildTime );
		DrawTextLine( "hit count = %d, node visits = %d, leaf visits = %d", hitCount, nodeVisits, leafVisits );
		DrawTextLine( "total ms = %.3f", ms );
		DrawTextLine( "min total ms = %.3f", m_minTime );

		float aveRayCost = 1000.0f * m_minTime / float( sampleCount );
		DrawTextLine( "average us = %.2f", aveRayCost );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkCast( context );
	}

	QueryType m_queryType;

	std::vector<b2Vec2> m_origins;
	std::vector<b2Vec2> m_translations;
	float m_minTime;
	float m_buildTime;

	int m_rowCount, m_columnCount;
	int m_updateType;
	int m_drawIndex;
	float m_radius;
	float m_fill;
	float m_ratio;
	float m_grid;
	bool m_topDown;
};

static int sampleCast = RegisterSample( "Benchmark", "Cast", BenchmarkCast::Create );

class BenchmarkSpinner : public Sample
{
public:
	explicit BenchmarkSpinner( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 32.0f };
			m_context->camera.m_zoom = 42.0f;
		}

		// b2_toiCalls = 0;
		// b2_toiHitCount = 0;

		CreateSpinner( m_worldId );
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == 1000 && false )
		{
			// 0.1 : 46544, 25752
			// 0.25 : 5745, 1947
			// 0.5 : 2197, 660
			m_context->pause = true;
		}

		// DrawTextLine( "toi calls, hits = %d, %d", b2_toiCalls, b2_toiHitCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSpinner( context );
	}
};

static int sampleSpinner = RegisterSample( "Benchmark", "Spinner", BenchmarkSpinner::Create );

class BenchmarkRain : public Sample
{
public:
	explicit BenchmarkRain( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 110.0f };
			m_context->camera.m_zoom = 125.0f;
			m_context->enableSleep = true;
		}

		m_context->drawJoints = false;

		CreateRain( m_worldId );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			StepRain( m_worldId, m_stepCount );
		}

		Sample::Step();

		if ( m_stepCount % 1000 == 0 )
		{
			m_stepCount += 0;
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkRain( context );
	}
};

static int benchmarkRain = RegisterSample( "Benchmark", "Rain", BenchmarkRain::Create );

class BenchmarkShapeDistance : public Sample
{
public:
	explicit BenchmarkShapeDistance( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 3.0f;
		}

		{
			b2Vec2 points[8] = {};
			b2Rot q = b2MakeRot( 2.0f * B2_PI / 8.0f );
			b2Vec2 p = { 0.5f, 0.0f };
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = b2RotateVector( q, points[i - 1] );
			}

			b2Hull hull = b2ComputeHull( points, 8 );
			m_polygonA = b2MakePolygon( &hull, 0.0f );
		}

		{
			b2Vec2 points[8] = {};
			b2Rot q = b2MakeRot( 2.0f * B2_PI / 8.0f );
			b2Vec2 p = { 0.5f, 0.0f };
			points[0] = p;
			for ( int i = 1; i < 8; ++i )
			{
				points[i] = b2RotateVector( q, points[i - 1] );
			}

			b2Hull hull = b2ComputeHull( points, 8 );
			m_polygonB = b2MakePolygon( &hull, 0.1f );
		}

		// todo arena
		m_transformAs = (b2Transform*)malloc( m_count * sizeof( b2Transform ) );
		m_transformBs = (b2Transform*)malloc( m_count * sizeof( b2Transform ) );
		m_outputs = (b2DistanceOutput*)calloc( m_count, sizeof( b2DistanceOutput ) );

		g_randomSeed = 42;
		for ( int i = 0; i < m_count; ++i )
		{
			m_transformAs[i] = { RandomVec2( -0.1f, 0.1f ), RandomRot() };
			m_transformBs[i] = { RandomVec2( 0.25f, 2.0f ), RandomRot() };
		}

		m_drawIndex = 0;
		m_minCycles = INT_MAX;
		m_minMilliseconds = FLT_MAX;
	}

	~BenchmarkShapeDistance() override
	{
		free( m_transformAs );
		free( m_transformBs );
		free( m_outputs );
	}

	void UpdateGui() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 220.0f, height ) );
		ImGui::Begin( "Benchmark: Shape Distance", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderInt( "draw index", &m_drawIndex, 0, m_count - 1 );

		ImGui::End();
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep == true )
		{
			b2DistanceInput input = {};
			input.proxyA = b2MakeProxy( m_polygonA.vertices, m_polygonA.count, m_polygonA.radius );
			input.proxyB = b2MakeProxy( m_polygonB.vertices, m_polygonB.count, m_polygonB.radius );
			input.useRadii = true;
			int totalIterations = 0;

			uint64_t start = b2GetTicks();
			uint64_t startCycles = GET_CYCLES;
			for ( int i = 0; i < m_count; ++i )
			{
				b2SimplexCache cache = {};
				input.transformA = m_transformAs[i];
				input.transformB = m_transformBs[i];
				m_outputs[i] = b2ShapeDistance( &input, &cache, nullptr, 0 );
				totalIterations += m_outputs[i].iterations;
			}
			uint64_t endCycles = GET_CYCLES;

			float ms = b2GetMilliseconds( start );
			m_minCycles = b2MinInt( m_minCycles, int( endCycles - startCycles ) );
			m_minMilliseconds = b2MinFloat( m_minMilliseconds, ms );

			DrawTextLine( "count = %d", m_count );
			DrawTextLine( "min cycles = %d", m_minCycles );
			DrawTextLine( "ave cycles = %g", float( m_minCycles ) / float( m_count ) );
			DrawTextLine( "min ms = %g, ave us = %g", m_minMilliseconds, 1000.0f * m_minMilliseconds / float( m_count ) );
			DrawTextLine( "average iterations = %g", totalIterations / float( m_count ) );
		}

		b2Transform xfA = m_transformAs[m_drawIndex];
		b2Transform xfB = m_transformBs[m_drawIndex];
		b2DistanceOutput output = m_outputs[m_drawIndex];
		m_context->draw.DrawSolidPolygon( xfA, m_polygonA.vertices, m_polygonA.count, m_polygonA.radius, b2_colorBox2DGreen );
		m_context->draw.DrawSolidPolygon( xfB, m_polygonB.vertices, m_polygonB.count, m_polygonB.radius, b2_colorBox2DBlue );
		m_context->draw.DrawSegment( output.pointA, output.pointB, b2_colorDimGray );
		m_context->draw.DrawPoint( output.pointA, 10.0f, b2_colorWhite );
		m_context->draw.DrawPoint( output.pointB, 10.0f, b2_colorWhite );
		m_context->draw.DrawSegment( output.pointA, output.pointA + 0.5f * output.normal, b2_colorYellow );
		DrawTextLine( "distance = %g", output.distance );

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkShapeDistance( context );
	}

	static constexpr int m_count = m_isDebug ? 100 : 10000;
	b2Transform* m_transformAs;
	b2Transform* m_transformBs;
	b2DistanceOutput* m_outputs;
	b2Polygon m_polygonA;
	b2Polygon m_polygonB;
	float m_minMilliseconds;
	int m_drawIndex;
	int m_minCycles;
};

static int benchmarkShapeDistance = RegisterSample( "Benchmark", "Shape Distance", BenchmarkShapeDistance::Create );

struct ShapeUserData
{
	bool shouldDestroyVisitors;
};

class BenchmarkSensor : public Sample
{
public:
	explicit BenchmarkSensor( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 105.0f };
			m_context->camera.m_zoom = 125.0f;
		}

		m_passiveSensor.shouldDestroyVisitors = false;
		m_activeSensor.shouldDestroyVisitors = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		{
			float gridSize = 3.0f;

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.isSensor = true;
			shapeDef.enableSensorEvents = true;
			shapeDef.userData = &m_activeSensor;

			float y = 0.0f;
			float x = -40.0f * gridSize;
			for ( int i = 0; i < 81; ++i )
			{
				b2Polygon box = b2MakeOffsetBox( 0.5f * gridSize, 0.5f * gridSize, { x, y }, b2Rot_identity );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
				x += gridSize;
			}
		}

		g_randomSeed = 42;

		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.isSensor = true;
		shapeDef.enableSensorEvents = true;
		shapeDef.userData = &m_passiveSensor;

		float yStart = 10.0f;

		for ( int j = 0; j < m_rowCount; ++j )
		{
			float y = j * shift + yStart;
			for ( int i = 0; i < m_columnCount; ++i )
			{
				float x = i * shift - xCenter;
				float yOffset = RandomFloatRange( -1.0f, 1.0f );
				b2Polygon box = b2MakeOffsetRoundedBox( 0.5f, 0.5f, { x, y + yOffset }, RandomRot(), 0.1f );
				b2CreatePolygonShape( groundId, &shapeDef, &box );
			}
		}

		m_maxBeginCount = 0;
		m_maxEndCount = 0;
		m_lastStepCount = 0;
	}

	void CreateRow( float y )
	{
		float shift = 5.0f;
		float xCenter = 0.5f * shift * m_columnCount;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.linearVelocity = { 0.0f, -5.0f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.enableSensorEvents = true;

		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		for ( int i = 0; i < m_columnCount; ++i )
		{
			bodyDef.position = { shift * i - xCenter, y };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void Step() override
	{
		Sample::Step();

		if ( m_stepCount == m_lastStepCount )
		{
			return;
		}

		std::set<b2BodyId> zombies;

		b2SensorEvents events = b2World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < events.beginCount; ++i )
		{
			b2SensorBeginTouchEvent* event = events.beginEvents + i;

			// shapes on begin touch are always valid

			ShapeUserData* userData = static_cast<ShapeUserData*>( b2Shape_GetUserData( event->sensorShapeId ) );
			if ( userData->shouldDestroyVisitors )
			{
				zombies.emplace( b2Shape_GetBody( event->visitorShapeId ) );
			}
			else
			{
				// Modify color while overlapped with a sensor
				b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
				surfaceMaterial.customColor = b2_colorLime;
				b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
			}
		}

		for ( int i = 0; i < events.endCount; ++i )
		{
			b2SensorEndTouchEvent* event = events.endEvents + i;

			if ( b2Shape_IsValid( event->visitorShapeId ) == false )
			{
				continue;
			}

			// Restore color to default
			b2SurfaceMaterial surfaceMaterial = b2Shape_GetSurfaceMaterial( event->visitorShapeId );
			surfaceMaterial.customColor = 0;
			b2Shape_SetSurfaceMaterial( event->visitorShapeId, surfaceMaterial );
		}

		for ( b2BodyId bodyId : zombies )
		{
			b2DestroyBody( bodyId );
		}

		int delay = 0x1F;

		if ( ( m_stepCount & delay ) == 0 )
		{
			CreateRow( 10.0f + m_rowCount * 5.0f );
		}

		m_lastStepCount = m_stepCount;

		m_maxBeginCount = b2MaxInt( events.beginCount, m_maxBeginCount );
		m_maxEndCount = b2MaxInt( events.endCount, m_maxEndCount );
		DrawTextLine( "max begin touch events = %d", m_maxBeginCount );
		DrawTextLine( "max end touch events = %d", m_maxEndCount );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BenchmarkSensor( context );
	}

	static constexpr int m_columnCount = 40;
	static constexpr int m_rowCount = 40;
	int m_maxBeginCount;
	int m_maxEndCount;
	ShapeUserData m_passiveSensor;
	ShapeUserData m_activeSensor;
	int m_lastStepCount;
};

static int benchmarkSensor = RegisterSample( "Benchmark", "Sensor", BenchmarkSensor::Create );
