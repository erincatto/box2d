// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Note: resetting the scene is non-deterministic because the world uses freelists
class BenchmarkBarrel : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_caspuleShape,
		e_mixShape,
		e_compoundShape,
		e_humanShape,
	};

	enum
	{
		e_maxColumns = 26,
		e_maxRows = 130,
	};

	explicit BenchmarkBarrel( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 8.0f, 53.0f };
			g_camera.m_zoom = 25.0f * 2.35f;
		}

		settings.drawJoints = false;

		float groundSize = 25.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( groundSize, 1.2f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.2f, 2.0f * groundSize, { -groundSize, 2.0f * groundSize }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.2f, 2.0f * groundSize, { groundSize, 2.0f * groundSize }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 800.0f, 10.0f, { 0.0f, -80.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		m_shapeType = e_circleShape;

		CreateScene();
	}

	void CreateScene()
	{
		srand( 42 );

		for ( int i = 0; i < e_maxRows * e_maxColumns; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodies[i] ) )
			{
				b2DestroyBody( m_bodies[i] );
				m_bodies[i] = b2_nullBodyId;
			}

			if ( m_humans[i].m_isSpawned )
			{
				m_humans[i].Despawn();
			}
		}

		m_columnCount = g_sampleDebug ? 10 : e_maxColumns;
		m_rowCount = g_sampleDebug ? 40 : e_maxRows;

		if ( m_shapeType == e_compoundShape )
		{
			if constexpr ( g_sampleDebug == false )
			{
				m_columnCount = 20;
			}
		}
		else if ( m_shapeType == e_humanShape )
		{
			if constexpr ( g_sampleDebug )
			{
				m_rowCount = 5;
				m_columnCount = 10;
			}
			else
			{
				m_columnCount = 15;
				m_rowCount = 50;
			}
		}

		float rad = 0.5f;

		float shift = 1.15f;
		float centerx = shift * m_columnCount / 2.0f;
		float centery = shift / 2.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = 0.5f;

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

		for ( int i = 0; i < m_columnCount; ++i )
		{
			float x = i * shift - centerx;

			for ( int j = 0; j < m_rowCount; ++j )
			{
				float y = j * ( shift + extray ) + centery + 2.0f;

				bodyDef.position = { x + side, y };
				side = -side;

				if ( m_shapeType == e_circleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					circle.radius = RandomFloat( 0.25f, 0.75f );
					b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
				}
				else if ( m_shapeType == e_caspuleShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );
					capsule.radius = RandomFloat( 0.25f, 0.5f );
					float length = RandomFloat( 0.25f, 1.0f );
					capsule.center1 = { 0.0f, -0.5f * length };
					capsule.center2 = { 0.0f, 0.5f * length };
					b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
				}
				else if ( m_shapeType == e_mixShape )
				{
					m_bodies[index] = b2CreateBody( m_worldId, &bodyDef );

					int mod = index % 3;
					if ( mod == 0 )
					{
						circle.radius = RandomFloat( 0.25f, 0.75f );
						b2CreateCircleShape( m_bodies[index], &shapeDef, &circle );
					}
					else if ( mod == 1 )
					{
						capsule.radius = RandomFloat( 0.25f, 0.5f );
						float length = RandomFloat( 0.25f, 1.0f );
						capsule.center1 = { 0.0f, -0.5f * length };
						capsule.center2 = { 0.0f, 0.5f * length };
						b2CreateCapsuleShape( m_bodies[index], &shapeDef, &capsule );
					}
					else if ( mod == 2 )
					{
						float width = RandomFloat( 0.1f, 0.5f );
						float height = RandomFloat( 0.5f, 0.75f );
						b2Polygon box = b2MakeBox( width, height );

						// Don't put a function call into a macro.
						float value = RandomFloat( -1.0f, 1.0f );
						box.radius = 0.25f * b2MaxFloat( 0.0f, value );
						b2CreatePolygonShape( m_bodies[index], &shapeDef, &box );
					}
					else
					{
						wedge.radius = RandomFloat( 0.1f, 0.25f );
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
					m_humans[index].Spawn( m_worldId, bodyDef.position, 3.5f, 0.05f, 0.0f, 0.0f, index + 1, nullptr, false );
				}

				index += 1;
			}
		}
	}

	void UpdateUI() override
	{
		float height = 80.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
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

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkBarrel( settings );
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
	explicit BenchmarkTumbler( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 1.5f, 10.0f };
			g_camera.m_zoom = 25.0f * 0.6f;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.enableSleep = true;
			bodyDef.position = { 0.0f, 10.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 50.0f;

			b2Polygon polygon;
			polygon = b2MakeOffsetBox( 0.5f, 10.0f, { 10.0f, 0.0f }, 0.0 );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			polygon = b2MakeOffsetBox( 0.5f, 10.0f, { -10.0f, 0.0f }, 0.0 );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			polygon = b2MakeOffsetBox( 10.0f, 0.5f, { 0.0f, 10.0f }, 0.0 );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
			polygon = b2MakeOffsetBox( 10.0f, 0.5f, { 0.0f, -10.0f }, 0.0 );
			b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

			// m_motorSpeed = 9.0f;
			m_motorSpeed = 25.0f;

			b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
			jd.bodyIdA = groundId;
			jd.bodyIdB = bodyId;
			jd.localAnchorA = { 0.0f, 10.0f };
			jd.localAnchorB = { 0.0f, 0.0f };
			jd.referenceAngle = 0.0f;
			jd.motorSpeed = ( b2_pi / 180.0f ) * m_motorSpeed;
			jd.maxMotorTorque = 1e8f;
			jd.enableMotor = true;

			m_jointId = b2CreateRevoluteJoint( m_worldId, &jd );
		}

		int gridCount = g_sampleDebug ? 20 : 45;
		b2Polygon polygon = b2MakeBox( 0.125f, 0.125f );
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = -0.2f * gridCount + 10.0f;
		for ( int i = 0; i < gridCount; ++i )
		{
			float x = -0.2f * gridCount;

			for ( int j = 0; j < gridCount; ++j )
			{
				bodyDef.position = { x, y };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				b2CreatePolygonShape( bodyId, &shapeDef, &polygon );

				x += 0.4f;
			}

			y += 0.4f;
		}
	}

	void UpdateUI() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );
		ImGui::Begin( "Benchmark: Tumbler", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 120.0f );

		if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, 0.0f, 100.0f, "%.f" ) )
		{
			b2RevoluteJoint_SetMotorSpeed( m_jointId, ( b2_pi / 180.0f ) * m_motorSpeed );

			if ( m_motorSpeed > 0.0f )
			{
				b2Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkTumbler( settings );
	}

	b2JointId m_jointId;
	float m_motorSpeed;
};

static int benchmarkTumbler = RegisterSample( "Benchmark", "Tumbler", BenchmarkTumbler::Create );

// todo try removing kinematics from graph coloring
class BenchmarkManyTumblers : public Sample
{
public:
	explicit BenchmarkManyTumblers( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 1.0f, -5.5 };
			g_camera.m_zoom = 25.0f * 3.4f;
			settings.drawJoints = false;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		m_rowCount = g_sampleDebug ? 2 : 19;
		m_columnCount = g_sampleDebug ? 2 : 19;

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
		bodyDef.angularVelocity = ( b2_pi / 180.0f ) * m_angularSpeed;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
		m_tumblerIds[index] = bodyId;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 50.0f;

		b2Polygon polygon;
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { 2.0f, 0.0f }, 0.0 );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 0.25f, 2.0f, { -2.0f, 0.0f }, 0.0 );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, 2.0f }, 0.0 );
		b2CreatePolygonShape( bodyId, &shapeDef, &polygon );
		polygon = b2MakeOffsetBox( 2.0f, 0.25f, { 0.0f, -2.0f }, 0.0 );
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

		int bodiesPerTumbler = g_sampleDebug ? 8 : 50;
		m_bodyCount = bodiesPerTumbler * m_tumblerCount;

		m_bodyIds = static_cast<b2BodyId*>( malloc( m_bodyCount * sizeof( b2BodyId ) ) );

		memset( m_bodyIds, 0, m_bodyCount * sizeof( b2BodyId ) );
		m_bodyIndex = 0;
	}

	void UpdateUI() override
	{
		float height = 110.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
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
				b2Body_SetAngularVelocity( m_tumblerIds[i], ( b2_pi / 180.0f ) * m_angularSpeed );
				b2Body_SetAwake( m_tumblerIds[i], true );
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

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

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkManyTumblers( settings );
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
	explicit BenchmarkLargePyramid( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 50.0f };
			g_camera.m_zoom = 25.0f * 2.2f;
		}

#ifdef NDEBUG
		int baseCount = 100;
#else
		int baseCount = 40;
#endif

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 100.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		float h = 0.5f;
		b2Polygon box = b2MakeRoundedBox( h - 0.05f, h - 0.05f, 0.05f );

		float shift = 1.0f * h;

		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * shift;

			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift - h * baseCount;

				bodyDef.position = { x, y };

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkLargePyramid( settings );
	}
};

static int benchmarkLargePyramid = RegisterSample( "Benchmark", "Large Pyramid", BenchmarkLargePyramid::Create );

class BenchmarkManyPyramids : public Sample
{
public:
	explicit BenchmarkManyPyramids( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 16.0f, 110.0f };
			g_camera.m_zoom = 25.0f * 5.0f;
		}

		m_extent = 0.5f;
		m_round = 0.0f;
		m_baseCount = 10;
		m_rowCount = g_sampleDebug ? 4 : 13;
		m_columnCount = g_sampleDebug ? 4 : 14;
		m_groundId = b2_nullBodyId;
		m_bodyIds = nullptr;
		m_bodyCount = 0;
		m_bodyIndex = 0;

		CreateScene();
	}

	~BenchmarkManyPyramids() override
	{
		free( m_bodyIds );
	}

	void CreatePyramid( float centerX, float baseY )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;

		float h = m_extent - m_round;
		b2Polygon box = b2MakeRoundedBox( h, h, m_round );

		float shift = 1.0f * h;

		for ( int i = 0; i < m_baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * shift + baseY;

			for ( int j = i; j < m_baseCount; ++j )
			{
				float x = ( i + 1.0f ) * shift + 2.0f * ( j - i ) * shift + centerX - 0.5f;

				bodyDef.position = { x, y };

				assert( m_bodyIndex < m_bodyCount );
				m_bodyIds[m_bodyIndex] = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( m_bodyIds[m_bodyIndex], &shapeDef, &box );

				m_bodyIndex += 1;
			}
		}
	}

	void CreateScene()
	{
		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
		}

		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
		}

		free( m_bodyIds );

		m_bodyCount = m_rowCount * m_columnCount * m_baseCount * ( m_baseCount + 1 ) / 2;
		m_bodyIds = (b2BodyId*)malloc( m_bodyCount * sizeof( b2BodyId ) );
		m_bodyIndex = 0;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		float groundDeltaY = 2.0f * m_extent * ( m_baseCount + 1.0f );
		float groundWidth = 2.0f * m_extent * m_columnCount * ( m_baseCount + 1.0f );
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float groundY = 0.0f;

		for ( int i = 0; i < m_rowCount; ++i )
		{
			// b2Segment segment = {{-0.5f * groundWidth, groundY}, {0.5f * groundWidth, groundY}};
			b2Segment segment = { { -0.5f * 2.0f * groundWidth, groundY }, { 0.5f * 2.0f * groundWidth, groundY } };
			b2CreateSegmentShape( m_groundId, &shapeDef, &segment );
			groundY += groundDeltaY;
		}

		float baseWidth = 2.0f * m_extent * m_baseCount;
		float baseY = 0.0f;

		for ( int i = 0; i < m_rowCount; ++i )
		{
			for ( int j = 0; j < m_columnCount; ++j )
			{
				float centerX = -0.5f * groundWidth + j * ( baseWidth + 2.0f * m_extent ) + m_extent;
				CreatePyramid( centerX, baseY );
			}

			baseY += groundDeltaY;
		}

		assert( m_bodyIndex == m_bodyCount );
	}

	void UpdateUI() override
	{
		float height = 160.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );
		ImGui::Begin( "Benchmark: Many Pyramids", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		bool changed = false;
		changed = changed || ImGui::SliderInt( "Row Count", &m_rowCount, 1, 32 );
		changed = changed || ImGui::SliderInt( "Column Count", &m_columnCount, 1, 32 );
		changed = changed || ImGui::SliderInt( "Base Count", &m_baseCount, 1, 30 );

		changed = changed || ImGui::SliderFloat( "Round", &m_round, 0.0f, 0.4f, "%.1f" );
		changed = changed || ImGui::Button( "Reset Scene" );

		if ( changed )
		{
			CreateScene();
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkManyPyramids( settings );
	}

	b2BodyId m_groundId;
	b2BodyId* m_bodyIds;
	int m_bodyCount;
	int m_bodyIndex;
	int m_baseCount;
	int m_rowCount;
	int m_columnCount;
	float m_round;
	float m_extent;
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

	explicit BenchmarkCreateDestroy( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 50.0f };
			g_camera.m_zoom = 25.0f * 2.2f;
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

		m_baseCount = g_sampleDebug ? 40 : 100;
		m_iterations = g_sampleDebug ? 1 : 10;
		m_bodyCount = 0;
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
		shapeDef.friction = 0.5f;

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

	void Step( Settings& settings ) override
	{
		b2Timer timer = b2CreateTimer();

		for ( int i = 0; i < m_iterations; ++i )
		{
			CreateScene();
		}

		float ms = b2GetMilliseconds( &timer );

		g_draw.DrawString( 5, m_textLine, "milliseconds = %g", ms );
		m_textLine += m_textIncrement;

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkCreateDestroy( settings );
	}

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

	explicit BenchmarkSleep( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 50.0f };
			g_camera.m_zoom = 25.0f * 2.2f;
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

		m_baseCount = g_sampleDebug ? 40 : 100;
		m_iterations = g_sampleDebug ? 1 : 41;
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
		shapeDef.friction = 0.5f;

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

	void Step( Settings& settings ) override
	{
		float timeStep = settings.hertz > 0.0f ? 1.0f / settings.hertz : float( 0.0f );

		b2Timer timer = b2CreateTimer();

		for ( int i = 0; i < m_iterations; ++i )
		{
			b2Body_SetAwake( m_bodies[0], m_awake );
			if ( m_awake )
			{
				m_wakeTotal += b2GetMillisecondsAndReset( &timer );
				m_wakeCount += 1;
			}
			else
			{
				m_sleepTotal += b2GetMillisecondsAndReset( &timer );
				m_sleepCount += 1;
			}
			m_awake = !m_awake;
		}

		if ( m_wakeCount > 0 )
		{
			g_draw.DrawString( 5, m_textLine, "wake ave = %g ms", m_wakeTotal / m_wakeCount );
			m_textLine += m_textIncrement;
		}

		if ( m_sleepCount > 0 )
		{
			g_draw.DrawString( 5, m_textLine, "sleep ave = %g ms", m_sleepTotal / m_sleepCount );
			m_textLine += m_textIncrement;
		}

		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkSleep( settings );
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
	explicit BenchmarkJointGrid( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 60.0f, -57.0f };
			g_camera.m_zoom = 25.0f * 2.5f;
		}

		constexpr int N = g_sampleDebug ? 10 : 100;

		// Allocate to avoid huge stack usage
		b2BodyId* bodies = static_cast<b2BodyId*>( malloc( N * N * sizeof( b2BodyId ) ) );
		int index = 0;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = ~2u;

		b2Circle circle = { { 0.0f, 0.0f }, 0.4f };

		b2RevoluteJointDef jd = b2DefaultRevoluteJointDef();
		b2BodyDef bodyDef = b2DefaultBodyDef();

		for ( int k = 0; k < N; ++k )
		{
			for ( int i = 0; i < N; ++i )
			{
				float fk = (float)k;
				float fi = (float)i;

				if ( k >= N / 2 - 3 && k <= N / 2 + 3 && i == 0 )
				{
					bodyDef.type = b2_staticBody;
				}
				else
				{
					bodyDef.type = b2_dynamicBody;
				}

				bodyDef.position = { fk, -fi };

				b2BodyId body = b2CreateBody( m_worldId, &bodyDef );

				b2CreateCircleShape( body, &shapeDef, &circle );

				if ( i > 0 )
				{
					jd.bodyIdA = bodies[index - 1];
					jd.bodyIdB = body;
					jd.localAnchorA = { 0.0f, -0.5f };
					jd.localAnchorB = { 0.0f, 0.5f };
					b2CreateRevoluteJoint( m_worldId, &jd );
				}

				if ( k > 0 )
				{
					jd.bodyIdA = bodies[index - N];
					jd.bodyIdB = body;
					jd.localAnchorA = { 0.5f, 0.0f };
					jd.localAnchorB = { -0.5f, 0.0f };
					b2CreateRevoluteJoint( m_worldId, &jd );
				}

				bodies[index++] = body;
			}
		}

		free( bodies );

		m_gravity = 10.0f;
	}

	void UpdateUI() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );
		ImGui::Begin( "Benchmark: Joint Grid", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::SliderFloat( "gravity", &m_gravity, 0.0f, 20.0f, "%.1f" ) )
		{
			b2World_SetGravity( m_worldId, { 0.0f, -m_gravity } );
		}

		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkJointGrid( settings );
	}

	float m_gravity;
};

static int benchmarkJointGridIndex = RegisterSample( "Benchmark", "Joint Grid", BenchmarkJointGrid::Create );

class BenchmarkSmash : public Sample
{
public:
	explicit BenchmarkSmash( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 60.0f, 6.0f };
			g_camera.m_zoom = 25.0f * 1.6f;
		}

		b2World_SetGravity( m_worldId, b2Vec2_zero );

		{
			b2Polygon box = b2MakeBox( 4.0f, 4.0f );

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -20.0f, 0.0f };
			bodyDef.linearVelocity = { 40.0f, 0.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 8.0f;
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		m_created = false;
	}

	void CreateScene1()
	{
		ImGuiIO& io = ImGui::GetIO();
		if ( io.Fonts->Fonts.size() == 0 )
		{
			return;
		}

		const ImFont* font = io.Fonts->Fonts[0];
		const unsigned char* pixels = font->ContainerAtlas->TexPixelsAlpha8;
		int width = font->ContainerAtlas->TexWidth;
		int height = font->ContainerAtlas->TexHeight;

		float scale = 0.1f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isAwake = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		for ( int i = 0; i < height; ++i )
		{
			for ( int j = 0; j < width; ++j )
			{
				unsigned char value = pixels[i * width + j];
				if ( value != 0 && value != 0xFF )
				{
					value += 0;
				}

				if ( value > 50 )
				{
					b2Polygon square = b2MakeSquare( 0.95f * scale * ( value / 255.0f ) );
					bodyDef.position = { 2.0f * j * scale, 2.0f * ( height - i ) * scale - 10.0f };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
					b2CreatePolygonShape( bodyId, &shapeDef, &square );
				}
			}
		}

		m_created = true;
	}

	void CreateScene2()
	{
		ImGuiIO& io = ImGui::GetIO();
		if ( io.Fonts->Fonts.size() == 0 )
		{
			return;
		}

		const ImFont* font = io.Fonts->Fonts.back();
		const unsigned char* pixels = font->ContainerAtlas->TexPixelsAlpha8;
		int width = font->ContainerAtlas->TexWidth;
		int height = font->ContainerAtlas->TexHeight;
		int fontSize = font->Ascent;

		float scale = 0.1f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isAwake = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		const char* text = "I";
		int n = (int)strlen( text );
		float zoom = 1.0f;

		float x = 0.0f;
		for ( int k = 0; k < n; ++k )
		{
			const ImFontGlyph* glyph = font->FindGlyph( text[k] );
			float x1 = glyph->X0;
			float x2 = glyph->X1;
			float y1 = glyph->Y0;
			float y2 = glyph->Y1;
			float u1 = glyph->U0;
			float v1 = glyph->V0;
			float u2 = glyph->U1;
			float v2 = glyph->V1;

			float w = zoom * ( x2 - x1 );
			float h = zoom * ( y2 - y1 );

			int gridx = int( w );
			int gridy = int( h );
			for ( int i = 0; i < gridy; ++i )
			{
				float v = v1 + i / h * ( v2 - v1 );
				int iy = int( v * height );

				for ( int j = 0; j < gridx; ++j )
				{
					float u = u1 + j / w * ( u2 - u1 );
					int ix = int( u * width );

					unsigned char value = pixels[iy * width + ix];
					if ( value > 50 )
					{
						b2Polygon square = b2MakeSquare( 0.9f * scale * value / 255.0f );
						bodyDef.position = { x + 2.0f * ( zoom * x1 + j ) * scale, -2.0f * ( zoom * y1 + i ) * scale + 13.0f };
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
						b2CreatePolygonShape( bodyId, &shapeDef, &square );
					}
				}
			}

			x += 2.0f * zoom * scale * glyph->AdvanceX;
		}

		m_created = true;
	}

	void CreateScene3()
	{
		float d = 0.4f;
		b2Polygon box = b2MakeSquare( 0.5f * d );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.isAwake = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();

		int columns = g_sampleDebug ? 20 : 120;
		int rows = g_sampleDebug ? 10 : 80;

		for ( int i = 0; i < columns; ++i )
		{
			for ( int j = 0; j < rows; ++j )
			{
				bodyDef.position.x = i * d + 30.0f;
				bodyDef.position.y = ( j - rows / 2.0f ) * d;
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}

		m_created = true;
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if ( m_created == false )
		{
			CreateScene3();
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkSmash( settings );
	}

	bool m_created;
};

static int sampleSmash = RegisterSample( "Benchmark", "Smash", BenchmarkSmash::Create );

class BenchmarkCompound : public Sample
{
public:
	explicit BenchmarkCompound( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 18.0f, 115.0f };
			g_camera.m_zoom = 25.0f * 5.5f;
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
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, 0.0f );
					b2CreatePolygonShape( groundId, &shapeDef, &square );
				}
			}

			for ( int i = 0; i < height; ++i )
			{
				float y = grid * i;
				for ( int j = i; j < width; ++j )
				{
					float x = -grid * j;
					b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, 0.0f );
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
			bodyDef.automaticMass = false;
			b2ShapeDef shapeDef = b2DefaultShapeDef();

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
							b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, 0.0f );
							b2CreatePolygonShape( bodyId, &shapeDef, &square );
						}
					}

					// All shapes have been added so I can efficiently compute the mass properties.
					b2Body_ApplyMassFromShapes( bodyId );
				}
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkCompound( settings );
	}
};

static int sampleCompound = RegisterSample( "Benchmark", "Compound", BenchmarkCompound::Create );

class BenchmarkKinematic : public Sample
{
public:
	explicit BenchmarkKinematic( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 150.0f;
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
		// defer mass properties to avoid n-squared mass computations
		bodyDef.automaticMass = false;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.filter.categoryBits = 1;
		shapeDef.filter.maskBits = 2;

		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		for ( int i = -span; i < span; ++i )
		{
			float y = i * grid;
			for ( int j = -span; j < span; ++j )
			{
				float x = j * grid;
				b2Polygon square = b2MakeOffsetBox( 0.5f * grid, 0.5f * grid, { x, y }, 0.0f );
				b2CreatePolygonShape( bodyId, &shapeDef, &square );
			}
		}

		// All shapes have been added so I can efficiently compute the mass properties.
		b2Body_ApplyMassFromShapes( bodyId );
	}

	static Sample* Create( Settings& settings )
	{
		return new BenchmarkKinematic( settings );
	}
};

static int sampleKinematic = RegisterSample( "Benchmark", "Kinematic", BenchmarkKinematic::Create );
