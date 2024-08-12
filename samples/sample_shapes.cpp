// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class ChainShape : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_capsuleShape,
		e_boxShape
	};

	explicit ChainShape( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 1.75f;
		}

		m_groundId = b2_nullBodyId;
		m_bodyId = b2_nullBodyId;
		m_chainId = b2_nullChainId;
		m_shapeId = b2_nullShapeId;
		m_shapeType = e_circleShape;
		m_restitution = 0.0f;
		m_friction = 0.2f;
		CreateScene();
		Launch();
	}

	void CreateScene()
	{
		if ( B2_IS_NON_NULL( m_groundId ) )
		{
			b2DestroyBody( m_groundId );
		}

		// https://betravis.github.io/shape-tools/path-to-polygon/
		// b2Vec2 points[] = {{-20.58325, 14.54175}, {-21.90625, 15.8645},		 {-24.552, 17.1875},
		//				   {-27.198, 11.89575},	  {-29.84375, 15.8645},		 {-29.84375, 21.15625},
		//				   {-25.875, 23.802},	  {-20.58325, 25.125},		 {-25.875, 29.09375},
		//				   {-20.58325, 31.7395},  {-11.0089998, 23.2290001}, {-8.67700005, 21.15625},
		//				   {-6.03125, 21.15625},  {-7.35424995, 29.09375},	 {-3.38549995, 29.09375},
		//				   {1.90625, 30.41675},	  {5.875, 17.1875},			 {11.16675, 25.125},
		//				   {9.84375, 29.09375},	  {13.8125, 31.7395},		 {21.75, 30.41675},
		//				   {28.3644981, 26.448},  {25.71875, 18.5105},		 {24.3957481, 13.21875},
		//				   {17.78125, 11.89575},  {15.1355, 7.92700005},	 {5.875, 9.25},
		//				   {1.90625, 11.89575},	  {-3.25, 11.89575},		 {-3.25, 9.9375},
		//				   {-4.70825005, 9.25},	  {-8.67700005, 9.25},		 {-11.323, 11.89575},
		//				   {-13.96875, 11.89575}, {-15.29175, 14.54175},	 {-19.2605, 14.54175}};

		b2Vec2 points[] = {
			{ -56.885498, 12.8985004 },	  { -56.885498, 16.2057495 },	{ 56.885498, 16.2057495 },	 { 56.885498, -16.2057514 },
			{ 51.5935059, -16.2057514 },  { 43.6559982, -10.9139996 },	{ 35.7184982, -10.9139996 }, { 27.7809982, -10.9139996 },
			{ 21.1664963, -14.2212505 },  { 11.9059982, -16.2057514 },	{ 0, -16.2057514 },			 { -10.5835037, -14.8827496 },
			{ -17.1980019, -13.5597477 }, { -21.1665001, -12.2370014 }, { -25.1355019, -9.5909977 }, { -31.75, -3.63799858 },
			{ -38.3644981, 6.2840004 },	  { -42.3334999, 9.59125137 },	{ -47.625, 11.5755005 },	 { -56.885498, 12.8985004 },
		};

		int count = sizeof( points ) / sizeof( points[0] );

		// float scale = 0.25f;
		// b2Vec2 lower = {FLT_MAX, FLT_MAX};
		// b2Vec2 upper = {-FLT_MAX, -FLT_MAX};
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i].x = 2.0f * scale * points[i].x;
		//	points[i].y = -scale * points[i].y;

		//	lower = b2Min(lower, points[i]);
		//	upper = b2Max(upper, points[i]);
		//}

		// b2Vec2 center = b2MulSV(0.5f, b2Add(lower, upper));
		// for (int i = 0; i < count; ++i)
		//{
		//	points[i] = b2Sub(points[i], center);
		// }

		// for (int i = 0; i < count / 2; ++i)
		//{
		//	b2Vec2 temp = points[i];
		//	points[i] = points[count - 1 - i];
		//	points[count - 1 - i] = temp;
		// }

		// printf("{");
		// for (int i = 0; i < count; ++i)
		//{
		//	printf("{%.9g, %.9g},", points[i].x, points[i].y);
		// }
		// printf("};\n");

		b2ChainDef chainDef = b2DefaultChainDef();
		chainDef.points = points;
		chainDef.count = count;
		chainDef.isLoop = true;
		chainDef.friction = 0.2f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );

		m_chainId = b2CreateChain( m_groundId, &chainDef );
	}

	void Launch()
	{
		if ( B2_IS_NON_NULL( m_bodyId ) )
		{
			b2DestroyBody( m_bodyId );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { -55.0f, 13.5f };
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.friction = m_friction;
		shapeDef.restitution = m_restitution;

		if ( m_shapeType == e_circleShape )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			m_shapeId = b2CreateCircleShape( m_bodyId, &shapeDef, &circle );
		}
		else if ( m_shapeType == e_capsuleShape )
		{
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0 }, 0.25f };
			m_shapeId = b2CreateCapsuleShape( m_bodyId, &shapeDef, &capsule );
		}
		else
		{
			float h = 0.5f;
			b2Polygon box = b2MakeBox( h, h );
			m_shapeId = b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
		}
	}

	void UpdateUI() override
	{
		float height = 135.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Chain Shape", nullptr, ImGuiWindowFlags_NoResize );

		const char* shapeTypes[] = { "Circle", "Capsule", "Box" };
		int shapeType = int( m_shapeType );
		if ( ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) ) )
		{
			m_shapeType = ShapeType( shapeType );
			Launch();
		}

		if ( ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.2f" ) )
		{
			b2Shape_SetFriction( m_shapeId, m_friction );
			b2Chain_SetFriction( m_chainId, m_friction );
		}

		if ( ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 2.0f, "%.1f" ) )
		{
			b2Shape_SetRestitution( m_shapeId, m_restitution );
		}

		if ( ImGui::Button( "Launch" ) )
		{
			Launch();
		}

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawSegment( b2Vec2_zero, { 0.5f, 0.0f }, b2_colorRed );
		g_draw.DrawSegment( b2Vec2_zero, { 0.0f, 0.5f }, b2_colorGreen );
	}

	static Sample* Create( Settings& settings )
	{
		return new ChainShape( settings );
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyId;
	b2ChainId m_chainId;
	ShapeType m_shapeType;
	b2ShapeId m_shapeId;
	float m_restitution;
	float m_friction;
};

static int sampleChainShape = RegisterSample( "Shapes", "Chain Shape", ChainShape::Create );

// This sample shows how careful creation of compound shapes leads to better simulation and avoids
// objects getting stuck.
// This also shows how to get the combined AABB for the body.
class CompoundShapes : public Sample
{
public:
	explicit CompoundShapes( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 6.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { 50.0f, 0.0f }, { -50.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Table 1
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -15.0f, 1.0f };
			m_table1Id = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon top = b2MakeOffsetBox( 3.0f, 0.5f, { 0.0f, 3.5f }, 0.0f );
			b2Polygon leftLeg = b2MakeOffsetBox( 0.5f, 1.5f, { -2.5f, 1.5f }, 0.0f );
			b2Polygon rightLeg = b2MakeOffsetBox( 0.5f, 1.5f, { 2.5f, 1.5f }, 0.0f );

			b2CreatePolygonShape( m_table1Id, &shapeDef, &top );
			b2CreatePolygonShape( m_table1Id, &shapeDef, &leftLeg );
			b2CreatePolygonShape( m_table1Id, &shapeDef, &rightLeg );
		}

		// Table 2
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -5.0f, 1.0f };
			m_table2Id = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon top = b2MakeOffsetBox( 3.0f, 0.5f, { 0.0f, 3.5f }, 0.0f );
			b2Polygon leftLeg = b2MakeOffsetBox( 0.5f, 2.0f, { -2.5f, 2.0f }, 0.0f );
			b2Polygon rightLeg = b2MakeOffsetBox( 0.5f, 2.0f, { 2.5f, 2.0f }, 0.0f );

			b2CreatePolygonShape( m_table2Id, &shapeDef, &top );
			b2CreatePolygonShape( m_table2Id, &shapeDef, &leftLeg );
			b2CreatePolygonShape( m_table2Id, &shapeDef, &rightLeg );
		}

		// Spaceship 1
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 5.0f, 1.0f };
			m_ship1Id = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Vec2 vertices[3];

			vertices[0] = { -2.0f, 0.0f };
			vertices[1] = { 0.0f, 4.0f / 3.0f };
			vertices[2] = { 0.0f, 4.0f };
			b2Hull hull = b2ComputeHull( vertices, 3 );
			b2Polygon left = b2MakePolygon( &hull, 0.0f );

			vertices[0] = { 2.0f, 0.0f };
			vertices[1] = { 0.0f, 4.0f / 3.0f };
			vertices[2] = { 0.0f, 4.0f };
			hull = b2ComputeHull( vertices, 3 );
			b2Polygon right = b2MakePolygon( &hull, 0.0f );

			b2CreatePolygonShape( m_ship1Id, &shapeDef, &left );
			b2CreatePolygonShape( m_ship1Id, &shapeDef, &right );
		}

		// Spaceship 2
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 15.0f, 1.0f };
			m_ship2Id = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Vec2 vertices[3];

			vertices[0] = { -2.0f, 0.0f };
			vertices[1] = { 1.0f, 2.0f };
			vertices[2] = { 0.0f, 4.0f };
			b2Hull hull = b2ComputeHull( vertices, 3 );
			b2Polygon left = b2MakePolygon( &hull, 0.0f );

			vertices[0] = { 2.0f, 0.0f };
			vertices[1] = { -1.0f, 2.0f };
			vertices[2] = { 0.0f, 4.0f };
			hull = b2ComputeHull( vertices, 3 );
			b2Polygon right = b2MakePolygon( &hull, 0.0f );

			b2CreatePolygonShape( m_ship2Id, &shapeDef, &left );
			b2CreatePolygonShape( m_ship2Id, &shapeDef, &right );
		}

		m_drawBodyAABBs = false;
	}

	void Spawn()
	{
		// Table 1 obstruction
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Body_GetPosition( m_table1Id );
			bodyDef.rotation = b2Body_GetRotation( m_table1Id );
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 4.0f, 0.1f, { 0.0f, 3.0f }, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// Table 2 obstruction
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Body_GetPosition( m_table2Id );
			bodyDef.rotation = b2Body_GetRotation( m_table2Id );
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 4.0f, 0.1f, { 0.0f, 3.0f }, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// Ship 1 obstruction
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Body_GetPosition( m_ship1Id );
			bodyDef.rotation = b2Body_GetRotation( m_ship1Id );
			// bodyDef.gravityScale = 0.0f;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Circle circle = { { 0.0f, 2.0f }, 0.5f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		// Ship 2 obstruction
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = b2Body_GetPosition( m_ship2Id );
			bodyDef.rotation = b2Body_GetRotation( m_ship2Id );
			// bodyDef.gravityScale = 0.0f;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Circle circle = { { 0.0f, 2.0f }, 0.5f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

		ImGui::Begin( "Compound Shapes", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Intrude" ) )
		{
			Spawn();
		}

		ImGui::Checkbox( "Body AABBs", &m_drawBodyAABBs );

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if ( m_drawBodyAABBs )
		{
			b2AABB aabb = b2Body_ComputeAABB( m_table1Id );
			g_draw.DrawAABB( aabb, b2_colorYellow );

			aabb = b2Body_ComputeAABB( m_table2Id );
			g_draw.DrawAABB( aabb, b2_colorYellow );

			aabb = b2Body_ComputeAABB( m_ship1Id );
			g_draw.DrawAABB( aabb, b2_colorYellow );

			aabb = b2Body_ComputeAABB( m_ship2Id );
			g_draw.DrawAABB( aabb, b2_colorYellow );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new CompoundShapes( settings );
	}

	b2BodyId m_table1Id;
	b2BodyId m_table2Id;
	b2BodyId m_ship1Id;
	b2BodyId m_ship2Id;
	bool m_drawBodyAABBs;
};

static int sampleCompoundShape = RegisterSample( "Shapes", "Compound Shapes", CompoundShapes::Create );

class ShapeFilter : public Sample
{
public:
	enum CollisionBits
	{
		GROUND = 0x00000001,
		TEAM1 = 0x00000002,
		TEAM2 = 0x00000004,
		TEAM3 = 0x00000008,

		ALL_BITS = ( ~0u )
	};

	explicit ShapeFilter( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_zoom = 25.0f * 0.5f;
			g_camera.m_center = { 0.0f, 5.0f };
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.filter.categoryBits = GROUND;
			shapeDef.filter.maskBits = ALL_BITS;

			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			bodyDef.position = { 0.0f, 2.0f };
			m_player1Id = b2CreateBody( m_worldId, &bodyDef );

			bodyDef.position = { 0.0f, 5.0f };
			m_player2Id = b2CreateBody( m_worldId, &bodyDef );

			bodyDef.position = { 0.0f, 8.0f };
			m_player3Id = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 2.0f, 1.0f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			shapeDef.filter.categoryBits = TEAM1;
			shapeDef.filter.maskBits = GROUND | TEAM2 | TEAM3;
			m_shape1Id = b2CreatePolygonShape( m_player1Id, &shapeDef, &box );

			shapeDef.filter.categoryBits = TEAM2;
			shapeDef.filter.maskBits = GROUND | TEAM1 | TEAM3;
			m_shape2Id = b2CreatePolygonShape( m_player2Id, &shapeDef, &box );

			shapeDef.filter.categoryBits = TEAM3;
			shapeDef.filter.maskBits = GROUND | TEAM1 | TEAM2;
			m_shape3Id = b2CreatePolygonShape( m_player3Id, &shapeDef, &box );
		}
	}

	void UpdateUI() override
	{
		float height = 240.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Shape Filter", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::Text( "Player 1 Collides With" );
		{
			b2Filter filter1 = b2Shape_GetFilter( m_shape1Id );
			bool team2 = ( filter1.maskBits & TEAM2 ) == TEAM2;
			if ( ImGui::Checkbox( "Team 2##1", &team2 ) )
			{
				if ( team2 )
				{
					filter1.maskBits |= TEAM2;
				}
				else
				{
					filter1.maskBits &= ~TEAM2;
				}

				b2Shape_SetFilter( m_shape1Id, filter1 );
			}

			bool team3 = ( filter1.maskBits & TEAM3 ) == TEAM3;
			if ( ImGui::Checkbox( "Team 3##1", &team3 ) )
			{
				if ( team3 )
				{
					filter1.maskBits |= TEAM3;
				}
				else
				{
					filter1.maskBits &= ~TEAM3;
				}

				b2Shape_SetFilter( m_shape1Id, filter1 );
			}
		}

		ImGui::Separator();

		ImGui::Text( "Player 2 Collides With" );
		{
			b2Filter filter2 = b2Shape_GetFilter( m_shape2Id );
			bool team1 = ( filter2.maskBits & TEAM1 ) == TEAM1;
			if ( ImGui::Checkbox( "Team 1##2", &team1 ) )
			{
				if ( team1 )
				{
					filter2.maskBits |= TEAM1;
				}
				else
				{
					filter2.maskBits &= ~TEAM1;
				}

				b2Shape_SetFilter( m_shape2Id, filter2 );
			}

			bool team3 = ( filter2.maskBits & TEAM3 ) == TEAM3;
			if ( ImGui::Checkbox( "Team 3##2", &team3 ) )
			{
				if ( team3 )
				{
					filter2.maskBits |= TEAM3;
				}
				else
				{
					filter2.maskBits &= ~TEAM3;
				}

				b2Shape_SetFilter( m_shape2Id, filter2 );
			}
		}

		ImGui::Separator();

		ImGui::Text( "Player 3 Collides With" );
		{
			b2Filter filter3 = b2Shape_GetFilter( m_shape3Id );
			bool team1 = ( filter3.maskBits & TEAM1 ) == TEAM1;
			if ( ImGui::Checkbox( "Team 1##3", &team1 ) )
			{
				if ( team1 )
				{
					filter3.maskBits |= TEAM1;
				}
				else
				{
					filter3.maskBits &= ~TEAM1;
				}

				b2Shape_SetFilter( m_shape3Id, filter3 );
			}

			bool team2 = ( filter3.maskBits & TEAM2 ) == TEAM2;
			if ( ImGui::Checkbox( "Team 2##3", &team2 ) )
			{
				if ( team2 )
				{
					filter3.maskBits |= TEAM2;
				}
				else
				{
					filter3.maskBits &= ~TEAM2;
				}

				b2Shape_SetFilter( m_shape3Id, filter3 );
			}
		}

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		b2Vec2 p1 = b2Body_GetPosition( m_player1Id );
		g_draw.DrawString( { p1.x - 0.5f, p1.y }, "player 1" );

		b2Vec2 p2 = b2Body_GetPosition( m_player2Id );
		g_draw.DrawString( { p2.x - 0.5f, p2.y }, "player 2" );

		b2Vec2 p3 = b2Body_GetPosition( m_player3Id );
		g_draw.DrawString( { p3.x - 0.5f, p3.y }, "player 3" );
	}

	static Sample* Create( Settings& settings )
	{
		return new ShapeFilter( settings );
	}

	b2BodyId m_player1Id;
	b2BodyId m_player2Id;
	b2BodyId m_player3Id;

	b2ShapeId m_shape1Id;
	b2ShapeId m_shape2Id;
	b2ShapeId m_shape3Id;
};

static int sampleShapeFilter = RegisterSample( "Shapes", "Filter", ShapeFilter::Create );

// This shows how to use custom filtering
class CustomFilter : public Sample
{
public:
	enum
	{
		e_count = 10
	};

	explicit CustomFilter( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 5.0f };
			g_camera.m_zoom = 10.0f;
		}

		// Register custom filter
		b2World_SetCustomFilterCallback( m_worldId, CustomFilterStatic, this );

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Polygon box = b2MakeSquare( 1.0f );
		float x = -e_count;

		for ( int i = 0; i < e_count; ++i )
		{
			bodyDef.position = { x, 5.0f };
			m_bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );

			shapeDef.userData = reinterpret_cast<void*>( intptr_t( i + 1 ) );
			m_shapeIds[i] = b2CreatePolygonShape( m_bodyIds[i], &shapeDef, &box );
			x += 2.0f;
		}
	}

	void Step( Settings& settings ) override
	{
		g_draw.DrawString( 5, m_textLine, "Custom filter disables collision between odd and even shapes" );
		m_textLine += m_textIncrement;

		Sample::Step( settings );

		for ( int i = 0; i < e_count; ++i )
		{
			b2Vec2 p = b2Body_GetPosition( m_bodyIds[i] );
			g_draw.DrawString( { p.x, p.y }, "%d", i );
		}
	}

	bool ShouldCollide( b2ShapeId shapeIdA, b2ShapeId shapeIdB )
	{
		void* userDataA = b2Shape_GetUserData( shapeIdA );
		void* userDataB = b2Shape_GetUserData( shapeIdB );

		if ( userDataA == NULL || userDataB == NULL )
		{
			return true;
		}

		int indexA = static_cast<int>( reinterpret_cast<intptr_t>( userDataA ) );
		int indexB = static_cast<int>( reinterpret_cast<intptr_t>( userDataB ) );

		return ( ( indexA & 1 ) + ( indexB & 1 ) ) != 1;
	}

	static bool CustomFilterStatic( b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context )
	{
		CustomFilter* customFilter = static_cast<CustomFilter*>( context );
		return customFilter->ShouldCollide( shapeIdA, shapeIdB );
	}

	static Sample* Create( Settings& settings )
	{
		return new CustomFilter( settings );
	}

	b2BodyId m_bodyIds[e_count];
	b2ShapeId m_shapeIds[e_count];
};

static int sampleCustomFilter = RegisterSample( "Shapes", "Custom Filter", CustomFilter::Create );

// Restitution is approximate since Box2D uses speculative collision
class Restitution : public Sample
{
public:
	enum
	{
		e_count = 40
	};

	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	explicit Restitution( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 4.0f, 17.0f };
			g_camera.m_zoom = 27.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			float h = 1.0f * e_count;
			b2Segment segment = { { -h, 0.0f }, { h, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		for ( int i = 0; i < e_count; ++i )
		{
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_shapeType = e_circleShape;

		CreateBodies();
	}

	void CreateBodies()
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = { 0 };
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.restitution = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		float dr = 1.0f / ( e_count > 1 ? e_count - 1 : 1 );
		float x = -1.0f * ( e_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < e_count; ++i )
		{
			bodyDef.position = { x, 40.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			m_bodyIds[i] = bodyId;

			if ( m_shapeType == e_circleShape )
			{
				b2CreateCircleShape( bodyId, &shapeDef, &circle );
			}
			else
			{
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}

			shapeDef.restitution += dr;
			x += dx;
		}
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Restitution", nullptr, ImGuiWindowFlags_NoResize );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		changed = changed || ImGui::Button( "Reset" );

		if ( changed )
		{
			CreateBodies();
		}

		ImGui::End();
	}

	static Sample* Create( Settings& settings )
	{
		return new Restitution( settings );
	}

	b2BodyId m_bodyIds[e_count];
	ShapeType m_shapeType;
};

static int sampleIndex = RegisterSample( "Shapes", "Restitution", Restitution::Create );

class Friction : public Sample
{
public:
	explicit Friction( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 14.0f };
			g_camera.m_zoom = 25.0f * 0.6f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.2f;

			b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			b2Polygon box = b2MakeOffsetBox( 13.0f, 0.25f, { -4.0f, 22.0f }, -0.25f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 0.25f, 1.0f, { 10.5f, 19.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 13.0f, 0.25f, { 4.0f, 14.0f }, 0.25f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 0.25f, 1.0f, { -10.5f, 11.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 13.0f, 0.25f, { -4.0f, 6.0f }, -0.25f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2Polygon box = b2MakeBox( 0.5f, 0.5f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 25.0f;

			float friction[5] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };

			for ( int i = 0; i < 5; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { -15.0f + 4.0f * i, 28.0f };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				shapeDef.friction = friction[i];
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Friction( settings );
	}
};

static int sampleIndex3 = RegisterSample( "Shapes", "Friction", Friction::Create );

// This sample shows how to modify the geometry on an existing shape. This is only supported on
// dynamic and kinematic shapes because static shapes don't look for new collisions.
class ModifyGeometry : public Sample
{
public:
	explicit ModifyGeometry( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_zoom = 25.0f * 0.25f;
			g_camera.m_center = { 0.0f, 5.0f };
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 10.0f, 1.0f, { 0.0f, -1.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 4.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( 1.0f, 1.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		{
			m_shapeType = b2_circleShape;
			m_scale = 1.0f;
			m_circle = { { 0.0f, 0.0f }, 0.5f };
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { 0.0f, 1.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			m_shapeId = b2CreateCircleShape( bodyId, &shapeDef, &m_circle );
		}
	}

	void UpdateShape()
	{
		switch ( m_shapeType )
		{
			case b2_circleShape:
				m_circle = { { 0.0f, 0.0f }, 0.5f * m_scale };
				b2Shape_SetCircle( m_shapeId, &m_circle );
				break;

			case b2_capsuleShape:
				m_capsule = { { -0.5f * m_scale, 0.0f }, { 0.0f, 0.5f * m_scale }, 0.5f * m_scale };
				b2Shape_SetCapsule( m_shapeId, &m_capsule );
				break;

			case b2_segmentShape:
				m_segment = { { -0.5f * m_scale, 0.0f }, { 0.75f * m_scale, 0.0f } };
				b2Shape_SetSegment( m_shapeId, &m_segment );
				break;

			case b2_polygonShape:
				m_polygon = b2MakeBox( 0.5f * m_scale, 0.75f * m_scale );
				b2Shape_SetPolygon( m_shapeId, &m_polygon );
				break;

			default:
				assert( false );
				break;
		}

		b2BodyId bodyId = b2Shape_GetBody( m_shapeId );
		b2Body_ApplyMassFromShapes( bodyId );
	}

	void UpdateUI() override
	{
		float height = 230.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Modify Geometry", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::RadioButton( "Circle", m_shapeType == b2_circleShape ) )
		{
			m_shapeType = b2_circleShape;
			UpdateShape();
		}

		if ( ImGui::RadioButton( "Capsule", m_shapeType == b2_capsuleShape ) )
		{
			m_shapeType = b2_capsuleShape;
			UpdateShape();
		}

		if ( ImGui::RadioButton( "Segment", m_shapeType == b2_segmentShape ) )
		{
			m_shapeType = b2_segmentShape;
			UpdateShape();
		}

		if ( ImGui::RadioButton( "Polygon", m_shapeType == b2_polygonShape ) )
		{
			m_shapeType = b2_polygonShape;
			UpdateShape();
		}

		if ( ImGui::SliderFloat( "Scale", &m_scale, 0.1f, 10.0f, "%.2f" ) )
		{
			UpdateShape();
		}

		b2BodyId bodyId = b2Shape_GetBody( m_shapeId );
		b2BodyType bodyType = b2Body_GetType( bodyId );

		if ( ImGui::RadioButton( "Static", bodyType == b2_staticBody ) )
		{
			b2Body_SetType( bodyId, b2_staticBody );
		}

		if ( ImGui::RadioButton( "Kinematic", bodyType == b2_kinematicBody ) )
		{
			b2Body_SetType( bodyId, b2_kinematicBody );
		}

		if ( ImGui::RadioButton( "Dynamic", bodyType == b2_dynamicBody ) )
		{
			b2Body_SetType( bodyId, b2_dynamicBody );
		}

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );
	}

	static Sample* Create( Settings& settings )
	{
		return new ModifyGeometry( settings );
	}

	b2ShapeId m_shapeId;
	b2ShapeType m_shapeType;
	float m_scale;

	union
	{
		b2Circle m_circle;
		b2Capsule m_capsule;
		b2Segment m_segment;
		b2Polygon m_polygon;
	};
};

static int sampleModifyGeometry = RegisterSample( "Shapes", "Modify Geometry", ModifyGeometry::Create );

// Shows how to link to chain shapes together. This is a useful technique for building large game levels with smooth collision.
class ChainLink : public Sample
{
public:
	explicit ChainLink( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 5.0f };
			g_camera.m_zoom = 25.0f * 0.5f;
		}

		b2Vec2 points1[] = { { 40.0f, 1.0f },	{ 0.0f, 0.0f },	 { -40.0f, 0.0f },
							 { -40.0f, -1.0f }, { 0.0f, -1.0f }, { 40.0f, -1.0f } };
		b2Vec2 points2[] = { { -40.0f, -1.0f }, { 0.0f, -1.0f }, { 40.0f, -1.0f },
							 { 40.0f, 0.0f },	{ 0.0f, 0.0f },	 { -40.0f, 0.0f } };

		int count1 = std::size( points1 );
		int count2 = std::size( points2 );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		{
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points1;
			chainDef.count = count1;
			chainDef.isLoop = false;
			b2CreateChain( groundId, &chainDef );
		}

		{
			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points2;
			chainDef.count = count2;
			chainDef.isLoop = false;
			b2CreateChain( groundId, &chainDef );
		}

		bodyDef.type = b2_dynamicBody;
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		{
			bodyDef.position = { -5.0f, 2.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}

		{
			bodyDef.position = { 0.0f, 2.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0 }, 0.25f };
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}

		{
			bodyDef.position = { 5.0f, 2.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			float h = 0.5f;
			b2Polygon box = b2MakeBox( h, h );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawString( 5, m_textLine, "This shows how to link together two chain shapes" );
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new ChainLink( settings );
	}
};

static int sampleChainLink = RegisterSample( "Shapes", "Chain Link", ChainLink::Create );

class RoundedShapes : public Sample
{
public:
	explicit RoundedShapes( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_zoom = 25.0f * 0.55f;
			g_camera.m_center = { 2.0f, 8.0f };
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 20.0f, 1.0f, { 0.0f, -1.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.0f, 5.0f, { 19.0f, 5.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 1.0f, 5.0f, { -19.0f, 5.0f }, 0.0f );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		// b2Capsule capsule = {{-0.25f, 0.0f}, {0.25f, 0.0f}, 0.25f};
		// b2Circle circle = {{0.0f, 0.0f}, 0.35f};
		// b2Polygon square = b2MakeSquare(0.35f);

		// b2Vec2 points[3] = {{-0.1f, -0.5f}, {0.1f, -0.5f}, {0.0f, 0.5f}};
		// b2Hull wedgeHull = b2ComputeHull(points, 3);
		// b2Polygon wedge = b2MakePolygon(&wedgeHull, 0.0f);

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float y = 2.0f;
		int xcount = 10, ycount = 10;

		for ( int i = 0; i < ycount; ++i )
		{
			float x = -5.0f;
			for ( int j = 0; j < xcount; ++j )
			{
				bodyDef.position = { x, y };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				b2Polygon poly = RandomPolygon( 0.5f );
				poly.radius = RandomFloat( 0.05f, 0.25f );
				b2CreatePolygonShape( bodyId, &shapeDef, &poly );

				x += 1.0f;
			}

			y += 1.0f;
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new RoundedShapes( settings );
	}
};

static int sampleRoundedShapes = RegisterSample( "Shapes", "Rounded", RoundedShapes::Create );

class OffsetShapes : public Sample
{
public:
	explicit OffsetShapes( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_zoom = 25.0f * 0.55f;
			g_camera.m_center = { 2.0f, 8.0f };
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -1.0f, 1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeOffsetBox( 1.0f, 1.0f, { 10.0f, -2.0f }, 0.5f * b2_pi );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		{
			b2Capsule capsule = { { -5.0f, 1.0f }, { -4.0f, 1.0f }, 0.25f };
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 13.5f, -0.75f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );
		}

		{
			b2Polygon box = b2MakeOffsetBox( 0.75f, 0.5f, { 9.0f, 2.0f }, 0.5f * b2_pi );
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		g_draw.DrawTransform( b2Transform_identity );
	}

	static Sample* Create( Settings& settings )
	{
		return new OffsetShapes( settings );
	}
};

static int sampleOffsetShapes = RegisterSample( "Shapes", "Offset", OffsetShapes::Create );
