// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <vector>

class SensorEvent : public Sample
{
public:
	enum
	{
		e_donut = 1,
		e_human = 2,
		e_count = 32
	};

	explicit SensorEvent( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 1.333f;
		}

		settings.drawJoints = false;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			// b2Vec2 points[] = {
			//{42.333, 44.979},	{177.271, 44.979},	{177.271, 100.542}, {142.875, 121.708}, {177.271, 121.708},
			//{177.271, 171.979}, {142.875, 193.146}, {177.271, 193.146}, {177.271, 222.250}, {124.354, 261.938},
			//{124.354, 293.688}, {95.250, 293.688},	{95.250, 261.938},	{42.333, 222.250},	{42.333, 193.146},
			//{76.729, 193.146},	{42.333, 171.979},	{42.333, 121.708},	{76.729, 121.708},	{42.333, 100.542},
			//};

			b2Vec2 points[] = {
				{ -16.8672504, 31.088623 },	 { 16.8672485, 31.088623 },		{ 16.8672485, 17.1978741 },
				{ 8.26824951, 11.906374 },	 { 16.8672485, 11.906374 },		{ 16.8672485, -0.661376953 },
				{ 8.26824951, -5.953125 },	 { 16.8672485, -5.953125 },		{ 16.8672485, -13.229126 },
				{ 3.63799858, -23.151123 },	 { 3.63799858, -31.088623 },	{ -3.63800049, -31.088623 },
				{ -3.63800049, -23.151123 }, { -16.8672504, -13.229126 },	{ -16.8672504, -5.953125 },
				{ -8.26825142, -5.953125 },	 { -16.8672504, -0.661376953 }, { -16.8672504, 11.906374 },
				{ -8.26825142, 11.906374 },	 { -16.8672504, 17.1978741 },
			};

			int count = sizeof( points ) / sizeof( points[0] );

			// float scale = 0.25f;
			// b2Vec2 lower = {FLT_MAX, FLT_MAX};
			// b2Vec2 upper = {-FLT_MAX, -FLT_MAX};
			// for (int i = 0; i < count; ++i)
			//{
			//	points[i].x = scale * points[i].x;
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
			b2CreateChain( groundId, &chainDef );

			float sign = 1.0f;
			float y = 14.0f;
			for ( int i = 0; i < 3; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.position = { 0.0f, y };
				bodyDef.type = b2_dynamicBody;

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				b2Polygon box = b2MakeBox( 6.0f, 0.5f );
				b2ShapeDef shapeDef = b2DefaultShapeDef();
				shapeDef.friction = 0.1f;
				shapeDef.restitution = 1.0f;
				shapeDef.density = 1.0f;

				b2CreatePolygonShape( bodyId, &shapeDef, &box );

				b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
				revoluteDef.bodyIdA = groundId;
				revoluteDef.bodyIdB = bodyId;
				revoluteDef.localAnchorA = bodyDef.position;
				revoluteDef.localAnchorB = b2Vec2_zero;
				revoluteDef.maxMotorTorque = 200.0f;
				revoluteDef.motorSpeed = 2.0f * sign;
				revoluteDef.enableMotor = true;

				b2CreateRevoluteJoint( m_worldId, &revoluteDef );

				y -= 14.0f;
				sign = -sign;
			}

			{
				b2Polygon box = b2MakeOffsetBox( 4.0f, 1.0f, { 0.0f, -30.5f }, b2Rot_identity );
				b2ShapeDef shapeDef = b2DefaultShapeDef();
				shapeDef.isSensor = true;
				b2CreatePolygonShape( groundId, &shapeDef, &box );
			}
		}

		m_wait = 0.5f;
		m_side = -15.0f;
		m_type = e_human;

		for ( int i = 0; i < e_count; ++i )
		{
			m_isSpawned[i] = false;
		}

		CreateElement();
	}

	void CreateElement()
	{
		int index = -1;
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_isSpawned[i] == false )
			{
				index = i;
				break;
			}
		}

		if ( index == -1 )
		{
			return;
		}

		b2Vec2 center = { m_side, 29.5f };

		if ( m_type == e_donut )
		{
			Donut* donut = m_donuts + index;
			// donut->Spawn(m_worldId, center, index + 1, donut);
			donut->Spawn( m_worldId, center, 1.0f, 0, donut );
		}
		else
		{
			Human* human = m_humans + index;
			human->Spawn( m_worldId, center, 2.0f, 0.05f, 0.0f, 0.0f, index + 1, human, false );
		}

		m_isSpawned[index] = true;
		m_side = -m_side;
	}

	void DestroyElement( int index )
	{
		if ( m_type == e_donut )
		{
			Donut* donut = m_donuts + index;
			donut->Despawn();
		}
		else
		{
			Human* human = m_humans + index;
			human->Despawn();
		}

		m_isSpawned[index] = false;
	}

	void Clear()
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( m_isSpawned[i] == true )
			{
				if ( m_type == e_donut )
				{
					m_donuts[i].Despawn();
				}
				else
				{
					m_humans[i].Despawn();
				}

				m_isSpawned[i] = false;
			}
		}
	}

	void UpdateUI() override
	{
		float height = 90.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 140.0f, height ) );

		ImGui::Begin( "Sensor Event", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::RadioButton( "donut", m_type == e_donut ) )
		{
			Clear();
			m_type = e_donut;
		}

		if ( ImGui::RadioButton( "human", m_type == e_human ) )
		{
			Clear();
			m_type = e_human;
		}

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		if ( m_stepCount == 832 )
		{
			m_stepCount += 0;
		}

		Sample::Step( settings );

		// Discover rings that touch the bottom sensor
		bool deferredDestructions[e_count] = {};
		b2SensorEvents sensorEvents = b2World_GetSensorEvents( m_worldId );
		for ( int i = 0; i < sensorEvents.beginCount; ++i )
		{
			b2SensorBeginTouchEvent event = sensorEvents.beginEvents[i];
			b2ShapeId visitorId = event.visitorShapeId;
			b2BodyId bodyId = b2Shape_GetBody( visitorId );

			if ( m_type == e_donut )
			{
				Donut* donut = (Donut*)b2Body_GetUserData( bodyId );
				if ( donut != nullptr )
				{
					int index = (int)( donut - m_donuts );
					assert( 0 <= index && index < e_count );

					// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
					deferredDestructions[index] = true;
				}
			}
			else
			{
				Human* human = (Human*)b2Body_GetUserData( bodyId );
				if ( human != nullptr )
				{
					int index = (int)( human - m_humans );
					assert( 0 <= index && index < e_count );

					// Defer destruction to avoid double destruction and event invalidation (orphaned shape ids)
					deferredDestructions[index] = true;
				}
			}
		}

		// todo destroy mouse joint if necessary

		// Safely destroy rings that hit the bottom sensor
		for ( int i = 0; i < e_count; ++i )
		{
			if ( deferredDestructions[i] )
			{
				DestroyElement( i );
			}
		}

		if ( settings.hertz > 0.0f && settings.pause == false )
		{
			m_wait -= 1.0f / settings.hertz;
			if ( m_wait < 0.0f )
			{
				CreateElement();
				m_wait += 0.5f;
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new SensorEvent( settings );
	}

	Human m_humans[e_count];
	Donut m_donuts[e_count];
	bool m_isSpawned[e_count];
	int m_type;
	float m_wait;
	float m_side;
};

static int sampleSensorEvent = RegisterSample( "Events", "Sensor", SensorEvent::Create );

struct BodyUserData
{
	int index;
};

class ContactEvent : public Sample
{
public:
	enum
	{
		e_count = 20
	};

	explicit ContactEvent( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 25.0f * 1.75f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[] = { { 40.0f, -40.0f }, { -40.0f, -40.0f }, { -40.0f, 40.0f }, { 40.0f, 40.0f } };

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.count = 4;
			chainDef.points = points;
			chainDef.isLoop = true;

			b2CreateChain( groundId, &chainDef );
		}

		// Player
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.gravityScale = 0.0f;
			bodyDef.linearDamping = 0.5f;
			bodyDef.angularDamping = 0.5f;
			bodyDef.isBullet = true;
			m_playerId = b2CreateBody( m_worldId, &bodyDef );

			b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// Enable contact events for the player shape
			shapeDef.enableContactEvents = true;

			m_coreShapeId = b2CreateCircleShape( m_playerId, &shapeDef, &circle );
		}

		for ( int i = 0; i < e_count; ++i )
		{
			m_debrisIds[i] = b2_nullBodyId;
			m_bodyUserData[i].index = i;
		}

		m_wait = 0.5f;
		m_force = 200.0f;
	}

	void SpawnDebris()
	{
		int index = -1;
		for ( int i = 0; i < e_count; ++i )
		{
			if ( B2_IS_NULL( m_debrisIds[i] ) )
			{
				index = i;
				break;
			}
		}

		if ( index == -1 )
		{
			return;
		}

		// Debris
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { RandomFloat( -38.0f, 38.0f ), RandomFloat( -38.0f, 38.0f ) };
		bodyDef.rotation = b2MakeRot( RandomFloat( -b2_pi, b2_pi ) );
		bodyDef.linearVelocity = { RandomFloat( -5.0f, 5.0f ), RandomFloat( -5.0f, 5.0f ) };
		bodyDef.angularVelocity = RandomFloat( -1.0f, 1.0f );
		bodyDef.gravityScale = 0.0f;
		bodyDef.userData = m_bodyUserData + index;
		m_debrisIds[index] = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.restitution = 0.8f;

		// No events when debris hits debris
		shapeDef.enableContactEvents = false;

		if ( ( index + 1 ) % 3 == 0 )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
			b2CreateCircleShape( m_debrisIds[index], &shapeDef, &circle );
		}
		else if ( ( index + 1 ) % 2 == 0 )
		{
			b2Capsule capsule = { { 0.0f, -0.25f }, { 0.0f, 0.25f }, 0.25f };
			b2CreateCapsuleShape( m_debrisIds[index], &shapeDef, &capsule );
		}
		else
		{
			b2Polygon box = b2MakeBox( 0.4f, 0.6f );
			b2CreatePolygonShape( m_debrisIds[index], &shapeDef, &box );
		}
	}

	void UpdateUI() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Contact Event", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "force", &m_force, 100.0f, 500.0f, "%.1f" );

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		g_draw.DrawString( 5, m_textLine, "move using WASD" );
		m_textLine += m_textIncrement;

		b2Vec2 position = b2Body_GetPosition( m_playerId );

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
		{
			b2Body_ApplyForce( m_playerId, { -m_force, 0.0f }, position, true );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
		{
			b2Body_ApplyForce( m_playerId, { m_force, 0.0f }, position, true );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_W ) == GLFW_PRESS )
		{
			b2Body_ApplyForce( m_playerId, { 0.0f, m_force }, position, true );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_S ) == GLFW_PRESS )
		{
			b2Body_ApplyForce( m_playerId, { 0.0f, -m_force }, position, true );
		}

		Sample::Step( settings );

		// Discover rings that touch the bottom sensor
		int debrisToAttach[e_count] = {};
		b2ShapeId shapesToDestroy[e_count] = { b2_nullShapeId };
		int attachCount = 0;
		int destroyCount = 0;

		std::vector<b2ContactData> contactData;

		b2ContactEvents contactEvents = b2World_GetContactEvents( m_worldId );
		for ( int i = 0; i < contactEvents.beginCount; ++i )
		{
			b2ContactBeginTouchEvent event = contactEvents.beginEvents[i];
			b2BodyId bodyIdA = b2Shape_GetBody( event.shapeIdA );
			b2BodyId bodyIdB = b2Shape_GetBody( event.shapeIdB );

			int capacityA = b2Shape_GetContactCapacity( event.shapeIdA );
			contactData.resize( capacityA );
			int countA = b2Shape_GetContactData( event.shapeIdA, contactData.data(), capacityA );
			assert( countA >= 1 );

			for (int j = 0; j < countA; ++j)
			{
				b2Manifold manifold = contactData[j].manifold;
				b2Vec2 normal = manifold.normal;
				assert( b2AbsFloat( b2Length( normal ) - 1.0f ) < 4.0f * FLT_EPSILON );

				for (int k = 0; k < manifold.pointCount; ++k)
				{
					b2ManifoldPoint point = manifold.points[k];
					g_draw.DrawSegment( point.point, point.point + 4.0f * normal, b2_colorBlueViolet );
					g_draw.DrawPoint( point.point, 10.0f, b2_colorWhite );
				}
			}

			int capacityB = b2Shape_GetContactCapacity( event.shapeIdB );
			contactData.resize( capacityB );
			int countB = b2Shape_GetContactData( event.shapeIdB, contactData.data(), capacityB );
			assert( countB >= 1 );

			for (int j = 0; j < countB; ++j)
			{
				b2Manifold manifold = contactData[j].manifold;
				b2Vec2 normal = manifold.normal;
				assert( b2AbsFloat( b2Length( normal ) - 1.0f ) < 4.0f * FLT_EPSILON );

				for (int k = 0; k < manifold.pointCount; ++k)
				{
					b2ManifoldPoint point = manifold.points[k];
					g_draw.DrawSegment( point.point, point.point + 4.0f * normal, b2_colorYellowGreen );
					g_draw.DrawPoint( point.point, 10.0f, b2_colorWhite );
				}
			}

			if ( B2_ID_EQUALS( bodyIdA, m_playerId ) )
			{
				BodyUserData* userDataB = static_cast<BodyUserData*>( b2Body_GetUserData( bodyIdB ) );
				if ( userDataB == nullptr )
				{
					if ( B2_ID_EQUALS( event.shapeIdA, m_coreShapeId ) == false && destroyCount < e_count )
					{
						// player non-core shape hit the wall

						bool found = false;
						for ( int j = 0; j < destroyCount; ++j )
						{
							if ( B2_ID_EQUALS( event.shapeIdA, shapesToDestroy[j] ) )
							{
								found = true;
								break;
							}
						}

						// avoid double deletion
						if ( found == false )
						{
							shapesToDestroy[destroyCount] = event.shapeIdA;
							destroyCount += 1;
						}
					}
				}
				else if ( attachCount < e_count )
				{
					debrisToAttach[attachCount] = userDataB->index;
					attachCount += 1;
				}
			}
			else
			{
				// Only expect events for the player
				assert( B2_ID_EQUALS( bodyIdB, m_playerId ) );
				BodyUserData* userDataA = static_cast<BodyUserData*>( b2Body_GetUserData( bodyIdA ) );
				if ( userDataA == nullptr )
				{
					if ( B2_ID_EQUALS( event.shapeIdB, m_coreShapeId ) == false && destroyCount < e_count )
					{
						// player non-core shape hit the wall

						bool found = false;
						for ( int j = 0; j < destroyCount; ++j )
						{
							if ( B2_ID_EQUALS( event.shapeIdB, shapesToDestroy[j] ) )
							{
								found = true;
								break;
							}
						}

						// avoid double deletion
						if ( found == false )
						{
							shapesToDestroy[destroyCount] = event.shapeIdB;
							destroyCount += 1;
						}
					}
				}
				else if ( attachCount < e_count )
				{
					debrisToAttach[attachCount] = userDataA->index;
					attachCount += 1;
				}
			}
		}

		// Attach debris to player body
		for ( int i = 0; i < attachCount; ++i )
		{
			int index = debrisToAttach[i];
			b2BodyId debrisId = m_debrisIds[index];
			if ( B2_IS_NULL( debrisId ) )
			{
				continue;
			}

			b2Transform playerTransform = b2Body_GetTransform( m_playerId );
			b2Transform debrisTransform = b2Body_GetTransform( debrisId );
			b2Transform relativeTransform = b2InvMulTransforms( playerTransform, debrisTransform );

			int shapeCount = b2Body_GetShapeCount( debrisId );
			if ( shapeCount == 0 )
			{
				continue;
			}

			b2ShapeId shapeId;
			b2Body_GetShapes( debrisId, &shapeId, 1 );

			b2ShapeType type = b2Shape_GetType( shapeId );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.enableContactEvents = true;

			switch ( type )
			{
				case b2_circleShape:
				{
					b2Circle circle = b2Shape_GetCircle( shapeId );
					circle.center = b2TransformPoint( relativeTransform, circle.center );

					b2CreateCircleShape( m_playerId, &shapeDef, &circle );
				}
				break;

				case b2_capsuleShape:
				{
					b2Capsule capsule = b2Shape_GetCapsule( shapeId );
					capsule.center1 = b2TransformPoint( relativeTransform, capsule.center1 );
					capsule.center2 = b2TransformPoint( relativeTransform, capsule.center2 );

					b2CreateCapsuleShape( m_playerId, &shapeDef, &capsule );
				}
				break;

				case b2_polygonShape:
				{
					b2Polygon originalPolygon = b2Shape_GetPolygon( shapeId );
					b2Polygon polygon = b2TransformPolygon( relativeTransform, &originalPolygon );

					b2CreatePolygonShape( m_playerId, &shapeDef, &polygon );
				}
				break;

				default:
					assert( false );
			}

			b2DestroyBody( debrisId );
			m_debrisIds[index] = b2_nullBodyId;
		}

		for ( int i = 0; i < destroyCount; ++i )
		{
			b2DestroyShape( shapesToDestroy[i] );
		}

		if ( settings.hertz > 0.0f && settings.pause == false )
		{
			m_wait -= 1.0f / settings.hertz;
			if ( m_wait < 0.0f )
			{
				SpawnDebris();
				m_wait += 0.5f;
			}
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new ContactEvent( settings );
	}

	b2BodyId m_playerId;
	b2ShapeId m_coreShapeId;
	b2BodyId m_debrisIds[e_count];
	BodyUserData m_bodyUserData[e_count];
	float m_force;
	float m_wait;
};

static int sampleWeeble = RegisterSample( "Events", "Contact", ContactEvent::Create );

// Shows how to make a rigid body character mover and use the pre-solve callback.
class Platformer : public Sample
{
public:
	explicit Platformer( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.5f, 7.5f };
			g_camera.m_zoom = 25.0f * 0.4f;
		}

		b2World_SetPreSolveCallback( m_worldId, PreSolveStatic, this );

		// Ground
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Platform
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_kinematicBody;
			bodyDef.position = { 0.0f, 6.0f };
			bodyDef.linearVelocity = { 2.0f, 0.0f };
			m_platformId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( 3.0f, 0.5f );
			m_platformShapeId = b2CreatePolygonShape( m_platformId, &shapeDef, &box );
		}

		// Actor
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.fixedRotation = true;
			bodyDef.linearDamping = 0.5f;
			bodyDef.position = { 0.0f, 1.0f };
			m_characterId = b2CreateBody( m_worldId, &bodyDef );

			m_radius = 0.5f;
			b2Capsule capsule = { { 0.0f, 0.0f }, { 0.0f, 1.0f }, m_radius };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.1f;

			// Need to turn this on to get the callback
			shapeDef.enablePreSolveEvents = true;

			b2CreateCapsuleShape( m_characterId, &shapeDef, &capsule );
		}

		m_force = 25.0f;
		m_impulse = 25.0f;
		m_jumpDelay = 0.25f;
		m_jumping = false;
	}

	static bool PreSolveStatic( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context )
	{
		Platformer* platformer = static_cast<Platformer*>( context );
		return platformer->PreSolve( shapeIdA, shapeIdB, manifold );
	}

	// This callback must be thread-safe. It may be called multiple times simultaneously.
	// Notice how this method is constant and doesn't change any data. It also
	// does not try to access any values in the world that may be changing, such as contact data.
	bool PreSolve( b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold ) const
	{
		assert( b2Shape_IsValid( shapeIdA ) );
		assert( b2Shape_IsValid( shapeIdB ) );

		b2ShapeId actorShapeId = b2_nullShapeId;
		float sign = 0.0f;
		if ( B2_ID_EQUALS( shapeIdA, m_platformShapeId ) )
		{
			sign = 1.0f;
			actorShapeId = shapeIdB;
		}
		else if ( B2_ID_EQUALS( shapeIdB, m_platformShapeId ) )
		{
			sign = -1.0f;
			actorShapeId = shapeIdA;
		}
		else
		{
			// not the platform, enable contact
			return true;
		}

		b2BodyId bodyId = b2Shape_GetBody( actorShapeId );
		if ( B2_ID_EQUALS( bodyId, m_characterId ) == false )
		{
			// not the character, enable contact
			return true;
		}

		b2Vec2 normal = manifold->normal;
		if ( sign * normal.y > 0.95f )
		{
			return true;
		}

		float separation = 0.0f;
		for ( int i = 0; i < manifold->pointCount; ++i )
		{
			float s = manifold->points[i].separation;
			separation = separation < s ? separation : s;
		}

		if ( separation > 0.1f * m_radius )
		{
			// shallow overlap
			return true;
		}

		// normal points down, disable contact
		return false;
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Platformer", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "force", &m_force, 0.0f, 50.0f, "%.1f" );
		ImGui::SliderFloat( "impulse", &m_impulse, 0.0f, 50.0f, "%.1f" );

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		bool canJump = false;
		b2Vec2 velocity = b2Body_GetLinearVelocity( m_characterId );
		if ( m_jumpDelay == 0.0f && m_jumping == false && velocity.y < 0.01f )
		{
			int capacity = b2Body_GetContactCapacity( m_characterId );
			capacity = b2MinInt( capacity, 4 );
			b2ContactData contactData[4];
			int count = b2Body_GetContactData( m_characterId, contactData, capacity );
			for ( int i = 0; i < count; ++i )
			{
				b2BodyId bodyIdA = b2Shape_GetBody( contactData[i].shapeIdA );
				float sign = 0.0f;
				if ( B2_ID_EQUALS( bodyIdA, m_characterId ) )
				{
					// normal points from A to B
					sign = -1.0f;
				}
				else
				{
					sign = 1.0f;
				}

				if ( sign * contactData[i].manifold.normal.y > 0.9f )
				{
					canJump = true;
					break;
				}
			}
		}

		// A kinematic body is moved by setting its velocity. This
		// ensure friction works correctly.
		b2Vec2 platformPosition = b2Body_GetPosition( m_platformId );
		if ( platformPosition.x < -15.0f )
		{
			b2Body_SetLinearVelocity( m_platformId, { 2.0f, 0.0f } );
		}
		else if ( platformPosition.x > 15.0f )
		{
			b2Body_SetLinearVelocity( m_platformId, { -2.0f, 0.0f } );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_A ) == GLFW_PRESS )
		{
			b2Body_ApplyForceToCenter( m_characterId, { -m_force, 0.0f }, true );
		}

		if ( glfwGetKey( g_mainWindow, GLFW_KEY_D ) == GLFW_PRESS )
		{
			b2Body_ApplyForceToCenter( m_characterId, { m_force, 0.0f }, true );
		}

		int keyState = glfwGetKey( g_mainWindow, GLFW_KEY_SPACE );
		if ( keyState == GLFW_PRESS )
		{
			if ( canJump )
			{
				b2Body_ApplyLinearImpulseToCenter( m_characterId, { 0.0f, m_impulse }, true );
				m_jumpDelay = 0.5f;
				m_jumping = true;
			}
		}
		else
		{
			m_jumping = false;
		}

		Sample::Step( settings );

		g_draw.DrawString( 5, m_textLine, "Movement: A/D/Space" );
		m_textLine += m_textIncrement;

		g_draw.DrawString( 5, m_textLine, "Can jump = %s", canJump ? "true" : "false" );
		m_textLine += m_textIncrement;

		if ( settings.hertz > 0.0f )
		{
			m_jumpDelay = b2MaxFloat( 0.0f, m_jumpDelay - 1.0f / settings.hertz );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new Platformer( settings );
	}

	bool m_jumping;
	float m_radius;
	float m_force;
	float m_impulse;
	float m_jumpDelay;
	b2BodyId m_characterId;
	b2BodyId m_platformId;
	b2ShapeId m_platformShapeId;
};

static int samplePlatformer = RegisterSample( "Events", "Platformer", Platformer::Create );

// This shows how to process body events.
class BodyMove : public Sample
{
public:
	enum
	{
		e_count = 50
	};

	explicit BodyMove( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 2.0f, 8.0f };
			g_camera.m_zoom = 25.0f * 0.55f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.friction = 0.1f;

			b2Polygon box = b2MakeOffsetBox( 12.0f, 0.1f, { -10.0f, -0.1f }, b2MakeRot(-0.15f * b2_pi) );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 12.0f, 0.1f, { 10.0f, -0.1f }, b2MakeRot(0.15f * b2_pi) );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			shapeDef.restitution = 0.8f;

			box = b2MakeOffsetBox( 0.1f, 10.0f, { 19.9f, 10.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 0.1f, 10.0f, { -19.9f, 10.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );

			box = b2MakeOffsetBox( 20.0f, 0.1f, { 0.0f, 20.1f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_sleepCount = 0;
		m_count = 0;

		m_explosionPosition = { 0.0f, -5.0f };
		m_explosionRadius = 10.0f;
		m_explosionMagnitude = 10.0f;
	}

	void CreateBodies()
	{
		b2Capsule capsule = { { -0.25f, 0.0f }, { 0.25f, 0.0f }, 0.25f };
		b2Circle circle = { { 0.0f, 0.0f }, 0.35f };
		b2Polygon square = b2MakeSquare( 0.35f );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		float x = -5.0f, y = 10.0f;
		for ( int32_t i = 0; i < 10 && m_count < e_count; ++i )
		{
			bodyDef.position = { x, y };
			bodyDef.userData = m_bodyIds + m_count;
			m_bodyIds[m_count] = b2CreateBody( m_worldId, &bodyDef );
			m_sleeping[m_count] = false;

			int remainder = m_count % 4;
			if ( remainder == 0 )
			{
				b2CreateCapsuleShape( m_bodyIds[m_count], &shapeDef, &capsule );
			}
			else if ( remainder == 1 )
			{
				b2CreateCircleShape( m_bodyIds[m_count], &shapeDef, &circle );
			}
			else if ( remainder == 2 )
			{
				b2CreatePolygonShape( m_bodyIds[m_count], &shapeDef, &square );
			}
			else
			{
				b2Polygon poly = RandomPolygon( 0.75f );
				poly.radius = 0.1f;
				b2CreatePolygonShape( m_bodyIds[m_count], &shapeDef, &poly );
			}

			m_count += 1;
			x += 1.0f;
		}
	}

	void UpdateUI() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, g_camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Body Move", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "Explode" ) )
		{
			b2ExplosionDef def = b2DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = m_explosionRadius;
			def.falloff = 0.1f;
			def.impulsePerLength = m_explosionMagnitude;
			b2World_Explode( m_worldId, &def );
		}

		ImGui::SliderFloat( "Magnitude", &m_explosionMagnitude, -20.0f, 20.0f, "%.1f" );

		ImGui::End();
	}

	void Step( Settings& settings ) override
	{
		if ( settings.pause == false && ( m_stepCount & 15 ) == 15 && m_count < e_count )
		{
			CreateBodies();
		}

		Sample::Step( settings );

		// Process body events
		b2BodyEvents events = b2World_GetBodyEvents( m_worldId );
		for ( int i = 0; i < events.moveCount; ++i )
		{
			// draw the transform of every body that moved (not sleeping)
			const b2BodyMoveEvent* event = events.moveEvents + i;
			g_draw.DrawTransform( event->transform );

			// this shows a somewhat contrived way to track body sleeping
			b2BodyId* bodyId = static_cast<b2BodyId*>( event->userData );
			ptrdiff_t diff = bodyId - m_bodyIds;
			bool* sleeping = m_sleeping + diff;

			if ( event->fellAsleep )
			{
				*sleeping = true;
				m_sleepCount += 1;
			}
			else
			{
				if ( *sleeping )
				{
					*sleeping = false;
					m_sleepCount -= 1;
				}
			}
		}

		g_draw.DrawCircle( m_explosionPosition, m_explosionRadius, b2_colorAzure );

		g_draw.DrawString( 5, m_textLine, "sleep count: %d", m_sleepCount );
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new BodyMove( settings );
	}

	b2BodyId m_bodyIds[e_count];
	bool m_sleeping[e_count];
	int m_count;
	int m_sleepCount;
	b2Vec2 m_explosionPosition;
	float m_explosionRadius;
	float m_explosionMagnitude;
};

static int sampleBodyMove = RegisterSample( "Events", "Body Move", BodyMove::Create );
