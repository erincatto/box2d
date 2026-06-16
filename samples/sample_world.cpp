// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "car.h"
#include "donut.h"
#include "draw.h"
#include "human.h"
#include "sample.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

class TileWorld : public Sample
{
public:
	explicit TileWorld( SampleContext* context )
		: Sample( context )
	{
		m_period = 40.0f;
		float omega = 2.0f * B2_PI / m_period;
		m_cycleCount = m_isDebug ? 10 : 600;
		m_gridSize = 1.0f;
		m_gridCount = (int)( m_cycleCount * m_period / m_gridSize );

		float xStart = -0.5f * ( m_cycleCount * m_period );

		m_viewPosition = { xStart, 15.0f };

		if ( m_context->restart == false )
		{
			m_context->camera.center = m_viewPosition;
			m_context->camera.zoom = 25.0f * 1.0f;
			m_context->debugDraw.drawJoints = false;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			// Setting this to false significantly reduces the cost of creating
			// static bodies and shapes.
			shapeDef.invokeContactCreation = false;

			float height = 4.0f;
			float xBody = xStart;
			float xShape = xStart;

			b2BodyId groundId;

			for ( int i = 0; i < m_gridCount; ++i )
			{
				// Create a new body regularly so that shapes are not too far from the body origin.
				// Most algorithms in Box2D work in local coordinates, but contact points are computed
				// relative to the body origin.
				// This makes a noticeable improvement in stability far from the origin.
				if ( i % 10 == 0 )
				{
					bodyDef.position.x = xBody;
					groundId = b2CreateBody( m_worldId, &bodyDef );
					xShape = 0.0f;
				}

				float y = 0.0f;

				int ycount = (int)roundf( height * cosf( omega * xBody ) ) + 12;

				for ( int j = 0; j < ycount; ++j )
				{
					b2Polygon square = b2MakeOffsetBox( 0.4f * m_gridSize, 0.4f * m_gridSize, { xShape, y }, b2Rot_identity );
					square.radius = 0.1f;
					b2CreatePolygonShape( groundId, &shapeDef, &square );

					y += m_gridSize;
				}

				xBody += m_gridSize;
				xShape += m_gridSize;
			}
		}

		int humanIndex = 0;
		for ( int cycleIndex = 0; cycleIndex < m_cycleCount; ++cycleIndex )
		{
			float xbase = ( 0.5f + cycleIndex ) * m_period + xStart;

			int remainder = cycleIndex % 3;
			if ( remainder == 0 )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { xbase - 3.0f, 10.0f };

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2Polygon box = b2MakeBox( 0.3f, 0.2f );

				for ( int i = 0; i < 10; ++i )
				{
					bodyDef.position.y = 10.0f;
					for ( int j = 0; j < 5; ++j )
					{
						b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
						b2CreatePolygonShape( bodyId, &shapeDef, &box );
						bodyDef.position.y += 0.5f;
					}
					bodyDef.position.x += 0.6f;
				}
			}
			else if ( remainder == 1 )
			{
				b2Pos position = { xbase - 2.0f, 10.0f };
				for ( int i = 0; i < 5; ++i )
				{
					Human human = {};
					CreateHuman( &human, m_worldId, position, 1.5f, 0.05f, 0.0f, 0.0f, humanIndex + 1, nullptr, false );
					humanIndex += 1;
					position.x += 1.0f;
				}
			}
			else
			{
				b2Pos position = { xbase - 4.0f, 12.0f };

				for ( int i = 0; i < 5; ++i )
				{
					Donut donut;
					donut.Create( m_worldId, position, 0.75f, 0, false, nullptr );
					position.x += 2.0f;
				}
			}
		}

		m_car.Spawn( m_worldId, { xStart + 20.0f, 40.0f }, 10.0f, 2.0f, 0.7f, 2000.0f, nullptr );

		m_cycleIndex = 0;
		m_speed = 0.0f;
		m_explosionPosition = { ( 0.5f + m_cycleIndex ) * m_period + xStart, 7.0f };
		m_explode = true;
		m_followCar = false;
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		ImGui::SliderFloat( "speed", &m_speed, -400.0f, 400.0f, "%.0f" );
		ImGui::PopItemWidth();
		if ( ImGui::Button( "stop" ) )
		{
			m_speed = 0.0f;
		}

		ImGui::Checkbox( "explode", &m_explode );
		ImGui::Checkbox( "follow car", &m_followCar );

		ImGui::Text( "world size = %g kilometers", m_gridSize * m_gridCount / 1000.0f );

		return true;
	}

	void Step() override
	{
		float span = 0.5f * ( m_period * m_cycleCount );
		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		if ( m_context->pause )
		{
			timeStep = 0.0f;
		}

		m_viewPosition.x += timeStep * m_speed;
		m_viewPosition.x = b2ClampFloat( m_viewPosition.x, -span, span );

		if ( m_speed != 0.0f )
		{
			m_context->camera.center = m_viewPosition;
		}

		if ( m_followCar )
		{
			m_context->camera.center.x = b2Body_GetPosition( m_car.m_chassisId ).x;
		}

		float radius = 2.0f;
		if ( ( m_stepCount & 0x1 ) == 0x1 && m_explode )
		{
			m_explosionPosition.x = ( 0.5f + m_cycleIndex ) * m_period - span;

			b2ExplosionDef def = b2DefaultExplosionDef();
			def.position = m_explosionPosition;
			def.radius = radius;
			def.falloff = 0.1f;
			def.impulsePerLength = 1.0f;
			b2World_Explode( m_worldId, &def );

			m_cycleIndex = ( m_cycleIndex + 1 ) % m_cycleCount;
		}

		if ( m_explode )
		{
			DrawWorldCircle( m_draw, m_explosionPosition, radius, b2_colorAzure );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 20.0f );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_car.SetSpeed( -5.0f );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new TileWorld( context );
	}

	Car m_car;
	b2Pos m_viewPosition;
	float m_period;
	int m_cycleCount;
	int m_cycleIndex;
	float m_gridCount;
	float m_gridSize;
	float m_speed;

	b2Pos m_explosionPosition;
	bool m_explode;
	bool m_followCar;
};

static int sampleTileWorld = RegisterSample( "World", "Tiles", TileWorld::Create );

// A pyramid built far from the origin to exercise double precision world positions. The contact
// solver runs in delta space, so the stack settles the same as one at the origin while the body
// transforms hold their full double coordinate. Record it (R) and open it in the Replay Viewer:
// the recorded doubles survive the snapshot and the motion reproduces with no divergence.
class FarPyramid : public Sample
{
public:
	explicit FarPyramid( SampleContext* context )
		: Sample( context )
	{
		// 1e7 is exactly representable in float, so integer box offsets stay exact and a run here
		// can be compared against one at the origin.
		b2Pos origin = b2Pos{ 10.0e6f, 0.0f };

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2OffsetPos( origin, { 0.0f, 12.0f } );
			m_context->camera.zoom = 17.0f;
		}

		float h = 0.25f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = origin;
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		b2Polygon box = b2MakeBox( h, h );
		bodyDef.type = b2_dynamicBody;

		int baseCount = 50;
		for ( int i = 0; i < baseCount; ++i )
		{
			float y = ( 2.0f * i + 1.0f ) * h;
			for ( int j = i; j < baseCount; ++j )
			{
				float x = ( i + 1.0f ) * h + 2.0f * ( j - i ) * h - h * baseCount;
				bodyDef.position = b2OffsetPos( origin, { x, y } );
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreatePolygonShape( bodyId, &shapeDef, &box );
			}
		}
	}

	void Step() override
	{
		Sample::Step();

		b2Pos c = m_context->camera.center;
		DrawScreenTextLine( "view center %.0f km from world origin", 0.001f * c.x );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarPyramid( context );
	}
};

static int sampleFarPyramid = RegisterSample( "World", "Far Pyramid", FarPyramid::Create );

class FarRagdolls : public Sample
{
public:
	explicit FarRagdolls( SampleContext* context )
		: Sample( context )
	{
		b2Pos origin = b2Pos{ 10.0e6f, 0.0f };

		if ( m_context->restart == false )
		{
			m_context->camera.center = b2OffsetPos( origin, { 0.0f, 6.0f } );
			m_context->camera.zoom = 10.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = origin;
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		float w = 6.0f;
		float h = 12.0f;
		b2Segment floor = { { -w, 0.0f }, { w, 0.0f } };
		b2Segment leftWall = { { -w, 0.0f }, { -w, h } };
		b2Segment rightWall = { { w, 0.0f }, { w, h } };
		b2CreateSegmentShape( groundId, &shapeDef, &floor );
		b2CreateSegmentShape( groundId, &shapeDef, &leftWall );
		b2CreateSegmentShape( groundId, &shapeDef, &rightWall );

		float scale = 1.0f;
		int index = 0;
		for ( int i = 0; i < e_rowCount; ++i )
		{
			for ( int j = 0; j < e_columnCount; ++j )
			{
				float x = 2.4f * scale * ( j - 0.5f * ( e_columnCount - 1 ) ) + RandomFloatRange( -0.3f, 0.3f );
				float y = 2.0f + 2.2f * scale * i;
				b2Pos p = b2OffsetPos( origin, { x, y } );
				CreateHuman( m_humans + index, m_worldId, p, scale, 0.05f, 1.0f, 0.5f, index + 1, nullptr, false );
				index += 1;
			}
		}

		m_humanCount = index;
	}

	void Step() override
	{
		Sample::Step();

		b2Pos c = m_context->camera.center;
		DrawScreenTextLine( "%d ragdolls piled %.0f km from the world origin", m_humanCount, 0.001f * c.x );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarRagdolls( context );
	}

	static constexpr int e_columnCount = 5;
	static constexpr int e_rowCount = 5;
	Human m_humans[e_columnCount * e_rowCount] = {};
	int m_humanCount = 0;
};

static int sampleFarRagdolls = RegisterSample( "World", "Far Ragdolls", FarRagdolls::Create );

// The Gear Lift mechanism from the Joints samples, rebuilt 1000 km from the origin. This sample is
// not guarded, so it compiles and renders in both precision modes. The debug draw subtracts the
// camera center before narrowing to float, so the view is correct either way. Build double and the
// gears mesh cleanly out here. Build single and the float grid (printed on screen) is coarser than
// the gear teeth, so the same mechanism grinds and jitters. One scene, two builds, a side by side.
class FarGate : public Sample
{
public:
	explicit FarGate( SampleContext* context )
		: Sample( context )
	{
		// 1e6 is exactly representable in float so the placement adds no error of its own. Any
		// difference between the two builds comes from how positions are stored as the sim runs.
		b2Pos origin = { 1.0e6, 0.0 };
		m_origin = origin;

		if ( m_context->restart == false )
		{
			m_context->camera.center = origin + b2Vec2{ 0.0f, 6.0f };
			m_context->camera.zoom = 7.0f;
			m_context->debugDraw.drawJoints = false;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = origin;
			groundId = b2CreateBody( m_worldId, &bodyDef );

			const char* path =
				"m 63.500002,201.08333 103.187498,0 1e-5,-37.04166 h -2.64584 l 0,34.39583 h -42.33333 v -2.64583 l "
				"-2.64584,-1e-5 v -2.64583 h -2.64583 v -2.64584 h -2.64584 v -2.64583 H 111.125 v -2.64583 h -2.64583 v "
				"-2.64583 h -2.64583 v -2.64584 l -2.64584,1e-5 v -2.64583 l -2.64583,-1e-5 V 174.625 h -2.645834 v -2.64584 l "
				"-2.645833,1e-5 v -2.64584 H 92.60417 v -2.64583 h -2.645834 v -2.64583 l -26.458334,0 0,37.04166";

			b2Vec2 points[128];

			b2Vec2 offset = { -120.0f, -200.0f };
			float scale = 0.2f;
			int count = ParsePath( path, offset, points, 64, scale, false );

			b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
			material.customColor = b2_colorDarkSeaGreen;

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = count;
			chainDef.isLoop = true;
			chainDef.materials = &material;
			chainDef.materialCount = 1;

			b2CreateChain( groundId, &chainDef );
		}

		float gearRadius = 1.0f;
		float toothHalfWidth = 0.09f;
		float toothHalfHeight = 0.06f;
		float toothRadius = 0.03f;
		float linkHalfLength = 0.07f;
		float linkRadius = 0.05f;
		float linkCount = 40;
		float doorHalfHeight = 1.5f;

		b2Pos gearPosition1 = origin + b2Vec2{ -4.25f, 9.75f };
		b2Pos gearPosition2 = gearPosition1 + b2Vec2{ 2.0f, 1.0f };
		b2Pos linkAttachPosition = gearPosition2 + b2Vec2{ gearRadius + 2.0f * toothHalfWidth + toothRadius, 0.0f };
		b2Pos doorPosition = linkAttachPosition - b2Vec2{ 0.0f, 2.0f * linkCount * linkHalfLength + doorHalfHeight };

		{
			b2Pos position = gearPosition1;
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = position;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = b2_colorSaddleBrown;
			b2Circle circle = { b2Vec2_zero, gearRadius };
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			int count = 16;
			float deltaAngle = 2.0f * B2_PI / 16;
			b2Rot dq = b2MakeRot( deltaAngle );
			b2Vec2 center = { gearRadius + toothHalfHeight, 0.0f };
			b2Rot rotation = b2Rot_identity;

			for ( int i = 0; i < count; ++i )
			{
				b2Polygon tooth = b2MakeOffsetRoundedBox( toothHalfWidth, toothHalfHeight, center, rotation, toothRadius );
				shapeDef.material.customColor = b2_colorGray;
				b2CreatePolygonShape( bodyId, &shapeDef, &tooth );

				rotation = b2MulRot( dq, rotation );
				center = b2RotateVector( rotation, { gearRadius + toothHalfHeight, 0.0f } );
			}

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();

			m_motorTorque = 80.0f;
			m_motorSpeed = -0.3f;
			m_enableMotor = true;

			revoluteDef.base.bodyIdA = groundId;
			revoluteDef.base.bodyIdB = bodyId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, position );
			revoluteDef.base.localFrameB.p = b2Vec2_zero;
			revoluteDef.enableMotor = m_enableMotor;
			revoluteDef.maxMotorTorque = m_motorTorque;
			revoluteDef.motorSpeed = m_motorSpeed;
			m_driverId = b2CreateRevoluteJoint( m_worldId, &revoluteDef );
		}

		b2BodyId followerId;

		{
			b2Pos position = gearPosition2;
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = position;

			followerId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = b2_colorSaddleBrown;
			b2Circle circle = { b2Vec2_zero, gearRadius };
			b2CreateCircleShape( followerId, &shapeDef, &circle );

			int count = 16;
			float deltaAngle = 2.0f * B2_PI / 16;
			b2Rot dq = b2MakeRot( deltaAngle );
			b2Vec2 center = { gearRadius + toothHalfWidth, 0.0f };
			b2Rot rotation = b2Rot_identity;

			for ( int i = 0; i < count; ++i )
			{
				b2Polygon tooth = b2MakeOffsetRoundedBox( toothHalfWidth, toothHalfHeight, center, rotation, toothRadius );
				shapeDef.material.customColor = b2_colorGray;
				b2CreatePolygonShape( followerId, &shapeDef, &tooth );

				rotation = b2MulRot( dq, rotation );
				center = b2RotateVector( rotation, { gearRadius + toothHalfWidth, 0.0f } );
			}

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();

			revoluteDef.base.bodyIdA = groundId;
			revoluteDef.base.bodyIdB = followerId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, position );
			revoluteDef.base.localFrameA.q = b2MakeRot( 0.25f * B2_PI );
			revoluteDef.base.localFrameB.p = b2Vec2_zero;
			revoluteDef.enableMotor = true;
			revoluteDef.maxMotorTorque = 0.5f;
			revoluteDef.lowerAngle = -0.3f * B2_PI;
			revoluteDef.upperAngle = 0.8f * B2_PI;
			revoluteDef.enableLimit = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );
		}

		b2BodyId lastLinkId;
		{
			b2Capsule capsule = { { 0.0f, -linkHalfLength }, { 0.0f, linkHalfLength }, linkRadius };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;
			shapeDef.material.customColor = b2_colorLightSteelBlue;

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.maxMotorTorque = 0.05f;
			jointDef.enableMotor = true;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			b2Pos position = linkAttachPosition + b2Vec2{ 0.0f, -linkHalfLength };

			int count = 40;
			b2BodyId prevBodyId = followerId;
			for ( int i = 0; i < count; ++i )
			{
				bodyDef.position = position;

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Pos pivot = { position.x, position.y + linkHalfLength };
				jointDef.base.bodyIdA = prevBodyId;
				jointDef.base.bodyIdB = bodyId;
				jointDef.base.localFrameA.p = b2Body_GetLocalPoint( jointDef.base.bodyIdA, pivot );
				jointDef.base.localFrameB.p = b2Body_GetLocalPoint( jointDef.base.bodyIdB, pivot );
				jointDef.base.drawScale = 0.2f;
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				position.y -= 2.0f * linkHalfLength;
				prevBodyId = bodyId;
			}

			lastLinkId = prevBodyId;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = doorPosition;

			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.15f, doorHalfHeight );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.1f;
			shapeDef.material.customColor = b2_colorDarkCyan;
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			{
				b2Pos pivot = doorPosition + b2Vec2{ 0.0f, doorHalfHeight };
				b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
				revoluteDef.base.bodyIdA = lastLinkId;
				revoluteDef.base.bodyIdB = bodyId;
				revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( lastLinkId, pivot );
				revoluteDef.base.localFrameB.p = { 0.0f, doorHalfHeight };
				revoluteDef.enableMotor = true;
				revoluteDef.maxMotorTorque = 0.05f;
				b2CreateRevoluteJoint( m_worldId, &revoluteDef );
			}

			{
				b2Vec2 localAxis = { 0.0f, 1.0f };
				b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
				jointDef.base.bodyIdA = groundId;
				jointDef.base.bodyIdB = bodyId;
				jointDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, doorPosition );
				jointDef.base.localFrameA.q = b2MakeRotFromUnitVector( localAxis );
				jointDef.base.localFrameB.p = b2Vec2_zero;
				jointDef.base.localFrameB.q = b2MakeRotFromUnitVector( localAxis );
				jointDef.maxMotorForce = 0.2f;
				jointDef.enableMotor = true;
				jointDef.base.collideConnected = true;
				b2CreatePrismaticJoint( m_worldId, &jointDef );
			}
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.rollingResistance = 0.3f;

			b2HexColor colors[5] = {
				b2_colorGray, b2_colorGainsboro, b2_colorLightGray, b2_colorLightSlateGray, b2_colorDarkGray,
			};

			float y = 4.25f;
			int xCount = 10, yCount = 20;
			for ( int i = 0; i < yCount; ++i )
			{
				float x = -3.15f;
				for ( int j = 0; j < xCount; ++j )
				{
					bodyDef.position = origin + b2Vec2{ x, y };
					b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
					b2Polygon poly = RandomPolygon( 0.1f );
					poly.radius = RandomFloatRange( 0.01f, 0.02f );

					int colorIndex = RandomIntRange( 0, 4 );
					shapeDef.material.customColor = colors[colorIndex];

					b2CreatePolygonShape( bodyId, &shapeDef, &poly );
					x += 0.2f;
				}

				y += 0.2f;
			}
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2RevoluteJoint_EnableMotor( m_driverId, m_enableMotor );
			b2Joint_WakeBodies( m_driverId );
		}

		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 100.0f, "%.0f" ) )
		{
			b2RevoluteJoint_SetMaxMotorTorque( m_driverId, m_motorTorque );
			b2Joint_WakeBodies( m_driverId );
		}

		if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -0.3f, 0.3f, "%.2f" ) )
		{
			b2RevoluteJoint_SetMotorSpeed( m_driverId, m_motorSpeed );
			b2Joint_WakeBodies( m_driverId );
		}

		ImGui::PopItemWidth();

		return true;
	}

	void Step() override
	{
		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) )
		{
			m_motorSpeed = b2MaxFloat( -0.3f, m_motorSpeed - 0.01f );
			b2RevoluteJoint_SetMotorSpeed( m_driverId, m_motorSpeed );
			b2Joint_WakeBodies( m_driverId );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_D ) )
		{
			m_motorSpeed = b2MinFloat( 0.3f, m_motorSpeed + 0.01f );
			b2RevoluteJoint_SetMotorSpeed( m_driverId, m_motorSpeed );
			b2Joint_WakeBodies( m_driverId );
		}

		Sample::Step();

		float fx = (float)m_origin.x;
		float gridStep = nextafterf( fx, FLT_MAX ) - fx;

#ifdef BOX2D_DOUBLE_PRECISION
		const char* mode = "double precision";
#else
		const char* mode = "single precision";
#endif
		DrawScreenTextLine( "%s build, gate %.0f km from the origin", mode, m_origin.x / 1000.0 );
		DrawScreenTextLine( "float grid step here = %g m, gear teeth are about 0.12 m", gridStep );
	}

	static Sample* Create( SampleContext* context )
	{
		return new FarGate( context );
	}

	b2Pos m_origin;
	b2JointId m_driverId;
	float m_motorTorque;
	float m_motorSpeed;
	bool m_enableMotor;
};

static int sampleFarGate = RegisterSample( "World", "Far Gate", FarGate::Create );
