// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "utils.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <stdio.h>
#include <vector>

static void ComputeEnergy( b2WorldId worldId, const b2BodyId* bodyIds, int count, float* linear, float* angular,
						   float* potential )
{
	b2Vec2 gravity = b2World_GetGravity( worldId );

	float linearSum = 0.0f;
	float angularSum = 0.0f;
	float potentialSum = 0.0f;

	for ( int i = 0; i < count; ++i )
	{
		b2MassData massData = b2Body_GetMassData( bodyIds[i] );
		b2Vec2 v = b2Body_GetLinearVelocity( bodyIds[i] );
		float w = b2Body_GetAngularVelocity( bodyIds[i] );

		linearSum += 0.5f * massData.mass * b2Dot( v, v );
		angularSum += 0.5f * massData.rotationalInertia * w * w;
		potentialSum -= massData.mass * b2Dot( gravity, b2ToVec2( b2Body_GetWorldCenter( bodyIds[i] ) ) );
	}

	*linear = linearSum;
	*angular = angularSum;
	*potential = potentialSum;
}

// Restitution is approximate since Box2D uses speculative collision
class VaryingRestitution : public Sample
{
public:
	enum ShapeType
	{
		e_circleShape = 0,
		e_boxShape
	};

	explicit VaryingRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 4.0f, 17.0f };
			m_context->camera.zoom = 27.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			float h = 1.0f * m_count;
			b2Segment segment = { { -h, 0.0f }, { h, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_shapeType = e_circleShape;

		CreateBodies();
	}

	void CreateBodies()
	{
		for ( int i = 0; i < m_count; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2Circle circle = {};
		circle.radius = 0.5f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 0.0f;
		shapeDef.material.friction = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		float dr = 1.0f / ( m_count > 1 ? m_count - 1 : 1 );
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			char buffer[32];
			snprintf( buffer, 32, "%.2f", shapeDef.material.restitution );
			bodyDef.name = buffer;
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

			shapeDef.material.restitution += dr;
			x += dx;
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		bool changed = false;
		const char* shapeTypes[] = { "Circle", "Box" };

		int shapeType = int( m_shapeType );
		changed = changed || ImGui::Combo( "Shape", &shapeType, shapeTypes, IM_ARRAYSIZE( shapeTypes ) );
		m_shapeType = ShapeType( shapeType );

		ImGui::PopItemWidth();

		changed = changed || ImGui::Button( "Reset" );

		if ( changed )
		{
			CreateBodies();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		float h = 1.0f * m_count;
		DrawLine( m_draw, { -h, 40.5f }, { h, 40.5f }, b2_colorRed );
	}

	static Sample* Create( SampleContext* context )
	{
		return new VaryingRestitution( context );
	}

	static constexpr int m_count = 40;

	b2BodyId m_bodyIds[m_count] = {};
	ShapeType m_shapeType;
};

static int sampleVaryingRestitution = RegisterSample( "Restitution", "Varying", VaryingRestitution::Create );

// Tests how a single bouncing box behaves.
class SingleBoxRestitution : public Sample
{
public:
	explicit SingleBoxRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 8.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -4.0f, 0.0f }, { 4.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_height = 5.0f;
		m_maxY = 5.0f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, m_height };
		bodyDef.safetyFactor = 0.01f;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );
		b2CreatePolygonShape( m_bodyId, &shapeDef, &box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_bodyId, 1, &linear, &angular, &potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	void Step() override
	{
		Sample::Step();

		DrawLine( m_draw, { -4.0f, m_height + 0.5f }, { 4.0f, m_height + 0.5f }, b2_colorRed );

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );

		b2Pos p = b2Body_GetPosition( m_bodyId );
		if ( events.hitCount == 1 )
		{
			m_maxY = (float)p.y;
		}
		else
		{
			m_maxY = b2MaxFloat( m_maxY, (float)p.y );
		}

		DrawScreenTextLine( "maxY = %.2f", m_maxY );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_bodyId, 1, &linear, &angular, &potential );
		float total = linear + angular + potential;
		m_peakEnergy = b2MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SingleBoxRestitution( context );
	}

	b2BodyId m_bodyId;
	float m_maxY;
	float m_startEnergy = 0.0f;
	float m_peakEnergy = 0.0f;
	float m_height;
};

static int sampleSingleBoxRestitution = RegisterSample( "Restitution", "Single Box Restitution", SingleBoxRestitution::Create );

class SingleCircleRestitution : public Sample
{
public:
	explicit SingleCircleRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 8.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -4.0f, 0.0f }, { 4.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2Circle circle = { b2Vec2_zero, 0.5f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.enableHitEvents = true;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 10.0f };
		bodyDef.safetyFactor = 0.01f;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );
		b2CreateCircleShape( m_bodyId, &shapeDef, &circle );

		m_maxY = bodyDef.position.y;

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_bodyId, 1, &linear, &angular, &potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	void Step() override
	{
		Sample::Step();

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );

		b2Pos p = b2Body_GetPosition( m_bodyId );
		if ( events.hitCount == 1 )
		{
			m_maxY = (float)p.y;
		}
		else
		{
			m_maxY = b2MaxFloat( m_maxY, (float)p.y );
		}

		DrawLine( m_draw, { -4.0f, 10.5f }, { 4.0f, 10.5f }, b2_colorRed );

		DrawScreenTextLine( "maxY = %.2f", m_maxY );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_bodyId, 1, &linear, &angular, &potential );
		float total = linear + angular + potential;
		m_peakEnergy = b2MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SingleCircleRestitution( context );
	}

	b2BodyId m_bodyId;
	float m_maxY;
	float m_startEnergy = 0.0f;
	float m_peakEnergy = 0.0f;
};

static int sampleSingleCircleRestitution =
	RegisterSample( "Restitution", "Single Circle Restitution", SingleCircleRestitution::Create );

// Similar to MeasureSupportedBounce unit test.
class CircleStackRestitution : public Sample
{
public:
	static constexpr float m_impactSpeed = 5.0f;
	static constexpr int m_maxStackCount = 3;

	explicit CircleStackRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 2.5f };
			m_context->camera.zoom = 6.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;
		b2Polygon box = b2MakeBox( 40.0f, 1.0f );
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		CreateScene();
	}

	b2BodyId CreateBall( float y, float velocityY, float restitution, bool hitEvents )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, y };
		bodyDef.linearVelocity = { 0.0f, velocityY };
		bodyDef.enableSleep = false;
		b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = restitution;
		shapeDef.enableHitEvents = hitEvents;
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
		b2CreateCircleShape( bodyId, &shapeDef, &circle );

		return bodyId;
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
		}
		m_bodyCount = 0;

		if ( m_gravity )
		{
			b2World_SetGravity( m_worldId, { 0.0f, -10.0f } );
		}
		else
		{
			b2World_SetGravity( m_worldId, b2Vec2_zero );
		}

		for ( int i = 0; i < m_stackCount; ++i )
		{
			m_bodyIds[m_bodyCount] = CreateBall( 0.5f + 1.0f * i, 0.0f, 0.0f, false );
			m_bodyCount += 1;
		}

		// Start half a step of travel above contact so the impact lands mid step, matching the test
		m_startHeight = 0.5f + 1.0f * m_stackCount + 0.5f * m_impactSpeed * ( 1.0f / 60.0f );

		m_impactorId = CreateBall( m_startHeight, -m_impactSpeed, m_restitution, true );
		m_bodyIds[m_bodyCount] = m_impactorId;
		m_bodyCount += 1;

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, m_bodyCount, &linear, &angular, &potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;

		m_coefficient = 0.0f;
		m_latched = false;
		m_hit = false;
	}

	bool DrawControls() override
	{
		bool rebuild = false;

		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		if ( ImGui::SliderInt( "Stack", &m_stackCount, 0, m_maxStackCount ) )
		{
			rebuild = true;
		}

		ImGui::PopItemWidth();

		if ( ImGui::Checkbox( "Gravity", &m_gravity ) )
		{
			rebuild = true;
		}

		if ( ImGui::Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		if ( events.hitCount > 0 )
		{
			m_hit = true;
		}

		float vy = b2Body_GetLinearVelocity( m_impactorId ).y;

		// The rebound only means something once the impact has happened and the impactor is leaving
		if ( m_latched == false && m_hit && vy > 0.0f )
		{
			m_coefficient = vy / m_impactSpeed;
			m_latched = true;
		}

		float supportSpeed = 0.0f;
		for ( int i = 0; i < m_stackCount; ++i )
		{
			supportSpeed = b2MaxFloat( supportSpeed, b2Length( b2Body_GetLinearVelocity( m_bodyIds[i] ) ) );
		}

		DrawLine( m_draw, { -2.0f, m_startHeight }, { 2.0f, m_startHeight }, b2_colorRed );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, m_bodyCount, &linear, &angular, &potential );
		float total = linear + angular + potential;
		m_peakEnergy = b2MaxFloat( m_peakEnergy, total );

		DrawScreenTextLine( "impactor vy = %.3f m/s", vy );

		DrawScreenTextLine( "coefficient = %.4f live (target %.2f)", vy / m_impactSpeed, m_restitution );

		if ( m_latched )
		{
			DrawScreenTextLine( "coefficient = %.4f at first rebound", m_coefficient );
		}

		DrawScreenTextLine( "support speed = %.4f m/s", supportSpeed );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new CircleStackRestitution( context );
	}

	b2BodyId m_bodyIds[m_maxStackCount + 1] = {};
	b2BodyId m_impactorId = b2_nullBodyId;
	int m_bodyCount = 0;
	int m_stackCount = 2;
	float m_restitution = 1.0f;
	float m_startHeight = 0.0f;
	float m_coefficient = 0.0f;
	float m_startEnergy = 0.0f;
	float m_peakEnergy = 0.0f;
	bool m_gravity = false;
	bool m_latched = false;
	bool m_hit = false;
};

static int sampleCircleStackRestitution = RegisterSample( "Restitution", "Circle Stack Restitution", CircleStackRestitution::Create );

// A bouncy ball landing on a resting body. Three balls fall together from the same height onto bare
// ground, one crate and two crates. Without restitution propagation the restitution stage only
// visits the ball's own contact, so the rebound is shared with the crate, which is driven into the
// ground and pushed back the next step. With propagation the ground contact under the crate can react
// in the restitution sover.
class RestitutionPropagation : public Sample
{
public:
	static constexpr int m_columnCount = 3;
	static constexpr float m_dropHeight = 5.0f;

	explicit RestitutionPropagation( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 4.0f };
			m_context->camera.zoom = 10.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.6f;
		b2Polygon box = b2MakeBox( 40.0f, 1.0f );
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		CreateScene();
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
		}
		m_bodyCount = 0;
		m_crateCount = 0;
		m_kick = 0.0f;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );
		b2Circle circle = { { 0.0f, 0.0f }, 0.5f };

		for ( int column = 0; column < m_columnCount; ++column )
		{
			float x = 4.0f * ( column - 1 );

			for ( int k = 0; k < column; ++k )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { x, 0.5f + 1.0f * k };
				bodyDef.enableSleep = false;
				b2BodyId crateId = b2CreateBody( m_worldId, &bodyDef );

				b2ShapeDef shapeDef = b2DefaultShapeDef();
				b2CreatePolygonShape( crateId, &shapeDef, &box );

				m_bodyIds[m_bodyCount++] = crateId;
				m_crateIds[m_crateCount++] = crateId;
			}

			m_restHeight[column] = 0.5f + 1.0f * column;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { x, m_restHeight[column] + m_dropHeight };
			bodyDef.enableSleep = false;
			b2BodyId ballId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.restitution = m_restitution;
			b2CreateCircleShape( ballId, &shapeDef, &circle );

			m_bodyIds[m_bodyCount++] = ballId;
			m_ballIds[column] = ballId;
			m_apex[column] = 0.0f;
			m_rebounding[column] = false;
			m_done[column] = false;
		}
	}

	bool DrawControls() override
	{
		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );
		bool rebuild = ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" );
		ImGui::PopItemWidth();

		if ( ImGui::Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		static const char* labels[m_columnCount] = { "ground", "one crate", "two crates" };

		for ( int column = 0; column < m_columnCount; ++column )
		{
			b2Vec2 v = b2Body_GetLinearVelocity( m_ballIds[column] );
			float height = float( b2Body_GetPosition( m_ballIds[column] ).y ) - m_restHeight[column];

			// The first rebound apex: the ball starts at rest and falls, so upward velocity means it bounced
			if ( m_rebounding[column] == false && v.y > 0.0f )
			{
				m_rebounding[column] = true;
			}

			if ( m_rebounding[column] && m_done[column] == false )
			{
				m_apex[column] = b2MaxFloat( m_apex[column], height );
				if ( v.y < 0.0f )
				{
					m_done[column] = true;
				}
			}

			float x = 4.0f * ( column - 1 );
			DrawLine( m_draw, { x - 1.0f, m_restHeight[column] + m_dropHeight },
					  { x + 1.0f, m_restHeight[column] + m_dropHeight }, b2_colorRed );
			if ( m_apex[column] > 0.0f )
			{
				DrawLine( m_draw, { x - 1.0f, m_restHeight[column] + m_apex[column] },
						  { x + 1.0f, m_restHeight[column] + m_apex[column] }, b2_colorGreen );
			}

			DrawScreenTextLine( "%s: apex %.2f m (%.0f%% of drop)", labels[column], m_apex[column],
								100.0f * m_apex[column] / m_dropHeight );
		}

		for ( int i = 0; i < m_crateCount; ++i )
		{
			m_kick = b2MaxFloat( m_kick, b2Length( b2Body_GetLinearVelocity( m_crateIds[i] ) ) );
		}

		DrawScreenTextLine( "peak crate speed %.2f m/s", m_kick );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RestitutionPropagation( context );
	}

	b2BodyId m_bodyIds[2 * m_columnCount] = {};
	b2BodyId m_ballIds[m_columnCount] = {};
	b2BodyId m_crateIds[m_columnCount] = {};
	float m_restHeight[m_columnCount] = {};
	float m_apex[m_columnCount] = {};
	bool m_rebounding[m_columnCount] = {};
	bool m_done[m_columnCount] = {};
	int m_bodyCount = 0;
	int m_crateCount = 0;
	float m_restitution = 0.9f;
	float m_kick = 0.0f;
};

static int sampleRestitutionPropagation = RegisterSample( "Restitution", "Restitution Propagation", RestitutionPropagation::Create );

// Similar to MeasureFlatBounce and SpinTest unit tests.
class BoxRestitution : public Sample
{
public:
	static constexpr float m_impactSpeed = 5.0f;

	explicit BoxRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 8.0f };
			m_context->camera.zoom = 10.0f;
		}

		// Gravity would bias the measured coefficient
		b2World_SetGravity( m_worldId, b2Vec2_zero );

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = { 0.0f, -1.0f };
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = 0.0f;
		b2Polygon box = b2MakeBox( 40.0f, 1.0f );
		b2CreatePolygonShape( groundId, &shapeDef, &box );

		CreateScene();
	}

	void CreateScene()
	{
		if ( B2_IS_NON_NULL( m_boxId ) )
		{
			b2DestroyBody( m_boxId );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = { 0.0f, 0.25f + 0.5f * m_impactSpeed * ( 1.0f / 60.0f ) };
		bodyDef.linearVelocity = { 0.0f, -m_impactSpeed };
		bodyDef.angularVelocity = m_spin;
		bodyDef.enableSleep = false;
		m_boxId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.friction = 0.0f;
		shapeDef.material.restitution = m_restitution;
		shapeDef.enableHitEvents = true;
		b2Polygon box = b2MakeBox( 1.0f, 0.25f );
		b2CreatePolygonShape( m_boxId, &shapeDef, &box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_boxId, 1, &linear, &angular, &potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;

		m_coefficient = 0.0f;
		m_latched = false;
		m_hit = false;
	}

	bool DrawControls() override
	{
		bool rebuild = false;

		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		if ( ImGui::SliderFloat( "Spin", &m_spin, 0.0f, 2.0f, "%.2f" ) )
		{
			rebuild = true;
		}

		ImGui::PopItemWidth();

		if ( ImGui::Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		b2ContactEvents events = b2World_GetContactEvents( m_worldId );
		if ( events.hitCount > 0 )
		{
			m_hit = true;
		}

		float vy = b2Body_GetLinearVelocity( m_boxId ).y;
		float w = b2Body_GetAngularVelocity( m_boxId );

		if ( m_latched == false && m_hit && vy > 0.0f )
		{
			m_coefficient = vy / m_impactSpeed;
			m_latched = true;
		}

		DrawPoint( m_draw, b2Body_GetWorldPoint( m_boxId, { -1.0f, -0.25f } ), 8.0f, b2_colorYellow );
		DrawPoint( m_draw, b2Body_GetWorldPoint( m_boxId, { 1.0f, -0.25f } ), 8.0f, b2_colorYellow );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, &m_boxId, 1, &linear, &angular, &potential );
		float total = linear + angular + potential;
		m_peakEnergy = b2MaxFloat( m_peakEnergy, total );

		DrawScreenTextLine( "vy = %.3f m/s", vy );

		DrawScreenTextLine( "coefficient = %.4f live (target %.2f)", vy / m_impactSpeed, m_restitution );

		if ( m_latched )
		{
			DrawScreenTextLine( "coefficient = %.4f at first rebound", m_coefficient );
		}

		DrawScreenTextLine( "spin in = %.2f, spin out = %.4f (ideal %.2f at e = 1)", m_spin, w, -m_spin );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new BoxRestitution( context );
	}

	b2BodyId m_boxId = b2_nullBodyId;
	float m_restitution = 0.9f;
	float m_spin = 0.0f;
	float m_coefficient = 0.0f;
	float m_startEnergy = 0.0f;
	float m_peakEnergy = 0.0f;
	bool m_latched = false;
	bool m_hit = false;
};

static int sampleBoxRestitution = RegisterSample( "Restitution", "Box Restitution", BoxRestitution::Create );

class RotatedBoxRestitution : public Sample
{
public:
	static constexpr int m_count = 12;
	static constexpr float m_dropHeight = 10.0f;

	explicit RotatedBoxRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 4.0f };
			m_context->camera.zoom = 14.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		float h = 2.0f * m_count + 1.0f;
		b2Segment segment = { { -h, 0.0f }, { h, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		CreateScene();
	}

	void CreateScene()
	{
		for ( int i = 0; i < m_count; ++i )
		{
			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.safetyFactor = m_safetyFactor;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.friction = m_friction;
		shapeDef.material.restitution = m_restitution;

		// Boxes in a negative group never collide with each other.
		shapeDef.filter.groupIndex = -1;

		b2Polygon box = b2MakeBox( 0.5f, 0.5f );

		// A square repeats every quarter turn
		float dq = 0.5f * B2_PI / m_count;
		float x = -1.0f * ( m_count - 1 );

		for ( int i = 0; i < m_count; ++i )
		{
			bodyDef.position = { x, m_dropHeight };
			bodyDef.rotation = b2MakeRot( i * dq );
			m_bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[i], &shapeDef, &box );

			float linear = 0.0f;
			float angular = 0.0f;
			float potential = 0.0f;
			ComputeEnergy( m_worldId, &m_bodyIds[i], 1, &linear, &angular, &potential );
			m_startEnergy[i] = linear + angular + potential;

			x += 2.0f;
		}
	}

	bool DrawControls() override
	{
		bool rebuild = false;

		ImGui::PushItemWidth( 6.0f * ImGui::GetFontSize() );

		if ( ImGui::SliderFloat( "Restitution", &m_restitution, 0.0f, 1.0f, "%.1f" ) )
		{
			rebuild = true;
		}

		if ( ImGui::SliderFloat( "Friction", &m_friction, 0.0f, 1.0f, "%.1f" ) )
		{
			rebuild = true;
		}

		if ( ImGui::SliderFloat( "Safety Factor", &m_safetyFactor, 0.01f, 0.5f, "%.2f" ) )
		{
			rebuild = true;
		}

		ImGui::PopItemWidth();

		if ( ImGui::Button( "Reset" ) )
		{
			rebuild = true;
		}

		if ( rebuild )
		{
			CreateScene();
		}

		return true;
	}

	void Step() override
	{
		Sample::Step();

		float h = 1.0f * m_count;
		DrawLine( m_draw, { -h, m_dropHeight + 0.5f }, { h, m_dropHeight + 0.5f }, b2_colorRed );

		float minPercent = 0.0f;
		float maxPercent = 0.0f;
		float x = -1.0f * ( m_count - 1 );

		for ( int i = 0; i < m_count; ++i )
		{
			float linear = 0.0f;
			float angular = 0.0f;
			float potential = 0.0f;
			ComputeEnergy( m_worldId, &m_bodyIds[i], 1, &linear, &angular, &potential );
			float total = linear + angular + potential;

			float percent = m_startEnergy[i] != 0.0f ? 100.0f * total / m_startEnergy[i] : 0.0f;
			minPercent = i == 0 ? percent : b2MinFloat( minPercent, percent );
			maxPercent = i == 0 ? percent : b2MaxFloat( maxPercent, percent );

			DrawString( m_draw, m_camera, b2ToPos( { x - 0.5f, m_dropHeight + 1.5f } ), b2_colorWhite, "%.1f%%", percent );
			x += 2.0f;
		}

		DrawScreenTextLine( "energy: min %.2f%%, max %.2f%% of start", minPercent, maxPercent );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RotatedBoxRestitution( context );
	}

	b2BodyId m_bodyIds[m_count] = {};
	float m_startEnergy[m_count] = {};
	float m_restitution = 0.9f;
	float m_friction = 0.0f;
	float m_safetyFactor = 0.5f;
};

static int sampleRotatedBoxRestitution = RegisterSample( "Restitution", "Rotated Box Restitution", RotatedBoxRestitution::Create );

// Based on https://github.com/erincatto/box2d/discussions/957
class TwoBoxRestitution : public Sample
{
public:
	explicit TwoBoxRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 3.0f };
			m_context->camera.zoom = 5.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -10.0f, 0.0f }, { 10.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2Polygon box = b2MakeSquare( 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.material.restitution = 0.9f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		bodyDef.position.y = 0.5f;
		m_bodyIds[0] = b2CreateBody( m_worldId, &bodyDef );
		b2CreatePolygonShape( m_bodyIds[0], &shapeDef, &box );

		bodyDef.position.x += 0.1f;
		bodyDef.position.y = 6.0f;
		m_bodyIds[1] = b2CreateBody( m_worldId, &bodyDef );
		b2CreatePolygonShape( m_bodyIds[1], &shapeDef, &box );

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, 2, &linear, &angular, &potential );
		m_startEnergy = linear + angular + potential;
		m_peakEnergy = m_startEnergy;
	}

	void Step() override
	{
		Sample::Step();

		float linear = 0.0f;
		float angular = 0.0f;
		float potential = 0.0f;
		ComputeEnergy( m_worldId, m_bodyIds, 2, &linear, &angular, &potential );
		float total = linear + angular + potential;
		m_peakEnergy = b2MaxFloat( m_peakEnergy, total );

		float scale = m_startEnergy != 0.0f ? 100.0f / m_startEnergy : 0.0f;
		DrawScreenTextLine( "kinetic   = %.3f J linear + %.3f J angular", linear, angular );
		DrawScreenTextLine( "potential = %.3f J", potential );
		DrawScreenTextLine( "total     = %.3f J (%.2f%% of start, peak %.2f%%)", total, scale * total, scale * m_peakEnergy );
	}

	static Sample* Create( SampleContext* context )
	{
		return new TwoBoxRestitution( context );
	}

	b2BodyId m_bodyIds[2] = {};
	float m_startEnergy = 0.0f;
	float m_peakEnergy = 0.0f;
};

static int sampleTwoBoxRestitution = RegisterSample( "Restitution", "Two Box Restitution", TwoBoxRestitution::Create );

class CircleRestitution : public Sample
{
public:
	explicit CircleRestitution( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 5.0f };
			m_context->camera.zoom = 10.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			float h = 2.0f * m_count;
			b2Segment segment = { { -h, 0.0f }, { h, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2Circle circle = { b2Vec2_zero, 0.5f };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 1.0f;
		shapeDef.material.restitution = 0.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;

		float dr = 1.0f / ( m_count > 1 ? m_count - 1 : 1 );
		float x = -1.0f * ( m_count - 1 );
		float dx = 2.0f;

		for ( int i = 0; i < m_count; ++i )
		{
			char buffer[32];
			snprintf( buffer, 32, "%.2f", shapeDef.material.restitution );

			bodyDef.position = { x, 1.0f };
			bodyDef.name = buffer;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			bodyDef.position = { x, 4.0f };
			bodyDef.name = buffer;
			bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			shapeDef.material.restitution += dr;
			x += dx;
		}
	}
	static Sample* Create( SampleContext* context )
	{
		return new CircleRestitution( context );
	}

	static constexpr int m_count = 10;
};

static int sampleCircleRestitution = RegisterSample( "Restitution", "Circle Restitution", CircleRestitution::Create );
