// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "car.h"
#include "donut.h"
#include "doohickey.h"
#include "draw.h"
#include "human.h"
#include "random.h"
#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

// Test the distance joint and all options
class DistanceJoint : public Sample
{
public:
	enum
	{
		e_maxCount = 10
	};

	explicit DistanceJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 12.0f };
			m_context->camera.m_zoom = 25.0f * 0.35f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			m_groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_count = 0;
		m_hertz = 2.0f;
		m_dampingRatio = 0.5f;
		m_length = 1.0f;
		m_minLength = m_length;
		m_maxLength = m_length;
		m_enableSpring = false;
		m_enableLimit = false;

		for ( int i = 0; i < e_maxCount; ++i )
		{
			m_bodyIds[i] = b2_nullBodyId;
			m_jointIds[i] = b2_nullJointId;
		}

		CreateScene( 1 );
	}

	void CreateScene( int newCount )
	{
		// Must destroy joints before bodies
		for ( int i = 0; i < m_count; ++i )
		{
			b2DestroyJoint( m_jointIds[i] );
			m_jointIds[i] = b2_nullJointId;
		}

		for ( int i = 0; i < m_count; ++i )
		{
			b2DestroyBody( m_bodyIds[i] );
			m_bodyIds[i] = b2_nullBodyId;
		}

		m_count = newCount;

		float radius = 0.25f;
		b2Circle circle = { { 0.0f, 0.0f }, radius };

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 20.0f;

		float yOffset = 20.0f;

		b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
		jointDef.hertz = m_hertz;
		jointDef.dampingRatio = m_dampingRatio;
		jointDef.length = m_length;
		jointDef.minLength = m_minLength;
		jointDef.maxLength = m_maxLength;
		jointDef.enableSpring = m_enableSpring;
		jointDef.enableLimit = m_enableLimit;

		b2BodyId prevBodyId = m_groundId;
		for ( int i = 0; i < m_count; ++i )
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.angularDamping = 0.1f;
			bodyDef.position = { m_length * ( i + 1.0f ), yOffset };
			m_bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( m_bodyIds[i], &shapeDef, &circle );

			b2Vec2 pivotA = { m_length * i, yOffset };
			b2Vec2 pivotB = { m_length * ( i + 1.0f ), yOffset };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = m_bodyIds[i];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivotA );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivotB );
			m_jointIds[i] = b2CreateDistanceJoint( m_worldId, &jointDef );

			prevBodyId = m_bodyIds[i];
		}
	}

	void UpdateGui() override
	{
		float height = 240.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

		ImGui::Begin( "Distance Joint", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		if ( ImGui::SliderFloat( "Length", &m_length, 0.1f, 4.0f, "%3.1f" ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b2DistanceJoint_SetLength( m_jointIds[i], m_length );
				b2Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b2DistanceJoint_EnableSpring( m_jointIds[i], m_enableSpring );
				b2Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 15.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b2DistanceJoint_SetSpringHertz( m_jointIds[i], m_hertz );
					b2Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b2DistanceJoint_SetSpringDampingRatio( m_jointIds[i], m_dampingRatio );
					b2Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			for ( int i = 0; i < m_count; ++i )
			{
				b2DistanceJoint_EnableLimit( m_jointIds[i], m_enableLimit );
				b2Joint_WakeBodies( m_jointIds[i] );
			}
		}

		if ( m_enableLimit )
		{
			if ( ImGui::SliderFloat( "Min Length", &m_minLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b2DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b2Joint_WakeBodies( m_jointIds[i] );
				}
			}

			if ( ImGui::SliderFloat( "Max Length", &m_maxLength, 0.1f, 4.0f, "%3.1f" ) )
			{
				for ( int i = 0; i < m_count; ++i )
				{
					b2DistanceJoint_SetLengthRange( m_jointIds[i], m_minLength, m_maxLength );
					b2Joint_WakeBodies( m_jointIds[i] );
				}
			}
		}

		int count = m_count;
		if ( ImGui::SliderInt( "Count", &count, 1, e_maxCount ) )
		{
			CreateScene( count );
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new DistanceJoint( context );
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyIds[e_maxCount];
	b2JointId m_jointIds[e_maxCount];
	int m_count;
	float m_hertz;
	float m_dampingRatio;
	float m_length;
	float m_minLength;
	float m_maxLength;
	bool m_enableSpring;
	bool m_enableLimit;
};

static int sampleDistanceJoint = RegisterSample( "Joints", "Distance Joint", DistanceJoint::Create );

/// This test shows how to use a motor joint. A motor joint
/// can be used to animate a dynamic body. With finite motor forces
/// the body can be blocked by collision with other bodies.
/// By setting the correction factor to zero, the motor joint acts
/// like top-down dry friction.
class MotorJoint : public Sample
{
public:
	explicit MotorJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 7.0f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Define motorized body
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 8.0f };
			m_bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 2.0f, 0.5f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( m_bodyId, &shapeDef, &box );

			m_maxForce = 500.0f;
			m_maxTorque = 500.0f;
			m_correctionFactor = 0.3f;

			b2MotorJointDef jointDef = b2DefaultMotorJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyId;
			jointDef.maxForce = m_maxForce;
			jointDef.maxTorque = m_maxTorque;
			jointDef.correctionFactor = m_correctionFactor;

			m_jointId = b2CreateMotorJoint( m_worldId, &jointDef );
		}

		m_go = true;
		m_time = 0.0f;
	}

	void UpdateGui() override
	{
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Motor Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Go", &m_go ) )
		{
		}

		if ( ImGui::SliderFloat( "Max Force", &m_maxForce, 0.0f, 10000.0f, "%.0f" ) )
		{
			b2MotorJoint_SetMaxForce( m_jointId, m_maxForce );
		}

		if ( ImGui::SliderFloat( "Max Torque", &m_maxTorque, 0.0f, 10000.0f, "%.0f" ) )
		{
			b2MotorJoint_SetMaxTorque( m_jointId, m_maxTorque );
		}

		if ( ImGui::SliderFloat( "Correction", &m_correctionFactor, 0.0f, 1.0f, "%.1f" ) )
		{
			b2MotorJoint_SetCorrectionFactor( m_jointId, m_correctionFactor );
		}

		if ( ImGui::Button( "Apply Impulse" ) )
		{
			b2Body_ApplyLinearImpulseToCenter( m_bodyId, { 100.0f, 0.0f }, true );
		}

		ImGui::End();
	}

	void Step() override
	{
		if ( m_go && m_context->hertz > 0.0f )
		{
			m_time += 1.0f / m_context->hertz;
		}

		b2Vec2 linearOffset;
		linearOffset.x = 6.0f * sinf( 2.0f * m_time );
		linearOffset.y = 8.0f + 4.0f * sinf( 1.0f * m_time );

		float angularOffset = 2.0f * m_time;

		b2MotorJoint_SetLinearOffset( m_jointId, linearOffset );
		b2MotorJoint_SetAngularOffset( m_jointId, angularOffset );

		b2Transform transform = { linearOffset, b2MakeRot( angularOffset ) };
		m_context->draw.DrawTransform( transform );

		Sample::Step();

		b2Vec2 force = b2Joint_GetConstraintForce( m_jointId );
		float torque = b2Joint_GetConstraintTorque( m_jointId );

		DrawTextLine( "force = {%3.f, %3.f}, torque = %3.f", force.x, force.y, torque );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MotorJoint( context );
	}

	b2BodyId m_bodyId;
	b2JointId m_jointId;
	float m_time;
	float m_maxForce;
	float m_maxTorque;
	float m_correctionFactor;
	bool m_go;
};

static int sampleMotorJoint = RegisterSample( "Joints", "Motor Joint", MotorJoint::Create );

// This sample shows how to use a filter joint to prevent collision between two bodies.
// This is more specific than shape filters. It also shows that sleeping is coupled by the filter joint.
class FilterJoint : public Sample
{
public:
	explicit FilterJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 7.0f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
		}

		{
			b2BodyId groundId;
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -4.0f, 2.0f };
			b2BodyId bodyId1 = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeSquare( 2.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( bodyId1, &shapeDef, &box );

			bodyDef.position = { 4.0f, 2.0f };
			b2BodyId bodyId2 = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId2, &shapeDef, &box );

			b2FilterJointDef jointDef = b2DefaultFilterJointDef();
			jointDef.bodyIdA = bodyId1;
			jointDef.bodyIdB = bodyId2;

			b2CreateFilterJoint( m_worldId, &jointDef );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new FilterJoint( context );
	}
};

static int sampleFilterJoint = RegisterSample( "Joints", "Filter Joint", FilterJoint::Create );

class RevoluteJoint : public Sample
{
public:
	explicit RevoluteJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 15.5f };
			m_context->camera.m_zoom = 25.0f * 0.7f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 40.0f, 1.0f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_enableSpring = false;
		m_enableLimit = true;
		m_enableMotor = false;
		m_hertz = 2.0f;
		m_dampingRatio = 0.5f;
		m_targetDegrees = 45.0f;
		m_motorSpeed = 1.0f;
		m_motorTorque = 1000.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -10.0f, 20.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;

			b2Capsule capsule = { { 0.0f, -1.0f }, { 0.0f, 6.0f }, 0.5f };
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			b2Vec2 pivot = { -10.0f, 20.5f };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.targetAngle = B2_PI * m_targetDegrees / 180.0f;
			jointDef.enableSpring = m_enableSpring;
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = m_motorTorque;
			jointDef.enableMotor = m_enableMotor;
			jointDef.referenceAngle = 0.5f * B2_PI;
			jointDef.lowerAngle = -0.5f * B2_PI;
			jointDef.upperAngle = 0.75f * B2_PI;
			jointDef.enableLimit = m_enableLimit;

			m_jointId1 = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		{
			b2Circle circle = {};
			circle.radius = 2.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 5.0f, 30.0f };
			m_ball = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;

			b2CreateCircleShape( m_ball, &shapeDef, &circle );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 20.0f, 10.0f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId body = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 10.0f, 0.5f, { -10.0f, 0.0f }, b2Rot_identity );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( body, &shapeDef, &box );

			b2Vec2 pivot = { 19.0f, 10.0f };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = body;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.lowerAngle = -0.25f * B2_PI;
			jointDef.upperAngle = 0.1f * B2_PI;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = m_motorTorque;

			m_jointId2 = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}
	}

	void UpdateGui() override
	{
		float height = 220.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Revolute Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			b2RevoluteJoint_EnableLimit( m_jointId1, m_enableLimit );
			b2Joint_WakeBodies( m_jointId1 );
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2RevoluteJoint_EnableMotor( m_jointId1, m_enableMotor );
			b2Joint_WakeBodies( m_jointId1 );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Torque", &m_motorTorque, 0.0f, 5000.0f, "%.0f" ) )
			{
				b2RevoluteJoint_SetMaxMotorTorque( m_jointId1, m_motorTorque );
				b2Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f" ) )
			{
				b2RevoluteJoint_SetMotorSpeed( m_jointId1, m_motorSpeed );
				b2Joint_WakeBodies( m_jointId1 );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b2RevoluteJoint_EnableSpring( m_jointId1, m_enableSpring );
			b2Joint_WakeBodies( m_jointId1 );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 30.0f, "%.1f" ) )
			{
				b2RevoluteJoint_SetSpringHertz( m_jointId1, m_hertz );
				b2Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b2RevoluteJoint_SetSpringDampingRatio( m_jointId1, m_dampingRatio );
				b2Joint_WakeBodies( m_jointId1 );
			}

			if ( ImGui::SliderFloat( "Degrees", &m_targetDegrees, -180.0f, 180.0f, "%.0f" ) )
			{
				b2RevoluteJoint_SetTargetAngle( m_jointId1, B2_PI * m_targetDegrees / 180.0f );
				b2Joint_WakeBodies( m_jointId1 );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		float angle1 = b2RevoluteJoint_GetAngle( m_jointId1 );
		DrawTextLine( "Angle (Deg) 1 = %2.1f", angle1 );

		float torque1 = b2RevoluteJoint_GetMotorTorque( m_jointId1 );
		DrawTextLine( "Motor Torque 1 = %4.1f", torque1 );

		float torque2 = b2RevoluteJoint_GetMotorTorque( m_jointId2 );
		DrawTextLine( "Motor Torque 2 = %4.1f", torque2 );
	}

	static Sample* Create( SampleContext* context )
	{
		return new RevoluteJoint( context );
	}

	b2BodyId m_ball;
	b2JointId m_jointId1;
	b2JointId m_jointId2;
	float m_motorSpeed;
	float m_motorTorque;
	float m_hertz;
	float m_dampingRatio;
	float m_targetDegrees;
	bool m_enableSpring;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleRevolute = RegisterSample( "Joints", "Revolute", RevoluteJoint::Create );

class PrismaticJoint : public Sample
{
public:
	explicit PrismaticJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 25.0f * 0.5f;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_enableSpring = false;
		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 2.0f;
		m_motorForce = 25.0f;
		m_hertz = 1.0f;
		m_dampingRatio = 0.5f;
		m_translation = 0.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 10.0f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( 0.5f, 2.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = { 0.0f, 9.0f };
			// b2Vec2 axis = b2Normalize({1.0f, 0.0f});
			b2Vec2 axis = b2Normalize( { 1.0f, 1.0f } );
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorForce = m_motorForce;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerTranslation = -10.0f;
			jointDef.upperTranslation = 10.0f;
			jointDef.enableLimit = m_enableLimit;
			jointDef.enableSpring = m_enableSpring;
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;

			m_jointId = b2CreatePrismaticJoint( m_worldId, &jointDef );
		}
	}

	void UpdateGui() override
	{
		float height = 240.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Prismatic Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			b2PrismaticJoint_EnableLimit( m_jointId, m_enableLimit );
			b2Joint_WakeBodies( m_jointId );
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2PrismaticJoint_EnableMotor( m_jointId, m_enableMotor );
			b2Joint_WakeBodies( m_jointId );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Max Force", &m_motorForce, 0.0f, 200.0f, "%.0f" ) )
			{
				b2PrismaticJoint_SetMaxMotorForce( m_jointId, m_motorForce );
				b2Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -40.0f, 40.0f, "%.0f" ) )
			{
				b2PrismaticJoint_SetMotorSpeed( m_jointId, m_motorSpeed );
				b2Joint_WakeBodies( m_jointId );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b2PrismaticJoint_EnableSpring( m_jointId, m_enableSpring );
			b2Joint_WakeBodies( m_jointId );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b2PrismaticJoint_SetSpringHertz( m_jointId, m_hertz );
				b2Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b2PrismaticJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
				b2Joint_WakeBodies( m_jointId );
			}

			if ( ImGui::SliderFloat( "Translation", &m_translation, -5.0f, 5.0f, "%.1f" ) )
			{
				b2PrismaticJoint_SetTargetTranslation( m_jointId, m_translation );
				b2Joint_WakeBodies( m_jointId );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		float force = b2PrismaticJoint_GetMotorForce( m_jointId );
		DrawTextLine( "Motor Force = %4.1f", force );

		float translation = b2PrismaticJoint_GetTranslation( m_jointId );
		DrawTextLine( "Translation = %4.1f", translation );

		float speed = b2PrismaticJoint_GetSpeed( m_jointId );
		DrawTextLine( "Speed = %4.8f", speed );
	}

	static Sample* Create( SampleContext* context )
	{
		return new PrismaticJoint( context );
	}

	b2JointId m_jointId;
	float m_motorSpeed;
	float m_motorForce;
	float m_hertz;
	float m_dampingRatio;
	float m_translation;
	bool m_enableSpring;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int samplePrismatic = RegisterSample( "Joints", "Prismatic", PrismaticJoint::Create );

class WheelJoint : public Sample
{
public:
	explicit WheelJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 10.0f };
			m_context->camera.m_zoom = 25.0f * 0.15f;
		}

		b2BodyId groundId;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_enableSpring = true;
		m_enableLimit = true;
		m_enableMotor = true;
		m_motorSpeed = 2.0f;
		m_motorTorque = 5.0f;
		m_hertz = 1.0f;
		m_dampingRatio = 0.7f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 10.25f };
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Capsule capsule = { { 0.0f, -0.5f }, { 0.0f, 0.5f }, 0.5f };
			b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

			b2Vec2 pivot = { 0.0f, 10.0f };
			b2Vec2 axis = b2Normalize( { 1.0f, 1.0f } );
			b2WheelJointDef jointDef = b2DefaultWheelJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, axis );
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.motorSpeed = m_motorSpeed;
			jointDef.maxMotorTorque = m_motorTorque;
			jointDef.enableMotor = m_enableMotor;
			jointDef.lowerTranslation = -3.0f;
			jointDef.upperTranslation = 3.0f;
			jointDef.enableLimit = m_enableLimit;
			jointDef.hertz = m_hertz;
			jointDef.dampingRatio = m_dampingRatio;

			m_jointId = b2CreateWheelJoint( m_worldId, &jointDef );
		}
	}

	void UpdateGui() override
	{
		float height = 220.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Wheel Joint", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Limit", &m_enableLimit ) )
		{
			b2WheelJoint_EnableLimit( m_jointId, m_enableLimit );
		}

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2WheelJoint_EnableMotor( m_jointId, m_enableMotor );
		}

		if ( m_enableMotor )
		{
			if ( ImGui::SliderFloat( "Torque", &m_motorTorque, 0.0f, 20.0f, "%.0f" ) )
			{
				b2WheelJoint_SetMaxMotorTorque( m_jointId, m_motorTorque );
			}

			if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f" ) )
			{
				b2WheelJoint_SetMotorSpeed( m_jointId, m_motorSpeed );
			}
		}

		if ( ImGui::Checkbox( "Spring", &m_enableSpring ) )
		{
			b2WheelJoint_EnableSpring( m_jointId, m_enableSpring );
		}

		if ( m_enableSpring )
		{
			if ( ImGui::SliderFloat( "Hertz", &m_hertz, 0.0f, 10.0f, "%.1f" ) )
			{
				b2WheelJoint_SetSpringHertz( m_jointId, m_hertz );
			}

			if ( ImGui::SliderFloat( "Damping", &m_dampingRatio, 0.0f, 2.0f, "%.1f" ) )
			{
				b2WheelJoint_SetSpringDampingRatio( m_jointId, m_dampingRatio );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		float torque = b2WheelJoint_GetMotorTorque( m_jointId );
		DrawTextLine( "Motor Torque = %4.1f", torque );
	}

	static Sample* Create( SampleContext* context )
	{
		return new WheelJoint( context );
	}

	b2JointId m_jointId;
	float m_hertz;
	float m_dampingRatio;
	float m_motorSpeed;
	float m_motorTorque;
	bool m_enableSpring;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int sampleWheel = RegisterSample( "Joints", "Wheel", WheelJoint::Create );

// A suspension bridge
class Bridge : public Sample
{
public:
	explicit Bridge( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_zoom = 25.0f * 2.5f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		{
			m_constraintHertz = 60.0f;
			m_constraintDampingRatio = 0.0f;
			m_springHertz = 2.0f;
			m_springDampingRatio = 0.7f;
			m_frictionTorque = 200.0f;

			b2Polygon box = b2MakeBox( 0.5f, 0.125f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			jointDef.enableSpring = true;
			jointDef.hertz = m_springHertz;
			jointDef.dampingRatio = m_springDampingRatio;

			int jointIndex = 0;

			float xbase = -80.0f;

			b2BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { xbase + 0.5f + 1.0f * i, 20.0f };
				bodyDef.linearDamping = 0.1f;
				bodyDef.angularDamping = 0.1f;

				m_bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );

				b2CreatePolygonShape( m_bodyIds[i], &shapeDef, &box );

				b2Vec2 pivot = { xbase + 1.0f * i, 20.0f };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = m_bodyIds[i];
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = m_bodyIds[i];
			}

			b2Vec2 pivot = { xbase + 1.0f * m_count, 20.0f };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = groundId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );

			assert( jointIndex == m_count + 1 );
		}

		for ( int i = 0; i < 2; ++i )
		{
			b2Vec2 vertices[3] = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, { 0.0f, 1.5f } };

			b2Hull hull = b2ComputeHull( vertices, 3 );
			b2Polygon triangle = b2MakePolygon( &hull, 0.0f );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -8.0f + 8.0f * i, 22.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &triangle );
		}

		for ( int i = 0; i < 3; ++i )
		{
			b2Circle circle = { { 0.0f, 0.0f }, 0.5f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -6.0f + 6.0f * i, 25.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCircleShape( bodyId, &shapeDef, &circle );
		}
	}

	void UpdateGui() override
	{
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 320.0f, height ) );

		ImGui::Begin( "Bridge", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.6f );

		bool updateFriction = ImGui::SliderFloat( "Joint Friction", &m_frictionTorque, 0.0f, 10000.0f, "%2.f" );
		if ( updateFriction )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2RevoluteJoint_SetMaxMotorTorque( m_jointIds[i], m_frictionTorque );
			}
		}

		if ( ImGui::SliderFloat( "Spring hertz", &m_springHertz, 0.0f, 30.0f, "%.0f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2RevoluteJoint_SetSpringHertz( m_jointIds[i], m_springHertz );
			}
		}

		if ( ImGui::SliderFloat( "Spring damping", &m_springDampingRatio, 0.0f, 2.0f, "%.1f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2RevoluteJoint_SetSpringDampingRatio( m_jointIds[i], m_springDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "Constraint hertz", &m_constraintHertz, 15.0f, 240.0f, "%.0f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2Joint_SetConstraintTuning( m_jointIds[i], m_constraintHertz, m_constraintDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "Constraint damping", &m_constraintDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2Joint_SetConstraintTuning( m_jointIds[i], m_constraintHertz, m_constraintDampingRatio );
			}
		}

		ImGui::PopItemWidth();

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Bridge( context );
	}

	static constexpr int m_count = 160;
	b2BodyId m_bodyIds[m_count];
	b2JointId m_jointIds[m_count + 1];
	float m_frictionTorque;
	float m_constraintHertz;
	float m_constraintDampingRatio;
	float m_springHertz;
	float m_springDampingRatio;
};

static int sampleBridgeIndex = RegisterSample( "Joints", "Bridge", Bridge::Create );

class BallAndChain : public Sample
{
public:
	explicit BallAndChain( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, -8.0f };
			m_context->camera.m_zoom = 27.5f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_frictionTorque = 100.0f;

		{
			float hx = 0.5f;
			b2Capsule capsule = { { -hx, 0.0f }, { hx, 0.0f }, 0.125f };

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;
			shapeDef.filter.categoryBits = 0x1;
			shapeDef.filter.maskBits = 0x2;
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

			int jointIndex = 0;

			b2BodyId prevBodyId = groundId;
			for ( int i = 0; i < m_count; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { ( 1.0f + 2.0f * i ) * hx, m_count * hx };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Vec2 pivot = { ( 2.0f * i ) * hx, m_count * hx };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				jointDef.enableMotor = true;
				jointDef.maxMotorTorque = m_frictionTorque;
				jointDef.enableSpring = i > 0;
				jointDef.hertz = 4.0f;
				m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Circle circle = { { 0.0f, 0.0f }, 4.0f };

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { ( 1.0f + 2.0f * m_count ) * hx + circle.radius - hx, m_count * hx };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			shapeDef.filter.categoryBits = 0x2;
			shapeDef.filter.maskBits = 0x1;
			b2CreateCircleShape( bodyId, &shapeDef, &circle );

			b2Vec2 pivot = { ( 2.0f * m_count ) * hx, m_count * hx };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = m_frictionTorque;
			jointDef.enableSpring = true;
			jointDef.hertz = 4.0f;
			m_jointIds[jointIndex++] = b2CreateRevoluteJoint( m_worldId, &jointDef );
			assert( jointIndex == m_count + 1 );
		}
	}

	void UpdateGui() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Ball and Chain", nullptr, ImGuiWindowFlags_NoResize );

		bool updateFriction = ImGui::SliderFloat( "Joint Friction", &m_frictionTorque, 0.0f, 1000.0f, "%2.f" );
		if ( updateFriction )
		{
			for ( int i = 0; i <= m_count; ++i )
			{
				b2RevoluteJoint_SetMaxMotorTorque( m_jointIds[i], m_frictionTorque );
			}
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BallAndChain( context );
	}

	static constexpr int m_count = 30;
	b2JointId m_jointIds[m_count + 1];
	float m_frictionTorque;
};

static int sampleBallAndChainIndex = RegisterSample( "Joints", "Ball & Chain", BallAndChain::Create );

// This sample shows the limitations of an iterative solver. The cantilever sags even though the weld
// joint is stiff as possible.
class Cantilever : public Sample
{
public:
	enum
	{
		e_count = 8
	};

	explicit Cantilever( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 25.0f * 0.35f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		{
			m_linearHertz = 15.0f;
			m_linearDampingRatio = 0.5f;
			m_angularHertz = 5.0f;
			m_angularDampingRatio = 0.5f;
			m_gravityScale = 1.0f;
			m_collideConnected = false;

			float hx = 0.5f;
			b2Capsule capsule = { { -hx, 0.0f }, { hx, 0.0f }, 0.125f };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 20.0f;

			b2WeldJointDef jointDef = b2DefaultWeldJointDef();

			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.isAwake = false;

			b2BodyId prevBodyId = groundId;
			for ( int i = 0; i < e_count; ++i )
			{
				bodyDef.position = { ( 1.0f + 2.0f * i ) * hx, 0.0f };
				m_bodyIds[i] = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( m_bodyIds[i], &shapeDef, &capsule );

				b2Vec2 pivot = { ( 2.0f * i ) * hx, 0.0f };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = m_bodyIds[i];
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				jointDef.linearHertz = m_linearHertz;
				jointDef.linearDampingRatio = m_linearDampingRatio;
				jointDef.angularHertz = m_angularHertz;
				jointDef.angularDampingRatio = m_angularDampingRatio;
				jointDef.collideConnected = m_collideConnected;
				m_jointIds[i] = b2CreateWeldJoint( m_worldId, &jointDef );

				prevBodyId = m_bodyIds[i];
			}

			m_tipId = prevBodyId;
		}
	}

	void UpdateGui() override
	{
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Cantilever", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		if ( ImGui::SliderFloat( "Linear Hertz", &m_linearHertz, 0.0f, 20.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2WeldJoint_SetLinearHertz( m_jointIds[i], m_linearHertz );
			}
		}

		if ( ImGui::SliderFloat( "Linear Damping Ratio", &m_linearDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2WeldJoint_SetLinearDampingRatio( m_jointIds[i], m_linearDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "Angular Hertz", &m_angularHertz, 0.0f, 20.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2WeldJoint_SetAngularHertz( m_jointIds[i], m_angularHertz );
			}
		}

		if ( ImGui::SliderFloat( "Angular Damping Ratio", &m_angularDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2WeldJoint_SetAngularDampingRatio( m_jointIds[i], m_angularDampingRatio );
			}
		}

		if ( ImGui::Checkbox( "Collide Connected", &m_collideConnected ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2Joint_SetCollideConnected( m_jointIds[i], m_collideConnected );
			}
		}

		if ( ImGui::SliderFloat( "Gravity Scale", &m_gravityScale, -1.0f, 1.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2Body_SetGravityScale( m_bodyIds[i], m_gravityScale );
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		b2Vec2 tipPosition = b2Body_GetPosition( m_tipId );
		DrawTextLine( "tip-y = %.2f", tipPosition.y );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Cantilever( context );
	}

	float m_linearHertz;
	float m_linearDampingRatio;
	float m_angularHertz;
	float m_angularDampingRatio;
	float m_gravityScale;
	b2BodyId m_tipId;
	b2BodyId m_bodyIds[e_count];
	b2JointId m_jointIds[e_count];
	bool m_collideConnected;
};

static int sampleCantileverIndex = RegisterSample( "Joints", "Cantilever", Cantilever::Create );

// This test ensures joints work correctly with bodies that have fixed rotation
class FixedRotation : public Sample
{
public:
	enum
	{
		e_count = 6
	};

	explicit FixedRotation( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 25.0f * 0.7f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		m_groundId = b2CreateBody( m_worldId, &bodyDef );
		m_fixedRotation = true;

		for ( int i = 0; i < e_count; ++i )
		{
			m_bodyIds[i] = b2_nullBodyId;
			m_jointIds[i] = b2_nullJointId;
		}

		CreateScene();
	}

	void CreateScene()
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( B2_IS_NON_NULL( m_jointIds[i] ) )
			{
				b2DestroyJoint( m_jointIds[i] );
				m_jointIds[i] = b2_nullJointId;
			}

			if ( B2_IS_NON_NULL( m_bodyIds[i] ) )
			{
				b2DestroyBody( m_bodyIds[i] );
				m_bodyIds[i] = b2_nullBodyId;
			}
		}

		b2Vec2 position = { -12.5f, 10.0f };
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = m_fixedRotation;

		b2Polygon box = b2MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			float length = 2.0f;
			b2Vec2 pivot1 = { position.x, position.y + 1.0f + length };
			b2Vec2 pivot2 = { position.x, position.y + 1.0f };
			b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot1 );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot2 );
			jointDef.length = length;
			m_jointIds[index] = b2CreateDistanceJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// motor joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2MotorJointDef jointDef = b2DefaultMotorJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.linearOffset = position;
			jointDef.maxForce = 200.0f;
			jointDef.maxTorque = 20.0f;
			m_jointIds[index] = b2CreateMotorJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// prismatic joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			m_jointIds[index] = b2CreatePrismaticJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// revolute joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			m_jointIds[index] = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// weld joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WeldJointDef jointDef = b2DefaultWeldJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.angularHertz = 1.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.linearHertz = 1.0f;
			jointDef.linearDampingRatio = 0.5f;
			m_jointIds[index] = b2CreateWeldJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// wheel joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WheelJointDef jointDef = b2DefaultWheelJointDef();
			jointDef.bodyIdA = m_groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			m_jointIds[index] = b2CreateWheelJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;
	}

	void UpdateGui() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

		ImGui::Begin( "Fixed Rotation", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Fixed Rotation", &m_fixedRotation ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2Body_SetFixedRotation( m_bodyIds[i], m_fixedRotation );
			}
		}

		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new FixedRotation( context );
	}

	b2BodyId m_groundId;
	b2BodyId m_bodyIds[e_count];
	b2JointId m_jointIds[e_count];
	bool m_fixedRotation;
};

static int sampleFixedRotation = RegisterSample( "Joints", "Fixed Rotation", FixedRotation::Create );

// This sample shows how to break joints when the internal reaction force becomes large.
class BreakableJoint : public Sample
{
public:
	enum
	{
		e_count = 6
	};

	explicit BreakableJoint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 25.0f * 0.7f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		for ( int i = 0; i < e_count; ++i )
		{
			m_jointIds[i] = b2_nullJointId;
		}

		b2Vec2 position = { -12.5f, 10.0f };
		bodyDef.type = b2_dynamicBody;
		bodyDef.enableSleep = false;

		b2Polygon box = b2MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			float length = 2.0f;
			b2Vec2 pivot1 = { position.x, position.y + 1.0f + length };
			b2Vec2 pivot2 = { position.x, position.y + 1.0f };
			b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot1 );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateDistanceJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// motor joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2MotorJointDef jointDef = b2DefaultMotorJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.linearOffset = position;
			jointDef.maxForce = 1000.0f;
			jointDef.maxTorque = 20.0f;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateMotorJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// prismatic joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreatePrismaticJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// revolute joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// weld joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WeldJointDef jointDef = b2DefaultWeldJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.angularHertz = 2.0f;
			jointDef.angularDampingRatio = 0.5f;
			jointDef.linearHertz = 2.0f;
			jointDef.linearDampingRatio = 0.5f;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateWeldJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		// wheel joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WheelJointDef jointDef = b2DefaultWheelJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateWheelJoint( m_worldId, &jointDef );
		}

		position.x += 5.0f;
		++index;

		m_breakForce = 1000.0f;
	}

	void UpdateGui() override
	{
		float height = 100.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Breakable Joint", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::SliderFloat( "break force", &m_breakForce, 0.0f, 10000.0f, "%.1f" );

		b2Vec2 gravity = b2World_GetGravity( m_worldId );
		if ( ImGui::SliderFloat( "gravity", &gravity.y, -50.0f, 50.0f, "%.1f" ) )
		{
			b2World_SetGravity( m_worldId, gravity );
		}

		ImGui::End();
	}

	void Step() override
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( B2_IS_NULL( m_jointIds[i] ) )
			{
				continue;
			}

			b2Vec2 force = b2Joint_GetConstraintForce( m_jointIds[i] );
			if ( b2LengthSquared( force ) > m_breakForce * m_breakForce )
			{
				b2DestroyJoint( m_jointIds[i] );
				m_jointIds[i] = b2_nullJointId;
			}
			else
			{
				b2Vec2 point = b2Joint_GetLocalAnchorA( m_jointIds[i] );
				m_context->draw.DrawString( point, "(%.1f, %.1f)", force.x, force.y );
			}
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new BreakableJoint( context );
	}

	b2JointId m_jointIds[e_count];
	float m_breakForce;
};

static int sampleBreakableJoint = RegisterSample( "Joints", "Breakable", BreakableJoint::Create );

// This sample shows how to measure joint separation. This is the unresolved constraint error.
class JointSeparation : public Sample
{
public:
	enum
	{
		e_count = 5
	};

	explicit JointSeparation( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 8.0f };
			m_context->camera.m_zoom = 25.0f;
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Segment segment = { { -40.0f, 0.0f }, { 40.0f, 0.0f } };
		b2CreateSegmentShape( groundId, &shapeDef, &segment );

		b2Vec2 position = { -20.0f, 10.0f };
		bodyDef.type = b2_dynamicBody;
		bodyDef.enableSleep = false;

		b2Polygon box = b2MakeBox( 1.0f, 1.0f );

		int index = 0;

		// distance joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			float length = 2.0f;
			b2Vec2 pivot1 = { position.x, position.y + 1.0f + length };
			b2Vec2 pivot2 = { position.x, position.y + 1.0f };
			b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot1 );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot2 );
			jointDef.length = length;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateDistanceJoint( m_worldId, &jointDef );
		}

		position.x += 10.0f;
		++index;

		// prismatic joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreatePrismaticJoint( m_worldId, &jointDef );
		}

		position.x += 10.0f;
		++index;

		// revolute joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		position.x += 10.0f;
		++index;

		// weld joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WeldJointDef jointDef = b2DefaultWeldJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateWeldJoint( m_worldId, &jointDef );
		}

		position.x += 10.0f;
		++index;

		// wheel joint
		{
			assert( index < e_count );

			bodyDef.position = position;
			m_bodyIds[index] = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( m_bodyIds[index], &shapeDef, &box );

			b2Vec2 pivot = { position.x - 1.0f, position.y };
			b2WheelJointDef jointDef = b2DefaultWheelJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_bodyIds[index];
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.localAxisA = b2Body_GetLocalVector( jointDef.bodyIdA, { 1.0f, 0.0f } );
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.7f;
			jointDef.lowerTranslation = -1.0f;
			jointDef.upperTranslation = 1.0f;
			jointDef.enableLimit = true;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 10.0f;
			jointDef.motorSpeed = 1.0f;
			jointDef.collideConnected = true;
			m_jointIds[index] = b2CreateWheelJoint( m_worldId, &jointDef );
		}

		m_impulse = 500.0f;
		m_jointHertz = 60.0f;
		m_jointDampingRatio = 2.0f;
	}

	void UpdateGui() override
	{
		float height = 180.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 260.0f, height ) );

		ImGui::Begin( "Joint Separation", nullptr, ImGuiWindowFlags_NoResize );

		b2Vec2 gravity = b2World_GetGravity( m_worldId );
		if ( ImGui::SliderFloat( "gravity", &gravity.y, -500.0f, 500.0f, "%.0f" ) )
		{
			b2World_SetGravity( m_worldId, gravity );
		}

		if ( ImGui::Button( "impulse" ) )
		{
			for (int i = 0; i < e_count; ++i)
			{
				b2Vec2 p = b2Body_GetWorldPoint( m_bodyIds[i], { 1.0f, 1.0f } );
				b2Body_ApplyLinearImpulse( m_bodyIds[i], { m_impulse, -m_impulse }, p, true );
			}
		}

		ImGui::SliderFloat( "magnitude", &m_impulse, 0.0f, 1000.0f, "%.0f" );

		if ( ImGui::SliderFloat( "hertz", &m_jointHertz, 15.0f, 120.0f, "%.0f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2Joint_SetConstraintTuning( m_jointIds[i], m_jointHertz, m_jointDampingRatio );
			}
		}

		if ( ImGui::SliderFloat( "damping", &m_jointDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			for ( int i = 0; i < e_count; ++i )
			{
				b2Joint_SetConstraintTuning( m_jointIds[i], m_jointHertz, m_jointDampingRatio );
			}
		}

		ImGui::End();
	}

	void Step() override
	{
		for ( int i = 0; i < e_count; ++i )
		{
			if ( B2_IS_NULL( m_jointIds[i] ) )
			{
				continue;
			}

			float linear = b2Joint_GetLinearSeparation( m_jointIds[i] );
			float angular = b2Joint_GetAngularSeparation( m_jointIds[i] );
			b2Vec2 point = b2Joint_GetLocalAnchorA( m_jointIds[i] );
			m_context->draw.DrawString( point, "%.2f m, %.1f deg", linear, 180.0f * angular / B2_PI );
		}

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new JointSeparation( context );
	}

	b2BodyId m_bodyIds[e_count];
	b2JointId m_jointIds[e_count];
	float m_impulse;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleJointSeparation = RegisterSample( "Joints", "Separation", JointSeparation::Create );

// This shows how you can implement a constraint outside of Box2D
class UserConstraint : public Sample
{
public:
	explicit UserConstraint( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 3.0f, -1.0f };
			m_context->camera.m_zoom = 25.0f * 0.15f;
		}

		b2Polygon box = b2MakeBox( 1.0f, 0.5f );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = 20.0f;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.gravityScale = 1.0f;
		bodyDef.angularDamping = 0.5f;
		bodyDef.linearDamping = 0.2f;
		m_bodyId = b2CreateBody( m_worldId, &bodyDef );
		b2CreatePolygonShape( m_bodyId, &shapeDef, &box );

		m_impulses[0] = 0.0f;
		m_impulses[1] = 0.0f;
	}

	void Step() override
	{
		Sample::Step();

		b2Transform axes = b2Transform_identity;
		m_context->draw.DrawTransform( axes );

		if ( m_context->pause )
		{
			return;
		}

		float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		if ( timeStep == 0.0f )
		{
			return;
		}

		float invTimeStep = m_context->hertz;

		static float hertz = 3.0f;
		static float zeta = 0.7f;
		static float maxForce = 1000.0f;
		float omega = 2.0f * B2_PI * hertz;
		float sigma = 2.0f * zeta + timeStep * omega;
		float s = timeStep * omega * sigma;
		float impulseCoefficient = 1.0f / ( 1.0f + s );
		float massCoefficient = s * impulseCoefficient;
		float biasCoefficient = omega / sigma;

		b2Vec2 localAnchors[2] = { { 1.0f, -0.5f }, { 1.0f, 0.5f } };
		float mass = b2Body_GetMass( m_bodyId );
		float invMass = mass < 0.0001f ? 0.0f : 1.0f / mass;
		float inertiaTensor = b2Body_GetRotationalInertia( m_bodyId );
		float invI = inertiaTensor < 0.0001f ? 0.0f : 1.0f / inertiaTensor;

		b2Vec2 vB = b2Body_GetLinearVelocity( m_bodyId );
		float omegaB = b2Body_GetAngularVelocity( m_bodyId );
		b2Vec2 pB = b2Body_GetWorldCenterOfMass( m_bodyId );

		for ( int i = 0; i < 2; ++i )
		{
			b2Vec2 anchorA = { 3.0f, 0.0f };
			b2Vec2 anchorB = b2Body_GetWorldPoint( m_bodyId, localAnchors[i] );

			b2Vec2 deltaAnchor = b2Sub( anchorB, anchorA );

			float slackLength = 1.0f;
			float length = b2Length( deltaAnchor );
			float C = length - slackLength;
			if ( C < 0.0f || length < 0.001f )
			{
				m_context->draw.DrawSegment( anchorA, anchorB, b2_colorLightCyan );
				m_impulses[i] = 0.0f;
				continue;
			}

			m_context->draw.DrawSegment( anchorA, anchorB, b2_colorViolet );
			b2Vec2 axis = b2Normalize( deltaAnchor );

			b2Vec2 rB = b2Sub( anchorB, pB );
			float Jb = b2Cross( rB, axis );
			float K = invMass + Jb * invI * Jb;
			float invK = K < 0.0001f ? 0.0f : 1.0f / K;

			float Cdot = b2Dot( vB, axis ) + Jb * omegaB;
			float impulse = -massCoefficient * invK * ( Cdot + biasCoefficient * C );
			float appliedImpulse = b2ClampFloat( impulse, -maxForce * timeStep, 0.0f );

			vB = b2MulAdd( vB, invMass * appliedImpulse, axis );
			omegaB += appliedImpulse * invI * Jb;

			m_impulses[i] = appliedImpulse;
		}

		b2Body_SetLinearVelocity( m_bodyId, vB );
		b2Body_SetAngularVelocity( m_bodyId, omegaB );

		DrawTextLine( "forces = %g, %g", m_impulses[0] * invTimeStep, m_impulses[1] * invTimeStep );
	}

	static Sample* Create( SampleContext* context )
	{
		return new UserConstraint( context );
	}

	b2BodyId m_bodyId;
	float m_impulses[2];
};

static int sampleUserConstraintIndex = RegisterSample( "Joints", "User Constraint", UserConstraint::Create );

// This is a fun demo that shows off the wheel joint
class Driving : public Sample
{
public:
	explicit Driving( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center.y = 5.0f;
			m_context->camera.m_zoom = 25.0f * 0.4f;
			m_context->drawJoints = false;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Vec2 points[25];
			int count = 24;

			// fill in reverse to match line list convention
			points[count--] = { -20.0f, -20.0f };
			points[count--] = { -20.0f, 0.0f };
			points[count--] = { 20.0f, 0.0f };

			float hs[10] = { 0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f };
			float x = 20.0f, dx = 5.0f;

			for ( int j = 0; j < 2; ++j )
			{
				for ( int i = 0; i < 10; ++i )
				{
					float y2 = hs[i];
					points[count--] = { x + dx, y2 };
					x += dx;
				}
			}

			// flat before bridge
			points[count--] = { x + 40.0f, 0.0f };
			points[count--] = { x + 40.0f, -20.0f };

			assert( count == -1 );

			b2ChainDef chainDef = b2DefaultChainDef();
			chainDef.points = points;
			chainDef.count = 25;
			chainDef.isLoop = true;
			b2CreateChain( groundId, &chainDef );

			// flat after bridge
			x += 80.0f;
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { x, 0.0f }, { x + 40.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			// jump ramp
			x += 40.0f;
			segment = { { x, 0.0f }, { x + 10.0f, 5.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			// final corner
			x += 20.0f;
			segment = { { x, 0.0f }, { x + 40.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			x += 40.0f;
			segment = { { x, 0.0f }, { x, 20.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Teeter
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 140.0f, 1.0f };
			bodyDef.angularVelocity = 1.0f;
			bodyDef.type = b2_dynamicBody;
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Polygon box = b2MakeBox( 10.0f, 0.25f );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			b2Vec2 pivot = bodyDef.position;
			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = bodyId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.lowerAngle = -8.0f * B2_PI / 180.0f;
			jointDef.upperAngle = 8.0f * B2_PI / 180.0f;
			jointDef.enableLimit = true;
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Bridge
		{
			int N = 20;
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.125f };

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();

			b2BodyId prevBodyId = groundId;
			for ( int i = 0; i < N; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;
				bodyDef.position = { 161.0f + 2.0f * i, -0.125f };
				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Vec2 pivot = { 160.0f + 2.0f * i, -0.125f };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
				b2CreateRevoluteJoint( m_worldId, &jointDef );

				prevBodyId = bodyId;
			}

			b2Vec2 pivot = { 160.0f + 2.0f * N, -0.125f };
			jointDef.bodyIdA = prevBodyId;
			jointDef.bodyIdB = groundId;
			jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
			jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = 50.0f;
			b2CreateRevoluteJoint( m_worldId, &jointDef );
		}

		// Boxes
		{
			b2Polygon box = b2MakeBox( 0.5f, 0.5f );

			b2BodyId bodyId;
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.material.friction = 0.25f;
			shapeDef.material.restitution = 0.25f;
			shapeDef.density = 0.25f;

			bodyDef.position = { 230.0f, 0.5f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			bodyDef.position = { 230.0f, 1.5f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			bodyDef.position = { 230.0f, 2.5f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			bodyDef.position = { 230.0f, 3.5f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );

			bodyDef.position = { 230.0f, 4.5f };
			bodyId = b2CreateBody( m_worldId, &bodyDef );
			b2CreatePolygonShape( bodyId, &shapeDef, &box );
		}

		// Car

		m_throttle = 0.0f;
		m_speed = 35.0f;
		m_torque = 5.0f;
		m_hertz = 5.0f;
		m_dampingRatio = 0.7f;

		m_car.Spawn( m_worldId, { 0.0f, 0.0f }, 1.0f, m_hertz, m_dampingRatio, m_torque, nullptr );
	}

	void UpdateGui() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 200.0f, height ) );

		ImGui::Begin( "Driving", nullptr, ImGuiWindowFlags_NoResize );

		ImGui::PushItemWidth( 100.0f );
		if ( ImGui::SliderFloat( "Spring Hertz", &m_hertz, 0.0f, 20.0f, "%.0f" ) )
		{
			m_car.SetHertz( m_hertz );
		}

		if ( ImGui::SliderFloat( "Damping Ratio", &m_dampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			m_car.SetDampingRadio( m_dampingRatio );
		}

		if ( ImGui::SliderFloat( "Speed", &m_speed, 0.0f, 50.0f, "%.0f" ) )
		{
			m_car.SetSpeed( m_throttle * m_speed );
		}

		if ( ImGui::SliderFloat( "Torque", &m_torque, 0.0f, 10.0f, "%.1f" ) )
		{
			m_car.SetTorque( m_torque );
		}
		ImGui::PopItemWidth();

		ImGui::End();
	}

	void Step() override
	{
		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) == GLFW_PRESS )
		{
			m_throttle = 1.0f;
			m_car.SetSpeed( m_speed );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_S ) == GLFW_PRESS )
		{
			m_throttle = 0.0f;
			m_car.SetSpeed( 0.0f );
		}

		if ( glfwGetKey( m_context->window, GLFW_KEY_D ) == GLFW_PRESS )
		{
			m_throttle = -1.0f;
			m_car.SetSpeed( -m_speed );
		}

		DrawTextLine( "Keys: left = a, brake = s, right = d" );

		b2Vec2 linearVelocity = b2Body_GetLinearVelocity( m_car.m_chassisId );
		float kph = linearVelocity.x * 3.6f;
		DrawTextLine( "speed in kph: %.2g", kph );

		b2Vec2 carPosition = b2Body_GetPosition( m_car.m_chassisId );
		m_context->camera.m_center.x = carPosition.x;

		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Driving( context );
	}

	Car m_car;

	float m_throttle;
	float m_hertz;
	float m_dampingRatio;
	float m_torque;
	float m_speed;
};

static int sampleDriving = RegisterSample( "Joints", "Driving", Driving::Create );

class Ragdoll : public Sample
{
public:
	explicit Ragdoll( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 12.0f };
			m_context->camera.m_zoom = 16.0f;

			// m_context->camera.m_center = { 0.0f, 26.0f };
			// m_context->camera.m_zoom = 1.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_jointFrictionTorque = 0.03f;
		m_jointHertz = 5.0f;
		m_jointDampingRatio = 0.5f;

		m_human = {};

		Spawn();

		b2World_SetContactTuning( m_worldId, 240.0f, 0.0f, 2.0f );
	}

	void Spawn()
	{
		CreateHuman( &m_human, m_worldId, { 0.0f, 25.0f }, 1.0f, m_jointFrictionTorque, m_jointHertz, m_jointDampingRatio, 1,
					 nullptr, false );
		//Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
	}

	void UpdateGui() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 180.0f, height ) );

		ImGui::Begin( "Ragdoll", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 100.0f );

		if ( ImGui::SliderFloat( "Friction", &m_jointFrictionTorque, 0.0f, 1.0f, "%3.2f" ) )
		{
			Human_SetJointFrictionTorque( &m_human, m_jointFrictionTorque );
		}

		if ( ImGui::SliderFloat( "Hertz", &m_jointHertz, 0.0f, 10.0f, "%3.1f" ) )
		{
			Human_SetJointSpringHertz( &m_human, m_jointHertz );
		}

		if ( ImGui::SliderFloat( "Damping", &m_jointDampingRatio, 0.0f, 4.0f, "%3.1f" ) )
		{
			Human_SetJointDampingRatio( &m_human, m_jointDampingRatio );
		}

		if ( ImGui::Button( "Respawn" ) )
		{
			DestroyHuman( &m_human );
			Spawn();
		}
		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new Ragdoll( context );
	}

	Human m_human;
	float m_jointFrictionTorque;
	float m_jointHertz;
	float m_jointDampingRatio;
};

static int sampleRagdoll = RegisterSample( "Joints", "Ragdoll", Ragdoll::Create );

class SoftBody : public Sample
{
public:
	explicit SoftBody( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 5.0f };
			m_context->camera.m_zoom = 25.0f * 0.25f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		m_donut.Create( m_worldId, { 0.0f, 10.0f }, 2.0f, 0, false, nullptr );
	}

	static Sample* Create( SampleContext* context )
	{
		return new SoftBody( context );
	}

	Donut m_donut;
};

static int sampleDonut = RegisterSample( "Joints", "Soft Body", SoftBody::Create );

class DoohickeyFarm : public Sample
{
public:
	explicit DoohickeyFarm( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 5.0f };
			m_context->camera.m_zoom = 25.0f * 0.35f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );

			b2Polygon box = b2MakeOffsetBox( 1.0f, 1.0f, { 0.0f, 1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		float y = 4.0f;
		for ( int i = 0; i < 4; ++i )
		{
			Doohickey doohickey;
			doohickey.Spawn( m_worldId, { 0.0f, y }, 0.5f );
			y += 2.0f;
		}
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new DoohickeyFarm( context );
	}
};

static int sampleDoohickey = RegisterSample( "Joints", "Doohickey", DoohickeyFarm::Create );

class ScissorLift : public Sample
{
public:
	explicit ScissorLift( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 9.0f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
		}

		// Need 8 sub-steps for smoother operation
		m_context->subStepCount = 8;

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.sleepThreshold = 0.01f;

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		b2Capsule capsule = { { -2.5f, 0.0f }, { 2.5f, 0.0f }, 0.15f };

		b2BodyId baseId1 = groundId;
		b2BodyId baseId2 = groundId;
		b2Vec2 baseAnchor1 = { -2.5f, 0.2f };
		b2Vec2 baseAnchor2 = { 2.5f, 0.2f };
		float y = 0.5f;

		b2BodyId linkId1;
		int N = 3;

		for ( int i = 0; i < N; ++i )
		{
			bodyDef.position = { 0.0f, y };
			bodyDef.rotation = b2MakeRot( 0.15f );
			b2BodyId bodyId1 = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId1, &shapeDef, &capsule );

			bodyDef.position = { 0.0f, y };
			bodyDef.rotation = b2MakeRot( -0.15f );

			b2BodyId bodyId2 = b2CreateBody( m_worldId, &bodyDef );
			b2CreateCapsuleShape( bodyId2, &shapeDef, &capsule );

			if ( i == 1 )
			{
				linkId1 = bodyId2;
			}

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();

			// left pin
			revoluteDef.bodyIdA = baseId1;
			revoluteDef.bodyIdB = bodyId1;
			revoluteDef.localAnchorA = baseAnchor1;
			revoluteDef.localAnchorB = { -2.5f, 0.0f };
			revoluteDef.collideConnected = ( i == 0 ) ? true : false;

			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			// right pin
			if ( i == 0 )
			{
				b2WheelJointDef wheelDef = b2DefaultWheelJointDef();
				wheelDef.bodyIdA = baseId2;
				wheelDef.bodyIdB = bodyId2;
				wheelDef.localAxisA = { 1.0f, 0.0f };
				wheelDef.localAnchorA = baseAnchor2;
				wheelDef.localAnchorB = { 2.5f, 0.0f };
				wheelDef.enableSpring = false;
				wheelDef.collideConnected = true;

				b2CreateWheelJoint( m_worldId, &wheelDef );
			}
			else
			{
				revoluteDef.bodyIdA = baseId2;
				revoluteDef.bodyIdB = bodyId2;
				revoluteDef.localAnchorA = baseAnchor2;
				revoluteDef.localAnchorB = { 2.5f, 0.0f };
				revoluteDef.collideConnected = false;

				b2CreateRevoluteJoint( m_worldId, &revoluteDef );
			}

			// middle pin
			revoluteDef.bodyIdA = bodyId1;
			revoluteDef.bodyIdB = bodyId2;
			revoluteDef.localAnchorA = { 0.0f, 0.0f };
			revoluteDef.localAnchorB = { 0.0f, 0.0f };
			revoluteDef.collideConnected = false;

			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			baseId1 = bodyId2;
			baseId2 = bodyId1;
			baseAnchor1 = { -2.5f, 0.0f };
			baseAnchor2 = { 2.5f, 0.0f };
			y += 1.0f;
		}

		bodyDef.position = { 0.0f, y };
		bodyDef.rotation = b2Rot_identity;
		b2BodyId platformId = b2CreateBody( m_worldId, &bodyDef );

		b2Polygon box = b2MakeBox( 3.0f, 0.2f );
		b2CreatePolygonShape( platformId, &shapeDef, &box );

		// left pin
		b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
		revoluteDef.bodyIdA = platformId;
		revoluteDef.bodyIdB = baseId1;
		revoluteDef.localAnchorA = { -2.5f, -0.4f };
		revoluteDef.localAnchorB = baseAnchor1;
		revoluteDef.collideConnected = true;
		b2CreateRevoluteJoint( m_worldId, &revoluteDef );

		// right pin
		b2WheelJointDef wheelDef = b2DefaultWheelJointDef();
		wheelDef.bodyIdA = platformId;
		wheelDef.bodyIdB = baseId2;
		wheelDef.localAxisA = { 1.0f, 0.0f };
		wheelDef.localAnchorA = { 2.5f, -0.4f };
		wheelDef.localAnchorB = baseAnchor2;
		wheelDef.enableSpring = false;
		wheelDef.collideConnected = true;
		b2CreateWheelJoint( m_worldId, &wheelDef );

		m_enableMotor = false;
		m_motorSpeed = 0.25f;
		m_motorForce = 2000.0f;

		b2DistanceJointDef distanceDef = b2DefaultDistanceJointDef();
		distanceDef.bodyIdA = groundId;
		distanceDef.bodyIdB = linkId1;
		distanceDef.localAnchorA = { -2.5f, 0.2f };
		distanceDef.localAnchorB = { 0.5f, 0.0f };
		distanceDef.enableSpring = true;
		distanceDef.minLength = 0.2f;
		distanceDef.maxLength = 5.5f;
		distanceDef.enableLimit = true;
		distanceDef.enableMotor = m_enableMotor;
		distanceDef.motorSpeed = m_motorSpeed;
		distanceDef.maxMotorForce = m_motorForce;
		m_liftJointId = b2CreateDistanceJoint( m_worldId, &distanceDef );

		Car car;
		car.Spawn( m_worldId, { 0.0f, y + 2.0f }, 1.0f, 3.0f, 0.7f, 0.0f, nullptr );
	}

	void UpdateGui() override
	{
		float height = 140.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Scissor Lift", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2DistanceJoint_EnableMotor( m_liftJointId, m_enableMotor );
			b2Joint_WakeBodies( m_liftJointId );
		}

		if ( ImGui::SliderFloat( "Max Force", &m_motorForce, 0.0f, 3000.0f, "%.0f" ) )
		{
			b2DistanceJoint_SetMaxMotorForce( m_liftJointId, m_motorForce );
			b2Joint_WakeBodies( m_liftJointId );
		}

		if ( ImGui::SliderFloat( "Speed", &m_motorSpeed, -0.3f, 0.3f, "%.2f" ) )
		{
			b2DistanceJoint_SetMotorSpeed( m_liftJointId, m_motorSpeed );
			b2Joint_WakeBodies( m_liftJointId );
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();
	}

	static Sample* Create( SampleContext* context )
	{
		return new ScissorLift( context );
	}

	b2JointId m_liftJointId;
	float m_motorForce;
	float m_motorSpeed;
	bool m_enableMotor;
};

static int sampleScissorLift = RegisterSample( "Joints", "Scissor Lift", ScissorLift::Create );

class GearLift : public Sample
{
public:
	explicit GearLift( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 6.0f };
			m_context->camera.m_zoom = 7.0f;
		}

		b2BodyId groundId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
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

		b2Vec2 gearPosition1 = { -4.25f, 9.75f };
		b2Vec2 gearPosition2 = gearPosition1 + b2Vec2{ 2.0f, 1.0f };
		b2Vec2 linkAttachPosition = gearPosition2 + b2Vec2{ gearRadius + 2.0f * toothHalfWidth + toothRadius, 0.0f };
		b2Vec2 doorPosition = linkAttachPosition - b2Vec2{ 0.0f, 2.0f * linkCount * linkHalfLength + doorHalfHeight };

		{
			b2Vec2 position = gearPosition1;
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
			m_motorSpeed = 0.0f;
			m_enableMotor = true;

			revoluteDef.bodyIdA = groundId;
			revoluteDef.bodyIdB = bodyId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint( groundId, position );
			revoluteDef.localAnchorB = b2Vec2_zero;
			revoluteDef.enableMotor = m_enableMotor;
			revoluteDef.maxMotorTorque = m_motorTorque;
			revoluteDef.motorSpeed = m_motorSpeed;
			m_driverId = b2CreateRevoluteJoint( m_worldId, &revoluteDef );
		}

		b2BodyId followerId;

		{
			b2Vec2 position = gearPosition2;
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

			revoluteDef.bodyIdA = groundId;
			revoluteDef.bodyIdB = followerId;
			revoluteDef.localAnchorA = b2Body_GetLocalPoint( groundId, position );
			revoluteDef.localAnchorB = b2Vec2_zero;
			revoluteDef.enableMotor = true;
			revoluteDef.maxMotorTorque = 0.5f;
			revoluteDef.referenceAngle = 0.25f * B2_PI;
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
			b2Vec2 position = linkAttachPosition + b2Vec2{ 0.0f, -linkHalfLength };

			int count = 40;
			b2BodyId prevBodyId = followerId;
			for ( int i = 0; i < count; ++i )
			{
				bodyDef.position = position;

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );
				b2CreateCapsuleShape( bodyId, &shapeDef, &capsule );

				b2Vec2 pivot = { position.x, position.y + linkHalfLength };
				jointDef.bodyIdA = prevBodyId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
				jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
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
				b2Vec2 pivot = doorPosition + b2Vec2{ 0.0f, doorHalfHeight };
				b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
				revoluteDef.bodyIdA = lastLinkId;
				revoluteDef.bodyIdB = bodyId;
				revoluteDef.localAnchorA = b2Body_GetLocalPoint( lastLinkId, pivot );
				revoluteDef.localAnchorB = { 0.0f, doorHalfHeight };
				revoluteDef.enableMotor = true;
				revoluteDef.maxMotorTorque = 0.05f;
				b2CreateRevoluteJoint( m_worldId, &revoluteDef );
			}

			{
				b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
				jointDef.bodyIdA = groundId;
				jointDef.bodyIdB = bodyId;
				jointDef.localAnchorA = b2Body_GetLocalPoint( groundId, doorPosition );
				jointDef.localAnchorB = b2Vec2_zero;
				jointDef.localAxisA = { 0.0f, 1.0f };
				jointDef.maxMotorForce = 0.2f;
				jointDef.enableMotor = true;
				jointDef.collideConnected = true;
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
					bodyDef.position = { x, y };
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

	void UpdateGui() override
	{
		float height = 120.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 25.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Gear Lift", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Checkbox( "Motor", &m_enableMotor ) )
		{
			b2RevoluteJoint_EnableMotor( m_driverId, m_enableMotor );
			b2Joint_WakeBodies( m_driverId );
		}

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

		ImGui::End();
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
	}

	static Sample* Create( SampleContext* context )
	{
		return new GearLift( context );
	}

	b2JointId m_driverId;
	float m_motorTorque;
	float m_motorSpeed;
	bool m_enableMotor;
};

static int sampleGearLift = RegisterSample( "Joints", "Gear Lift", GearLift::Create );

// A top down door
class Door : public Sample
{
public:
	explicit Door( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 0.0f };
			m_context->camera.m_zoom = 4.0f;
		}

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, 0.0f };
			groundId = b2CreateBody( m_worldId, &bodyDef );
		}

		m_enableLimit = true;
		m_impulse = 50000.0f;
		m_translationError = 0.0f;
		m_jointHertz = 240.0f;
		m_jointDampingRatio = 1.0f;

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { 0.0f, 1.5f };
			bodyDef.gravityScale = 0.0f;

			m_doorId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1000.0f;

			b2Polygon box = b2MakeBox( 0.1f, 1.5f );
			b2CreatePolygonShape( m_doorId, &shapeDef, &box );

			b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
			jointDef.bodyIdA = groundId;
			jointDef.bodyIdB = m_doorId;
			jointDef.localAnchorA = {0.0f, 0.0f};
			jointDef.localAnchorB = {0.0f, -1.5f};
			jointDef.targetAngle = 0.0f;
			jointDef.enableSpring = true;
			jointDef.hertz = 1.0f;
			jointDef.dampingRatio = 0.5f;
			jointDef.motorSpeed = 0.0f;
			jointDef.maxMotorTorque = 0.0f;
			jointDef.enableMotor = false;
			jointDef.referenceAngle = 0.0f;
			jointDef.lowerAngle = -0.5f * B2_PI;
			jointDef.upperAngle = 0.5f * B2_PI;
			jointDef.enableLimit = m_enableLimit;

			m_jointId = b2CreateRevoluteJoint( m_worldId, &jointDef );
			b2Joint_SetConstraintTuning( m_jointId, m_jointHertz, m_jointDampingRatio );
		}
	}

	void UpdateGui() override
	{
		float height = 220.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 240.0f, height ) );

		ImGui::Begin( "Door", nullptr, ImGuiWindowFlags_NoResize );

		if ( ImGui::Button( "impulse" ) )
		{
			b2Vec2 p = b2Body_GetWorldPoint( m_doorId, { 0.0f, 1.5f } );
			b2Body_ApplyLinearImpulse( m_doorId, { m_impulse, 0.0f }, p, true );
			m_translationError = 0.0f;
		}

		ImGui::SliderFloat( "magnitude", &m_impulse, 1000.0f, 100000.0f, "%.0f" );

		if ( ImGui::Checkbox( "limit", &m_enableLimit ) )
		{
			b2RevoluteJoint_EnableLimit( m_jointId, m_enableLimit );
		}

		if ( ImGui::SliderFloat( "hertz", &m_jointHertz, 15.0f, 480.0f, "%.0f" ) )
		{
			b2Joint_SetConstraintTuning( m_jointId, m_jointHertz, m_jointDampingRatio );
		}

		if ( ImGui::SliderFloat( "damping", &m_jointDampingRatio, 0.0f, 10.0f, "%.1f" ) )
		{
			b2Joint_SetConstraintTuning( m_jointId, m_jointHertz, m_jointDampingRatio );
		}

		ImGui::End();
	}

	void Step() override
	{
		Sample::Step();

		b2Vec2 p = b2Body_GetWorldPoint( m_doorId, { 0.0f, 1.5f } );
		m_draw->DrawPoint( p, 5.0f, b2_colorDarkKhaki );

		m_draw->DrawTransform( b2Transform_identity );

		float translationError = b2Joint_GetLinearSeparation( m_jointId );
		m_translationError = b2MaxFloat( m_translationError, translationError );

		DrawTextLine( "translation error = %g", m_translationError );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Door( context );
	}

	b2BodyId m_doorId;
	b2JointId m_jointId;
	float m_impulse;
	float m_translationError;
	float m_jointHertz;
	float m_jointDampingRatio;
	bool m_enableLimit;
};

static int sampleDoor = RegisterSample( "Joints", "Door", Door::Create );

class ScaleRagdoll : public Sample
{
public:
	explicit ScaleRagdoll( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 4.5f };
			m_context->camera.m_zoom = 6.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );
			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Polygon box = b2MakeOffsetBox( 20.0f, 1.0f, { 0.0f, -1.0f }, b2Rot_identity );
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		m_scale = 1.0f;

		m_human = {};

		Spawn();
	}

	void Spawn()
	{
		float jointFrictionTorque = 0.03f;
		float jointHertz = 1.0f;
		float jointDampingRatio = 0.5f;
		CreateHuman( &m_human, m_worldId, { 0.0f, 5.0f }, m_scale, jointFrictionTorque, jointHertz, jointDampingRatio, 1, nullptr,
					 false );
		Human_ApplyRandomAngularImpulse( &m_human, 10.0f );
	}

	void UpdateGui() override
	{
		float height = 60.0f;
		ImGui::SetNextWindowPos( ImVec2( 10.0f, m_context->camera.m_height - height - 50.0f ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 260.0f, height ) );

		ImGui::Begin( "Scale Ragdoll", nullptr, ImGuiWindowFlags_NoResize );
		ImGui::PushItemWidth( 200.0f );

		if ( ImGui::SliderFloat( "Scale", &m_scale, 0.1f, 10.0f, "%3.2f", ImGuiSliderFlags_ClampOnInput ) )
		{
			Human_SetScale( &m_human, m_scale );
		}

		ImGui::PopItemWidth();
		ImGui::End();
	}

	static Sample* Create( SampleContext* context )
	{
		return new ScaleRagdoll( context );
	}

	Human m_human;
	float m_scale;
};

static int sampleScaleRagdoll = RegisterSample( "Joints", "Scale Ragdoll", ScaleRagdoll::Create );

