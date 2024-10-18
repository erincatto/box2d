// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"

#include "sample.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Human::Human()
{
	for ( int i = 0; i < Bone::e_count; ++i )
	{
		m_bones[i].bodyId = b2_nullBodyId;
		m_bones[i].jointId = b2_nullJointId;
		m_bones[i].frictionScale = 1.0f;
		m_bones[i].parentIndex = -1;
	}

	m_scale = 1.0f;
	m_isSpawned = false;
}

void Human::Spawn( b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz, float dampingRatio,
				   int groupIndex, void* userData, bool colorize )
{
	assert( m_isSpawned == false );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.sleepThreshold = 0.1f;
	bodyDef.userData = userData;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.friction = 0.2f;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = (1 | 2);

	b2ShapeDef footShapeDef = shapeDef;
	footShapeDef.friction = 0.05f;

	// feet don't collide with ragdolls
	footShapeDef.filter.categoryBits = 2;
	footShapeDef.filter.maskBits = 1;

	if ( colorize )
	{
		footShapeDef.customColor = b2_colorSaddleBrown;
	}

	m_scale = scale;
	float s = scale;
	float maxTorque = frictionTorque * s;
	bool enableMotor = true;
	bool enableLimit = true;
	float drawSize = 0.05f;

	b2HexColor shirtColor = b2_colorMediumTurquoise;
	b2HexColor pantColor = b2_colorDodgerBlue;

	b2HexColor skinColors[4] = { b2_colorNavajoWhite, b2_colorLightYellow, b2_colorPeru, b2_colorTan };
	b2HexColor skinColor = skinColors[groupIndex % 4];

	// hip
	{
		Bone* bone = m_bones + Bone::e_hip;
		bone->parentIndex = -1;

		bodyDef.position = b2Add( { 0.0f, 0.95f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.02f * s }, { 0.0f, 0.02f * s }, 0.095f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	// torso
	{
		Bone* bone = m_bones + Bone::e_torso;
		bone->parentIndex = Bone::e_hip;

		bodyDef.position = b2Add( { 0.0f, 1.2f * s }, position );
		bodyDef.linearDamping = 0.0f;
		//bodyDef.type = b2_staticBody;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;
		bodyDef.type = b2_dynamicBody;

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.135f * s }, { 0.0f, 0.135f * s }, 0.09f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.0f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.25f * b2_pi;
		jointDef.upperAngle = 0.0f;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// head
	{
		Bone* bone = m_bones + Bone::e_head;
		bone->parentIndex = Bone::e_torso;

		bodyDef.position = b2Add( { 0.0f * s, 1.475f * s }, position );
		bodyDef.linearDamping = 0.1f;

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.25f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.038f * s }, { 0.0f, 0.039f * s }, 0.075f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		//// neck
		// capsule = { { 0.0f, -0.12f * s }, { 0.0f, -0.08f * s }, 0.05f * s };
		// b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.4f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.3f * b2_pi;
		jointDef.upperAngle = 0.1f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper left leg
	{
		Bone* bone = m_bones + Bone::e_upperLeftLeg;
		bone->parentIndex = Bone::e_hip;

		bodyDef.position = b2Add( { 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.4f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	b2Vec2 points[4] = {
		{ -0.03f * s, -0.185f * s },
		{ 0.11f * s, -0.185f * s },
		{ 0.11f * s, -0.16f * s },
		{ -0.03f * s, -0.14f * s },
	};

	b2Hull footHull = b2ComputeHull( points, 4 );
	b2Polygon footPolygon = b2MakePolygon( &footHull, 0.015f * s );

	// lower left leg
	{
		Bone* bone = m_bones + Bone::e_lowerLeftLeg;
		bone->parentIndex = Bone::e_upperLeftLeg;

		bodyDef.position = b2Add( { 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		//capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		//b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( { 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = -0.02f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper right leg
	{
		Bone* bone = m_bones + Bone::e_upperRightLeg;
		bone->parentIndex = Bone::e_hip;

		bodyDef.position = b2Add( { 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * b2_pi;
		jointDef.upperAngle = 0.4f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower right leg
	{
		Bone* bone = m_bones + Bone::e_lowerRightLeg;
		bone->parentIndex = Bone::e_upperRightLeg;

		bodyDef.position = b2Add( { 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		//capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		//b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( { 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * b2_pi;
		jointDef.upperAngle = -0.02f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper left arm
	{
		Bone* bone = m_bones + Bone::e_upperLeftArm;
		bone->parentIndex = Bone::e_torso;
		bone->frictionScale = 0.5f;

		bodyDef.position = b2Add( { 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * b2_pi;
		jointDef.upperAngle = 0.8f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower left arm
	{
		Bone* bone = m_bones + Bone::e_lowerLeftArm;
		bone->parentIndex = Bone::e_upperLeftArm;

		bodyDef.position = b2Add( { 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * b2_pi;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * b2_pi;
		jointDef.upperAngle = 0.3f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// upper right arm
	{
		Bone* bone = m_bones + Bone::e_upperRightArm;
		bone->parentIndex = Bone::e_torso;

		bodyDef.position = b2Add( { 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * b2_pi;
		jointDef.upperAngle = 0.8f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	// lower right arm
	{
		Bone* bone = m_bones + Bone::e_lowerRightArm;
		bone->parentIndex = Bone::e_upperRightArm;

		bodyDef.position = b2Add( { 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( { 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = m_bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * b2_pi;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * b2_pi;
		jointDef.upperAngle = 0.3f * b2_pi;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	m_isSpawned = true;
}

void Human::Despawn()
{
	assert( m_isSpawned == true );

	for ( int i = 0; i < Bone::e_count; ++i )
	{
		if ( B2_IS_NULL( m_bones[i].jointId ) )
		{
			continue;
		}

		b2DestroyJoint( m_bones[i].jointId );
		m_bones[i].jointId = b2_nullJointId;
	}

	for ( int i = 0; i < Bone::e_count; ++i )
	{
		if ( B2_IS_NULL( m_bones[i].bodyId ) )
		{
			continue;
		}

		b2DestroyBody( m_bones[i].bodyId );
		m_bones[i].bodyId = b2_nullBodyId;
	}

	m_isSpawned = false;
}

void Human::ApplyRandomAngularImpulse( float magnitude )
{
	if ( m_isSpawned == false )
	{
		return;
	}

	float impulse = RandomFloat( -magnitude, magnitude );
	b2Body_ApplyAngularImpulse( m_bones[Bone::e_torso].bodyId, impulse, true );
}

void Human::SetJointFrictionTorque( float torque )
{
	if ( m_isSpawned == false )
	{
		return;
	}

	if ( torque == 0.0f )
	{
		for ( int i = 1; i < Bone::e_count; ++i )
		{
			b2RevoluteJoint_EnableMotor( m_bones[i].jointId, false );
		}
	}
	else
	{
		for ( int i = 1; i < Bone::e_count; ++i )
		{
			b2RevoluteJoint_EnableMotor( m_bones[i].jointId, true );
			float scale = m_scale * m_bones[i].frictionScale;
			b2RevoluteJoint_SetMaxMotorTorque( m_bones[i].jointId, scale * torque );
		}
	}
}

void Human::SetJointSpringHertz( float hertz )
{
	if ( m_isSpawned == false )
	{
		return;
	}

	if ( hertz == 0.0f )
	{
		for ( int i = 1; i < Bone::e_count; ++i )
		{
			b2RevoluteJoint_EnableSpring( m_bones[i].jointId, false );
		}
	}
	else
	{
		for ( int i = 1; i < Bone::e_count; ++i )
		{
			b2RevoluteJoint_EnableSpring( m_bones[i].jointId, true );
			b2RevoluteJoint_SetSpringHertz( m_bones[i].jointId, hertz );
		}
	}
}

void Human::SetJointDampingRatio( float dampingRatio )
{
	if ( m_isSpawned == false )
	{
		return;
	}

	for ( int i = 1; i < Bone::e_count; ++i )
	{
		b2RevoluteJoint_SetSpringDampingRatio( m_bones[i].jointId, dampingRatio );
	}
}
