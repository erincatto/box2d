// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"

#include "random.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

void CreateHuman( Human* human, b2WorldId worldId, b2Vec2 position, float scale, float frictionTorque, float hertz,
				  float dampingRatio, int groupIndex, void* userData, bool colorize )
{
	assert( human->isSpawned == false );

	for ( int i = 0; i < bone_count; ++i )
	{
		human->bones[i].bodyId = b2_nullBodyId;
		human->bones[i].jointId = b2_nullJointId;
		human->bones[i].frictionScale = 1.0f;
		human->bones[i].parentIndex = -1;
	}

	human->originalScale = scale;
	human->scale = scale;
	human->frictionTorque = frictionTorque;

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.sleepThreshold = 0.1f;
	bodyDef.userData = userData;

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.material.friction = 0.2f;
	shapeDef.filter.groupIndex = -groupIndex;
	shapeDef.filter.categoryBits = 2;
	shapeDef.filter.maskBits = ( 1 | 2 );

	b2ShapeDef footShapeDef = shapeDef;
	footShapeDef.material.friction = 0.05f;

	// feet don't collide with ragdolls
	footShapeDef.filter.categoryBits = 2;
	footShapeDef.filter.maskBits = 1;

	if ( colorize )
	{
		footShapeDef.material.customColor = b2_colorSaddleBrown;
	}

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
		Bone* bone = human->bones + bone_hip;
		bone->parentIndex = -1;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.95f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "hip";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.material.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.02f * s }, { 0.0f, 0.02f * s }, 0.095f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	// torso
	{
		Bone* bone = human->bones + bone_torso;
		bone->parentIndex = bone_hip;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 1.2f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "torso";

		// bodyDef.type = b2_staticBody;
		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;
		bodyDef.type = b2_dynamicBody;

		if ( colorize )
		{
			shapeDef.material.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.135f * s }, { 0.0f, 0.135f * s }, 0.09f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.0f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.25f * B2_PI;
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
		Bone* bone = human->bones + bone_head;
		bone->parentIndex = bone_torso;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f * s, 1.475f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bodyDef.name = "head";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.25f;

		if ( colorize )
		{
			shapeDef.material.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.038f * s }, { 0.0f, 0.039f * s }, 0.075f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		//// neck
		// capsule = { { 0.0f, -0.12f * s }, { 0.0f, -0.08f * s }, 0.05f * s };
		// b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.4f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.3f * B2_PI;
		jointDef.upperAngle = 0.1f * B2_PI;
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
		Bone* bone = human->bones + bone_upperLeftLeg;
		bone->parentIndex = bone_hip;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "upper_left_leg";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.material.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * B2_PI;
		jointDef.upperAngle = 0.4f * B2_PI;
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
		Bone* bone = human->bones + bone_lowerLeftLeg;
		bone->parentIndex = bone_upperLeftLeg;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "lower_left_leg";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.material.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		// b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * B2_PI;
		jointDef.upperAngle = -0.02f * B2_PI;
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
		Bone* bone = human->bones + bone_upperRightLeg;
		bone->parentIndex = bone_hip;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.775f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "upper_right_leg";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 1.0f;

		if ( colorize )
		{
			shapeDef.material.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.06f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 0.9f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.05f * B2_PI;
		jointDef.upperAngle = 0.4f * B2_PI;
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
		Bone* bone = human->bones + bone_lowerRightLeg;
		bone->parentIndex = bone_upperRightLeg;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.475f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "lower_right_leg";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.material.customColor = pantColor;
		}

		b2Capsule capsule = { { 0.0f, -0.155f * s }, { 0.0f, 0.125f * s }, 0.045f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		// b2Polygon box = b2MakeOffsetBox(0.1f * s, 0.03f * s, {0.05f * s, -0.175f * s}, 0.0f);
		// b2CreatePolygonShape(bone->bodyId, &shapeDef, &box);

		// capsule = { { -0.02f * s, -0.175f * s }, { 0.13f * s, -0.175f * s }, 0.03f * s };
		// b2CreateCapsuleShape( bone->bodyId, &footShapeDef, &capsule );

		b2CreatePolygonShape( bone->bodyId, &footShapeDef, &footPolygon );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 0.625f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.5f * B2_PI;
		jointDef.upperAngle = -0.02f * B2_PI;
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
		Bone* bone = human->bones + bone_upperLeftArm;
		bone->parentIndex = bone_torso;
		bone->frictionScale = 0.5f;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "upper_left_arm";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );

		if ( colorize )
		{
			shapeDef.material.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * B2_PI;
		jointDef.upperAngle = 0.8f * B2_PI;
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
		Bone* bone = human->bones + bone_lowerLeftArm;
		bone->parentIndex = bone_upperLeftArm;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bodyDef.name = "lower_left_arm";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.material.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * B2_PI;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * B2_PI;
		jointDef.upperAngle = 0.3f * B2_PI;
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
		Bone* bone = human->bones + bone_upperRightArm;
		bone->parentIndex = bone_torso;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 1.225f * s }, position );
		bodyDef.linearDamping = 0.0f;
		bodyDef.name = "upper_right_arm";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.5f;

		if ( colorize )
		{
			shapeDef.material.customColor = shirtColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.035f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.35f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.1f * B2_PI;
		jointDef.upperAngle = 0.8f * B2_PI;
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
		Bone* bone = human->bones + bone_lowerRightArm;
		bone->parentIndex = bone_upperRightArm;

		bodyDef.position = b2Add( (b2Vec2){ 0.0f, 0.975f * s }, position );
		bodyDef.linearDamping = 0.1f;
		bodyDef.name = "lower_right_arm";

		bone->bodyId = b2CreateBody( worldId, &bodyDef );
		bone->frictionScale = 0.1f;

		if ( colorize )
		{
			shapeDef.material.customColor = skinColor;
		}

		b2Capsule capsule = { { 0.0f, -0.125f * s }, { 0.0f, 0.125f * s }, 0.03f * s };
		b2CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		b2Vec2 pivot = b2Add( (b2Vec2){ 0.0f, 1.1f * s }, position );
		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.bodyIdA = human->bones[bone->parentIndex].bodyId;
		jointDef.bodyIdB = bone->bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		jointDef.referenceAngle = 0.25f * B2_PI;
		jointDef.enableLimit = enableLimit;
		jointDef.lowerAngle = -0.2f * B2_PI;
		jointDef.upperAngle = 0.3f * B2_PI;
		jointDef.enableMotor = enableMotor;
		jointDef.maxMotorTorque = bone->frictionScale * maxTorque;
		jointDef.enableSpring = hertz > 0.0f;
		jointDef.hertz = hertz;
		jointDef.dampingRatio = dampingRatio;
		jointDef.drawSize = drawSize;

		bone->jointId = b2CreateRevoluteJoint( worldId, &jointDef );
	}

	human->isSpawned = true;
}

void DestroyHuman( Human* human )
{
	assert( human->isSpawned == true );

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B2_IS_NULL( human->bones[i].jointId ) )
		{
			continue;
		}

		b2DestroyJoint( human->bones[i].jointId );
		human->bones[i].jointId = b2_nullJointId;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B2_IS_NULL( human->bones[i].bodyId ) )
		{
			continue;
		}

		b2DestroyBody( human->bones[i].bodyId );
		human->bones[i].bodyId = b2_nullBodyId;
	}

	human->isSpawned = false;
}

void Human_SetVelocity( Human* human, b2Vec2 velocity )
{
	for ( int i = 0; i < bone_count; ++i )
	{
		b2BodyId bodyId = human->bones[i].bodyId;

		if ( B2_IS_NULL( bodyId ) )
		{
			continue;
		}

		b2Body_SetLinearVelocity( bodyId, velocity );
	}
}

void Human_ApplyRandomAngularImpulse( Human* human, float magnitude )
{
	assert( human->isSpawned == true );
	float impulse = RandomFloatRange( -magnitude, magnitude );
	b2Body_ApplyAngularImpulse( human->bones[bone_torso].bodyId, impulse, true );
}

void Human_SetJointFrictionTorque( Human* human, float torque )
{
	assert( human->isSpawned == true );
	if ( torque == 0.0f )
	{
		for ( int i = 1; i < bone_count; ++i )
		{
			b2RevoluteJoint_EnableMotor( human->bones[i].jointId, false );
		}
	}
	else
	{
		for ( int i = 1; i < bone_count; ++i )
		{
			b2RevoluteJoint_EnableMotor( human->bones[i].jointId, true );
			float scale = human->scale * human->bones[i].frictionScale;
			b2RevoluteJoint_SetMaxMotorTorque( human->bones[i].jointId, scale * torque );
		}
	}
}

void Human_SetJointSpringHertz( Human* human, float hertz )
{
	assert( human->isSpawned == true );
	if ( hertz == 0.0f )
	{
		for ( int i = 1; i < bone_count; ++i )
		{
			b2RevoluteJoint_EnableSpring( human->bones[i].jointId, false );
		}
	}
	else
	{
		for ( int i = 1; i < bone_count; ++i )
		{
			b2RevoluteJoint_EnableSpring( human->bones[i].jointId, true );
			b2RevoluteJoint_SetSpringHertz( human->bones[i].jointId, hertz );
		}
	}
}

void Human_SetJointDampingRatio( Human* human, float dampingRatio )
{
	assert( human->isSpawned == true );
	for ( int i = 1; i < bone_count; ++i )
	{
		b2RevoluteJoint_SetSpringDampingRatio( human->bones[i].jointId, dampingRatio );
	}
}

void Human_EnableSensorEvents( Human* human, bool enable )
{
	assert( human->isSpawned == true );
	b2BodyId bodyId = human->bones[bone_torso].bodyId;

	b2ShapeId shapeId;
	int count = b2Body_GetShapes( bodyId, &shapeId, 1 );
	if ( count == 1 )
	{
		b2Shape_EnableSensorEvents( shapeId, enable );
	}
}

void Human_SetScale( Human* human, float scale )
{
	assert( human->isSpawned == true );
	assert( 0.01f < scale && scale < 100.0f );
	assert( 0.0f < human->scale );

	float ratio = scale / human->scale;

	// Torque scales by pow(length, 4) due to mass change and length change. However, gravity is also a factor
	// so I'm using pow(length, 3)
	float originalRatio = scale / human->originalScale;
	float frictionTorque = ( originalRatio * originalRatio * originalRatio ) * human->frictionTorque;

	b2Vec2 origin = b2Body_GetPosition( human->bones[0].bodyId );

	for ( int boneIndex = 0; boneIndex < bone_count; ++boneIndex )
	{
		Bone* bone = human->bones + boneIndex;

		if ( boneIndex > 0 )
		{
			b2Transform transform = b2Body_GetTransform( bone->bodyId );
			transform.p = b2MulAdd( origin, ratio, b2Sub( transform.p, origin ) );
			b2Body_SetTransform( bone->bodyId, transform.p, transform.q );

			b2Vec2 localAnchorA = b2Joint_GetLocalAnchorA( bone->jointId );
			b2Vec2 localAnchorB = b2Joint_GetLocalAnchorB( bone->jointId );
			localAnchorA = b2MulSV( ratio, localAnchorA );
			localAnchorB = b2MulSV( ratio, localAnchorB );
			b2Joint_SetLocalAnchorA( bone->jointId, localAnchorA );
			b2Joint_SetLocalAnchorB( bone->jointId, localAnchorB );

			b2JointType type = b2Joint_GetType( bone->jointId );
			if ( type == b2_revoluteJoint )
			{
				b2RevoluteJoint_SetMaxMotorTorque( bone->jointId, bone->frictionScale * frictionTorque );
			}
		}

		b2ShapeId shapeIds[2];
		int shapeCount = b2Body_GetShapes( bone->bodyId, shapeIds, 2 );
		for ( int shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex )
		{
			b2ShapeType type = b2Shape_GetType( shapeIds[shapeIndex] );
			if ( type == b2_capsuleShape )
			{
				b2Capsule capsule = b2Shape_GetCapsule( shapeIds[shapeIndex] );
				capsule.center1 = b2MulSV( ratio, capsule.center1 );
				capsule.center2 = b2MulSV( ratio, capsule.center2 );
				capsule.radius *= ratio;
				b2Shape_SetCapsule( shapeIds[shapeIndex], &capsule );
			}
			else if ( type == b2_polygonShape )
			{
				b2Polygon polygon = b2Shape_GetPolygon( shapeIds[shapeIndex] );
				for ( int pointIndex = 0; pointIndex < polygon.count; ++pointIndex )
				{
					polygon.vertices[pointIndex] = b2MulSV( ratio, polygon.vertices[pointIndex] );
				}

				polygon.centroid = b2MulSV( ratio, polygon.centroid );
				polygon.radius *= ratio;

				b2Shape_SetPolygon( shapeIds[shapeIndex], &polygon );
			}
		}

		b2Body_ApplyMassFromShapes( bone->bodyId );
	}

	human->scale = scale;
}
