/*
* Copyright (c) 2014 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#ifndef SOUP_STIRRER_H
#define SOUP_STIRRER_H
#include "Soup.h"
#include <math.h>

// Soup stirred by a circular dynamic body.
// A force vector (following a circle) is continuously applied to the body
// while by default the body is attached to a joint restricting motion
// to the x-axis.
class SoupStirrer : public Soup
{
public:
	SoupStirrer() :
		m_stirrer(NULL),
		m_joint(NULL),
		m_oscillationOffset(0.0f)
	{
		m_particleSystem->SetDamping(1.0f);

		// Shape of the stirrer.
		b2CircleShape shape;
		shape.m_p.Set(0, 0.7f);
		shape.m_radius = 0.4f;

		// Create the stirrer.
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		m_stirrer = m_world->CreateBody(&bd);
		m_stirrer->CreateFixture(&shape, 1.0f);

		// Destroy all particles under the stirrer.
		b2Transform xf;
		xf.SetIdentity();
		m_particleSystem->DestroyParticlesInShape(shape, xf);

		// By default attach the body to a joint to restrict movement.
		CreateJoint();
	}

	// Create a joint to fix the stirrer to a single axis of movement.
	void CreateJoint()
	{
		b2Assert(!m_joint);
		// Create a prismatic joint and connect to the ground, and have it
		// slide along the x axis.
		// Disconnect the body from this joint to have more fun.
		b2PrismaticJointDef prismaticJointDef;
		prismaticJointDef.bodyA = m_ground;
		prismaticJointDef.bodyB = m_stirrer;
		prismaticJointDef.collideConnected = true;
		prismaticJointDef.localAxisA.Set(1,0);
		prismaticJointDef.localAnchorA = m_stirrer->GetPosition();
		m_joint = m_world->CreateJoint(&prismaticJointDef);
	}

	// Enable the joint if it's disabled, disable it if it's enabled.
	virtual void ToggleJoint()
	{
		if (m_joint)
		{
			m_world->DestroyJoint(m_joint);
			m_joint = 0;
		}
		else
		{
			CreateJoint();
		}
	}

	// Press "t" to enable / disable the joint restricting the stirrer's
	// movement.
	virtual void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 't':
			ToggleJoint();
			break;
		default:
			Soup::Keyboard(key);
			break;
		}
	}

	// Click the soup to toggle between enabling / disabling the joint.
	void MouseUp(const b2Vec2& p)
	{
		if (InSoup(p))
		{
			ToggleJoint();
		}
	}

	// Determine whether a point is in the soup.
	bool InSoup(const b2Vec2& pos)
	{
		// The soup dimensions are from the container initialization in the
		// Soup test.
		return pos.y > -1.0f && pos.y < 2.0f && pos.x > -3.0f && pos.x < 3.0f;
	}

	// Apply a force to the stirrer.
	void Step(Settings* settings)
	{
		// Magnitude of the force applied to the body.
		static const float32 k_forceMagnitude = 10.0f;
		// How often the force vector rotates.
		static const float32 k_forceOscillationPerSecond = 0.2f;
		static const float32 k_forceOscillationPeriod =
			1.0f / k_forceOscillationPerSecond;
		// Maximum speed of the body.
		static const float32 k_maxSpeed = 2.0f;

		m_oscillationOffset += (1.0f / settings->hz);
		if (m_oscillationOffset > k_forceOscillationPeriod)
		{
			m_oscillationOffset -= k_forceOscillationPeriod;
		}

		// Calculate the force vector.
		const float32 forceAngle = m_oscillationOffset *
			k_forceOscillationPerSecond * 2.0f * b2_pi;
		const b2Vec2 forceVector =
			b2Vec2(sinf(forceAngle), cosf(forceAngle)) * k_forceMagnitude;

		// Only apply force to the body when it's within the soup.
		if (InSoup(m_stirrer->GetPosition()) &&
			m_stirrer->GetLinearVelocity().Length() < k_maxSpeed)
		{
			m_stirrer->ApplyForceToCenter(forceVector, true);
		}
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new SoupStirrer;
	}

private:
	b2Body* m_stirrer;
	b2Joint* m_joint;
	float32 m_oscillationOffset;
};

#endif
