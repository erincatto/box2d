/*
* Copyright (c) 2006-2011 Erin Catto http://www.gphysics.com
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

#ifndef CAR_H
#define CAR_H

// A line joint with a limit and friction.
class Car : public Test
{
public:
	Car()
	{		
		m_hz = 2.0f;
		m_zeta = 0.7f;

		{
			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));

			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape box;
			box.SetAsBox(2.0f, 0.5f);

			b2CircleShape circle;
			circle.m_radius = 0.5f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 1.0f);
			m_car = m_world->CreateBody(&bd);
			m_car->CreateFixture(&box, 1.0f);

			bd.position.Set(-1.5f, 0.5f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&circle, 1.0f);

			bd.position.Set(1.5f, 0.5f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&circle, 1.0f);

			b2LineJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 1.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2LineJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2LineJoint*)m_world->CreateJoint(&jd);
		}
	}

	static Test* Create()
	{
		return new Car;
	}

	b2Body* m_car;
	b2Body* m_wheel1;
	b2Body* m_wheel2;

	float32 m_hz;
	float32 m_zeta;
	b2LineJoint* m_spring1;
	b2LineJoint* m_spring2;
};

#endif
