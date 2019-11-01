/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <math.h>

#define PI 3.14159265

#ifndef WATER_WHEEL_H
#define WATER_WHEEL_H

// whater wheel with central rotating axel

class WaterWheel : public Test
{
public:
	WaterWheel()
	{
        float32 timeStep = 1/200.0;      //the length of time passed to simulate (seconds)
        int32 velocityIterations = 100;   //how strongly to correct velocity
        int32 positionIterations = 100;   //how strongly to correct position
        
        m_world->Step( timeStep, velocityIterations, positionIterations);

        b2Body* prevBody = NULL;

        // Fixed centre at 0,0
        {
            b2CircleShape shape;
            shape.m_radius = 0.1f;
            
            b2BodyDef bd;
            bd.position.Set(0.0f, 0.0f);
            
            b2Body* body = m_world->CreateBody(&bd);
            
            body->CreateFixture(&shape, 0.0f);

            prevBody = body;
        }
        
        // Define axel
        {
            b2CircleShape shape;
            shape.m_radius = 1.0f;
            
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 0.0f);
            
            b2Body* body = m_world->CreateBody(&bd);
            
            body->CreateFixture(&shape, 1.0f);
            
            b2RevoluteJointDef rjd;
            rjd.Initialize(prevBody, body, b2Vec2(0.0f, 0.0f));
            m_world->CreateJoint(&rjd);

            prevBody = body;
        }
        
        float r = 8.0;
        float sin = sinf(PI/3);
        float cos = cosf(PI/3);
        CreateCup(prevBody, 0.0, r);
        CreateCup(prevBody, -r*sin, -r*cos);
        CreateCup(prevBody, +r*sin, -r*cos);
	}

    void CreateCup(b2Body* prevBody, float x, float y) {
        b2PolygonShape bottom;
        bottom.SetAsBox(1.5f, 0.15f);

        b2PolygonShape left;
        left.SetAsBox(0.15f, 1.5f, b2Vec2(-1.45f, 2.35f), 0.2);

        b2PolygonShape right;
        right.SetAsBox(0.15f, 1.5f, b2Vec2(1.45f, 2.35f), -0.2);
        
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x, y-5);
        bd.fixedRotation = true;
        b2Body* container = m_world->CreateBody(&bd);
        
        container->CreateFixture(&bottom, 1.0f);
        container->CreateFixture(&left, 1.0f);
        container->CreateFixture(&right, 1.0f);
        
        b2RevoluteJointDef rjd;
        rjd.Initialize(prevBody, container, b2Vec2(x, y));
        m_world->CreateJoint(&rjd);
        
        //m_world->CreateJoint(pl.RevoluteJoint({}, follower, container, Vec2(x, y)));
    }
    
	void Step(Settings* settings)
	{
		Test::Step(settings);
        
        // Inlet
        if(step++ % 10 == 0 && step < 2000) {
            b2CircleShape shape;
            shape.m_radius = 0.25f;
            
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(RandomFloat(0.0f, 1.0f), 15.0f);
        
            b2Body* body = m_world->CreateBody(&bd);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 5.0f;
            fd.friction = 0.10f;
            
            body->CreateFixture(&fd);

            body->SetLinearVelocity(b2Vec2(0.0f, -5.0f));
        }
	}

	static Test* Create()
	{
		return new WaterWheel;
	}

	b2PrismaticJoint* m_joint2;
    int step = 0;
};

#endif
