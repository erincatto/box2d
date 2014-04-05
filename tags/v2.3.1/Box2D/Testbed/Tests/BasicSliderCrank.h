/*
 * Copyright (c) 2006-2014 Erin Catto http://www.box2d.org
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

#ifndef BASIC_SLIDER_CRANK_H
#define BASIC_SLIDER_CRANK_H

// A basic slider crank created for GDC tutorial: Understanding Constraints
class BasicSliderCrank : public Test
{
public:
	BasicSliderCrank()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
            bd.position.Set(0.0f, 17.0f);
			ground = m_world->CreateBody(&bd);
		}
        
		{
			b2Body* prevBody = ground;
            
			// Define crank.
			{
				b2PolygonShape shape;
				shape.SetAsBox(4.0f, 1.0f);
                
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-8.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);
                
				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(-12.0f, 20.0f));
				m_world->CreateJoint(&rjd);
                
				prevBody = body;
			}
            
			// Define connecting rod
			{
				b2PolygonShape shape;
				shape.SetAsBox(8.0f, 1.0f);
                
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(4.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);
                
				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(-4.0f, 20.0f));
				m_world->CreateJoint(&rjd);
                
				prevBody = body;
			}
            
			// Define piston
			{
				b2PolygonShape shape;
				shape.SetAsBox(3.0f, 3.0f);
                
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.fixedRotation = true;
				bd.position.Set(12.0f, 20.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0f);
                
				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(12.0f, 20.0f));
				m_world->CreateJoint(&rjd);
                
				b2PrismaticJointDef pjd;
				pjd.Initialize(ground, body, b2Vec2(12.0f, 17.0f), b2Vec2(1.0f, 0.0f));
				m_world->CreateJoint(&pjd);
			}
  		}
	}
    
	static Test* Create()
	{
		return new BasicSliderCrank;
	}
};

#endif
