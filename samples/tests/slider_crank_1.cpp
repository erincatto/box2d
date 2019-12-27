// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "test.h"

// A basic slider crank created for GDC tutorial: Understanding Constraints
class SliderCrank1 : public Test
{
public:
	SliderCrank1()
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
		return new SliderCrank1;
	}
};

static int testIndex = RegisterTest("Examples", "Slider Crank 1", SliderCrank1::Create);
