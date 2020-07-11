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

class Heavy2 : public Test
{
public:
    
    Heavy2()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);
            
			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}
        
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(0.0f, 2.5f);
		b2Body* body = m_world->CreateBody(&bd);
        
		b2CircleShape shape;
		shape.m_radius = 0.5f;
        body->CreateFixture(&shape, 10.0f);
        
        bd.position.Set(0.0f, 3.5f);
        body = m_world->CreateBody(&bd);
        body->CreateFixture(&shape, 10.0f);
        
        m_heavy = NULL;
	}
    
    void ToggleHeavy()
    {
        if (m_heavy)
        {
            m_world->DestroyBody(m_heavy);
            m_heavy = NULL;
        }
        else
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(0.0f, 9.0f);
            m_heavy = m_world->CreateBody(&bd);
            
            b2CircleShape shape;
            shape.m_radius = 5.0f;
            m_heavy->CreateFixture(&shape, 10.0f);
        }
    }
    
	void Keyboard(int key) override
	{
		switch (key)
		{
        case GLFW_KEY_H:
            ToggleHeavy();
            break;
		}
	}
    
	static Test* Create()
	{
		return new Heavy2;
	}
    
	b2Body* m_heavy;
};

static int testIndex = RegisterTest("Solver", "Heavy 2", Heavy2::Create);
