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

class ShapeEditing : public Test
{
public:

	ShapeEditing()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(0.0f, 10.0f);
		m_body = m_world->CreateBody(&bd);

		b2PolygonShape shape;
		shape.SetAsBox(4.0f, 4.0f, b2Vec2(0.0f, 0.0f), 0.0f);
		m_fixture1 = m_body->CreateFixture(&shape, 10.0f);

		m_fixture2 = NULL;

		m_sensor = false;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_C:
			if (m_fixture2 == NULL)
			{
				b2CircleShape shape;
				shape.m_radius = 3.0f;
				shape.m_p.Set(0.5f, -4.0f);
				m_fixture2 = m_body->CreateFixture(&shape, 10.0f);
				m_body->SetAwake(true);
			}
			break;

		case GLFW_KEY_D:
			if (m_fixture2 != NULL)
			{
				m_body->DestroyFixture(m_fixture2);
				m_fixture2 = NULL;
				m_body->SetAwake(true);
			}
			break;

		case GLFW_KEY_S:
			if (m_fixture2 != NULL)
			{
				m_sensor = !m_sensor;
				m_fixture2->SetSensor(m_sensor);
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "sensor = %d", m_sensor);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new ShapeEditing;
	}

	b2Body* m_body;
	b2Fixture* m_fixture1;
	b2Fixture* m_fixture2;
	bool m_sensor;
};

static int testIndex = RegisterTest("Examples", "Shape Editing", ShapeEditing::Create);
