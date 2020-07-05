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
#include "imgui/imgui.h"

class CompoundShapes : public Test
{
public:
	CompoundShapes()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 0.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(50.0f, 0.0f), b2Vec2(-50.0f, 0.0f));

			body->CreateFixture(&shape, 0.0f);
		}

		// Table 1
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-15.0f, 1.0f);
			m_table1 = m_world->CreateBody(&bd);

			b2PolygonShape top;
			top.SetAsBox(3.0f, 0.5f, b2Vec2(0.0f, 3.5f), 0.0f);

			b2PolygonShape leftLeg;
			leftLeg.SetAsBox(0.5f, 1.5f, b2Vec2(-2.5f, 1.5f), 0.0f);

			b2PolygonShape rightLeg;
			rightLeg.SetAsBox(0.5f, 1.5f, b2Vec2(2.5f, 1.5f), 0.0f);

			m_table1->CreateFixture(&top, 2.0f);
			m_table1->CreateFixture(&leftLeg, 2.0f);
			m_table1->CreateFixture(&rightLeg, 2.0f);
		}

		// Table 2
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-5.0f, 1.0f);
			m_table2 = m_world->CreateBody(&bd);

			b2PolygonShape top;
			top.SetAsBox(3.0f, 0.5f, b2Vec2(0.0f, 3.5f), 0.0f);

			b2PolygonShape leftLeg;
			leftLeg.SetAsBox(0.5f, 2.0f, b2Vec2(-2.5f, 2.0f), 0.0f);

			b2PolygonShape rightLeg;
			rightLeg.SetAsBox(0.5f, 2.0f, b2Vec2(2.5f, 2.0f), 0.0f);

			m_table2->CreateFixture(&top, 2.0f);
			m_table2->CreateFixture(&leftLeg, 2.0f);
			m_table2->CreateFixture(&rightLeg, 2.0f);
		}

		// Spaceship 1
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(5.0f, 1.0f);
			m_ship1 = m_world->CreateBody(&bd);

			b2Vec2 vertices[3];

			b2PolygonShape left;
			vertices[0].Set(-2.0f, 0.0f);
			vertices[1].Set(0.0f, 4.0f / 3.0f);
			vertices[2].Set(0.0f, 4.0f);
			left.Set(vertices, 3);

			b2PolygonShape right;
			vertices[0].Set(2.0f, 0.0f);
			vertices[1].Set(0.0f, 4.0f / 3.0f);
			vertices[2].Set(0.0f, 4.0f);
			right.Set(vertices, 3);

			m_ship1->CreateFixture(&left, 2.0f);
			m_ship1->CreateFixture(&right, 2.0f);
		}

		// Spaceship 2
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(15.0f, 1.0f);
			m_ship2 = m_world->CreateBody(&bd);

			b2Vec2 vertices[3];

			b2PolygonShape left;
			vertices[0].Set(-2.0f, 0.0f);
			vertices[1].Set(1.0f, 2.0f);
			vertices[2].Set(0.0f, 4.0f);
			left.Set(vertices, 3);

			b2PolygonShape right;
			vertices[0].Set(2.0f, 0.0f);
			vertices[1].Set(-1.0f, 2.0f);
			vertices[2].Set(0.0f, 4.0f);
			right.Set(vertices, 3);

			m_ship2->CreateFixture(&left, 2.0f);
			m_ship2->CreateFixture(&right, 2.0f);
		}
	}

	void Spawn()
	{
		// Table 1 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_table1->GetPosition();
			bd.angle = m_table1->GetAngle();

			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(4.0f, 0.1f, b2Vec2(0.0f, 3.0f), 0.0f);
			
			body->CreateFixture(&box, 2.0f);
		}

		// Table 2 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_table2->GetPosition();
			bd.angle = m_table2->GetAngle();

			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(4.0f, 0.1f, b2Vec2(0.0f, 3.0f), 0.0f);
			
			body->CreateFixture(&box, 2.0f);
		}

		// Ship 1 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_ship1->GetPosition();
			bd.angle = m_ship1->GetAngle();
			bd.gravityScale = 0.0f;

			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape circle;
			circle.m_radius = 0.5f;
			circle.m_p.Set(0.0f, 2.0f);

			body->CreateFixture(&circle, 2.0f);
		}

		// Ship 2 obstruction
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position = m_ship2->GetPosition();
			bd.angle = m_ship2->GetAngle();
			bd.gravityScale = 0.0f;

			b2Body* body = m_world->CreateBody(&bd);

			b2CircleShape circle;
			circle.m_radius = 0.5f;
			circle.m_p.Set(0.0f, 2.0f);

			body->CreateFixture(&circle, 2.0f);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Spawn"))
		{
			Spawn();
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}

	b2Body* m_table1;
	b2Body* m_table2;
	b2Body* m_ship1;
	b2Body* m_ship2;
};

static int testIndex = RegisterTest("Examples", "Compound Shapes", CompoundShapes::Create);
