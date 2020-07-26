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

#include "settings.h"
#include "test.h"
#include "imgui/imgui.h"

class RevoluteJoint : public Test
{
public:
	RevoluteJoint()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));

			b2FixtureDef fd;
			fd.shape = &shape;
			//fd.filter.categoryBits = 2;

			ground->CreateFixture(&fd);
		}

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 1.0f;

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 3.0f, b2Vec2(0.0f, 3.0f), 0.0f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-10.0f, 20.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0f);

			b2RevoluteJointDef jd;
			jd.Initialize(ground, body, b2Vec2(-10.0f, 20.5f));
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 10000.0f;
			jd.enableMotor = m_enableMotor;
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.5f * b2_pi;
			jd.enableLimit = m_enableLimit;

			m_joint1 = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		{
			b2CircleShape circle_shape;
			circle_shape.m_radius = 2.0f;

			b2BodyDef circle_bd;
			circle_bd.type = b2_dynamicBody;
			circle_bd.position.Set(5.0f, 30.0f);

			b2FixtureDef fd;
			fd.density = 5.0f;
			fd.filter.maskBits = 1;
			fd.shape = &circle_shape;

			m_ball = m_world->CreateBody(&circle_bd);
			m_ball->CreateFixture(&fd);

			b2PolygonShape polygon_shape;
			polygon_shape.SetAsBox(10.0f, 0.5f, b2Vec2 (-10.0f, 0.0f), 0.0f);

			b2BodyDef polygon_bd;
			polygon_bd.position.Set(20.0f, 10.0f);
			polygon_bd.type = b2_dynamicBody;
			polygon_bd.bullet = true;
			b2Body* polygon_body = m_world->CreateBody(&polygon_bd);
			polygon_body->CreateFixture(&polygon_shape, 2.0f);

			b2RevoluteJointDef jd;
			jd.Initialize(ground, polygon_body, b2Vec2(19.0f, 10.0f));
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.0f * b2_pi;
			jd.enableLimit = true;
			jd.enableMotor = true;
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10000.0f;

			m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			m_joint1->EnableLimit(m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			m_joint1->EnableMotor(m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
			m_joint1->SetMotorSpeed(m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		
		float torque1 = m_joint1->GetMotorTorque(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque 1= %4.0f", torque1);
		m_textLine += m_textIncrement;

		float torque2 = m_joint2->GetMotorTorque(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque 2= %4.0f", torque2);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new RevoluteJoint;
	}

	b2Body* m_ball;
	b2RevoluteJoint* m_joint1;
	b2RevoluteJoint* m_joint2;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int testIndex = RegisterTest("Joints", "Revolute", RevoluteJoint::Create);
