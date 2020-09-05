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

// Test the wheel joint with motor, spring, and limit options.
class WheelJoint : public Test
{
public:
	WheelJoint()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.SetTwoSided(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 10.0f;

		{
			b2CircleShape shape;
			shape.m_radius = 2.0f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 10.0f);
			bd.allowSleep = false;
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0f);

			b2WheelJointDef jd;

			// Horizontal
			jd.Initialize(ground, body, bd.position, b2Vec2(0.0f, 1.0f));

			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 10000.0f;
			jd.enableMotor = m_enableMotor;
			jd.lowerTranslation = -3.0f;
			jd.upperTranslation = 3.0f;
			jd.enableLimit = m_enableLimit;

			float hertz = 1.0f;
			float dampingRatio = 0.7f;
			b2LinearStiffness(jd.stiffness, jd.damping, hertz, dampingRatio, ground, body);

			m_joint = (b2WheelJoint*)m_world->CreateJoint(&jd);
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		float torque = m_joint->GetMotorTorque(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f", torque);
		m_textLine += m_textIncrement;

		b2Vec2 F = m_joint->GetReactionForce(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Reaction Force = (%4.1f, %4.1f)", F.x, F.y);
		m_textLine += m_textIncrement;
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			m_joint->EnableLimit(m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			m_joint->EnableMotor(m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -100.0f, 100.0f, "%.0f"))
		{
			m_joint->SetMotorSpeed(m_motorSpeed);
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new WheelJoint;
	}

	b2WheelJoint* m_joint;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int testIndex = RegisterTest("Joints", "Wheel", WheelJoint::Create);
