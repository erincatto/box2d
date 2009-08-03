/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef CCD_TEST_H
#define CCD_TEST_H

class CCDTest : public Test
{
public:

	CCDTest()
	{
#if 1
		{
			b2PolygonShape shape;
			shape.SetAsBox(10.0f, 0.2f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;

			b2BodyDef bd;
			bd.position.Set(0.0f, -0.2f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);

			shape.SetAsBox(0.2f, 1.0f, b2Vec2(0.5f, 1.2f), 0.0f);
			body->CreateFixture(&fd);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(2.0f, 0.1f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.restitution = 0.0f;

			m_angularVelocity = RandomFloat(-50.0f, 50.0f);
			m_angularVelocity = -30.669577f;

			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
			body->SetMassFromShapes();
			body->SetLinearVelocity(b2Vec2(0.0f, -100.0f));
			body->SetAngularVelocity(m_angularVelocity);
		}
#elif 0
		{
			b2FixtureDef fd;
			fd.SetAsBox(10.0f, 0.1f);
			fd.density = 0.0f;

			b2BodyDef bd;
			bd.type = b2BodyDef::e_static;
			bd.position.Set(0.0f, -0.2f);
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&fd);
		}

		{
			b2FixtureDef fd;
			fd.SetAsBox(2.0f, 0.1f);
			fd.density = 1.0f;
			fd.restitution = 0.0f;

			b2BodyDef bd1;
			bd1.type = b2BodyDef::e_dynamic;
			bd1.isBullet = true;
			bd1.allowSleep = false;
			bd1.position.Set(0.0f, 20.0f);
			b2Body* b1 = m_world->Create(&bd1);
			b1->CreateFixture(&fd);
			b1->SetMassFromShapes();
			b1->SetLinearVelocity(b2Vec2(0.0f, -100.0f));

			fd.SetAsBox(1.0f, 0.1f);
			b2BodyDef bd2;
			bd2.type = b2BodyDef::e_dynamic;
			bd2.isBullet = true;
			bd2.allowSleep = false;
			bd2.position.Set(0.0f, 20.2f);
			b2Body* b2 = m_world->Create(&bd2);
			b2->CreateFixture(&fd);
			b2->SetMassFromShapes();
			b2->SetLinearVelocity(b2Vec2(0.0f, -100.0f));

			fd.SetAsBox(0.25f, 0.25f);
			fd.density = 10.0f;
			b2BodyDef bd3;
			bd3.type = b2BodyDef::e_dynamic;
			bd3.isBullet = true;
			bd3.allowSleep = false;
			bd3.position.Set(0.0f, 100.0f);
			b2Body* b3 = m_world->Create(&bd3);
			b3->CreateFixture(&fd);
			b3->SetMassFromShapes();
			b3->SetLinearVelocity(b2Vec2(0.0f, -150.0f));
		}
#else
		const float32 k_restitution = 1.4f;

		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			b2Body* body = m_world->CreateBody(&bd);

			b2FixtureDef fd;
			fd.density = 0.0f;
			fd.restitution = k_restitution;

			fd.SetAsBox(0.1f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0f);
			body->CreateFixture(&fd);

			fd.SetAsBox(0.1f, 10.0f, b2Vec2(10.0f, 0.0f), 0.0f);
			body->CreateFixture(&fd);

			fd.SetAsBox(0.1f, 10.0f, b2Vec2(0.0f, -10.0f), 0.5f * b2_pi);
			body->CreateFixture(&fd);

			fd.SetAsBox(0.1f, 10.0f, b2Vec2(0.0f, 10.0f), -0.5f * b2_pi);
			body->CreateFixture(&fd);
		}

#if 0
		{
			b2FixtureDef sd_bottom;
			sd_bottom.SetAsBox(1.0f, 0.1f, b2Vec2(0.0f, -1.0f), 0.0f);
			sd_bottom.density = 4.0f;

			b2FixtureDef sd_top;
			sd_top.SetAsBox(1.0f, 0.1f, b2Vec2(0.0f,  1.0f), 0.0f);
			sd_top.density = 4.0f;

			b2FixtureDef sd_left;
			sd_left.SetAsBox(0.1f, 1.0f, b2Vec2(-1.0f, 0.0f), 0.0f);
			sd_left.density = 4.0f;

			b2FixtureDef sd_right;
			sd_right.SetAsBox(0.1f, 1.0f, b2Vec2(1.0f, 0.0f), 0.0f);
			sd_right.density = 4.0f;

			b2BodyDef bd;
			bd.type = b2BodyDef::e_dynamicBody;
			bd.position.Set(0.0f, 15.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&sd_bottom);
			body->CreateFixture(&sd_top);
			body->CreateFixture(&sd_left);
			body->CreateFixture(&sd_right);
			body->SetMassFromShapes();
		}
#elif 0
		{
			b2FixtureDef sd_bottom;
			sd_bottom.SetAsBox( 1.5f, 0.15f );
			sd_bottom.density = 4.0f;

			b2FixtureDef sd_left;
			sd_left.SetAsBox(0.15f, 2.7f, b2Vec2(-1.45f, 2.35f), 0.2f);
			sd_left.density = 4.0f;

			b2FixtureDef sd_right;
			sd_right.SetAsBox(0.15f, 2.7f, b2Vec2(1.45f, 2.35f), -0.2f);
			sd_right.density = 4.0f;

			b2BodyDef bd;
			bd.position.Set( 0.0f, 15.0f );
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&sd_bottom);
			body->CreateFixture(&sd_left);
			body->CreateFixture(&sd_right);
			body->SetMassFromShapes();
		}
#else
		{
			b2BodyDef bd;
			bd.position.Set(-5.0f, 20.0f);
			bd.isBullet = true;
			b2Body* body = m_world->CreateBody(&bd);
			body->SetAngularVelocity(RandomFloat(-50.0f, 50.0f));

			b2FixtureDef fd;
			fd.SetAsBox(0.1f, 4.0f);
			fd.density = 1.0f;
			fd.restitution = 0.0f;
			body->CreateFixture(&fd);
			body->SetMassFromShapes();
		}
#endif

		for (int32 i = 0; i < 0; ++i)
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 15.0f + i);
			bd.isBullet = true;
			b2Body* body = m_world->CreateBody(&bd);
			body->SetAngularVelocity(RandomFloat(-50.0f, 50.0f));

			b2FixtureDef fd;
			fd.radius = 0.25f;
			fd.density = 1.0f;
			fd.restitution = 0.0f;
			body->CreateFixture(&fd);
			body->SetMassFromShapes();
		}
#endif
	}

	void Step(Settings* settings)
	{
		if (m_stepCount	== 10)
		{
			m_stepCount += 0;
		}

		Test::Step(settings);

		extern int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

		if (b2_gjkCalls > 0)
		{
			m_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				b2_gjkCalls, b2_gjkIters / float32(b2_gjkCalls), b2_gjkMaxIters);
			m_textLine += 15;
		}

		extern int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
		extern int32 b2_toiRootIters, b2_toiMaxRootIters;

		if (b2_toiCalls > 0)
		{
			m_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
								b2_toiCalls, b2_toiIters / float32(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += 15;
			m_debugDraw.DrawString(5, m_textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
				b2_toiRootIters / float32(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += 15;
		}
	}

	static Test* Create()
	{
		return new CCDTest;
	}

	float32 m_angularVelocity;
};

#endif
