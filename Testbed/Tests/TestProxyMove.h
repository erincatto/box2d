#ifndef TEST_PROXY_MOVE_H
#define TEST_PROXY_MOVE_H

class TestProxyMove : public Test
{
public:
	TestProxyMove()
	{
		{
			b2BodyDef bd;
			b2Body* body = m_world->CreateBody(&bd);
			b2ChainShape shape;
			const b2Vec2 vertices[4] = {
				b2Vec2(-20.0f, 0.0f),
				b2Vec2(20.0f, 0.0f),
				b2Vec2(20.0f, 40.0f),
				b2Vec2(-20.0f, 40.0f)
			};
			shape.CreateLoop(vertices, 4);
			body->CreateFixture(&shape, 0.0f);
		}

		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.linearVelocity.Set(10.0f, 10.0f);
			b2Body *body = m_world->CreateBody(&bd);
			m_ball = body;
			b2FixtureDef fd;
			fd.density = 0.5f;
			fd.friction = 0.0f;
			fd.restitution = 1.0f;
			b2CircleShape shape;
			fd.shape = &shape;
			shape.m_radius = 2.0f;
			shape.m_p.Set(0.0f, 20.0f);
			body->CreateFixture(&fd);
		}
	}

	virtual void Step(Settings *settings)
	{
		if (!settings->pause || settings->singleStep) {
			m_xf0 = m_ball->GetTransform();
			m_wc0 = m_ball->GetWorldCenter();
		}

		settings->drawAABBs = true;
		settings->drawCOMs = true;

		m_ball->ApplyAngularImpulse(2 * b2_pi - m_ball->GetAngularVelocity(), true);

		Test::Step(settings);

		b2Transform xf0 = m_xf0;
		b2Transform xf1 = m_ball->GetTransform();
		g_debugDraw.DrawSolidCircle(xf0.p, 0.5, xf0.q.GetXAxis(), b2Color(0.5f, 0.5f, 0.0f));
		g_debugDraw.DrawSegment(xf0.p, xf1.p, b2Color(0.5f, 0.5f, 0.0f));
		g_debugDraw.DrawSolidCircle(xf1.p, 0.5, xf1.q.GetXAxis(), b2Color(1.0f, 1.0f, 0.0f));

		b2Vec2 wc0 = m_wc0;
		b2Vec2 wc1 = m_ball->GetWorldCenter();
		g_debugDraw.DrawCircle(wc0, 0.5, b2Color(0.0f, 0.5f, 0.5f));
		g_debugDraw.DrawSegment(wc0, wc1, b2Color(0.0f, 0.5f, 0.5f));
		g_debugDraw.DrawCircle(wc1, 0.5, b2Color(0.0f, 1.0f, 1.0f));

		g_debugDraw.Flush();
	}

	static Test* Create()
	{
		return new TestProxyMove;
	}

private:
	b2Body* m_ball;
	b2Transform m_xf0;
	b2Vec2 m_wc0;
};

#endif
