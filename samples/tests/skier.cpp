/*
Test case for collision/jerking issue.
*/

#include "test.h"

#include <vector>
#include <iostream>

class Skier : public Test
{
public:
	Skier()
	{		
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			float const PlatformWidth = 8.0f;

			/*
			First angle is from the horizontal and should be negative for a downward slope.
			Second angle is relative to the preceding slope, and should be positive, creating a kind of
			loose 'Z'-shape from the 3 edges.
			If A1 = -10, then A2 <= ~1.5 will result in the collision glitch.
			If A1 = -30, then A2 <= ~10.0 will result in the glitch.
			*/
			float const Angle1Degrees = -30.0f;
			float const Angle2Degrees = 10.0f;
			
			/*
			The larger the value of SlopeLength, the less likely the glitch will show up.
			*/
			float const SlopeLength = 2.0f;

			float const SurfaceFriction = 0.2f;

			// Convert to radians
			float const Slope1Incline = -Angle1Degrees * b2_pi / 180.0f;
			float const Slope2Incline = Slope1Incline - Angle2Degrees * b2_pi / 180.0f;
			//

			m_platform_width = PlatformWidth;

			std::vector< b2Vec2 > verts;

			// Horizontal platform
			verts.emplace_back(-PlatformWidth, 0.0f);
			verts.emplace_back(0.0f, 0.0f);

			// Slope
			verts.emplace_back(
				verts.back().x + SlopeLength * cosf(Slope1Incline),
				verts.back().y - SlopeLength * sinf(Slope1Incline)
				);

			verts.emplace_back(
				verts.back().x + SlopeLength * cosf(Slope2Incline),
				verts.back().y - SlopeLength * sinf(Slope2Incline)
				);

			{
				b2EdgeShape shape;
				shape.Set(verts[0], verts[1]);
				shape.m_hasVertex3 = true;
				shape.m_vertex3 = verts[2];

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 0.0f;
				fd.friction = SurfaceFriction;

				ground->CreateFixture(&fd);
			}

			{
				b2EdgeShape shape;
				shape.Set(verts[1], verts[2]);
				shape.m_hasVertex0 = true;
				shape.m_hasVertex3 = true;
				shape.m_vertex0 = verts[0];
				shape.m_vertex3 = verts[3];

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 0.0f;
				fd.friction = SurfaceFriction;

				ground->CreateFixture(&fd);
			}
			
			{
				b2EdgeShape shape;
				shape.Set(verts[2], verts[3]);
				shape.m_hasVertex0 = true;
				shape.m_vertex0 = verts[1];

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 0.0f;
				fd.friction = SurfaceFriction;

				ground->CreateFixture(&fd);
			}
		}

		{
			bool const EnableCircularSkiTips = false;

			float const BodyWidth = 1.0f;
			float const BodyHeight = 2.5f;
			float const SkiLength = 3.0f;

			/*
			Larger values for this seem to alleviate the issue to some extent.
			*/
			float const SkiThickness = 0.3f;

			float const SkiFriction = 0.0f;
			float const SkiRestitution = 0.15f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			float initial_y = BodyHeight / 2 + SkiThickness;
			if(EnableCircularSkiTips)
			{
				initial_y += SkiThickness / 6;
			}
			bd.position.Set(-m_platform_width / 2, initial_y);

			b2Body* skier = m_world->CreateBody(&bd);

			b2PolygonShape body;
			body.SetAsBox(BodyWidth / 2, BodyHeight / 2);

			b2PolygonShape ski;
			std::vector< b2Vec2 > verts;
			verts.emplace_back(-SkiLength / 2 - SkiThickness, -BodyHeight / 2);
			verts.emplace_back(-SkiLength / 2, -BodyHeight / 2 - SkiThickness);
			verts.emplace_back(SkiLength / 2, -BodyHeight / 2 - SkiThickness);
			verts.emplace_back(SkiLength / 2 + SkiThickness, -BodyHeight / 2);
			ski.Set(verts.data(), (int32)verts.size());

			b2CircleShape ski_back_shape;
			ski_back_shape.m_p.Set(-SkiLength / 2.0f, -BodyHeight / 2 - SkiThickness * (2.0f / 3));
			ski_back_shape.m_radius = SkiThickness / 2;

			b2CircleShape ski_front_shape;
			ski_front_shape.m_p.Set(SkiLength / 2, -BodyHeight / 2 - SkiThickness * (2.0f / 3));
			ski_front_shape.m_radius = SkiThickness / 2;

			b2FixtureDef fd;
			fd.shape = &body;
			fd.density = 1.0f;
			skier->CreateFixture(&fd);

			fd.friction = SkiFriction;
			fd.restitution = SkiRestitution;

			fd.shape = &ski;
			skier->CreateFixture(&fd);

			if(EnableCircularSkiTips)
			{
				fd.shape = &ski_back_shape;
				skier->CreateFixture(&fd);

				fd.shape = &ski_front_shape;
				skier->CreateFixture(&fd);
			}

			skier->SetLinearVelocity(b2Vec2(0.5f, 0.0f));

			m_skier = skier;
		}

		g_camera.m_center = b2Vec2(m_platform_width / 2.0f, 0.0f);
		g_camera.m_zoom = 0.4f;
		m_fixed_camera = true;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
			case GLFW_KEY_C:
			m_fixed_camera = !m_fixed_camera;
			if(m_fixed_camera)
			{
				g_camera.m_center = b2Vec2(m_platform_width / 2.0f, 0.0f);
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: c = Camera fixed/tracking");
		m_textLine += m_textIncrement;

		if(!m_fixed_camera)
		{
			g_camera.m_center = m_skier->GetPosition();
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Skier;
	}

	b2Body* m_skier;
	float m_platform_width;
	bool m_fixed_camera;
};

static int testIndex = RegisterTest("Bugs", "Skier", Skier::Create);
