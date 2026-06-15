#include "GLFW/glfw3.h"
#include "imgui.h"
#include "sample.h"

#include "box2d/box2d.h"

class BadSteiner : public Sample
{
public:
	explicit BadSteiner( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 2.5f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -100.0f, 0.0f }, { 100.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -48.0f, 62.0f };
			b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();

			b2Vec2 points[3] = {
				{ 48.7599983f, -60.5699997f },
				{ 48.7400017f, -60.5400009f },
				{ 48.6800003f, -60.5600014f },
			};

			b2Hull hull = b2ComputeHull( points, 3 );
			b2Polygon poly = b2MakePolygon( &hull, 0.0f );
			b2CreatePolygonShape( bodyId, &shapeDef, &poly );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new BadSteiner( context );
	}
};

static int sampleBadSteiner = RegisterSample( "Issues", "Bad Steiner", BadSteiner::Create );

class DisableCrash : public Sample
{
public:
	explicit DisableCrash( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.8f, 6.4f };
			m_context->camera.zoom = 25.0f * 0.4f;
		}

		m_isEnabled = true;

		// Define attachment
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -2.0f, 3.0f };
			bodyDef.isEnabled = m_isEnabled;
			m_attachmentId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.5f, 2.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_attachmentId, &shapeDef, &box );
		}

		// Define platform
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { -4.0f, 5.0f };
			m_platformId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 0.5f, 4.0f, { 4.0f, 0.0f }, b2MakeRot( 0.5f * B2_PI ) );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( m_platformId, &shapeDef, &box );

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
			b2Position pivot = { -2.0f, 5.0f };
			revoluteDef.base.bodyIdA = m_attachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.base.localFrameB.p = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::Checkbox( "Enable", &m_isEnabled ) )
		{
			if ( m_isEnabled )
			{
				b2Body_Enable( m_attachmentId );
			}
			else
			{
				b2Body_Disable( m_attachmentId );
			}
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new DisableCrash( context );
	}

	b2BodyId m_attachmentId;
	b2BodyId m_platformId;
	bool m_isEnabled;
};

static int sampleDisableCrash = RegisterSample( "Issues", "Disable", DisableCrash::Create );

class Crash01 : public Sample
{
public:
	explicit Crash01( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.8f, 6.4f };
			m_context->camera.zoom = 25.0f * 0.4f;
		}

		m_type = b2_dynamicBody;
		m_isEnabled = true;

		b2BodyId groundId = b2_nullBodyId;
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.name = "ground";
			groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Segment segment = { { -20.0f, 0.0f }, { 20.0f, 0.0f } };
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		// Define attachment
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = b2_dynamicBody;
			bodyDef.position = { -2.0f, 3.0f };
			bodyDef.name = "attach1";
			m_attachmentId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 0.5f, 2.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 1.0f;
			b2CreatePolygonShape( m_attachmentId, &shapeDef, &box );
		}

		// Define platform
		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.type = m_type;
			bodyDef.isEnabled = m_isEnabled;
			bodyDef.position = { -4.0f, 5.0f };
			bodyDef.name = "platform";
			m_platformId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeOffsetBox( 0.5f, 4.0f, { 4.0f, 0.0f }, b2MakeRot( 0.5f * B2_PI ) );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			shapeDef.density = 2.0f;
			b2CreatePolygonShape( m_platformId, &shapeDef, &box );

			b2RevoluteJointDef revoluteDef = b2DefaultRevoluteJointDef();
			b2Position pivot = { -2.0f, 5.0f };
			revoluteDef.base.bodyIdA = m_attachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.base.localFrameB.p = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
			b2Position anchor = { 0.0f, 5.0f };
			prismaticDef.base.bodyIdA = groundId;
			prismaticDef.base.bodyIdB = m_platformId;
			prismaticDef.base.localFrameA.p = b2Body_GetLocalPoint( groundId, anchor );
			prismaticDef.base.localFrameB.p = b2Body_GetLocalPoint( m_platformId, anchor );
			prismaticDef.maxMotorForce = 1000.0f;
			prismaticDef.motorSpeed = 0.0f;
			prismaticDef.enableMotor = true;
			prismaticDef.lowerTranslation = -10.0f;
			prismaticDef.upperTranslation = 10.0f;
			prismaticDef.enableLimit = true;

			b2CreatePrismaticJoint( m_worldId, &prismaticDef );
		}
	}

	bool DrawControls() override
	{
		if ( ImGui::RadioButton( "Static", m_type == b2_staticBody ) )
		{
			m_type = b2_staticBody;
			b2Body_SetType( m_platformId, b2_staticBody );
		}

		if ( ImGui::RadioButton( "Kinematic", m_type == b2_kinematicBody ) )
		{
			m_type = b2_kinematicBody;
			b2Body_SetType( m_platformId, b2_kinematicBody );
			b2Body_SetLinearVelocity( m_platformId, { -0.1f, 0.0f } );
		}

		if ( ImGui::RadioButton( "Dynamic", m_type == b2_dynamicBody ) )
		{
			m_type = b2_dynamicBody;
			b2Body_SetType( m_platformId, b2_dynamicBody );
		}

		if ( ImGui::Checkbox( "Enable", &m_isEnabled ) )
		{
			if ( m_isEnabled )
			{
				b2Body_Enable( m_attachmentId );
			}
			else
			{
				b2Body_Disable( m_attachmentId );
			}
		}

		return true;
	}

	static Sample* Create( SampleContext* context )
	{
		return new Crash01( context );
	}

	b2BodyId m_attachmentId;
	b2BodyId m_platformId;
	b2BodyType m_type;
	bool m_isEnabled;
};

static int sampleBodyType = RegisterSample( "Issues", "Crash01", Crash01::Create );

class StaticVsBulletBug : public Sample
{
public:
	explicit StaticVsBulletBug( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 48.8525391, 68.1518555 };
			m_context->camera.zoom = 100.0f * 0.5f;
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody; // NOTE(bug): Changing this to b2_staticBody fixes the issue
			b2BodyId staticBodyId = b2CreateBody( m_worldId, &bd );

			const b2Vec2 verts[] = {
				{ 48.8525391, 68.1518555 }, { 49.1821289, 68.1152344 }, { 68.8476562, 68.1152344 },
				{ 68.8476562, 70.2392578 }, { 48.8525391, 70.2392578 },
			};

			const b2Hull hull = b2ComputeHull( verts, ARRAY_COUNT( verts ) );
			const b2Polygon poly = b2MakePolygon( &hull, 0.0f );

			b2ShapeDef sd = b2DefaultShapeDef();
			sd.density = 1.0f;
			sd.material.friction = 0.5f;
			sd.material.restitution = 0.1f;

			b2CreatePolygonShape( staticBodyId, &sd, &poly );
			b2Body_SetType( staticBodyId, b2_staticBody );
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.position = { 58.9243050, 77.5401459 };
			bd.type = b2_dynamicBody;
			bd.motionLocks.angularZ = true;
			bd.linearVelocity = { 104.868881, -281.073883 };
			bd.isBullet = true;

			b2BodyId ballBodyId = b2CreateBody( m_worldId, &bd );
			const b2Circle ball = { .center = {}, .radius = 0.3f };

			b2ShapeDef ballShape = b2DefaultShapeDef();
			ballShape.density = 3.0f;
			ballShape.material.friction = 0.2f;
			ballShape.material.restitution = 0.9f;

			b2CreateCircleShape( ballBodyId, &ballShape, &ball );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new StaticVsBulletBug( context );
	}
};

static int staticVsBulletBug = RegisterSample( "Issues", "StaticVsBulletBug", StaticVsBulletBug::Create );

// This simulations stresses the solver by putting a light mass between two bodies on a prismatic joint with a stiff spring.
// This can be made stable by increasing the size of the middle circle and/or increasing the number of sub-steps.
class UnstablePrismaticJoints : public Sample
{
public:
	explicit UnstablePrismaticJoints( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 32.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -100.0f, 0.0f }, { 100.0f, 0.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyId centerId;
		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { 0, 3 };
			centerId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };

			// Note: this will crash due to divergence (inf/nan) with a radius of 0.1
			// circle.radius = 0.1f;
			circle.radius = 0.5f;

			b2CreateCircleShape( centerId, &sd, &circle );
		}

		b2PrismaticJointDef jd = b2DefaultPrismaticJointDef();
		jd.enableSpring = true;
		jd.hertz = 10.0f;
		jd.dampingRatio = 2.0f;

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { -3.5, 3 };

			b2BodyId leftId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };
			circle.radius = 2.0f;
			b2CreateCircleShape( leftId, &sd, &circle );

			jd.base.bodyIdA = centerId;
			jd.base.bodyIdB = leftId;
			jd.targetTranslation = -3.0f;
			b2CreatePrismaticJoint( m_worldId, &jd );
		}

		{
			b2BodyDef bd = b2DefaultBodyDef();
			bd.type = b2_dynamicBody;
			bd.position = { 3.5, 3 };
			b2BodyId rightId = b2CreateBody( m_worldId, &bd );

			b2ShapeDef sd = b2DefaultShapeDef();

			b2Circle circle;
			circle.center = { 0, 0 };
			circle.radius = 2.0f;

			b2CreateCircleShape( rightId, &sd, &circle );

			jd.base.bodyIdA = centerId;
			jd.base.bodyIdB = rightId;
			jd.targetTranslation = 3.0f;
			b2CreatePrismaticJoint( m_worldId, &jd );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new UnstablePrismaticJoints( context );
	}
};

static int sampleUnstablePrismaticJoints =
	RegisterSample( "Issues", "Unstable Prismatic Joints", UnstablePrismaticJoints::Create );

class UnstableWindmill : public Sample
{
public:
	explicit UnstableWindmill( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.center = { 0.0f, 1.75f };
			m_context->camera.zoom = 32.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2Segment segment = { { -100.0f, -10.0f }, { 100.0f, -10.0f } };
			b2CreateSegmentShape( groundId, &shapeDef, &segment );
		}

		b2BodyDef bdef = b2DefaultBodyDef();
		bdef.gravityScale = 0.0f;
		bdef.type = b2_dynamicBody;
		b2ShapeDef sdef = b2DefaultShapeDef();
		sdef.material = b2DefaultSurfaceMaterial();
		sdef.material.friction = 0.1f;

		// center
		bdef.position = { 10, 10 };
		b2BodyId center = b2CreateBody( m_worldId, &bdef );
		b2Circle circle = { .center = { 0, 0 }, .radius = 5 };
		b2CreateCircleShape( center, &sdef, &circle );

		// rotors
		b2WeldJointDef wjdef = b2DefaultWeldJointDef();

		// This simulation can be stabilized by using a lower constraint stiffness
		wjdef.base.constraintHertz = 30.0f;
		wjdef.base.bodyIdA = center;

		b2Polygon polygon;

		bdef.position = { 10, 0 };
		b2BodyId body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 4, 5 ) ) );
		wjdef.base.localFrameA = { .p = { 0, -5 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 0, 5 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 20, 10 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 5, 4 ) ) );
		wjdef.base.localFrameA = { .p = { 5, 0 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { -5, 0 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 10, 20 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 4, 5 ) ) );
		wjdef.base.localFrameA = { .p = { 0, 5 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 0, -5 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );

		bdef.position = { 0, 10 };
		body = b2CreateBody( m_worldId, &bdef );
		b2CreatePolygonShape( body, &sdef, &( polygon = b2MakeBox( 5, 4 ) ) );
		wjdef.base.localFrameA = { .p = { -5, 0 }, .q = b2Rot_identity };
		wjdef.base.bodyIdB = body;
		wjdef.base.localFrameB = { .p = { 5, 0 }, .q = b2Rot_identity };
		b2CreateWeldJoint( m_worldId, &wjdef );
	}

	static Sample* Create( SampleContext* context )
	{
		return new UnstableWindmill( context );
	}
};

static int sampleUnstableWindmill = RegisterSample( "Issues", "Unstable Windmill", UnstableWindmill::Create );
