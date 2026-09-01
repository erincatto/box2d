#include "sample.h"

#include "box2d/box2d.h"

// This simulations stresses the solver by putting a light mass between two bodies on a prismatic joint with a stiff spring.
// This can be made stable by increasing the size of the middle circle and/or increasing the number of sub-steps.
// todo try applying the impulse at the mid point to avoid breaking Newton's 3rd law
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

// This configuration has stability problems.
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
		//wjdef.base.constraintHertz = 30.0f;
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
