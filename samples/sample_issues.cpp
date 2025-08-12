#include "GLFW/glfw3.h"
#include "imgui.h"
#include "sample.h"

#include "box2d/box2d.h"

struct PhysicsHitQueryResult2D
{
	b2ShapeId shapeId = b2_nullShapeId;
	b2Vec2 point = b2Vec2_zero;
	b2Vec2 normal = b2Vec2_zero;
	b2Vec2 endPos = b2Vec2_zero;
	float fraction = 0.0f;
	bool startPenetrating = false;
	bool blockingHit = false;
};

struct CastContext_Single
{
	b2Vec2 startPos;
	b2Vec2 translation;
	PhysicsHitQueryResult2D result;
	bool hit = false;
};

static float b2CastResult_Closest( b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* c )
{
	CastContext_Single* context = static_cast<CastContext_Single*>( c );

	if ( b2Dot( context->translation, normal ) >= 0.0f )
		return -1.0f;

	context->result.shapeId = shapeId;
	context->result.point = point;
	context->result.normal = normal;
	context->result.endPos = context->startPos + context->translation * fraction;
	context->result.fraction = fraction;
	context->result.blockingHit = true;
	context->hit = true;

	return fraction;
}

static b2ShapeProxy TransformShapeProxy( const b2Transform& t, const b2ShapeProxy& proxy )
{
	b2ShapeProxy ret;
	ret.count = proxy.count;
	ret.radius = proxy.radius;

	for ( int i = 0; i < proxy.count; ++i )
	{
		ret.points[i] = b2TransformPoint( t, proxy.points[i] );
	}

	return ret;
}

// This showed a problem with shape casts hitting the back-side of a chain shape
class ShapeCastChain : public Sample
{
public:
	explicit ShapeCastChain( SampleContext* context )
		: Sample( context )
	{
		// World Body & Shape
		b2BodyDef worldBodyDef = b2DefaultBodyDef();
		const b2BodyId groundId = b2CreateBody( m_worldId, &worldBodyDef );

		b2Vec2 points[4] = { { 1.0f, 0.0f }, { -1.0f, 0.0f }, { -1.0f, -1.0f }, { 1.0f, -1.0f } };

		b2ChainDef worldChainDef = b2DefaultChainDef();
		worldChainDef.userData = nullptr;
		worldChainDef.points = points;
		worldChainDef.count = 4;
		worldChainDef.filter.categoryBits = 0x1;
		worldChainDef.filter.maskBits = 0x1;
		worldChainDef.filter.groupIndex = 0;
		worldChainDef.isLoop = true;
		worldChainDef.enableSensorEvents = false;
		b2CreateChain( groundId, &worldChainDef );

		// "Character" Body & Shape
		b2BodyDef characterBodyDef = b2DefaultBodyDef();
		characterBodyDef.position = { 0.0f, 0.103f };
		characterBodyDef.rotation = b2MakeRot( 0.0f );
		characterBodyDef.linearDamping = 0.0f;
		characterBodyDef.angularDamping = 0.0f;
		characterBodyDef.gravityScale = 1.0f;
		characterBodyDef.enableSleep = true;
		characterBodyDef.isAwake = false;
		characterBodyDef.motionLocks.angularZ = true;
		characterBodyDef.isEnabled = true;
		characterBodyDef.userData = nullptr;
		characterBodyDef.type = b2_kinematicBody;

		characterBodyId_ = b2CreateBody( m_worldId, &characterBodyDef );

		b2ShapeDef characterShapeDef = b2DefaultShapeDef();

		characterShapeDef.userData = nullptr;
		characterShapeDef.filter.categoryBits = 0x1;
		characterShapeDef.filter.maskBits = 0x1;
		characterShapeDef.filter.groupIndex = 0;
		characterShapeDef.isSensor = false;
		characterShapeDef.enableSensorEvents = false;
		characterShapeDef.enableContactEvents = false;
		characterShapeDef.enableHitEvents = false;
		characterShapeDef.enablePreSolveEvents = false;
		characterShapeDef.invokeContactCreation = false;
		characterShapeDef.updateBodyMass = false;

		characterBox_ = b2MakeBox( 0.1f, 0.1f );
		b2CreatePolygonShape( characterBodyId_, &characterShapeDef, &characterBox_ );

		context->camera.m_center = b2Vec2_zero;
	}

	void Step() override
	{
		const float timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;

		bool noXInput = true;
		if ( glfwGetKey( m_context->window, GLFW_KEY_A ) )
		{
			characterVelocity_.x -= timeStep * 5.0f;
			noXInput = false;
		}
		if ( glfwGetKey( m_context->window, GLFW_KEY_D ) )
		{
			characterVelocity_.x += timeStep * 5.0f;
			noXInput = false;
		}

		bool noYInput = true;
		if ( glfwGetKey( m_context->window, GLFW_KEY_S ) )
		{
			characterVelocity_.y -= timeStep * 5.0f;
			noYInput = false;
		}
		if ( glfwGetKey( m_context->window, GLFW_KEY_W ) )
		{
			characterVelocity_.y += timeStep * 5.0f;
			noYInput = false;
		}

		if ( noXInput )
		{
			if ( b2AbsFloat( characterVelocity_.x ) > 0.01f )
			{
				const float decel = characterVelocity_.x > 0.0f ? 5.0f : -5.0f;
				if ( b2AbsFloat( decel ) < characterVelocity_.x )
				{
					characterVelocity_.x -= decel;
				}
				else
				{
					characterVelocity_.x = 0.0f;
				}
			}
			else
			{
				characterVelocity_.x = 0.0f;
			}
		}

		if ( noYInput )
		{
			if ( b2AbsFloat( characterVelocity_.y ) > 0.01f )
			{
				const float decel = characterVelocity_.y > 0.0f ? 5.0f : -5.0f;
				if ( b2AbsFloat( decel ) < characterVelocity_.y )
				{
					characterVelocity_.y -= decel;
				}
				else
				{
					characterVelocity_.y = 0.0f;
				}
			}
			else
			{
				characterVelocity_.y = 0.0f;
			}
		}

		b2Vec2 characterPos = b2Body_GetPosition( characterBodyId_ );
		b2Vec2 newCharacterPos = characterPos;
		newCharacterPos.x += characterVelocity_.x * timeStep;
		newCharacterPos.y += characterVelocity_.y * timeStep;
		b2Body_SetTransform( characterBodyId_, newCharacterPos, b2Rot_identity );

		PhysicsHitQueryResult2D hitResult;
		const b2ShapeProxy shapeProxy = b2MakeProxy( characterBox_.vertices, characterBox_.count, characterBox_.radius );
		if ( ShapeCastSingle( hitResult, characterPos, newCharacterPos, 0.0f, shapeProxy ) )
		{
			hitPos = hitResult.point;
			hitNormal = hitResult.normal;
		}

		m_draw->DrawLine( hitPos, { hitPos.x + hitNormal.x, hitPos.y + hitNormal.y }, b2_colorRed );

		Sample::Step();
	}

	bool ShapeCastSingle( PhysicsHitQueryResult2D& outResult, b2Vec2 start, b2Vec2 end, float rotation,
						  const b2ShapeProxy& shape ) const
	{
		const b2Transform transform = { start, b2MakeRot( rotation ) };
		const b2ShapeProxy transformedShape = TransformShapeProxy( transform, shape );

		const b2Vec2 translation = { end.x - start.x, end.y - start.y };
		const b2QueryFilter filter = { 0x1, 0x1 };
		CastContext_Single context;
		context.startPos = start;
		context.translation = { translation.x, translation.y };
		b2World_CastShape( m_worldId, &transformedShape, translation, filter, &b2CastResult_Closest, &context );

		if ( context.hit )
		{
			outResult = context.result;
			return true;
		}

		return false;
	}

	static Sample* Create( SampleContext* context )
	{
		return new ShapeCastChain( context );
	}

private:
	b2BodyId characterBodyId_ = b2_nullBodyId;
	b2Polygon characterBox_;
	b2Vec2 characterVelocity_ = b2Vec2_zero;
	b2Vec2 hitPos = b2Vec2_zero;
	b2Vec2 hitNormal = b2Vec2_zero;
};

static int sampleShapeCastChain = RegisterSample( "Issues", "Shape Cast Chain", ShapeCastChain::Create );

class BadSteiner : public Sample
{
public:
	explicit BadSteiner( SampleContext* context )
		: Sample( context )
	{
		if ( m_context->restart == false )
		{
			m_context->camera.m_center = { 0.0f, 1.75f };
			m_context->camera.m_zoom = 2.5f;
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
			m_context->camera.m_center = { 0.8f, 6.4f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
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
			b2Vec2 pivot = { -2.0f, 5.0f };
			revoluteDef.base.bodyIdA = m_attachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.base.localFrameB.p = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );
		}
	}

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 11.0f * fontSize;
		float winX = 0.5f * fontSize;
		float winY = m_camera->m_height - height - 2.0f * fontSize;
		ImGui::SetNextWindowPos( { winX, winY }, ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 9.0f * fontSize, height ) );
		ImGui::Begin( "Disable Crash", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

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

		ImGui::End();
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
			m_context->camera.m_center = { 0.8f, 6.4f };
			m_context->camera.m_zoom = 25.0f * 0.4f;
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
			b2Vec2 pivot = { -2.0f, 5.0f };
			revoluteDef.base.bodyIdA = m_attachmentId;
			revoluteDef.base.bodyIdB = m_platformId;
			revoluteDef.base.localFrameA.p = b2Body_GetLocalPoint( m_attachmentId, pivot );
			revoluteDef.base.localFrameB.p = b2Body_GetLocalPoint( m_platformId, pivot );
			revoluteDef.maxMotorTorque = 50.0f;
			revoluteDef.enableMotor = true;
			b2CreateRevoluteJoint( m_worldId, &revoluteDef );

			b2PrismaticJointDef prismaticDef = b2DefaultPrismaticJointDef();
			b2Vec2 anchor = { 0.0f, 5.0f };
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

	void UpdateGui() override
	{
		float fontSize = ImGui::GetFontSize();
		float height = 11.0f * fontSize;
		ImGui::SetNextWindowPos( ImVec2( 0.5f * fontSize, m_camera->m_height - height - 2.0f * fontSize ), ImGuiCond_Once );
		ImGui::SetNextWindowSize( ImVec2( 9.0f * fontSize, height ) );
		ImGui::Begin( "Crash 01", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

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

		ImGui::End();
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
			m_context->camera.m_center = { 48.8525391, 68.1518555 };
			m_context->camera.m_zoom = 100.0f * 0.5f;
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
