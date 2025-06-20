#include "sample.h"

#include "box2d/box2d.h"

#include <GLFW/glfw3.h>

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
