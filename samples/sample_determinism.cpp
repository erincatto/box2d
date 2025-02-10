// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"
#include "sample.h"
#include "settings.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <stdio.h>

// This sample provides a visual representation of the cross platform determinism unit test.
// The scenario is designed to produce a chaotic result engaging:
// - continuous collision
// - joint limits (approximate atan2)
// - b2MakeRot (approximate sin/cos)
// Once all the bodies go to sleep the step counter and transform hash is emitted which
// can then be transferred to the unit test and tested in GitHub build actions.
// See CrossPlatformTest in the unit tests.
class FallingHinges : public Sample
{
public:
	enum
	{
		e_columns = 4,
		e_rows = 30,
	};

	explicit FallingHinges( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 7.5f };
			g_camera.m_zoom = 10.0f;
		}

		{
			b2BodyDef bodyDef = b2DefaultBodyDef();
			bodyDef.position = { 0.0f, -1.0f };
			b2BodyId groundId = b2CreateBody( m_worldId, &bodyDef );

			b2Polygon box = b2MakeBox( 20.0f, 1.0f );
			b2ShapeDef shapeDef = b2DefaultShapeDef();
			b2CreatePolygonShape( groundId, &shapeDef, &box );
		}

		for ( int i = 0; i < e_rows * e_columns; ++i )
		{
			m_bodies[i] = b2_nullBodyId;
		}

		float h = 0.25f;
		float r = 0.1f * h;
		b2Polygon box = b2MakeRoundedBox( h - r, h - r, r );

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.friction = 0.3f;

		float offset = 0.4f * h;
		float dx = 10.0f * h;
		float xroot = -0.5f * dx * ( e_columns - 1.0f );

		b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
		jointDef.enableLimit = true;
		jointDef.lowerAngle = -0.1f * B2_PI;
		jointDef.upperAngle = 0.2f * B2_PI;
		jointDef.enableSpring = true;
		jointDef.hertz = 0.5f;
		jointDef.dampingRatio = 0.5f;
		jointDef.localAnchorA = { h, h };
		jointDef.localAnchorB = { offset, -h };
		jointDef.drawSize = 0.1f;

		int bodyIndex = 0;
		int bodyCount = e_rows * e_columns;

		for ( int j = 0; j < e_columns; ++j )
		{
			float x = xroot + j * dx;

			b2BodyId prevBodyId = b2_nullBodyId;

			for ( int i = 0; i < e_rows; ++i )
			{
				b2BodyDef bodyDef = b2DefaultBodyDef();
				bodyDef.type = b2_dynamicBody;

				bodyDef.position.x = x + offset * i;
				bodyDef.position.y = h + 2.0f * h * i;

				// this tests the deterministic cosine and sine functions
				bodyDef.rotation = b2MakeRot( 0.1f * i - 1.0f );

				b2BodyId bodyId = b2CreateBody( m_worldId, &bodyDef );

				if ( ( i & 1 ) == 0 )
				{
					prevBodyId = bodyId;
				}
				else
				{
					jointDef.bodyIdA = prevBodyId;
					jointDef.bodyIdB = bodyId;
					b2CreateRevoluteJoint( m_worldId, &jointDef );
					prevBodyId = b2_nullBodyId;
				}

				b2CreatePolygonShape( bodyId, &shapeDef, &box );

				assert( bodyIndex < bodyCount );
				m_bodies[bodyIndex] = bodyId;

				bodyIndex += 1;
			}
		}

		m_hash = 0;
		m_sleepStep = -1;

		// PrintTransforms();
	}

	void PrintTransforms()
	{
		uint32_t hash = B2_HASH_INIT;
		int bodyCount = e_rows * e_columns;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2Transform xf = b2Body_GetTransform( m_bodies[i] );
			printf( "%d %.9f %.9f %.9f %.9f\n", i, xf.p.x, xf.p.y, xf.q.c, xf.q.s );
			hash = b2Hash( hash, reinterpret_cast<uint8_t*>( &xf ), sizeof( b2Transform ) );
		}

		printf( "hash = 0x%08x\n", hash );
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		if ( m_hash == 0 )
		{
			b2BodyEvents bodyEvents = b2World_GetBodyEvents( m_worldId );

			if ( bodyEvents.moveCount == 0 )
			{
				uint32_t hash = B2_HASH_INIT;
				int bodyCount = e_rows * e_columns;
				for ( int i = 0; i < bodyCount; ++i )
				{
					b2Transform xf = b2Body_GetTransform( m_bodies[i] );
					// printf( "%d %.9f %.9f %.9f %.9f\n", i, xf.p.x, xf.p.y, xf.q.c, xf.q.s );
					hash = b2Hash( hash, reinterpret_cast<uint8_t*>( &xf ), sizeof( b2Transform ) );
				}

				m_sleepStep = m_stepCount - 1;
				m_hash = hash;
				printf( "sleep step = %d, hash = 0x%08x\n", m_sleepStep, m_hash );
			}
		}

		g_draw.DrawString( 5, m_textLine, "sleep step = %d, hash = 0x%08x", m_sleepStep, m_hash );
		m_textLine += m_textIncrement;
	}

	static Sample* Create( Settings& settings )
	{
		return new FallingHinges( settings );
	}

	b2BodyId m_bodies[e_rows * e_columns];
	uint32_t m_hash;
	int m_sleepStep;
};

static int sampleFallingHinges = RegisterSample( "Determinism", "Falling Hinges", FallingHinges::Create );

#if 0

#include <stdlib.h>

#define WALL_THICKNESS 4.0f
#define STANDARD_WALL_RESTITUTION 0.01f
#define WALL_DENSITY 4.0f

#define DRONE_RADIUS 1.0f
#define DRONE_DENSITY 1.25f
#define DRONE_LINEAR_DAMPING 1.0f

enum entityType
{
	STANDARD_WALL_ENTITY,
	PROJECTILE_ENTITY,
	DRONE_ENTITY,
};

enum shapeCategory
{
	WALL_SHAPE = 1,
	PROJECTILE_SHAPE = 4,
	DRONE_SHAPE = 16,
};

typedef struct entity
{
	enum entityType type;
	void* entityPtr;
} entity;

typedef struct wallEntity
{
	b2BodyId bodyID;
	b2ShapeId shapeID;
	b2Vec2 pos;
	b2Rot rot;
	b2Vec2 velocity;
	b2Vec2 extent;
	int16_t mapCellIdx;
	bool isFloating;
	enum entityType type;
	bool isSuddenDeath;
} wallEntity;

typedef struct weaponInformation
{
	const float fireMagnitude;
	const float recoilMagnitude;
	const float charge;
	const float coolDown;
	const float maxDistance;
	const float radius;
	const float density;
	const float invMass;
	const uint8_t maxBounces;
} weaponInformation;

typedef struct droneEntity
{
	b2BodyId bodyID;
	b2ShapeId shapeID;
	weaponInformation* weaponInfo;
	int8_t ammo;
	float weaponCooldown;

	uint8_t idx;
	b2Vec2 initalPos;
	b2Vec2 pos;
	b2Vec2 lastPos;
	b2Vec2 lastMove;
	b2Vec2 lastAim;
	b2Vec2 velocity;
	b2Vec2 lastVelocity;
	bool dead;
} droneEntity;

typedef struct projectileEntity
{
	uint8_t droneIdx;

	b2BodyId bodyID;
	b2ShapeId shapeID;
	weaponInformation* weaponInfo;
	b2Vec2 pos;
	b2Vec2 lastPos;
	b2Vec2 velocity;
	float lastSpeed;
	float distance;
	uint8_t bounces;
} projectileEntity;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

#define INV_MASS( density, radius ) ( 1.0f / ( density * PI * radius * radius ) )

#define MACHINEGUN_RADIUS 0.15f
#define MACHINEGUN_DENSITY 3.0f

weaponInformation machineGun = {
	25.0f,
	12.8f,
	0.0f,
	0.07f,
	225.0f,
	MACHINEGUN_RADIUS,
	MACHINEGUN_DENSITY,
	INV_MASS( MACHINEGUN_DENSITY, MACHINEGUN_RADIUS ),
	2,
};

// clang-format off

const char boringLayout[] = {
    'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','O','W',
    'W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W','W',
};

// clang-format on

static inline bool b2VecEqual( const b2Vec2 v1, const b2Vec2 v2 )
{
	return v1.x == v2.x && v1.y == v2.y;
}

entity* createWall( b2WorldId worldID, const float posX, const float posY, uint16_t cellIdx )
{
	const b2Vec2 pos = { posX, posY };
	b2BodyDef wallBodyDef = b2DefaultBodyDef();
	wallBodyDef.position = pos;
	b2BodyId wallBodyID = b2CreateBody( worldID, &wallBodyDef );
	b2Vec2 extent = { WALL_THICKNESS / 2.0f, WALL_THICKNESS / 2.0f };
	b2ShapeDef wallShapeDef = b2DefaultShapeDef();
	wallShapeDef.density = WALL_DENSITY;
	wallShapeDef.restitution = STANDARD_WALL_RESTITUTION;
	wallShapeDef.filter.categoryBits = WALL_SHAPE;
	wallShapeDef.filter.maskBits = PROJECTILE_SHAPE | DRONE_SHAPE;

	wallEntity* wall = (wallEntity*)calloc( 1, sizeof( wallEntity ) );
	wall->bodyID = wallBodyID;
	wall->pos = pos;
	wall->rot = b2Rot_identity;
	wall->velocity = b2Vec2_zero;
	wall->extent = extent;
	wall->mapCellIdx = cellIdx;

	entity* ent = (entity*)calloc( 1, sizeof( entity ) );
	ent->type = STANDARD_WALL_ENTITY;
	ent->entityPtr = wall;

	wallShapeDef.userData = ent;
	const b2Polygon wallPolygon = b2MakeBox( extent.x, extent.y );
	wall->shapeID = b2CreatePolygonShape( wallBodyID, &wallShapeDef, &wallPolygon );
	b2Body_SetUserData( wall->bodyID, ent );

	return ent;
}

void setupMap( b2WorldId worldID )
{
	const uint8_t columns = 21;
	const uint8_t rows = 21;
	const char* layout = boringLayout;

	uint16_t cellIdx = 0;
	for ( int row = 0; row < rows; row++ )
	{
		for ( int col = 0; col < columns; col++ )
		{
			char cellType = layout[col + ( row * columns )];
			if ( cellType == 'O' )
			{
				continue;
			}

			const float x = ( col - ( ( columns - 1 ) / 2.0f ) ) * WALL_THICKNESS;
			const float y = ( row - ( rows - 1 ) / 2.0f ) * WALL_THICKNESS;

			createWall( worldID, x, y, cellIdx );
			cellIdx++;
		}
	}
}

droneEntity* createDrone( b2WorldId worldID )
{
	b2BodyDef droneBodyDef = b2DefaultBodyDef();
	droneBodyDef.type = b2_dynamicBody;
	droneBodyDef.position = b2Vec2_zero;
	droneBodyDef.fixedRotation = true;
	droneBodyDef.linearDamping = DRONE_LINEAR_DAMPING;
	b2BodyId droneBodyID = b2CreateBody( worldID, &droneBodyDef );
	b2ShapeDef droneShapeDef = b2DefaultShapeDef();
	droneShapeDef.density = DRONE_DENSITY;
	droneShapeDef.friction = 0.0f;
	droneShapeDef.restitution = 0.3f;
	droneShapeDef.filter.categoryBits = DRONE_SHAPE;
	droneShapeDef.filter.maskBits = WALL_SHAPE | PROJECTILE_SHAPE | DRONE_SHAPE;
	droneShapeDef.enableContactEvents = true;
	// droneShapeDef.enableSensorEvents = true;
	const b2Circle droneCircle = { b2Vec2_zero, DRONE_RADIUS };

	droneEntity* drone = (droneEntity*)calloc( 1, sizeof( droneEntity ) );
	drone->bodyID = droneBodyID;
	drone->weaponInfo = &machineGun;
	drone->ammo = -1;
	drone->weaponCooldown = 0.0f;
	drone->initalPos = droneBodyDef.position;
	drone->pos = droneBodyDef.position;
	drone->lastPos = b2Vec2_zero;
	drone->lastMove = b2Vec2_zero;
	drone->lastAim = { 0.0f, -1.0f };
	drone->velocity = b2Vec2_zero;
	drone->lastVelocity = b2Vec2_zero;
	drone->dead = false;

	entity* ent = (entity*)calloc( 1, sizeof( entity ) );
	ent->type = DRONE_ENTITY;
	ent->entityPtr = drone;

	droneShapeDef.userData = ent;
	drone->shapeID = b2CreateCircleShape( droneBodyID, &droneShapeDef, &droneCircle );
	b2Body_SetUserData( drone->bodyID, ent );

	return drone;
}

void createProjectile( b2WorldId worldID, droneEntity* drone, const b2Vec2 normAim )
{
	b2BodyDef projectileBodyDef = b2DefaultBodyDef();
	projectileBodyDef.type = b2_dynamicBody;
	projectileBodyDef.fixedRotation = true;
	projectileBodyDef.isBullet = true;
	projectileBodyDef.enableSleep = false;
	float radius = drone->weaponInfo->radius;

	projectileBodyDef.position = b2MulAdd( drone->pos, 1.0f + ( radius * 1.5f ), normAim );
	printf( "projectile: %g %g\n", projectileBodyDef.position.x, projectileBodyDef.position.y );

	b2BodyId projectileBodyID = b2CreateBody( worldID, &projectileBodyDef );
	b2ShapeDef projectileShapeDef = b2DefaultShapeDef();
	projectileShapeDef.enableContactEvents = true;
	projectileShapeDef.density = drone->weaponInfo->density;
	projectileShapeDef.friction = 0.0f;
	projectileShapeDef.restitution = 1.0f;
	projectileShapeDef.filter.categoryBits = PROJECTILE_SHAPE;
	projectileShapeDef.filter.maskBits = WALL_SHAPE | PROJECTILE_SHAPE | DRONE_SHAPE;
	const b2Circle projectileCircle = { b2Vec2_zero, radius };

	b2ShapeId projectileShapeID = b2CreateCircleShape( projectileBodyID, &projectileShapeDef, &projectileCircle );

	b2Vec2 fire = b2MulSV( machineGun.fireMagnitude, normAim );
	b2Body_ApplyLinearImpulseToCenter( projectileBodyID, fire, true );

	projectileEntity* projectile = (projectileEntity*)calloc( 1, sizeof( projectileEntity ) );
	projectile->droneIdx = drone->idx;
	projectile->bodyID = projectileBodyID;
	projectile->shapeID = projectileShapeID;
	projectile->weaponInfo = drone->weaponInfo;
	projectile->pos = projectileBodyDef.position;
	projectile->lastPos = projectileBodyDef.position;
	projectile->velocity = b2Body_GetLinearVelocity( projectileBodyID );
	projectile->lastSpeed = b2Length( projectile->velocity );
	projectile->distance = 0.0f;
	projectile->bounces = 0;

	entity* ent = (entity*)calloc( 1, sizeof( entity ) );
	ent->type = PROJECTILE_ENTITY;
	ent->entityPtr = projectile;

	b2Body_SetUserData( projectile->bodyID, ent );
	b2Shape_SetUserData( projectile->shapeID, ent );
}

static void droneShoot( b2WorldId worldID, droneEntity* drone, const b2Vec2 aim )
{
	if ( drone->weaponCooldown != 0.0f )
	{
		return;
	}

	drone->weaponCooldown = drone->weaponInfo->coolDown;

	b2Vec2 normAim = drone->lastAim;
	if ( !b2VecEqual( aim, b2Vec2_zero ) )
	{
		normAim = b2Normalize( aim );
	}
	b2Vec2 recoil = b2MulSV( -drone->weaponInfo->recoilMagnitude, normAim );
	b2Body_ApplyLinearImpulseToCenter( drone->bodyID, recoil, true );

	createProjectile( worldID, drone, normAim );
}

class BulletBug : public Sample
{
public:
	explicit BulletBug( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 0.0f };
			g_camera.m_zoom = 80.0f;
		}

		b2World_SetGravity( m_worldId, b2Vec2_zero );

		setupMap( m_worldId );
		m_drone = createDrone( m_worldId );
	}

	void Step( Settings& settings ) override
	{
		float aimX = tanhf( rand() / (float)RAND_MAX );
		float aimY = tanhf( rand() / (float)RAND_MAX );

		droneShoot( m_worldId, m_drone, { aimX, aimY } );

		g_draw.DrawPoint( { -40.0f, -40.0f }, 5.0f, b2_colorViolet );
		g_draw.DrawPoint( { 40.0f, -40.0f }, 5.0f, b2_colorViolet );
		g_draw.DrawPoint( { -40.0f, 40.0f }, 5.0f, b2_colorViolet );
		g_draw.DrawPoint( { 40.0f, 40.0f }, 5.0f, b2_colorViolet );

		if (m_wasOut)
		{
			g_draw.DrawPoint( m_pos, 5.0f, b2_colorLightBlue );
		}

		Sample::Step( settings );

		//b2World_Step( m_worldId, 1.0f / 10.0f, 1 );

		b2BodyEvents events = b2World_GetBodyEvents( m_worldId );
		for ( int i = 0; i < events.moveCount; i++ )
		{
			const b2BodyMoveEvent* event = events.moveEvents + i;
			assert( b2Body_IsValid( event->bodyId ) );
			entity* ent = (entity*)event->userData;
			if ( ent == NULL )
			{
				continue;
			}

			const b2Vec2 pos = event->transform.p;
			m_pos = pos;

			// check if the body is beyond the outside bounds of map
			if ( pos.x < -48.0f || pos.x > 48.0f || pos.y < -48.0f || pos.y > 48.0f )
			{
				printf( "body is outside of wall bounds: (%f, %f)\n", pos.x, pos.y );
				m_wasOut = true;
				m_pos = pos;
				settings.pause = true;
				//exit( 1 );
			}
		}

		g_draw.DrawPoint( m_pos, 10.0f, b2_colorWhite );
	}

	static Sample* Create( Settings& settings )
	{
		return new BulletBug( settings );
	}

	b2Vec2 m_pos = {0.0f, 0.0f};
	bool m_wasOut = false;
	droneEntity* m_drone;
};

static int sampleBulletBug = RegisterSample( "Bugs", "Bullet Bug", BulletBug::Create );

class OverlapBug : public Sample
{
public:
	explicit OverlapBug( Settings& settings )
		: Sample( settings )
	{
		if ( settings.restart == false )
		{
			g_camera.m_center = { 0.0f, 2.5f };
			g_camera.m_zoom = 3.5f;
		}

		float boxSize = 0.5f;
		b2BodyDef body_def = b2DefaultBodyDef();
		body_def.type = b2_staticBody;
		body_def.position = { m_x, m_y };
		b2BodyId body_id = b2CreateBody( m_worldId, &body_def );
		b2Polygon polygon = b2MakeSquare( boxSize );
		b2ShapeDef shape_def = b2DefaultShapeDef();
		b2CreatePolygonShape( body_id, &shape_def, &polygon );
	}

	static bool Callback( b2ShapeId id, void* context )
	{
		OverlapBug* self = static_cast<OverlapBug*>( context );
		self->m_overlap = true;
		return false;
	}

	void Step( Settings& settings ) override
	{
		Sample::Step( settings );

		float testSize = 0.4f;
		b2Polygon test_polygon = b2MakeSquare( testSize );
		b2Transform tfm = { { m_x, m_y }, { 1.0f, 0.0f } };
		b2World_OverlapPolygon( m_worldId, &test_polygon, tfm, b2DefaultQueryFilter(), OverlapBug::Callback, this );

		b2Vec2 vertices[4];
		vertices[0] = b2TransformPoint(tfm, test_polygon.vertices[0]);
		vertices[1] = b2TransformPoint(tfm, test_polygon.vertices[1]);
		vertices[2] = b2TransformPoint(tfm, test_polygon.vertices[2]);
		vertices[3] = b2TransformPoint(tfm, test_polygon.vertices[3]);
		g_draw.DrawPolygon(vertices, 4, b2_colorOrange);

		if ( m_overlap )
		{
			DrawTextLine( "overlap" );
		}
		else
		{
			DrawTextLine( "no overlap" );
		}
	}

	static Sample* Create( Settings& settings )
	{
		return new OverlapBug( settings );
	}

	float m_x = 3.0f;
	float m_y = 5.0f;
	bool m_overlap = false;
};

static int sampleSingleBox = RegisterSample( "Bugs", "Overlap", OverlapBug::Create );
#endif
