/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* iPhone port by Simon Oliver - http://www.simonoliver.com - http://www.handcircus.com
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

#include "iPhoneTest.h"
#include "GLES-Render.h"

#include <cstdio>

void DestructionListener::SayGoodbye(b2Joint* joint)
{
	if (test->m_mouseJoint == joint)
	{
		test->m_mouseJoint = NULL;
	}
	else
	{
		test->JointDestroyed(joint);
	}
}

void BoundaryListener::Violation(b2Body* body)
{
	if (test->m_bomb != body)
	{
		test->BoundaryViolated(body);
	}
}

void ContactListener::Add(const b2ContactPoint* point)
{
	if (test->m_pointCount == k_maxContactPoints)
	{
		return;
	}

	ContactPoint* cp = test->m_points + test->m_pointCount;
	cp->shape1 = point->shape1;
	cp->shape2 = point->shape2;
	cp->position = point->position;
	cp->normal = point->normal;
	cp->id = point->id;
	cp->state = e_contactAdded;

	++test->m_pointCount;
}

void ContactListener::Persist(const b2ContactPoint* point)
{
	if (test->m_pointCount == k_maxContactPoints)
	{
		return;
	}

	ContactPoint* cp = test->m_points + test->m_pointCount;
	cp->shape1 = point->shape1;
	cp->shape2 = point->shape2;
	cp->position = point->position;
	cp->normal = point->normal;
	cp->id = point->id;
	cp->state = e_contactPersisted;

	++test->m_pointCount;
}

void ContactListener::Remove(const b2ContactPoint* point)
{
	if (test->m_pointCount == k_maxContactPoints)
	{
		return;
	}

	ContactPoint* cp = test->m_points + test->m_pointCount;
	cp->shape1 = point->shape1;
	cp->shape2 = point->shape2;
	cp->position = point->position;
	cp->normal = point->normal;
	cp->id = point->id;
	cp->state = e_contactRemoved;

	++test->m_pointCount;
}

Test::Test()
{
	m_worldAABB.lowerBound.Set(-200.0f, -100.0f);
	m_worldAABB.upperBound.Set(200.0f, 200.0f);
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(m_worldAABB, gravity, doSleep);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
	m_pointCount = 0;

	m_destructionListener.test = this;
	m_boundaryListener.test = this;
	m_contactListener.test = this;
	m_world->SetDestructionListener(&m_destructionListener);
	m_world->SetBoundaryListener(&m_boundaryListener);
	m_world->SetContactListener(&m_contactListener);
	m_world->SetDebugDraw(&m_debugDraw);
	
	m_bombSpawning = false;
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void Test::SetGravity(float x,float y)
{
	float tVectorLength=sqrt(x*x+y*y);
	float newGravityX=9.81f*x/tVectorLength;
	float newGravityY=9.81f*y/tVectorLength;
	m_world->SetGravity(b2Vec2(newGravityX,newGravityY));
}

void Test::DrawTitle(int x, int y, const char *string)
{
    m_debugDraw.DrawString(x, y, string);
}

void Test::MouseDown(const b2Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	const int32 k_maxCount = 10;
	b2Shape* shapes[k_maxCount];
	int32 count = m_world->Query(aabb, shapes, k_maxCount);
	b2Body* body = NULL;
	for (int32 i = 0; i < count; ++i)
	{
		b2Body* shapeBody = shapes[i]->GetBody();
		if (shapeBody->IsStatic() == false && shapeBody->GetMass() > 0.0f)
		{
			bool inside = shapes[i]->TestPoint(shapeBody->GetXForm(), p);
			if (inside)
			{
				body = shapes[i]->GetBody();
				break;
			}
		}
	}

	if (body)
	{
		b2MouseJointDef md;
		md.body1 = m_world->GetGroundBody();
		md.body2 = body;
		md.target = p;
#ifdef TARGET_FLOAT32_IS_FIXED
		md.maxForce = (body->GetMass() < 16.0)? 
			(1000.0f * body->GetMass()) : float32(16000.0);
#else
		md.maxForce = 1000.0f * body->GetMass();
#endif
		m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
		body->WakeUp();
	}
}

void Test::SpawnBomb(const b2Vec2& worldPt)
{
	m_bombSpawnPoint = worldPt;
	m_bombSpawning = true;
}
    
void Test::CompleteBombSpawn(const b2Vec2& p)
{
	if (!m_bombSpawning) return;
	const float multiplier = 30.0f;
	b2Vec2 vel = m_bombSpawnPoint - p;
	vel *= multiplier;
	LaunchBomb(m_bombSpawnPoint,vel);
	m_bombSpawning = false;
}

void Test::ShiftMouseDown(const b2Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	SpawnBomb(p);
}

void Test::MouseUp(const b2Vec2& p)
{
	if (m_mouseJoint)
	{
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
	
	if (m_bombSpawning)
	{
		CompleteBombSpawn(p);
	}
}

void Test::MouseMove(const b2Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint)
	{
		m_mouseJoint->SetTarget(p);
	}
}

void Test::LaunchBomb()
{
	b2Vec2 p(RandomFloat(-15.0f, 15.0f), 30.0f);
	b2Vec2 v = -5.0f * p;
	LaunchBomb(p, v);
}

void Test::LaunchBomb(const b2Vec2& position, const b2Vec2& velocity)
{
	if (m_bomb)
	{
		m_world->DestroyBody(m_bomb);
		m_bomb = NULL;
	}

	b2BodyDef bd;
	bd.allowSleep = true;
	bd.position = position;
	
	bd.isBullet = true;
	m_bomb = m_world->CreateBody(&bd);
	m_bomb->SetLinearVelocity(velocity);
	
	b2CircleDef sd;
	sd.radius = 0.3f;
	sd.density = 20.0f;
	sd.restitution = 0.1f;
	
	b2Vec2 minV = position - b2Vec2(0.3f,0.3f);
	b2Vec2 maxV = position + b2Vec2(0.3f,0.3f);
	
	b2AABB aabb;
	aabb.lowerBound = minV;
	aabb.upperBound = maxV;
	
	bool inRange = m_world->InRange(aabb);

	if (inRange)
	{
		m_bomb->CreateShape(&sd);
		m_bomb->SetMassFromShapes();
	}
}

void Test::Step(Settings* settings)
{
	float32 timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

	if (settings->pause)
	{
		if (settings->singleStep)
		{
			settings->singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += 15;
	}

	uint32 flags = 0;
	flags += settings->drawShapes			* b2DebugDraw::e_shapeBit;
	flags += settings->drawJoints			* b2DebugDraw::e_jointBit;
	flags += settings->drawControllers		* b2DebugDraw::e_controllerBit;
	flags += settings->drawCoreShapes		* b2DebugDraw::e_coreShapeBit;
	flags += settings->drawAABBs			* b2DebugDraw::e_aabbBit;
	flags += settings->drawOBBs				* b2DebugDraw::e_obbBit;
	flags += settings->drawPairs			* b2DebugDraw::e_pairBit;
	flags += settings->drawCOMs				* b2DebugDraw::e_centerOfMassBit;
	m_debugDraw.SetFlags(flags);

	m_world->SetWarmStarting(settings->enableWarmStarting > 0);
	m_world->SetContinuousPhysics(settings->enableTOI > 0);

	m_pointCount = 0;

	m_world->Step(timeStep, settings->velocityIterations, settings->positionIterations);

	m_world->Validate();

	if (m_bomb != NULL && m_bomb->IsFrozen())
	{
		m_world->DestroyBody(m_bomb);
		m_bomb = NULL;
	}

	if (settings->drawStats)
	{
		m_debugDraw.DrawString(5, m_textLine, "proxies(max) = %d(%d), pairs(max) = %d(%d)",
			m_world->GetProxyCount(), b2_maxProxies,
			m_world->GetPairCount(), b2_maxPairs);
		m_textLine += 15;

		m_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d",
			m_world->GetBodyCount(), m_world->GetContactCount(), m_world->GetJointCount());
		m_textLine += 15;

		m_debugDraw.DrawString(5, m_textLine, "heap bytes = %d", b2_byteCount);
		m_textLine += 15;
	}

	if (m_mouseJoint)
	{
		b2Body* body = m_mouseJoint->GetBody2();
		b2Vec2 p1 = body->GetWorldPoint(m_mouseJoint->m_localAnchor);
		b2Vec2 p2 = m_mouseJoint->m_target;

		m_debugDraw.DrawSegment(p1, p2, b2Color(0.8f,0.8f,0.8f));
		m_debugDraw.DrawPoint(p1, 4, b2Color(0,1,0));
		m_debugDraw.DrawPoint(p2, 4, b2Color(0,1,0));

		
		
	}
	
	if (m_bombSpawning){
		/*
		glPointSize(4.0f);
		glColor4f(0.0f, 0.0f, 1.0f,1);
		glBegin(GL_POINTS);
		glColor4f(0.0f, 0.0f, 1.0f,1);
		glVertex2f(m_bombSpawnPoint.x, m_bombSpawnPoint.y);
		glEnd();
		
		glColor4f(0.8f, 0.8f, 0.8f,1);
		glBegin(GL_LINES);
		glVertex2f(m_mouseWorld.x, m_mouseWorld.y);
		glVertex2f(m_bombSpawnPoint.x, m_bombSpawnPoint.y);
		glEnd();
		 */
	}

	if (settings->drawContactPoints)
	{
		//const float32 k_impulseScale = 0.1f;
		const float32 k_axisScale = 0.3f;

		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			if (point->state == 0)
			{
				// Add
				m_debugDraw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
			}
			else if (point->state == 1)
			{
				// Persist
				m_debugDraw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
			}
			else
			{
				// Remove
				m_debugDraw.DrawPoint(point->position, 10.0f, b2Color(0.95f, 0.3f, 0.3f));
			}

			if (settings->drawContactNormals == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_axisScale * point->normal;
				m_debugDraw.DrawSegment(p1, p2, b2Color(0.4f, 0.9f, 0.4f));
			}
			else if (settings->drawContactForces == 1)
			{
				//b2Vec2 p1 = point->position;
				//b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
				//DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}

			if (settings->drawFrictionForces == 1)
			{
				//b2Vec2 tangent = b2Cross(point->normal, 1.0f);
				//b2Vec2 p1 = point->position;
				//b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
				//DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}
		}
	}
}
