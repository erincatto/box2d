/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef DUMP_SHELL_H
#define DUMP_SHELL_H

// This test holds worlds dumped using b2World::Dump.
class DumpShell : public Test
{
public:

	DumpShell()
	{

b2Vec2 g(0.000000000000000e+000f, 0.00000000000000e+00f);
m_world->SetGravity(g);
b2Body** bodies = (b2Body**)b2Alloc(2 * sizeof(b2Body*));
b2Joint** joints = (b2Joint**)b2Alloc(0 * sizeof(b2Joint*));

{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(3.000000000000000e+001f, 6.909857940673828e+001f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, -8.618643951416016e+001f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(16);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[0] = m_world->CreateBody(&bd);
  {
    b2FixtureDef fd;
    fd.friction = 6.000000238418579e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e-001f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(2);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-1.950000047683716e+000f, -4.750000000000000e+000f);
    vs[1].Set(1.950000047683716e+000f, -4.750000000000000e+000f);
    vs[2].Set(1.950000047683716e+000f, 4.750000000000000e+000f);
    vs[3].Set(-1.950000047683716e+000f, 4.750000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[0]->CreateFixture(&fd);
  }
}

{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(5.120000457763672e+001f, 7.580000305175781e+001f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[1] = m_world->CreateBody(&bd);
  {
    b2FixtureDef fd;
    fd.friction = 0.000000000000000e+000f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 1.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(2);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.120000076293945e+001f, -5.000000000000000e-001f);
    vs[1].Set(5.120000076293945e+001f, -5.000000000000000e-001f);
    vs[2].Set(5.120000076293945e+001f, 5.000000000000000e-001f);
    vs[3].Set(-5.120000076293945e+001f, 5.000000000000000e-001f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}

b2Free(joints);
b2Free(bodies);
joints = NULL;
bodies = NULL;

	}

	static Test* Create()
	{
		return new DumpShell;
	}
};

#endif
