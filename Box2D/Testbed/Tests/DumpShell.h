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
b2Vec2 g(0.000000000000000e+000f, -1.000000000000000e+001f);
m_world->SetGravity(g);
b2Body** bodies = (b2Body**)b2Alloc(6 * sizeof(b2Body*));
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(2.296126633882523e-002f, 2.719760322570801e+001f);
  bd.angle = -7.404115422104951e-006f;
  bd.linearVelocity.Set(1.703398674726486e-002f, -1.535694599151611e+000f);
  bd.angularVelocity = -2.208980731666088e-007f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;

  b2Body* b = m_world->CreateBody(&bd);
  bodies[0] = b;

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 2.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-1.500000000000000e+000f, -1.500000000000000e+000f);
    vs[1].Set(1.500000000000000e+000f, -1.500000000000000e+000f);
    vs[2].Set(1.500000000000000e+000f, 1.500000000000000e+000f);
    vs[3].Set(-1.500000000000000e+000f, 1.500000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[0]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-1.458674087189138e-004f, 9.725848197937012e+000f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(-1.258938689716160e-003f, 5.417145252227783e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(16);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;

  b2Body* b = m_world->CreateBody(&bd);
  bodies[1] = b;

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 2.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-1.500000000000000e+000f, -1.500000000000000e+000f);
    vs[1].Set(1.500000000000000e+000f, -1.500000000000000e+000f);
    vs[2].Set(1.500000000000000e+000f, 1.500000000000000e+000f);
    vs[3].Set(-1.500000000000000e+000f, 1.500000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.473340511322022e+000f, 6.008808135986328e+000f);
  bd.angle = 3.771593272686005e-001f;
  bd.linearVelocity.Set(4.475612163543701e+000f, 7.098069190979004e+000f);
  bd.angularVelocity = 1.195354819297791e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;

  b2Body* b = m_world->CreateBody(&bd);
  bodies[2] = b;

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 2.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.000000000000000e-001f, -4.000000000000000e+000f);
    vs[1].Set(5.000000000000000e-001f, -4.000000000000000e+000f);
    vs[2].Set(5.000000000000000e-001f, 4.000000000000000e+000f);
    vs[3].Set(-5.000000000000000e-001f, 4.000000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[2]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.472161293029785e+000f, 3.646211862564087e+000f);
  bd.angle = 1.025203990936279e+001f;
  bd.linearVelocity.Set(4.478031635284424e+000f, 4.394995212554932e+000f);
  bd.angularVelocity = 3.137226104736328e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;

  b2Body* b = m_world->CreateBody(&bd);
  bodies[3] = b;

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 2.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.000000000000000e-001f, -2.000000000000000e+000f);
    vs[1].Set(5.000000000000000e-001f, -2.000000000000000e+000f);
    vs[2].Set(5.000000000000000e-001f, 2.000000000000000e+000f);
    vs[3].Set(-5.000000000000000e-001f, 2.000000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[3]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
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

  b2Body* b = m_world->CreateBody(&bd);
  bodies[4] = b;

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 0.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2EdgeShape shape;
    shape.m_radius = 9.999999776482582e-003f;
    shape.m_vertex0.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
    shape.m_vertex1.Set(-4.000000000000000e+001f, 0.000000000000000e+000f);
    shape.m_vertex2.Set(4.000000000000000e+001f, 0.000000000000000e+000f);
    shape.m_vertex3.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
    shape.m_hasVertex0 = bool(0);
    shape.m_hasVertex3 = bool(0);

    fd.shape = &shape;

    bodies[4]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
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

  b2Body* b = m_world->CreateBody(&bd);
  bodies[5] = b;

}
{
  b2PrismaticJointDef jd;
  jd.bodyA = bodies[4];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 1.700000000000000e+001f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.localAxisA.Set(0.000000000000000e+000f, 1.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerTranslation = 0.000000000000000e+000f;
  jd.upperTranslation = 0.000000000000000e+000f;
  jd.enableMotor = bool(1);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorForce = 1.000000000000000e+003f;
  m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[2];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 4.000000000000000e+000f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[3];
  jd.bodyB = bodies[2];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 2.000000000000000e+000f);
  jd.localAnchorB.Set(0.000000000000000e+000f, -4.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[4];
  jd.bodyB = bodies[3];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 5.000000000000000e+000f);
  jd.localAnchorB.Set(0.000000000000000e+000f, -2.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(1);
  jd.motorSpeed = 3.141592741012573e+000f;
  jd.maxMotorTorque = 1.000000000000000e+004f;
  m_world->CreateJoint(&jd);
}

	}

	static Test* Create()
	{
		return new DumpShell;
	}
};

#endif
