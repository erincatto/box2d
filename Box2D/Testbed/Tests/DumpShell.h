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
b2Body** bodies = (b2Body**)b2Alloc(8 * sizeof(b2Body*));
b2Joint** joints = (b2Joint**)b2Alloc(8 * sizeof(b2Joint*));
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(2.500000000000000e+000f, 6.995135784149170e+000f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[0] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.000000000000000e-001f, -5.000000000000000e+000f);
    vs[1].Set(5.000000000000000e-001f, -5.000000000000000e+000f);
    vs[2].Set(5.000000000000000e-001f, 5.000000000000000e+000f);
    vs[3].Set(-5.000000000000000e-001f, 5.000000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[0]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(0.000000000000000e+000f, 1.200000000000000e+001f);
  bd.angle = -2.502432107925415e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[1] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 2.000000000000000e+000f;
    shape.m_p.Set(0.000000000000000e+000f, 0.000000000000000e+000f);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-3.000000000000000e+000f, 1.200000000000000e+001f);
  bd.angle = 5.004864215850830e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[2] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 1.000000000000000e+000f;
    shape.m_p.Set(0.000000000000000e+000f, 0.000000000000000e+000f);

    fd.shape = &shape;

    bodies[2]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.000000000000000e+001f, 5.999999523162842e+000f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[3] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 2.000000000000000e+000f;
    shape.m_p.Set(0.000000000000000e+000f, 0.000000000000000e+000f);

    fd.shape = &shape;

    bodies[3]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.000000000000000e+001f, 8.000000000000000e+000f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[4] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 5.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.000000000000000e-001f, -5.000000000000000e+000f);
    vs[1].Set(5.000000000000000e-001f, -5.000000000000000e+000f);
    vs[2].Set(5.000000000000000e-001f, 5.000000000000000e+000f);
    vs[3].Set(-5.000000000000000e-001f, 5.000000000000000e+000f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[4]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(1.000000000000000e+001f, 9.000000000000000e+000f);
  bd.angle = 0.000000000000000e+000f;
  bd.linearVelocity.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  bd.angularVelocity = 0.000000000000000e+000f;
  bd.linearDamping = 0.000000000000000e+000f;
  bd.angularDamping = 0.000000000000000e+000f;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[5] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-001f;
    fd.restitution = 0.000000000000000e+000f;
    fd.density = 0.000000000000000e+000f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 1.000000000000000e+000f;
    shape.m_p.Set(0.000000000000000e+000f, 0.000000000000000e+000f);

    fd.shape = &shape;

    bodies[5]->CreateFixture(&fd);
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
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+000f;
  bodies[6] = m_world->CreateBody(&bd);

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
    shape.m_vertex1.Set(5.000000000000000e+001f, 0.000000000000000e+000f);
    shape.m_vertex2.Set(-5.000000000000000e+001f, 0.000000000000000e+000f);
    shape.m_vertex3.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
    shape.m_hasVertex0 = bool(0);
    shape.m_hasVertex3 = bool(0);

    fd.shape = &shape;

    bodies[6]->CreateFixture(&fd);
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
  bodies[7] = m_world->CreateBody(&bd);

}
{
  b2PrismaticJointDef jd;
  jd.bodyA = bodies[6];
  jd.bodyB = bodies[0];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(2.500000000000000e+000f, 1.200000000000000e+001f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.localAxisA.Set(0.000000000000000e+000f, 1.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(1);
  jd.lowerTranslation = -5.000000000000000e+000f;
  jd.upperTranslation = 5.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorForce = 0.000000000000000e+000f;
  joints[2] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[6];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 1.200000000000000e+001f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  joints[3] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[6];
  jd.bodyB = bodies[2];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(-3.000000000000000e+000f, 1.200000000000000e+001f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  joints[4] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[4];
  jd.bodyB = bodies[3];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, -2.000000000000000e+000f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  joints[6] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[4];
  jd.bodyB = bodies[5];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+000f, 1.000000000000000e+000f);
  jd.localAnchorB.Set(0.000000000000000e+000f, 0.000000000000000e+000f);
  jd.referenceAngle = 0.000000000000000e+000f;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0.000000000000000e+000f;
  jd.upperAngle = 0.000000000000000e+000f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+000f;
  jd.maxMotorTorque = 0.000000000000000e+000f;
  joints[7] = m_world->CreateJoint(&jd);
}
{
  b2GearJointDef jd;
  jd.bodyA = bodies[1];
  jd.bodyB = bodies[0];
  jd.collideConnected = bool(0);
  jd.joint1 = joints[3];
  jd.joint2 = joints[2];
  jd.ratio = -5.000000000000000e-001f;
  joints[0] = m_world->CreateJoint(&jd);
}
{
  b2GearJointDef jd;
  jd.bodyA = bodies[2];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.joint1 = joints[4];
  jd.joint2 = joints[3];
  jd.ratio = 2.000000000000000e+000f;
  joints[1] = m_world->CreateJoint(&jd);
}
{
  b2GearJointDef jd;
  jd.bodyA = bodies[5];
  jd.bodyB = bodies[3];
  jd.collideConnected = bool(0);
  jd.joint1 = joints[7];
  jd.joint2 = joints[6];
  jd.ratio = 2.000000000000000e+000f;
  joints[5] = m_world->CreateJoint(&jd);
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
