//Source code dump of Box2D scene: buoyancy2.rube
//
//  Created by R.U.B.E 1.1.0
//  Using Box2D version 2.3.0
//  Wed January 16 2013 22:07:41
//
//  This code is originally intended for use in the Box2D testbed,
//  but you can easily use it in other applications by providing
//  a b2World for use as the 'm_world' variable in the code below.

b2Vec2 g(0.000000000000000e+00f, -1.000000000000000e+01f);
m_world->SetGravity(g);
b2Body** bodies = (b2Body**)b2Alloc(8 * sizeof(b2Body*));
b2Joint** joints = (b2Joint**)b2Alloc(8 * sizeof(b2Joint*));
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(1.451381206512451e+00f, 1.672816085815430e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[0] = m_world->CreateBody(&bd);

}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-6.430840015411377e+00f, 1.568270683288574e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[1] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.764345169067383e-02f, -5.531330108642578e-01f);
    vs[1].Set(1.764345169067383e-02f, 5.531330108642578e-01f);
    vs[2].Set(-1.764392852783203e-02f, 5.531330108642578e-01f);
    vs[3].Set(-1.764392852783203e-02f, -5.531330108642578e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-6.430990219116211e+00f, 1.464763545989990e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[2] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(3.197097778320312e-02f, -5.531330108642578e-01f);
    vs[1].Set(3.197097778320312e-02f, 5.531339645385742e-01f);
    vs[2].Set(-3.197050094604492e-02f, 5.531339645385742e-01f);
    vs[3].Set(-3.197050094604492e-02f, -5.531330108642578e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[2]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-6.430990219116211e+00f, 1.369460010528564e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[3] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(4.803900048136711e-02f, -4.919070005416870e-01f);
    vs[1].Set(4.803900048136711e-02f, 4.919080138206482e-01f);
    vs[2].Set(-4.803850129246712e-02f, 4.919080138206482e-01f);
    vs[3].Set(-4.803850129246712e-02f, -4.919070005416870e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[3]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(2.485094757080078e+02f, 0.000000000000000e+00f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[4] = m_world->CreateBody(&bd);

}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-2.250442504882812e+00f, 1.404000377655029e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[5] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 5.000000000000000e-01f;
    shape.m_p.Set(0.000000000000000e+00f, 0.000000000000000e+00f);

    fd.shape = &shape;

    bodies[5]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(2.373831787109375e+02f, 1.750797271728516e+00f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[6] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 2.000000000000000e+00f;
    fd.isSensor = bool(1);
    fd.filter.categoryBits = uint16(2);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(3.626168212890625e+02f, -2.446432189941406e+02f);
    vs[1].Set(3.626168212890625e+02f, 8.249202728271484e+00f);
    vs[2].Set(-8.373831787109375e+02f, 8.249202728271484e+00f);
    vs[3].Set(-8.373831787109375e+02f, -2.446432189941406e+02f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[6]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-7.380447387695312e+00f, 1.079848384857178e+01f);
  bd.angle = 0.000000000000000e+00f;
  bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  bd.angularVelocity = 0.000000000000000e+00f;
  bd.linearDamping = 0.000000000000000e+00f;
  bd.angularDamping = 0.000000000000000e+00f;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.active = bool(32);
  bd.gravityScale = 1.000000000000000e+00f;
  bodies[7] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.013520717620850e+00f, 1.934565544128418e+00f);
    vs[1].Set(1.013520717620850e+00f, 2.470788955688477e+00f);
    vs[2].Set(8.867602348327637e-01f, 2.470788955688477e+00f);
    vs[3].Set(8.867602348327637e-01f, 1.934565544128418e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 8.500000238418579e-01f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65534);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.528183174133301e+01f, 1.991326332092285e+00f);
    vs[1].Set(1.105670034885406e-01f, 1.926270008087158e+00f);
    vs[2].Set(-7.963920235633850e-01f, -5.357021093368530e-02f);
    vs[3].Set(5.319980144500732e+00f, -2.321330010890961e-01f);
    vs[4].Set(1.177239990234375e+01f, 2.138149924576283e-02f);
    vs[5].Set(1.415202140808105e+01f, 7.623749971389771e-01f);
    shape.Set(vs, 6);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 3.000000000000000e+01f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2CircleShape shape;
    shape.m_radius = 2.314362227916718e-01f;
    shape.m_p.Set(0.000000000000000e+00f, 0.000000000000000e+00f);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(0);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.060933017730713e+01f, 1.968769788742065e+00f);
    vs[1].Set(8.380570411682129e+00f, 3.367011070251465e+00f);
    vs[2].Set(6.143830299377441e+00f, 3.367011070251465e+00f);
    vs[3].Set(5.670685768127441e+00f, 1.950999021530151e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 9.999999776482582e-03f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(5.426879882812500e+00f, 2.667344093322754e+00f);
    vs[1].Set(4.776306152343750e+00f, 2.667344093322754e+00f);
    vs[2].Set(4.679199218750000e+00f, 1.952551841735840e+00f);
    vs[3].Set(5.346374511718750e+00f, 1.952551841735840e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
}
{
  b2DistanceJointDef jd;
  jd.bodyA = bodies[7];
  jd.bodyB = bodies[3];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(-8.293628692626953e-03f, 2.902970314025879e+00f);
  jd.localAnchorB.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  jd.length = 9.604415893554688e-01f;
  jd.frequencyHz = 4.000000000000000e+00f;
  jd.dampingRatio = 5.000000000000000e-01f;
  joints[0] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[7];
  jd.bodyB = bodies[3];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(9.498000144958496e-01f, 2.437500000000000e+00f);
  jd.localAnchorB.Set(3.418920096009970e-04f, -4.586170017719269e-01f);
  jd.referenceAngle = 0.000000000000000e+00f;
  jd.enableLimit = bool(1);
  jd.lowerAngle = -5.235987901687622e-01f;
  jd.upperAngle = 5.235987901687622e-01f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+00f;
  jd.maxMotorTorque = 0.000000000000000e+00f;
  joints[1] = m_world->CreateJoint(&jd);
}
{
  b2DistanceJointDef jd;
  jd.bodyA = bodies[3];
  jd.bodyB = bodies[2];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(-9.550929069519043e-01f, 9.493741989135742e-01f);
  jd.localAnchorB.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  jd.length = 9.530349969863892e-01f;
  jd.frequencyHz = 8.000000000000000e+00f;
  jd.dampingRatio = 5.000000000000000e-01f;
  joints[2] = m_world->CreateJoint(&jd);
}
{
  b2DistanceJointDef jd;
  jd.bodyA = bodies[2];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(-9.494266510009766e-01f, 1.029441833496094e+00f);
  jd.localAnchorB.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
  jd.length = 9.506250619888306e-01f;
  jd.frequencyHz = 4.500000000000000e+00f;
  jd.dampingRatio = 5.000000000000000e-01f;
  joints[3] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[2];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(7.486339745810255e-05f, 5.175359845161438e-01f);
  jd.localAnchorB.Set(-7.534030009992421e-05f, -5.175349712371826e-01f);
  jd.referenceAngle = 0.000000000000000e+00f;
  jd.enableLimit = bool(1);
  jd.lowerAngle = -6.981316804885864e-01f;
  jd.upperAngle = 6.981316804885864e-01f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+00f;
  jd.maxMotorTorque = 0.000000000000000e+00f;
  joints[4] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[3];
  jd.bodyB = bodies[2];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0.000000000000000e+00f, 4.459049999713898e-01f);
  jd.localAnchorB.Set(0.000000000000000e+00f, -5.071309804916382e-01f);
  jd.referenceAngle = 0.000000000000000e+00f;
  jd.enableLimit = bool(1);
  jd.lowerAngle = -5.235987901687622e-01f;
  jd.upperAngle = 5.235987901687622e-01f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+00f;
  jd.maxMotorTorque = 0.000000000000000e+00f;
  joints[5] = m_world->CreateJoint(&jd);
}
{
  b2DistanceJointDef jd;
  jd.bodyA = bodies[7];
  jd.bodyB = bodies[5];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(5.124664306640625e+00f, 3.237409591674805e+00f);
  jd.localAnchorB.Set(-6.856380105018616e-01f, 7.816310040652752e-03f);
  jd.length = 6.806582808494568e-01f;
  jd.frequencyHz = 1.500000000000000e+00f;
  jd.dampingRatio = 5.000000000000000e-01f;
  joints[6] = m_world->CreateJoint(&jd);
}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[7];
  jd.bodyB = bodies[5];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(5.092770099639893e+00f, 2.584709882736206e+00f);
  jd.localAnchorB.Set(-3.643799945712090e-02f, -6.567929983139038e-01f);
  jd.referenceAngle = 0.000000000000000e+00f;
  jd.enableLimit = bool(1);
  jd.lowerAngle = -6.071379780769348e-01f;
  jd.upperAngle = 3.783507049083710e-01f;
  jd.enableMotor = bool(0);
  jd.motorSpeed = 0.000000000000000e+00f;
  jd.maxMotorTorque = 0.000000000000000e+00f;
  joints[7] = m_world->CreateJoint(&jd);
}
b2Free(joints);
b2Free(bodies);
joints = NULL;
bodies = NULL;

