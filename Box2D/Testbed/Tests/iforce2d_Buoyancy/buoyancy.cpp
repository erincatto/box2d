//Source code dump of Box2D scene: buoyancy.rube
//
//  Created by R.U.B.E 1.1.0
//  Using Box2D version 2.3.0
//  Wed January 16 2013 17:17:04
//
//  This code is originally intended for use in the Box2D testbed,
//  but you can easily use it in other applications by providing
//  a b2World for use as the 'm_world' variable in the code below.

b2Vec2 g(0.000000000000000e+00f, -1.000000000000000e+01f);
m_world->SetGravity(g);
b2Body** bodies = (b2Body**)b2Alloc(18 * sizeof(b2Body*));
b2Joint** joints = (b2Joint**)b2Alloc(0 * sizeof(b2Joint*));
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-1.015610504150391e+01f, 1.752653121948242e+00f);
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

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.000000000000000e+00f, -1.000000000000000e+00f);
    vs[1].Set(1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[2].Set(-1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[3].Set(-1.000000000000000e+00f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[0]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
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
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(2.000000000000000e+01f, -5.000000000000000e-01f);
    vs[1].Set(2.000000000000000e+01f, 5.000000000000000e-01f);
    vs[2].Set(-5.485061645507812e+01f, 5.000000000000000e-01f);
    vs[3].Set(-5.485061645507812e+01f, -5.000000000000000e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-2.811175155639648e+01f, 3.515510559082031e+01f);
  bd.angle = -7.853981852531433e-01f;
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
    fd.density = 4.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(9.294319152832031e-02f, -1.000000000000000e+00f);
    vs[1].Set(9.294319152832031e-02f, 1.000000000000000e+00f);
    vs[2].Set(-9.816074371337891e-02f, 1.000000000000000e+00f);
    vs[3].Set(-9.816074371337891e-02f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[2]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.343957805633545e+01f, 1.412565040588379e+01f);
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
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.000000000000000e+00f, 0.000000000000000e+00f);
    vs[1].Set(7.071059942245483e-01f, 7.071075439453125e-01f);
    vs[2].Set(0.000000000000000e+00f, 1.000000000000000e+00f);
    vs[3].Set(-7.071059942245483e-01f, 7.071075439453125e-01f);
    vs[4].Set(-1.000000000000000e+00f, 0.000000000000000e+00f);
    vs[5].Set(-7.071059942245483e-01f, -7.071075439453125e-01f);
    vs[6].Set(0.000000000000000e+00f, -1.000000000000000e+00f);
    vs[7].Set(7.071080207824707e-01f, -7.071075439453125e-01f);
    shape.Set(vs, 8);

    fd.shape = &shape;

    bodies[3]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(-1.112629318237305e+01f, 1.750797271728516e+00f);
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

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 2.000000000000000e+00f;
    fd.isSensor = bool(1);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.544052124023438e-01f, -1.515119433403015e+00f);
    vs[1].Set(-5.544052124023438e-01f, 7.516476631164551e+00f);
    vs[2].Set(-4.341683959960938e+01f, 7.516476631164551e+00f);
    vs[3].Set(-4.341683959960938e+01f, -1.515119433403015e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[4]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-2.586175537109375e+01f, 3.515510559082031e+01f);
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
    fd.density = 4.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(9.294319152832031e-02f, -1.000000000000000e+00f);
    vs[1].Set(9.294319152832031e-02f, 1.000000000000000e+00f);
    vs[2].Set(-9.816074371337891e-02f, 1.000000000000000e+00f);
    vs[3].Set(-9.816074371337891e-02f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[5]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-2.373113632202148e+01f, 3.515510177612305e+01f);
  bd.angle = 1.570796370506287e+00f;
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
    fd.density = 4.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(9.294319152832031e-02f, -1.000000000000000e+00f);
    vs[1].Set(9.294319152832031e-02f, 1.000000000000000e+00f);
    vs[2].Set(-9.816074371337891e-02f, 1.000000000000000e+00f);
    vs[3].Set(-9.816074371337891e-02f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[6]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(2.000000000000000e+01f, 1.999999809265137e+01f);
  bd.angle = -1.570796370506287e+00f;
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
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(2.000000000000000e+01f, -5.000000000000000e-01f);
    vs[1].Set(2.000000000000000e+01f, 5.000000000000000e-01f);
    vs[2].Set(-2.000000000000000e+01f, 5.000000000000000e-01f);
    vs[3].Set(-2.000000000000000e+01f, -5.000000000000000e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[7]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.332015132904053e+01f, 2.823232650756836e+00f);
  bd.angle = -3.126464605331421e+00f;
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
  bodies[8] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(5.458000659942627e+00f, 1.932244658470154e+00f);
    vs[1].Set(4.231588363647461e+00f, 1.035382747650146e+00f);
    vs[2].Set(4.136718273162842e+00f, -1.974030256271362e+00f);
    vs[3].Set(5.366768836975098e+00f, -2.028761625289917e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[8]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(5.458000659942627e+00f, 1.932244658470154e+00f);
    vs[1].Set(-5.803962707519531e+00f, 2.050609827041626e+00f);
    vs[2].Set(-4.514650821685791e+00f, 1.105765700340271e+00f);
    vs[3].Set(4.231588363647461e+00f, 1.035382747650146e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[8]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-4.514650821685791e+00f, 1.105765700340271e+00f);
    vs[1].Set(-5.803962707519531e+00f, 2.050609827041626e+00f);
    vs[2].Set(-5.854610443115234e+00f, -1.812623858451843e+00f);
    vs[3].Set(-4.601483821868896e+00f, -1.712271928787231e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[8]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 2.000000000000000e+00f;
    fd.isSensor = bool(1);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(4.919295787811279e+00f, 1.530433893203735e+00f);
    vs[1].Set(-5.178795337677002e+00f, 1.683210015296936e+00f);
    vs[2].Set(-5.225200653076172e+00f, -1.384050726890564e+00f);
    vs[3].Set(4.872890472412109e+00f, -1.536840081214905e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[8]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-5.409978389739990e+00f, 1.763031005859375e+00f);
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
  bodies[9] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 2.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.000000000000000e+00f, -1.000000000000000e+00f);
    vs[1].Set(1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[2].Set(-1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[3].Set(-1.000000000000000e+00f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[9]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-6.527210474014282e-01f, 2.409656524658203e+00f);
  bd.angle = 3.141592741012573e+00f;
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
  bodies[10] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(9.797759056091309e-01f, -2.595954895019531e+00f);
    vs[1].Set(9.797759056091309e-01f, 1.404045104980469e+00f);
    vs[2].Set(-1.020224094390869e+00f, 1.404045104980469e+00f);
    vs[3].Set(-1.020224094390869e+00f, -5.959548950195312e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[10]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-4.322952270507812e+01f, 1.334864711761475e+01f);
  bd.angle = 3.824857473373413e-01f;
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
  bodies[11] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(2.595919609069824e+00f, -1.319291234016418e+00f);
    vs[1].Set(-2.777174711227417e+00f, 9.063234329223633e-01f);
    vs[2].Set(-1.861940383911133e+00f, -4.768543243408203e-01f);
    vs[3].Set(-1.460509777069092e+00f, -1.000003814697266e+00f);
    vs[4].Set(-9.373636245727539e-01f, -1.401435852050781e+00f);
    vs[5].Set(-3.281402587890625e-01f, -1.653779983520508e+00f);
    vs[6].Set(3.256297111511230e-01f, -1.739847183227539e+00f);
    vs[7].Set(9.794044494628906e-01f, -1.653779983520508e+00f);
    shape.Set(vs, 8);

    fd.shape = &shape;

    bodies[11]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(1.292107200622559e+01f, 9.424398422241211e+00f);
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
  bodies[12] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.000000000000000e+00f, 0.000000000000000e+00f);
    vs[1].Set(7.071059942245483e-01f, 7.071075439453125e-01f);
    vs[2].Set(0.000000000000000e+00f, 1.000000000000000e+00f);
    vs[3].Set(-7.071059942245483e-01f, 7.071075439453125e-01f);
    vs[4].Set(-1.000000000000000e+00f, 0.000000000000000e+00f);
    vs[5].Set(-7.071059942245483e-01f, -7.071075439453125e-01f);
    vs[6].Set(0.000000000000000e+00f, -1.000000000000000e+00f);
    vs[7].Set(7.071080207824707e-01f, -7.071075439453125e-01f);
    shape.Set(vs, 8);

    fd.shape = &shape;

    bodies[12]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(-2.000000000000000e+01f, 2.000000000000000e+01f);
  bd.angle = -1.570796370506287e+00f;
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
  bodies[13] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(2.000000190734863e+01f, -3.535061645507812e+01f);
    vs[1].Set(2.000000190734863e+01f, -3.435061645507812e+01f);
    vs[2].Set(-1.999999809265137e+01f, -3.435061645507812e+01f);
    vs[3].Set(-1.999999809265137e+01f, -3.535061645507812e+01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[13]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(6.069438457489014e+00f, 3.902924537658691e+00f);
  bd.angle = 1.570796370506287e+00f;
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
  bodies[14] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(3.146587848663330e+00f, 5.416240692138672e-01f);
    vs[1].Set(1.722242832183838e+00f, -4.027786254882812e-01f);
    vs[2].Set(1.737724781036377e+00f, -1.532964706420898e+00f);
    vs[3].Set(3.115623950958252e+00f, -1.532964706420898e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[14]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(3.146587848663330e+00f, 5.416240692138672e-01f);
    vs[1].Set(-3.167480945587158e+00f, 5.974674224853516e-01f);
    vs[2].Set(-1.167480945587158e+00f, -4.182605743408203e-01f);
    vs[3].Set(1.722242832183838e+00f, -4.027786254882812e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[14]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-1.167480945587158e+00f, -1.402532577514648e+00f);
    vs[1].Set(-1.167480945587158e+00f, -4.182605743408203e-01f);
    vs[2].Set(-3.167480945587158e+00f, 5.974674224853516e-01f);
    vs[3].Set(-3.167480945587158e+00f, -1.402532577514648e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[14]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-7.782896041870117e+00f, 1.825901985168457e+00f);
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
  bodies[15] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.500000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.000000000000000e+00f, -1.000000000000000e+00f);
    vs[1].Set(1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[2].Set(-1.000000000000000e+00f, 1.000000000000000e+00f);
    vs[3].Set(-1.000000000000000e+00f, -1.000000000000000e+00f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[15]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0.000000000000000e+00f, 4.000000000000000e+01f);
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
  bodies[16] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(2.000000000000000e+01f, -5.000000000000000e-01f);
    vs[1].Set(2.000000000000000e+01f, 5.000000000000000e-01f);
    vs[2].Set(-5.485061645507812e+01f, 5.000000000000000e-01f);
    vs[3].Set(-5.485061645507812e+01f, -5.000000000000000e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[16]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(8.500895500183105e-01f, 2.240486145019531e+00f);
  bd.angle = 3.141592741012573e+00f;
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
  bodies[17] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-5.787930488586426e-01f, 1.404045104980469e+00f);
    vs[1].Set(-1.020224094390869e+00f, 1.404045104980469e+00f);
    vs[2].Set(-1.020224094390869e+00f, -5.959548950195312e-01f);
    vs[3].Set(-5.787935256958008e-01f, -4.697570800781250e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[17]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 2.000000029802322e-01f;
    fd.restitution = 0.000000000000000e+00f;
    fd.density = 1.000000000000000e+00f;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(1.554737567901611e+00f, -3.189464569091797e+00f);
    vs[1].Set(1.554737567901611e+00f, -2.628318786621094e+00f);
    vs[2].Set(-5.787935256958008e-01f, -4.697570800781250e-01f);
    vs[3].Set(-1.020224094390869e+00f, -5.959548950195312e-01f);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[17]->CreateFixture(&fd);
  }
}
b2Free(joints);
b2Free(bodies);
joints = NULL;
bodies = NULL;

