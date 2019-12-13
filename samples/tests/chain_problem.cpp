/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef CHAIN_PROBLEM_H
#define CHAIN_PROBLEM_H

class chainProblem : public Test
{
public:
    chainProblem()
    {
        //dump
        {
            b2Vec2 g(0.000000000000000e+00f, -1.000000000000000e+01f);
            m_world->SetGravity(g);
            b2Body** bodies = (b2Body**)b2Alloc(2 * sizeof(b2Body*));
            b2Joint** joints = (b2Joint**)b2Alloc(0 * sizeof(b2Joint*));
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
                bodies[0] = m_world->CreateBody(&bd);

                {
                    b2FixtureDef fd;
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 0.000000000000000e+00f;
                    fd.isSensor = bool(0);
                    fd.filter.categoryBits = uint16(1);
                    fd.filter.maskBits = uint16(65535);
                    fd.filter.groupIndex = int16(0);
                    b2ChainShape shape;
                    b2Vec2 vs[3];
                    vs[0].Set(0.000000000000000e+00f, 1.000000000000000e+00f);
                    vs[1].Set(0.000000000000000e+00f, 0.000000000000000e+00f);
                    vs[2].Set(4.000000000000000e+00f, 0.000000000000000e+00f);
                    shape.CreateChain(vs, 3);
                    shape.m_prevVertex.Set(4.719737010713663e-34f, 8.266340761211261e-34f);
                    shape.m_nextVertex.Set(1.401298464324817e-45f, 8.266340761211261e-34f);
                    shape.m_hasPrevVertex = bool(0);
                    shape.m_hasNextVertex = bool(0);

                    fd.shape = &shape;

                    bodies[0]->CreateFixture(&fd);
                }
            }
            {
                b2BodyDef bd;
                bd.type = b2BodyType(2);
                bd.position.Set(6.033980250358582e-01f, 3.028350114822388e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity.Set(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = bool(4);
                bd.awake = bool(2);
                bd.fixedRotation = bool(0);
                bd.bullet = bool(1);
                bd.active = bool(32);
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[1] = m_world->CreateBody(&bd);

                {
                    b2FixtureDef fd;
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 0.000000000000000e+00f;
                    fd.density = 1.000000000000000e+01f;
                    fd.isSensor = bool(0);
                    fd.filter.categoryBits = uint16(1);
                    fd.filter.maskBits = uint16(65535);
                    fd.filter.groupIndex = int16(0);
                    b2PolygonShape shape;
                    b2Vec2 vs[8];
                    vs[0].Set(5.000000000000000e-01f, -3.000000000000000e+00f);
                    vs[1].Set(5.000000000000000e-01f, 3.000000000000000e+00f);
                    vs[2].Set(-5.000000000000000e-01f, 3.000000000000000e+00f);
                    vs[3].Set(-5.000000000000000e-01f, -3.000000000000000e+00f);
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
    }

    static Test* Create()
    {
        return new chainProblem;
    }

};

#endif
