/*
* Author: Chris Campbell - www.iforce2d.net
*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef IFORCE2D_BUOYANCY_H
#define IFORCE2D_BUOYANCY_H

#include "iforce2d_Buoyancy_functions.h"

class iforce2d_Buoyancy : public Test
{
public:
    iforce2d_Buoyancy()
    {
        //include dumped world data
        #include "buoyancy.cpp"
    }

    void BeginContact(b2Contact* contact)
    {
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();

        //This assumes every sensor fixture is fluid, and will interact
        //with every dynamic body.
        if ( fixtureA->IsSensor() &&
             fixtureB->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.insert( make_pair(fixtureA, fixtureB) );
        else if ( fixtureB->IsSensor() &&
                  fixtureA->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.insert( make_pair(fixtureB, fixtureA) );
    }

    void EndContact(b2Contact* contact)
    {
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();

        //This check should be the same as for BeginContact, but here
        //we remove the fixture pair
        if ( fixtureA->IsSensor() &&
             fixtureB->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.erase( make_pair(fixtureA, fixtureB) );
        else if ( fixtureB->IsSensor() &&
                  fixtureA->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.erase( make_pair(fixtureB, fixtureA) );
    }

    void Step(Settings* settings)
    {
        Test::Step(settings);

        //go through all buoyancy fixture pairs and apply necessary forces
        std::set<fixturePair>::iterator it = m_fixturePairs.begin();
        std::set<fixturePair>::iterator end = m_fixturePairs.end();
        while (it != end) {

            //fixtureA is the fluid
            b2Fixture* fixtureA = it->first;
            b2Fixture* fixtureB = it->second;

            float density = fixtureA->GetDensity();

            std::vector<b2Vec2> intersectionPoints;
            if ( findIntersectionOfFixtures(fixtureA, fixtureB, intersectionPoints) ) {

                //find centroid
                float area = 0;
                b2Vec2 centroid = ComputeCentroid( intersectionPoints, area);

                //apply buoyancy force
                float displacedMass = fixtureA->GetDensity() * area;
                b2Vec2 gravity( 0, -10 );
                b2Vec2 buoyancyForce = displacedMass * -gravity;
                fixtureB->GetBody()->ApplyForce( buoyancyForce, centroid );

                /*
                //simple drag
                //find relative velocity between object and fluid
                b2Vec2 velDir = fixtureB->GetBody()->GetLinearVelocityFromWorldPoint( centroid ) -
                        fixtureA->GetBody()->GetLinearVelocityFromWorldPoint( centroid );
                float vel = velDir.Normalize();

                float dragMod = 1;//adjust as desired
                float dragMag = fixtureA->GetDensity() * vel * vel;
                b2Vec2 dragForce = dragMod * dragMag * -velDir;
                fixtureB->GetBody()->ApplyForce( dragForce, centroid );
                float angularDrag = area * -fixtureB->GetBody()->GetAngularVelocity();
                fixtureB->GetBody()->ApplyTorque( angularDrag );
                */

                //apply complex drag
                float dragMod = 0.25f;//adjust as desired
                float liftMod = 0.25f;//adjust as desired
                float maxDrag = 2000;//adjust as desired
                float maxLift = 500;//adjust as desired
                for (int i = 0; i < intersectionPoints.size(); i++) {
                    b2Vec2 v0 = intersectionPoints[i];
                    b2Vec2 v1 = intersectionPoints[(i+1)%intersectionPoints.size()];
                    b2Vec2 midPoint = 0.5f * (v0+v1);

                    //find relative velocity between object and fluid at edge midpoint
                    b2Vec2 velDir = fixtureB->GetBody()->GetLinearVelocityFromWorldPoint( midPoint ) -
                            fixtureA->GetBody()->GetLinearVelocityFromWorldPoint( midPoint );
                    float vel = velDir.Normalize();

                    b2Vec2 edge = v1 - v0;
                    float edgeLength = edge.Normalize();
                    b2Vec2 normal = b2Cross(-1,edge);
                    float dragDot = b2Dot(normal, velDir);
                    if ( dragDot < 0 )
                        continue;//normal points backwards - this is not a leading edge

                    //apply drag
                    float dragMag = dragDot * dragMod * edgeLength * density * vel * vel;
                    dragMag = b2Min( dragMag, maxDrag );
                    b2Vec2 dragForce = dragMag * -velDir;
                    fixtureB->GetBody()->ApplyForce( dragForce, midPoint );

                    //apply lift
                    float liftDot = b2Dot(edge, velDir);
                    float liftMag =  dragDot * liftDot * liftMod * edgeLength * density * vel * vel;
                    liftMag = b2Min( liftMag, maxLift );
                    b2Vec2 liftDir = b2Cross(1,velDir);
                    b2Vec2 liftForce = liftMag * liftDir;
                    fixtureB->GetBody()->ApplyForce( liftForce, midPoint );
                }

                //draw debug info
                glColor3f(0,1,1);
                glLineWidth(2);
                glBegin(GL_LINE_LOOP);
                for (int i = 0; i < intersectionPoints.size(); i++)
                    glVertex2f( intersectionPoints[i].x, intersectionPoints[i].y );
                glEnd();
                glLineWidth(1);
                /*
                //line showing buoyancy force
                if ( area > 0 ) {
                    glBegin(GL_LINES);
                    glVertex2f( centroid.x, centroid.y );
                    glVertex2f( centroid.x, centroid.y + area );
                    glEnd();
                }*/
            }

            ++it;
        }
    }

    static Test* Create()
    {
        return new iforce2d_Buoyancy;
    }

    std::set<fixturePair> m_fixturePairs;
};

#endif









