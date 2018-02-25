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

#ifndef IFORCE2D_BUOYANCY_BOAT_H
#define IFORCE2D_BUOYANCY_BOAT_H

#include "iforce2d_Buoyancy_functions.h"

#if defined(__APPLE_CC__)

#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#else
#include "glew/glew.h"
#endif

#include "glfw/glfw3.h"

#include "imgui/imgui.h"

#define MAX_BUBBLES 32

//random number between 0 and 1
float rnd_1()
{
    return rand() / (float)RAND_MAX;
}

class iforce2d_Buoyancy_boat : public Test
{
public:
    iforce2d_Buoyancy_boat()
    {
        //include dumped world data
        #include "buoyancy2.cpp"

        //set up some irregular lines to help visualize speed
        for (int i = 0; i < 100; i++)
            m_groundPoints[i].Set( i * 3, rnd_1() * 3 );

        m_boatBody = NULL;
        m_boatDrive = false;

        //initialize bubble particles array
        memset(m_bubbleBodies, 0, sizeof(m_bubbleBodies));
        m_stepsSinceLastBubble = 0;
        m_nextBubbleIndex = 0;
    }

    void BeginContact(b2Contact* contact)
    {
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();

        //This assumes every sensor fixture is fluid, and will interact
        //with every dynamic body.
        if ( fixtureA->IsSensor() &&
             fixtureB->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.insert( std::make_pair(fixtureA, fixtureB) );
        else if ( fixtureB->IsSensor() &&
                  fixtureA->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.insert( std::make_pair(fixtureB, fixtureA) );

        //The boat hull will be the first fixture to touch the water, so we
        //can set a reference to it here. Usually you would have a better
        //method for finding bodies within a scene :)
        if ( fixtureA->IsSensor() &&
             fixtureB->GetBody()->GetType() == b2_dynamicBody && !m_boatBody )
            m_boatBody = fixtureB->GetBody();
        else
            if ( fixtureB->IsSensor() &&
                 fixtureA->GetBody()->GetType() == b2_dynamicBody && !m_boatBody )
                m_boatBody = fixtureA->GetBody();
    }

    void EndContact(b2Contact* contact)
    {
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();

        //This check should be the same as for BeginContact, but here
        //we remove the fixture pair
        if ( fixtureA->IsSensor() &&
             fixtureB->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.erase( std::make_pair(fixtureA, fixtureB) );
        else if ( fixtureB->IsSensor() &&
                  fixtureA->GetBody()->GetType() == b2_dynamicBody )
            m_fixturePairs.erase( std::make_pair(fixtureB, fixtureA) );
    }

    void Keyboard(int key)
    {
        switch (key) {
        case GLFW_KEY_A : m_boatDrive = true; break;
        default: Test::Keyboard(key);
        }
    }

    void KeyboardUp(int key)
    {
        switch (key) {
        case GLFW_KEY_A : m_boatDrive = false; break;
        default: Test::Keyboard(key);
        }
    }

    void Step(Settings* settings)
    {
        g_debugDraw.DrawString(5, m_textLine, "Press a to drive the boat");
        m_textLine += 15;

        Test::Step(settings);

        //update boat forces, bubble particles, check if boat should warp to left side of scene
        if ( m_boatBody ) {

            //if currently driving and engine is underwater, apply drive force and make bubbles
            if ( m_boatDrive && m_boatBody->GetPosition().y < 10.1 ) {

                //a little downforce helps keep the engine underwater
                m_boatBody->ApplyForce( m_boatBody->GetWorldVector( b2Vec2(400,-25) ), m_boatBody->GetPosition(), true );

                if ( m_stepsSinceLastBubble > 0 ) {

                    b2Vec2 bubblePos = m_boatBody->GetPosition();//the boat body has 0,0 in local coords at the engine position
                    b2Vec2 bubbleDir = m_boatBody->GetWorldVector( b2Vec2(-50,-20-rnd_1()*40) );

                    b2Body* b = m_bubbleBodies[m_nextBubbleIndex];
                    if ( !b ) {
                        b2BodyDef bd;
                        bd.type = b2_dynamicBody;
                        b = m_bubbleBodies[m_nextBubbleIndex] = m_world->CreateBody(&bd);

                        b2PolygonShape ps;
                        ps.SetAsBox(0.25,0.25);
                        b->CreateFixture( &ps, 1.75 );//water is density 2
                    }
                    b->SetTransform( bubblePos, rnd_1() );
                    b->SetLinearVelocity( bubbleDir );

                    m_stepsSinceLastBubble = 0;
                    m_nextBubbleIndex++;
                    m_nextBubbleIndex %= MAX_BUBBLES;
                }
            }
            m_stepsSinceLastBubble++;

            //adjust view center as boat moves
//            b2Vec2 oldViewCenter = settings->viewCenter;
//            b2Vec2 posOfBoatVerySoon = m_boatBody->GetWorldCenter() + 0.25f * m_boatBody->GetLinearVelocity();
//            settings->viewCenter = 0.9f * oldViewCenter + 0.1f * posOfBoatVerySoon;
//
//            b2Vec2 pos = m_boatBody->GetPosition();
//            if ( pos.x > 450 ) {
//                //warp all dynamic bodies (there are only two) back to the left
//                for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext()) {
//                    if ( b->GetType() == b2_dynamicBody )
//                        b->SetTransform( b->GetPosition() + b2Vec2(-900,0), b->GetAngle() );
//                }
//                settings->viewCenter.x -= 900;
//            }
        }

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
                fixtureB->GetBody()->ApplyForce( buoyancyForce, centroid, true );

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
                    fixtureB->GetBody()->ApplyForce( dragForce, midPoint, true );

                    //apply lift
                    float liftDot = b2Dot(edge, velDir);
                    float liftMag =  dragDot * liftDot * liftMod * edgeLength * density * vel * vel;
                    liftMag = b2Min( liftMag, maxLift );
                    b2Vec2 liftDir = b2Cross(1,velDir);
                    b2Vec2 liftForce = liftMag * liftDir;
                    fixtureB->GetBody()->ApplyForce( liftForce, midPoint, true );
                }
            }

            ++it;
        }

        //draw repeating ground bumps
        glColor3f(0.5, 0.9, 0.5);
        for (int k = -2; k < 2; k++) {
            glPushMatrix();
            glTranslatef(k * 300, 0, 0);
            glBegin(GL_LINES);
            for (int i = 0; i < 99; i++) {
                glVertex2fv( (GLfloat*)&m_groundPoints[i] );
                glVertex2fv( (GLfloat*)&m_groundPoints[i+1] );
            }
            glVertex2fv( (GLfloat*)&m_groundPoints[99] );
            glVertex2f( m_groundPoints[0].x + 300, m_groundPoints[0].y );
            glEnd();
            glPopMatrix();
        }
    }

    static Test* Create()
    {
        return new iforce2d_Buoyancy_boat;
    }

    std::set<fixturePair> m_fixturePairs;

    b2Body* m_boatBody;
    bool m_boatDrive;

    b2Vec2 m_groundPoints[100];

    b2Body* m_bubbleBodies[MAX_BUBBLES];
    int m_stepsSinceLastBubble;
    int m_nextBubbleIndex;
};

#endif









