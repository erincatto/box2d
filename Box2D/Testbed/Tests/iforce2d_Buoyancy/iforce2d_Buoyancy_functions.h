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

#ifndef IFORCE2D_BUOYANCY_FUNCTIONS_H
#define IFORCE2D_BUOYANCY_FUNCTIONS_H

// This file contains the support functions for finding the intersecting portion
// of two polygon fixtures.

#include <set>
#include <vector>
typedef std::pair<b2Fixture*, b2Fixture*> fixturePair;

static b2Vec2 ComputeCentroid(std::vector<b2Vec2> vs, float& area)
{
    int count = (int)vs.size();
    b2Assert(count >= 3);

    b2Vec2 c; c.Set(0.0f, 0.0f);
    area = 0.0f;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    b2Vec2 pRef(0.0f, 0.0f);

    const float32 inv3 = 1.0f / 3.0f;

    for (int32 i = 0; i < count; ++i)
    {
        // Triangle vertices.
        b2Vec2 p1 = pRef;
        b2Vec2 p2 = vs[i];
        b2Vec2 p3 = i + 1 < count ? vs[i+1] : vs[0];

        b2Vec2 e1 = p2 - p1;
        b2Vec2 e2 = p3 - p1;

        float32 D = b2Cross(e1, e2);

        float32 triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        c += triangleArea * inv3 * (p1 + p2 + p3);
    }

    // Centroid
    if (area > b2_epsilon)
        c *= 1.0f / area;
    else
        area = 0;
    return c;
}

bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p) {
    return (cp2.x-cp1.x)*(p.y-cp1.y) > (cp2.y-cp1.y)*(p.x-cp1.x);
}

b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e) {
    b2Vec2 dc( cp1.x - cp2.x, cp1.y - cp2.y );
    b2Vec2 dp( s.x - e.x, s.y - e.y );
    float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
    float n2 = s.x * e.y - s.y * e.x;
    float n3 = 1.0 / (dc.x * dp.y - dc.y * dp.x);
    return b2Vec2( (n1*dp.x - n2*dc.x) * n3, (n1*dp.y - n2*dc.y) * n3);
}

//http://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#JavaScript
bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, std::vector<b2Vec2>& outputVertices)
{
    //currently this only handles polygon vs polygon
    if ( fA->GetShape()->GetType() != b2Shape::e_polygon ||
         fB->GetShape()->GetType() != b2Shape::e_polygon )
        return false;

    b2PolygonShape* polyA = (b2PolygonShape*)fA->GetShape();
    b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();

    //fill 'subject polygon' from fixtureA polygon
    for (int i = 0; i < polyA->m_count; i++)
        outputVertices.push_back( fA->GetBody()->GetWorldPoint( polyA->m_vertices[i] ) );

    //fill 'clip polygon' from fixtureB polygon
    std::vector<b2Vec2> clipPolygon;
    for (int i = 0; i < polyB->m_count; i++)
        clipPolygon.push_back( fB->GetBody()->GetWorldPoint( polyB->m_vertices[i] ) );

    b2Vec2 cp1 = clipPolygon[clipPolygon.size()-1];
    for (int j = 0; j < clipPolygon.size(); j++) {
        b2Vec2 cp2 = clipPolygon[j];
        if ( outputVertices.empty() )
            return false;
        std::vector<b2Vec2> inputList = outputVertices;
        outputVertices.clear();
        b2Vec2 s = inputList[inputList.size() - 1]; //last on the input list
        for (int i = 0; i < inputList.size(); i++) {
            b2Vec2 e = inputList[i];
            if (inside(cp1, cp2, e)) {
                if (!inside(cp1, cp2, s)) {
                    outputVertices.push_back( intersection(cp1, cp2, s, e) );
                }
                outputVertices.push_back(e);
            }
            else if (inside(cp1, cp2, s)) {
                outputVertices.push_back( intersection(cp1, cp2, s, e) );
            }
            s = e;
        }
        cp1 = cp2;
    }

    return !outputVertices.empty();
}

#endif









