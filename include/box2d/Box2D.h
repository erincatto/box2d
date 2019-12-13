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

#ifndef BOX2D_H
#define BOX2D_H

/**
\mainpage Box2D API Documentation

\section intro_sec Getting Started

For documentation please see http://box2d.org/documentation.html

For discussion please visit http://box2d.org/forum
*/

// These include files constitute the main Box2D API

#include "b2Settings.h"
#include "b2Draw.h"
#include "b2Timer.h"

#include "b2CircleShape.h"
#include "b2EdgeShape.h"
#include "b2ChainShape.h"
#include "b2PolygonShape.h"

#include "b2BroadPhase.h"
#include "b2Distance.h"
#include "b2DynamicTree.h"
#include "b2TimeOfImpact.h"

#include "b2Body.h"
#include "b2Fixture.h"
#include "b2WorldCallbacks.h"
#include "b2TimeStep.h"
#include "b2World.h"

#include "b2Contact.h"

#include "b2DistanceJoint.h"
#include "b2FrictionJoint.h"
#include "b2GearJoint.h"
#include "b2MotorJoint.h"
#include "b2MouseJoint.h"
#include "b2PrismaticJoint.h"
#include "b2PulleyJoint.h"
#include "b2RevoluteJoint.h"
#include "b2RopeJoint.h"
#include "b2WeldJoint.h"
#include "b2WheelJoint.h"

#endif
