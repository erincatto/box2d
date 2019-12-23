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

#include "b2_settings.h"
#include "b2_draw.h"
#include "b2_timer.h"

#include "b2_chain_shape.h"
#include "b2_circle_shape.h"
#include "b2_edge_shape.h"
#include "b2_polygon_shape.h"

#include "b2_broad_phase.h"
#include "b2_dynamic_tree.h"

#include "b2_body.h"
#include "b2_contact.h"
#include "b2_fixture.h"
#include "b2_time_step.h"
#include "b2_world.h"
#include "b2_world_callbacks.h"

#include "b2_distance_joint.h"
#include "b2_friction_joint.h"
#include "b2_gear_joint.h"
#include "b2_motor_joint.h"
#include "b2_mouse_joint.h"
#include "b2_prismatic_joint.h"
#include "b2_pulley_joint.h"
#include "b2_revolute_joint.h"
#include "b2_rope_joint.h"
#include "b2_weld_joint.h"
#include "b2_wheel_joint.h"

#endif
