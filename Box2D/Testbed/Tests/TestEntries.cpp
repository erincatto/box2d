/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Copyright (c) 2013 Google, Inc.
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

#include "../Framework/Test.h"
#include "../Framework/Main.h"
#include "../Framework/ParticleParameter.h"

#include "AddPair.h"
#include "ApplyForce.h"
#include "BasicSliderCrank.h"
#include "BodyTypes.h"
#include "Breakable.h"
#include "Bridge.h"
#include "BulletTest.h"
#include "Cantilever.h"
#include "Car.h"
#include "ContinuousTest.h"
#include "Chain.h"
#include "CharacterCollision.h"
#include "CollisionFiltering.h"
#include "CollisionProcessing.h"
#include "CompoundShapes.h"
#include "Confined.h"
#include "ConvexHull.h"
#include "ConveyorBelt.h"
#include "DistanceTest.h"
#include "Dominos.h"
#include "DumpShell.h"
#include "DynamicTreeTest.h"
#include "EdgeShapes.h"
#include "EdgeTest.h"
#include "Gears.h"
#include "HeavyOnLight.h"
#include "HeavyOnLightTwo.h"
#include "Mobile.h"
#include "MobileBalanced.h"
#include "MotorJoint.h"
#include "OneSidedPlatform.h"
#include "Pinball.h"
#include "PolyCollision.h"
#include "PolyShapes.h"
#include "Prismatic.h"
#include "Pulleys.h"
#include "Pyramid.h"
#include "RayCast.h"
#include "Revolute.h"
#include "RopeJoint.h"
#include "SensorTest.h"
#include "ShapeEditing.h"
#include "SliderCrank.h"
#include "SphereStack.h"
#include "TheoJansen.h"
#include "Tiles.h"
#include "TimeOfImpact.h"
#include "Tumbler.h"
#include "VaryingFriction.h"
#include "VaryingRestitution.h"
#include "VerticalStack.h"
#include "Web.h"

#include "AntiPointy.h"
#include "CornerCase.h"
#include "DamBreak.h"
#include "DrawingParticles.h"
#include "ElasticParticles.h"
#include "Faucet.h"
#include "Fracker.h"
#include "Impulse.h"
#include "LiquidTimer.h"
#include "Maxwell.h"
#include "MultipleParticleSystems.h"
#include "Particles.h"
#include "ParticleCollisionFilter.h"
#include "ParticlesSurfaceTension.h"
#include "Pointy.h"
#include "Ramp.h"
#include "RigidParticles.h"
#include "Sandbox.h"
#include "Soup.h"
#include "SoupStirrer.h"
#include "Sparky.h"
#include "WaveMachine.h"

TestEntry g_testEntries[] =
{
	{"Sandbox", Sandbox::Create},
	{"Add Pair Stress Test", AddPair::Create},
	{"AntiPointy", AntiPointy::Create},
	{"Apply Force", ApplyForce::Create},
	{"Body Types", BodyTypes::Create},
	{"Breakable", Breakable::Create},
	{"Bridge", Bridge::Create},
	{"Bullet Test", BulletTest::Create},
	{"Cantilever", Cantilever::Create},
	{"Car", Car::Create},
	{"Chain", Chain::Create},
	{"Character Collision", CharacterCollision::Create},
	{"Collision Filtering", CollisionFiltering::Create},
	{"Collision Processing", CollisionProcessing::Create},
	{"Compound Shapes", CompoundShapes::Create},
	{"Confined", Confined::Create},
	{"Continuous Test", ContinuousTest::Create},
	{"Convex Hull", ConvexHull::Create},
	{"Conveyor Belt", ConveyorBelt::Create},
	{"Corner Case", CornerCase::Create},
	{"DamBreak", DamBreak::Create},
	{"Distance Test", DistanceTest::Create},
	{"Dominos", Dominos::Create},
	{"Dump Shell", DumpShell::Create},
	{"Dynamic Tree", DynamicTreeTest::Create},
	{"Edge Shapes", EdgeShapes::Create},
	{"Edge Test", EdgeTest::Create},
	{"Elastic Particles", ElasticParticles::Create},
	{"Faucet", Faucet::Create},
	{"Fracker", Fracker::Create},
	{"Gears", Gears::Create},
	{"Impulse", Impulse::Create},
	{"Liquid Timer", LiquidTimer::Create},
	{"Maxwell", Maxwell::Create},
	{"Mobile", Mobile::Create},
	{"MobileBalanced", MobileBalanced::Create},
	{"Motor Joint", MotorJoint::Create},
	{"Multiple Systems", MultipleParticleSystems::Create},
	{"One-Sided Platform", OneSidedPlatform::Create},
	{"Particle Collisions", ParticleCollisionFilter::Create},
	{"Particle Drawing", DrawingParticles::Create},
	{"Particles", Particles::Create},
	{"Pinball", Pinball::Create},
	{"Pointy", Pointy::Create},
	{"PolyCollision", PolyCollision::Create},
	{"Polygon Shapes", PolyShapes::Create},
	{"Prismatic", Prismatic::Create},
	{"Pulleys", Pulleys::Create},
	{"Pyramid", Pyramid::Create},
	{"Ramp", Ramp::Create},
	{"Ray-Cast", RayCast::Create},
	{"Revolute", Revolute::Create},
	{"Rigid Particles", RigidParticles::Create},
	{"RopeJoint", RopeJoint::Create},
	{"Sensor Test", SensorTest::Create},
	{"Shape Editing", ShapeEditing::Create},
	{"Slider Crank", SliderCrank::Create},
	{"Soup Stirrer", SoupStirrer::Create},
	{"Soup", Soup::Create},
	{"Sparky", Sparky::Create},
	{"SphereStack", SphereStack::Create},
	{"Surface Tension", ParticlesSurfaceTension::Create},
	{"Theo Jansen's Walker", TheoJansen::Create},
	{"Tiles", Tiles::Create},
	{"Time of Impact", TimeOfImpact::Create},
	{"Tumbler", Tumbler::Create},
	{"Varying Friction", VaryingFriction::Create},
	{"Varying Restitution", VaryingRestitution::Create},
	{"Vertical Stack", VerticalStack::Create},
	{"Wave Machine", WaveMachine::Create},
	{"Web", Web::Create},

	{NULL, NULL}
};
