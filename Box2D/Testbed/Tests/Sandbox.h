/*
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

#ifndef SANDBOX_H
#define SANDBOX_H
#include <math.h>
#include "../Framework/ParticleEmitter.h"

// The following parameters are not static const members of the Sandbox class
// with values assigned inline as it can result in link errors when using gcc.
namespace SandboxParams {

// Total possible pump squares
static const int k_maxPumps = 5;
// Total possible emitters
static const int k_maxEmitters = 5;
// Number of seconds to push one direction or the other on the pumps
static const float32 k_flipTime = 6;
// Radius of a tile
static const float32 k_tileRadius = 2;
// Diameter of a tile
static const float32 k_tileDiameter = 4;
// Pump radius; slightly smaller than a tile
static const float32 k_pumpRadius = 2.0f - 0.05f;

static const float32 k_playfieldLeftEdge = -20;
static const float32 k_playfieldRightEdge = 20;
static const float32 k_playfieldBottomEdge = 40;

// The world size in the TILE
static const int k_tileWidth = 10;
static const int k_tileHeight = 11;

// Particles/second
static const float32 k_defaultEmitterRate = 30;
// Fit cleanly inside one block
static const float32 k_defaultEmitterSize = 3;
// How fast particles coming out of the particles should drop
static const float32 k_particleExitSpeedY = -9.8f;
// How hard the pumps can push
static const float32 k_pumpForce = 600;

// Number of *special* particles.
static const uint32 k_numberOfSpecialParticles = 256;

}  // namespace SandboxParams


// Class which tracks a set of particles and applies a special effect to them.
class SpecialParticleTracker : public b2DestructionListener
{
public:
	// Initialize
	SpecialParticleTracker() :
		m_world(NULL),
		m_particleSystem(NULL),
		m_colorOscillationTime(0.0f),
		m_colorOscillationPeriod(2.0f)
	{
	}

	~SpecialParticleTracker()
	{
		m_world->SetDestructionListener(NULL);
	}

	// Register this class as a destruction listener so that it's possible
	// to keep track of special particles.
	void Init(b2World* const world, b2ParticleSystem* const system)
	{
		b2Assert(world);
		b2Assert(system);
		m_world = world;
		m_particleSystem = system;
		m_world->SetDestructionListener(this);
	}

	// Add as many of the specified particles to the set of special particles.
	void Add(const int32* const particleIndices, const int32 numberOfParticles)
	{
		b2Assert(m_particleSystem);
		for (int32 i = 0; i < numberOfParticles && m_particles.size() <
				 SandboxParams::k_numberOfSpecialParticles; ++i)
		{
			const int32 particleIndex = particleIndices[i];
			m_particleSystem->SetParticleFlags(
				particleIndex,
				m_particleSystem->GetFlagsBuffer()[particleIndex] |
					b2_destructionListenerParticle);
			m_particles.insert(m_particleSystem->GetParticleHandleFromIndex(
								   particleIndex));
		}
	}

	// Apply effects to special particles.
	void Step(float32 dt)
	{
		// Oscillate the shade of color over m_colorOscillationPeriod seconds.
		m_colorOscillationTime = fmodf(m_colorOscillationTime + dt,
									   m_colorOscillationPeriod);
		const float32 colorCoeff = 2.0f * fabsf(
			(m_colorOscillationTime / m_colorOscillationPeriod) - 0.5f);
		const b2ParticleColor color(
			128 + (int8)(128.0f * (1.0f - colorCoeff)),
			128 + (int8)(256.0f * fabsf(0.5f - colorCoeff)),
			128 + (int8)(128.0f * colorCoeff), 255);
		// Update the color of all special particles.
		for (std::set<const b2ParticleHandle*>::const_iterator it =
				 m_particles.begin(); it != m_particles.end(); ++it)
		{
			m_particleSystem->GetColorBuffer()[(*it)->GetIndex()] =
				color;
		}
	}

	virtual void SayGoodbye(b2Joint* joint) { B2_NOT_USED(joint); }
	virtual void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }
	virtual void SayGoodbye(b2ParticleGroup* group) { B2_NOT_USED(group); }

	// When a particle is about to be destroyed, remove it from the list of
	// special particles as the handle will become invalid.
	virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
	{
		if (particleSystem != m_particleSystem)
			return;

		// NOTE: user data could be used as an alternative method to look up
		// the local handle pointer from the index.
		size_t erased = m_particles.erase(
			m_particleSystem->GetParticleHandleFromIndex(index));
		b2Assert(erased == 1);
		B2_NOT_USED(erased);
	}

private:
	// Set of particle handles used to track special particles.
	std::set<const b2ParticleHandle*> m_particles;
	// Pointer to the world used to enable / disable this class as a
	// destruction listener.
	b2World* m_world;
	// Pointer to the particle system used to retrieve particle handles.
	b2ParticleSystem* m_particleSystem;
	// Current offset into m_colorOscillationPeriod.
	float32 m_colorOscillationTime;
	// Color oscillation period in seconds.
	float32 m_colorOscillationPeriod;
};

// Sandbox test creates a maze of faucets, pumps, ramps, circles, and blocks
// based on a string constant.  Please modify and play with this string to make
// new mazes, and also add new maze elements!
class Sandbox : public Test
{
public:

	Sandbox()
	{
		using namespace SandboxParams;

		// We need some ground for the pumps to slide against
		b2BodyDef bd;
		b2Body* ground = m_world->CreateBody(&bd);

		// Reset our pointers
		for (int i = 0; i < k_maxEmitters; i++) {
			m_emitters[i] = NULL;
		}

		for (int i = 0; i < k_maxPumps; i++) {
			m_pumps[i] = NULL;
		}

		m_world->SetGravity(b2Vec2(0.0, -20));

		// Create physical box, no top
		{
			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(-40, -10),
					b2Vec2(40, -10),
					b2Vec2(40, 0),
					b2Vec2(-40, 0)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(k_playfieldLeftEdge - 20, -1),
					b2Vec2(k_playfieldLeftEdge, -1),
					b2Vec2(k_playfieldLeftEdge, 50),
					b2Vec2(k_playfieldLeftEdge - 20, 50)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}

			{
				b2PolygonShape shape;
				const b2Vec2 vertices[4] = {
					b2Vec2(k_playfieldRightEdge, -1),
					b2Vec2(k_playfieldRightEdge + 20, -1),
					b2Vec2(k_playfieldRightEdge + 20, 50),
					b2Vec2(k_playfieldRightEdge, 50)};
				shape.Set(vertices, 4);
				ground->CreateFixture(&shape, 0.0f);
			}
		}

		m_particleSystem->SetRadius(0.25f);

		m_specialTracker.Init(m_world, m_particleSystem);

		m_pumpTimer = 0;

		SetupMaze();

		// Create killfield shape and transform
		m_killfieldShape = b2PolygonShape();
		m_killfieldShape.SetAsBox(k_playfieldRightEdge -
								  k_playfieldLeftEdge,1);

		// Put this at the bottom of the world
		m_killfieldTransform = b2Transform();
		b2Vec2 loc = b2Vec2(-20, 1);
		m_killfieldTransform.Set(loc, 0);

		// Setup particle parameters.
		TestMain::SetParticleParameters(k_paramDef, k_paramDefCount);
		m_particleFlags = TestMain::GetParticleParameterValue();
		TestMain::SetRestartOnParticleParameterChange(false);
	}

	~Sandbox()
	{
		// deallocate our emitters
		for (int i = 0; i < m_faucetEmitterIndex; i++) {
			delete m_emitters[i];
		}
	}


	// Create a maze of blocks, ramps, pumps, and faucets.
	// The maze is defined in a string; feel free to modify it.
	// Items in the maze include:
	//   # = a block
	//   / = a right-to-left ramp triangle
	//   A = a left-to-right ramp triangle (can't be \ or string formatting
	//       would be weird)
	//   r, g, b = colored faucets pointing down
	//   p = a pump block that rocks back and forth.  You can drag them
	//       yourself with your finger.
	//   C = a loose circle
	//   K = an ignored placeholder for a killfield to remove particles;
	//       entire bottom row is a killfield.
	void SetupMaze() {
		using namespace SandboxParams;

		const char *maze =
			"# r#g #r##"
			"  /#  #  #"
			" ###     p"
			"A  #  /###"
			"## # /#  C"
			"  /# #   #"
			" ### # / #"
			" ## p /#  "
			" #  ####  "
			"A        /"
			"#####KK###";

		b2Assert(strlen(maze) == k_tileWidth * k_tileHeight);

		m_faucetEmitterIndex = 0;
		m_pumpIndex = 0;

		// Set up some standard shapes/vertices we'll use later.
		b2PolygonShape boxShape;
		boxShape.SetAsBox(k_tileRadius, k_tileRadius);

		b2Vec2 triangle[3];
		triangle[0].Set(-k_tileRadius, -k_tileRadius);
		triangle[1].Set(k_tileRadius, k_tileRadius);
		triangle[2].Set(k_tileRadius, -k_tileRadius);
		b2PolygonShape rightTriangleShape;
		rightTriangleShape.Set(triangle, 3);

		triangle[1].Set(-k_tileRadius, k_tileRadius);
		b2PolygonShape leftTriangleShape;
		leftTriangleShape.Set(triangle, 3);

		// Make these just a touch smaller than a tile
		b2CircleShape circleShape = b2CircleShape();
		circleShape.m_radius = k_tileRadius * 0.7f;

		b2ParticleColor red = b2ParticleColor(255, 128, 128, 255);
		b2ParticleColor green = b2ParticleColor(128, 255, 128, 255);
		b2ParticleColor blue = b2ParticleColor(128, 128, 255, 255);

		m_pumpForce = b2Vec2(k_pumpForce,0);

		for (int i = 0; i < k_tileWidth; i++) {
			for (int j = 0; j < k_tileHeight; j++) {
				char item = maze[j * k_tileWidth + i];

				// Calculate center of this square
				b2Vec2 center = b2Vec2(
					k_playfieldLeftEdge + k_tileRadius * 2 * i + k_tileRadius,
					k_playfieldBottomEdge - k_tileRadius * 2 * j +
						k_tileRadius);

				// Let's add some items
				switch (item) {
				case '#':
					// Block
					CreateBody(center, boxShape, b2_staticBody);
					break;
				case 'A':
					// Left-to-right ramp
					CreateBody(center, leftTriangleShape, b2_staticBody);
					break;
				case '/':
					// Right-to-left ramp
					CreateBody(center, rightTriangleShape, b2_staticBody);
					break;
				case 'C':
					// A circle to play with
					CreateBody(center, circleShape, b2_dynamicBody);
					break;
				case 'p':
					AddPump(center);
					break;
				case 'b':
					// Blue emitter
					AddFaucetEmitter(center, blue);
					break;
				case 'r':
					// Red emitter
					AddFaucetEmitter(center, red);
					break;
				case 'g':
					// Green emitter
					AddFaucetEmitter(center, green);
					break;
				default:
					// add nothing
					break;
				}
			}
		}
	}

	void CreateBody(b2Vec2 &center, b2Shape &shape, b2BodyType type)
	{
		b2BodyDef def = b2BodyDef();
		def.position = center;
		def.type = type;
		b2Body *body = m_world->CreateBody(&def);
		body->CreateFixture(&shape, 10.0);
	}

	// Inititalizes a pump and its prismatic joint, and adds it to the world
	void AddPump(const b2Vec2 &center)
	{
		using namespace SandboxParams;

		// Don't make too many pumps
		b2Assert(m_pumpIndex < k_maxPumps);

		b2PolygonShape shape = b2PolygonShape();
		shape.SetAsBox(k_pumpRadius, k_pumpRadius);

		b2BodyDef def = b2BodyDef();
		def.position = center;
		def.type = b2_dynamicBody;
		def.angle = 0;
		b2Body *body = m_world->CreateBody(&def);
		body->CreateFixture(&shape, 5.0);

		// Create a prismatic joint and connect to the ground, and have it
		// slide along the x axis.
		b2PrismaticJointDef prismaticJointDef;
		prismaticJointDef.bodyA = m_groundBody;
		prismaticJointDef.bodyB = body;
		prismaticJointDef.collideConnected = false;
		prismaticJointDef.localAxisA.Set(1,0);
		prismaticJointDef.localAnchorA = center;

		m_world->CreateJoint(&prismaticJointDef);

		m_pumps[m_pumpIndex] = body;
		m_pumpIndex++;
	}

	// Initializes and adds a faucet emitter
	void AddFaucetEmitter(const b2Vec2 &center, b2ParticleColor &color)
	{
		using namespace SandboxParams;

		// Don't make too many emitters
		b2Assert(m_faucetEmitterIndex < SandboxParams::k_maxPumps);

		b2Vec2 startingVelocity = b2Vec2(0, k_particleExitSpeedY);

		RadialEmitter * const emitter = new RadialEmitter();
		emitter->SetParticleSystem(m_particleSystem);
		emitter->SetPosition(center);
		emitter->SetVelocity(startingVelocity);
		emitter->SetSize(b2Vec2(k_defaultEmitterSize, 0.0f));
		emitter->SetEmitRate(k_defaultEmitterRate);
		emitter->SetColor(color);
		m_emitters[m_faucetEmitterIndex] = emitter;
		m_faucetEmitterIndex++;
	}

	// Per-frame step updater overridden from Test
	virtual void Step(Settings *settings)
	{
		using namespace SandboxParams;

		Test::Step(settings);

		m_particleFlags = TestMain::GetParticleParameterValue();

		float32 dt = 1.0f/settings->hz;

		// Step all the emitters
		for (int i = 0; i < m_faucetEmitterIndex; i++)
		{
			int32 particleIndices[k_numberOfSpecialParticles];
			RadialEmitter *emitter = m_emitters[i];

			emitter->SetParticleFlags(m_particleFlags);
			const int32 particlesCreated = emitter->Step(
				dt, particleIndices, B2_ARRAY_SIZE(particleIndices));
			m_specialTracker.Add(particleIndices, particlesCreated);
		}

		// Step the special tracker.
		m_specialTracker.Step(dt);

		// Do killfield work--kill every particle near the bottom of the screen
		m_particleSystem->DestroyParticlesInShape(m_killfieldShape,
												  m_killfieldTransform);

		// Move the pumps
		for (int i = 0; i < m_pumpIndex; i++)
		{
			b2Body* pump = m_pumps[i];

			// Pumps can and will clog up if the pile of particles they're
			// trying to push is too heavy. Increase k_pumpForce to make
			// stronger pumps.
			pump->ApplyForceToCenter(m_pumpForce, true);

			m_pumpTimer+=dt;

			// Reset pump to go back right again
			if (m_pumpTimer > k_flipTime) {
				m_pumpTimer -= k_flipTime;
				m_pumpForce.x *= -1;
			}
		}

		g_debugDraw.DrawString(
			5, m_textLine, "Keys: (a) zero out (water), (q) powder");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(
			5, m_textLine, "      (t) tensile, (v) viscous");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	// Allows you to set particle flags on devices with keyboards
	void Keyboard(unsigned char key)
	{
		uint32 toggle = 0;
		switch (key)
		{
			case 'a':
				m_particleFlags = 0;
				break;
			case 'q':
				toggle = b2_powderParticle;
				break;
			case 't':
				toggle = b2_tensileParticle;
				break;
			case 'v':
				toggle = b2_viscousParticle;
				break;
			case 'w':
				toggle = b2_wallParticle;
				break;
		}
		if (toggle) {
			if (m_particleFlags & toggle) {
				m_particleFlags = m_particleFlags & ~toggle;
			} else {
				m_particleFlags = m_particleFlags | toggle;
			}
		}
		TestMain::SetParticleParameterValue(m_particleFlags);
	}

	static Test* Create()
	{
		return new Sandbox;
	}

private:
	// Count of faucets in the world
	int m_faucetEmitterIndex;
	// Count of pumps in the world
	int m_pumpIndex;

	// How long have we been pushing the pumps?
	float32 m_pumpTimer;
	// Particle creation flags
	uint32 m_particleFlags;

	// Pump force
	b2Vec2 m_pumpForce;

	// The shape we will use for the killfield
	b2PolygonShape m_killfieldShape;
	// Transform for the killfield shape
	b2Transform m_killfieldTransform;

	// Pumps and emitters
	b2Body* m_pumps[SandboxParams::k_maxPumps];
	RadialEmitter *m_emitters[SandboxParams::k_maxEmitters];

	// Special particle tracker.
	SpecialParticleTracker m_specialTracker;

	static const ParticleParameter::Value k_paramValues[];
	static const ParticleParameter::Definition k_paramDef[];
	static const uint32 k_paramDefCount;
};

const ParticleParameter::Value Sandbox::k_paramValues[] =
{
	{b2_waterParticle, ParticleParameter::k_DefaultOptions, "water"},
	{b2_waterParticle, ParticleParameter::k_DefaultOptions |
				ParticleParameter::OptionStrictContacts, "water (strict)" },
	{b2_powderParticle, ParticleParameter::k_DefaultOptions, "powder"},
	{b2_tensileParticle, ParticleParameter::k_DefaultOptions, "tensile"},
	{b2_viscousParticle, ParticleParameter::k_DefaultOptions, "viscous"},
	{b2_tensileParticle | b2_powderParticle,
		ParticleParameter::k_DefaultOptions,
		"tensile powder"},
	{b2_viscousParticle | b2_powderParticle,
		ParticleParameter::k_DefaultOptions,
		"viscous powder"},
	{b2_viscousParticle | b2_tensileParticle | b2_powderParticle,
		ParticleParameter::k_DefaultOptions, "viscous tensile powder"},
	{b2_viscousParticle | b2_tensileParticle,
		ParticleParameter::k_DefaultOptions,
		"tensile viscous water"}
};

const ParticleParameter::Definition Sandbox::k_paramDef[] =
{
	{ Sandbox::k_paramValues, B2_ARRAY_SIZE(Sandbox::k_paramValues) },
};
const uint32 Sandbox::k_paramDefCount =
	B2_ARRAY_SIZE(Sandbox::k_paramDef);


#endif
