/*
* Copyright (c) 2014 Google, Inc
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
#ifndef FRACKER_H
#define FRACKER_H

#include "../Framework/ParticleEmitter.h"
#include <map>
#include <set>
#include <vector>
#include <stdio.h>

// Tracks instances of RadialEmitter and destroys them after a specified
// period of time.
class EmitterTracker {
public:
	EmitterTracker() {}

	// Delete all emitters.
	~EmitterTracker()
	{
		for (std::map<RadialEmitter*, float32>::const_iterator it =
				 m_emitterLifetime.begin(); it != m_emitterLifetime.end();
				 ++it)
		{
			delete it->first;
		}
	}

	// Add an emitter to the tracker.
	// This assumes emitter was allocated using "new" and ownership of the
	// object is handed to this class.
	void Add(RadialEmitter *const emitter, const float32 lifetime)
	{
		m_emitterLifetime[emitter] = lifetime;
	}

	// Update all emitters destroying those who are too old.
	void Step(const float32 dt)
	{
		std::vector<RadialEmitter*> emittersToDestroy;
		for (std::map<RadialEmitter*, float32>::const_iterator it =
				 m_emitterLifetime.begin(); it != m_emitterLifetime.end();
				 ++it)
		{
			RadialEmitter * const emitter = it->first;
			const float32 lifetime = it->second - dt;
			if (lifetime <= 0.0f)
			{
				emittersToDestroy.push_back(emitter);
			}
			m_emitterLifetime[emitter] = lifetime;
			emitter->Step(dt, NULL, 0);
		}
		for (std::vector<RadialEmitter*>::const_iterator it =
				 emittersToDestroy.begin(); it != emittersToDestroy.end();
				 ++it)
		{
			RadialEmitter *emitter = *it;
			delete emitter;
			m_emitterLifetime.erase(m_emitterLifetime.find(emitter));
		}
	}

protected:
	std::map<RadialEmitter*, float32> m_emitterLifetime;
};

// Keep track of particle groups in a set, removing them when they're
// destroyed.
class ParticleGroupTracker : public b2DestructionListener
{
public:
	ParticleGroupTracker() {}
	virtual ~ParticleGroupTracker() {}
	virtual void SayGoodbye(b2Joint* joint) { B2_NOT_USED(joint); }
	virtual void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }
	virtual void SayGoodbye(b2ParticleSystem* system, int32 index)
	{
		B2_NOT_USED(system);
		B2_NOT_USED(index);
	}

	// Called when any particle group is about to be destroyed.
	virtual void SayGoodbye(b2ParticleGroup* group)
	{
		RemoveParticleGroup(group);
	}

	// Add a particle group to the tracker.
	void AddParticleGroup(b2ParticleGroup* const group)
	{
		m_particleGroups.insert(group);
	}

	// Remove a particle group from the tracker.
	void RemoveParticleGroup(b2ParticleGroup* const group)
	{
		std::set<b2ParticleGroup*>::iterator it = m_particleGroups.find(group);
		m_particleGroups.erase(it);
	}

	// Get the current set of particle groups.
	const std::set<b2ParticleGroup*>& GetParticleGroups() const
	{
		return m_particleGroups;
	}

private:
	std::set<b2ParticleGroup*> m_particleGroups;
};

namespace FrackerSettings {

// Width and height of the world in tiles.
const int32 k_worldWidthTiles = 24;
const int32 k_worldHeightTiles = 16;
// Total number of tiles.
const int32 k_worldTiles = k_worldWidthTiles * k_worldHeightTiles;
// Center of the world in world coordinates.
const float32 k_worldCenterX = 0.0f;
const float32 k_worldCenterY = 2.0f;
// Size of each tile in world units.
const float32 k_tileWidth = 0.2f;
const float32 k_tileHeight = 0.2f;
// Half width and height of tiles in world units.
const float32 k_tileHalfWidth = k_tileWidth * 0.5f;
const float32 k_tileHalfHeight = k_tileHeight * 0.5f;
// Half width and height of the world in world coordinates.
const float32 k_worldHalfWidth =
	((float32)k_worldWidthTiles * k_tileWidth) * 0.5f;
const float32 k_worldHalfHeight =
	((float32)k_worldHeightTiles * k_tileHeight) * 0.5f;

// Colors of tiles.
const b2Color k_playerColor(1.0f, 1.0f, 1.0f);
const b2Color k_playerFrackColor(1.0f, 0.5f, 0.5f);
const b2Color k_wellColor(0.5f, 0.5f, 0.5f);
const b2ParticleColor k_oilColor(b2Color(1.0f, 0.0f, 0.0f));
const b2ParticleColor k_waterColor(b2Color(0.0f, 0.2f, 1.0f));
const b2ParticleColor k_frackingFluidColor(b2Color(0.8f, 0.4f, 0));

// Default density of each body.
const float32 k_density = 0.1f;

// Radius of oil / water / fracking fluid particles.
const float32 k_particleRadius = ((k_tileWidth + k_tileHeight) * 0.5f) * 0.2f;

// Probability (0..100%) of generating each tile (must sum to 1.0f).
const int32 k_dirtProbability = 80;
const int32 k_emptyProbability = 10;
const int32 k_oilProbability = 7;
const int32 k_waterProbability = 3;

// Lifetime of a fracking fluid emitter in seconds.
const float32 k_frackingFluidEmitterLifetime = 5.0f;

// Speed particles are sucked up the well.
const float32 k_wellSuckSpeedInside = k_tileHeight * 5.0f;
// Speed particle are sucket towards the well bottom.
const float32 k_wellSuckSpeedOutside = k_tileWidth * 1.0f;

// Time mouse button must be held before emitting fracking fluid.
const float32 k_frackingFluidChargeTime = 1.0f;

// Scores.
const int32 k_scorePerOilParticle = 1;
const int32 k_scorePerWaterParticle = -1;
const int32 k_scorePerFrackingParticle = 0;
const int32 k_scorePerFrackingDeployment = -10;

}  // namespace FrackerSettings

// Oil Fracking simulator. Dig down to move the oil (red) to the well (gray).
// Try not to contaminate the ground water (blue). To deploy fracking fluid
// press 'space'.  Fracking fluid can be used to push other fluids to the
// well head and ultimately score points.
class Fracker : public Test
{
private:
	// Type of material in a tile.
	enum Material
	{
		EMPTY = 0,
		DIRT,
		ROCK,
		OIL,
		WATER,
		WELL,
		PUMP,
	};

	// Keep track of particle groups which are drawn up the well and tracks
	// the score of the game.
	class DestructionListener : public ParticleGroupTracker
	{
	public:
		// Initialize the score.
		DestructionListener() :
			m_score(0),
			m_oil(0),
			m_world(NULL),
			m_previousListener(NULL)
		{
		}

		virtual ~DestructionListener()
		{
			if (m_world)
			{
				m_world->SetDestructionListener(m_previousListener);
			}
		}

		// Initialize the particle system and world, setting this class as
		// a destruction listener for the world.
		void Initialize(b2World* const world)
		{
			b2Assert(world);
			m_world = world;
			m_world->SetDestructionListener(this);
		}

		// Add to the current score.
		void AddScore(int32 score)
		{
			m_score += score;
		}

		// Get the current score.
		int32 GetScore() const
		{
			return m_score;
		}

		// Add to the remaining oil.
		void AddOil(int32 oil)
		{
			m_oil += oil;
		}

		// Get the total oil.
		int32 GetOil() const
		{
			return m_oil;
		}

		// Update the score when certain particles are destroyed.
		virtual void SayGoodbye(b2ParticleSystem* particleSystem, int32 index)
		{
			b2Assert(particleSystem);
			const void * const userData =
				particleSystem->GetUserDataBuffer()[index];
			if (userData)
			{
				const Material material = *((Material*)userData);
				switch (material)
				{
				case OIL:
					AddScore(FrackerSettings::k_scorePerOilParticle);
					AddOil(-1);
					break;
				case WATER:
					AddScore(FrackerSettings::k_scorePerWaterParticle);
					break;
				default:
					break;
				}
			}
		}

	private:
		int32 m_score;
		int32 m_oil;
		b2World *m_world;
		b2DestructionListener* m_previousListener;
	};

public:
	Fracker() :
		m_wellX(FrackerSettings::k_worldWidthTiles -
				(FrackerSettings::k_worldWidthTiles / 4)),
		m_wellTop(FrackerSettings::k_worldHeightTiles - 1),
		m_wellBottom(FrackerSettings::k_worldHeightTiles / 8),
		m_allowInput(false),
		m_frackingFluidChargeTime(-1.0f)
	{
		m_listener.Initialize(m_world);
		m_particleSystem->SetRadius(FrackerSettings::k_particleRadius);
		InitializeLayout();
		// Create the boundaries of the play area.
		CreateGround();
		// Create the well.
		CreateWell();
		// Create the geography / features (tiles of the world).
		CreateGeo();
		// Create the player.
		CreatePlayer();
	}

	virtual ~Fracker() { }

	// Initialize the data structures used to track the material in each
	// tile and the bodies associated with each tile.
	void InitializeLayout()
	{
		for (int32 i = 0; i < FrackerSettings::k_worldTiles; ++i)
		{
			m_material[i] = EMPTY;
			m_bodies[i] = NULL;
		}
	}

	// Get the material of the tile at the specified tile position.
	Material GetMaterial(const int32 x, const int32 y) const
	{
		return *const_cast<Fracker*>(this)->GetMaterialStorage(x, y);
	}


	// Set the material of the tile at the specified tile position.
	void SetMaterial(const int32 x, const int32 y, const Material material)
	{
		*GetMaterialStorage(x, y) = material;
	}

	// Get the body associated with the specified tile position.
	b2Body* GetBody(const int32 x, const int32 y) const
	{
		return *const_cast<Fracker*>(this)->GetBodyStorage(x, y);
	}

	// Set the body associated with the specified tile position.
	void SetBody(const int32 x, const int32 y, b2Body* const body)
	{
		b2Body** const currentBody = GetBodyStorage(x, y);
		if (*currentBody)
		{
			m_world->DestroyBody(*currentBody);
		}
		*currentBody = body;
	}

	// Create the player.
	void CreatePlayer()
	{
		b2BodyDef bd;
		bd.type = b2_kinematicBody;
		m_player = m_world->CreateBody(&bd);
		b2PolygonShape shape;
		shape.SetAsBox(FrackerSettings::k_tileHalfWidth,
					   FrackerSettings::k_tileHalfHeight,
					   b2Vec2(FrackerSettings::k_tileHalfWidth,
							  FrackerSettings::k_tileHalfHeight), 0);
		m_player->CreateFixture(&shape, FrackerSettings::k_density);
		m_player->SetTransform(
			TileToWorld(FrackerSettings::k_worldWidthTiles / 2,
						FrackerSettings::k_worldHeightTiles / 2), 0);
	}

	// Create the geography / features of the world.
	void CreateGeo()
	{
		b2Assert(FrackerSettings::k_dirtProbability +
				 FrackerSettings::k_emptyProbability +
				 FrackerSettings::k_oilProbability +
				 FrackerSettings::k_waterProbability == 100);
		for (int32 x = 0; x < FrackerSettings::k_worldWidthTiles; x++)
		{
			for (int32 y = 0; y < FrackerSettings::k_worldHeightTiles; y++)
			{
				if (GetMaterial(x, y) != EMPTY)
				{
					continue;
				}
				// Choose a tile at random.
				const int32 chance = (int32)(((float32)rand() /
											  (float32)RAND_MAX) * 100.0f);
				// Create dirt if this is the bottom row or chance dictates it.
				if (chance < FrackerSettings::k_dirtProbability || y == 0)
				{
					CreateDirtBlock(x, y);
				}
				else if (chance < FrackerSettings::k_dirtProbability +
						 FrackerSettings::k_emptyProbability)
				{
					SetMaterial(x, y, EMPTY);
				}
				else if (chance < FrackerSettings::k_dirtProbability +
								  FrackerSettings::k_emptyProbability +
								  FrackerSettings::k_oilProbability)
				{
					CreateReservoirBlock(x, y, OIL);
				}
				else
				{
					CreateReservoirBlock(x, y, WATER);
				}
			}
		}
	}

	// Create the boundary of the world.
	void CreateGround()
	{
		b2BodyDef bd;
		b2Body* ground = m_world->CreateBody(&bd);
		b2ChainShape shape;
		b2Vec2 bottomLeft, topRight;
		GetExtents(&bottomLeft, &topRight);
		const b2Vec2 vertices[4] = {
			b2Vec2(bottomLeft.x, bottomLeft.y),
			b2Vec2(topRight.x, bottomLeft.y),
			b2Vec2(topRight.x, topRight.y),
			b2Vec2(bottomLeft.x, topRight.y)
		};
		shape.CreateLoop(vertices, 4);
		ground->CreateFixture(&shape, 0.0f);
	}

	// Create a dirt block at the specified world position.
	void CreateDirtBlock(const int32 x, const int32 y)
	{
		const b2Vec2 position = TileToWorld(x, y);
		b2BodyDef bd;
		b2Body* const body = m_world->CreateBody(&bd);
		b2PolygonShape shape;
		shape.SetAsBox(FrackerSettings::k_tileHalfWidth,
					   FrackerSettings::k_tileHalfHeight,
					   CenteredPosition(position), 0);
		body->CreateFixture(&shape, FrackerSettings::k_density);
		SetBody(x, y, body);
		SetMaterial(x, y, DIRT);
	}

	// Create particles in a tile with resources.
	void CreateReservoirBlock(const int32 x, const int32 y,
							  const Material material)
	{
		const b2Vec2 position = TileToWorld(x, y);
		b2PolygonShape shape;
		SetMaterial(x, y, material);
		shape.SetAsBox(FrackerSettings::k_tileHalfWidth,
					   FrackerSettings::k_tileHalfHeight,
					   CenteredPosition(position), 0);
		b2ParticleGroupDef pd;
		pd.flags = b2_tensileParticle | b2_viscousParticle |
			b2_destructionListenerParticle;
		pd.shape = &shape;
		pd.color = material == OIL ?
			FrackerSettings::k_oilColor : FrackerSettings::k_waterColor;
		b2ParticleGroup *group = m_particleSystem->CreateParticleGroup(pd);
		m_listener.AddParticleGroup(group);

		// Tag each particle with its type.
		const int32 particleCount = group->GetParticleCount();
		void** const userDataBuffer =
			m_particleSystem->GetUserDataBuffer() + group->GetBufferIndex();;
		for (int32 i = 0; i < particleCount; ++i)
		{
			userDataBuffer[i] = GetMaterialStorage(x, y);
		}
		// Keep track of the total available oil.
		if (material == OIL)
		{
			m_listener.AddOil(particleCount);
		}
	}

	// Create a well and the region which applies negative pressure to
	// suck out fluid.
	void CreateWell()
	{
		for (int32 y = m_wellBottom; y <= m_wellTop; y++)
		{
			SetMaterial(m_wellX, y, WELL);
		}
	}

	// Create a fracking fluid emitter.
	void CreateFrackingFluidEmitter(const b2Vec2& position)
	{
		b2ParticleGroupDef groupDef;
		b2ParticleGroup * const group = m_particleSystem->CreateParticleGroup(
			groupDef);
		m_listener.AddParticleGroup(group);
		RadialEmitter * const emitter = new RadialEmitter();
		emitter->SetGroup(group);
		emitter->SetParticleSystem(m_particleSystem);
		emitter->SetPosition(CenteredPosition(position));
		emitter->SetVelocity(b2Vec2(0.0f, -FrackerSettings::k_tileHalfHeight));
		emitter->SetSpeed(FrackerSettings::k_tileHalfWidth * 0.1f);
		emitter->SetSize(b2Vec2(FrackerSettings::k_tileHalfWidth,
								FrackerSettings::k_tileHalfHeight));
		emitter->SetEmitRate(20.0f);
		emitter->SetColor(FrackerSettings::k_frackingFluidColor);
		emitter->SetParticleFlags(b2_tensileParticle | b2_viscousParticle);
		m_tracker.Add(emitter,
					  FrackerSettings::k_frackingFluidEmitterLifetime);
		m_listener.AddScore(FrackerSettings::k_scorePerFrackingDeployment);
	}

	// Update the player's position.
	void SetPlayerPosition(int32 playerX, int32 playerY)
	{
		const b2Vec2& playerPosition = m_player->GetTransform().p;
		int32 currentPlayerX, currentPlayerY;
		WorldToTile(playerPosition, &currentPlayerX, &currentPlayerY);

		playerX = b2Clamp(playerX, 0, FrackerSettings::k_worldWidthTiles - 1);
		playerY = b2Clamp(playerY, 0, FrackerSettings::k_worldHeightTiles - 1);

		// Only update if the player has moved and isn't attempting to
		// move through the well.
		if (GetMaterial(playerX, playerY) != WELL &&
			(currentPlayerX != playerX ||
			 currentPlayerY != playerY))
		{
			// Try to deploy any fracking fluid that was charging.
			DeployFrackingFluid();
			// Move the player.
			m_player->SetTransform(TileToWorld(playerX, playerY), 0);
		}
	}

	// Try to deploy fracking fluid at the player's position, returning
	// true if successful.
	bool DeployFrackingFluid()
	{
		bool deployed = false;
		const b2Vec2& playerPosition = m_player->GetTransform().p;
		if (m_frackingFluidChargeTime >
			FrackerSettings::k_frackingFluidChargeTime)
		{
			CreateFrackingFluidEmitter(playerPosition);
			deployed = true;
		}
		m_frackingFluidChargeTime = -1.0f;
		return deployed;
	}

	// Try to deploy the fracking fluid or move the player.
	virtual void MouseUp(const b2Vec2& p)
	{
		if (!m_allowInput)
		{
			return;
		}

		// If fracking fluid isn't being released, move the player.
		if (!DeployFrackingFluid())
		{
			const b2Vec2& playerPosition = m_player->GetTransform().p;
			int32 playerX, playerY;
			WorldToTile(playerPosition, &playerX, &playerY);
			// Move the player towards the mouse position, preferring to move
			// along the axis with the maximal distance from the cursor.
			const b2Vec2 distance = p - CenteredPosition(playerPosition);
			const float32 absDistX = fabsf(distance.x);
			const float32 absDistY = fabsf(distance.y);
			if (absDistX > absDistY &&
				absDistX >= FrackerSettings::k_tileHalfWidth)
			{
				playerX += distance.x > 0.0f ? 1 : -1;
			}
			else if (absDistY >= FrackerSettings::k_tileHalfWidth)
			{
				playerY += distance.y > 0.0f ? 1 : -1;
			}
			SetPlayerPosition(playerX, playerY);
		}
		m_allowInput = false;
	}

	// Start preparing the fracking fluid.
	virtual void MouseDown(const b2Vec2& p)
	{
		B2_NOT_USED(p);
		m_frackingFluidChargeTime = 0.0f;
	}

	// a = left, d = right, a = up, s = down, e = deploy fracking fluid.
	virtual void Keyboard(unsigned char key)
	{
		// Only allow 1 move per simulation step.
		if (!m_allowInput)
		{
			return;
		}

		const b2Vec2& playerPosition = m_player->GetTransform().p;
		int32 playerX, playerY;
		WorldToTile(playerPosition, &playerX, &playerY);
		switch (key)
		{
		case 'a':
			playerX--;
			break;
		case 's':
			playerY--;
			break;
		case 'd':
			playerX++;
			break;
		case 'w':
			playerY++;
			break;
		case 'e':
			// Start charging the fracking fluid.
			if (m_frackingFluidChargeTime < 0.0f)
			{
				m_frackingFluidChargeTime = 0.0f;
			}
			else
			{
				// KeyboardUp() in freeglut (at least on OSX) is called
				// repeatedly while a key is held.  This means there isn't
				// a way for fracking fluid to be deployed when the user
				// releases 'e'.  This works around the issue by attempting
				// to deploy the fluid when 'e' is pressed again.
				DeployFrackingFluid();
			}
			break;
		default:
			Test::Keyboard(key);
			break;
		}
		SetPlayerPosition(playerX, playerY);
		m_allowInput = false;
	}

	// Destroy all particles in the box specified by a set of tile coordinates.
	void DestroyParticlesInTiles(const int32 startX, const int32 startY,
								 const int32 endX, const int32 endY)
	{
		b2PolygonShape shape;
		const int32 width = endX - startX + 1;
		const int32 height = endY - startY + 1;
		const int32 centerX = startX + width / 2;
		const int32 centerY = startY + height / 2;
		shape.SetAsBox(
			FrackerSettings::k_tileHalfWidth * (float32)width,
			FrackerSettings::k_tileHalfHeight * (float32)height);
		b2Transform killLocation;
		killLocation.Set(CenteredPosition(TileToWorld(centerX, centerY)), 0);
		m_particleSystem->DestroyParticlesInShape(shape, killLocation);
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		const float32 dt =  1.0f / settings->hz;
		m_tracker.Step(dt);
		// Allow the user to move again.
		m_allowInput = true;
		// Charge up fracking fluid.
		if (m_frackingFluidChargeTime >= 0.0f)
		{
			m_frackingFluidChargeTime += dt;
		}

		const b2Vec2& playerPosition = m_player->GetTransform().p;
		int32 playerX, playerY;
		WorldToTile(playerPosition, &playerX, &playerY);
		// If the player is moved to a square with dirt, remove it.
		if (GetMaterial(playerX, playerY) == DIRT) {
			SetMaterial(playerX, playerY, EMPTY);
			SetBody(playerX, playerY, NULL);
		}

		// Destroy particles at the top of the well.
		DestroyParticlesInTiles(m_wellX, m_wellTop, m_wellX, m_wellTop);

		// Only move particles in the groups being tracked.
		const std::set<b2ParticleGroup*> &particleGroups =
			m_listener.GetParticleGroups();
		for (std::set<b2ParticleGroup*>::const_iterator it =
				 particleGroups.begin(); it != particleGroups.end(); ++it)
		{
			b2ParticleGroup * const particleGroup = *it;
			const int32 index = particleGroup->GetBufferIndex();
			const b2Vec2* const positionBuffer =
				m_particleSystem->GetPositionBuffer() + index;
			b2Vec2* const velocityBuffer =
				m_particleSystem->GetVelocityBuffer() + index;
			const int32 particleCount = particleGroup->GetParticleCount();
			for (int32 i = 0; i < particleCount; ++i)
			{
				// Apply velocity to particles near the bottom or in the well
				// sucking them up to the top.
				const b2Vec2 wellEnd = CenteredPosition(
					TileToWorld(m_wellX, m_wellBottom - 2));
				const b2Vec2& particlePosition = positionBuffer[i];
				// Distance from the well's bottom.
				const b2Vec2 distance = particlePosition - wellEnd;
				// Distance from either well side wall.
				const float32 absDistX = fabsf(distance.x);
				if (absDistX < FrackerSettings::k_tileWidth &&
					// If the particles are just below the well bottom.
					distance.y > FrackerSettings::k_tileWidth * -2.0f &&
					distance.y < 0.0f)
				{
					// Suck the particles towards the end of the well.
					b2Vec2 velocity = wellEnd - particlePosition;
					velocity.Normalize();
					velocityBuffer[i] = velocity *
						FrackerSettings::k_wellSuckSpeedOutside;
				}
				else if (absDistX <= FrackerSettings::k_tileHalfWidth &&
						 distance.y > 0.0f)
				{
					// Suck the particles up the well with a random
					// x component moving them side to side in the well.
					const float32 randomX =
						(((float32)rand() /  (float32)RAND_MAX) *
						 FrackerSettings::k_tileHalfWidth) - distance.x;
					b2Vec2 velocity = b2Vec2(randomX,
											 FrackerSettings::k_tileHeight);
					velocity.Normalize();
					velocityBuffer[i] = velocity *
						FrackerSettings::k_wellSuckSpeedInside;
				}
			}
		}

		// Draw everything.
		DrawPlayer();
		DrawWell();
		DrawScore();
	}

	// Render the well.
	void DrawWell()
	{
		for (int32 y = m_wellBottom; y <= m_wellTop; ++y)
		{
			DrawQuad(TileToWorld(m_wellX, y), FrackerSettings::k_wellColor);
		}
	}

	// Render the player / fracker.
	void DrawPlayer()
	{
		DrawQuad(
			m_player->GetTransform().p,
			Lerp(FrackerSettings::k_playerColor,
				 FrackerSettings::k_playerFrackColor,
				 b2Max(m_frackingFluidChargeTime /
					   FrackerSettings::k_frackingFluidChargeTime, 0.0f)),
			true);
	}

	// Render the score and the instructions / keys.
	void DrawScore()
	{
		char score[512];
		sprintf(score, "Score: %d, Remaining Oil %d",
		        m_listener.GetScore(), m_listener.GetOil());
		const char *lines[] = { score,  "Move: a,s,d,w   Fracking Fluid: e" };
		for (uint32 i = 0; i < B2_ARRAY_SIZE(lines); ++i)
		{
			g_debugDraw.DrawString(5, m_textLine, lines[i]);
			m_textLine += DRAW_STRING_NEW_LINE;
		}
	}

	// Draw a quad at position of color that is either just an outline
	// (fill = false) or solid (fill = true).
	void DrawQuad(const b2Vec2 &position, const b2Color &color,
				  bool fill = false)
	{
		b2Vec2 verts[4];
		const float32 maxX = position.x + FrackerSettings::k_tileWidth;
		const float32 maxY = position.y + FrackerSettings::k_tileHeight;
		verts[0].Set(position.x, maxY);
		verts[1].Set(position.x, position.y);
		verts[2].Set(maxX, position.y);
		verts[3].Set(maxX, maxY);
		if (fill)
		{
			g_debugDraw.DrawFlatPolygon(verts, 4, color);
		}
		else
		{
			g_debugDraw.DrawSolidPolygon(verts, 4, color);
		}
	}

	float32 GetDefaultViewZoom() const
	{
		return 0.1f;
	}

private:
	// Get a pointer to the material of the tile at the specified position.
	Material* GetMaterialStorage(const int32 x, const int32 y)
	{
		return &m_material[TileToArrayOffset(x, y)];
	}

	// A pointer to the body storage associated with the specified tile
	// position.
	b2Body** GetBodyStorage(const int32 x, const int32 y)
	{
		return &m_bodies[TileToArrayOffset(x, y)];
	}

public:
	static Test* Create()
	{
		return new Fracker;
	}

	// Get the bottom left position of the world in world units.
	static void GetBottomLeft(b2Vec2 * const bottomLeft)
	{
		bottomLeft->Set(FrackerSettings::k_worldCenterX -
						FrackerSettings::k_worldHalfWidth,
						FrackerSettings::k_worldCenterY -
						FrackerSettings::k_worldHalfHeight);
	}

	// Get the extents of the world in world units.
	static void GetExtents(b2Vec2 * const bottomLeft, b2Vec2 * const topRight)
	{
		GetBottomLeft(bottomLeft);
		topRight->Set(FrackerSettings::k_worldCenterX +
					  FrackerSettings::k_worldHalfWidth,
					  FrackerSettings::k_worldCenterY +
					  FrackerSettings::k_worldHalfHeight);
	}

	// Convert a point in world coordintes to a tile location
	static void WorldToTile(const b2Vec2& position, int32* const x,
							int32* const y)
	{
		// Translate relative to the world center and scale based upon the
		// tile size.
		b2Vec2 bottomLeft;
		GetBottomLeft(&bottomLeft);
		*x = (int32)(((position.x - bottomLeft.x) /
					  FrackerSettings::k_tileWidth) +
					 FrackerSettings::k_tileHalfWidth);
		*y = (int32)(((position.y - bottomLeft.y) /
					  FrackerSettings::k_tileHeight) +
					 FrackerSettings::k_tileHalfHeight);
	}

	// Convert a tile position to a point  in world coordinates.
	static b2Vec2 TileToWorld(const int32 x, const int32 y)
	{
		// Scale based upon the tile size and translate relative to the world
		// center.
		b2Vec2 bottomLeft;
		GetBottomLeft(&bottomLeft);
		return b2Vec2(
			((float32)x * FrackerSettings::k_tileWidth) + bottomLeft.x,
			((float32)y * FrackerSettings::k_tileHeight) + bottomLeft.y);
	}

	// Calculate the offset within an array of all world tiles using
	// the specified tile coordinates.
	static int32 TileToArrayOffset(const int32 x, const int32 y)
	{
		b2Assert(x >= 0);
		b2Assert(x < FrackerSettings::k_worldWidthTiles);
		b2Assert(y >= 0);
		b2Assert(y < FrackerSettings::k_worldHeightTiles);
		return x + (y * FrackerSettings::k_worldWidthTiles);
	}

	// Calculate the center of a tile position in world units.
	static b2Vec2 CenteredPosition(const b2Vec2& position)
	{
		return b2Vec2(position.x + FrackerSettings::k_tileHalfWidth,
					  position.y + FrackerSettings::k_tileHalfHeight);
	}

	// Interpolate between color a and b using t.
	static b2Color Lerp(const b2Color& a, const b2Color& b, const float32 t)
	{
		return b2Color(Lerp(a.r, b.r, t),
					   Lerp(a.g, b.g, t),
					   Lerp(a.b, b.b, t));
	}

	// Interpolate between a and b using t.
	static float32 Lerp(const float32 a, const float32 b, const float32 t)
	{
		return a * (1.0f - t) + b * t;
	}

private:
	b2Body* m_player;
	int32 m_wellX;
	int32 m_wellTop;
	int32 m_wellBottom;
	EmitterTracker m_tracker;
	bool m_allowInput;
	float32 m_frackingFluidChargeTime;
	Material m_material[FrackerSettings::k_worldTiles];
	b2Body* m_bodies[FrackerSettings::k_worldTiles];
	// Set of particle groups the well has influence over.
	DestructionListener m_listener;
};

#endif
