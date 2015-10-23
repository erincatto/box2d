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
#ifndef B2_PARTICLE_SYSTEM_H
#define B2_PARTICLE_SYSTEM_H

#include <Box2D/Common/b2SlabAllocator.h>
#include <Box2D/Common/b2GrowableBuffer.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Dynamics/b2TimeStep.h>

#if BOX2D_UNIT_TESTS
#include <gtest/gtest.h>
#endif // BOX2D_UNIT_TESTS

#if BOX2D_EXTERNAL_LANGUAGE_API
#include <cstring>
#endif // BOX2D_EXTERNAL_LANGUAGE_API

class b2World;
class b2Body;
class b2Shape;
class b2ParticleGroup;
class b2BlockAllocator;
class b2StackAllocator;
class b2QueryCallback;
class b2RayCastCallback;
class b2Fixture;
class b2ContactFilter;
class b2ContactListener;
class b2ParticlePairSet;
class FixtureParticleSet;
struct b2ParticleGroupDef;
struct b2Vec2;
struct b2AABB;
struct FindContactInput;
struct FindContactCheck;

struct b2ParticleContact
{
private:
	// 16-bit particle indices consume less memory and thus improve
	// performance. We iterate through m_contactBuffer many times during
	// b2ParticleSystem::Solve, so reducing the amount of data we churn
	// through speeds things up. Also, FindContactsFromChecks_Simd takes
	// advantage of the reduced size for specific optimizations.
	#ifdef B2_USE_16_BIT_PARTICLE_INDICES
		typedef int16 b2ParticleIndex;
	#else
		typedef int32 b2ParticleIndex;
	#endif

	/// Indices of the respective particles making contact.
	b2ParticleIndex indexA, indexB;

	/// Weight of the contact. A value between 0.0f and 1.0f.
	/// 0.0f ==> particles are just barely touching
	/// 1.0f ==> particles are perfectly on top of each other
	float32 weight;

	/// The normalized direction from A to B.
	b2Vec2 normal;

	/// The logical sum of the particle behaviors that have been set.
	/// See the b2ParticleFlag enum.
	uint32 flags;

public:
	void SetIndices(int32 a, int32 b);
	void SetWeight(float32 w) { weight = w; }
	void SetNormal(const b2Vec2& n) { normal = n; }
	void SetFlags(uint32 f) { flags = f; }

	int32 GetIndexA() const { return indexA; }
	int32 GetIndexB() const { return indexB; }
	float32 GetWeight() const { return weight; }
	const b2Vec2& GetNormal() const { return normal; }
	uint32 GetFlags() const { return flags; }

	bool operator==(const b2ParticleContact& rhs) const;
	bool operator!=(const b2ParticleContact& rhs) const { return !operator==(rhs); }
	bool ApproximatelyEqual(const b2ParticleContact& rhs) const;
};

struct b2ParticleBodyContact
{
	/// Index of the particle making contact.
	int32 index;

	/// The body making contact.
	b2Body* body;

	/// The specific fixture making contact
	b2Fixture* fixture;

	/// Weight of the contact. A value between 0.0f and 1.0f.
	float32 weight;

	/// The normalized direction from the particle to the body.
	b2Vec2 normal;

	/// The effective mass used in calculating force.
	float32 mass;
};

/// Connection between two particles
struct b2ParticlePair
{
	/// Indices of the respective particles making pair.
	int32 indexA, indexB;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// The initial distance of the particles.
	float32 distance;
};

/// Connection between three particles
struct b2ParticleTriad
{
	/// Indices of the respective particles making triad.
	int32 indexA, indexB, indexC;

	/// The logical sum of the particle flags. See the b2ParticleFlag enum.
	uint32 flags;

	/// The strength of cohesion among the particles.
	float32 strength;

	/// Values used for calculation.
	b2Vec2 pa, pb, pc;
	float32 ka, kb, kc, s;
};

struct b2ParticleSystemDef
{
	b2ParticleSystemDef()
	{
		strictContactCheck = false;
		density = 1.0f;
		gravityScale = 1.0f;
		radius = 1.0f;
		maxCount = 0;

		// Initialize physical coefficients to the maximum values that
		// maintain numerical stability.
		pressureStrength = 0.05f;
		dampingStrength = 1.0f;
		elasticStrength = 0.25f;
		springStrength = 0.25f;
		viscousStrength = 0.25f;
		surfaceTensionPressureStrength = 0.2f;
		surfaceTensionNormalStrength = 0.2f;
		repulsiveStrength = 1.0f;
		powderStrength = 0.5f;
		ejectionStrength = 0.5f;
		staticPressureStrength = 0.2f;
		staticPressureRelaxation = 0.2f;
		staticPressureIterations = 8;
		colorMixingStrength = 0.5f;
		destroyByAge = true;
		lifetimeGranularity = 1.0f / 60.0f;
	}

	/// Enable strict Particle/Body contact check.
	/// See SetStrictContactCheck for details.
	bool strictContactCheck;

	/// Set the particle density.
	/// See SetDensity for details.
	float32 density;

	/// Change the particle gravity scale. Adjusts the effect of the global
	/// gravity vector on particles. Default value is 1.0f.
	float32 gravityScale;

	/// Particles behave as circles with this radius. In Box2D units.
	float32 radius;

	/// Set the maximum number of particles.
	/// By default, there is no maximum. The particle buffers can continue to
	/// grow while b2World's block allocator still has memory.
	/// See SetMaxParticleCount for details.
	int32 maxCount;

	/// Increases pressure in response to compression
	/// Smaller values allow more compression
	float32 pressureStrength;

	/// Reduces velocity along the collision normal
	/// Smaller value reduces less
	float32 dampingStrength;

	/// Restores shape of elastic particle groups
	/// Larger values increase elastic particle velocity
	float32 elasticStrength;

	/// Restores length of spring particle groups
	/// Larger values increase spring particle velocity
	float32 springStrength;

	/// Reduces relative velocity of viscous particles
	/// Larger values slow down viscous particles more
	float32 viscousStrength;

	/// Produces pressure on tensile particles
	/// 0~0.2. Larger values increase the amount of surface tension.
	float32 surfaceTensionPressureStrength;

	/// Smoothes outline of tensile particles
	/// 0~0.2. Larger values result in rounder, smoother, water-drop-like
	/// clusters of particles.
	float32 surfaceTensionNormalStrength;

	/// Produces additional pressure on repulsive particles
	/// Larger values repulse more
	/// Negative values mean attraction. The range where particles behave
	/// stably is about -0.2 to 2.0.
	float32 repulsiveStrength;

	/// Produces repulsion between powder particles
	/// Larger values repulse more
	float32 powderStrength;

	/// Pushes particles out of solid particle group
	/// Larger values repulse more
	float32 ejectionStrength;

	/// Produces static pressure
	/// Larger values increase the pressure on neighboring partilces
	/// For a description of static pressure, see
	/// http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
	float32 staticPressureStrength;

	/// Reduces instability in static pressure calculation
	/// Larger values make stabilize static pressure with fewer iterations
	float32 staticPressureRelaxation;

	/// Computes static pressure more precisely
	/// See SetStaticPressureIterations for details
	int32 staticPressureIterations;

	/// Determines how fast colors are mixed
	/// 1.0f ==> mixed immediately
	/// 0.5f ==> mixed half way each simulation step (see b2World::Step())
	float32 colorMixingStrength;

	/// Whether to destroy particles by age when no more particles can be
	/// created.  See #b2ParticleSystem::SetDestructionByAge() for
	/// more information.
	bool destroyByAge;

	/// Granularity of particle lifetimes in seconds.  By default this is
	/// set to (1.0f / 60.0f) seconds.  b2ParticleSystem uses a 32-bit signed
	/// value to track particle lifetimes so the maximum lifetime of a
	/// particle is (2^32 - 1) / (1.0f / lifetimeGranularity) seconds.
	/// With the value set to 1/60 the maximum lifetime or age of a particle is
	/// 2.27 years.
	float32 lifetimeGranularity;
};


class b2ParticleSystem
{
public:
	/// Create a particle whose properties have been defined.
	/// No reference to the definition is retained.
	/// A simulation step must occur before it's possible to interact with a
	/// newly created particle.  For example, DestroyParticleInShape() will
	/// not destroy a particle until b2World::Step() has been called.
	/// @warning This function is locked during callbacks.
	/// @return the index of the particle.
	int32 CreateParticle(const b2ParticleDef& def);

	/// Retrieve a handle to the particle at the specified index.
	/// Please see #b2ParticleHandle for why you might want a handle.
	const b2ParticleHandle* GetParticleHandleFromIndex(const int32 index);

	/// Destroy a particle.
	/// The particle is removed after the next simulation step (see
	/// b2World::Step()).
	void DestroyParticle(int32 index)
	{
		DestroyParticle(index, false);
	}

	/// Destroy a particle.
	/// The particle is removed after the next step.
	/// @param Index of the particle to destroy.
	/// @param Whether to call the destruction listener just before the
	/// particle is destroyed.
	void DestroyParticle(int32 index, bool callDestructionListener);

	/// Destroy the Nth oldest particle in the system.
	/// The particle is removed after the next b2World::Step().
	/// @param Index of the Nth oldest particle to destroy, 0 will destroy the
	/// oldest particle in the system, 1 will destroy the next oldest
	/// particle etc.
	/// @param Whether to call the destruction listener just before the
	/// particle is destroyed.
	void DestroyOldestParticle(const int32 index,
							   const bool callDestructionListener);

	/// Destroy particles inside a shape without enabling the destruction
	/// callback for destroyed particles.
	/// This function is locked during callbacks.
	/// For more information see
	/// DestroyParticleInShape(const b2Shape&, const b2Transform&,bool).
	/// @param Shape which encloses particles that should be destroyed.
	/// @param Transform applied to the shape.
	/// @warning This function is locked during callbacks.
	/// @return Number of particles destroyed.
	int32 DestroyParticlesInShape(const b2Shape& shape, const b2Transform& xf)
	{
		return DestroyParticlesInShape(shape, xf, false);
	}

	/// Destroy particles inside a shape.
	/// This function is locked during callbacks.
	/// In addition, this function immediately destroys particles in the shape
	/// in constrast to DestroyParticle() which defers the destruction until
	/// the next simulation step.
	/// @param Shape which encloses particles that should be destroyed.
	/// @param Transform applied to the shape.
	/// @param Whether to call the world b2DestructionListener for each
	/// particle destroyed.
	/// @warning This function is locked during callbacks.
	/// @return Number of particles destroyed.
	int32 DestroyParticlesInShape(const b2Shape& shape, const b2Transform& xf,
	                              bool callDestructionListener);

	/// Create a particle group whose properties have been defined. No
	/// reference to the definition is retained.
	/// @warning This function is locked during callbacks.
	b2ParticleGroup* CreateParticleGroup(const b2ParticleGroupDef& def);

	/// Join two particle groups.
	/// @param the first group. Expands to encompass the second group.
	/// @param the second group. It is destroyed.
	/// @warning This function is locked during callbacks.
	void JoinParticleGroups(b2ParticleGroup* groupA, b2ParticleGroup* groupB);

	/// Split particle group into multiple disconnected groups.
	/// @param the group to be split.
	/// @warning This function is locked during callbacks.
	void SplitParticleGroup(b2ParticleGroup* group);

	/// Get the world particle group list. With the returned group, use
	/// b2ParticleGroup::GetNext to get the next group in the world list.
	/// A NULL group indicates the end of the list.
	/// @return the head of the world particle group list.
	b2ParticleGroup* GetParticleGroupList();
	const b2ParticleGroup* GetParticleGroupList() const;

	/// Get the number of particle groups.
	int32 GetParticleGroupCount() const;

	/// Get the number of particles.
	int32 GetParticleCount() const;

	/// Get the maximum number of particles.
	int32 GetMaxParticleCount() const;

	/// Set the maximum number of particles.
	/// A value of 0 means there is no maximum. The particle buffers can
	/// continue to grow while b2World's block allocator still has memory.
	/// Note: If you try to CreateParticle() with more than this count,
	/// b2_invalidParticleIndex is returned unless
	/// SetDestructionByAge() is used to enable the destruction of the
	/// oldest particles in the system.
	void SetMaxParticleCount(int32 count);

	/// Get all existing particle flags.
	uint32 GetAllParticleFlags() const;

	/// Get all existing particle group flags.
	uint32 GetAllGroupFlags() const;

	/// Pause or unpause the particle system. When paused, b2World::Step()
	/// skips over this particle system. All b2ParticleSystem function calls
	/// still work.
	/// @param paused is true to pause, false to un-pause.
	void SetPaused(bool paused);

	/// @return true if the particle system is being updated in
	/// b2World::Step().
	/// Initially, true, then, the last value passed into SetPaused().
	bool GetPaused() const;

	/// Change the particle density.
	/// Particle density affects the mass of the particles, which in turn
	/// affects how the particles interact with b2Bodies. Note that the density
	/// does not affect how the particles interact with each other.
	void SetDensity(float32 density);

	/// Get the particle density.
	float32 GetDensity() const;

	/// Change the particle gravity scale. Adjusts the effect of the global
	/// gravity vector on particles.
	void SetGravityScale(float32 gravityScale);

	/// Get the particle gravity scale.
	float32 GetGravityScale() const;

	/// Damping is used to reduce the velocity of particles. The damping
	/// parameter can be larger than 1.0f but the damping effect becomes
	/// sensitive to the time step when the damping parameter is large.
	void SetDamping(float32 damping);

	/// Get damping for particles
	float32 GetDamping() const;

	/// Change the number of iterations when calculating the static pressure of
	/// particles. By default, 8 iterations. You can reduce the number of
	/// iterations down to 1 in some situations, but this may cause
	/// instabilities when many particles come together. If you see particles
	/// popping away from each other like popcorn, you may have to increase the
	/// number of iterations.
	/// For a description of static pressure, see
	/// http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
	void SetStaticPressureIterations(int32 iterations);

	/// Get the number of iterations for static pressure of particles.
	int32 GetStaticPressureIterations() const;

	/// Change the particle radius.
	/// You should set this only once, on world start.
	/// If you change the radius during execution, existing particles may
	/// explode, shrink, or behave unexpectedly.
	void SetRadius(float32 radius);

	/// Get the particle radius.
	float32 GetRadius() const;

	/// Get the position of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle positions array.
	b2Vec2* GetPositionBuffer();
	const b2Vec2* GetPositionBuffer() const;

	/// Get the velocity of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle velocities array.
	b2Vec2* GetVelocityBuffer();
	const b2Vec2* GetVelocityBuffer() const;

	/// Get the color of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle colors array.
	b2ParticleColor* GetColorBuffer();
	const b2ParticleColor* GetColorBuffer() const;

	/// Get the particle-group of each particle.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle group array.
	b2ParticleGroup* const* GetGroupBuffer();
	const b2ParticleGroup* const* GetGroupBuffer() const;

	/// Get the weight of each particle
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle positions array.
	float32* GetWeightBuffer();
	const float32* GetWeightBuffer() const;

	/// Get the user-specified data of each particle.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle user-data array.
	void** GetUserDataBuffer();
	void* const* GetUserDataBuffer() const;

	/// Get the flags for each particle. See the b2ParticleFlag enum.
	/// Array is length GetParticleCount()
	/// @return the pointer to the head of the particle-flags array.
	const uint32* GetFlagsBuffer() const;

	/// Set flags for a particle. See the b2ParticleFlag enum.
	void SetParticleFlags(int32 index, uint32 flags);
	/// Get flags for a particle. See the b2ParticleFlag enum.
	uint32 GetParticleFlags(const int32 index);

	/// Set an external buffer for particle data.
	/// Normally, the b2World's block allocator is used for particle data.
	/// However, sometimes you may have an OpenGL or Java buffer for particle
	/// data. To avoid data duplication, you may supply this external buffer.
	///
	/// Note that, when b2World's block allocator is used, the particle data
	/// buffers can grow as required. However, when external buffers are used,
	/// the maximum number of particles is clamped to the size of the smallest
	/// external buffer.
	///
	/// @param buffer is a pointer to a block of memory.
	/// @param size is the number of values in the block.
	void SetFlagsBuffer(uint32* buffer, int32 capacity);
	void SetPositionBuffer(b2Vec2* buffer, int32 capacity);
	void SetVelocityBuffer(b2Vec2* buffer, int32 capacity);
	void SetColorBuffer(b2ParticleColor* buffer, int32 capacity);
	void SetUserDataBuffer(void** buffer, int32 capacity);

	/// Get contacts between particles
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	const b2ParticleContact* GetContacts() const;
	int32 GetContactCount() const;

	/// Get contacts between particles and bodies
	/// Contact data can be used for many reasons, for example to trigger
	/// rendering or audio effects.
	const b2ParticleBodyContact* GetBodyContacts() const;
	int32 GetBodyContactCount() const;

	/// Get array of particle pairs. The particles in a pair:
	///   (1) are contacting,
	///   (2) are in the same particle group,
	///   (3) are part of a rigid particle group, or are spring, elastic,
	///       or wall particles.
	///   (4) have at least one particle that is a spring or barrier
	///       particle (i.e. one of the types in k_pairFlags),
	///   (5) have at least one particle that returns true for
	///       ConnectionFilter::IsNecessary,
	///   (6) are not zombie particles.
	/// Essentially, this is an array of spring or barrier particles that
	/// are interacting. The array is sorted by b2ParticlePair's indexA,
	/// and then indexB. There are no duplicate entries.
	const b2ParticlePair* GetPairs() const;
	int32 GetPairCount() const;

	/// Get array of particle triads. The particles in a triad:
	///   (1) are in the same particle group,
	///   (2) are in a Voronoi triangle together,
	///   (3) are within b2_maxTriadDistance particle diameters of each
	///       other,
	///   (4) return true for ConnectionFilter::ShouldCreateTriad
	///   (5) have at least one particle of type elastic (i.e. one of the
	///       types in k_triadFlags),
	///   (6) are part of a rigid particle group, or are spring, elastic,
	///       or wall particles.
	///   (7) are not zombie particles.
	/// Essentially, this is an array of elastic particles that are
	/// interacting. The array is sorted by b2ParticleTriad's indexA,
	/// then indexB, then indexC. There are no duplicate entries.
	const b2ParticleTriad* GetTriads() const;
	int32 GetTriadCount() const;

	/// Set an optional threshold for the maximum number of
	/// consecutive particle iterations that a particle may contact
	/// multiple bodies before it is considered a candidate for being
	/// "stuck". Setting to zero or less disables.
	void SetStuckThreshold(int32 iterations);

	/// Get potentially stuck particles from the last step; the user must
	/// decide if they are stuck or not, and if so, delete or move them
	const int32* GetStuckCandidates() const;

	/// Get the number of stuck particle candidates from the last step.
	int32 GetStuckCandidateCount() const;

	/// Compute the kinetic energy that can be lost by damping force
	float32 ComputeCollisionEnergy() const;

	/// Set strict Particle/Body contact check.
	/// This is an option that will help ensure correct behavior if there are
	/// corners in the world model where Particle/Body contact is ambiguous.
	/// This option scales at n*log(n) of the number of Particle/Body contacts,
	/// so it is best to only enable if it is necessary for your geometry.
	/// Enable if you see strange particle behavior around b2Body
	/// intersections.
	void SetStrictContactCheck(bool enabled);
	/// Get the status of the strict contact check.
	bool GetStrictContactCheck() const;

	/// Set the lifetime (in seconds) of a particle relative to the current
	/// time.  A lifetime of less than or equal to 0.0f results in the particle
	/// living forever until it's manually destroyed by the application.
	void SetParticleLifetime(const int32 index, const float32 lifetime);
	/// Get the lifetime (in seconds) of a particle relative to the current
	/// time.  A value > 0.0f is returned if the particle is scheduled to be
	/// destroyed in the future, values <= 0.0f indicate the particle has an
	/// infinite lifetime.
	float32 GetParticleLifetime(const int32 index);

	/// Enable / disable destruction of particles in CreateParticle() when
	/// no more particles can be created due to a prior call to
	/// SetMaxParticleCount().  When this is enabled, the oldest particle is
	/// destroyed in CreateParticle() favoring the destruction of particles
	/// with a finite lifetime over particles with infinite lifetimes.
	/// This feature is enabled by default when particle lifetimes are
	/// tracked.  Explicitly enabling this feature using this function enables
	/// particle lifetime tracking.
	void SetDestructionByAge(const bool enable);
	/// Get whether the oldest particle will be destroyed in CreateParticle()
	/// when the maximum number of particles are present in the system.
	bool GetDestructionByAge() const;

	/// Get the array of particle expiration times indexed by particle index.
	/// GetParticleCount() items are in the returned array.
	const int32* GetExpirationTimeBuffer();
	/// Convert a expiration time value in returned by
	/// GetExpirationTimeBuffer() to a time in seconds relative to the
	/// current simulation time.
	float32 ExpirationTimeToLifetime(const int32 expirationTime) const;
	/// Get the array of particle indices ordered by reverse lifetime.
	/// The oldest particle indexes are at the end of the array with the
	/// newest at the start.  Particles with infinite lifetimes
	/// (i.e expiration times less than or equal to 0) are placed at the start
	///  of the array.
	/// ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index])
	/// is equivalent to GetParticleLifetime(index).
	/// GetParticleCount() items are in the returned array.
	const int32* GetIndexByExpirationTimeBuffer();

	/// Apply an impulse to one particle. This immediately modifies the
	/// velocity. Similar to b2Body::ApplyLinearImpulse.
	/// @param index the particle that will be modified.
	/// @param impulse the world impulse vector, usually in N-seconds or
    ///        kg-m/s.
	void ParticleApplyLinearImpulse(int32 index, const b2Vec2& impulse);

	/// Apply an impulse to all particles between 'firstIndex' and 'lastIndex'.
	/// This immediately modifies the velocity. Note that the impulse is
	/// applied to the total mass of all particles. So, calling
	/// ParticleApplyLinearImpulse(0, impulse) and
	/// ParticleApplyLinearImpulse(1, impulse) will impart twice as much
	/// velocity as calling just ApplyLinearImpulse(0, 1, impulse).
	/// @param firstIndex the first particle to be modified.
	/// @param lastIndex the last particle to be modified.
	/// @param impulse the world impulse vector, usually in N-seconds or
    ///        kg-m/s.
	void ApplyLinearImpulse(int32 firstIndex, int32 lastIndex,
							const b2Vec2& impulse);

	/// Apply a force to the center of a particle.
	/// @param index the particle that will be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ParticleApplyForce(int32 index, const b2Vec2& force);

	/// Distribute a force across several particles. The particles must not be
	/// wall particles. Note that the force is distributed across all the
	/// particles, so calling this function for indices 0..N is not the same as
	/// calling ParticleApplyForce(i, force) for i in 0..N.
	/// @param firstIndex the first particle to be modified.
	/// @param lastIndex the last particle to be modified.
	/// @param force the world force vector, usually in Newtons (N).
	void ApplyForce(int32 firstIndex, int32 lastIndex, const b2Vec2& force);

	/// Get the next particle-system in the world's particle-system list.
	b2ParticleSystem* GetNext();
	const b2ParticleSystem* GetNext() const;

	/// Query the particle system for all particles that potentially overlap
	/// the provided AABB. b2QueryCallback::ShouldQueryParticleSystem is
	/// ignored.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

	/// Query the particle system for all particles that potentially overlap
	/// the provided shape's AABB. Calls QueryAABB internally.
	/// b2QueryCallback::ShouldQueryParticleSystem is ignored.
	/// @param callback a user implemented callback class.
	/// @param shape the query shape
	/// @param xf the transform of the AABB
	void QueryShapeAABB(b2QueryCallback* callback, const b2Shape& shape,
						const b2Transform& xf) const;

	/// Ray-cast the particle system for all particles in the path of the ray.
	/// Your callback controls whether you get the closest point, any point, or
	/// n-points. The ray-cast ignores particles that contain the starting
	/// point. b2RayCastCallback::ShouldQueryParticleSystem is ignored.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b2RayCastCallback* callback, const b2Vec2& point1,
				 const b2Vec2& point2) const;

	/// Compute the axis-aligned bounding box for all particles contained
	/// within this particle system.
	/// @param aabb Returns the axis-aligned bounding box of the system.
	void ComputeAABB(b2AABB* const aabb) const;

#if BOX2D_EXTERNAL_LANGUAGE_API
public:
	enum b2ExceptionType
	{
		b2_bufferTooSmall,
		b2_particleIndexOutOfBounds,
		b2_numErrors,
		b2_noExceptions,
	};

	/// Set the velocity of particle at index with direct floats.
	void SetParticleVelocity(int32 index, float32 vx, float32 vy);

	/// Get the x-coordinate of particle at index.
	float GetParticlePositionX(int32 index) const;

	/// Get the y-coordinate of particle at index.
	float GetParticlePositionY(int32 index) const;

	/// Copy position buffer into a specified buffer, starting from startIndex.
	int CopyPositionBuffer(int startIndex, int numParticles, void* outBuf,
						   int size) const;

	/// Copy color buffer into a specified buffer, starting from startIndex.
	int CopyColorBuffer(int startIndex, int numParticles, void* outBuf,
					    int size) const;

	/// Copy color buffer into a specified buffer, starting from startIndex.
	int CopyWeightBuffer(int startIndex, int numParticles, void* outBuf,
						 int size) const;

private:
	/// Helper function for buffer copies.
	int CopyBuffer(int startIndex, int numParticles, void* inBufWithOffset,
				   void* outBuf, int outBufSize, int copySize) const;

	/// Check if buffer copy is valid for the Get*Buffer functions that have
	/// a user-supplied output buffer.
	b2ExceptionType IsBufCopyValid(int startIndex, int numParticles,
								   int copySize, int bufSize) const;
#endif // BOX2D_EXTERNAL_LANGUAGE_API

private:
	friend class b2World;
	friend class b2ParticleGroup;
	friend class b2ParticleBodyContactRemovePredicate;
	friend class b2FixtureParticleQueryCallback;
#ifdef BOX2D_UNIT_TESTS
	FRIEND_TEST(FunctionTests, GetParticleMass);
	FRIEND_TEST(FunctionTests, AreProxyBuffersTheSame);
#endif // BOX2D_UNIT_TESTS

	template <typename T>
	struct UserOverridableBuffer
	{
		UserOverridableBuffer()
		{
			data = NULL;
			userSuppliedCapacity = 0;
		}
		T* data;
		int32 userSuppliedCapacity;
	};

	/// Used for detecting particle contacts
	struct Proxy
	{
		int32 index;
		uint32 tag;
		friend inline bool operator<(const Proxy &a, const Proxy &b)
		{
			return a.tag < b.tag;
		}
		friend inline bool operator<(uint32 a, const Proxy &b)
		{
			return a < b.tag;
		}
		friend inline bool operator<(const Proxy &a, uint32 b)
		{
			return a.tag < b;
		}
	};

	/// Class for filtering pairs or triads.
	class ConnectionFilter
	{
	public:
		virtual ~ConnectionFilter() {}
		/// Is the particle necessary for connection?
		/// A pair or a triad should contain at least one 'necessary' particle.
		virtual bool IsNecessary(int32 index) const
		{
			B2_NOT_USED(index);
			return true;
		}
		/// An additional condition for creating a pair.
		virtual bool ShouldCreatePair(int32 a, int32 b) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			return true;
		}
		/// An additional condition for creating a triad.
		virtual bool ShouldCreateTriad(int32 a, int32 b, int32 c) const
		{
			B2_NOT_USED(a);
			B2_NOT_USED(b);
			B2_NOT_USED(c);
			return true;
		}
	};

	/// InsideBoundsEnumerator enumerates all particles inside the given bounds.
	class InsideBoundsEnumerator
	{
	public:
		/// Construct an enumerator with bounds of tags and a range of proxies.
		InsideBoundsEnumerator(
			uint32 lower, uint32 upper,
			const Proxy* first, const Proxy* last);

		/// Get index of the next particle. Returns b2_invalidParticleIndex if
		/// there are no more particles.
		int32 GetNext();
	private:
		/// The lower and upper bound of x component in the tag.
		uint32 m_xLower, m_xUpper;
		/// The lower and upper bound of y component in the tag.
		uint32 m_yLower, m_yUpper;
		/// The range of proxies.
		const Proxy* m_first;
		const Proxy* m_last;
	};

	/// Node of linked lists of connected particles
	struct ParticleListNode
	{
		/// The head of the list.
		ParticleListNode* list;
		/// The next node in the list.
		ParticleListNode* next;
		/// Number of entries in the list. Valid only for the node at the head
		/// of the list.
		int32 count;
		/// Particle index.
		int32 index;
	};

	/// All particle types that require creating pairs
	static const int32 k_pairFlags =
		b2_springParticle |
		b2_barrierParticle;
	/// All particle types that require creating triads
	static const int32 k_triadFlags =
		b2_elasticParticle;
	/// All particle types that do not produce dynamic pressure
	static const int32 k_noPressureFlags =
		b2_powderParticle |
		b2_tensileParticle;
	/// All particle types that apply extra damping force with bodies
	static const int32 k_extraDampingFlags =
		b2_staticPressureParticle;

	b2ParticleSystem(const b2ParticleSystemDef* def, b2World* world);
	~b2ParticleSystem();

	template <typename T> void FreeBuffer(T** b, int capacity);
	template <typename T> void FreeUserOverridableBuffer(
		UserOverridableBuffer<T>* b);
	template <typename T> T* ReallocateBuffer(T* buffer, int32 oldCapacity,
											  int32 newCapacity);
	template <typename T> T* ReallocateBuffer(
		T* buffer, int32 userSuppliedCapacity, int32 oldCapacity,
		int32 newCapacity, bool deferred);
	template <typename T> T* ReallocateBuffer(
		UserOverridableBuffer<T>* buffer, int32 oldCapacity, int32 newCapacity,
		bool deferred);
	template <typename T> T* RequestBuffer(T* buffer);

	/// Reallocate the handle / index map and schedule the allocation of a new
	/// pool for handle allocation.
	void ReallocateHandleBuffers(int32 newCapacity);

	void ReallocateInternalAllocatedBuffers(int32 capacity);
	int32 CreateParticleForGroup(
		const b2ParticleGroupDef& groupDef,
		const b2Transform& xf, const b2Vec2& position);
	void CreateParticlesStrokeShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesFillShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesWithShapeForGroup(
		const b2Shape* shape,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	void CreateParticlesWithShapesForGroup(
		const b2Shape* const* shapes, int32 shapeCount,
		const b2ParticleGroupDef& groupDef, const b2Transform& xf);
	int32 CloneParticle(int32 index, b2ParticleGroup* group);
	void DestroyParticleGroup(b2ParticleGroup* group);

	void UpdatePairsAndTriads(
		int32 firstIndex, int32 lastIndex, const ConnectionFilter& filter);
	void UpdatePairsAndTriadsWithReactiveParticles();
	static bool ComparePairIndices(const b2ParticlePair& a, const b2ParticlePair& b);
	static bool MatchPairIndices(const b2ParticlePair& a, const b2ParticlePair& b);
	static bool CompareTriadIndices(const b2ParticleTriad& a, const b2ParticleTriad& b);
	static bool MatchTriadIndices(const b2ParticleTriad& a, const b2ParticleTriad& b);

	static void InitializeParticleLists(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer);
	void MergeParticleListsInContact(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer) const;
	static void MergeParticleLists(
		ParticleListNode* listA, ParticleListNode* listB);
	static ParticleListNode* FindLongestParticleList(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer);
	void MergeZombieParticleListNodes(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
		ParticleListNode* survivingList) const;
	static void MergeParticleListAndNode(
		ParticleListNode* list, ParticleListNode* node);
	void CreateParticleGroupsFromParticleList(
		const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
		const ParticleListNode* survivingList);
	void UpdatePairsAndTriadsWithParticleList(
		const b2ParticleGroup* group, const ParticleListNode* nodeBuffer);

	void ComputeDepth();

	InsideBoundsEnumerator GetInsideBoundsEnumerator(const b2AABB& aabb) const;

	void UpdateAllParticleFlags();
	void UpdateAllGroupFlags();
	void AddContact(int32 a, int32 b,
		b2GrowableBuffer<b2ParticleContact>& contacts) const;
	void FindContacts_Reference(
		b2GrowableBuffer<b2ParticleContact>& contacts) const;
	void ReorderForFindContact(FindContactInput* reordered,
		                       int alignedCount) const;
	void GatherChecksOneParticle(
		const uint32 bound,
		const int startIndex,
		const int particleIndex,
		int* nextUncheckedIndex,
		b2GrowableBuffer<FindContactCheck>& checks) const;
	void GatherChecks(b2GrowableBuffer<FindContactCheck>& checks) const;
	void FindContacts_Simd(
		b2GrowableBuffer<b2ParticleContact>& contacts) const;
	void FindContacts(
		b2GrowableBuffer<b2ParticleContact>& contacts) const;
	static void UpdateProxyTags(
		const uint32* const tags,
		b2GrowableBuffer<Proxy>& proxies);
	static bool ProxyBufferHasIndex(
		int32 index, const Proxy* const a, int count);
	static int NumProxiesWithSameTag(
		const Proxy* const a, const Proxy* const b, int count);
	static bool AreProxyBuffersTheSame(const b2GrowableBuffer<Proxy>& a,
								   	   const b2GrowableBuffer<Proxy>& b);
	void UpdateProxies_Reference(b2GrowableBuffer<Proxy>& proxies) const;
	void UpdateProxies_Simd(b2GrowableBuffer<Proxy>& proxies) const;
	void UpdateProxies(b2GrowableBuffer<Proxy>& proxies) const;
	void SortProxies(b2GrowableBuffer<Proxy>& proxies) const;
	void FilterContacts(b2GrowableBuffer<b2ParticleContact>& contacts);
	void NotifyContactListenerPreContact(
		b2ParticlePairSet* particlePairs) const;
	void NotifyContactListenerPostContact(b2ParticlePairSet& particlePairs);
	void UpdateContacts(bool exceptZombie);
	void NotifyBodyContactListenerPreContact(
		FixtureParticleSet* fixtureSet) const;
	void NotifyBodyContactListenerPostContact(FixtureParticleSet& fixtureSet);
	void UpdateBodyContacts();

	void Solve(const b2TimeStep& step);
	void SolveCollision(const b2TimeStep& step);
	void LimitVelocity(const b2TimeStep& step);
	void SolveGravity(const b2TimeStep& step);
	void SolveBarrier(const b2TimeStep& step);
	void SolveStaticPressure(const b2TimeStep& step);
	void ComputeWeight();
	void SolvePressure(const b2TimeStep& step);
	void SolveDamping(const b2TimeStep& step);
	void SolveRigidDamping();
	void SolveExtraDamping();
	void SolveWall();
	void SolveRigid(const b2TimeStep& step);
	void SolveElastic(const b2TimeStep& step);
	void SolveSpring(const b2TimeStep& step);
	void SolveTensile(const b2TimeStep& step);
	void SolveViscous();
	void SolveRepulsive(const b2TimeStep& step);
	void SolvePowder(const b2TimeStep& step);
	void SolveSolid(const b2TimeStep& step);
	void SolveForce(const b2TimeStep& step);
	void SolveColorMixing();
	void SolveZombie();
	/// Destroy all particles which have outlived their lifetimes set by
	/// SetParticleLifetime().
	void SolveLifetimes(const b2TimeStep& step);
	void RotateBuffer(int32 start, int32 mid, int32 end);

	float32 GetCriticalVelocity(const b2TimeStep& step) const;
	float32 GetCriticalVelocitySquared(const b2TimeStep& step) const;
	float32 GetCriticalPressure(const b2TimeStep& step) const;
	float32 GetParticleStride() const;
	float32 GetParticleMass() const;
	float32 GetParticleInvMass() const;

	// Get the world's contact filter if any particles with the
	// b2_contactFilterParticle flag are present in the system.
	b2ContactFilter* GetFixtureContactFilter() const;

	// Get the world's contact filter if any particles with the
	// b2_particleContactFilterParticle flag are present in the system.
	b2ContactFilter* GetParticleContactFilter() const;

	// Get the world's contact listener if any particles with the
	// b2_fixtureContactListenerParticle flag are present in the system.
	b2ContactListener* GetFixtureContactListener() const;

	// Get the world's contact listener if any particles with the
	// b2_particleContactListenerParticle flag are present in the system.
	b2ContactListener* GetParticleContactListener() const;

	template <typename T> void SetUserOverridableBuffer(
		UserOverridableBuffer<T>* buffer, T* newBufferData, int32 newCapacity);

	void SetGroupFlags(b2ParticleGroup* group, uint32 flags);

	void RemoveSpuriousBodyContacts();
	static bool BodyContactCompare(const b2ParticleBodyContact& lhs,
								   const b2ParticleBodyContact& rhs);

	void DetectStuckParticle(int32 particle);

	/// Determine whether a particle index is valid.
	bool ValidateParticleIndex(const int32 index) const;

	/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
	int32 GetQuantizedTimeElapsed() const;
	/// Convert a lifetime in seconds to an expiration time.
	int64 LifetimeToExpirationTime(const float32 lifetime) const;

	bool ForceCanBeApplied(uint32 flags) const;
	void PrepareForceBuffer();

	bool IsRigidGroup(b2ParticleGroup *group) const;
	b2Vec2 GetLinearVelocity(
		b2ParticleGroup *group, int32 particleIndex,
		const b2Vec2 &point) const;
	void InitDampingParameter(
		float32* invMass, float32* invInertia, float32* tangentDistance,
		float32 mass, float32 inertia, const b2Vec2& center,
		const b2Vec2& point, const b2Vec2& normal) const;
	void InitDampingParameterWithRigidGroupOrParticle(
		float32* invMass, float32* invInertia, float32* tangentDistance,
		bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
		const b2Vec2& point, const b2Vec2& normal) const;
	float32 ComputeDampingImpulse(
		float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
		float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
		float32 normalVelocity) const;
	void ApplyDamping(
		float32 invMass, float32 invInertia, float32 tangentDistance,
		bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
		float32 impulse, const b2Vec2& normal);

	bool m_paused;
	int32 m_timestamp;
	int32 m_allParticleFlags;
	bool m_needsUpdateAllParticleFlags;
	int32 m_allGroupFlags;
	bool m_needsUpdateAllGroupFlags;
	bool m_hasForce;
	int32 m_iterationIndex;
	float32 m_inverseDensity;
	float32 m_particleDiameter;
	float32 m_inverseDiameter;
	float32 m_squaredDiameter;

	int32 m_count;
	int32 m_internalAllocatedCapacity;
	/// Allocator for b2ParticleHandle instances.
	b2SlabAllocator<b2ParticleHandle> m_handleAllocator;
	/// Maps particle indicies to  handles.
	UserOverridableBuffer<b2ParticleHandle*> m_handleIndexBuffer;
	UserOverridableBuffer<uint32> m_flagsBuffer;
	UserOverridableBuffer<b2Vec2> m_positionBuffer;
	UserOverridableBuffer<b2Vec2> m_velocityBuffer;
	b2Vec2* m_forceBuffer;
	/// m_weightBuffer is populated in ComputeWeight and used in
	/// ComputeDepth(), SolveStaticPressure() and SolvePressure().
	float32* m_weightBuffer;
	/// When any particles have the flag b2_staticPressureParticle,
	/// m_staticPressureBuffer is first allocated and used in
	/// SolveStaticPressure() and SolvePressure().  It will be reallocated on
	/// subsequent CreateParticle() calls.
	float32* m_staticPressureBuffer;
	/// m_accumulationBuffer is used in many functions as a temporary buffer
	/// for scalar values.
	float32* m_accumulationBuffer;
	/// When any particles have the flag b2_tensileParticle,
	/// m_accumulation2Buffer is first allocated and used in SolveTensile()
	/// as a temporary buffer for vector values.  It will be reallocated on
	/// subsequent CreateParticle() calls.
	b2Vec2* m_accumulation2Buffer;
	/// When any particle groups have the flag b2_solidParticleGroup,
	/// m_depthBuffer is first allocated and populated in ComputeDepth() and
	/// used in SolveSolid(). It will be reallocated on subsequent
	/// CreateParticle() calls.
	float32* m_depthBuffer;
	UserOverridableBuffer<b2ParticleColor> m_colorBuffer;
	b2ParticleGroup** m_groupBuffer;
	UserOverridableBuffer<void*> m_userDataBuffer;

	/// Stuck particle detection parameters and record keeping
	int32 m_stuckThreshold;
	UserOverridableBuffer<int32> m_lastBodyContactStepBuffer;
	UserOverridableBuffer<int32> m_bodyContactCountBuffer;
	UserOverridableBuffer<int32> m_consecutiveContactStepsBuffer;
	b2GrowableBuffer<int32> m_stuckParticleBuffer;
	b2GrowableBuffer<Proxy> m_proxyBuffer;
	b2GrowableBuffer<b2ParticleContact> m_contactBuffer;
	b2GrowableBuffer<b2ParticleBodyContact> m_bodyContactBuffer;
	b2GrowableBuffer<b2ParticlePair> m_pairBuffer;
	b2GrowableBuffer<b2ParticleTriad> m_triadBuffer;

	/// Time each particle should be destroyed relative to the last time
	/// m_timeElapsed was initialized.  Each unit of time corresponds to
	/// b2ParticleSystemDef::lifetimeGranularity seconds.
	UserOverridableBuffer<int32> m_expirationTimeBuffer;
	/// List of particle indices sorted by expiration time.
	UserOverridableBuffer<int32> m_indexByExpirationTimeBuffer;
	/// Time elapsed in 32:32 fixed point.  Each non-fractional unit of time
	/// corresponds to b2ParticleSystemDef::lifetimeGranularity seconds.
	int64 m_timeElapsed;
	/// Whether the expiration time buffer has been modified and needs to be
	/// resorted.
	bool m_expirationTimeBufferRequiresSorting;

	int32 m_groupCount;
	b2ParticleGroup* m_groupList;

	b2ParticleSystemDef m_def;

	b2World* m_world;
	b2ParticleSystem* m_prev;
	b2ParticleSystem* m_next;
};

inline void b2ParticleContact::SetIndices(int32 a, int32 b)
{
	b2Assert(a <= b2_maxParticleIndex && b <= b2_maxParticleIndex);
	indexA = (b2ParticleIndex)a;
	indexB = (b2ParticleIndex)b;
}


inline bool b2ParticleContact::operator==(
	const b2ParticleContact& rhs) const
{
	return indexA == rhs.indexA
		&& indexB == rhs.indexB
		&& flags == rhs.flags
		&& weight == rhs.weight
		&& normal == rhs.normal;
}

// The reciprocal sqrt function differs between SIMD and non-SIMD, but they
// should create approximately equal results.
inline bool b2ParticleContact::ApproximatelyEqual(
	const b2ParticleContact& rhs) const
{
	static const float MAX_WEIGHT_DIFF = 0.01f; // Weight 0 ~ 1, so about 1%
	static const float MAX_NORMAL_DIFF = 0.01f; // Normal length = 1, so 1%
	return indexA == rhs.indexA
		&& indexB == rhs.indexB
		&& flags == rhs.flags
		&& b2Abs(weight - rhs.weight) < MAX_WEIGHT_DIFF
		&& (normal - rhs.normal).Length() < MAX_NORMAL_DIFF;
}

inline b2ParticleGroup* b2ParticleSystem::GetParticleGroupList()
{
	return m_groupList;
}

inline const b2ParticleGroup* b2ParticleSystem::GetParticleGroupList() const
{
	return m_groupList;
}

inline int32 b2ParticleSystem::GetParticleGroupCount() const
{
	return m_groupCount;
}

inline int32 b2ParticleSystem::GetParticleCount() const
{
	return m_count;
}

inline void b2ParticleSystem::SetPaused(bool paused)
{
	m_paused = paused;
}

inline bool b2ParticleSystem::GetPaused() const
{
	return m_paused;
}

inline const b2ParticleContact* b2ParticleSystem::GetContacts() const
{
	return m_contactBuffer.Data();
}

inline int32 b2ParticleSystem::GetContactCount() const
{
	return m_contactBuffer.GetCount();
}

inline const b2ParticleBodyContact* b2ParticleSystem::GetBodyContacts() const
{
	return m_bodyContactBuffer.Data();
}

inline int32 b2ParticleSystem::GetBodyContactCount() const
{
	return m_bodyContactBuffer.GetCount();
}

inline const b2ParticlePair* b2ParticleSystem::GetPairs() const
{
	return m_pairBuffer.Data();
}

inline int32 b2ParticleSystem::GetPairCount() const
{
	return m_pairBuffer.GetCount();
}

inline const b2ParticleTriad* b2ParticleSystem::GetTriads() const
{
	return m_triadBuffer.Data();
}

inline int32 b2ParticleSystem::GetTriadCount() const
{
	return m_triadBuffer.GetCount();
}

inline b2ParticleSystem* b2ParticleSystem::GetNext()
{
	return m_next;
}

inline const b2ParticleSystem* b2ParticleSystem::GetNext() const
{
	return m_next;
}

inline const int32* b2ParticleSystem::GetStuckCandidates() const
{
	return m_stuckParticleBuffer.Data();
}

inline int32 b2ParticleSystem::GetStuckCandidateCount() const
{
	return m_stuckParticleBuffer.GetCount();
}

inline void b2ParticleSystem::SetStrictContactCheck(bool enabled)
{
	m_def.strictContactCheck = enabled;
}

inline bool b2ParticleSystem::GetStrictContactCheck() const
{
	return m_def.strictContactCheck;
}

inline void b2ParticleSystem::SetRadius(float32 radius)
{
	m_particleDiameter = 2 * radius;
	m_squaredDiameter = m_particleDiameter * m_particleDiameter;
	m_inverseDiameter = 1 / m_particleDiameter;
}

inline void b2ParticleSystem::SetDensity(float32 density)
{
	m_def.density = density;
	m_inverseDensity =  1 / m_def.density;
}

inline float32 b2ParticleSystem::GetDensity() const
{
	return m_def.density;
}

inline void b2ParticleSystem::SetGravityScale(float32 gravityScale)
{
	m_def.gravityScale = gravityScale;
}

inline float32 b2ParticleSystem::GetGravityScale() const
{
	return m_def.gravityScale;
}

inline void b2ParticleSystem::SetDamping(float32 damping)
{
	m_def.dampingStrength = damping;
}

inline float32 b2ParticleSystem::GetDamping() const
{
	return m_def.dampingStrength;
}

inline void b2ParticleSystem::SetStaticPressureIterations(int32 iterations)
{
	m_def.staticPressureIterations = iterations;
}

inline int32 b2ParticleSystem::GetStaticPressureIterations() const
{
	return m_def.staticPressureIterations;
}

inline float32 b2ParticleSystem::GetRadius() const
{
	return m_particleDiameter / 2;
}

inline float32 b2ParticleSystem::GetCriticalVelocity(const b2TimeStep& step) const
{
	return m_particleDiameter * step.inv_dt;
}

inline float32 b2ParticleSystem::GetCriticalVelocitySquared(
	const b2TimeStep& step) const
{
	float32 velocity = GetCriticalVelocity(step);
	return velocity * velocity;
}

inline float32 b2ParticleSystem::GetCriticalPressure(const b2TimeStep& step) const
{
	return m_def.density * GetCriticalVelocitySquared(step);
}

inline float32 b2ParticleSystem::GetParticleStride() const
{
	return b2_particleStride * m_particleDiameter;
}

inline float32 b2ParticleSystem::GetParticleMass() const
{
	float32 stride = GetParticleStride();
	return m_def.density * stride * stride;
}

inline float32 b2ParticleSystem::GetParticleInvMass() const
{
	// mass = density * stride^2, so we take the inverse of this.
	float32 inverseStride = m_inverseDiameter * (1.0f / b2_particleStride);
	return m_inverseDensity * inverseStride * inverseStride;
}

inline b2Vec2* b2ParticleSystem::GetPositionBuffer()
{
	return m_positionBuffer.data;
}

inline b2Vec2* b2ParticleSystem::GetVelocityBuffer()
{
	return m_velocityBuffer.data;
}

inline float32* b2ParticleSystem::GetWeightBuffer()
{
	return m_weightBuffer;
}

inline int32 b2ParticleSystem::GetMaxParticleCount() const
{
	return m_def.maxCount;
}

inline void b2ParticleSystem::SetMaxParticleCount(int32 count)
{
	b2Assert(m_count <= count);
	m_def.maxCount = count;
}

inline uint32 b2ParticleSystem::GetAllParticleFlags() const
{
	return m_allParticleFlags;
}

inline uint32 b2ParticleSystem::GetAllGroupFlags() const
{
	return m_allGroupFlags;
}

inline const uint32* b2ParticleSystem::GetFlagsBuffer() const
{
	return m_flagsBuffer.data;
}

inline const b2Vec2* b2ParticleSystem::GetPositionBuffer() const
{
	return m_positionBuffer.data;
}

inline const b2Vec2* b2ParticleSystem::GetVelocityBuffer() const
{
	return m_velocityBuffer.data;
}

inline const b2ParticleColor* b2ParticleSystem::GetColorBuffer() const
{
	return ((b2ParticleSystem*) this)->GetColorBuffer();
}

inline const b2ParticleGroup* const* b2ParticleSystem::GetGroupBuffer() const
{
	return m_groupBuffer;
}

inline const float32* b2ParticleSystem::GetWeightBuffer() const
{
	return m_weightBuffer;
}

inline void* const* b2ParticleSystem::GetUserDataBuffer() const
{
	return ((b2ParticleSystem*) this)->GetUserDataBuffer();
}

inline b2ParticleGroup* const* b2ParticleSystem::GetGroupBuffer()
{
	return m_groupBuffer;
}

inline uint32 b2ParticleSystem::GetParticleFlags(int32 index)
{
	return GetFlagsBuffer()[index];
}

inline bool b2ParticleSystem::ValidateParticleIndex(const int32 index) const
{
	return index >= 0 && index < GetParticleCount() &&
		index != b2_invalidParticleIndex;
}

inline bool b2ParticleSystem::GetDestructionByAge() const
{
	return m_def.destroyByAge;
}

inline void b2ParticleSystem::ParticleApplyLinearImpulse(int32 index,
														 const b2Vec2& impulse)
{
	ApplyLinearImpulse(index, index + 1, impulse);
}


// Note: These functions must go in the header so the unit tests will compile
// them. b2ParticleSystem.cpp does not compile with this #define.
#if BOX2D_EXTERNAL_LANGUAGE_API

inline void b2ParticleSystem::SetParticleVelocity(int32 index,
												  float32 vx,
												  float32 vy)
{
	b2Vec2& v = GetVelocityBuffer()[index];
	v.x = vx;
	v.y = vy;
}

inline float b2ParticleSystem::GetParticlePositionX(int32 index) const
{
	return GetPositionBuffer()[index].x;
}

inline float b2ParticleSystem::GetParticlePositionY(int32 index) const
{
	return GetPositionBuffer()[index].y;
}

inline int b2ParticleSystem::CopyPositionBuffer(int startIndex,
												int numParticles,
												void* outBuf,
												int size) const
{
	int copySize = numParticles * sizeof(b2Vec2);
	void* inBufWithOffset = (void*) (GetPositionBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyColorBuffer(int startIndex,
											 int numParticles,
											 void* outBuf,
											 int size) const
{
	int copySize = numParticles * sizeof(b2ParticleColor);
	void* inBufWithOffset = (void*) (GetColorBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyWeightBuffer(int startIndex,
											  int numParticles,
											  void* outBuf,
											  int size) const
{
	int copySize = numParticles * sizeof(float32);
	void* inBufWithOffset = (void*) (GetWeightBuffer() + startIndex);
	return CopyBuffer(startIndex, numParticles, inBufWithOffset, outBuf, size,
					  copySize);
}

inline int b2ParticleSystem::CopyBuffer(int startIndex, int numParticles,
										void* inBufWithOffset, void* outBuf,
										int outBufSize, int copySize) const
{
	b2ExceptionType exception = IsBufCopyValid(startIndex, numParticles,
											   copySize, outBufSize);
	if (exception != b2_noExceptions)
	{
		return exception;
	}

	memcpy(outBuf, inBufWithOffset, copySize);
	return b2_noExceptions;
}

#endif // BOX2D_EXTERNAL_LANGUAGE_API

#endif

