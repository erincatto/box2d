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
#ifndef B2_PARTICLE_GROUP
#define B2_PARTICLE_GROUP

#include <Box2D/Particle/b2Particle.h>

class b2Shape;
class b2World;
class b2ParticleSystem;
class b2ParticleGroup;
class b2ParticleColor;
#if BOX2D_EXTERNAL_LANGUAGE_API
class b2CircleShape;
#endif // BOX2D_EXTERNAL_LANGUAGE_API

/// @file

/// The particle group type.  Can be combined with the | operator.
enum b2ParticleGroupFlag
{
	/// Prevents overlapping or leaking.
	b2_solidParticleGroup = 1 << 0,
	/// Keeps its shape.
	b2_rigidParticleGroup = 1 << 1,
	/// Won't be destroyed if it gets empty.
	b2_particleGroupCanBeEmpty = 1 << 2,
	/// Will be destroyed on next simulation step.
	b2_particleGroupWillBeDestroyed = 1 << 3,
	/// Updates depth data on next simulation step.
	b2_particleGroupNeedsUpdateDepth = 1 << 4,
	b2_particleGroupInternalMask =
		b2_particleGroupWillBeDestroyed |
		b2_particleGroupNeedsUpdateDepth,
};

/// A particle group definition holds all the data needed to construct a
/// particle group.  You can safely re-use these definitions.
struct b2ParticleGroupDef
{

	b2ParticleGroupDef()
	{
		flags = 0;
		groupFlags = 0;
		position = b2Vec2_zero;
		angle = 0;
		linearVelocity = b2Vec2_zero;
		angularVelocity = 0;
		color = b2ParticleColor_zero;
		strength = 1;
		shape = NULL;
		shapes = NULL;
		shapeCount = 0;
		stride = 0;
		particleCount = 0;
		positionData = NULL;
		lifetime = 0.0f;
		userData = NULL;
		group = NULL;

#if BOX2D_EXTERNAL_LANGUAGE_API
		circleShapes = NULL;
		ownShapesArray = false;
#endif // BOX2D_EXTERNAL_LANGUAGE_API
	}

	~b2ParticleGroupDef()
	{
#if BOX2D_EXTERNAL_LANGUAGE_API
		FreeShapesMemory();
#endif // BOX2D_EXTERNAL_LANGUAGE_API
	}

	/// The particle-behavior flags (See #b2ParticleFlag).
	uint32 flags;

	/// The group-construction flags (See #b2ParticleGroupFlag).
	uint32 groupFlags;

	/// The world position of the group.
	/// Moves the group's shape a distance equal to the value of position.
	b2Vec2 position;

	/// The world angle of the group in radians.
	/// Rotates the shape by an angle equal to the value of angle.
	float32 angle;

	/// The linear velocity of the group's origin in world co-ordinates.
	b2Vec2 linearVelocity;

	/// The angular velocity of the group.
	float32 angularVelocity;

	/// The color of all particles in the group.
	b2ParticleColor color;

	/// The strength of cohesion among the particles in a group with flag
	/// b2_elasticParticle or b2_springParticle.
	float32 strength;

	/// The shape where particles will be added.
	const b2Shape* shape;

	/// A array of shapes where particles will be added.
	const b2Shape* const* shapes;

	/// The number of shapes.
	int32 shapeCount;

	/// The interval of particles in the shape.
	/// If it is 0, b2_particleStride * particleDiameter is used instead.
	float32 stride;

	/// The number of particles in addition to ones added in the shape.
	int32 particleCount;

	/// The initial positions of the particleCount particles.
	const b2Vec2* positionData;

	/// Lifetime of the particle group in seconds.  A value <= 0.0f indicates a
	/// particle group with infinite lifetime.
	float32 lifetime;

	/// Use this to store application-specific group data.
	void* userData;

	/// An existing particle group to which the particles will be added.
	b2ParticleGroup* group;

#if BOX2D_EXTERNAL_LANGUAGE_API
	/// Storage for constructed CircleShapes from an incoming vertex list
	const b2CircleShape* circleShapes;

	/// True if we create the shapes array internally.
	bool ownShapesArray;

	/// Clean up all memory associated with SetCircleShapesFromVertexList
	void FreeShapesMemory();

	/// From a vertex list created by an external language API, construct
	/// a list of circle shapes that can be used to create a b2ParticleGroup
	/// This eliminates cumbersome array-interfaces between languages.
	void SetCircleShapesFromVertexList(void* inBuf,
									   int numShapes,
									   float radius);

	/// Set position with direct floats.
	void SetPosition(float32 x, float32 y);

	/// Set color with direct ints.
	void SetColor(int32 r, int32 g, int32 b, int32 a);
#endif // BOX2D_EXTERNAL_LANGUAGE_API
};

/// A group of particles. b2ParticleGroup::CreateParticleGroup creates these.
class b2ParticleGroup
{

public:

	/// Get the next particle group from the list in b2_World.
	b2ParticleGroup* GetNext();
	const b2ParticleGroup* GetNext() const;

	/// Get the particle system that holds this particle group.
	b2ParticleSystem* GetParticleSystem();
	const b2ParticleSystem* GetParticleSystem() const;

	/// Get the number of particles.
	int32 GetParticleCount() const;

	/// Get the offset of this group in the global particle buffer
	int32 GetBufferIndex() const;

	/// Does this group contain the particle.
	bool ContainsParticle(int32 index) const;

	/// Get the logical sum of particle flags.
	uint32 GetAllParticleFlags() const;

	/// Get the construction flags for the group.
	uint32 GetGroupFlags() const;

	/// Set the construction flags for the group.
	void SetGroupFlags(uint32 flags);

	/// Get the total mass of the group: the sum of all particles in it.
	float32 GetMass() const;

	/// Get the moment of inertia for the group.
	float32 GetInertia() const;

	/// Get the center of gravity for the group.
	b2Vec2 GetCenter() const;

	/// Get the linear velocity of the group.
	b2Vec2 GetLinearVelocity() const;

	/// Get the angular velocity of the group.
	float32 GetAngularVelocity() const;

	/// Get the position of the group's origin and rotation.
	/// Used only with groups of rigid particles.
	const b2Transform& GetTransform() const;

	/// Get position of the particle group as a whole.
	/// Used only with groups of rigid particles.
	const b2Vec2& GetPosition() const;

	/// Get the rotational angle of the particle group as a whole.
	/// Used only with groups of rigid particles.
	float32 GetAngle() const;

	/// Get the world linear velocity of a world point, from the average linear
	/// and angular velocities of the particle group.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const;

	/// Get the user data pointer that was provided in the group definition.
	void* GetUserData() const;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Call b2ParticleSystem::ApplyForce for every particle in the group.
	void ApplyForce(const b2Vec2& force);

	/// Call b2ParticleSystem::ApplyLinearImpulse for every particle in the
	/// group.
	void ApplyLinearImpulse(const b2Vec2& impulse);

	/// Destroy all the particles in this group.
	/// This function is locked during callbacks.
	/// @param Whether to call the world b2DestructionListener for each
	/// particle is destroyed.
	/// @warning This function is locked during callbacks.
	void DestroyParticles(bool callDestructionListener);

	/// Destroy all particles in this group without enabling the destruction
	/// callback for destroyed particles.
	/// This function is locked during callbacks.
	/// @warning This function is locked during callbacks.
	void DestroyParticles();

private:

	friend class b2ParticleSystem;

	b2ParticleSystem* m_system;
	int32 m_firstIndex, m_lastIndex;
	uint32 m_groupFlags;
	float32 m_strength;
	b2ParticleGroup* m_prev;
	b2ParticleGroup* m_next;

	mutable int32 m_timestamp;
	mutable float32 m_mass;
	mutable float32 m_inertia;
	mutable b2Vec2 m_center;
	mutable b2Vec2 m_linearVelocity;
	mutable float32 m_angularVelocity;
	mutable b2Transform m_transform;

	void* m_userData;

	b2ParticleGroup();
	~b2ParticleGroup();
	void UpdateStatistics() const;

};

inline b2ParticleGroup* b2ParticleGroup::GetNext()
{
	return m_next;
}

inline const b2ParticleGroup* b2ParticleGroup::GetNext() const
{
	return m_next;
}

inline b2ParticleSystem* b2ParticleGroup::GetParticleSystem()
{
	return m_system;
}

inline const b2ParticleSystem* b2ParticleGroup::GetParticleSystem() const
{
	return m_system;
}

inline int32 b2ParticleGroup::GetParticleCount() const
{
	return m_lastIndex - m_firstIndex;
}

inline bool b2ParticleGroup::ContainsParticle(int32 index) const
{
	return m_firstIndex <= index && index < m_lastIndex;
}

inline b2ParticleGroup::~b2ParticleGroup()
{
}

inline int32 b2ParticleGroup::GetBufferIndex() const
{
  return m_firstIndex;
}

inline uint32 b2ParticleGroup::GetGroupFlags() const
{
	return m_groupFlags & ~b2_particleGroupInternalMask;
}

inline float32 b2ParticleGroup::GetMass() const
{
	UpdateStatistics();
	return m_mass;
}

inline float32 b2ParticleGroup::GetInertia() const
{
	UpdateStatistics();
	return m_inertia;
}

inline b2Vec2 b2ParticleGroup::GetCenter() const
{
	UpdateStatistics();
	return m_center;
}

inline b2Vec2 b2ParticleGroup::GetLinearVelocity() const
{
	UpdateStatistics();
	return m_linearVelocity;
}

inline float32 b2ParticleGroup::GetAngularVelocity() const
{
	UpdateStatistics();
	return m_angularVelocity;
}

inline const b2Transform& b2ParticleGroup::GetTransform() const
{
	return m_transform;
}

inline const b2Vec2& b2ParticleGroup::GetPosition() const
{
	return m_transform.p;
}

inline float32 b2ParticleGroup::GetAngle() const
{
	return m_transform.q.GetAngle();
}

inline b2Vec2 b2ParticleGroup::GetLinearVelocityFromWorldPoint(
												const b2Vec2& worldPoint) const
{
	UpdateStatistics();
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_center);
}

inline void* b2ParticleGroup::GetUserData() const
{
	return m_userData;
}

inline void b2ParticleGroup::SetUserData(void* data)
{
	m_userData = data;
}

inline void b2ParticleGroup::DestroyParticles()
{
	DestroyParticles(false);
}

#if BOX2D_EXTERNAL_LANGUAGE_API
inline void b2ParticleGroupDef::SetPosition(float32 x, float32 y)
{
	position.Set(x, y);
}

inline void b2ParticleGroupDef::SetColor(int32 r, int32 g, int32 b, int32 a)
{
	color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
}
#endif // BOX2D_EXTERNAL_LANGUAGE_API


#endif
