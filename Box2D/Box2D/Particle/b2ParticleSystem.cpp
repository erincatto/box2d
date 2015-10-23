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
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Particle/b2ParticleGroup.h>
#include <Box2D/Particle/b2VoronoiDiagram.h>
#include <Box2D/Particle/b2ParticleAssembly.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <algorithm>

// Define BOX2D_SIMD_TEST_VS_REFERENCE to run both SIMD and reference
// versions, and assert that the results are identical. This is useful when
// modifying one of the functions, to help verify correctness.
// #define BOX2D_SIMD_TEST_VS_REFERENCE

// For ease of debugging, remove 'inline'. Then, when an assert hits in the
// test-vs-reference functions, you can easily jump the instruction pointer
// to the top of the function to re-run the test.
#define BOX2D_SIMD_INLINE inline


static const uint32 xTruncBits = 12;
static const uint32 yTruncBits = 12;
static const uint32 tagBits = 8u * sizeof(uint32);
static const uint32 yOffset = 1u << (yTruncBits - 1u);
static const uint32 yShift = tagBits - yTruncBits;
static const uint32 xShift = tagBits - yTruncBits - xTruncBits;
static const uint32 xScale = 1u << xShift;
static const uint32 xOffset = xScale * (1u << (xTruncBits - 1u));
static const uint32 yMask = ((1u << yTruncBits) - 1u) << yShift;
static const uint32 xMask = ~yMask;
static const uint32 relativeTagRight = 1u << xShift;
static const uint32 relativeTagBottomLeft = (uint32)((1 << yShift) +
                                                    (-1 << xShift));

static const uint32 relativeTagBottomRight = (1u << yShift) + (1u << xShift);

// This functor is passed to std::remove_if in RemoveSpuriousBodyContacts
// to implement the algorithm described there.  It was hoisted out and friended
// as it would not compile with g++ 4.6.3 as a local class.  It is only used in
// that function.
class b2ParticleBodyContactRemovePredicate
{
public:
	b2ParticleBodyContactRemovePredicate(b2ParticleSystem* system,
										 int32* discarded)
		: m_system(system), m_lastIndex(-1), m_currentContacts(0),
		  m_discarded(discarded) {}

	bool operator()(const b2ParticleBodyContact& contact)
	{
		// This implements the selection criteria described in
		// RemoveSpuriousBodyContacts().
		// This functor is iterating through a list of Body contacts per
		// Particle, ordered from near to far.  For up to the maximum number of
		// contacts we allow per point per step, we verify that the contact
		// normal of the Body that genenerated the contact makes physical sense
		// by projecting a point back along that normal and seeing if it
		// intersects the fixture generating the contact.

		if (contact.index != m_lastIndex)
		{
			m_currentContacts = 0;
			m_lastIndex = contact.index;
		}

		if (m_currentContacts++ > k_maxContactsPerPoint)
		{
			++(*m_discarded);
			return true;
		}

		// Project along inverse normal (as returned in the contact) to get the
		// point to check.
		b2Vec2 n = contact.normal;
		// weight is 1-(inv(diameter) * distance)
		n *= m_system->m_particleDiameter * (1 - contact.weight);
		b2Vec2 pos = m_system->m_positionBuffer.data[contact.index] + n;

		// pos is now a point projected back along the contact normal to the
		// contact distance. If the surface makes sense for a contact, pos will
		// now lie on or in the fixture generating
		if (!contact.fixture->TestPoint(pos))
		{
			int32 childCount = contact.fixture->GetShape()->GetChildCount();
			for (int32 childIndex = 0; childIndex < childCount; childIndex++)
			{
				float32 distance;
				b2Vec2 normal;
				contact.fixture->ComputeDistance(pos, &distance, &normal,
																	childIndex);
				if (distance < b2_linearSlop)
				{
					return false;
				}
			}
			++(*m_discarded);
			return true;
		}

		return false;
	}
private:
	// Max number of contacts processed per particle, from nearest to farthest.
	// This must be at least 2 for correctness with concave shapes; 3 was
	// experimentally arrived at as looking reasonable.
	static const int32 k_maxContactsPerPoint = 3;
	const b2ParticleSystem* m_system;
	// Index of last particle processed.
	int32 m_lastIndex;
	// Number of contacts processed for the current particle.
	int32 m_currentContacts;
	// Output the number of discarded contacts.
	int32* m_discarded;
};

namespace {

// Compares the expiration time of two particle indices.
class ExpirationTimeComparator
{
public:
	// Initialize the class with a pointer to an array of particle
	// lifetimes.
	ExpirationTimeComparator(const int32* const expirationTimes) :
		m_expirationTimes(expirationTimes)
	{
	}
	// Empty destructor.
	~ExpirationTimeComparator() { }

	// Compare the lifetime of particleIndexA and particleIndexB
	// returning true if the lifetime of A is greater than B for particles
	// that will expire.  If either particle's lifetime is infinite (<= 0.0f)
	// this function return true if the lifetime of A is lesser than B.
	// When used with std::sort() this results in an array of particle
	// indicies sorted in reverse order by particle lifetime.
	// For example, the set of lifetimes
	// (1.0, 0.7, 0.3, 0.0, -1.0, -2.0)
	// would be sorted as
	// (0.0, -1.0, -2.0, 1.0, 0.7, 0.3)
	bool operator() (const int32 particleIndexA,
					 const int32 particleIndexB) const
	{
		const int32 expirationTimeA = m_expirationTimes[particleIndexA];
		const int32 expirationTimeB = m_expirationTimes[particleIndexB];
		const bool infiniteExpirationTimeA = expirationTimeA <= 0.0f;
		const bool infiniteExpirationTimeB = expirationTimeB <= 0.0f;
		return infiniteExpirationTimeA == infiniteExpirationTimeB ?
			expirationTimeA > expirationTimeB : infiniteExpirationTimeA;
	}

private:
	const int32* m_expirationTimes;
};

// *Very* lightweight pair implementation.
template<typename A, typename B>
struct LightweightPair
{
	A first;
	B second;

	// Compares the value of two FixtureParticle objects returning
	// true if left is a smaller value than right.
	static bool Compare(const LightweightPair& left,
						const LightweightPair& right)
	{
		return left.first < right.first &&
			left.second < right.second;
	}

};

// Allocator for a fixed set of items.
class FixedSetAllocator
{
public:
	// Associate a memory allocator with this object.
	FixedSetAllocator(b2StackAllocator* allocator);
	// Deallocate storage for this class.
	~FixedSetAllocator()
	{
		Clear();
	}

	// Allocate internal storage for this object returning the size.
	int32 Allocate(const int32 itemSize, const int32 count);

	// Deallocate the internal buffer if it's allocated.
	void Clear();

	// Get the number of items in the set.
	int32 GetCount() const { return m_count; }

	// Invalidate an item from the set by index.
	void Invalidate(const int32 itemIndex)
	{
		b2Assert(m_valid);
		m_valid[itemIndex] = 0;
	}

	// Get the buffer which indicates whether items are valid in the set.
	const int8* GetValidBuffer() const { return m_valid; }

protected:
	// Get the internal buffer.
	void* GetBuffer() const { return m_buffer; }
	void* GetBuffer() { return m_buffer; }

	// Reduce the number of items in the set.
	void SetCount(int32 count)
	{
		b2Assert(count <= m_count);
		m_count = count;
	}

private:
	// Set buffer.
	void* m_buffer;
	// Array of size m_count which indicates whether an item is in the
	// corresponding index of m_set (1) or the item is invalid (0).
	int8* m_valid;
	// Number of items in m_set.
	int32 m_count;
	// Allocator used to allocate / free the set.
	b2StackAllocator* m_allocator;
};

// Allocator for a fixed set of objects.
template<typename T>
class TypedFixedSetAllocator : public FixedSetAllocator
{
public:
	// Initialize members of this class.
	TypedFixedSetAllocator(b2StackAllocator* allocator) :
		FixedSetAllocator(allocator) { }

	// Allocate a set of objects, returning the new size of the set.
	int32 Allocate(const int32 numberOfObjects)
	{
		Clear();
		return FixedSetAllocator::Allocate(sizeof(T), numberOfObjects);
	}

	// Get the index of an item in the set if it's valid return an index
	// >= 0, -1 otherwise.
	int32 GetIndex(const T* item) const
	{
		if (item)
		{
			b2Assert(item >= GetBuffer() &&
					 item < GetBuffer() + GetCount());
			const int32 index =
				(int32)(((uint8*)item - (uint8*)GetBuffer()) /
						sizeof(*item));
			if (GetValidBuffer()[index])
			{
				return index;
			}
		}
		return -1;
	}

	// Get the internal buffer.
	const T* GetBuffer() const
	{
		return (const T*)FixedSetAllocator::GetBuffer();
	}
	T* GetBuffer() { return (T*)FixedSetAllocator::GetBuffer(); }
};

// Associates a fixture with a particle index.
typedef LightweightPair<b2Fixture*,int32> FixtureParticle;

// Associates a fixture with a particle index.
typedef LightweightPair<int32,int32> ParticlePair;

}  // namespace

// Set of fixture / particle indices.
class FixtureParticleSet :
	public TypedFixedSetAllocator<FixtureParticle>
{
public:
	// Initialize members of this class.
	FixtureParticleSet(b2StackAllocator* allocator) :
		TypedFixedSetAllocator<FixtureParticle>(allocator) { }


	// Initialize from a set of particle / body contacts for particles
	// that have the b2_fixtureContactListenerParticle flag set.
	void Initialize(const b2ParticleBodyContact * const bodyContacts,
					const int32 numBodyContacts,
					const uint32 * const particleFlagsBuffer);

	// Find the index of a particle / fixture pair in the set or -1
	// if it's not present.
	// NOTE: This was not written as a template function to avoid
	// exposing any dependencies via this header.
	int32 Find(const FixtureParticle& fixtureParticle) const;
};

// Set of particle / particle pairs.
class b2ParticlePairSet : public TypedFixedSetAllocator<ParticlePair>
{
public:
	// Initialize members of this class.
	b2ParticlePairSet(b2StackAllocator* allocator) :
		TypedFixedSetAllocator<ParticlePair>(allocator) { }

	// Initialize from a set of particle contacts.
	void Initialize(const b2ParticleContact * const contacts,
					const int32 numContacts,
					const uint32 * const particleFlagsBuffer);

	// Find the index of a particle pair in the set or -1
	// if it's not present.
	// NOTE: This was not written as a template function to avoid
	// exposing any dependencies via this header.
	int32 Find(const ParticlePair& pair) const;
};

static inline uint32 computeTag(float32 x, float32 y)
{
	return ((uint32)(y + yOffset) << yShift) + (uint32)(xScale * x + xOffset);
}

static inline uint32 computeRelativeTag(uint32 tag, int32 x, int32 y)
{
	return tag + (y << yShift) + (x << xShift);
}

b2ParticleSystem::InsideBoundsEnumerator::InsideBoundsEnumerator(
	uint32 lower, uint32 upper, const Proxy* first, const Proxy* last)
{
	m_xLower = lower & xMask;
	m_xUpper = upper & xMask;
	m_yLower = lower & yMask;
	m_yUpper = upper & yMask;
	m_first = first;
	m_last = last;
	b2Assert(m_first <= m_last);
}

int32 b2ParticleSystem::InsideBoundsEnumerator::GetNext()
{
	while (m_first < m_last)
	{
		uint32 xTag = m_first->tag & xMask;
#if B2_ASSERT_ENABLED
		uint32 yTag = m_first->tag & yMask;
		b2Assert(yTag >= m_yLower);
		b2Assert(yTag <= m_yUpper);
#endif
		if (xTag >= m_xLower && xTag <= m_xUpper)
		{
			return (m_first++)->index;
		}
		m_first++;
	}
	return b2_invalidParticleIndex;
}

b2ParticleSystem::b2ParticleSystem(const b2ParticleSystemDef* def,
								   b2World* world) :
	m_handleAllocator(b2_minParticleSystemBufferCapacity),
	m_stuckParticleBuffer(world->m_blockAllocator),
	m_proxyBuffer(world->m_blockAllocator),
	m_contactBuffer(world->m_blockAllocator),
	m_bodyContactBuffer(world->m_blockAllocator),
	m_pairBuffer(world->m_blockAllocator),
	m_triadBuffer(world->m_blockAllocator)
{
	b2Assert(def);
	m_paused = false;
	m_timestamp = 0;
	m_allParticleFlags = 0;
	m_needsUpdateAllParticleFlags = false;
	m_allGroupFlags = 0;
	m_needsUpdateAllGroupFlags = false;
	m_hasForce = false;
	m_iterationIndex = 0;

	SetStrictContactCheck(def->strictContactCheck);
	SetDensity(def->density);
	SetGravityScale(def->gravityScale);
	SetRadius(def->radius);
	SetMaxParticleCount(def->maxCount);

	m_count = 0;
	m_internalAllocatedCapacity = 0;
	m_forceBuffer = NULL;
	m_weightBuffer = NULL;
	m_staticPressureBuffer = NULL;
	m_accumulationBuffer = NULL;
	m_accumulation2Buffer = NULL;
	m_depthBuffer = NULL;
	m_groupBuffer = NULL;

	m_groupCount = 0;
	m_groupList = NULL;

	b2Assert(def->lifetimeGranularity > 0.0f);
	m_def = *def;

	m_world = world;

	m_stuckThreshold = 0;

	m_timeElapsed = 0;
	m_expirationTimeBufferRequiresSorting = false;

	SetDestructionByAge(m_def.destroyByAge);
}

b2ParticleSystem::~b2ParticleSystem()
{
	while (m_groupList)
	{
		DestroyParticleGroup(m_groupList);
	}

	FreeUserOverridableBuffer(&m_handleIndexBuffer);
	FreeUserOverridableBuffer(&m_flagsBuffer);
	FreeUserOverridableBuffer(&m_lastBodyContactStepBuffer);
	FreeUserOverridableBuffer(&m_bodyContactCountBuffer);
	FreeUserOverridableBuffer(&m_consecutiveContactStepsBuffer);
	FreeUserOverridableBuffer(&m_positionBuffer);
	FreeUserOverridableBuffer(&m_velocityBuffer);
	FreeUserOverridableBuffer(&m_colorBuffer);
	FreeUserOverridableBuffer(&m_userDataBuffer);
	FreeUserOverridableBuffer(&m_expirationTimeBuffer);
	FreeUserOverridableBuffer(&m_indexByExpirationTimeBuffer);
	FreeBuffer(&m_forceBuffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_weightBuffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_staticPressureBuffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_accumulationBuffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_accumulation2Buffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_depthBuffer, m_internalAllocatedCapacity);
	FreeBuffer(&m_groupBuffer, m_internalAllocatedCapacity);
}

template <typename T> void b2ParticleSystem::FreeBuffer(T** b, int capacity)
{
	if (*b == NULL)
		return;

	m_world->m_blockAllocator.Free(*b, sizeof(**b) * capacity);
	*b = NULL;
}

// Free buffer, if it was allocated with b2World's block allocator
template <typename T> void b2ParticleSystem::FreeUserOverridableBuffer(
	UserOverridableBuffer<T>* b)
{
	if (b->userSuppliedCapacity == 0)
	{
		FreeBuffer(&b->data, m_internalAllocatedCapacity);
	}
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	T* oldBuffer, int32 oldCapacity, int32 newCapacity)
{
	b2Assert(newCapacity > oldCapacity);
	T* newBuffer = (T*) m_world->m_blockAllocator.Allocate(
		sizeof(T) * newCapacity);
	if (oldBuffer)
	{
		memcpy(newBuffer, oldBuffer, sizeof(T) * oldCapacity);
		m_world->m_blockAllocator.Free(oldBuffer, sizeof(T) * oldCapacity);
	}
	return newBuffer;
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	T* buffer, int32 userSuppliedCapacity, int32 oldCapacity,
	int32 newCapacity, bool deferred)
{
	b2Assert(newCapacity > oldCapacity);
	// A 'deferred' buffer is reallocated only if it is not NULL.
	// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must
	// be kept.
	b2Assert(!userSuppliedCapacity || newCapacity <= userSuppliedCapacity);
	if ((!deferred || buffer) && !userSuppliedCapacity)
	{
		buffer = ReallocateBuffer(buffer, oldCapacity, newCapacity);
	}
	return buffer;
}

// Reallocate a buffer
template <typename T> T* b2ParticleSystem::ReallocateBuffer(
	UserOverridableBuffer<T>* buffer, int32 oldCapacity, int32 newCapacity,
	bool deferred)
{
	b2Assert(newCapacity > oldCapacity);
	return ReallocateBuffer(buffer->data, buffer->userSuppliedCapacity,
							oldCapacity, newCapacity, deferred);
}

/// Reallocate the handle / index map and schedule the allocation of a new
/// pool for handle allocation.
void b2ParticleSystem::ReallocateHandleBuffers(int32 newCapacity)
{
	b2Assert(newCapacity > m_internalAllocatedCapacity);
	// Reallocate a new handle / index map buffer, copying old handle pointers
	// is fine since they're kept around.
	m_handleIndexBuffer.data = ReallocateBuffer(
		&m_handleIndexBuffer, m_internalAllocatedCapacity, newCapacity,
		true);
	// Set the size of the next handle allocation.
	m_handleAllocator.SetItemsPerSlab(newCapacity -
									  m_internalAllocatedCapacity);
}

template <typename T> T* b2ParticleSystem::RequestBuffer(T* buffer)
{
	if (!buffer)
	{
		if (m_internalAllocatedCapacity == 0)
		{
			ReallocateInternalAllocatedBuffers(
				b2_minParticleSystemBufferCapacity);
		}
		buffer = (T*) (m_world->m_blockAllocator.Allocate(
						   sizeof(T) * m_internalAllocatedCapacity));
		b2Assert(buffer);
		memset(buffer, 0, sizeof(T) * m_internalAllocatedCapacity);
	}
	return buffer;
}

b2ParticleColor* b2ParticleSystem::GetColorBuffer()
{
	m_colorBuffer.data = RequestBuffer(m_colorBuffer.data);
	return m_colorBuffer.data;
}

void** b2ParticleSystem::GetUserDataBuffer()
{
	m_userDataBuffer.data = RequestBuffer(m_userDataBuffer.data);
	return m_userDataBuffer.data;
}

static int32 LimitCapacity(int32 capacity, int32 maxCount)
{
	return maxCount && capacity > maxCount ? maxCount : capacity;
}

void b2ParticleSystem::ReallocateInternalAllocatedBuffers(int32 capacity)
{
	// Don't increase capacity beyond the smallest user-supplied buffer size.
	capacity = LimitCapacity(capacity, m_def.maxCount);
	capacity = LimitCapacity(capacity, m_flagsBuffer.userSuppliedCapacity);
	capacity = LimitCapacity(capacity, m_positionBuffer.userSuppliedCapacity);
	capacity = LimitCapacity(capacity, m_velocityBuffer.userSuppliedCapacity);
	capacity = LimitCapacity(capacity, m_colorBuffer.userSuppliedCapacity);
	capacity = LimitCapacity(capacity, m_userDataBuffer.userSuppliedCapacity);
	if (m_internalAllocatedCapacity < capacity)
	{
		ReallocateHandleBuffers(capacity);
		m_flagsBuffer.data = ReallocateBuffer(
			&m_flagsBuffer, m_internalAllocatedCapacity, capacity, false);

		// Conditionally defer these as they are optional if the feature is
		// not enabled.
		const bool stuck = m_stuckThreshold > 0;
		m_lastBodyContactStepBuffer.data = ReallocateBuffer(
			&m_lastBodyContactStepBuffer, m_internalAllocatedCapacity,
			capacity, stuck);
		m_bodyContactCountBuffer.data = ReallocateBuffer(
			&m_bodyContactCountBuffer, m_internalAllocatedCapacity, capacity,
			stuck);
		m_consecutiveContactStepsBuffer.data = ReallocateBuffer(
			&m_consecutiveContactStepsBuffer, m_internalAllocatedCapacity,
			capacity, stuck);
		m_positionBuffer.data = ReallocateBuffer(
			&m_positionBuffer, m_internalAllocatedCapacity, capacity, false);
		m_velocityBuffer.data = ReallocateBuffer(
			&m_velocityBuffer, m_internalAllocatedCapacity, capacity, false);
		m_forceBuffer = ReallocateBuffer(
			m_forceBuffer, 0, m_internalAllocatedCapacity, capacity, false);
		m_weightBuffer = ReallocateBuffer(
			m_weightBuffer, 0, m_internalAllocatedCapacity, capacity, false);
		m_staticPressureBuffer = ReallocateBuffer(
			m_staticPressureBuffer, 0, m_internalAllocatedCapacity, capacity,
			true);
		m_accumulationBuffer = ReallocateBuffer(
			m_accumulationBuffer, 0, m_internalAllocatedCapacity, capacity,
			false);
		m_accumulation2Buffer = ReallocateBuffer(
			m_accumulation2Buffer, 0, m_internalAllocatedCapacity, capacity,
			true);
		m_depthBuffer = ReallocateBuffer(
			m_depthBuffer, 0, m_internalAllocatedCapacity, capacity, true);
		m_colorBuffer.data = ReallocateBuffer(
			&m_colorBuffer, m_internalAllocatedCapacity, capacity, true);
		m_groupBuffer = ReallocateBuffer(
			m_groupBuffer, 0, m_internalAllocatedCapacity, capacity, false);
		m_userDataBuffer.data = ReallocateBuffer(
			&m_userDataBuffer, m_internalAllocatedCapacity, capacity, true);
		m_expirationTimeBuffer.data = ReallocateBuffer(
			&m_expirationTimeBuffer, m_internalAllocatedCapacity, capacity,
			true);
		m_indexByExpirationTimeBuffer.data = ReallocateBuffer(
			&m_indexByExpirationTimeBuffer, m_internalAllocatedCapacity,
			capacity, true);
		m_internalAllocatedCapacity = capacity;
	}
}

int32 b2ParticleSystem::CreateParticle(const b2ParticleDef& def)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

	if (m_count >= m_internalAllocatedCapacity)
	{
		// Double the particle capacity.
		int32 capacity =
			m_count ? 2 * m_count : b2_minParticleSystemBufferCapacity;
		ReallocateInternalAllocatedBuffers(capacity);
	}
	if (m_count >= m_internalAllocatedCapacity)
	{
		// If the oldest particle should be destroyed...
		if (m_def.destroyByAge)
		{
			DestroyOldestParticle(0, false);
			// Need to destroy this particle *now* so that it's possible to
			// create a new particle.
			SolveZombie();
		}
		else
		{
			return b2_invalidParticleIndex;
		}
	}
	int32 index = m_count++;
	m_flagsBuffer.data[index] = 0;
	if (m_lastBodyContactStepBuffer.data)
	{
		m_lastBodyContactStepBuffer.data[index] = 0;
	}
	if (m_bodyContactCountBuffer.data)
	{
		m_bodyContactCountBuffer.data[index] = 0;
	}
	if (m_consecutiveContactStepsBuffer.data)
	{
		m_consecutiveContactStepsBuffer.data[index] = 0;
	}
	m_positionBuffer.data[index] = def.position;
	m_velocityBuffer.data[index] = def.velocity;
	m_weightBuffer[index] = 0;
	m_forceBuffer[index] = b2Vec2_zero;
	if (m_staticPressureBuffer)
	{
		m_staticPressureBuffer[index] = 0;
	}
	if (m_depthBuffer)
	{
		m_depthBuffer[index] = 0;
	}
	if (m_colorBuffer.data || !def.color.IsZero())
	{
		m_colorBuffer.data = RequestBuffer(m_colorBuffer.data);
		m_colorBuffer.data[index] = def.color;
	}
	if (m_userDataBuffer.data || def.userData)
	{
		m_userDataBuffer.data= RequestBuffer(m_userDataBuffer.data);
		m_userDataBuffer.data[index] = def.userData;
	}
	if (m_handleIndexBuffer.data)
	{
		m_handleIndexBuffer.data[index] = NULL;
	}
	Proxy& proxy = m_proxyBuffer.Append();

	// If particle lifetimes are enabled or the lifetime is set in the particle
	// definition, initialize the lifetime.
	const bool finiteLifetime = def.lifetime > 0;
	if (m_expirationTimeBuffer.data || finiteLifetime)
	{
		SetParticleLifetime(index, finiteLifetime ? def.lifetime :
								ExpirationTimeToLifetime(
									-GetQuantizedTimeElapsed()));
		// Add a reference to the newly added particle to the end of the
		// queue.
		m_indexByExpirationTimeBuffer.data[index] = index;
	}

	proxy.index = index;
	b2ParticleGroup* group = def.group;
	m_groupBuffer[index] = group;
	if (group)
	{
		if (group->m_firstIndex < group->m_lastIndex)
		{
			// Move particles in the group just before the new particle.
			RotateBuffer(group->m_firstIndex, group->m_lastIndex, index);
			b2Assert(group->m_lastIndex == index);
			// Update the index range of the group to contain the new particle.
			group->m_lastIndex = index + 1;
		}
		else
		{
			// If the group is empty, reset the index range to contain only the
			// new particle.
			group->m_firstIndex = index;
			group->m_lastIndex = index + 1;
		}
	}
	SetParticleFlags(index, def.flags);
	return index;
}

/// Retrieve a handle to the particle at the specified index.
const b2ParticleHandle* b2ParticleSystem::GetParticleHandleFromIndex(
	const int32 index)
{
	b2Assert(index >= 0 && index < GetParticleCount() &&
			 index != b2_invalidParticleIndex);
	m_handleIndexBuffer.data = RequestBuffer(m_handleIndexBuffer.data);
	b2ParticleHandle* handle = m_handleIndexBuffer.data[index];
	if (handle)
	{
		return handle;
	}
	// Create a handle.
	handle = m_handleAllocator.Allocate();
	b2Assert(handle);
	handle->SetIndex(index);
	m_handleIndexBuffer.data[index] = handle;
	return handle;
}


void b2ParticleSystem::DestroyParticle(
	int32 index, bool callDestructionListener)
{
	uint32 flags = b2_zombieParticle;
	if (callDestructionListener)
	{
		flags |= b2_destructionListenerParticle;
	}
	SetParticleFlags(index, m_flagsBuffer.data[index] | flags);
}

void b2ParticleSystem::DestroyOldestParticle(
	const int32 index, const bool callDestructionListener)
{
	const int32 particleCount = GetParticleCount();
	b2Assert(index >= 0 && index < particleCount);
	// Make sure particle lifetime tracking is enabled.
	b2Assert(m_indexByExpirationTimeBuffer.data);
	// Destroy the oldest particle (preferring to destroy finite
	// lifetime particles first) to free a slot in the buffer.
	const int32 oldestFiniteLifetimeParticle =
		m_indexByExpirationTimeBuffer.data[particleCount - (index + 1)];
	const int32 oldestInfiniteLifetimeParticle =
		m_indexByExpirationTimeBuffer.data[index];
	DestroyParticle(
		m_expirationTimeBuffer.data[oldestFiniteLifetimeParticle] > 0.0f ?
			oldestFiniteLifetimeParticle : oldestInfiniteLifetimeParticle,
		callDestructionListener);
}

int32 b2ParticleSystem::DestroyParticlesInShape(
	const b2Shape& shape, const b2Transform& xf,
	bool callDestructionListener)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

	class DestroyParticlesInShapeCallback : public b2QueryCallback
	{
	public:
		DestroyParticlesInShapeCallback(
			b2ParticleSystem* system, const b2Shape& shape,
			const b2Transform& xf, bool callDestructionListener)
		{
			m_system = system;
			m_shape = &shape;
			m_xf = xf;
			m_callDestructionListener = callDestructionListener;
			m_destroyed = 0;
		}

		bool ReportFixture(b2Fixture* fixture)
		{
			B2_NOT_USED(fixture);
			return false;
		}

		bool ReportParticle(const b2ParticleSystem* particleSystem, int32 index)
		{
			if (particleSystem != m_system)
				return false;

			b2Assert(index >=0 && index < m_system->m_count);
			if (m_shape->TestPoint(m_xf,
								   m_system->m_positionBuffer.data[index]))
			{
				m_system->DestroyParticle(index, m_callDestructionListener);
				m_destroyed++;
			}
			return true;
		}

		int32 Destroyed() { return m_destroyed; }

	private:
		b2ParticleSystem* m_system;
		const b2Shape* m_shape;
		b2Transform m_xf;
		bool m_callDestructionListener;
		int32 m_destroyed;
	} callback(this, shape, xf, callDestructionListener);
	b2AABB aabb;
	shape.ComputeAABB(&aabb, xf, 0);
	m_world->QueryAABB(&callback, aabb);
	return callback.Destroyed();
}

int32 b2ParticleSystem::CreateParticleForGroup(
	const b2ParticleGroupDef& groupDef, const b2Transform& xf, const b2Vec2& p)
{
	b2ParticleDef particleDef;
	particleDef.flags = groupDef.flags;
	particleDef.position = b2Mul(xf, p);
	particleDef.velocity =
		groupDef.linearVelocity +
		b2Cross(groupDef.angularVelocity,
				particleDef.position - groupDef.position);
	particleDef.color = groupDef.color;
	particleDef.lifetime = groupDef.lifetime;
	particleDef.userData = groupDef.userData;
	return CreateParticle(particleDef);
}

void b2ParticleSystem::CreateParticlesStrokeShapeForGroup(
	const b2Shape *shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride;
	if (stride == 0)
	{
		stride = GetParticleStride();
	}
	float32 positionOnEdge = 0;
	int32 childCount = shape->GetChildCount();
	for (int32 childIndex = 0; childIndex < childCount; childIndex++)
	{
		b2EdgeShape edge;
		if (shape->GetType() == b2Shape::e_edge)
		{
			edge = *(b2EdgeShape*) shape;
		}
		else
		{
			b2Assert(shape->GetType() == b2Shape::e_chain);
			((b2ChainShape*) shape)->GetChildEdge(&edge, childIndex);
		}
		b2Vec2 d = edge.m_vertex2 - edge.m_vertex1;
		float32 edgeLength = d.Length();
		while (positionOnEdge < edgeLength)
		{
			b2Vec2 p = edge.m_vertex1 + positionOnEdge / edgeLength * d;
			CreateParticleForGroup(groupDef, xf, p);
			positionOnEdge += stride;
		}
		positionOnEdge -= edgeLength;
	}
}

void b2ParticleSystem::CreateParticlesFillShapeForGroup(
	const b2Shape *shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	float32 stride = groupDef.stride;
	if (stride == 0)
	{
		stride = GetParticleStride();
	}
	b2Transform identity;
	identity.SetIdentity();
	b2AABB aabb;
	b2Assert(shape->GetChildCount() == 1);
	shape->ComputeAABB(&aabb, identity, 0);
	for (float32 y = floorf(aabb.lowerBound.y / stride) * stride;
		y < aabb.upperBound.y; y += stride)
	{
		for (float32 x = floorf(aabb.lowerBound.x / stride) * stride;
			x < aabb.upperBound.x; x += stride)
		{
			b2Vec2 p(x, y);
			if (shape->TestPoint(identity, p))
			{
				CreateParticleForGroup(groupDef, xf, p);
			}
		}
	}
}

void b2ParticleSystem::CreateParticlesWithShapeForGroup(
	const b2Shape* shape,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	switch (shape->GetType()) {
	case b2Shape::e_edge:
	case b2Shape::e_chain:
		CreateParticlesStrokeShapeForGroup(shape, groupDef, xf);
		break;
	case b2Shape::e_polygon:
	case b2Shape::e_circle:
		CreateParticlesFillShapeForGroup(shape, groupDef, xf);
		break;
	default:
		b2Assert(false);
		break;
	}
}

void b2ParticleSystem::CreateParticlesWithShapesForGroup(
	const b2Shape* const* shapes, int32 shapeCount,
	const b2ParticleGroupDef& groupDef, const b2Transform& xf)
{
	class CompositeShape : public b2Shape
	{
	public:
		CompositeShape(const b2Shape* const* shapes, int32 shapeCount)
		{
			m_shapes = shapes;
			m_shapeCount = shapeCount;
		}
		b2Shape* Clone(b2BlockAllocator* allocator) const
		{
			b2Assert(false);
			B2_NOT_USED(allocator);
			return NULL;
		}
		int32 GetChildCount() const
		{
			return 1;
		}
		bool TestPoint(const b2Transform& xf, const b2Vec2& p) const
		{
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				if (m_shapes[i]->TestPoint(xf, p))
				{
					return true;
				}
			}
			return false;
		}
		void ComputeDistance(const b2Transform& xf, const b2Vec2& p,
					float32* distance, b2Vec2* normal, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(xf);
			B2_NOT_USED(p);
			B2_NOT_USED(distance);
			B2_NOT_USED(normal);
			B2_NOT_USED(childIndex);
		}
		bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
						const b2Transform& transform, int32 childIndex) const
		{
			b2Assert(false);
			B2_NOT_USED(output);
			B2_NOT_USED(input);
			B2_NOT_USED(transform);
			B2_NOT_USED(childIndex);
			return false;
		}
		void ComputeAABB(
				b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
		{
			B2_NOT_USED(childIndex);
			aabb->lowerBound.x = +FLT_MAX;
			aabb->lowerBound.y = +FLT_MAX;
			aabb->upperBound.x = -FLT_MAX;
			aabb->upperBound.y = -FLT_MAX;
			b2Assert(childIndex == 0);
			for (int32 i = 0; i < m_shapeCount; i++)
			{
				int32 childCount = m_shapes[i]->GetChildCount();
				for (int32 j = 0; j < childCount; j++)
				{
					b2AABB subaabb;
					m_shapes[i]->ComputeAABB(&subaabb, xf, j);
					aabb->Combine(subaabb);
				}
			}
		}
		void ComputeMass(b2MassData* massData, float32 density) const
		{
			b2Assert(false);
			B2_NOT_USED(massData);
			B2_NOT_USED(density);
		}
	private:
		const b2Shape* const* m_shapes;
		int32 m_shapeCount;
	} compositeShape(shapes, shapeCount);
	CreateParticlesFillShapeForGroup(&compositeShape, groupDef, xf);
}

b2ParticleGroup* b2ParticleSystem::CreateParticleGroup(
	const b2ParticleGroupDef& groupDef)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return 0;
	}

	b2Transform transform;
	transform.Set(groupDef.position, groupDef.angle);
	int32 firstIndex = m_count;
	if (groupDef.shape)
	{
		CreateParticlesWithShapeForGroup(groupDef.shape, groupDef, transform);
	}
	if (groupDef.shapes)
	{
		CreateParticlesWithShapesForGroup(
					groupDef.shapes, groupDef.shapeCount, groupDef, transform);
	}
	if (groupDef.particleCount)
	{
		b2Assert(groupDef.positionData);
		for (int32 i = 0; i < groupDef.particleCount; i++)
		{
			b2Vec2 p = groupDef.positionData[i];
			CreateParticleForGroup(groupDef, transform, p);
		}
	}
	int32 lastIndex = m_count;

	void* mem = m_world->m_blockAllocator.Allocate(sizeof(b2ParticleGroup));
	b2ParticleGroup* group = new (mem) b2ParticleGroup();
	group->m_system = this;
	group->m_firstIndex = firstIndex;
	group->m_lastIndex = lastIndex;
	group->m_strength = groupDef.strength;
	group->m_userData = groupDef.userData;
	group->m_transform = transform;
	group->m_prev = NULL;
	group->m_next = m_groupList;
	if (m_groupList)
	{
		m_groupList->m_prev = group;
	}
	m_groupList = group;
	++m_groupCount;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		m_groupBuffer[i] = group;
	}
	SetGroupFlags(group, groupDef.groupFlags);

	// Create pairs and triads between particles in the group.
	ConnectionFilter filter;
	UpdateContacts(true);
	UpdatePairsAndTriads(firstIndex, lastIndex, filter);

	if (groupDef.group)
	{
		JoinParticleGroups(groupDef.group, group);
		group = groupDef.group;
	}

	return group;
}

void b2ParticleSystem::JoinParticleGroups(b2ParticleGroup* groupA,
										  b2ParticleGroup* groupB)
{
	b2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked())
	{
		return;
	}

	b2Assert(groupA != groupB);
	RotateBuffer(groupB->m_firstIndex, groupB->m_lastIndex, m_count);
	b2Assert(groupB->m_lastIndex == m_count);
	RotateBuffer(groupA->m_firstIndex, groupA->m_lastIndex,
				 groupB->m_firstIndex);
	b2Assert(groupA->m_lastIndex == groupB->m_firstIndex);

	// Create pairs and triads connecting groupA and groupB.
	class JoinParticleGroupsFilter : public ConnectionFilter
	{
		bool ShouldCreatePair(int32 a, int32 b) const
		{
			return
				(a < m_threshold && m_threshold <= b) ||
				(b < m_threshold && m_threshold <= a);
		}
		bool ShouldCreateTriad(int32 a, int32 b, int32 c) const
		{
			return
				(a < m_threshold || b < m_threshold || c < m_threshold) &&
				(m_threshold <= a || m_threshold <= b || m_threshold <= c);
		}
		int32 m_threshold;
	public:
		JoinParticleGroupsFilter(int32 threshold)
		{
			m_threshold = threshold;
		}
	} filter(groupB->m_firstIndex);
	UpdateContacts(true);
	UpdatePairsAndTriads(groupA->m_firstIndex, groupB->m_lastIndex, filter);

	for (int32 i = groupB->m_firstIndex; i < groupB->m_lastIndex; i++)
	{
		m_groupBuffer[i] = groupA;
	}
	uint32 groupFlags = groupA->m_groupFlags | groupB->m_groupFlags;
	SetGroupFlags(groupA, groupFlags);
	groupA->m_lastIndex = groupB->m_lastIndex;
	groupB->m_firstIndex = groupB->m_lastIndex;
	DestroyParticleGroup(groupB);
}

void b2ParticleSystem::SplitParticleGroup(b2ParticleGroup* group)
{
	UpdateContacts(true);
	int32 particleCount = group->GetParticleCount();
	// We create several linked lists. Each list represents a set of connected
	// particles.
	ParticleListNode* nodeBuffer =
		(ParticleListNode*) m_world->m_stackAllocator.Allocate(
									sizeof(ParticleListNode) * particleCount);
	InitializeParticleLists(group, nodeBuffer);
	MergeParticleListsInContact(group, nodeBuffer);
	ParticleListNode* survivingList =
									FindLongestParticleList(group, nodeBuffer);
	MergeZombieParticleListNodes(group, nodeBuffer, survivingList);
	CreateParticleGroupsFromParticleList(group, nodeBuffer, survivingList);
	UpdatePairsAndTriadsWithParticleList(group, nodeBuffer);
	m_world->m_stackAllocator.Free(nodeBuffer);
}

void b2ParticleSystem::InitializeParticleLists(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group->GetBufferIndex();
	int32 particleCount = group->GetParticleCount();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		node->list = node;
		node->next = NULL;
		node->count = 1;
		node->index = i + bufferIndex;
	}
}

void b2ParticleSystem::MergeParticleListsInContact(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer) const
{
	int32 bufferIndex = group->GetBufferIndex();
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		if (!group->ContainsParticle(a) || !group->ContainsParticle(b)) {
			continue;
		}
		ParticleListNode* listA = nodeBuffer[a - bufferIndex].list;
		ParticleListNode* listB = nodeBuffer[b - bufferIndex].list;
		if (listA == listB) {
			continue;
		}
		// To minimize the cost of insertion, make sure listA is longer than
		// listB.
		if (listA->count < listB->count)
		{
			b2Swap(listA, listB);
		}
		b2Assert(listA->count >= listB->count);
		MergeParticleLists(listA, listB);
	}
}

void b2ParticleSystem::MergeParticleLists(
	ParticleListNode* listA, ParticleListNode* listB)
{
	// Insert listB between index 0 and 1 of listA
	// Example:
	//     listA => a1 => a2 => a3 => NULL
	//     listB => b1 => b2 => NULL
	// to
	//     listA => listB => b1 => b2 => a1 => a2 => a3 => NULL
	b2Assert(listA != listB);
	for (ParticleListNode* b = listB;;)
	{
		b->list = listA;
		ParticleListNode* nextB = b->next;
		if (nextB)
		{
			b = nextB;
		}
		else
		{
			b->next = listA->next;
			break;
		}
	}
	listA->next = listB;
	listA->count += listB->count;
	listB->count = 0;
}

b2ParticleSystem::ParticleListNode* b2ParticleSystem::FindLongestParticleList(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer)
{
	int32 particleCount = group->GetParticleCount();
	ParticleListNode* result = nodeBuffer;
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		if (result->count < node->count)
		{
			result = node;
		}
	}
	return result;
}

void b2ParticleSystem::MergeZombieParticleListNodes(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
	ParticleListNode* survivingList) const
{
	int32 particleCount = group->GetParticleCount();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* node = &nodeBuffer[i];
		if (node != survivingList &&
			(m_flagsBuffer.data[node->index] & b2_zombieParticle))
		{
			MergeParticleListAndNode(survivingList, node);
		}
	}
}

void b2ParticleSystem::MergeParticleListAndNode(
	ParticleListNode* list, ParticleListNode* node)
{
	// Insert node between index 0 and 1 of list
	// Example:
	//     list => a1 => a2 => a3 => NULL
	//     node => NULL
	// to
	//     list => node => a1 => a2 => a3 => NULL
	b2Assert(node != list);
	b2Assert(node->list == node);
	b2Assert(node->count == 1);
	node->list = list;
	node->next = list->next;
	list->next = node;
	list->count++;
	node->count = 0;
}

void b2ParticleSystem::CreateParticleGroupsFromParticleList(
	const b2ParticleGroup* group, ParticleListNode* nodeBuffer,
	const ParticleListNode* survivingList)
{
	int32 particleCount = group->GetParticleCount();
	b2ParticleGroupDef def;
	def.groupFlags = group->GetGroupFlags();
	def.userData = group->GetUserData();
	for (int32 i = 0; i < particleCount; i++)
	{
		ParticleListNode* list = &nodeBuffer[i];
		if (!list->count || list == survivingList)
		{
			continue;
		}
		b2Assert(list->list == list);
		b2ParticleGroup* newGroup = CreateParticleGroup(def);
		for (ParticleListNode* node = list; node; node = node->next)
		{
			int32 oldIndex = node->index;
			uint32& flags = m_flagsBuffer.data[oldIndex];
			b2Assert(!(flags & b2_zombieParticle));
			int32 newIndex = CloneParticle(oldIndex, newGroup);
			flags |= b2_zombieParticle;
			node->index = newIndex;
		}
	}
}

void b2ParticleSystem::UpdatePairsAndTriadsWithParticleList(
	const b2ParticleGroup* group, const ParticleListNode* nodeBuffer)
{
	int32 bufferIndex = group->GetBufferIndex();
	// Update indices in pairs and triads. If an index belongs to the group,
	// replace it with the corresponding value in nodeBuffer.
	// Note that nodeBuffer is allocated only for the group and the index should
	// be shifted by bufferIndex.
	for (int32 k = 0; k < m_pairBuffer.GetCount(); k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		int32 a = pair.indexA;
		int32 b = pair.indexB;
		if (group->ContainsParticle(a))
		{
			pair.indexA = nodeBuffer[a - bufferIndex].index;
		}
		if (group->ContainsParticle(b))
		{
			pair.indexB = nodeBuffer[b - bufferIndex].index;
		}
	}
	for (int32 k = 0; k < m_triadBuffer.GetCount(); k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		int32 a = triad.indexA;
		int32 b = triad.indexB;
		int32 c = triad.indexC;
		if (group->ContainsParticle(a))
		{
			triad.indexA = nodeBuffer[a - bufferIndex].index;
		}
		if (group->ContainsParticle(b))
		{
			triad.indexB = nodeBuffer[b - bufferIndex].index;
		}
		if (group->ContainsParticle(c))
		{
			triad.indexC = nodeBuffer[c - bufferIndex].index;
		}
	}
}

int32 b2ParticleSystem::CloneParticle(int32 oldIndex, b2ParticleGroup* group)
{
	b2ParticleDef def;
	def.flags = m_flagsBuffer.data[oldIndex];
	def.position = m_positionBuffer.data[oldIndex];
	def.velocity = m_velocityBuffer.data[oldIndex];
	if (m_colorBuffer.data)
	{
		def.color = m_colorBuffer.data[oldIndex];
	}
	if (m_userDataBuffer.data)
	{
		def.userData = m_userDataBuffer.data[oldIndex];
	}
	def.group = group;
	int32 newIndex = CreateParticle(def);
	if (m_handleIndexBuffer.data)
	{
		b2ParticleHandle* handle = m_handleIndexBuffer.data[oldIndex];
		if (handle) handle->SetIndex(newIndex);
		m_handleIndexBuffer.data[newIndex] = handle;
		m_handleIndexBuffer.data[oldIndex] = NULL;
	}
	if (m_lastBodyContactStepBuffer.data)
	{
		m_lastBodyContactStepBuffer.data[newIndex] =
			m_lastBodyContactStepBuffer.data[oldIndex];
	}
	if (m_bodyContactCountBuffer.data)
	{
		m_bodyContactCountBuffer.data[newIndex] =
			m_bodyContactCountBuffer.data[oldIndex];
	}
	if (m_consecutiveContactStepsBuffer.data)
	{
		m_consecutiveContactStepsBuffer.data[newIndex] =
			m_consecutiveContactStepsBuffer.data[oldIndex];
	}
	if (m_hasForce)
	{
		m_forceBuffer[newIndex] = m_forceBuffer[oldIndex];
	}
	if (m_staticPressureBuffer)
	{
		m_staticPressureBuffer[newIndex] = m_staticPressureBuffer[oldIndex];
	}
	if (m_depthBuffer)
	{
		m_depthBuffer[newIndex] = m_depthBuffer[oldIndex];
	}
	if (m_expirationTimeBuffer.data)
	{
		m_expirationTimeBuffer.data[newIndex] =
			m_expirationTimeBuffer.data[oldIndex];
	}
	return newIndex;
}

void b2ParticleSystem::UpdatePairsAndTriadsWithReactiveParticles()
{
	class ReactiveFilter : public ConnectionFilter
	{
		bool IsNecessary(int32 index) const
		{
			return (m_flagsBuffer[index] & b2_reactiveParticle) != 0;
		}
		const uint32* m_flagsBuffer;
	public:
		ReactiveFilter(uint32* flagsBuffer)
		{
			m_flagsBuffer = flagsBuffer;
		}
	} filter(m_flagsBuffer.data);
	UpdatePairsAndTriads(0, m_count, filter);

	for (int32 i = 0; i < m_count; i++)
	{
		m_flagsBuffer.data[i] &= ~b2_reactiveParticle;
	}
	m_allParticleFlags &= ~b2_reactiveParticle;
}

static bool ParticleCanBeConnected(
	uint32 flags, b2ParticleGroup* group)
{
	return
		(flags & (b2_wallParticle | b2_springParticle | b2_elasticParticle)) ||
		(group && group->GetGroupFlags() & b2_rigidParticleGroup);
}

void b2ParticleSystem::UpdatePairsAndTriads(
	int32 firstIndex, int32 lastIndex, const ConnectionFilter& filter)
{
	// Create pairs or triads.
	// All particles in each pair/triad should satisfy the following:
	// * firstIndex <= index < lastIndex
	// * don't have b2_zombieParticle
	// * ParticleCanBeConnected returns true
	// * ShouldCreatePair/ShouldCreateTriad returns true
	// Any particles in each pair/triad should satisfy the following:
	// * filter.IsNeeded returns true
	// * have one of k_pairFlags/k_triadsFlags
	b2Assert(firstIndex <= lastIndex);
	uint32 particleFlags = 0;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		particleFlags |= m_flagsBuffer.data[i];
	}
	if (particleFlags & k_pairFlags)
	{
		for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
		{
			const b2ParticleContact& contact = m_contactBuffer[k];
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			uint32 af = m_flagsBuffer.data[a];
			uint32 bf = m_flagsBuffer.data[b];
			b2ParticleGroup* groupA = m_groupBuffer[a];
			b2ParticleGroup* groupB = m_groupBuffer[b];
			if (a >= firstIndex && a < lastIndex &&
				b >= firstIndex && b < lastIndex &&
				!((af | bf) & b2_zombieParticle) &&
				((af | bf) & k_pairFlags) &&
				(filter.IsNecessary(a) || filter.IsNecessary(b)) &&
				ParticleCanBeConnected(af, groupA) &&
				ParticleCanBeConnected(bf, groupB) &&
				filter.ShouldCreatePair(a, b))
			{
				b2ParticlePair& pair = m_pairBuffer.Append();
				pair.indexA = a;
				pair.indexB = b;
				pair.flags = contact.GetFlags();
				pair.strength = b2Min(
					groupA ? groupA->m_strength : 1,
					groupB ? groupB->m_strength : 1);
				pair.distance = b2Distance(m_positionBuffer.data[a],
										   m_positionBuffer.data[b]);
			}
		}
		std::stable_sort(
			m_pairBuffer.Begin(), m_pairBuffer.End(), ComparePairIndices);
		m_pairBuffer.Unique(MatchPairIndices);
	}
	if (particleFlags & k_triadFlags)
	{
		b2VoronoiDiagram diagram(
			&m_world->m_stackAllocator, lastIndex - firstIndex);
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			uint32 flags = m_flagsBuffer.data[i];
			b2ParticleGroup* group = m_groupBuffer[i];
			if (!(flags & b2_zombieParticle) &&
				ParticleCanBeConnected(flags, group))
			{
				diagram.AddGenerator(
					m_positionBuffer.data[i], i, filter.IsNecessary(i));
			}
		}
		float32 stride = GetParticleStride();
		diagram.Generate(stride / 2, stride * 2);
		class UpdateTriadsCallback : public b2VoronoiDiagram::NodeCallback
		{
			void operator()(int32 a, int32 b, int32 c)
			{
				uint32 af = m_system->m_flagsBuffer.data[a];
				uint32 bf = m_system->m_flagsBuffer.data[b];
				uint32 cf = m_system->m_flagsBuffer.data[c];
				if (((af | bf | cf) & k_triadFlags) &&
					m_filter->ShouldCreateTriad(a, b, c))
				{
					const b2Vec2& pa = m_system->m_positionBuffer.data[a];
					const b2Vec2& pb = m_system->m_positionBuffer.data[b];
					const b2Vec2& pc = m_system->m_positionBuffer.data[c];
					b2Vec2 dab = pa - pb;
					b2Vec2 dbc = pb - pc;
					b2Vec2 dca = pc - pa;
					float32 maxDistanceSquared = b2_maxTriadDistanceSquared *
												 m_system->m_squaredDiameter;
					if (b2Dot(dab, dab) > maxDistanceSquared ||
						b2Dot(dbc, dbc) > maxDistanceSquared ||
						b2Dot(dca, dca) > maxDistanceSquared)
					{
						return;
					}
					b2ParticleGroup* groupA = m_system->m_groupBuffer[a];
					b2ParticleGroup* groupB = m_system->m_groupBuffer[b];
					b2ParticleGroup* groupC = m_system->m_groupBuffer[c];
					b2ParticleTriad& triad = m_system->m_triadBuffer.Append();
					triad.indexA = a;
					triad.indexB = b;
					triad.indexC = c;
					triad.flags = af | bf | cf;
					triad.strength = b2Min(b2Min(
						groupA ? groupA->m_strength : 1,
						groupB ? groupB->m_strength : 1),
						groupC ? groupC->m_strength : 1);
					b2Vec2 midPoint = (float32) 1 / 3 * (pa + pb + pc);
					triad.pa = pa - midPoint;
					triad.pb = pb - midPoint;
					triad.pc = pc - midPoint;
					triad.ka = -b2Dot(dca, dab);
					triad.kb = -b2Dot(dab, dbc);
					triad.kc = -b2Dot(dbc, dca);
					triad.s = b2Cross(pa, pb) + b2Cross(pb, pc) + b2Cross(pc, pa);
				}
			}
			b2ParticleSystem* m_system;
			const ConnectionFilter* m_filter;
		public:
			UpdateTriadsCallback(
				b2ParticleSystem* system, const ConnectionFilter* filter)
			{
				m_system = system;
				m_filter = filter;
			}
		} callback(this, &filter);
		diagram.GetNodes(callback);
		std::stable_sort(
			m_triadBuffer.Begin(), m_triadBuffer.End(), CompareTriadIndices);
		m_triadBuffer.Unique(MatchTriadIndices);
	}
}

bool b2ParticleSystem::ComparePairIndices(
							const b2ParticlePair& a, const b2ParticlePair& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	return a.indexB < b.indexB;
}

bool b2ParticleSystem::MatchPairIndices(
							const b2ParticlePair& a, const b2ParticlePair& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB;
}

bool b2ParticleSystem::CompareTriadIndices(
							const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	int32 diffA = a.indexA - b.indexA;
	if (diffA != 0) return diffA < 0;
	int32 diffB = a.indexB - b.indexB;
	if (diffB != 0) return diffB < 0;
	return a.indexC < b.indexC;
}

bool b2ParticleSystem::MatchTriadIndices(
							const b2ParticleTriad& a, const b2ParticleTriad& b)
{
	return a.indexA == b.indexA && a.indexB == b.indexB && a.indexC == b.indexC;
}

// Only called from SolveZombie() or JoinParticleGroups().
void b2ParticleSystem::DestroyParticleGroup(b2ParticleGroup* group)
{
	b2Assert(m_groupCount > 0);
	b2Assert(group);

	if (m_world->m_destructionListener)
	{
		m_world->m_destructionListener->SayGoodbye(group);
	}

	SetGroupFlags(group, 0);
	for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
	{
		m_groupBuffer[i] = NULL;
	}

	if (group->m_prev)
	{
		group->m_prev->m_next = group->m_next;
	}
	if (group->m_next)
	{
		group->m_next->m_prev = group->m_prev;
	}
	if (group == m_groupList)
	{
		m_groupList = group->m_next;
	}

	--m_groupCount;
	group->~b2ParticleGroup();
	m_world->m_blockAllocator.Free(group, sizeof(b2ParticleGroup));
}

void b2ParticleSystem::ComputeWeight()
{
	// calculates the sum of contact-weights for each particle
	// that means dimensionless density
	memset(m_weightBuffer, 0, sizeof(*m_weightBuffer) * m_count);
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		float32 w = contact.weight;
		m_weightBuffer[a] += w;
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		float32 w = contact.GetWeight();
		m_weightBuffer[a] += w;
		m_weightBuffer[b] += w;
	}
}

void b2ParticleSystem::ComputeDepth()
{
	b2ParticleContact* contactGroups = (b2ParticleContact*) m_world->
		m_stackAllocator.Allocate(sizeof(b2ParticleContact) * m_contactBuffer.GetCount());
	int32 contactGroupsCount = 0;
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		const b2ParticleGroup* groupA = m_groupBuffer[a];
		const b2ParticleGroup* groupB = m_groupBuffer[b];
		if (groupA && groupA == groupB &&
			(groupA->m_groupFlags & b2_particleGroupNeedsUpdateDepth))
		{
			contactGroups[contactGroupsCount++] = contact;
		}
	}
	b2ParticleGroup** groupsToUpdate = (b2ParticleGroup**) m_world->
		m_stackAllocator.Allocate(sizeof(b2ParticleGroup*) * m_groupCount);
	int32 groupsToUpdateCount = 0;
	for (b2ParticleGroup* group = m_groupList; group; group = group->GetNext())
	{
		if (group->m_groupFlags & b2_particleGroupNeedsUpdateDepth)
		{
			groupsToUpdate[groupsToUpdateCount++] = group;
			SetGroupFlags(group,
						  group->m_groupFlags &
						  ~b2_particleGroupNeedsUpdateDepth);
			for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
			{
				m_accumulationBuffer[i] = 0;
			}
		}
	}
	// Compute sum of weight of contacts except between different groups.
	for (int32 k = 0; k < contactGroupsCount; k++)
	{
		const b2ParticleContact& contact = contactGroups[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		float32 w = contact.GetWeight();
		m_accumulationBuffer[a] += w;
		m_accumulationBuffer[b] += w;
	}
	b2Assert(m_depthBuffer);
	for (int32 i = 0; i < groupsToUpdateCount; i++)
	{
		const b2ParticleGroup* group = groupsToUpdate[i];
		for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
		{
			float32 w = m_accumulationBuffer[i];
			m_depthBuffer[i] = w < 0.8f ? 0 : b2_maxFloat;
		}
	}
	// The number of iterations is equal to particle number from the deepest
	// particle to the nearest surface particle, and in general it is smaller
	// than sqrt of total particle number.
	int32 iterationCount = (int32)b2Sqrt((float)m_count);
	for (int32 t = 0; t < iterationCount; t++)
	{
		bool updated = false;
		for (int32 k = 0; k < contactGroupsCount; k++)
		{
			const b2ParticleContact& contact = contactGroups[k];
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			float32 r = 1 - contact.GetWeight();
			float32& ap0 = m_depthBuffer[a];
			float32& bp0 = m_depthBuffer[b];
			float32 ap1 = bp0 + r;
			float32 bp1 = ap0 + r;
			if (ap0 > ap1)
			{
				ap0 = ap1;
				updated = true;
			}
			if (bp0 > bp1)
			{
				bp0 = bp1;
				updated = true;
			}
		}
		if (!updated)
		{
			break;
		}
	}
	for (int32 i = 0; i < groupsToUpdateCount; i++)
	{
		const b2ParticleGroup* group = groupsToUpdate[i];
		for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
		{
			float32& p = m_depthBuffer[i];
			if (p < b2_maxFloat)
			{
				p *= m_particleDiameter;
			}
			else
			{
				p = 0;
			}
		}
	}
	m_world->m_stackAllocator.Free(groupsToUpdate);
	m_world->m_stackAllocator.Free(contactGroups);
}

b2ParticleSystem::InsideBoundsEnumerator
b2ParticleSystem::GetInsideBoundsEnumerator(const b2AABB& aabb) const
{
	uint32 lowerTag = computeTag(m_inverseDiameter * aabb.lowerBound.x - 1,
								 m_inverseDiameter * aabb.lowerBound.y - 1);
	uint32 upperTag = computeTag(m_inverseDiameter * aabb.upperBound.x + 1,
								 m_inverseDiameter * aabb.upperBound.y + 1);
	const Proxy* beginProxy = m_proxyBuffer.Begin();
	const Proxy* endProxy = m_proxyBuffer.End();
	const Proxy* firstProxy = std::lower_bound(beginProxy, endProxy, lowerTag);
	const Proxy* lastProxy = std::upper_bound(firstProxy, endProxy, upperTag);
	return InsideBoundsEnumerator(lowerTag, upperTag, firstProxy, lastProxy);
}

inline void b2ParticleSystem::AddContact(int32 a, int32 b,
	b2GrowableBuffer<b2ParticleContact>& contacts) const
{
	b2Vec2 d = m_positionBuffer.data[b] - m_positionBuffer.data[a];
	float32 distBtParticlesSq = b2Dot(d, d);
	if (distBtParticlesSq < m_squaredDiameter)
	{
		float32 invD = b2InvSqrt(distBtParticlesSq);
		b2ParticleContact& contact = contacts.Append();
		contact.SetIndices(a, b);
		contact.SetFlags(m_flagsBuffer.data[a] | m_flagsBuffer.data[b]);
		// 1 - distBtParticles / diameter
		contact.SetWeight(1 - distBtParticlesSq * invD * m_inverseDiameter);
		contact.SetNormal(invD * d);
	}
}

void b2ParticleSystem::FindContacts_Reference(
	b2GrowableBuffer<b2ParticleContact>& contacts) const
{
	const Proxy* beginProxy = m_proxyBuffer.Begin();
	const Proxy* endProxy = m_proxyBuffer.End();

	contacts.SetCount(0);
	for (const Proxy *a = beginProxy, *c = beginProxy; a < endProxy; a++)
	{
		uint32 rightTag = computeRelativeTag(a->tag, 1, 0);
		for (const Proxy* b = a + 1; b < endProxy; b++)
		{
			if (rightTag < b->tag) break;
			AddContact(a->index, b->index, contacts);
		}
		uint32 bottomLeftTag = computeRelativeTag(a->tag, -1, 1);
		for (; c < endProxy; c++)
		{
			if (bottomLeftTag <= c->tag) break;
		}
		uint32 bottomRightTag = computeRelativeTag(a->tag, 1, 1);
		for (const Proxy* b = c; b < endProxy; b++)
		{
			if (bottomRightTag < b->tag) break;
			AddContact(a->index, b->index, contacts);
		}
	}
}

// Put the positions and indices in proxy-order. This allows us to process
// particles with SIMD, since adjacent particles are adjacent in memory.
void b2ParticleSystem::ReorderForFindContact(FindContactInput* reordered,
	                                         int alignedCount) const
{
	int i = 0;
	for (; i < m_count; ++i)
	{
		const int proxyIndex = m_proxyBuffer[i].index;
		FindContactInput& r = reordered[i];
		r.proxyIndex = proxyIndex;
		r.position = m_positionBuffer.data[proxyIndex];
	}

	// We process multiple elements at a time, so we may read off the end of
	// the array. Pad the array with a few elements, so we don't end up
	// outputing spurious contacts.
	for (; i < alignedCount; ++i)
	{
		FindContactInput& r = reordered[i];
		r.proxyIndex = 0;
		r.position = b2Vec2(b2_maxFloat, b2_maxFloat);
	}
}

// Check particles to the right of 'startIndex', outputing FindContactChecks
// until we find an index that is greater than 'bound'. We skip over the
// indices NUM_V32_SLOTS at a time, because they are processed in groups
// in the SIMD function.
inline void b2ParticleSystem::GatherChecksOneParticle(
	const uint32 bound,
	const int startIndex,
	const int particleIndex,
	int* nextUncheckedIndex,
	b2GrowableBuffer<FindContactCheck>& checks) const
{
	// The particles have to be heavily packed together in order for this
	// loop to iterate more than once. In almost all situations, it will
	// iterate less than twice.
	for (int comparatorIndex = startIndex;
		 comparatorIndex < m_count;
	     comparatorIndex += NUM_V32_SLOTS)
	{
		if (m_proxyBuffer[comparatorIndex].tag > bound)
			break;

		FindContactCheck& out = checks.Append();
		out.particleIndex = (uint16)particleIndex;
		out.comparatorIndex = (uint16)comparatorIndex;

		// This is faster inside the 'for' since there are so few iterations.
		if (nextUncheckedIndex != NULL)
		{
			*nextUncheckedIndex = comparatorIndex + NUM_V32_SLOTS;
		}
	}
}

void b2ParticleSystem::GatherChecks(
	b2GrowableBuffer<FindContactCheck>& checks) const
{
	int bottomLeftIndex = 0;
	for (int particleIndex = 0; particleIndex < m_count; ++particleIndex)
	{
		const uint32 particleTag = m_proxyBuffer[particleIndex].tag;

		// Add checks for particles to the right.
		const uint32 rightBound = particleTag + relativeTagRight;
		int nextUncheckedIndex = particleIndex + 1;
		GatherChecksOneParticle(rightBound,
								particleIndex + 1,
								particleIndex,
								&nextUncheckedIndex,
								checks);

		// Find comparator index below and to left of particle.
		const uint32 bottomLeftTag = particleTag + relativeTagBottomLeft;
		for (; bottomLeftIndex < m_count; ++bottomLeftIndex)
		{
			if (bottomLeftTag <= m_proxyBuffer[bottomLeftIndex].tag)
				break;
		}

		// Add checks for particles below.
		const uint32 bottomRightBound = particleTag + relativeTagBottomRight;
		const int bottomStartIndex = b2Max(bottomLeftIndex, nextUncheckedIndex);
		GatherChecksOneParticle(bottomRightBound,
								bottomStartIndex,
								particleIndex,
								NULL,
								checks);
	}
}

#if defined(BOX2D_SIMD_NEON)
void b2ParticleSystem::FindContacts_Simd(
	b2GrowableBuffer<b2ParticleContact>& contacts) const
{
	contacts.SetCount(0);

	const int alignedCount = m_count + NUM_V32_SLOTS;
	FindContactInput* reordered = (FindContactInput*)
		m_world->m_stackAllocator.Allocate(
			sizeof(FindContactInput) * alignedCount);

	// Put positions and indices into proxy-order.
	// This allows us to efficiently check for contacts using SIMD.
	ReorderForFindContact(reordered, alignedCount);

	// Perform broad-band contact check using tags to approximate
	// positions. This reduces the number of narrow-band contact checks
	// that use actual positions.
	static const int MAX_EXPECTED_CHECKS_PER_PARTICLE = 3;
	b2GrowableBuffer<FindContactCheck> checks(m_world->m_blockAllocator);
	checks.Reserve(MAX_EXPECTED_CHECKS_PER_PARTICLE * m_count);
	GatherChecks(checks);

	// Perform narrow-band contact checks using actual positions.
	// Any particles whose centers are within one diameter of each other are
	// considered contacting.
	FindContactsFromChecks_Simd(reordered, checks.Data(), checks.GetCount(),
								m_squaredDiameter, m_inverseDiameter,
								m_flagsBuffer.data, contacts);

	m_world->m_stackAllocator.Free(reordered);
}
#endif // defined(BOX2D_SIMD_NEON)

BOX2D_SIMD_INLINE
void b2ParticleSystem::FindContacts(
	b2GrowableBuffer<b2ParticleContact>& contacts) const
{
	#if defined(BOX2D_SIMD_NEON)
		FindContacts_Simd(contacts);
	#else
		FindContacts_Reference(contacts);
	#endif

	#if defined(BOX2D_SIMD_TEST_VS_REFERENCE)
		b2GrowableBuffer<b2ParticleContact>
			reference(m_world->m_blockAllocator);
		FindContacts_Reference(reference);

		b2Assert(contacts.GetCount() == reference.GetCount());
		for (int32 i = 0; i < contacts.GetCount(); ++i)
		{
			b2Assert(contacts[i].ApproximatelyEqual(reference[i]));
		}
	#endif // defined(BOX2D_SIMD_TEST_VS_REFERENCE)
}

static inline bool b2ParticleContactIsZombie(const b2ParticleContact& contact)
{
	return (contact.GetFlags() & b2_zombieParticle) == b2_zombieParticle;
}

// Get the world's contact filter if any particles with the
// b2_particleContactFilterParticle flag are present in the system.
inline b2ContactFilter* b2ParticleSystem::GetParticleContactFilter() const
{
	return (m_allParticleFlags & b2_particleContactFilterParticle) ?
		m_world->m_contactManager.m_contactFilter : NULL;
}

// Get the world's contact listener if any particles with the
// b2_particleContactListenerParticle flag are present in the system.
inline b2ContactListener* b2ParticleSystem::GetParticleContactListener() const
{
	return (m_allParticleFlags & b2_particleContactListenerParticle) ?
		m_world->m_contactManager.m_contactListener : NULL;
}

// Recalculate 'tag' in proxies using m_positionBuffer.
// The 'tag' is an approximation of position, in left-right, top-bottom order.
void b2ParticleSystem::UpdateProxies_Reference(
	b2GrowableBuffer<Proxy>& proxies) const
{
	const Proxy* const endProxy = proxies.End();
	for (Proxy* proxy = proxies.Begin(); proxy < endProxy; ++proxy)
	{
		int32 i = proxy->index;
		b2Vec2 p = m_positionBuffer.data[i];
		proxy->tag = computeTag(m_inverseDiameter * p.x,
								m_inverseDiameter * p.y);
	}
}

#if defined(BOX2D_SIMD_NEON)
// static
void b2ParticleSystem::UpdateProxyTags(
	const uint32* const tags,
	b2GrowableBuffer<Proxy>& proxies)
{
	const Proxy* const endProxy = proxies.End();
	for (Proxy* proxy = proxies.Begin(); proxy < endProxy; ++proxy)
	{
		proxy->tag = tags[proxy->index];
	}
}

void b2ParticleSystem::UpdateProxies_Simd(
	b2GrowableBuffer<Proxy>& proxies) const
{
	uint32* tags = (uint32*)
		m_world->m_stackAllocator.Allocate(m_count * sizeof(uint32));

	// Calculate tag for every position.
	// 'tags' array is in position-order.
	CalculateTags_Simd(m_positionBuffer.data, m_count,
					   m_inverseDiameter, tags);

	// Update 'tag' element in the 'proxies' array to the new values.
	UpdateProxyTags(tags, proxies);

	m_world->m_stackAllocator.Free(tags);
}
#endif // defined(BOX2D_SIMD_NEON)

// static
bool b2ParticleSystem::ProxyBufferHasIndex(
	int32 index, const Proxy* const a, int count)
{
	for (int j = 0; j < count; ++j)
	{
		if (a[j].index == index)
			return true;
	}
	return false;
}

// static
int b2ParticleSystem::NumProxiesWithSameTag(
	const Proxy* const a, const Proxy* const b, int count)
{
	const uint32 tag = a[0].tag;
	for (int num = 0; num < count; ++num)
	{
		if (a[num].tag != tag || b[num].tag != tag)
			return num;
	}
	return count;
}

// Precondition: both 'a' and 'b' should be sorted by tag, but don't need to be
// sorted by index.
// static
bool b2ParticleSystem::AreProxyBuffersTheSame(const b2GrowableBuffer<Proxy>& a,
								   			  const b2GrowableBuffer<Proxy>& b)
{
	if (a.GetCount() != b.GetCount())
		return false;

	// A given tag may have several indices. The order of these indices is
	// not important, but the set must be equivalent.
	for (int i = 0; i < a.GetCount();)
	{
		const int numWithSameTag = NumProxiesWithSameTag(
			&a[i], &b[i], a.GetCount() - i);
		if (numWithSameTag == 0)
			return false;

		for (int j = 0; j < numWithSameTag; ++j)
		{
			const bool hasIndex = ProxyBufferHasIndex(
				a[i + j].index, &b[i], numWithSameTag);
			if (!hasIndex)
				return false;
		}

		i += numWithSameTag;
	}
	return true;
}

BOX2D_SIMD_INLINE
void b2ParticleSystem::UpdateProxies(
	b2GrowableBuffer<Proxy>& proxies) const
{
	#if defined(BOX2D_SIMD_TEST_VS_REFERENCE)
		b2GrowableBuffer<Proxy> reference(proxies);
	#endif

	#if defined(BOX2D_SIMD_NEON)
		UpdateProxies_Simd(proxies);
	#else
		UpdateProxies_Reference(proxies);
	#endif

	#if defined(BOX2D_SIMD_TEST_VS_REFERENCE)
		UpdateProxies_Reference(reference);
		b2Assert(AreProxyBuffersTheSame(proxies, reference));
	#endif
}


// Sort the proxy array by 'tag'. This orders the particles into rows that
// run left-to-right, top-to-bottom. The rows are spaced m_particleDiameter
// apart, such that a particle in one row can only collide with the rows
// immediately above and below it. This ordering makes collision computation
// tractable.
//
// TODO OPT: The sort is a hot spot on the profiles. We could use SIMD to
// speed this up. See http://www.vldb.org/pvldb/1/1454171.pdf for an excellent
// explanation of a SIMD mergesort algorithm.
void b2ParticleSystem::SortProxies(b2GrowableBuffer<Proxy>& proxies) const
{
	std::sort(proxies.Begin(), proxies.End());
}

class b2ParticleContactRemovePredicate
{
public:
	b2ParticleContactRemovePredicate(
		b2ParticleSystem* system,
		b2ContactFilter* contactFilter) :
		m_system(system),
		m_contactFilter(contactFilter)
	{}

	bool operator()(const b2ParticleContact& contact)
	{
	    return (contact.GetFlags() & b2_particleContactFilterParticle)
	        && !m_contactFilter->ShouldCollide(m_system, contact.GetIndexA(),
	        								   contact.GetIndexB());
	}

private:
	b2ParticleSystem* m_system;
	b2ContactFilter* m_contactFilter;
};

// Only changes 'contacts', but the contact filter has a non-const 'this'
// pointer, so this member function cannot be const.
void b2ParticleSystem::FilterContacts(
	b2GrowableBuffer<b2ParticleContact>& contacts)
{
	// Optionally filter the contact.
	b2ContactFilter* const contactFilter = GetParticleContactFilter();
	if (contactFilter == NULL)
		return;

	contacts.RemoveIf(b2ParticleContactRemovePredicate(this, contactFilter));
}

void b2ParticleSystem::NotifyContactListenerPreContact(
	b2ParticlePairSet* particlePairs) const
{
	b2ContactListener* const contactListener = GetParticleContactListener();
	if (contactListener == NULL)
		return;

	particlePairs->Initialize(m_contactBuffer.Begin(),
							  m_contactBuffer.GetCount(),
						      GetFlagsBuffer());
}

// Note: This function is not const because 'this' in BeginContact and
// EndContact callbacks must be non-const. However, this function itself
// does not change any internal data (though the callbacks might).
void b2ParticleSystem::NotifyContactListenerPostContact(
	b2ParticlePairSet& particlePairs)
{
	b2ContactListener* const contactListener = GetParticleContactListener();
	if (contactListener == NULL)
		return;

	// Loop through all new contacts, reporting any new ones, and
	// "invalidating" the ones that still exist.
	const b2ParticleContact* const endContact = m_contactBuffer.End();
	for (b2ParticleContact* contact = m_contactBuffer.Begin();
		 contact < endContact; ++contact)
	{
		ParticlePair pair;
		pair.first = contact->GetIndexA();
		pair.second = contact->GetIndexB();
		const int32 itemIndex = particlePairs.Find(pair);
		if (itemIndex >= 0)
		{
			// Already touching, ignore this contact.
			particlePairs.Invalidate(itemIndex);
		}
		else
		{
			// Just started touching, inform the listener.
			contactListener->BeginContact(this, contact);
		}
	}

	// Report particles that are no longer touching.
	// That is, any pairs that were not invalidated above.
	const int32 pairCount = particlePairs.GetCount();
	const ParticlePair* const pairs = particlePairs.GetBuffer();
	const int8* const valid = particlePairs.GetValidBuffer();
	for (int32 i = 0; i < pairCount; ++i)
	{
		if (valid[i])
		{
			contactListener->EndContact(this, pairs[i].first,
										pairs[i].second);
		}
	}
}

void b2ParticleSystem::UpdateContacts(bool exceptZombie)
{
	UpdateProxies(m_proxyBuffer);
	SortProxies(m_proxyBuffer);

	b2ParticlePairSet particlePairs(&m_world->m_stackAllocator);
	NotifyContactListenerPreContact(&particlePairs);

	FindContacts(m_contactBuffer);
	FilterContacts(m_contactBuffer);

	NotifyContactListenerPostContact(particlePairs);

	if (exceptZombie)
	{
		m_contactBuffer.RemoveIf(b2ParticleContactIsZombie);
	}
}

void b2ParticleSystem::DetectStuckParticle(int32 particle)
{
	// Detect stuck particles
	//
	// The basic algorithm is to allow the user to specify an optional
	// threshold where we detect whenever a particle is contacting
	// more than one fixture for more than threshold consecutive
	// steps. This is considered to be "stuck", and these are put
	// in a list the user can query per step, if enabled, to deal with
	// such particles.

	if (m_stuckThreshold <= 0)
	{
		return;
	}

	// Get the state variables for this particle.
	int32 * const consecutiveCount =
			&m_consecutiveContactStepsBuffer.data[particle];
	int32 * const lastStep = &m_lastBodyContactStepBuffer.data[particle];
	int32 * const bodyCount = &m_bodyContactCountBuffer.data[particle];

	// This is only called when there is a body contact for this particle.
	++(*bodyCount);

	// We want to only trigger detection once per step, the first time we
	// contact more than one fixture in a step for a given particle.
	if (*bodyCount == 2)
	{
		++(*consecutiveCount);
		if (*consecutiveCount > m_stuckThreshold)
		{
			int32& newStuckParticle = m_stuckParticleBuffer.Append();
			newStuckParticle = particle;
		}
	}
	*lastStep = m_timestamp;
}

// Get the world's contact listener if any particles with the
// b2_fixtureContactListenerParticle flag are present in the system.
inline b2ContactListener* b2ParticleSystem::GetFixtureContactListener() const
{
	return (m_allParticleFlags & b2_fixtureContactListenerParticle) ?
		m_world->m_contactManager.m_contactListener : NULL;
}

// Get the world's contact filter if any particles with the
// b2_fixtureContactFilterParticle flag are present in the system.
inline b2ContactFilter* b2ParticleSystem::GetFixtureContactFilter() const
{
	return (m_allParticleFlags & b2_fixtureContactFilterParticle) ?
		m_world->m_contactManager.m_contactFilter : NULL;
}

/// Compute the axis-aligned bounding box for all particles contained
/// within this particle system.
/// @param aabb Returns the axis-aligned bounding box of the system.
void b2ParticleSystem::ComputeAABB(b2AABB* const aabb) const
{
	const int32 particleCount = GetParticleCount();
	b2Assert(aabb);
	aabb->lowerBound.x = +b2_maxFloat;
	aabb->lowerBound.y = +b2_maxFloat;
	aabb->upperBound.x = -b2_maxFloat;
	aabb->upperBound.y = -b2_maxFloat;

	for (int32 i = 0; i < particleCount; i++)
	{
		b2Vec2 p = m_positionBuffer.data[i];
		aabb->lowerBound = b2Min(aabb->lowerBound, p);
		aabb->upperBound = b2Max(aabb->upperBound, p);
	}
	aabb->lowerBound.x -= m_particleDiameter;
	aabb->lowerBound.y -= m_particleDiameter;
	aabb->upperBound.x += m_particleDiameter;
	aabb->upperBound.y += m_particleDiameter;
}

// Associate a memory allocator with this object.
FixedSetAllocator::FixedSetAllocator(
		b2StackAllocator* allocator) :
	m_buffer(NULL), m_valid(NULL), m_count(0), m_allocator(allocator)
{
	b2Assert(allocator);
}

// Allocate internal storage for this object.
int32 FixedSetAllocator::Allocate(
	const int32 itemSize, const int32 count)
{
	Clear();
	if (count)
	{
		m_buffer = m_allocator->Allocate(
			(itemSize + sizeof(*m_valid)) * count);
		b2Assert(m_buffer);
		m_valid = (int8*)m_buffer + (itemSize * count);
		memset(m_valid, 1, sizeof(*m_valid) * count);
		m_count = count;
	}
	return m_count;
}

// Deallocate the internal buffer if it's allocated.
void FixedSetAllocator::Clear()
{
	if (m_buffer)
	{
		m_allocator->Free(m_buffer);
		m_buffer = NULL;
        m_count = 0;
	}
}

// Search set for item returning the index of the item if it's found, -1
// otherwise.
template<typename T>
static int32 FindItemIndexInFixedSet(const TypedFixedSetAllocator<T>& set,
									 const T& item)
{
	if (set.GetCount())
	{
		const T* buffer = set.GetBuffer();
		const T* last = buffer + set.GetCount();
		const T* found = std::lower_bound( buffer, buffer + set.GetCount(),
											item, T::Compare);
		if( found != last )
		{
			return set.GetIndex( found );
		}
	}
	return -1;
}

// Initialize from a set of particle / body contacts for particles
// that have the b2_fixtureContactListenerParticle flag set.
void FixtureParticleSet::Initialize(
	const b2ParticleBodyContact * const bodyContacts,
	const int32 numBodyContacts,
	const uint32 * const particleFlagsBuffer)
{
	Clear();
	if (Allocate(numBodyContacts))
	{
		FixtureParticle* set = GetBuffer();
		int32 insertedContacts = 0;
		for (int32 i = 0; i < numBodyContacts; ++i)
		{
			FixtureParticle* const fixtureParticle = &set[i];
			const b2ParticleBodyContact& bodyContact = bodyContacts[i];
			if (bodyContact.index == b2_invalidParticleIndex ||
				!(particleFlagsBuffer[bodyContact.index] &
				  b2_fixtureContactListenerParticle))
			{
				continue;
			}
			fixtureParticle->first = bodyContact.fixture;
			fixtureParticle->second = bodyContact.index;
			insertedContacts++;
		}
		SetCount(insertedContacts);
		std::sort(set, set + insertedContacts, FixtureParticle::Compare);
	}
}

// Find the index of a particle / fixture pair in the set or -1 if it's not
// present.
int32 FixtureParticleSet::Find(
	const FixtureParticle& fixtureParticle) const
{
	return FindItemIndexInFixedSet(*this, fixtureParticle);
}

// Initialize from a set of particle contacts.
void b2ParticlePairSet::Initialize(
	const b2ParticleContact * const contacts, const int32 numContacts,
	const uint32 * const particleFlagsBuffer)
{
	Clear();
	if (Allocate(numContacts))
	{
		ParticlePair* set = GetBuffer();
		int32 insertedContacts = 0;
		for (int32 i = 0; i < numContacts; ++i)
		{
			ParticlePair* const pair = &set[i];
			const b2ParticleContact& contact = contacts[i];
			if (contact.GetIndexA() == b2_invalidParticleIndex ||
				contact.GetIndexB() == b2_invalidParticleIndex ||
				!((particleFlagsBuffer[contact.GetIndexA()] |
				   particleFlagsBuffer[contact.GetIndexB()]) &
				  b2_particleContactListenerParticle))
			{
				continue;
			}
			pair->first = contact.GetIndexA();
			pair->second = contact.GetIndexB();
			insertedContacts++;
		}
		SetCount(insertedContacts);
		std::sort(set, set + insertedContacts, ParticlePair::Compare);
	}
}

// Find the index of a particle pair in the set or -1 if it's not present.
int32 b2ParticlePairSet::Find(const ParticlePair& pair) const
{
	int32 index = FindItemIndexInFixedSet(*this, pair);
	if (index < 0)
	{
		ParticlePair swapped;
		swapped.first = pair.second;
		swapped.second = pair.first;
		index = FindItemIndexInFixedSet(*this, swapped);
	}
	return index;
}

/// Callback class to receive pairs of fixtures and particles which may be
/// overlapping. Used as an argument of b2World::QueryAABB.
class b2FixtureParticleQueryCallback : public b2QueryCallback
{
public:
	explicit b2FixtureParticleQueryCallback(b2ParticleSystem* system)
	{
		m_system = system;
	}

private:
	// Skip reporting particles.
	bool ShouldQueryParticleSystem(const b2ParticleSystem* system)
	{
		B2_NOT_USED(system);
		return false;
	}

	// Receive a fixture and call ReportFixtureAndParticle() for each particle
	// inside aabb of the fixture.
	bool ReportFixture(b2Fixture* fixture)
	{
		if (fixture->IsSensor())
		{
			return true;
		}
		const b2Shape* shape = fixture->GetShape();
		int32 childCount = shape->GetChildCount();
		for (int32 childIndex = 0; childIndex < childCount; childIndex++)
		{
			b2AABB aabb = fixture->GetAABB(childIndex);
			b2ParticleSystem::InsideBoundsEnumerator enumerator =
								m_system->GetInsideBoundsEnumerator(aabb);
			int32 index;
			while ((index = enumerator.GetNext()) >= 0)
			{
				ReportFixtureAndParticle(fixture, childIndex, index);
			}
		}
		return true;
	}

	// Receive a fixture and a particle which may be overlapping.
	virtual void ReportFixtureAndParticle(
						b2Fixture* fixture, int32 childIndex, int32 index) = 0;

protected:
	b2ParticleSystem* m_system;
};

void b2ParticleSystem::NotifyBodyContactListenerPreContact(
	FixtureParticleSet* fixtureSet) const
{
	b2ContactListener* const contactListener = GetFixtureContactListener();
	if (contactListener == NULL)
		return;

	fixtureSet->Initialize(m_bodyContactBuffer.Begin(),
						   m_bodyContactBuffer.GetCount(),
						   GetFlagsBuffer());
}

// If a contact listener is present and the contact is just starting
// report the contact.  If the contact is already in progress invalid
// the contact from m_fixtureSet.
void b2ParticleSystem::NotifyBodyContactListenerPostContact(
	FixtureParticleSet& fixtureSet)
{
	b2ContactListener* const contactListener = GetFixtureContactListener();
	if (contactListener == NULL)
		return;

	// Loop through all new contacts, reporting any new ones, and
	// "invalidating" the ones that still exist.
	for (b2ParticleBodyContact* contact = m_bodyContactBuffer.Begin();
		 contact != m_bodyContactBuffer.End(); ++contact)
	{
		b2Assert(contact);
		FixtureParticle fixtureParticleToFind;
		fixtureParticleToFind.first = contact->fixture;
		fixtureParticleToFind.second = contact->index;
		const int32 index = fixtureSet.Find(fixtureParticleToFind);
		if (index >= 0)
		{
			// Already touching remove this from the set.
			fixtureSet.Invalidate(index);
		}
		else
		{
			// Just started touching, report it!
			contactListener->BeginContact(this, contact);
		}
	}

	// If the contact listener is enabled, report all fixtures that are no
	// longer in contact with particles.
	const FixtureParticle* const fixtureParticles = fixtureSet.GetBuffer();
	const int8* const fixtureParticlesValid = fixtureSet.GetValidBuffer();
	const int32 fixtureParticleCount = fixtureSet.GetCount();
	for (int32 i = 0; i < fixtureParticleCount; ++i)
	{
		if (fixtureParticlesValid[i])
		{
			const FixtureParticle* const fixtureParticle =
				&fixtureParticles[i];
			contactListener->EndContact(fixtureParticle->first, this,
										fixtureParticle->second);
		}
	}
}


void b2ParticleSystem::UpdateBodyContacts()
{
	// If the particle contact listener is enabled, generate a set of
	// fixture / particle contacts.
	FixtureParticleSet fixtureSet(&m_world->m_stackAllocator);
	NotifyBodyContactListenerPreContact(&fixtureSet);

	if (m_stuckThreshold > 0)
	{
		const int32 particleCount = GetParticleCount();
		for (int32 i = 0; i < particleCount; i++)
		{
			// Detect stuck particles, see comment in
			// b2ParticleSystem::DetectStuckParticle()
			m_bodyContactCountBuffer.data[i] = 0;
			if (m_timestamp > (m_lastBodyContactStepBuffer.data[i] + 1))
			{
				m_consecutiveContactStepsBuffer.data[i] = 0;
			}
		}
	}
	m_bodyContactBuffer.SetCount(0);
	m_stuckParticleBuffer.SetCount(0);

	class UpdateBodyContactsCallback : public b2FixtureParticleQueryCallback
	{
		// Call the contact filter if it's set, to determine whether to
		// filter this contact.  Returns true if contact calculations should
		// be performed, false otherwise.
		inline bool ShouldCollide(b2Fixture * const fixture,
								  int32 particleIndex)
		{
			if (m_contactFilter)
			{
				const uint32* const flags = m_system->GetFlagsBuffer();
				if (flags[particleIndex] & b2_fixtureContactFilterParticle)
				{
					return m_contactFilter->ShouldCollide(fixture, m_system,
														  particleIndex);
				}
			}
			return true;
		}

		void ReportFixtureAndParticle(
								b2Fixture* fixture, int32 childIndex, int32 a)
		{
			b2Vec2 ap = m_system->m_positionBuffer.data[a];
			float32 d;
			b2Vec2 n;
			fixture->ComputeDistance(ap, &d, &n, childIndex);
			if (d < m_system->m_particleDiameter && ShouldCollide(fixture, a))
			{
				b2Body* b = fixture->GetBody();
				b2Vec2 bp = b->GetWorldCenter();
				float32 bm = b->GetMass();
				float32 bI =
					b->GetInertia() - bm * b->GetLocalCenter().LengthSquared();
				float32 invBm = bm > 0 ? 1 / bm : 0;
				float32 invBI = bI > 0 ? 1 / bI : 0;
				float32 invAm =
					m_system->m_flagsBuffer.data[a] &
					b2_wallParticle ? 0 : m_system->GetParticleInvMass();
				b2Vec2 rp = ap - bp;
				float32 rpn = b2Cross(rp, n);
				float32 invM = invAm + invBm + invBI * rpn * rpn;

				b2ParticleBodyContact& contact =
					m_system->m_bodyContactBuffer.Append();
				contact.index = a;
				contact.body = b;
				contact.fixture = fixture;
				contact.weight = 1 - d * m_system->m_inverseDiameter;
				contact.normal = -n;
				contact.mass = invM > 0 ? 1 / invM : 0;
				m_system->DetectStuckParticle(a);
			}
		}

		b2ContactFilter* m_contactFilter;

	public:
		UpdateBodyContactsCallback(
			b2ParticleSystem* system, b2ContactFilter* contactFilter):
			b2FixtureParticleQueryCallback(system)
		{
			m_contactFilter = contactFilter;
		}
	} callback(this, GetFixtureContactFilter());

	b2AABB aabb;
	ComputeAABB(&aabb);
	m_world->QueryAABB(&callback, aabb);

	if (m_def.strictContactCheck)
	{
		RemoveSpuriousBodyContacts();
	}

	NotifyBodyContactListenerPostContact(fixtureSet);
}

void b2ParticleSystem::RemoveSpuriousBodyContacts()
{
	// At this point we have a list of contact candidates based on AABB
	// overlap.The AABB query that  generated this returns all collidable
	// fixtures overlapping particle bounding boxes.  This breaks down around
	// vertices where two shapes intersect, such as a "ground" surface made
	// of multiple b2PolygonShapes; it potentially applies a lot of spurious
	// impulses from normals that should not actually contribute.  See the
	// Ramp example in Testbed.
	//
	// To correct for this, we apply this algorithm:
	//   * sort contacts by particle and subsort by weight (nearest to farthest)
	//   * for each contact per particle:
	//      - project a point at the contact distance along the inverse of the
	//        contact normal
	//      - if this intersects the fixture that generated the contact, apply
	//         it, otherwise discard as impossible
	//      - repeat for up to n nearest contacts, currently we get good results
	//        from n=3.
	std::sort(m_bodyContactBuffer.Begin(), m_bodyContactBuffer.End(),
				b2ParticleSystem::BodyContactCompare);

	int32 discarded = 0;
	std::remove_if(m_bodyContactBuffer.Begin(),
					m_bodyContactBuffer.End(),
					b2ParticleBodyContactRemovePredicate(this, &discarded));

	m_bodyContactBuffer.SetCount(m_bodyContactBuffer.GetCount() - discarded);
}

bool b2ParticleSystem::BodyContactCompare(const b2ParticleBodyContact &lhs,
										  const b2ParticleBodyContact &rhs)
{
	if (lhs.index == rhs.index)
	{
		// Subsort by weight, decreasing.
		return lhs.weight > rhs.weight;
	}
	return lhs.index < rhs.index;
}


void b2ParticleSystem::SolveCollision(const b2TimeStep& step)
{
	// This function detects particles which are crossing boundary of bodies
	// and modifies velocities of them so that they will move just in front of
	// boundary. This function function also applies the reaction force to
	// bodies as precisely as the numerical stability is kept.
	b2AABB aabb;
	aabb.lowerBound.x = +b2_maxFloat;
	aabb.lowerBound.y = +b2_maxFloat;
	aabb.upperBound.x = -b2_maxFloat;
	aabb.upperBound.y = -b2_maxFloat;
	for (int32 i = 0; i < m_count; i++)
	{
		b2Vec2 v = m_velocityBuffer.data[i];
		b2Vec2 p1 = m_positionBuffer.data[i];
		b2Vec2 p2 = p1 + step.dt * v;
		aabb.lowerBound = b2Min(aabb.lowerBound, b2Min(p1, p2));
		aabb.upperBound = b2Max(aabb.upperBound, b2Max(p1, p2));
	}
	class SolveCollisionCallback : public b2FixtureParticleQueryCallback
	{
		// Call the contact filter if it's set, to determine whether to
		// filter this contact.  Returns true if contact calculations should
		// be performed, false otherwise.
		inline bool ShouldCollide(b2Fixture * const fixture,
								  int32 particleIndex)
		{
			if (m_contactFilter) {
				const uint32* const flags = m_system->GetFlagsBuffer();
				if (flags[particleIndex] & b2_fixtureContactFilterParticle) {
					return m_contactFilter->ShouldCollide(fixture, m_system,
														  particleIndex);
				}
			}
			return true;
		}

		void ReportFixtureAndParticle(
								b2Fixture* fixture, int32 childIndex, int32 a)
		{
			if (ShouldCollide(fixture, a)) {
				b2Body* body = fixture->GetBody();
				b2Vec2 ap = m_system->m_positionBuffer.data[a];
				b2Vec2 av = m_system->m_velocityBuffer.data[a];
				b2RayCastOutput output;
				b2RayCastInput input;
				if (m_system->m_iterationIndex == 0)
				{
					// Put 'ap' in the local space of the previous frame
					b2Vec2 p1 = b2MulT(body->m_xf0, ap);
					if (fixture->GetShape()->GetType() == b2Shape::e_circle)
					{
						// Make relative to the center of the circle
						p1 -= body->GetLocalCenter();
						// Re-apply rotation about the center of the
						// circle
						p1 = b2Mul(body->m_xf0.q, p1);
						// Subtract rotation of the current frame
						p1 = b2MulT(body->m_xf.q, p1);
						// Return to local space
						p1 += body->GetLocalCenter();
					}
					// Return to global space and apply rotation of current frame
					input.p1 = b2Mul(body->m_xf, p1);
				}
				else
				{
					input.p1 = ap;
				}
				input.p2 = ap + m_step.dt * av;
				input.maxFraction = 1;
				if (fixture->RayCast(&output, input, childIndex))
				{
					b2Vec2 n = output.normal;
					b2Vec2 p =
						(1 - output.fraction) * input.p1 +
						output.fraction * input.p2 +
						b2_linearSlop * n;
					b2Vec2 v = m_step.inv_dt * (p - ap);
					m_system->m_velocityBuffer.data[a] = v;
					b2Vec2 f = m_step.inv_dt *
						m_system->GetParticleMass() * (av - v);
					m_system->ParticleApplyForce(a, f);
				}
			}
		}

		b2TimeStep m_step;
		b2ContactFilter* m_contactFilter;

	public:
		SolveCollisionCallback(
			b2ParticleSystem* system, const b2TimeStep& step, b2ContactFilter* contactFilter) :
			b2FixtureParticleQueryCallback(system)
		{
			m_step = step;
			m_contactFilter = contactFilter;
		}
	} callback(this, step, GetFixtureContactFilter());
	m_world->QueryAABB(&callback, aabb);
}

void b2ParticleSystem::SolveBarrier(const b2TimeStep& step)
{
	// If a particle is passing between paired barrier particles,
	// its velocity will be decelerated to avoid passing.
	for (int32 i = 0; i < m_count; i++)
	{
		uint32 flags = m_flagsBuffer.data[i];
		static const uint32 k_barrierWallFlags =
										b2_barrierParticle | b2_wallParticle;
		if ((flags & k_barrierWallFlags) == k_barrierWallFlags)
		{
			m_velocityBuffer.data[i].SetZero();
		}
	}
	float32 tmax = b2_barrierCollisionTime * step.dt;
	for (int32 k = 0; k < m_pairBuffer.GetCount(); k++)
	{
		const b2ParticlePair& pair = m_pairBuffer[k];
		if (pair.flags & b2_barrierParticle)
		{
			int32 a = pair.indexA;
			int32 b = pair.indexB;
			b2Vec2 pa = m_positionBuffer.data[a];
			b2Vec2 pb = m_positionBuffer.data[b];
			b2AABB aabb;
			aabb.lowerBound = b2Min(pa, pb);
			aabb.upperBound = b2Max(pa, pb);
			b2ParticleGroup *aGroup = m_groupBuffer[a];
			b2ParticleGroup *bGroup = m_groupBuffer[b];
			b2Vec2 va = GetLinearVelocity(aGroup, a, pa);
			b2Vec2 vb = GetLinearVelocity(bGroup, b, pb);
			b2Vec2 pba = pb - pa;
			b2Vec2 vba = vb - va;
			InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
			int32 c;
			while ((c = enumerator.GetNext()) >= 0)
			{
				b2Vec2 pc = m_positionBuffer.data[c];
				b2ParticleGroup *cGroup = m_groupBuffer[c];
				if (aGroup != cGroup && bGroup != cGroup)
				{
					b2Vec2 vc = GetLinearVelocity(cGroup, c, pc);
					// Solve the equation below:
					//   (1-s)*(pa+t*va)+s*(pb+t*vb) = pc+t*vc
					// which expresses that the particle c will pass a line
					// connecting the particles a and b at the time of t.
					// if s is between 0 and 1, c will pass between a and b.
					b2Vec2 pca = pc - pa;
					b2Vec2 vca = vc - va;
					float32 e2 = b2Cross(vba, vca);
					float32 e1 = b2Cross(pba, vca) - b2Cross(pca, vba);
					float32 e0 = b2Cross(pba, pca);
					float32 s, t;
					b2Vec2 qba, qca;
					if (e2 == 0)
					{
						if (e1 == 0) continue;
						t = - e0 / e1;
						if (!(t >= 0 && t < tmax)) continue;
						qba = pba + t * vba;
						qca = pca + t * vca;
						s = b2Dot(qba, qca) / b2Dot(qba, qba);
						if (!(s >= 0 && s <= 1)) continue;
					}
					else
					{
						float32 det = e1 * e1 - 4 * e0 * e2;
						if (det < 0) continue;
						float32 sqrtDet = b2Sqrt(det);
						float32 t1 = (- e1 - sqrtDet) / (2 * e2);
						float32 t2 = (- e1 + sqrtDet) / (2 * e2);
						if (t1 > t2) b2Swap(t1, t2);
						t = t1;
						qba = pba + t * vba;
						qca = pca + t * vca;
						s = b2Dot(qba, qca) / b2Dot(qba, qba);
						if (!(t >= 0 && t < tmax && s >= 0 && s <= 1))
						{
							t = t2;
							if (!(t >= 0 && t < tmax)) continue;
							qba = pba + t * vba;
							qca = pca + t * vca;
							s = b2Dot(qba, qca) / b2Dot(qba, qba);
							if (!(s >= 0 && s <= 1)) continue;
						}
					}
					// Apply a force to particle c so that it will have the
					// interpolated velocity at the collision point on line ab.
					b2Vec2 dv = va + s * vba - vc;
					b2Vec2 f = GetParticleMass() * dv;
					if (IsRigidGroup(cGroup))
					{
						// If c belongs to a rigid group, the force will be
						// distributed in the group.
						float32 mass = cGroup->GetMass();
						float32 inertia = cGroup->GetInertia();
						if (mass > 0)
						{
							cGroup->m_linearVelocity += 1 / mass * f;
						}
						if (inertia > 0)
						{
							cGroup->m_angularVelocity +=
								b2Cross(pc - cGroup->GetCenter(), f) / inertia;
						}
					}
					else
					{
						m_velocityBuffer.data[c] += dv;
					}
					// Apply a reversed force to particle c after particle
					// movement so that momentum will be preserved.
					ParticleApplyForce(c, -step.inv_dt * f);
				}
			}
		}
	}
}

void b2ParticleSystem::Solve(const b2TimeStep& step)
{
	if (m_count == 0)
	{
		return;
	}
	// If particle lifetimes are enabled, destroy particles that are too old.
	if (m_expirationTimeBuffer.data)
	{
		SolveLifetimes(step);
	}
	if (m_allParticleFlags & b2_zombieParticle)
	{
		SolveZombie();
	}
	if (m_needsUpdateAllParticleFlags)
	{
		UpdateAllParticleFlags();
	}
	if (m_needsUpdateAllGroupFlags)
	{
		UpdateAllGroupFlags();
	}
	if (m_paused)
	{
		return;
	}
	for (m_iterationIndex = 0;
		m_iterationIndex < step.particleIterations;
		m_iterationIndex++)
	{
		++m_timestamp;
		b2TimeStep subStep = step;
		subStep.dt /= step.particleIterations;
		subStep.inv_dt *= step.particleIterations;
		UpdateContacts(false);
		UpdateBodyContacts();
		ComputeWeight();
		if (m_allGroupFlags & b2_particleGroupNeedsUpdateDepth)
		{
			ComputeDepth();
		}
		if (m_allParticleFlags & b2_reactiveParticle)
		{
			UpdatePairsAndTriadsWithReactiveParticles();
		}
		if (m_hasForce)
		{
			SolveForce(subStep);
		}
		if (m_allParticleFlags & b2_viscousParticle)
		{
			SolveViscous();
		}
		if (m_allParticleFlags & b2_repulsiveParticle)
		{
			SolveRepulsive(subStep);
		}
		if (m_allParticleFlags & b2_powderParticle)
		{
			SolvePowder(subStep);
		}
		if (m_allParticleFlags & b2_tensileParticle)
		{
			SolveTensile(subStep);
		}
		if (m_allGroupFlags & b2_solidParticleGroup)
		{
			SolveSolid(subStep);
		}
		if (m_allParticleFlags & b2_colorMixingParticle)
		{
			SolveColorMixing();
		}
		SolveGravity(subStep);
		if (m_allParticleFlags & b2_staticPressureParticle)
		{
			SolveStaticPressure(subStep);
		}
		SolvePressure(subStep);
		SolveDamping(subStep);
		if (m_allParticleFlags & k_extraDampingFlags)
		{
			SolveExtraDamping();
		}
		// SolveElastic and SolveSpring refer the current velocities for
		// numerical stability, they should be called as late as possible.
		if (m_allParticleFlags & b2_elasticParticle)
		{
			SolveElastic(subStep);
		}
		if (m_allParticleFlags & b2_springParticle)
		{
			SolveSpring(subStep);
		}
		LimitVelocity(subStep);
		if (m_allGroupFlags & b2_rigidParticleGroup)
		{
			SolveRigidDamping();
		}
		if (m_allParticleFlags & b2_barrierParticle)
		{
			SolveBarrier(subStep);
		}
		// SolveCollision, SolveRigid and SolveWall should be called after
		// other force functions because they may require particles to have
		// specific velocities.
		SolveCollision(subStep);
		if (m_allGroupFlags & b2_rigidParticleGroup)
		{
			SolveRigid(subStep);
		}
		if (m_allParticleFlags & b2_wallParticle)
		{
			SolveWall();
		}
		// The particle positions can be updated only at the end of substep.
		for (int32 i = 0; i < m_count; i++)
		{
			m_positionBuffer.data[i] += subStep.dt * m_velocityBuffer.data[i];
		}
	}
}

void b2ParticleSystem::UpdateAllParticleFlags()
{
	m_allParticleFlags = 0;
	for (int32 i = 0; i < m_count; i++)
	{
		m_allParticleFlags |= m_flagsBuffer.data[i];
	}
	m_needsUpdateAllParticleFlags = false;
}

void b2ParticleSystem::UpdateAllGroupFlags()
{
	m_allGroupFlags = 0;
	for (const b2ParticleGroup* group = m_groupList; group;
		 group = group->GetNext())
	{
		m_allGroupFlags |= group->m_groupFlags;
	}
	m_needsUpdateAllGroupFlags = false;
}

void b2ParticleSystem::LimitVelocity(const b2TimeStep& step)
{
	float32 criticalVelocitySquared = GetCriticalVelocitySquared(step);
	for (int32 i = 0; i < m_count; i++)
	{
		b2Vec2& v = m_velocityBuffer.data[i];
		float32 v2 = b2Dot(v, v);
		if (v2 > criticalVelocitySquared)
		{
			v *= b2Sqrt(criticalVelocitySquared / v2);
		}
	}
}

void b2ParticleSystem::SolveGravity(const b2TimeStep& step)
{
	b2Vec2 gravity = step.dt * m_def.gravityScale * m_world->GetGravity();
	for (int32 i = 0; i < m_count; i++)
	{
		m_velocityBuffer.data[i] += gravity;
	}
}

void b2ParticleSystem::SolveStaticPressure(const b2TimeStep& step)
{
	m_staticPressureBuffer = RequestBuffer(m_staticPressureBuffer);
	float32 criticalPressure = GetCriticalPressure(step);
	float32 pressurePerWeight = m_def.staticPressureStrength * criticalPressure;
	float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	float32 relaxation = m_def.staticPressureRelaxation;
	/// Compute pressure satisfying the modified Poisson equation:
	///     Sum_for_j((p_i - p_j) * w_ij) + relaxation * p_i =
	///     pressurePerWeight * (w_i - b2_minParticleWeight)
	/// by iterating the calculation:
	///     p_i = (Sum_for_j(p_j * w_ij) + pressurePerWeight *
	///           (w_i - b2_minParticleWeight)) / (w_i + relaxation)
	/// where
	///     p_i and p_j are static pressure of particle i and j
	///     w_ij is contact weight between particle i and j
	///     w_i is sum of contact weight of particle i
	for (int32 t = 0; t < m_def.staticPressureIterations; t++)
	{
		memset(m_accumulationBuffer, 0,
			   sizeof(*m_accumulationBuffer) * m_count);
		for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
		{
			const b2ParticleContact& contact = m_contactBuffer[k];
			if (contact.GetFlags() & b2_staticPressureParticle)
			{
				int32 a = contact.GetIndexA();
				int32 b = contact.GetIndexB();
				float32 w = contact.GetWeight();
				m_accumulationBuffer[a] +=
					w * m_staticPressureBuffer[b]; // a <- b
				m_accumulationBuffer[b] +=
					w * m_staticPressureBuffer[a]; // b <- a
			}
		}
		for (int32 i = 0; i < m_count; i++)
		{
			float32 w = m_weightBuffer[i];
			if (m_flagsBuffer.data[i] & b2_staticPressureParticle)
			{
				float32 wh = m_accumulationBuffer[i];
				float32 h =
					(wh + pressurePerWeight * (w - b2_minParticleWeight)) /
					(w + relaxation);
				m_staticPressureBuffer[i] = b2Clamp(h, 0.0f, maxPressure);
			}
			else
			{
				m_staticPressureBuffer[i] = 0;
			}
		}
	}
}

void b2ParticleSystem::SolvePressure(const b2TimeStep& step)
{
	// calculates pressure as a linear function of density
	float32 criticalPressure = GetCriticalPressure(step);
	float32 pressurePerWeight = m_def.pressureStrength * criticalPressure;
	float32 maxPressure = b2_maxParticlePressure * criticalPressure;
	for (int32 i = 0; i < m_count; i++)
	{
		float32 w = m_weightBuffer[i];
		float32 h = pressurePerWeight * b2Max(0.0f, w - b2_minParticleWeight);
		m_accumulationBuffer[i] = b2Min(h, maxPressure);
	}
	// ignores particles which have their own repulsive force
	if (m_allParticleFlags & k_noPressureFlags)
	{
		for (int32 i = 0; i < m_count; i++)
		{
			if (m_flagsBuffer.data[i] & k_noPressureFlags)
			{
				m_accumulationBuffer[i] = 0;
			}
		}
	}
	// static pressure
	if (m_allParticleFlags & b2_staticPressureParticle)
	{
		b2Assert(m_staticPressureBuffer);
		for (int32 i = 0; i < m_count; i++)
		{
			if (m_flagsBuffer.data[i] & b2_staticPressureParticle)
			{
				m_accumulationBuffer[i] += m_staticPressureBuffer[i];
			}
		}
	}
	// applies pressure between each particles in contact
	float32 velocityPerPressure = step.dt / (m_def.density * m_particleDiameter);
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		b2Body* b = contact.body;
		float32 w = contact.weight;
		float32 m = contact.mass;
		b2Vec2 n = contact.normal;
		b2Vec2 p = m_positionBuffer.data[a];
		float32 h = m_accumulationBuffer[a] + pressurePerWeight * w;
		b2Vec2 f = velocityPerPressure * w * m * h * n;
		m_velocityBuffer.data[a] -= GetParticleInvMass() * f;
		b->ApplyLinearImpulse(f, p, true);
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		float32 w = contact.GetWeight();
		b2Vec2 n = contact.GetNormal();
		float32 h = m_accumulationBuffer[a] + m_accumulationBuffer[b];
		b2Vec2 f = velocityPerPressure * w * h * n;
		m_velocityBuffer.data[a] -= f;
		m_velocityBuffer.data[b] += f;
	}
}

void b2ParticleSystem::SolveDamping(const b2TimeStep& step)
{
	// reduces normal velocity of each contact
	float32 linearDamping = m_def.dampingStrength;
	float32 quadraticDamping = 1 / GetCriticalVelocity(step);
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		b2Body* b = contact.body;
		float32 w = contact.weight;
		float32 m = contact.mass;
		b2Vec2 n = contact.normal;
		b2Vec2 p = m_positionBuffer.data[a];
		b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
				   m_velocityBuffer.data[a];
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			float32 damping =
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * m * vn * n;
			m_velocityBuffer.data[a] += GetParticleInvMass() * f;
			b->ApplyLinearImpulse(-f, p, true);
		}
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		float32 w = contact.GetWeight();
		b2Vec2 n = contact.GetNormal();
		b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			float32 damping =
				b2Max(linearDamping * w, b2Min(- quadraticDamping * vn, 0.5f));
			b2Vec2 f = damping * vn * n;
			m_velocityBuffer.data[a] += f;
			m_velocityBuffer.data[b] -= f;
		}
	}
}

inline bool b2ParticleSystem::IsRigidGroup(b2ParticleGroup *group) const
{
	return group && (group->m_groupFlags & b2_rigidParticleGroup);
}

inline b2Vec2 b2ParticleSystem::GetLinearVelocity(
	b2ParticleGroup *group, int32 particleIndex,
	const b2Vec2 &point) const
{
	if (IsRigidGroup(group))
	{
		return group->GetLinearVelocityFromWorldPoint(point);
	}
	else
	{
		return m_velocityBuffer.data[particleIndex];
	}
}

inline void b2ParticleSystem::InitDampingParameter(
	float32* invMass, float32* invInertia, float32* tangentDistance,
	float32 mass, float32 inertia, const b2Vec2& center,
	const b2Vec2& point, const b2Vec2& normal) const
{
	*invMass = mass > 0 ? 1 / mass : 0;
	*invInertia = inertia > 0 ? 1 / inertia : 0;
	*tangentDistance = b2Cross(point - center, normal);
}

inline void b2ParticleSystem::InitDampingParameterWithRigidGroupOrParticle(
	float32* invMass, float32* invInertia, float32* tangentDistance,
	bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
	const b2Vec2& point, const b2Vec2& normal) const
{
	if (isRigidGroup)
	{
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			group->GetMass(), group->GetInertia(), group->GetCenter(),
			point, normal);
	}
	else
	{
		uint32 flags = m_flagsBuffer.data[particleIndex];
		InitDampingParameter(
			invMass, invInertia, tangentDistance,
			flags & b2_wallParticle ? 0 : GetParticleMass(), 0, point,
			point, normal);
	}
}

inline float32 b2ParticleSystem::ComputeDampingImpulse(
	float32 invMassA, float32 invInertiaA, float32 tangentDistanceA,
	float32 invMassB, float32 invInertiaB, float32 tangentDistanceB,
	float32 normalVelocity) const
{
	float32 invMass =
		invMassA + invInertiaA * tangentDistanceA * tangentDistanceA +
		invMassB + invInertiaB * tangentDistanceB * tangentDistanceB;
	return invMass > 0 ? normalVelocity / invMass : 0;
}

inline void b2ParticleSystem::ApplyDamping(
	float32 invMass, float32 invInertia, float32 tangentDistance,
	bool isRigidGroup, b2ParticleGroup* group, int32 particleIndex,
	float32 impulse, const b2Vec2& normal)
{
	if (isRigidGroup)
	{
		group->m_linearVelocity += impulse * invMass * normal;
		group->m_angularVelocity += impulse * tangentDistance * invInertia;
	}
	else
	{
		m_velocityBuffer.data[particleIndex] += impulse * invMass * normal;
	}
}

void b2ParticleSystem::SolveRigidDamping()
{
	// Apply impulse to rigid particle groups colliding with other objects
	// to reduce relative velocity at the colliding point.
	float32 damping = m_def.dampingStrength;
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		b2ParticleGroup* aGroup = m_groupBuffer[a];
		if (IsRigidGroup(aGroup))
		{
			b2Body* b = contact.body;
			b2Vec2 n = contact.normal;
			float32 w = contact.weight;
			b2Vec2 p = m_positionBuffer.data[a];
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
					   aGroup->GetLinearVelocityFromWorldPoint(p);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			// The group's average velocity at particle position 'p' is pushing
			// the particle into the body.
			{
				float32 invMassA, invInertiaA, tangentDistanceA;
				float32 invMassB, invInertiaB, tangentDistanceB;
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassA, &invInertiaA, &tangentDistanceA,
					true, aGroup, a, p, n);
				InitDampingParameter(
					&invMassB, &invInertiaB, &tangentDistanceB,
					b->GetMass(),
					// Calculate b->m_I from public functions of b2Body.
					b->GetInertia() -
							b->GetMass() * b->GetLocalCenter().LengthSquared(),
					b->GetWorldCenter(),
					p, n);
				float32 f = damping * b2Min(w, 1.0f) * ComputeDampingImpulse(
					invMassA, invInertiaA, tangentDistanceA,
					invMassB, invInertiaB, tangentDistanceB,
					vn);
				ApplyDamping(
					invMassA, invInertiaA, tangentDistanceA,
					true, aGroup, a, f, n);
				b->ApplyLinearImpulse(-f * n, p, true);
			}
		}
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		b2Vec2 n = contact.GetNormal();
		float32 w = contact.GetWeight();
		b2ParticleGroup* aGroup = m_groupBuffer[a];
		b2ParticleGroup* bGroup = m_groupBuffer[b];
		bool aRigid = IsRigidGroup(aGroup);
		bool bRigid = IsRigidGroup(bGroup);
		if (aGroup != bGroup && (aRigid || bRigid))
		{
			b2Vec2 p =
				0.5f * (m_positionBuffer.data[a] + m_positionBuffer.data[b]);
			b2Vec2 v =
				GetLinearVelocity(bGroup, b, p) -
				GetLinearVelocity(aGroup, a, p);
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				float32 invMassA, invInertiaA, tangentDistanceA;
				float32 invMassB, invInertiaB, tangentDistanceB;
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassA, &invInertiaA, &tangentDistanceA,
					aRigid, aGroup, a,
					p, n);
				InitDampingParameterWithRigidGroupOrParticle(
					&invMassB, &invInertiaB, &tangentDistanceB,
					bRigid, bGroup, b,
					p, n);
				float32 f = damping * w * ComputeDampingImpulse(
					invMassA, invInertiaA, tangentDistanceA,
					invMassB, invInertiaB, tangentDistanceB,
					vn);
				ApplyDamping(
					invMassA, invInertiaA, tangentDistanceA,
					aRigid, aGroup, a, f, n);
				ApplyDamping(
					invMassB, invInertiaB, tangentDistanceB,
					bRigid, bGroup, b, -f, n);
			}
		}
	}
}

void b2ParticleSystem::SolveExtraDamping()
{
	// Applies additional damping force between bodies and particles which can
	// produce strong repulsive force. Applying damping force multiple times
	// is effective in suppressing vibration.
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		if (m_flagsBuffer.data[a] & k_extraDampingFlags)
		{
			b2Body* b = contact.body;
			float32 m = contact.mass;
			b2Vec2 n = contact.normal;
			b2Vec2 p = m_positionBuffer.data[a];
			b2Vec2 v =
				b->GetLinearVelocityFromWorldPoint(p) -
				m_velocityBuffer.data[a];
			float32 vn = b2Dot(v, n);
			if (vn < 0)
			{
				b2Vec2 f = 0.5f * m * vn * n;
				m_velocityBuffer.data[a] += GetParticleInvMass() * f;
				b->ApplyLinearImpulse(-f, p, true);
			}
		}
	}
}

void b2ParticleSystem::SolveWall()
{
	for (int32 i = 0; i < m_count; i++)
	{
		if (m_flagsBuffer.data[i] & b2_wallParticle)
		{
			m_velocityBuffer.data[i].SetZero();
		}
	}
}

void b2ParticleSystem::SolveRigid(const b2TimeStep& step)
{
	for (b2ParticleGroup* group = m_groupList; group; group = group->GetNext())
	{
		if (group->m_groupFlags & b2_rigidParticleGroup)
		{
			group->UpdateStatistics();
			b2Rot rotation(step.dt * group->m_angularVelocity);
			b2Transform transform(
				group->m_center + step.dt * group->m_linearVelocity -
				b2Mul(rotation, group->m_center), rotation);
			group->m_transform = b2Mul(transform, group->m_transform);
			b2Transform velocityTransform;
			velocityTransform.p.x = step.inv_dt * transform.p.x;
			velocityTransform.p.y = step.inv_dt * transform.p.y;
			velocityTransform.q.s = step.inv_dt * transform.q.s;
			velocityTransform.q.c = step.inv_dt * (transform.q.c - 1);
			for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
			{
				m_velocityBuffer.data[i] = b2Mul(velocityTransform,
												 m_positionBuffer.data[i]);
			}
		}
	}
}

void b2ParticleSystem::SolveElastic(const b2TimeStep& step)
{
	float32 elasticStrength = step.inv_dt * m_def.elasticStrength;
	for (int32 k = 0; k < m_triadBuffer.GetCount(); k++)
	{
		const b2ParticleTriad& triad = m_triadBuffer[k];
		if (triad.flags & b2_elasticParticle)
		{
			int32 a = triad.indexA;
			int32 b = triad.indexB;
			int32 c = triad.indexC;
			const b2Vec2& oa = triad.pa;
			const b2Vec2& ob = triad.pb;
			const b2Vec2& oc = triad.pc;
			b2Vec2 pa = m_positionBuffer.data[a];
			b2Vec2 pb = m_positionBuffer.data[b];
			b2Vec2 pc = m_positionBuffer.data[c];
			b2Vec2& va = m_velocityBuffer.data[a];
			b2Vec2& vb = m_velocityBuffer.data[b];
			b2Vec2& vc = m_velocityBuffer.data[c];
			pa += step.dt * va;
			pb += step.dt * vb;
			pc += step.dt * vc;
			b2Vec2 midPoint = (float32) 1 / 3 * (pa + pb + pc);
			pa -= midPoint;
			pb -= midPoint;
			pc -= midPoint;
			b2Rot r;
			r.s = b2Cross(oa, pa) + b2Cross(ob, pb) + b2Cross(oc, pc);
			r.c = b2Dot(oa, pa) + b2Dot(ob, pb) + b2Dot(oc, pc);
			float32 r2 = r.s * r.s + r.c * r.c;
			float32 invR = b2InvSqrt(r2);
			r.s *= invR;
			r.c *= invR;
			float32 strength = elasticStrength * triad.strength;
			va += strength * (b2Mul(r, oa) - pa);
			vb += strength * (b2Mul(r, ob) - pb);
			vc += strength * (b2Mul(r, oc) - pc);
		}
	}
}

void b2ParticleSystem::SolveSpring(const b2TimeStep& step)
{
	float32 springStrength = step.inv_dt * m_def.springStrength;
	for (int32 k = 0; k < m_pairBuffer.GetCount(); k++)
	{
		const b2ParticlePair& pair = m_pairBuffer[k];
		if (pair.flags & b2_springParticle)
		{
			int32 a = pair.indexA;
			int32 b = pair.indexB;
			b2Vec2 pa = m_positionBuffer.data[a];
			b2Vec2 pb = m_positionBuffer.data[b];
			b2Vec2& va = m_velocityBuffer.data[a];
			b2Vec2& vb = m_velocityBuffer.data[b];
			pa += step.dt * va;
			pb += step.dt * vb;
			b2Vec2 d = pb - pa;
			float32 r0 = pair.distance;
			float32 r1 = d.Length();
			float32 strength = springStrength * pair.strength;
			b2Vec2 f = strength * (r0 - r1) / r1 * d;
			va -= f;
			vb += f;
		}
	}
}

void b2ParticleSystem::SolveTensile(const b2TimeStep& step)
{
	b2Assert(m_accumulation2Buffer);
	for (int32 i = 0; i < m_count; i++)
	{
		m_accumulation2Buffer[i] = b2Vec2_zero;
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		if (contact.GetFlags() & b2_tensileParticle)
		{
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			float32 w = contact.GetWeight();
			b2Vec2 n = contact.GetNormal();
			b2Vec2 weightedNormal = (1 - w) * w * n;
			m_accumulation2Buffer[a] -= weightedNormal;
			m_accumulation2Buffer[b] += weightedNormal;
		}
	}
	float32 criticalVelocity = GetCriticalVelocity(step);
	float32 pressureStrength = m_def.surfaceTensionPressureStrength
							 * criticalVelocity;
	float32 normalStrength = m_def.surfaceTensionNormalStrength
						   * criticalVelocity;
	float32 maxVelocityVariation = b2_maxParticleForce * criticalVelocity;
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		if (contact.GetFlags() & b2_tensileParticle)
		{
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			float32 w = contact.GetWeight();
			b2Vec2 n = contact.GetNormal();
			float32 h = m_weightBuffer[a] + m_weightBuffer[b];
			b2Vec2 s = m_accumulation2Buffer[b] - m_accumulation2Buffer[a];
			float32 fn = b2Min(
					pressureStrength * (h - 2) + normalStrength * b2Dot(s, n),
					maxVelocityVariation) * w;
			b2Vec2 f = fn * n;
			m_velocityBuffer.data[a] -= f;
			m_velocityBuffer.data[b] += f;
		}
	}
}

void b2ParticleSystem::SolveViscous()
{
	float32 viscousStrength = m_def.viscousStrength;
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		const b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		int32 a = contact.index;
		if (m_flagsBuffer.data[a] & b2_viscousParticle)
		{
			b2Body* b = contact.body;
			float32 w = contact.weight;
			float32 m = contact.mass;
			b2Vec2 p = m_positionBuffer.data[a];
			b2Vec2 v = b->GetLinearVelocityFromWorldPoint(p) -
					   m_velocityBuffer.data[a];
			b2Vec2 f = viscousStrength * m * w * v;
			m_velocityBuffer.data[a] += GetParticleInvMass() * f;
			b->ApplyLinearImpulse(-f, p, true);
		}
	}
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		if (contact.GetFlags() & b2_viscousParticle)
		{
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			float32 w = contact.GetWeight();
			b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
			b2Vec2 f = viscousStrength * w * v;
			m_velocityBuffer.data[a] += f;
			m_velocityBuffer.data[b] -= f;
		}
	}
}

void b2ParticleSystem::SolveRepulsive(const b2TimeStep& step)
{
	float32 repulsiveStrength =
		m_def.repulsiveStrength * GetCriticalVelocity(step);
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		if (contact.GetFlags() & b2_repulsiveParticle)
		{
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			if (m_groupBuffer[a] != m_groupBuffer[b])
			{
				float32 w = contact.GetWeight();
				b2Vec2 n = contact.GetNormal();
				b2Vec2 f = repulsiveStrength * w * n;
				m_velocityBuffer.data[a] -= f;
				m_velocityBuffer.data[b] += f;
			}
		}
	}
}

void b2ParticleSystem::SolvePowder(const b2TimeStep& step)
{
	float32 powderStrength = m_def.powderStrength * GetCriticalVelocity(step);
	float32 minWeight = 1.0f - b2_particleStride;
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		if (contact.GetFlags() & b2_powderParticle)
		{
			float32 w = contact.GetWeight();
			if (w > minWeight)
			{
				int32 a = contact.GetIndexA();
				int32 b = contact.GetIndexB();
				b2Vec2 n = contact.GetNormal();
				b2Vec2 f = powderStrength * (w - minWeight) * n;
				m_velocityBuffer.data[a] -= f;
				m_velocityBuffer.data[b] += f;
			}
		}
	}
}

void b2ParticleSystem::SolveSolid(const b2TimeStep& step)
{
	// applies extra repulsive force from solid particle groups
	b2Assert(m_depthBuffer);
	float32 ejectionStrength = step.inv_dt * m_def.ejectionStrength;
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		if (m_groupBuffer[a] != m_groupBuffer[b])
		{
			float32 w = contact.GetWeight();
			b2Vec2 n = contact.GetNormal();
			float32 h = m_depthBuffer[a] + m_depthBuffer[b];
			b2Vec2 f = ejectionStrength * h * w * n;
			m_velocityBuffer.data[a] -= f;
			m_velocityBuffer.data[b] += f;
		}
	}
}

void b2ParticleSystem::SolveForce(const b2TimeStep& step)
{
	float32 velocityPerForce = step.dt * GetParticleInvMass();
	for (int32 i = 0; i < m_count; i++)
	{
		m_velocityBuffer.data[i] += velocityPerForce * m_forceBuffer[i];
	}
	m_hasForce = false;
}

void b2ParticleSystem::SolveColorMixing()
{
	// mixes color between contacting particles
	b2Assert(m_colorBuffer.data);
	const int32 colorMixing128 = (int32) (128 * m_def.colorMixingStrength);
	if (colorMixing128) {
		for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
		{
			const b2ParticleContact& contact = m_contactBuffer[k];
			int32 a = contact.GetIndexA();
			int32 b = contact.GetIndexB();
			if (m_flagsBuffer.data[a] & m_flagsBuffer.data[b] &
				b2_colorMixingParticle)
			{
				b2ParticleColor& colorA = m_colorBuffer.data[a];
				b2ParticleColor& colorB = m_colorBuffer.data[b];
				// Use the static method to ensure certain compilers inline
				// this correctly.
				b2ParticleColor::MixColors(&colorA, &colorB, colorMixing128);
			}
		}
	}
}

void b2ParticleSystem::SolveZombie()
{
	// removes particles with zombie flag
	int32 newCount = 0;
	int32* newIndices = (int32*) m_world->m_stackAllocator.Allocate(
		sizeof(int32) * m_count);
	uint32 allParticleFlags = 0;
	for (int32 i = 0; i < m_count; i++)
	{
		int32 flags = m_flagsBuffer.data[i];
		if (flags & b2_zombieParticle)
		{
			b2DestructionListener * const destructionListener =
				m_world->m_destructionListener;
			if ((flags & b2_destructionListenerParticle) &&
				destructionListener)
			{
				destructionListener->SayGoodbye(this, i);
			}
			// Destroy particle handle.
			if (m_handleIndexBuffer.data)
			{
				b2ParticleHandle * const handle = m_handleIndexBuffer.data[i];
				if (handle)
				{
					handle->SetIndex(b2_invalidParticleIndex);
					m_handleIndexBuffer.data[i] = NULL;
					m_handleAllocator.Free(handle);
				}
			}
			newIndices[i] = b2_invalidParticleIndex;
		}
		else
		{
			newIndices[i] = newCount;
			if (i != newCount)
			{
				// Update handle to reference new particle index.
				if (m_handleIndexBuffer.data)
				{
					b2ParticleHandle * const handle =
						m_handleIndexBuffer.data[i];
					if (handle) handle->SetIndex(newCount);
					m_handleIndexBuffer.data[newCount] = handle;
				}
				m_flagsBuffer.data[newCount] = m_flagsBuffer.data[i];
				if (m_lastBodyContactStepBuffer.data)
				{
					m_lastBodyContactStepBuffer.data[newCount] =
						m_lastBodyContactStepBuffer.data[i];
				}
				if (m_bodyContactCountBuffer.data)
				{
					m_bodyContactCountBuffer.data[newCount] =
						m_bodyContactCountBuffer.data[i];
				}
				if (m_consecutiveContactStepsBuffer.data)
				{
					m_consecutiveContactStepsBuffer.data[newCount] =
						m_consecutiveContactStepsBuffer.data[i];
				}
				m_positionBuffer.data[newCount] = m_positionBuffer.data[i];
				m_velocityBuffer.data[newCount] = m_velocityBuffer.data[i];
				m_groupBuffer[newCount] = m_groupBuffer[i];
				if (m_hasForce)
				{
					m_forceBuffer[newCount] = m_forceBuffer[i];
				}
				if (m_staticPressureBuffer)
				{
					m_staticPressureBuffer[newCount] =
						m_staticPressureBuffer[i];
				}
				if (m_depthBuffer)
				{
					m_depthBuffer[newCount] = m_depthBuffer[i];
				}
				if (m_colorBuffer.data)
				{
					m_colorBuffer.data[newCount] = m_colorBuffer.data[i];
				}
				if (m_userDataBuffer.data)
				{
					m_userDataBuffer.data[newCount] = m_userDataBuffer.data[i];
				}
				if (m_expirationTimeBuffer.data)
				{
					m_expirationTimeBuffer.data[newCount] =
						m_expirationTimeBuffer.data[i];
				}
			}
			newCount++;
			allParticleFlags |= flags;
		}
	}

	// predicate functions
	struct Test
	{
		static bool IsProxyInvalid(const Proxy& proxy)
		{
			return proxy.index < 0;
		}
		static bool IsContactInvalid(const b2ParticleContact& contact)
		{
			return contact.GetIndexA() < 0 || contact.GetIndexB() < 0;
		}
		static bool IsBodyContactInvalid(const b2ParticleBodyContact& contact)
		{
			return contact.index < 0;
		}
		static bool IsPairInvalid(const b2ParticlePair& pair)
		{
			return pair.indexA < 0 || pair.indexB < 0;
		}
		static bool IsTriadInvalid(const b2ParticleTriad& triad)
		{
			return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
		}
	};

	// update proxies
	for (int32 k = 0; k < m_proxyBuffer.GetCount(); k++)
	{
		Proxy& proxy = m_proxyBuffer.Begin()[k];
		proxy.index = newIndices[proxy.index];
	}
	m_proxyBuffer.RemoveIf(Test::IsProxyInvalid);

	// update contacts
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		b2ParticleContact& contact = m_contactBuffer[k];
		contact.SetIndices(newIndices[contact.GetIndexA()],
						   newIndices[contact.GetIndexB()]);
	}
	m_contactBuffer.RemoveIf(Test::IsContactInvalid);

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		contact.index = newIndices[contact.index];
	}
	m_bodyContactBuffer.RemoveIf(Test::IsBodyContactInvalid);

	// update pairs
	for (int32 k = 0; k < m_pairBuffer.GetCount(); k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		pair.indexA = newIndices[pair.indexA];
		pair.indexB = newIndices[pair.indexB];
	}
	m_pairBuffer.RemoveIf(Test::IsPairInvalid);

	// update triads
	for (int32 k = 0; k < m_triadBuffer.GetCount(); k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		triad.indexA = newIndices[triad.indexA];
		triad.indexB = newIndices[triad.indexB];
		triad.indexC = newIndices[triad.indexC];
	}
	m_triadBuffer.RemoveIf(Test::IsTriadInvalid);

	// Update lifetime indices.
	if (m_indexByExpirationTimeBuffer.data)
	{
		int32 writeOffset = 0;
		for (int32 readOffset = 0; readOffset < m_count; readOffset++)
		{
			const int32 newIndex = newIndices[
				m_indexByExpirationTimeBuffer.data[readOffset]];
			if (newIndex != b2_invalidParticleIndex)
			{
				m_indexByExpirationTimeBuffer.data[writeOffset++] = newIndex;
			}
		}
	}

	// update groups
	for (b2ParticleGroup* group = m_groupList; group; group = group->GetNext())
	{
		int32 firstIndex = newCount;
		int32 lastIndex = 0;
		bool modified = false;
		for (int32 i = group->m_firstIndex; i < group->m_lastIndex; i++)
		{
			int32 j = newIndices[i];
			if (j >= 0) {
				firstIndex = b2Min(firstIndex, j);
				lastIndex = b2Max(lastIndex, j + 1);
			} else {
				modified = true;
			}
		}
		if (firstIndex < lastIndex)
		{
			group->m_firstIndex = firstIndex;
			group->m_lastIndex = lastIndex;
			if (modified)
			{
				if (group->m_groupFlags & b2_solidParticleGroup)
				{
					SetGroupFlags(group,
								  group->m_groupFlags |
								  b2_particleGroupNeedsUpdateDepth);
				}
			}
		}
		else
		{
			group->m_firstIndex = 0;
			group->m_lastIndex = 0;
			if (!(group->m_groupFlags & b2_particleGroupCanBeEmpty))
			{
				SetGroupFlags(group,
					group->m_groupFlags | b2_particleGroupWillBeDestroyed);
			}
		}
	}

	// update particle count
	m_count = newCount;
	m_world->m_stackAllocator.Free(newIndices);
	m_allParticleFlags = allParticleFlags;
	m_needsUpdateAllParticleFlags = false;

	// destroy bodies with no particles
	for (b2ParticleGroup* group = m_groupList; group;)
	{
		b2ParticleGroup* next = group->GetNext();
		if (group->m_groupFlags & b2_particleGroupWillBeDestroyed)
		{
			DestroyParticleGroup(group);
		}
		group = next;
	}
}

/// Destroy all particles which have outlived their lifetimes set by
/// SetParticleLifetime().
void b2ParticleSystem::SolveLifetimes(const b2TimeStep& step)
{
	b2Assert(m_expirationTimeBuffer.data);
	b2Assert(m_indexByExpirationTimeBuffer.data);
	// Update the time elapsed.
	m_timeElapsed = LifetimeToExpirationTime(step.dt);
	// Get the floor (non-fractional component) of the elapsed time.
	const int32 quantizedTimeElapsed = GetQuantizedTimeElapsed();

	const int32* const expirationTimes = m_expirationTimeBuffer.data;
	int32* const expirationTimeIndices = m_indexByExpirationTimeBuffer.data;
	const int32 particleCount = GetParticleCount();
	// Sort the lifetime buffer if it's required.
	if (m_expirationTimeBufferRequiresSorting)
	{
		const ExpirationTimeComparator expirationTimeComparator(
			expirationTimes);
		std::sort(expirationTimeIndices,
				  expirationTimeIndices + particleCount,
				  expirationTimeComparator);
		m_expirationTimeBufferRequiresSorting = false;
	}

	// Destroy particles which have expired.
	for (int32 i = particleCount - 1; i >= 0; --i)
	{
		const int32 particleIndex = expirationTimeIndices[i];
		const int32 expirationTime = expirationTimes[particleIndex];
		// If no particles need to be destroyed, skip this.
		if (quantizedTimeElapsed < expirationTime || expirationTime <= 0)
		{
			break;
		}
		// Destroy this particle.
		DestroyParticle(particleIndex);
	}
}

void b2ParticleSystem::RotateBuffer(int32 start, int32 mid, int32 end)
{
	// move the particles assigned to the given group toward the end of array
	if (start == mid || mid == end)
	{
		return;
	}
	b2Assert(mid >= start && mid <= end);
	struct NewIndices
	{
		int32 operator[](int32 i) const
		{
			if (i < start)
			{
				return i;
			}
			else if (i < mid)
			{
				return i + end - mid;
			}
			else if (i < end)
			{
				return i + start - mid;
			}
			else
			{
				return i;
			}
		}
		int32 start, mid, end;
	} newIndices;
	newIndices.start = start;
	newIndices.mid = mid;
	newIndices.end = end;

	std::rotate(m_flagsBuffer.data + start, m_flagsBuffer.data + mid,
				m_flagsBuffer.data + end);
	if (m_lastBodyContactStepBuffer.data)
	{
		std::rotate(m_lastBodyContactStepBuffer.data + start,
					m_lastBodyContactStepBuffer.data + mid,
					m_lastBodyContactStepBuffer.data + end);
	}
	if (m_bodyContactCountBuffer.data)
	{
		std::rotate(m_bodyContactCountBuffer.data + start,
					m_bodyContactCountBuffer.data + mid,
					m_bodyContactCountBuffer.data + end);
	}
	if (m_consecutiveContactStepsBuffer.data)
	{
		std::rotate(m_consecutiveContactStepsBuffer.data + start,
					m_consecutiveContactStepsBuffer.data + mid,
					m_consecutiveContactStepsBuffer.data + end);
	}
	std::rotate(m_positionBuffer.data + start, m_positionBuffer.data + mid,
				m_positionBuffer.data + end);
	std::rotate(m_velocityBuffer.data + start, m_velocityBuffer.data + mid,
				m_velocityBuffer.data + end);
	std::rotate(m_groupBuffer + start, m_groupBuffer + mid,
				m_groupBuffer + end);
	if (m_hasForce)
	{
		std::rotate(m_forceBuffer + start, m_forceBuffer + mid,
					m_forceBuffer + end);
	}
	if (m_staticPressureBuffer)
	{
		std::rotate(m_staticPressureBuffer + start,
					m_staticPressureBuffer + mid,
					m_staticPressureBuffer + end);
	}
	if (m_depthBuffer)
	{
		std::rotate(m_depthBuffer + start, m_depthBuffer + mid,
					m_depthBuffer + end);
	}
	if (m_colorBuffer.data)
	{
		std::rotate(m_colorBuffer.data + start,
					m_colorBuffer.data + mid, m_colorBuffer.data + end);
	}
	if (m_userDataBuffer.data)
	{
		std::rotate(m_userDataBuffer.data + start,
					m_userDataBuffer.data + mid, m_userDataBuffer.data + end);
	}

	// Update handle indices.
	if (m_handleIndexBuffer.data)
	{
		std::rotate(m_handleIndexBuffer.data + start,
					m_handleIndexBuffer.data + mid,
					m_handleIndexBuffer.data + end);
		for (int32 i = start; i < end; ++i)
		{
			b2ParticleHandle * const handle = m_handleIndexBuffer.data[i];
			if (handle) handle->SetIndex(newIndices[handle->GetIndex()]);
		}
	}

	if (m_expirationTimeBuffer.data)
	{
		std::rotate(m_expirationTimeBuffer.data + start,
					m_expirationTimeBuffer.data + mid,
					m_expirationTimeBuffer.data + end);
		// Update expiration time buffer indices.
		const int32 particleCount = GetParticleCount();
		int32* const indexByExpirationTime =
			m_indexByExpirationTimeBuffer.data;
		for (int32 i = 0; i < particleCount; ++i)
		{
			indexByExpirationTime[i] = newIndices[indexByExpirationTime[i]];
		}
	}

	// update proxies
	for (int32 k = 0; k < m_proxyBuffer.GetCount(); k++)
	{
		Proxy& proxy = m_proxyBuffer.Begin()[k];
		proxy.index = newIndices[proxy.index];
	}

	// update contacts
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		b2ParticleContact& contact = m_contactBuffer[k];
		contact.SetIndices(newIndices[contact.GetIndexA()],
						   newIndices[contact.GetIndexB()]);
	}

	// update particle-body contacts
	for (int32 k = 0; k < m_bodyContactBuffer.GetCount(); k++)
	{
		b2ParticleBodyContact& contact = m_bodyContactBuffer[k];
		contact.index = newIndices[contact.index];
	}

	// update pairs
	for (int32 k = 0; k < m_pairBuffer.GetCount(); k++)
	{
		b2ParticlePair& pair = m_pairBuffer[k];
		pair.indexA = newIndices[pair.indexA];
		pair.indexB = newIndices[pair.indexB];
	}

	// update triads
	for (int32 k = 0; k < m_triadBuffer.GetCount(); k++)
	{
		b2ParticleTriad& triad = m_triadBuffer[k];
		triad.indexA = newIndices[triad.indexA];
		triad.indexB = newIndices[triad.indexB];
		triad.indexC = newIndices[triad.indexC];
	}

	// update groups
	for (b2ParticleGroup* group = m_groupList; group; group = group->GetNext())
	{
		group->m_firstIndex = newIndices[group->m_firstIndex];
		group->m_lastIndex = newIndices[group->m_lastIndex - 1] + 1;
	}
}

/// Set the lifetime (in seconds) of a particle relative to the current
/// time.
void b2ParticleSystem::SetParticleLifetime(const int32 index,
										   const float32 lifetime)
{
	b2Assert(ValidateParticleIndex(index));
	const bool initializeExpirationTimes =
		m_indexByExpirationTimeBuffer.data == NULL;
	m_expirationTimeBuffer.data = RequestBuffer(
		m_expirationTimeBuffer.data);
	m_indexByExpirationTimeBuffer.data = RequestBuffer(
		m_indexByExpirationTimeBuffer.data);

	// Initialize the inverse mapping buffer.
	if (initializeExpirationTimes)
	{
		const int32 particleCount = GetParticleCount();
		for (int32 i = 0; i < particleCount; ++i)
		{
			m_indexByExpirationTimeBuffer.data[i] = i;
		}
	}
	const int32 quantizedLifetime = (int32)(lifetime /
											m_def.lifetimeGranularity);
	// Use a negative lifetime so that it's possible to track which
	// of the infinite lifetime particles are older.
	const int32 newExpirationTime = quantizedLifetime > 0 ?
		GetQuantizedTimeElapsed() + quantizedLifetime : quantizedLifetime;
	if (newExpirationTime != m_expirationTimeBuffer.data[index])
	{
		m_expirationTimeBuffer.data[index] = newExpirationTime;
		m_expirationTimeBufferRequiresSorting = true;
	}
}


/// Convert a lifetime value in returned by GetExpirationTimeBuffer()
/// to a value in seconds relative to the current simulation time.
float32 b2ParticleSystem::ExpirationTimeToLifetime(
	const int32 expirationTime) const
{
	return (float32)(expirationTime > 0 ?
					 	expirationTime - GetQuantizedTimeElapsed() :
					 	expirationTime) * m_def.lifetimeGranularity;
}

/// Get the lifetime (in seconds) of a particle relative to the current
/// time.
float32 b2ParticleSystem::GetParticleLifetime(const int32 index)
{
	b2Assert(ValidateParticleIndex(index));
	return ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]);
}

/// Get the array of particle lifetimes indexed by particle index.
/// GetParticleCount() items are in the returned array.
const int32* b2ParticleSystem::GetExpirationTimeBuffer()
{
	m_expirationTimeBuffer.data = RequestBuffer(
		m_expirationTimeBuffer.data);
	return m_expirationTimeBuffer.data;
}

/// Get the array of particle indices ordered by lifetime.
/// GetExpirationTimeBuffer(
///    GetIndexByExpirationTimeBuffer()[index])
/// is equivalent to GetParticleLifetime(index).
/// GetParticleCount() items are in the returned array.
const int32* b2ParticleSystem::GetIndexByExpirationTimeBuffer()
{
	// If particles are present, initialize / reinitialize the lifetime buffer.
	if (GetParticleCount())
	{
		SetParticleLifetime(0, GetParticleLifetime(0));
	}
	else
	{
		m_indexByExpirationTimeBuffer.data = RequestBuffer(
			m_indexByExpirationTimeBuffer.data);
	}
	return m_indexByExpirationTimeBuffer.data;
}

void b2ParticleSystem::SetDestructionByAge(const bool enable)
{
	if (enable)
	{
		GetExpirationTimeBuffer();
	}
	m_def.destroyByAge = enable;
}

/// Get the time elapsed in b2ParticleSystemDef::lifetimeGranularity.
int32 b2ParticleSystem::GetQuantizedTimeElapsed() const
{
	return (int32)(m_timeElapsed >> 32);
}

/// Convert a lifetime in seconds to an expiration time.
int64 b2ParticleSystem::LifetimeToExpirationTime(const float32 lifetime) const
{
	return m_timeElapsed + (int64)((lifetime / m_def.lifetimeGranularity) *
								   (float32)(1LL << 32));
}

template <typename T> void b2ParticleSystem::SetUserOverridableBuffer(
	UserOverridableBuffer<T>* buffer, T* newData, int32 newCapacity)
{
	b2Assert((newData && newCapacity) || (!newData && !newCapacity));
	if (!buffer->userSuppliedCapacity && buffer->data)
	{
		m_world->m_blockAllocator.Free(
			buffer->data, sizeof(T) * m_internalAllocatedCapacity);
	}
	buffer->data = newData;
	buffer->userSuppliedCapacity = newCapacity;
}

void b2ParticleSystem::SetFlagsBuffer(uint32* buffer, int32 capacity)
{
	SetUserOverridableBuffer(&m_flagsBuffer, buffer, capacity);
}

void b2ParticleSystem::SetPositionBuffer(b2Vec2* buffer,
												 int32 capacity)
{
	SetUserOverridableBuffer(&m_positionBuffer, buffer, capacity);
}

void b2ParticleSystem::SetVelocityBuffer(b2Vec2* buffer,
												 int32 capacity)
{
	SetUserOverridableBuffer(&m_velocityBuffer, buffer, capacity);
}

void b2ParticleSystem::SetColorBuffer(b2ParticleColor* buffer,
											  int32 capacity)
{
	SetUserOverridableBuffer(&m_colorBuffer, buffer, capacity);
}

void b2ParticleSystem::SetUserDataBuffer(void** buffer, int32 capacity)
{
	SetUserOverridableBuffer(&m_userDataBuffer, buffer, capacity);
}

void b2ParticleSystem::SetParticleFlags(int32 index, uint32 newFlags)
{
	uint32* oldFlags = &m_flagsBuffer.data[index];
	if (*oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllParticleFlags = true;
	}
	if (~m_allParticleFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_tensileParticle)
		{
			m_accumulation2Buffer = RequestBuffer(
				m_accumulation2Buffer);
		}
		if (newFlags & b2_colorMixingParticle)
		{
			m_colorBuffer.data = RequestBuffer(m_colorBuffer.data);
		}
		m_allParticleFlags |= newFlags;
	}
	*oldFlags = newFlags;
}

void b2ParticleSystem::SetGroupFlags(
	b2ParticleGroup* group, uint32 newFlags)
{
	uint32* oldFlags = &group->m_groupFlags;
	if ((*oldFlags ^ newFlags) & b2_solidParticleGroup)
	{
		// If the b2_solidParticleGroup flag changed schedule depth update.
		newFlags |= b2_particleGroupNeedsUpdateDepth;
	}
	if (*oldFlags & ~newFlags)
	{
		// If any flags might be removed
		m_needsUpdateAllGroupFlags = true;
	}
	if (~m_allGroupFlags & newFlags)
	{
		// If any flags were added
		if (newFlags & b2_solidParticleGroup)
		{
			m_depthBuffer = RequestBuffer(m_depthBuffer);
		}
		m_allGroupFlags |= newFlags;
	}
	*oldFlags = newFlags;
}

static inline bool IsSignificantForce(const b2Vec2& force)
{
	return force.x != 0 || force.y != 0;
}

inline bool b2ParticleSystem::ForceCanBeApplied(uint32 flags) const
{
	return !(flags & b2_wallParticle);
}

inline void b2ParticleSystem::PrepareForceBuffer()
{
	if (!m_hasForce)
	{
		memset(m_forceBuffer, 0, sizeof(*m_forceBuffer) * m_count);
		m_hasForce = true;
	}
}

void b2ParticleSystem::ApplyForce(int32 firstIndex, int32 lastIndex,
								  const b2Vec2& force)
{
	// Ensure we're not trying to apply force to particles that can't move,
	// such as wall particles.
#if B2_ASSERT_ENABLED
	uint32 flags = 0;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		flags |= m_flagsBuffer.data[i];
	}
	b2Assert(ForceCanBeApplied(flags));
#endif

	// Early out if force does nothing (optimization).
	const b2Vec2 distributedForce = force / (float32)(lastIndex - firstIndex);
	if (IsSignificantForce(distributedForce))
	{
		PrepareForceBuffer();

		// Distribute the force over all the particles.
		for (int32 i = firstIndex; i < lastIndex; i++)
		{
			m_forceBuffer[i] += distributedForce;
		}
	}
}

void b2ParticleSystem::ParticleApplyForce(int32 index, const b2Vec2& force)
{
	if (IsSignificantForce(force) &&
		ForceCanBeApplied(m_flagsBuffer.data[index]))
	{
		PrepareForceBuffer();
		m_forceBuffer[index] += force;
	}
}

void b2ParticleSystem::ApplyLinearImpulse(int32 firstIndex, int32 lastIndex,
										  const b2Vec2& impulse)
{
	const float32 numParticles = (float32)(lastIndex - firstIndex);
	const float32 totalMass = numParticles * GetParticleMass();
	const b2Vec2 velocityDelta = impulse / totalMass;
	for (int32 i = firstIndex; i < lastIndex; i++)
	{
		m_velocityBuffer.data[i] += velocityDelta;
	}
}

void b2ParticleSystem::QueryAABB(b2QueryCallback* callback,
								 const b2AABB& aabb) const
{
	if (m_proxyBuffer.GetCount() == 0)
	{
		return;
	}
	const Proxy* beginProxy = m_proxyBuffer.Begin();
	const Proxy* endProxy = m_proxyBuffer.End();
	const Proxy* firstProxy = std::lower_bound(
		beginProxy, endProxy,
		computeTag(
			m_inverseDiameter * aabb.lowerBound.x,
			m_inverseDiameter * aabb.lowerBound.y));
	const Proxy* lastProxy = std::upper_bound(
		firstProxy, endProxy,
		computeTag(
			m_inverseDiameter * aabb.upperBound.x,
			m_inverseDiameter * aabb.upperBound.y));
	for (const Proxy* proxy = firstProxy; proxy < lastProxy; ++proxy)
	{
		int32 i = proxy->index;
		const b2Vec2& p = m_positionBuffer.data[i];
		if (aabb.lowerBound.x < p.x && p.x < aabb.upperBound.x &&
			aabb.lowerBound.y < p.y && p.y < aabb.upperBound.y)
		{
			if (!callback->ReportParticle(this, i))
			{
				break;
			}
		}
	}
}

void b2ParticleSystem::QueryShapeAABB(b2QueryCallback* callback,
									  const b2Shape& shape,
									  const b2Transform& xf) const
{
	b2AABB aabb;
	shape.ComputeAABB(&aabb, xf, 0);
	QueryAABB(callback, aabb);
}

void b2ParticleSystem::RayCast(b2RayCastCallback* callback,
							   const b2Vec2& point1,
							   const b2Vec2& point2) const
{
	if (m_proxyBuffer.GetCount() == 0)
	{
		return;
	}
	b2AABB aabb;
	aabb.lowerBound = b2Min(point1, point2);
	aabb.upperBound = b2Max(point1, point2);
	float32 fraction = 1;
	// solving the following equation:
	// ((1-t)*point1+t*point2-position)^2=diameter^2
	// where t is a potential fraction
	b2Vec2 v = point2 - point1;
	float32 v2 = b2Dot(v, v);
	InsideBoundsEnumerator enumerator = GetInsideBoundsEnumerator(aabb);
	int32 i;
	while ((i = enumerator.GetNext()) >= 0)
	{
		b2Vec2 p = point1 - m_positionBuffer.data[i];
		float32 pv = b2Dot(p, v);
		float32 p2 = b2Dot(p, p);
		float32 determinant = pv * pv - v2 * (p2 - m_squaredDiameter);
		if (determinant >= 0)
		{
			float32 sqrtDeterminant = b2Sqrt(determinant);
			// find a solution between 0 and fraction
			float32 t = (-pv - sqrtDeterminant) / v2;
			if (t > fraction)
			{
				continue;
			}
			if (t < 0)
			{
				t = (-pv + sqrtDeterminant) / v2;
				if (t < 0 || t > fraction)
				{
					continue;
				}
			}
			b2Vec2 n = p + t * v;
			n.Normalize();
			float32 f = callback->ReportParticle(this, i, point1 + t * v, n, t);
			fraction = b2Min(fraction, f);
			if (fraction <= 0)
			{
				break;
			}
		}
	}
}

float32 b2ParticleSystem::ComputeCollisionEnergy() const
{
	float32 sum_v2 = 0;
	for (int32 k = 0; k < m_contactBuffer.GetCount(); k++)
	{
		const b2ParticleContact& contact = m_contactBuffer[k];
		int32 a = contact.GetIndexA();
		int32 b = contact.GetIndexB();
		b2Vec2 n = contact.GetNormal();
		b2Vec2 v = m_velocityBuffer.data[b] - m_velocityBuffer.data[a];
		float32 vn = b2Dot(v, n);
		if (vn < 0)
		{
			sum_v2 += vn * vn;
		}
	}
	return 0.5f * GetParticleMass() * sum_v2;
}

void b2ParticleSystem::SetStuckThreshold(int32 steps)
{
	m_stuckThreshold = steps;

	if (steps > 0)
	{
		m_lastBodyContactStepBuffer.data = RequestBuffer(
			m_lastBodyContactStepBuffer.data);
		m_bodyContactCountBuffer.data = RequestBuffer(
			m_bodyContactCountBuffer.data);
		m_consecutiveContactStepsBuffer.data = RequestBuffer(
			m_consecutiveContactStepsBuffer.data);
	}
}

#if BOX2D_EXTERNAL_LANGUAGE_API

b2ParticleSystem::b2ExceptionType b2ParticleSystem::IsBufCopyValid(
	int startIndex, int numParticles, int copySize, int bufSize) const
{
	const int maxNumParticles = GetParticleCount();

	// are we actually copying?
	if (copySize == 0)
	{
		return b2_noExceptions;
	}

	// is the index out of bounds?
	if (startIndex < 0 ||
		startIndex >= maxNumParticles ||
		numParticles < 0 ||
		numParticles + startIndex > maxNumParticles)
	{
		return b2_particleIndexOutOfBounds;
	}

	// are we copying within the boundaries?
	if (copySize > bufSize)
	{
		return b2_bufferTooSmall;
	}

	return b2_noExceptions;
}

#endif // BOX2D_EXTERNAL_LANGUAGE_API
