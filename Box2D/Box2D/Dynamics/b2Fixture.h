/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

class b2BlockAllocator;
class b2Body;
class b2BroadPhase;

/// This holds contact filtering data.
struct b2Filter
{
	/// The collision category bits. Normally you would just set one bit.
	uint16 categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint16 maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	int16 groupIndex;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	b2FixtureDef()
	{
		shape = NULL;
		userData = NULL;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		filter.categoryBits = 0x0001;
		filter.maskBits = 0xFFFF;
		filter.groupIndex = 0;
		isSensor = false;
	}

	virtual ~b2FixtureDef() {}

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const b2Shape* shape;

	/// Use this to store application specific fixture data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float32 friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float32 restitution;

	/// The density, usually in kg/m^2.
	float32 density;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor;

	/// Contact filtering data.
	b2Filter filter;
};


/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class b2Fixture
{
public:
	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	b2Shape::Type GetType() const;

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	const b2Shape* GetShape() const;
	b2Shape* GetShape();

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	bool IsSensor() const;

	/// Set if this fixture is a sensor.
	void SetSensor(bool sensor);

	/// Set the contact filtering data. This is an expensive operation and should
	/// not be called frequently. This will not update contacts until the next time
	/// step when either parent body is awake.
	void SetFilterData(const b2Filter& filter);

	/// Get the contact filtering data.
	const b2Filter& GetFilterData() const;

	/// Get the parent body of this fixture. This is NULL if the fixture is not attached.
	/// @return the parent body.
	b2Body* GetBody();

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	b2Fixture* GetNext();

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData();

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Test a point for containment in this fixture. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	bool TestPoint(const b2Vec2& p) const;

	/// Perform a ray cast against this shape.
	/// @param xf the shape world transform.
	/// @param lambda returns the hit fraction. You can use this to compute the contact point
	/// p = (1 - lambda) * segment.p1 + lambda * segment.p2.
	/// @param normal returns the normal at the contact point. If there is no intersection, the normal
	/// is not set.
	/// @param segment defines the begin and end point of the ray cast.
	/// @param maxLambda a number typically in the range [0,1].
	b2SegmentCollide TestSegment(float32* lambda, b2Vec2* normal, const b2Segment& segment, float32 maxLambda) const;

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin, not the centroid.
	/// @param massData returns the mass data for this shape.
	void ComputeMass(b2MassData* massData) const;

	/// Get the coefficient of friction.
	float32 GetFriction() const;

	/// Set the coefficient of friction.
	void SetFriction(float32 friction);

	/// Get the coefficient of restitution.
	float32 GetRestitution() const;

	/// Set the coefficient of restitution.
	void SetRestitution(float32 restitution);

	/// Get the density.
	float32 GetDensity() const;

	/// Set the density.
	/// @warning this does not automatically update the mass of the parent body.
	void SetDensity(float32 density);

protected:

	friend class b2Body;
	friend class b2World;
	friend class b2Contact;
	friend class b2ContactManager;

	b2Fixture();
	~b2Fixture();

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator or broad-phase (no destructor arguments allowed by C++).
	void Create(b2BlockAllocator* allocator, b2BroadPhase* broadPhase, b2Body* body, const b2Transform& xf, const b2FixtureDef* def);
	void Destroy(b2BlockAllocator* allocator, b2BroadPhase* broadPhase);

	void Synchronize(b2BroadPhase* broadPhase, const b2Transform& xf1, const b2Transform& xf2);

	b2AABB m_aabb;

	b2Fixture* m_next;
	b2Body* m_body;

	b2Shape* m_shape;

	float32 m_density;
	float32 m_friction;
	float32 m_restitution;

	int32 m_proxyId;
	b2Filter m_filter;

	bool m_isSensor;

	void* m_userData;
};

inline b2Shape::Type b2Fixture::GetType() const
{
	return m_shape->GetType();
}

inline const b2Shape* b2Fixture::GetShape() const
{
	return m_shape;
}

inline b2Shape* b2Fixture::GetShape()
{
	return m_shape;
}

inline bool b2Fixture::IsSensor() const
{
	return m_isSensor;
}

inline const b2Filter& b2Fixture::GetFilterData() const
{
	return m_filter;
}

inline void* b2Fixture::GetUserData()
{
	return m_userData;
}

inline void b2Fixture::SetUserData(void* data)
{
	m_userData = data;
}

inline b2Body* b2Fixture::GetBody()
{
	return m_body;
}

inline b2Fixture* b2Fixture::GetNext()
{
	return m_next;
}

inline float32 b2Fixture::GetFriction() const
{
	return m_friction;
}

inline void b2Fixture::SetFriction(float32 friction)
{
	m_friction = friction;
}

inline float32 b2Fixture::GetRestitution() const
{
	return m_restitution;
}

inline void b2Fixture::SetRestitution(float32 restitution)
{
	m_restitution = restitution;
}

inline float32 b2Fixture::GetDensity() const
{
	return m_density;
}

inline void b2Fixture::SetDensity(float32 density)
{
	m_density = density;
}

inline void b2Fixture::ComputeMass(b2MassData* massData) const
{
	m_shape->ComputeMass(massData, m_density);
}

inline bool b2Fixture::TestPoint(const b2Vec2& p) const
{
	return m_shape->TestPoint(m_body->GetTransform(), p);
}

inline b2SegmentCollide
b2Fixture::TestSegment(	float32* lambda,
						 b2Vec2* normal,
						 const b2Segment& segment,
						 float32 maxLambda) const
{
	return m_shape->TestSegment(m_body->GetTransform(), lambda, normal, segment, maxLambda);
}

#endif
