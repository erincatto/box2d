// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "collision.h"
#include "id.h"
#include "types.h"

#include <stdbool.h>

/**
 * @defgroup world World
 * These functions allow you to create a simulation world.
 *
 * You can add rigid bodies and joint constraints to the world and run the simulation. You can get contact
 * information to get contact points and normals as well as events. You can query to world, checking for overlaps and casting rays
 * or shapes. There is also debugging information such as debug draw, timing information, and counters. You can find documentation
 * here: https://box2d.org/
 * @{
 */

/// Create a world for rigid body simulation. A world contains bodies, shapes, and constraints. You make create
/// up to 128 worlds. Each world is completely independent and may be simulated in parallel.
/// @return the world id.
B2_API b2WorldId b2CreateWorld( const b2WorldDef* def );

/// Destroy a world
B2_API void b2DestroyWorld( b2WorldId worldId );

/// World id validation. Provides validation for up to 64K allocations.
B2_API bool b2World_IsValid( b2WorldId id );

/// Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
/// @param worldId The world to simulate
/// @param timeStep The amount of time to simulate, this should be a fixed number. Usually 1/60.
/// @param subStepCount The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.
B2_API void b2World_Step( b2WorldId worldId, float timeStep, int subStepCount );

/// Call this to draw shapes and other debug draw data
B2_API void b2World_Draw( b2WorldId worldId, b2DebugDraw* draw );

/// Get the body events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2BodyEvents b2World_GetBodyEvents( b2WorldId worldId );

/// Get sensor events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2SensorEvents b2World_GetSensorEvents( b2WorldId worldId );

/// Get contact events for this current time step. The event data is transient. Do not store a reference to this data.
B2_API b2ContactEvents b2World_GetContactEvents( b2WorldId worldId );

/// Get the joint events for the current time step. The event data is transient. Do not store a reference to this data.
B2_API b2JointEvents b2World_GetJointEvents( b2WorldId worldId );

/// Overlap test for all shapes that *potentially* overlap the provided AABB
B2_API b2TreeStats b2World_OverlapAABB( b2WorldId worldId, b2AABB aabb, b2QueryFilter filter, b2OverlapResultFcn* fcn,
										void* context );

/// Overlap test for all shapes that overlap the provided shape proxy.
B2_API b2TreeStats b2World_OverlapShape( b2WorldId worldId, const b2ShapeProxy* proxy, b2QueryFilter filter,
										 b2OverlapResultFcn* fcn, void* context );

/// Cast a ray into the world to collect shapes in the path of the ray.
/// Your callback function controls whether you get the closest point, any point, or n-points.
/// @note The callback function may receive shapes in any order
/// @param worldId The world to cast the ray against
/// @param origin The start point of the ray
/// @param translation The translation of the ray from the start point to the end point
/// @param filter Contains bit flags to filter unwanted shapes from the results
/// @param fcn A user implemented callback function
/// @param context A user context that is passed along to the callback function
///	@return traversal performance counters
B2_API b2TreeStats b2World_CastRay( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter,
									b2CastResultFcn* fcn, void* context );

/// Cast a ray into the world to collect the closest hit. This is a convenience function. Ignores initial overlap.
/// This is less general than b2World_CastRay() and does not allow for custom filtering.
B2_API b2RayResult b2World_CastRayClosest( b2WorldId worldId, b2Vec2 origin, b2Vec2 translation, b2QueryFilter filter );

/// Cast a shape through the world. Similar to a cast ray except that a shape is cast instead of a point.
///	@see b2World_CastRay
B2_API b2TreeStats b2World_CastShape( b2WorldId worldId, const b2ShapeProxy* proxy, b2Vec2 translation, b2QueryFilter filter,
									  b2CastResultFcn* fcn, void* context );

/// Cast a capsule mover through the world. This is a special shape cast that handles sliding along other shapes while reducing
/// clipping.
B2_API float b2World_CastMover( b2WorldId worldId, const b2Capsule* mover, b2Vec2 translation, b2QueryFilter filter );

/// Collide a capsule mover with the world, gathering collision planes that can be fed to b2SolvePlanes. Useful for
/// kinematic character movement.
B2_API void b2World_CollideMover( b2WorldId worldId, const b2Capsule* mover, b2QueryFilter filter, b2PlaneResultFcn* fcn,
								  void* context );

/// Enable/disable sleep. If your application does not need sleeping, you can gain some performance
/// by disabling sleep completely at the world level.
/// @see b2WorldDef
B2_API void b2World_EnableSleeping( b2WorldId worldId, bool flag );

/// Is body sleeping enabled?
B2_API bool b2World_IsSleepingEnabled( b2WorldId worldId );

/// Enable/disable continuous collision between dynamic and static bodies. Generally you should keep continuous
/// collision enabled to prevent fast moving objects from going through static objects. The performance gain from
/// disabling continuous collision is minor.
/// @see b2WorldDef
B2_API void b2World_EnableContinuous( b2WorldId worldId, bool flag );

/// Is continuous collision enabled?
B2_API bool b2World_IsContinuousEnabled( b2WorldId worldId );

/// Adjust the restitution threshold. It is recommended not to make this value very small
/// because it will prevent bodies from sleeping. Usually in meters per second.
/// @see b2WorldDef
B2_API void b2World_SetRestitutionThreshold( b2WorldId worldId, float value );

/// Get the the restitution speed threshold. Usually in meters per second.
B2_API float b2World_GetRestitutionThreshold( b2WorldId worldId );

/// Adjust the hit event threshold. This controls the collision speed needed to generate a b2ContactHitEvent.
/// Usually in meters per second.
/// @see b2WorldDef::hitEventThreshold
B2_API void b2World_SetHitEventThreshold( b2WorldId worldId, float value );

/// Get the the hit event speed threshold. Usually in meters per second.
B2_API float b2World_GetHitEventThreshold( b2WorldId worldId );

/// Register the custom filter callback. This is optional.
B2_API void b2World_SetCustomFilterCallback( b2WorldId worldId, b2CustomFilterFcn* fcn, void* context );

/// Register the pre-solve callback. This is optional.
B2_API void b2World_SetPreSolveCallback( b2WorldId worldId, b2PreSolveFcn* fcn, void* context );

/// Set the gravity vector for the entire world. Box2D has no concept of an up direction and this
/// is left as a decision for the application. Usually in m/s^2.
/// @see b2WorldDef
B2_API void b2World_SetGravity( b2WorldId worldId, b2Vec2 gravity );

/// Get the gravity vector
B2_API b2Vec2 b2World_GetGravity( b2WorldId worldId );

/// Apply a radial explosion
/// @param worldId The world id
/// @param explosionDef The explosion definition
B2_API void b2World_Explode( b2WorldId worldId, const b2ExplosionDef* explosionDef );

/// Adjust contact tuning parameters
/// @param worldId The world id
/// @param hertz The contact stiffness (cycles per second)
/// @param dampingRatio The contact bounciness with 1 being critical damping (non-dimensional)
/// @param pushSpeed The maximum contact constraint push out speed (meters per second)
/// @note Advanced feature
B2_API void b2World_SetContactTuning( b2WorldId worldId, float hertz, float dampingRatio, float pushSpeed );

/// Set the maximum linear speed. Usually in m/s.
B2_API void b2World_SetMaximumLinearSpeed( b2WorldId worldId, float maximumLinearSpeed );

/// Get the maximum linear speed. Usually in m/s.
B2_API float b2World_GetMaximumLinearSpeed( b2WorldId worldId );

/// Enable/disable constraint warm starting. Advanced feature for testing. Disabling
/// warm starting greatly reduces stability and provides no performance gain.
B2_API void b2World_EnableWarmStarting( b2WorldId worldId, bool flag );

/// Is constraint warm starting enabled?
B2_API bool b2World_IsWarmStartingEnabled( b2WorldId worldId );

/// Get the number of awake bodies.
B2_API int b2World_GetAwakeBodyCount( b2WorldId worldId );

/// Get the current world performance profile
B2_API b2Profile b2World_GetProfile( b2WorldId worldId );

/// Get world counters and sizes
B2_API b2Counters b2World_GetCounters( b2WorldId worldId );

/// Set the user data pointer.
B2_API void b2World_SetUserData( b2WorldId worldId, void* userData );

/// Get the user data pointer.
B2_API void* b2World_GetUserData( b2WorldId worldId );

/// Set the friction callback. Passing NULL resets to default.
B2_API void b2World_SetFrictionCallback( b2WorldId worldId, b2FrictionCallback* callback );

/// Set the restitution callback. Passing NULL resets to default.
B2_API void b2World_SetRestitutionCallback( b2WorldId worldId, b2RestitutionCallback* callback );

/// Dump memory stats to box2d_memory.txt
B2_API void b2World_DumpMemoryStats( b2WorldId worldId );

/// This is for internal testing
B2_API void b2World_RebuildStaticTree( b2WorldId worldId );

/// This is for internal testing
B2_API void b2World_EnableSpeculative( b2WorldId worldId, bool flag );

/** @} */

/**
 * @defgroup body Body
 * This is the body API.
 * @{
 */

/// Create a rigid body given a definition. No reference to the definition is retained. So you can create the definition
/// on the stack and pass it as a pointer.
/// @code{.c}
/// b2BodyDef bodyDef = b2DefaultBodyDef();
/// b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
/// @endcode
/// @warning This function is locked during callbacks.
B2_API b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def );

/// Destroy a rigid body given an id. This destroys all shapes and joints attached to the body.
/// Do not keep references to the associated shapes and joints.
B2_API void b2DestroyBody( b2BodyId bodyId );

/// Body identifier validation. A valid body exists in a world and is non-null.
/// This can be used to detect orphaned ids. Provides validation for up to 64K allocations.
B2_API bool b2Body_IsValid( b2BodyId id );

/// Get the body type: static, kinematic, or dynamic
B2_API b2BodyType b2Body_GetType( b2BodyId bodyId );

/// Change the body type. This is an expensive operation. This automatically updates the mass
/// properties regardless of the automatic mass setting.
B2_API void b2Body_SetType( b2BodyId bodyId, b2BodyType type );

/// Set the body name. Up to 31 characters excluding 0 termination.
B2_API void b2Body_SetName( b2BodyId bodyId, const char* name );

/// Get the body name.
B2_API const char* b2Body_GetName( b2BodyId bodyId );

/// Set the user data for a body
B2_API void b2Body_SetUserData( b2BodyId bodyId, void* userData );

/// Get the user data stored in a body
B2_API void* b2Body_GetUserData( b2BodyId bodyId );

/// Get the world position of a body. This is the location of the body origin.
B2_API b2Vec2 b2Body_GetPosition( b2BodyId bodyId );

/// Get the world rotation of a body as a cosine/sine pair (complex number)
B2_API b2Rot b2Body_GetRotation( b2BodyId bodyId );

/// Get the world transform of a body.
B2_API b2Transform b2Body_GetTransform( b2BodyId bodyId );

/// Set the world transform of a body. This acts as a teleport and is fairly expensive.
/// @note Generally you should create a body with then intended transform.
/// @see b2BodyDef::position and b2BodyDef::rotation
B2_API void b2Body_SetTransform( b2BodyId bodyId, b2Vec2 position, b2Rot rotation );

/// Get a local point on a body given a world point
B2_API b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Vec2 worldPoint );

/// Get a world point on a body given a local point
B2_API b2Vec2 b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint );

/// Get a local vector on a body given a world vector
B2_API b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector );

/// Get a world vector on a body given a local vector
B2_API b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector );

/// Get the linear velocity of a body's center of mass. Usually in meters per second.
B2_API b2Vec2 b2Body_GetLinearVelocity( b2BodyId bodyId );

/// Get the angular velocity of a body in radians per second
B2_API float b2Body_GetAngularVelocity( b2BodyId bodyId );

/// Set the linear velocity of a body. Usually in meters per second.
B2_API void b2Body_SetLinearVelocity( b2BodyId bodyId, b2Vec2 linearVelocity );

/// Set the angular velocity of a body in radians per second
B2_API void b2Body_SetAngularVelocity( b2BodyId bodyId, float angularVelocity );

/// Set the velocity to reach the given transform after a given time step.
/// The result will be close but maybe not exact. This is meant for kinematic bodies.
/// The target is not applied if the velocity would be below the sleep threshold.
/// This will automatically wake the body if asleep.
B2_API void b2Body_SetTargetTransform( b2BodyId bodyId, b2Transform target, float timeStep );

/// Get the linear velocity of a local point attached to a body. Usually in meters per second.
B2_API b2Vec2 b2Body_GetLocalPointVelocity( b2BodyId bodyId, b2Vec2 localPoint );

/// Get the linear velocity of a world point attached to a body. Usually in meters per second.
B2_API b2Vec2 b2Body_GetWorldPointVelocity( b2BodyId bodyId, b2Vec2 worldPoint );

/// Apply a force at a world point. If the force is not applied at the center of mass,
/// it will generate a torque and affect the angular velocity. This optionally wakes up the body.
/// The force is ignored if the body is not awake.
/// @param bodyId The body id
/// @param force The world force vector, usually in newtons (N)
/// @param point The world position of the point of application
/// @param wake Option to wake up the body
B2_API void b2Body_ApplyForce( b2BodyId bodyId, b2Vec2 force, b2Vec2 point, bool wake );

/// Apply a force to the center of mass. This optionally wakes up the body.
/// The force is ignored if the body is not awake.
/// @param bodyId The body id
/// @param force the world force vector, usually in newtons (N).
/// @param wake also wake up the body
B2_API void b2Body_ApplyForceToCenter( b2BodyId bodyId, b2Vec2 force, bool wake );

/// Apply a torque. This affects the angular velocity without affecting the linear velocity.
/// This optionally wakes the body. The torque is ignored if the body is not awake.
/// @param bodyId The body id
/// @param torque about the z-axis (out of the screen), usually in N*m.
/// @param wake also wake up the body
B2_API void b2Body_ApplyTorque( b2BodyId bodyId, float torque, bool wake );

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This optionally wakes the body.
/// The impulse is ignored if the body is not awake.
/// @param bodyId The body id
/// @param impulse the world impulse vector, usually in N*s or kg*m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
/// @warning This should be used for one-shot impulses. If you need a steady force,
/// use a force instead, which will work better with the sub-stepping solver.
B2_API void b2Body_ApplyLinearImpulse( b2BodyId bodyId, b2Vec2 impulse, b2Vec2 point, bool wake );

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// The impulse is ignored if the body is not awake. This optionally wakes the body.
/// @param bodyId The body id
/// @param impulse the world impulse vector, usually in N*s or kg*m/s.
/// @param wake also wake up the body
/// @warning This should be used for one-shot impulses. If you need a steady force,
/// use a force instead, which will work better with the sub-stepping solver.
B2_API void b2Body_ApplyLinearImpulseToCenter( b2BodyId bodyId, b2Vec2 impulse, bool wake );

/// Apply an angular impulse. The impulse is ignored if the body is not awake.
/// This optionally wakes the body.
/// @param bodyId The body id
/// @param impulse the angular impulse, usually in units of kg*m*m/s
/// @param wake also wake up the body
/// @warning This should be used for one-shot impulses. If you need a steady torque,
/// use a torque instead, which will work better with the sub-stepping solver.
B2_API void b2Body_ApplyAngularImpulse( b2BodyId bodyId, float impulse, bool wake );

/// Get the mass of the body, usually in kilograms
B2_API float b2Body_GetMass( b2BodyId bodyId );

/// Get the rotational inertia of the body, usually in kg*m^2
B2_API float b2Body_GetRotationalInertia( b2BodyId bodyId );

/// Get the center of mass position of the body in local space
B2_API b2Vec2 b2Body_GetLocalCenterOfMass( b2BodyId bodyId );

/// Get the center of mass position of the body in world space
B2_API b2Vec2 b2Body_GetWorldCenterOfMass( b2BodyId bodyId );

/// Override the body's mass properties. Normally this is computed automatically using the
/// shape geometry and density. This information is lost if a shape is added or removed or if the
/// body type changes.
B2_API void b2Body_SetMassData( b2BodyId bodyId, b2MassData massData );

/// Get the mass data for a body
B2_API b2MassData b2Body_GetMassData( b2BodyId bodyId );

/// This update the mass properties to the sum of the mass properties of the shapes.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
/// You may also use this when automatic mass computation has been disabled.
/// You should call this regardless of body type.
/// Note that sensor shapes may have mass.
B2_API void b2Body_ApplyMassFromShapes( b2BodyId bodyId );

/// Adjust the linear damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetLinearDamping( b2BodyId bodyId, float linearDamping );

/// Get the current linear damping.
B2_API float b2Body_GetLinearDamping( b2BodyId bodyId );

/// Adjust the angular damping. Normally this is set in b2BodyDef before creation.
B2_API void b2Body_SetAngularDamping( b2BodyId bodyId, float angularDamping );

/// Get the current angular damping.
B2_API float b2Body_GetAngularDamping( b2BodyId bodyId );

/// Adjust the gravity scale. Normally this is set in b2BodyDef before creation.
/// @see b2BodyDef::gravityScale
B2_API void b2Body_SetGravityScale( b2BodyId bodyId, float gravityScale );

/// Get the current gravity scale
B2_API float b2Body_GetGravityScale( b2BodyId bodyId );

/// @return true if this body is awake
B2_API bool b2Body_IsAwake( b2BodyId bodyId );

/// Wake a body from sleep. This wakes the entire island the body is touching.
/// @warning Putting a body to sleep will put the entire island of bodies touching this body to sleep,
/// which can be expensive and possibly unintuitive.
B2_API void b2Body_SetAwake( b2BodyId bodyId, bool awake );

/// Enable or disable sleeping for this body. If sleeping is disabled the body will wake.
B2_API void b2Body_EnableSleep( b2BodyId bodyId, bool enableSleep );

/// Returns true if sleeping is enabled for this body
B2_API bool b2Body_IsSleepEnabled( b2BodyId bodyId );

/// Set the sleep threshold, usually in meters per second
B2_API void b2Body_SetSleepThreshold( b2BodyId bodyId, float sleepThreshold );

/// Get the sleep threshold, usually in meters per second.
B2_API float b2Body_GetSleepThreshold( b2BodyId bodyId );

/// Returns true if this body is enabled
B2_API bool b2Body_IsEnabled( b2BodyId bodyId );

/// Disable a body by removing it completely from the simulation. This is expensive.
B2_API void b2Body_Disable( b2BodyId bodyId );

/// Enable a body by adding it to the simulation. This is expensive.
B2_API void b2Body_Enable( b2BodyId bodyId );

/// Set the motion locks on this body.
B2_API void b2Body_SetMotionLocks( b2BodyId bodyId, b2MotionLocks locks );

/// Get the motion locks for this body.
B2_API b2MotionLocks b2Body_GetMotionLocks( b2BodyId bodyId );

/// Set this body to be a bullet. A bullet does continuous collision detection
/// against dynamic bodies (but not other bullets).
B2_API void b2Body_SetBullet( b2BodyId bodyId, bool flag );

/// Is this body a bullet?
B2_API bool b2Body_IsBullet( b2BodyId bodyId );

/// Enable/disable contact events on all shapes.
/// @see b2ShapeDef::enableContactEvents
/// @warning changing this at runtime may cause mismatched begin/end touch events
B2_API void b2Body_EnableContactEvents( b2BodyId bodyId, bool flag );

/// Enable/disable hit events on all shapes
/// @see b2ShapeDef::enableHitEvents
B2_API void b2Body_EnableHitEvents( b2BodyId bodyId, bool flag );

/// Get the world that owns this body
B2_API b2WorldId b2Body_GetWorld( b2BodyId bodyId );

/// Get the number of shapes on this body
B2_API int b2Body_GetShapeCount( b2BodyId bodyId );

/// Get the shape ids for all shapes on this body, up to the provided capacity.
/// @returns the number of shape ids stored in the user array
B2_API int b2Body_GetShapes( b2BodyId bodyId, b2ShapeId* shapeArray, int capacity );

/// Get the number of joints on this body
B2_API int b2Body_GetJointCount( b2BodyId bodyId );

/// Get the joint ids for all joints on this body, up to the provided capacity
/// @returns the number of joint ids stored in the user array
B2_API int b2Body_GetJoints( b2BodyId bodyId, b2JointId* jointArray, int capacity );

/// Get the maximum capacity required for retrieving all the touching contacts on a body
B2_API int b2Body_GetContactCapacity( b2BodyId bodyId );

/// Get the touching contact data for a body.
/// @note Box2D uses speculative collision so some contact points may be separated.
/// @returns the number of elements filled in the provided array
/// @warning do not ignore the return value, it specifies the valid number of elements
B2_API int b2Body_GetContactData( b2BodyId bodyId, b2ContactData* contactData, int capacity );

/// Get the current world AABB that contains all the attached shapes. Note that this may not encompass the body origin.
/// If there are no shapes attached then the returned AABB is empty and centered on the body origin.
B2_API b2AABB b2Body_ComputeAABB( b2BodyId bodyId );

/** @} */

/**
 * @defgroup shape Shape
 * Functions to create, destroy, and access.
 * Shapes bind raw geometry to bodies and hold material properties including friction and restitution.
 * @{
 */

/// Create a circle shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
/// @return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle );

/// Create a line segment shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
/// @return the shape id for accessing the shape
B2_API b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment );

/// Create a capsule shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
/// @return the shape id for accessing the shape
B2_API b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule );

/// Create a polygon shape and attach it to a body. The shape definition and geometry are fully cloned.
/// Contacts are not created until the next time step.
/// @return the shape id for accessing the shape
B2_API b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon );

/// Destroy a shape. You may defer the body mass update which can improve performance if several shapes on a
///	body are destroyed at once.
///	@see b2Body_ApplyMassFromShapes
B2_API void b2DestroyShape( b2ShapeId shapeId, bool updateBodyMass );

/// Shape identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Shape_IsValid( b2ShapeId id );

/// Get the type of a shape
B2_API b2ShapeType b2Shape_GetType( b2ShapeId shapeId );

/// Get the id of the body that a shape is attached to
B2_API b2BodyId b2Shape_GetBody( b2ShapeId shapeId );

/// Get the world that owns this shape
B2_API b2WorldId b2Shape_GetWorld( b2ShapeId shapeId );

/// Returns true if the shape is a sensor. It is not possible to change a shape
/// from sensor to solid dynamically because this breaks the contract for
/// sensor events.
B2_API bool b2Shape_IsSensor( b2ShapeId shapeId );

/// Set the user data for a shape
B2_API void b2Shape_SetUserData( b2ShapeId shapeId, void* userData );

/// Get the user data for a shape. This is useful when you get a shape id
/// from an event or query.
B2_API void* b2Shape_GetUserData( b2ShapeId shapeId );

/// Set the mass density of a shape, usually in kg/m^2.
/// This will optionally update the mass properties on the parent body.
/// @see b2ShapeDef::density, b2Body_ApplyMassFromShapes
B2_API void b2Shape_SetDensity( b2ShapeId shapeId, float density, bool updateBodyMass );

/// Get the density of a shape, usually in kg/m^2
B2_API float b2Shape_GetDensity( b2ShapeId shapeId );

/// Set the friction on a shape
/// @see b2ShapeDef::friction
B2_API void b2Shape_SetFriction( b2ShapeId shapeId, float friction );

/// Get the friction of a shape
B2_API float b2Shape_GetFriction( b2ShapeId shapeId );

/// Set the shape restitution (bounciness)
/// @see b2ShapeDef::restitution
B2_API void b2Shape_SetRestitution( b2ShapeId shapeId, float restitution );

/// Get the shape restitution
B2_API float b2Shape_GetRestitution( b2ShapeId shapeId );

/// Set the shape material identifier
/// @see b2ShapeDef::material
B2_API void b2Shape_SetMaterial( b2ShapeId shapeId, int material );

/// Get the shape material identifier
B2_API int b2Shape_GetMaterial( b2ShapeId shapeId );

/// Set the shape surface material
B2_API void b2Shape_SetSurfaceMaterial( b2ShapeId shapeId, b2SurfaceMaterial surfaceMaterial );

/// Get the shape surface material
B2_API b2SurfaceMaterial b2Shape_GetSurfaceMaterial( b2ShapeId shapeId );

/// Get the shape filter
B2_API b2Filter b2Shape_GetFilter( b2ShapeId shapeId );

/// Set the current filter. This is almost as expensive as recreating the shape. This may cause
/// contacts to be immediately destroyed. However contacts are not created until the next world step.
/// Sensor overlap state is also not updated until the next world step.
/// @see b2ShapeDef::filter
B2_API void b2Shape_SetFilter( b2ShapeId shapeId, b2Filter filter );

/// Enable sensor events for this shape.
/// @see b2ShapeDef::enableSensorEvents
B2_API void b2Shape_EnableSensorEvents( b2ShapeId shapeId, bool flag );

/// Returns true if sensor events are enabled.
B2_API bool b2Shape_AreSensorEventsEnabled( b2ShapeId shapeId );

/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors.
/// @see b2ShapeDef::enableContactEvents
/// @warning changing this at run-time may lead to lost begin/end events
B2_API void b2Shape_EnableContactEvents( b2ShapeId shapeId, bool flag );

/// Returns true if contact events are enabled
B2_API bool b2Shape_AreContactEventsEnabled( b2ShapeId shapeId );

/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
/// and must be carefully handled due to multithreading. Ignored for sensors.
/// @see b2PreSolveFcn
B2_API void b2Shape_EnablePreSolveEvents( b2ShapeId shapeId, bool flag );

/// Returns true if pre-solve events are enabled
B2_API bool b2Shape_ArePreSolveEventsEnabled( b2ShapeId shapeId );

/// Enable contact hit events for this shape. Ignored for sensors.
/// @see b2WorldDef.hitEventThreshold
B2_API void b2Shape_EnableHitEvents( b2ShapeId shapeId, bool flag );

/// Returns true if hit events are enabled
B2_API bool b2Shape_AreHitEventsEnabled( b2ShapeId shapeId );

/// Test a point for overlap with a shape
B2_API bool b2Shape_TestPoint( b2ShapeId shapeId, b2Vec2 point );

/// Ray cast a shape directly
B2_API b2CastOutput b2Shape_RayCast( b2ShapeId shapeId, const b2RayCastInput* input );

/// Get a copy of the shape's circle. Asserts the type is correct.
B2_API b2Circle b2Shape_GetCircle( b2ShapeId shapeId );

/// Get a copy of the shape's line segment. Asserts the type is correct.
B2_API b2Segment b2Shape_GetSegment( b2ShapeId shapeId );

/// Get a copy of the shape's chain segment. These come from chain shapes.
/// Asserts the type is correct.
B2_API b2ChainSegment b2Shape_GetChainSegment( b2ShapeId shapeId );

/// Get a copy of the shape's capsule. Asserts the type is correct.
B2_API b2Capsule b2Shape_GetCapsule( b2ShapeId shapeId );

/// Get a copy of the shape's convex polygon. Asserts the type is correct.
B2_API b2Polygon b2Shape_GetPolygon( b2ShapeId shapeId );

/// Allows you to change a shape to be a circle or update the current circle.
/// This does not modify the mass properties.
/// @see b2Body_ApplyMassFromShapes
B2_API void b2Shape_SetCircle( b2ShapeId shapeId, const b2Circle* circle );

/// Allows you to change a shape to be a capsule or update the current capsule.
/// This does not modify the mass properties.
/// @see b2Body_ApplyMassFromShapes
B2_API void b2Shape_SetCapsule( b2ShapeId shapeId, const b2Capsule* capsule );

/// Allows you to change a shape to be a segment or update the current segment.
B2_API void b2Shape_SetSegment( b2ShapeId shapeId, const b2Segment* segment );

/// Allows you to change a shape to be a polygon or update the current polygon.
/// This does not modify the mass properties.
/// @see b2Body_ApplyMassFromShapes
B2_API void b2Shape_SetPolygon( b2ShapeId shapeId, const b2Polygon* polygon );

/// Get the parent chain id if the shape type is a chain segment, otherwise
/// returns b2_nullChainId.
B2_API b2ChainId b2Shape_GetParentChain( b2ShapeId shapeId );

/// Get the maximum capacity required for retrieving all the touching contacts on a shape
B2_API int b2Shape_GetContactCapacity( b2ShapeId shapeId );

/// Get the touching contact data for a shape. The provided shapeId will be either shapeIdA or shapeIdB on the contact data.
/// @note Box2D uses speculative collision so some contact points may be separated.
/// @returns the number of elements filled in the provided array
/// @warning do not ignore the return value, it specifies the valid number of elements
B2_API int b2Shape_GetContactData( b2ShapeId shapeId, b2ContactData* contactData, int capacity );

/// Get the maximum capacity required for retrieving all the overlapped shapes on a sensor shape.
/// This returns 0 if the provided shape is not a sensor.
/// @param shapeId the id of a sensor shape
/// @returns the required capacity to get all the overlaps in b2Shape_GetSensorOverlaps
B2_API int b2Shape_GetSensorCapacity( b2ShapeId shapeId );

/// Get the overlap data for a sensor shape.
/// @param shapeId the id of a sensor shape
/// @param visitorIds a user allocated array that is filled with the overlapping shapes (visitors)
/// @param capacity the capacity of overlappedShapes
/// @returns the number of elements filled in the provided array
/// @warning do not ignore the return value, it specifies the valid number of elements
/// @warning overlaps may contain destroyed shapes so use b2Shape_IsValid to confirm each overlap
B2_API int b2Shape_GetSensorData( b2ShapeId shapeId, b2ShapeId* visitorIds, int capacity );

/// Get the current world AABB
B2_API b2AABB b2Shape_GetAABB( b2ShapeId shapeId );

/// Compute the mass data for a shape
B2_API b2MassData b2Shape_ComputeMassData( b2ShapeId shapeId );

/// Get the closest point on a shape to a target point. Target and result are in world space.
/// todo need sample
B2_API b2Vec2 b2Shape_GetClosestPoint( b2ShapeId shapeId, b2Vec2 target );

/// Chain Shape

/// Create a chain shape
/// @see b2ChainDef for details
B2_API b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def );

/// Destroy a chain shape
B2_API void b2DestroyChain( b2ChainId chainId );

/// Get the world that owns this chain shape
B2_API b2WorldId b2Chain_GetWorld( b2ChainId chainId );

/// Get the number of segments on this chain
B2_API int b2Chain_GetSegmentCount( b2ChainId chainId );

/// Fill a user array with chain segment shape ids up to the specified capacity. Returns
/// the actual number of segments returned.
B2_API int b2Chain_GetSegments( b2ChainId chainId, b2ShapeId* segmentArray, int capacity );

/// Set the chain friction
/// @see b2ChainDef::friction
B2_API void b2Chain_SetFriction( b2ChainId chainId, float friction );

/// Get the chain friction
B2_API float b2Chain_GetFriction( b2ChainId chainId );

/// Set the chain restitution (bounciness)
/// @see b2ChainDef::restitution
B2_API void b2Chain_SetRestitution( b2ChainId chainId, float restitution );

/// Get the chain restitution
B2_API float b2Chain_GetRestitution( b2ChainId chainId );

/// Set the chain material
/// @see b2ChainDef::material
B2_API void b2Chain_SetMaterial( b2ChainId chainId, int material );

/// Get the chain material
B2_API int b2Chain_GetMaterial( b2ChainId chainId );

/// Chain identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Chain_IsValid( b2ChainId id );

/** @} */

/**
 * @defgroup joint Joint
 * @brief Joints allow you to connect rigid bodies together while allowing various forms of relative motions.
 * @{
 */

/// Destroy a joint
B2_API void b2DestroyJoint( b2JointId jointId );

/// Joint identifier validation. Provides validation for up to 64K allocations.
B2_API bool b2Joint_IsValid( b2JointId id );

/// Get the joint type
B2_API b2JointType b2Joint_GetType( b2JointId jointId );

/// Get body A id on a joint
B2_API b2BodyId b2Joint_GetBodyA( b2JointId jointId );

/// Get body B id on a joint
B2_API b2BodyId b2Joint_GetBodyB( b2JointId jointId );

/// Get the world that owns this joint
B2_API b2WorldId b2Joint_GetWorld( b2JointId jointId );

/// Set the local frame on bodyA
B2_API void b2Joint_SetLocalFrameA( b2JointId jointId, b2Transform localFrame );

/// Get the local frame on bodyA
B2_API b2Transform b2Joint_GetLocalFrameA( b2JointId jointId );

/// Set the local frame on bodyB
B2_API void b2Joint_SetLocalFrameB( b2JointId jointId, b2Transform localFrame );

/// Get the local frame on bodyB
B2_API b2Transform b2Joint_GetLocalFrameB( b2JointId jointId );

/// Toggle collision between connected bodies
B2_API void b2Joint_SetCollideConnected( b2JointId jointId, bool shouldCollide );

/// Is collision allowed between connected bodies?
B2_API bool b2Joint_GetCollideConnected( b2JointId jointId );

/// Set the user data on a joint
B2_API void b2Joint_SetUserData( b2JointId jointId, void* userData );

/// Get the user data on a joint
B2_API void* b2Joint_GetUserData( b2JointId jointId );

/// Wake the bodies connect to this joint
B2_API void b2Joint_WakeBodies( b2JointId jointId );

/// Get the current constraint force for this joint. Usually in Newtons.
B2_API b2Vec2 b2Joint_GetConstraintForce( b2JointId jointId );

/// Get the current constraint torque for this joint. Usually in Newton * meters.
B2_API float b2Joint_GetConstraintTorque( b2JointId jointId );

/// Get the current linear separation error for this joint. Does not consider admissible movement. Usually in meters.
B2_API float b2Joint_GetLinearSeparation( b2JointId jointId );

/// Get the current angular separation error for this joint. Does not consider admissible movement. Usually in meters.
B2_API float b2Joint_GetAngularSeparation( b2JointId jointId );

/// Set the joint constraint tuning. Advanced feature.
/// @param jointId the joint
/// @param hertz the stiffness in Hertz (cycles per second)
/// @param dampingRatio the non-dimensional damping ratio (one for critical damping)
B2_API void b2Joint_SetConstraintTuning( b2JointId jointId, float hertz, float dampingRatio );

/// Get the joint constraint tuning. Advanced feature.
B2_API void b2Joint_GetConstraintTuning( b2JointId jointId, float* hertz, float* dampingRatio );

/// Set the force threshold for joint events (Newtons)
B2_API void b2Joint_SetForceThreshold( b2JointId jointId, float threshold );

/// Get the force threshold for joint events (Newtons)
B2_API float b2Joint_GetForceThreshold( b2JointId jointId );

/// Set the torque threshold for joint events (N-m)
B2_API void b2Joint_SetTorqueThreshold( b2JointId jointId, float threshold );

/// Get the torque threshold for joint events (N-m)
B2_API float b2Joint_GetTorqueThreshold( b2JointId jointId );


/**
 * @defgroup distance_joint Distance Joint
 * @brief Functions for the distance joint.
 * @{
 */

/// Create a distance joint
/// @see b2DistanceJointDef for details
B2_API b2JointId b2CreateDistanceJoint( b2WorldId worldId, const b2DistanceJointDef* def );

/// Set the rest length of a distance joint
/// @param jointId The id for a distance joint
/// @param length The new distance joint length
B2_API void b2DistanceJoint_SetLength( b2JointId jointId, float length );

/// Get the rest length of a distance joint
B2_API float b2DistanceJoint_GetLength( b2JointId jointId );

/// Enable/disable the distance joint spring. When disabled the distance joint is rigid.
B2_API void b2DistanceJoint_EnableSpring( b2JointId jointId, bool enableSpring );

/// Is the distance joint spring enabled?
B2_API bool b2DistanceJoint_IsSpringEnabled( b2JointId jointId );

/// Set the force range for the spring.
B2_API void b2DistanceJoint_SetSpringForceRange( b2JointId jointId, float lowerForce, float upperForce );

/// Get the force range for the spring.
B2_API void b2DistanceJoint_GetSpringForceRange( b2JointId jointId, float* lowerForce, float* upperForce );

/// The the spring resist compression?
B2_API bool b2DistanceJoint_IsCompressionEnabled( b2JointId jointId );

/// Set the spring stiffness in Hertz
B2_API void b2DistanceJoint_SetSpringHertz( b2JointId jointId, float hertz );

/// Set the spring damping ratio, non-dimensional
B2_API void b2DistanceJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the spring Hertz
B2_API float b2DistanceJoint_GetSpringHertz( b2JointId jointId );

/// Get the spring damping ratio
B2_API float b2DistanceJoint_GetSpringDampingRatio( b2JointId jointId );

/// Enable joint limit. The limit only works if the joint spring is enabled. Otherwise the joint is rigid
/// and the limit has no effect.
B2_API void b2DistanceJoint_EnableLimit( b2JointId jointId, bool enableLimit );

/// Is the distance joint limit enabled?
B2_API bool b2DistanceJoint_IsLimitEnabled( b2JointId jointId );

/// Set the minimum and maximum length parameters of a distance joint
B2_API void b2DistanceJoint_SetLengthRange( b2JointId jointId, float minLength, float maxLength );

/// Get the distance joint minimum length
B2_API float b2DistanceJoint_GetMinLength( b2JointId jointId );

/// Get the distance joint maximum length
B2_API float b2DistanceJoint_GetMaxLength( b2JointId jointId );

/// Get the current length of a distance joint
B2_API float b2DistanceJoint_GetCurrentLength( b2JointId jointId );

/// Enable/disable the distance joint motor
B2_API void b2DistanceJoint_EnableMotor( b2JointId jointId, bool enableMotor );

/// Is the distance joint motor enabled?
B2_API bool b2DistanceJoint_IsMotorEnabled( b2JointId jointId );

/// Set the distance joint motor speed, usually in meters per second
B2_API void b2DistanceJoint_SetMotorSpeed( b2JointId jointId, float motorSpeed );

/// Get the distance joint motor speed, usually in meters per second
B2_API float b2DistanceJoint_GetMotorSpeed( b2JointId jointId );

/// Set the distance joint maximum motor force, usually in newtons
B2_API void b2DistanceJoint_SetMaxMotorForce( b2JointId jointId, float force );

/// Get the distance joint maximum motor force, usually in newtons
B2_API float b2DistanceJoint_GetMaxMotorForce( b2JointId jointId );

/// Get the distance joint current motor force, usually in newtons
B2_API float b2DistanceJoint_GetMotorForce( b2JointId jointId );

/** @} */

/**
 * @defgroup motor_joint Motor Joint
 * @brief Functions for the motor joint.
 *
 * The motor joint is used to drive the relative transform between two bodies. The target
 * is set by updating the local frames using b2Joint_SetLocalFrameA or b2Joint_SetLocalFrameB.
 * @{
 */

/// Create a motor joint
/// @see b2MotorJointDef for details
B2_API b2JointId b2CreateMotorJoint( b2WorldId worldId, const b2MotorJointDef* def );

/// Set the desired relative linear velocity in meters per second
B2_API void b2MotorJoint_SetLinearVelocity( b2JointId jointId, b2Vec2 velocity );

/// Get the desired relative linear velocity in meters per second
B2_API b2Vec2 b2MotorJoint_GetLinearVelocity( b2JointId jointId );

/// Set the desired relative angular velocity in radians per second
B2_API void b2MotorJoint_SetAngularVelocity( b2JointId jointId, float velocity );

/// Get the desired relative angular velocity in radians per second
B2_API float b2MotorJoint_GetAngularVelocity( b2JointId jointId );

/// Set the motor joint maximum force, usually in newtons
B2_API void b2MotorJoint_SetMaxVelocityForce( b2JointId jointId, float maxForce );

/// Get the motor joint maximum force, usually in newtons
B2_API float b2MotorJoint_GetMaxVelocityForce( b2JointId jointId );

/// Set the motor joint maximum torque, usually in newton-meters
B2_API void b2MotorJoint_SetMaxVelocityTorque( b2JointId jointId, float maxTorque );

/// Get the motor joint maximum torque, usually in newton-meters
B2_API float b2MotorJoint_GetMaxVelocityTorque( b2JointId jointId );

/// Set the spring linear hertz stiffness
B2_API void b2MotorJoint_SetLinearHertz( b2JointId jointId, float hertz );

/// Get the spring linear hertz stiffness
B2_API float b2MotorJoint_GetLinearHertz( b2JointId jointId );

/// Set the spring linear damping ratio. Use 1.0 for critical damping.
B2_API void b2MotorJoint_SetLinearDampingRatio( b2JointId jointId, float damping );

/// Get the spring linear damping ratio.
B2_API float b2MotorJoint_GetLinearDampingRatio( b2JointId jointId );

/// Set the spring angular hertz stiffness
B2_API void b2MotorJoint_SetAngularHertz( b2JointId jointId, float hertz );

/// Get the spring angular hertz stiffness
B2_API float b2MotorJoint_GetAngularHertz( b2JointId jointId );

/// Set the spring angular damping ratio. Use 1.0 for critical damping.
B2_API void b2MotorJoint_SetAngularDampingRatio( b2JointId jointId, float damping );

/// Get the spring angular damping ratio.
B2_API float b2MotorJoint_GetAngularDampingRatio( b2JointId jointId );

/// Set the maximum spring force in newtons.
B2_API void b2MotorJoint_SetMaxSpringForce( b2JointId jointId, float maxForce );

/// Get the maximum spring force in newtons.
B2_API float b2MotorJoint_GetMaxSpringForce( b2JointId jointId );

/// Set the maximum spring torque in newtons * meters
B2_API void b2MotorJoint_SetMaxSpringTorque( b2JointId jointId, float maxTorque );

/// Get the maximum spring torque in newtons * meters
B2_API float b2MotorJoint_GetMaxSpringTorque( b2JointId jointId );

/**@}*/

/**
 * @defgroup mouse_joint Mouse Joint
 * @brief Functions for the mouse joint.
 *
 * The mouse joint is designed for use in the samples application, but you may find it useful in applications where
 * the user moves a rigid body with a cursor.
 * @{
 */

/// Create a mouse joint
/// @see b2MouseJointDef for details
B2_API b2JointId b2CreateMouseJoint( b2WorldId worldId, const b2MouseJointDef* def );

/// Set the mouse joint spring stiffness in Hertz
B2_API void b2MouseJoint_SetSpringHertz( b2JointId jointId, float hertz );

/// Get the mouse joint spring stiffness in Hertz
B2_API float b2MouseJoint_GetSpringHertz( b2JointId jointId );

/// Set the mouse joint spring damping ratio, non-dimensional
B2_API void b2MouseJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the mouse joint damping ratio, non-dimensional
B2_API float b2MouseJoint_GetSpringDampingRatio( b2JointId jointId );

/// Set the mouse joint maximum force, usually in newtons
B2_API void b2MouseJoint_SetMaxForce( b2JointId jointId, float maxForce );

/// Get the mouse joint maximum force, usually in newtons
B2_API float b2MouseJoint_GetMaxForce( b2JointId jointId );

/**@}*/

/**
 * @defgroup filter_joint Filter Joint
 * @brief Functions for the filter joint.
 *
 * The filter joint is used to disable collision between two bodies. As a side effect of being a joint, it also
 * keeps the two bodies in the same simulation island.
 * @{
 */

/// Create a filter joint.
/// @see b2FilterJointDef for details
B2_API b2JointId b2CreateFilterJoint( b2WorldId worldId, const b2FilterJointDef* def );

/**@}*/

/**
 * @defgroup prismatic_joint Prismatic Joint
 * @brief A prismatic joint allows for translation along a single axis with no rotation.
 *
 * The prismatic joint is useful for things like pistons and moving platforms, where you want a body to translate
 * along an axis and have no rotation. Also called a *slider* joint.
 * @{
 */

/// Create a prismatic (slider) joint.
/// @see b2PrismaticJointDef for details
B2_API b2JointId b2CreatePrismaticJoint( b2WorldId worldId, const b2PrismaticJointDef* def );

/// Enable/disable the joint spring.
B2_API void b2PrismaticJoint_EnableSpring( b2JointId jointId, bool enableSpring );

/// Is the prismatic joint spring enabled or not?
B2_API bool b2PrismaticJoint_IsSpringEnabled( b2JointId jointId );

/// Set the prismatic joint stiffness in Hertz.
/// This should usually be less than a quarter of the simulation rate. For example, if the simulation
/// runs at 60Hz then the joint stiffness should be 15Hz or less.
B2_API void b2PrismaticJoint_SetSpringHertz( b2JointId jointId, float hertz );

/// Get the prismatic joint stiffness in Hertz
B2_API float b2PrismaticJoint_GetSpringHertz( b2JointId jointId );

/// Set the prismatic joint damping ratio (non-dimensional)
B2_API void b2PrismaticJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the prismatic spring damping ratio (non-dimensional)
B2_API float b2PrismaticJoint_GetSpringDampingRatio( b2JointId jointId );

/// Set the prismatic joint spring target angle, usually in meters
B2_API void b2PrismaticJoint_SetTargetTranslation( b2JointId jointId, float translation );

/// Get the prismatic joint spring target translation, usually in meters
B2_API float b2PrismaticJoint_GetTargetTranslation( b2JointId jointId );

/// Enable/disable a prismatic joint limit
B2_API void b2PrismaticJoint_EnableLimit( b2JointId jointId, bool enableLimit );

/// Is the prismatic joint limit enabled?
B2_API bool b2PrismaticJoint_IsLimitEnabled( b2JointId jointId );

/// Get the prismatic joint lower limit
B2_API float b2PrismaticJoint_GetLowerLimit( b2JointId jointId );

/// Get the prismatic joint upper limit
B2_API float b2PrismaticJoint_GetUpperLimit( b2JointId jointId );

/// Set the prismatic joint limits
B2_API void b2PrismaticJoint_SetLimits( b2JointId jointId, float lower, float upper );

/// Enable/disable a prismatic joint motor
B2_API void b2PrismaticJoint_EnableMotor( b2JointId jointId, bool enableMotor );

/// Is the prismatic joint motor enabled?
B2_API bool b2PrismaticJoint_IsMotorEnabled( b2JointId jointId );

/// Set the prismatic joint motor speed, usually in meters per second
B2_API void b2PrismaticJoint_SetMotorSpeed( b2JointId jointId, float motorSpeed );

/// Get the prismatic joint motor speed, usually in meters per second
B2_API float b2PrismaticJoint_GetMotorSpeed( b2JointId jointId );

/// Set the prismatic joint maximum motor force, usually in newtons
B2_API void b2PrismaticJoint_SetMaxMotorForce( b2JointId jointId, float force );

/// Get the prismatic joint maximum motor force, usually in newtons
B2_API float b2PrismaticJoint_GetMaxMotorForce( b2JointId jointId );

/// Get the prismatic joint current motor force, usually in newtons
B2_API float b2PrismaticJoint_GetMotorForce( b2JointId jointId );

/// Get the current joint translation, usually in meters.
B2_API float b2PrismaticJoint_GetTranslation( b2JointId jointId );

/// Get the current joint translation speed, usually in meters per second.
B2_API float b2PrismaticJoint_GetSpeed( b2JointId jointId );

/** @} */

/**
 * @defgroup revolute_joint Revolute Joint
 * @brief A revolute joint allows for relative rotation in the 2D plane with no relative translation.
 *
 * The revolute joint is probably the most common joint. It can be used for ragdolls and chains.
 * Also called a *hinge* or *pin* joint.
 * @{
 */

/// Create a revolute joint
/// @see b2RevoluteJointDef for details
B2_API b2JointId b2CreateRevoluteJoint( b2WorldId worldId, const b2RevoluteJointDef* def );

/// Enable/disable the revolute joint spring
B2_API void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring );

/// It the revolute angular spring enabled?
B2_API bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId );

/// Set the revolute joint spring stiffness in Hertz
B2_API void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, float hertz );

/// Get the revolute joint spring stiffness in Hertz
B2_API float b2RevoluteJoint_GetSpringHertz( b2JointId jointId );

/// Set the revolute joint spring damping ratio, non-dimensional
B2_API void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the revolute joint spring damping ratio, non-dimensional
B2_API float b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId );

/// Set the revolute joint spring target angle, radians
B2_API void b2RevoluteJoint_SetTargetAngle( b2JointId jointId, float angle );

/// Get the revolute joint spring target angle, radians
B2_API float b2RevoluteJoint_GetTargetAngle( b2JointId jointId );

/// Get the revolute joint current angle in radians relative to the reference angle
/// @see b2RevoluteJointDef::referenceAngle
B2_API float b2RevoluteJoint_GetAngle( b2JointId jointId );

/// Enable/disable the revolute joint limit
B2_API void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit );

/// Is the revolute joint limit enabled?
B2_API bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId );

/// Get the revolute joint lower limit in radians
B2_API float b2RevoluteJoint_GetLowerLimit( b2JointId jointId );

/// Get the revolute joint upper limit in radians
B2_API float b2RevoluteJoint_GetUpperLimit( b2JointId jointId );

/// Set the revolute joint limits in radians. It is expected that lower <= upper
/// and that -0.99 * B2_PI <= lower && upper <= -0.99 * B2_PI.
B2_API void b2RevoluteJoint_SetLimits( b2JointId jointId, float lower, float upper );

/// Enable/disable a revolute joint motor
B2_API void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor );

/// Is the revolute joint motor enabled?
B2_API bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId );

/// Set the revolute joint motor speed in radians per second
B2_API void b2RevoluteJoint_SetMotorSpeed( b2JointId jointId, float motorSpeed );

/// Get the revolute joint motor speed in radians per second
B2_API float b2RevoluteJoint_GetMotorSpeed( b2JointId jointId );

/// Get the revolute joint current motor torque, usually in newton-meters
B2_API float b2RevoluteJoint_GetMotorTorque( b2JointId jointId );

/// Set the revolute joint maximum motor torque, usually in newton-meters
B2_API void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, float torque );

/// Get the revolute joint maximum motor torque, usually in newton-meters
B2_API float b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId );

/**@}*/

/**
 * @defgroup weld_joint Weld Joint
 * @brief A weld joint fully constrains the relative transform between two bodies while allowing for springiness
 *
 * A weld joint constrains the relative rotation and translation between two bodies. Both rotation and translation
 * can have damped springs.
 *
 * @note The accuracy of weld joint is limited by the accuracy of the solver. Long chains of weld joints may flex.
 * @{
 */

/// Create a weld joint
/// @see b2WeldJointDef for details
B2_API b2JointId b2CreateWeldJoint( b2WorldId worldId, const b2WeldJointDef* def );

/// Set the weld joint linear stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetLinearHertz( b2JointId jointId, float hertz );

/// Get the weld joint linear stiffness in Hertz
B2_API float b2WeldJoint_GetLinearHertz( b2JointId jointId );

/// Set the weld joint linear damping ratio (non-dimensional)
B2_API void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the weld joint linear damping ratio (non-dimensional)
B2_API float b2WeldJoint_GetLinearDampingRatio( b2JointId jointId );

/// Set the weld joint angular stiffness in Hertz. 0 is rigid.
B2_API void b2WeldJoint_SetAngularHertz( b2JointId jointId, float hertz );

/// Get the weld joint angular stiffness in Hertz
B2_API float b2WeldJoint_GetAngularHertz( b2JointId jointId );

/// Set weld joint angular damping ratio, non-dimensional
B2_API void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the weld joint angular damping ratio, non-dimensional
B2_API float b2WeldJoint_GetAngularDampingRatio( b2JointId jointId );

/** @} */

/**
 * @defgroup wheel_joint Wheel Joint
 * The wheel joint can be used to simulate wheels on vehicles.
 *
 * The wheel joint restricts body B to move along a local axis in body A. Body B is free to
 * rotate. Supports a linear spring, linear limits, and a rotational motor.
 *
 * @{
 */

/// Create a wheel joint
/// @see b2WheelJointDef for details
B2_API b2JointId b2CreateWheelJoint( b2WorldId worldId, const b2WheelJointDef* def );

/// Enable/disable the wheel joint spring
B2_API void b2WheelJoint_EnableSpring( b2JointId jointId, bool enableSpring );

/// Is the wheel joint spring enabled?
B2_API bool b2WheelJoint_IsSpringEnabled( b2JointId jointId );

/// Set the wheel joint stiffness in Hertz
B2_API void b2WheelJoint_SetSpringHertz( b2JointId jointId, float hertz );

/// Get the wheel joint stiffness in Hertz
B2_API float b2WheelJoint_GetSpringHertz( b2JointId jointId );

/// Set the wheel joint damping ratio, non-dimensional
B2_API void b2WheelJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );

/// Get the wheel joint damping ratio, non-dimensional
B2_API float b2WheelJoint_GetSpringDampingRatio( b2JointId jointId );

/// Enable/disable the wheel joint limit
B2_API void b2WheelJoint_EnableLimit( b2JointId jointId, bool enableLimit );

/// Is the wheel joint limit enabled?
B2_API bool b2WheelJoint_IsLimitEnabled( b2JointId jointId );

/// Get the wheel joint lower limit
B2_API float b2WheelJoint_GetLowerLimit( b2JointId jointId );

/// Get the wheel joint upper limit
B2_API float b2WheelJoint_GetUpperLimit( b2JointId jointId );

/// Set the wheel joint limits
B2_API void b2WheelJoint_SetLimits( b2JointId jointId, float lower, float upper );

/// Enable/disable the wheel joint motor
B2_API void b2WheelJoint_EnableMotor( b2JointId jointId, bool enableMotor );

/// Is the wheel joint motor enabled?
B2_API bool b2WheelJoint_IsMotorEnabled( b2JointId jointId );

/// Set the wheel joint motor speed in radians per second
B2_API void b2WheelJoint_SetMotorSpeed( b2JointId jointId, float motorSpeed );

/// Get the wheel joint motor speed in radians per second
B2_API float b2WheelJoint_GetMotorSpeed( b2JointId jointId );

/// Set the wheel joint maximum motor torque, usually in newton-meters
B2_API void b2WheelJoint_SetMaxMotorTorque( b2JointId jointId, float torque );

/// Get the wheel joint maximum motor torque, usually in newton-meters
B2_API float b2WheelJoint_GetMaxMotorTorque( b2JointId jointId );

/// Get the wheel joint current motor torque, usually in newton-meters
B2_API float b2WheelJoint_GetMotorTorque( b2JointId jointId );

/**@}*/

/**@}*/

/**
 * @defgroup contact Contact
 * Access to contacts
 * @{
 */

/// Contact identifier validation. Provides validation for up to 2^32 allocations.
B2_API bool b2Contact_IsValid( b2ContactId id );

/// Get the data for a contact. The manifold may have no points if the contact is not touching.
B2_API b2ContactData b2Contact_GetData( b2ContactId contactId );

/**@}*/
