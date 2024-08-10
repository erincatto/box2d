# Migration Guide

## Version 2.4 to Version 3.0
Box2D version 3.0 is a full rewrite. You can read some background information [here](https://box2d.org/posts/2023/01/starting-box2d-3.0/).

Here are the highlights that affect the API:
- moved from C++ to C
- identifiers (handles) instead of pointers
- multithreading support
- fewer callbacks
- more features (such as capsules and shape casts)
- new sub-stepping solver (*Soft Step*)
- gear and pulley joint removed (temporarily)

However, the scope of what Box2D does has not changed much. It is still a 2D rigid body engine. It is just faster and more robust (hopefully). And hopefully it is easier to work with and port/wrap for other languages/platforms.

I'm going to describe migration by comparing code snippets between 2.4 and 3.0. These should give you and idea of the sort of transformations you need to make to your code to migrate to v3.0. These snippets are written in C and may need some small adjustments to work with C++.

I'm not going to cover all the details of v3.0 in this guide. That is the job of the manual, the doxygen reference, and the samples.

The surface area of the Box2D is smaller in v3.0 because C++ is not good at hiding details. So hopefully you find the new API easier to work with.

### Creating a world
Version 2.4:
```cpp
#include "box2d/box2d.h"
b2Vec2 gravity(0.0f, -10.0f);
b2World world(gravity);
```
Version 3.0:
```c
#include "box2d/box2d.h"
b2Vec2 gravity = {0.0f, -10.0f};
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.gravity = gravity;
b2WorldId worldId = b2CreateWorld(&worldDef);
```
There is now a required world definition. C does not have constructors, so you need to initialize **ALL** structures that you pass to Box2D. Box2D provides an initialization helper for almost all structures. For example `b2DefaultWorldDef()` is used here to initialize `b2WorldDef`. `b2WorldDef` provides many options, but the defaults are good enough to get going.

In Version 3.0, Box2D objects are generally hidden and you only have an identifier. This keeps the API small. So when you create a world you just get a `b2WorldId` which you should treat as an atomic object, like `int` or `float`. It is small and should be passed by value.

In Version 3.0 there are also no destructors, so you must destroy the world.
```c
b2DestroyWorld(worldId);
worldId = b2_nullWorldId;
```
This destroys all bodies, shapes, and joints as well. This is quicker than destroying them individually. Just like pointers, it is good practice to nullify identifiers. Box2D provides null values for all identifiers and also macros such as `B2_IS_NULL` to test if an identifier is null.

### Creating a body
Version 2.4:
```cpp
b2BodyDef bodyDef;
bodyDef.type = b2_dynamicBody;
bodyDef.position.Set(0.0f, 4.0f);
b2Body* body = world.CreateBody(&bodyDef);
```
Version 3.0:
```c
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.type = b2_dynamicBody;
bodyDef.position = (b2Vec2){0.0f, 4.0f};
b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
```
Body creation is very similar in v3.0. In this case there is a definition initialization function `b2DefaultBodyDef()`. This can help save a bit of typing in some cases. In v3.0 I recommend getting comfortable with curly brace initialization for initializing vectors. There are no member functions in C. Notice that the body is created using a loose function and providing the `b2WorldId` as an argument. Basically what you would expect going from C++ to C.

Destroying a body is also similar.
Version 2.4:
```cpp
world.DestroyBody(body);
body = nullptr;
```
Version 3.0:
```c
b2DestroyBody(bodyId);
bodyId = b2_nullBodyId;
```
Notice there is a little magic here in Version 3.0. `b2BodyId` knows what world it comes from. So you do not need to provide `worldId` when destroying the body. Version 3.0 supports up to 128 worlds. This may be increased or be overridden in the future.

Shapes and joints are still destroyed automatically. However, `b2DestructionListener` is gone. This holds to the theme of fewer callbacks. However, you can now use 
`b2Shape_IsValid()` and `b2Joint_IsValid()`.

### Creating a shape
Shape creation has been streamlined in Version 3.0. `b2Fixture` is gone. I feel like it was a confusing concept so I hope you don't miss it.

Version 2.4:
```cpp
b2PolygonShape box;
box.SetAsBox(1.0f, 1.0f);

b2FixtureDef fixtureDef;
fixtureDef.shape = &box;
fixtureDef.density = 1.0f;
fixtureDef.friction = 0.3f;

b2Fixture* fixture = body->CreateFixture(&fixtureDef);
```

Version 3.0:
```c
b2Polygon box = b2MakeBox(1.0f, 1.0f);

b2ShapeDef shapeDef = b2DefaultShapeDef();
shapeDef.density = 1.0f;
shapeDef.friction = 0.3f;

b2ShapeId shapeId = b2CreatePolygonShape(bodyId, &shapeDef, &box);
```

So basically v2.4 shapes are no longer shapes, they are *primitives* or *geometry* with no inheritance (of course). This freed the term _shape_ to be used where _fixture_ was used before. In v3.0 the shape definition is generic and there are different functions for creating each shape type, such as `b2CreateCircleShape` or `b2CreateSegmentShape`.

Again notice the structure initialization with `b2DefaultShapeDef()`. Unfortunately we cannot have meaningful definitions with zero initialization. You must initialize your structures.

Another important change for shapes is that the default density in the shape definition is now 1 instead of 0. Static and kinematic bodies will ignore the density. You can now make an entire game without touching the density.

Destroying shapes is straight forward.

Version 2.4:
```cpp
body->DestroyFixture(fixture);
fixture = nullptr;
```

Version 3.0:
```c
b2DestroyShape(shapeId);
shapeId = b2_nullShapeId;
```

### Chains
In Version 2.4 chains are a type of shape. In Version 3.0 they are a separate concept. This leads to significant simplifications internally. In Version 2.4 all shapes had to support the notion of child shapes. This is gone.

Version 2.4:
```cpp
b2Vec2 points[5];
points[0].Set(-8.0f, 6.0f);
points[1].Set(-8.0f, 20.0f);
points[2].Set(8.0f, 20.0f);
points[3].Set(8.0f, 6.0f);
points[4].Set(0.0f, -2.0f);

b2ChainShape chain;
chain.CreateLoop(points, 5);
b2FixtureDef fixtureDef;
fixtureDef.shape = &chain;
b2Fixture* chainFixture = body->CreateFixture(&fixtureDef);
```

Version 3.0:
```c
b2Vec2 points[5] = {
    {-8.0f, 6.0f},
    {-8.0f, 20.0f},
    {8.0f, 20.0f},
    {8.0f, 6.0f},
    {0.0f, -2.0f}
};

b2ChainDef chainDef = b2DefaultChainDef();
chainDef.points = points;
chainDef.count = 5;
chainDef.loop = true;
b2ChainId chainId = b2CreateChain(bodyId, &chainDef);
```

Since chains are their own concept now, they get their own identifier, `b2ChainId`. You can view chains as macro objects, they create many `b2SmoothSegment` shapes internally. Normally you don't interact with these. However they are returned from queries. You can use `b2Shape_GetParentChain()` to get the `b2ChainId` for a smooth segment that you get from a query.

> DO NOT destroy or modify a `b2SmoothSegment` that belongs to a chain shape directly

### Creating a joint
Joints are very similar in v3.0. The lack of C member functions changes initialization.

Version 2.4:
```cpp
b2RevoluteJointDef jointDef;
jointDef.Initialize(ground, body, b2Vec2(-10.0f, 20.5f));
jointDef.motorSpeed = 1.0f;
jointDef.maxMotorTorque = 100.0f;
jointDef.enableMotor = true;
jointDef.lowerAngle = -0.25f * b2_pi;
jointDef.upperAngle = 0.5f * b2_pi;
jointDef.enableLimit = true;:
b2RevolutionJoint* joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
```
Version 3.0:
```c
b2Vec2 pivot = {-10.0f, 20.5f};
b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
jointDef.bodyIdA = groundId;
jointDef.bodyIdB = bodyId;
jointDef.localAnchorA = b2Body_GetLocalPoint(jointDef.bodyIdA, pivot);
jointDef.localAnchorB = b2Body_GetLocalPoint(jointDef.bodyIdB, pivot);
jointDef.motorSpeed = 1.0f;
jointDef.maxMotorTorque = 100.0f;
jointDef.enableMotor = true;
jointDef.lowerAngle = -0.25f * b2_pi;
jointDef.upperAngle = 0.5f * b2_pi;
jointDef.enableLimit = true;
b2JointId jointId = b2CreateRevoluteJoint(worldId, &jointDef);
```

Some of the joints have more options now. Check the code comments and samples for details.

The friction joint has been removed since it is a subset of the motor joint.

The pulley and gear joints have been removed. I'm not satisfied with how they work in 2.4 and plan to implement improved versions in the future.

### New solver
There is a new solver that uses sub-stepping called *Soft Step*. Instead of specifying velocity iterations or position iterations, you now specify the number of sub-steps.
```c
void b2World_Step(b2WorldId worldId, float timeStep, int32_t subStepCount);
```
It is recommended to start with 4 sub-steps and adjust as needed. The sub-stepping only computes contact points once per full time step, so contact events are for the full time step.

With a sub-stepping solver you need to think differently about how you interact with bodies. Externally applied impulses or velocity adjustments no longer exist after the first sub-step. So if you try to control the movement of a body by setting the velocity every time step then you may get unexpected results. You will get more predictable results by applying a force and/or torque. Forces and torques are spread across all time steps.

If you want full control over the movement of a body, considering setting the body type to `b2_kinematicBody`. Preferably this is done in the `b2BodyDef`:
```c
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.type = b2_kinematicBody;
```

### Contact data
In v2.4 `b2ContactListener` provided `BeginContact`, `EndContact`, `PreSolve`, and `PostSolve`. You could also iterate over the contacts associated with a body using `b2Body::GetContactList`. The latter was rarely used due to how continuous collision worked in v2.4 meant that you could miss some contacts using `GetContactList`.

In v3.0 there is a strong emphasis on multithreading. Callbacks in multithreading are problematic for a few reasons:
  * chance of race conditions in user code
  * user code becomes non-deterministic
  * uncertain performance impact

Therefore all callbacks except `PreSolve` have been removed. Instead you can now access all events and contact data after the time step. Version 3.0 no longer uses collision sub-stepping for continuous collision. This means all contacts data are valid at the end of the time step. Just keep in mind that Box2D computes contact points at the beginning of the time step, so the contact points apply to the previous position of the body.

Here is how you access contact data in v3.0:
```c
b2ContactEvents contactEvents = b2World_GetContactEvents(worldId);
```
The contact events structure has begin and end events:
```c 
typedef struct b2ContactEvents
{
	b2ContactBeginTouchEvent* beginEvents;
	b2ContactEndTouchEvent* endEvents;
  b2ContactHitEvent* hitEvents;
	int beginCount;
	int endCount;
  int hitCount;
} b2ContactEvents;
```
You can loop through these events after the time step. These events are in deterministic order, even with multithreading. See the `sample_events.cpp` file for examples.

You may not want Box2D to save all contact events, so you can disable them for a given shape using `enableContactEvents` on `b2ShapeDef`.

If you want to access persistent contacts, you can get the data from bodies or shapes.
```c
b2ContactData contactData[10];
int count = b2Body_GetContactData(bodyId, contactData, 10);
```
```c
b2ContactData contactData[10];
int count = b2Shape_GetContactData(shapeId, contactData, 10);
```
This includes contact data for contacts reported in begin events. This data is also in deterministic order.

Pre-solve contact modification is available using a callback.
```c
typedef bool b2PreSolveFcn(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context);
void b2World_SetPreSolveCallback(b2WorldId worldId, b2PreSolveFcn* fcn, void* context);
```
You can define a pre-solve callback and register that with the world. You can also provide a context variable that will be passed back to your callback. This is **not** enough to get a pre-solve callback. You also need to enable it on your shape using `enablePreSolveEvents` in `b2ShapeDef`. This is false by default.

> Pre-solve callbacks are dangerous. You must avoid race conditions and you must understand that behavior may not be deterministic. This is especially true if you have multiple pre-solve callbacks that are sensitive to order.

### Sensors
In v2.4 sensor events were mixed in with contact events. I have split them up to make user code simpler.
```c
b2SensorEvents sensorEvents = b2World_GetSensorEvents(b2WorldId worldId);
```
Note that contact data on bodies and shapes have no information about sensors. That data only has touching contacts.

Sensor events are available to all shapes on dynamic bodies except chains. You can disable them using `enableSensorEvents` on `b2ShapeDef`.

### Queries
Version 2.4 has `b2World::QueryAABB` and `b2World::RayCast`. This functionality is largely the same in v3.0, but more features have been added such as precise overlap tests and shape casts.

Another new feature is `b2QueryFilter` which allows you to filter raycast results before they reach your callback.
This query filter is tested against `b2Filter` on shapes that the query encounters.

Ray casts now take an origin and translation rather than start and end points. This convention works better with the added shape cast functions.

### World iteration
Iterating over all bodies/shapes/joints/contacts in a world is very inefficient and has been removed from Version 3.0. Instead, you should be using `b2BodyEvents` and `b2ContactEvents`. Events are efficient and data-oriented.

### Library configuration
Version 3.0 offers more library configuration. You can override the allocator and you can intercept assertions by registering global callbacks. These are for expert users and they must be thread safe.
```c
void b2SetAllocator(b2AllocFcn* allocFcn, b2FreeFcn* freeFcn);
void b2SetAssertFcn(b2AssertFcn* assertFcn);
```
