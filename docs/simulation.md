# Simulation
Rigid body simulation is the primary feature of Box2D. It is the most complex part of
Box2D and is the part you will likely interact with the most. Simulation sits on top of
the foundation and collision types and functions, so you should be somewhat familiar
with those by now.

Rigid body simulation contains:
- worlds
- bodies
- shapes
- contacts
- joints
- events

There are many dependencies between these objects so it is difficult to
describe one without referring to another. In the following, you
may see some references to objects that have not been described yet.
Therefore, you may want to quickly skim this section before reading it
closely.

## Ids
Box2D has a C interface. Typically in a C/C++ library when you create an object with a long lifetime
you will keep a pointer (or smart pointer) to the object.

Box2D works differently. Instead of pointers, you are given an *id* when you create an object.
This *id* acts as a [handle](https://en.wikipedia.org/wiki/Handle_(computing)) and help avoid
problems with [dangling pointers](https://en.wikipedia.org/wiki/Dangling_pointer).

This also allows Box2D to use [data-oriented design](https://en.wikipedia.org/wiki/Data-oriented_design) internally.
This helps to reduce cache misses drastically and also allows for [SIMD](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data)
optimizations.

So you will be dealing with `b2WorldId`, `b2BodyId`, etc. These are small opaque structures that you
will pass around by value, just like pointers. Box2D creation functions return an id. Functions
that operate on Box2D objects take ids.

```c
b2BodyId myBodyId = b2CreateBody(myWorldId, &myBodyDef);
```

There are functions to check if an id is valid. Box2D functions will assert if you use an invalid id.
This makes debugging easier than using dangling pointers.

```c
if (b2Body_IsValid(myBodyId) == false)
{
    // oops
}
```

## World
The Box2D world contains the bodies and joints. It manages all aspects
of the simulation and allows for asynchronous queries (like AABB queries
and ray-casts). Much of your interactions with Box2D will be with a
world object, using `b2WorldId`.

### World Definition
Worlds are created using a *definition* structure. This is temporary structure that
you can use to configure options for world creation. You **must** initialize the world definition
using `b2DefaultWorldDef()`.

```c
b2WorldDef worldDef = b2DefaultWorldDef();
```

The world definition has lots of options, but for most you will use the defaults. You may want to set the gravity:

```c
worldDef.gravity = (b2Vec2){0.0f, -10.0f};
```

If your game doesn't need sleep, you can get a performance boost by completely disabling sleep:

```c
worldDef.enableSleep = false;
```

You can also configure multithreading to improve performance:

```c
worldDef.workerCount = 4;
worldDef.enqueueTask = myAddTaskFunction;
worldDef.finishTask = myFinishTaskFunction;
worldDef.userTaskContext = &myTaskSystem;
```

Multithreading is not required but it can improve performance substantially. Read more [here](#multi).

### World Lifetime
Creating a world is done using a world definition.

```c
b2WorldId myWorldId = b2CreateWorld(&worldDef);

// ... do stuff ...

b2DestroyWorld(myWorldId);

// Nullify id for safety
myWorldId = b2_nullWorldId;
```

You can create up to 128 worlds. These worlds do not interact and may be simulated in parallel.

When you destroy a world, every body, shape, and joint is also destroyed. This is much faster
than destroying individual objects.

### Simulation
The world is used to drive the simulation. You specify a time step
and a sub-step count. For example:

```c
float timeStep = 1.0f / 60.f;
int32_t subSteps = 10;
b2World_Step(myWorldId, timeStep, subSteps);
```

After the time step you can examine your bodies and joints for
information. Most likely you will grab the position off the bodies so
that you can update your game objects and render them. Or more optimally, you
will use `b2World_GetBodyEvents()`.

You can perform the time step anywhere in your game loop, but you should be aware of the
order of things. For example, you must create bodies before the time
step if you want to get collision results for the new bodies in that
frame.

As I discussed in the [HelloWorld tutorial](#hello), you should use a fixed
time step. By using a larger time step you can improve performance in
low frame rate scenarios. But generally you should use a time step no
larger than 1/30 seconds (30Hz). A time step of 1/60 seconds (60Hz) will usually
deliver a high quality simulation.

The sub-step count is used to increase accuracy. By sub-stepping the solver
divides up time into small increments and the bodies move by a small amount.
This allows joints and contacts to respond with finer detail. The recommended
sub-step count is 4. However, increasing the sub-step count may improve 
accuracy. For example, long joint chains will stretch less with more sub-steps.

The scissor lift sample shown [here](#samples) works better with more sub-steps
and is configured to use 8 sub-steps. With a primary time step of 1/60 seconds,
the scissor lift is taking sub-steps at 480Hz!

## Rigid Bodies
Rigid bodies, or just *bodies* have position and velocity. You can apply forces, torques,
and impulses to bodies. Bodies can be static, kinematic, or dynamic. Here
are the body type definitions:

### Body types
#b2_staticBody:
A static body does not move under simulation and behaves as if it has infinite mass.
Internally, Box2D stores zero for the mass and the inverse mass. A static body has zero
velocity. Static bodies do not collide with other static or kinematic bodies.

#b2_kinematicBody:
A kinematic body moves under simulation according to its velocity.
Kinematic bodies do not respond to forces. A kinematic body is moved by setting its
velocity. A kinematic body behaves as if it has infinite mass, however,
Box2D stores zero for the mass and the inverse mass. Kinematic bodies do
not collide with other kinematic or static bodies. Generally you should use
a kinematic body if you want a shape to be animated and not affected by
forces or collisions.

#b2_dynamicBody:
A dynamic body is fully simulated and moves according to forces and torques.
A dynamic body can collide with all body types. A dynamic body always has
finite, non-zero mass.

> **Caution**:
> Generally you should not set the transform on bodies after creation.
> Box2D treats this as a teleport and may result in undesirable behavior.

Bodies carry shapes and moves them around in the world. Bodies are always
rigid bodies in Box2D. That means that two shapes attached to the same rigid body never move
relative to each other and shapes attached to the same body don't
collide.

Shapes have collision geometry and density. Normally, bodies acquire
their mass properties from the shapes. However, you can override the
mass properties after a body is constructed.

You usually keep ids to all the bodies you create. This way you can
query the body positions to update the positions of your graphical
entities. You should also keep body ids so you can destroy them
when you are done with them.

### Body Definition
Before a body is created you must create a body definition (`b2BodyDef`).
The body definition holds the data needed to create and initialize a
body correctly.

Because Box2D uses a C API, a function is provided to create a default
body definition.

```c
b2BodyDef myBodyDef = b2DefaultBodyDef();
```

This ensures the body definition is valid and this initialization is **mandatory**.

Box2D copies the data out of the body definition; it does not keep a
pointer to the body definition. This means you can recycle a body
definition to create multiple bodies.

Let's go over some of the key members of the body definition.

### Body Type
As discussed previously, there are three different
body types: static, kinematic, and dynamic. b2_staticBody is the default.
You should establish the body type at creation because changing the body type
later is expensive.

```c
b2BodyDef bodyDef;
bodyDef.type = b2_dynamicBody;
```

### Position and Angle
You can initialize the body position and angle in the body definition. This has far
better performance than creating the body at the world origin and then moving the body.

> **Caution**:
> Do not create a body at the origin and then move it. If you create
> several bodies at the origin, then performance will suffer.

A body has two main points of interest. The first point is the body's
origin. Shapes and joints are attached relative to the body's origin.
The second point of interest is the center of mass. The center of mass
is determined from the mass distribution of the attached shapes or is
explicitly set with `b2MassData`. Much of Box2D's internal computations
use the center of mass position. For example the body stores the linear
velocity for the center of mass, not the body origin.

![Body Origin and Center of Mass](images/center_of_mass.svg)

When you are building the body definition, you may not know where the
center of mass is located. Therefore you specify the position of the
body's origin. You may also specify the body's angle in radians. If you later
change the mass properties of the body, then the center of mass may move
on the body, but the origin position and body angle does not change and the attached
shapes and joints do not move.

```c
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.position = (b2Vec2){0.0f, 2.0f};
bodyDef.angle = 0.25f * b2_pi;
```

A rigid body is a frame of reference. You can define shapes and
joints in that frame. Those shapes and joint anchors never move in the
local frame of the body.

### Damping
Damping is used to reduce the world velocity of bodies. Damping is
different than friction because friction only occurs with contact.
Damping is not a replacement for friction and the two effects are
used together.

Damping parameter are non-negative. Normally you will use a
damping value between 0 and 1. I generally do not use linear damping
because it makes bodies look like they are floating.

```c
bodyDef.linearDamping = 0.0f;
bodyDef.angularDamping = 0.1f;
```

Damping is approximated to improve performance. At small damping
values the damping effect is mostly independent of the time step. At
larger damping values, the damping effect will vary with the time step.
This is not an issue if you use a fixed time step (recommended).

Here's some math for the curious. A first-order different equation for velocity damping is:

\f[
\frac{dv}{dt} + c v = 0
\f]

The solution with initial velocity \f$v_0\f$ is
\f[
v = v_0 e^{-c t}
\f]

Across a single time step \f$h\f$ the velocity evolves like so
\f[
v(t + h) = v_0 e^{-c (t + h)} = v_0 e^{-c t} e^{-c h} = v(t) e^{-c h}
\f]

Using the [Pade approximation](https://en.wikipedia.org/wiki/Pad%C3%A9_table) for the
exponential function gives the update formula:
\f[
v(t + h) \approx \frac{1}{1 + c h} v(t)
\f]

This is the formula used in the Box2D solver.

### Gravity Scale
You can use the gravity scale to adjust the gravity on a single body. Be
careful though, a large gravity magnitude can decrease stability.

```c
// Set the gravity scale to zero so this body will float
bodyDef.gravityScale = 0.0f;
```

### Sleep Parameters
What does sleep mean? Well it is expensive to simulate bodies, so the
less we have to simulate the better. When a body comes to rest we would
like to stop simulating it.

When Box2D determines that a body (or group of bodies) has come to rest,
the body enters a sleep state which has very little CPU overhead. If a
body is awake and collides with a sleeping body, then the sleeping body
wakes up. Bodies will also wake up if a joint or contact attached to
them is destroyed. You can also wake a body manually.

The body definition lets you specify whether a body can sleep and
whether a body is created sleeping.

```c
bodyDef.enableSleep = true;
bodyDef.isAwake = true;
```

The `isAwake` flag is ignored if `enableSleep` is false.

### Fixed Rotation
You may want a rigid body, such as a character, to have a fixed
rotation. Such a body does not rotate, even under load. You can use
the fixed rotation setting to achieve this:

```c
bodyDef.fixedRotation = true;
```

The fixed rotation flag causes the rotational inertia and its inverse to
be set to zero.

### Bullets {#bullets}
Game simulation usually generates a sequence of transforms that are played
at some frame rate. This is called discrete simulation. In discrete
simulation, rigid bodies can move by a large amount in one time step. If
a physics engine doesn't account for the large motion, you may see some
objects incorrectly pass through each other. This effect is called
*tunneling*.

By default, Box2D uses continuous collision detection (CCD) to prevent
dynamic bodies from tunneling through static bodies. This is done by
sweeping shapes from their old position to their new positions. The
engine looks for new collisions during the sweep and computes the time
of impact (TOI) for these collisions. Bodies are moved to their first
TOI at the end of the time step.

Normally CCD is not used between dynamic bodies. This is done to keep
performance reasonable. In some game scenarios you need dynamic bodies
to use CCD. For example, you may want to shoot a high speed bullet at a
stack of dynamic bricks. Without CCD, the bullet might tunnel through
the bricks.

Fast moving objects in Box2D can be configured as *bullets*. Bullets will
perform CCD with all body types, but not other bullets. You should decide what
bodies should be bullets based on your game design. If you decide a body
should be treated as a bullet, use the following setting.

```c
bodyDef.isBullet = true;
```

The bullet flag only affects dynamic bodies. I recommend using bullets sparingly.

### Disabling
You may wish a body to be created but not participate in collision or
simulation. This state is similar to sleeping except the body will not be
woken by other bodies and the body's shapes will not collide with anything.
This means the body will not participate in collisions, ray
casts, etc.

You can create a body as disabled and later enable it.

```c
bodyDef.isEnabled = false;

// Later ...
b2Body_Enable(myBodyId);
```

Joints may be connected to disabled bodies. These joints will not be
simulated. You should be careful when you enable a body that its
joints are not distorted.

Note that enabling a body is almost as expensive as creating the body
from scratch. So you should not use body disabling for streaming worlds. Instead, use
creation/destruction for streaming worlds to save memory.

Body disabling is a convenience and is generally not good for performance.

### User Data
User data is a void pointer. This gives you a hook to link your
application objects to bodies. You should be consistent to use the same
object type for all body user data.

```c
bodyDef.userData = &myGameObject;
```

This is useful when you receive results from a query such as a ray-cast
or event and you want to get back to your game object. You can acquire the
use data from a body using `b2Body_GetUserData()`.

### Body Lifetime
Bodies are created and destroyed using a world id. This lets the world create
the body with an efficient allocator and add the body to the world data structure.

```c
b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);

// ... do stuff ...

b2DestroyBody(myBodyId);

// Nullify body id for safety
myBodyId = b2_nullBodyId;
```

Box2D does not keep a reference to the body definition or any of the
data it holds (except user data pointers). So you can create temporary
body definitions and reuse the same body definitions.

Box2D allows you to avoid destroying bodies by destroying the world
directly using `b2DestroyWorld()`, which does all the cleanup work for you.
However, you should be mindful to nullify body ids that you keep in your application.

When you destroy a body, the attached shapes and joints are
automatically destroyed. This has important implications for how you
manage shape and joint ids. You should nullify these ids after destroying
a body.

### Using a Body
After creating a body, there are many operations you can perform on the
body. These include setting mass properties, accessing position and
velocity, applying forces, and transforming points and vectors.

### Mass Data
A body has mass (scalar), center of mass (2-vector), and rotational
inertia (scalar). For static bodies, the mass and rotational inertia are
set to zero. When a body has fixed rotation, its rotational inertia is
zero.

Normally the mass properties of a body are established automatically
when shapes are added to the body. You can also adjust the mass of a
body at run-time. This is usually done when you have special game
scenarios that require altering the mass.

```c
b2MassData myMassData;
myMassData.mass = 10.0f;
myMassData.center = (b2Vec2){0.0f, 0.0f};
myMassData.rotationalInertia = 100.0f;
b2Body_SetMassData(myBodyId, myMassData);
```

After setting a body's mass directly, you may wish to revert to the 
mass determined by the shapes. You can do this with:

```c
b2Body_ApplyMassFromShapes(myBodyId);
```

The body's mass data is available through the following functions:

```c
float mass = b2Body_GetMass(myBodyId);
float inertia = b2Body_GetRotationalInertia(myBodyId);
b2Vec2 localCenter b2Body_GetLocalCenterOfMass(myBodyId);
b2MassData massData = b2Body_GetMassData(myBodyId);
```

### State Information
There are many aspects to the body's state. You can access this state
data through the following functions:

```c
b2Body_SetType(myBodyId, b2_kinematicBody);
b2BodyType bodyType = b2Body_GetType(myBodyId);
b2Body_SetBullet(myBodyId, true);
bool isBullet = b2Body_IsBullet(myBodyId);
b2Body_EnableSleep(myBodyId, false);
bool isSleepEnabled = b2Body_IsSleepingEnabled(myBodyId);
b2Body_SetAwake(myBodyId, true);
bool isAwake = b2Body_IsAwake(myBodyId);
b2Body_Disable(myBodyId);
b2Body_Enable(myBodyId);
bool isEnabled = b2Body_IsEnabled(myBodyId);
b2Body_SetFixedRotation(myBodyId, true);
bool isFixedRotation = b2Body_IsFixedRotation(myBodyId);
```

Please see the comments on these functions for more details.

### Position and Velocity
You can access the position and rotation of a body. This is common when
rendering your associated game object. You can also set the position and angle,
although this is less common since you will normally use Box2D to
simulate movement.

Keep in mind that the Box2D interface uses *radians*.

```c
b2Body_SetTransform(myBodyId, position, rotation);
b2Transform transform = b2Body_GetTransform(myBodyId);
b2Vec2 position = b2Body_GetPosition(myBodyId);
b2Rot rotation = b2Body_GetRotation(myBodyId);
float angleInRadians = b2Rot_GetAngle(rotation);
```

You can access the center of mass position in local and world
coordinates. Much of the internal simulation in Box2D uses the center of
mass. However, you should normally not need to access it. Instead you
will usually work with the body transform. For example, you may have a
body that is square. The body origin might be a corner of the square,
while the center of mass is located at the center of the square.

```c
b2Vec2 worldCenter = b2Body_GetWorldCenterOfMass(myBodyId);
b2Vec2 localCenter = b2Body_GetLocalCenterOfMass(myBodyId);
```

You can access the linear and angular velocity. The linear velocity is
for the center of mass. Therefore, the linear velocity may change if the
mass properties change. Since Box2D uses radians, the angular velocity is
in radians per second.

```c
b2Vec2 linearVelocity = b2Body_GetLinearVelocity(myBodyId);
float angularVelocity = b2Body_GetAngularVelocity(myBodyId);
```

### Forces and Impulses
You can apply forces, torques, and impulses to a body. When you apply a
force or an impulse, you can provide a world point where the load is
applied. This often results in a torque about the center of mass.

```c
b2Body_ApplyForce(myBodyId, force, worldPoint, wake);
b2Body_ApplyTorque(myBodyId, torque, wake);
b2Body_ApplyLinearImpulse(myBodyId, linearImpulse, worldPoint, wake);
b2Body_ApplyAngularImpulse(myBodyId, angularImpulse, wake);
```

Applying a force, torque, or impulse optionally wakes the body. If you don't
wake the body and it is asleep, then the force or impulse will be ignored.

You can also apply a force and linear impulse to the center of mass to avoid rotation.

```c
b2Body_ApplyForceToCenter(myBodyId, force, wake);
b2Body_ApplyLinearImpulseToCenter(myBodyId, linearImpulse, wake);
```

> **Caution**:
> Since Box2D uses sub-stepping, you should not apply a steady impulse
> for several frames. Instead you should apply a force which Box2D will
> spread out evenly across the sub-steps, resulting in smoother movement.

### Coordinate Transformations
The body has some utility functions to help you transform points
and vectors between local and world space. If you don't understand
these concepts, I recommend reading \"Essential Mathematics for Games and
Interactive Applications\" by Jim Van Verth and Lars Bishop.

```c
b2Vec2 worldPoint = b2Body_GetWorldPoint(myBodyId, localPoint);
b2Vec2 worldVector = b2Body_GetWorldVector(myBodyId, localVector);
b2Vec2 localPoint = b2Body_GetLocalPoint(myBodyId, worldPoint);
b2Vec2 localVector = b2Body_GetLocalVector(myBodyId, worldVector);
```

### Accessing Shapes and Joints
You can access the shapes on a body. You can get the number of shapes first.

```c
int shapeCount = b2Body_GetShapeCount(myBodyId);
```

If you have bodies with many shapes, you can allocate an array or if you
know the number is limited you can use a fixed size array.

```c
b2ShapeId shapeIds[10];
int returnCount = b2Body_GetShapes(myBodyId, shapeIds, 10);

for (int i = 0; i < returnCount; ++i)
{
    b2ShapeId shapeId = shapeIds[i];

    // do something with shapeId
}
```

You can similarly get an array of the joints on a body.

### Body Events
While you can gather transforms from all your bodies after every time step, this is inefficient.
Many bodies may not have moved because they are sleeping. Also iterating across many bodies
will have lots of cache misses.

Box2D provides `b2BodyEvents` that you can access after every call to `b2World_Step()` to get
an array of body movement events. Since this data is contiguous, it is cache friendly.

```c
b2BodyEvents events = b2World_GetBodyEvents(m_worldId);
for (int i = 0; i < events.moveCount; ++i)
{
    const b2BodyMoveEvent* event = events.moveEvents + i;
    MyGameObject* gameObject = event->userData;
    MoveGameObject(gameObject, event->transform);
    if (event->fellAsleep)
    {
        SleepGameObject(gameObject);
    }
}
```

The body event also indicates if the body fell asleep this time step. This might be useful to
optimize your application.

## Shapes
A body may have zero or more shapes. A body with multiple shapes is sometimes
called a *compound body.*

Shapes hold the following:
- a shape primitive
- density, friction, and restitution
- collision filtering flags
- parent body id
- user data
- sensor flag

These are described in the following sections.

### Shape Lifetime
Shapes are created by initializing a shape definition and a shape primitive.
These are passed to a creation function specific to each shape type.

```c
b2ShapeDef shapeDef = b2DefaultShapeDef();
shapeDef.density = 10.0f;
shapeDef.friction = 0.7f;

b2Polygon box = b2MakeBox(0.5f, 1.0f);
b2ShapeId myShapeId = b2CreatePolygonShape(myBodyId, &shapeDef, &box);
```

This creates a polygon and attaches it to the body. You do not need to
store the shape id since the shape will automatically be
destroyed when the parent body is destroyed. However, you may wish to store the shape id if you plan
to change properties on it later.

You can create multiple shapes on a single body. They all can contribute
to the mass of the body. These shapes never collide with each other and may overlap.

You can destroy a shape on the parent body. You may do this to model a
breakable object. Otherwise you can just leave the shape alone and let
the body destruction take care of destroying the attached shapes.

```c
b2DestroyShape(myShapeId);
```

Material properties such as density, friction, and restitution are associated with shapes
instead of bodies. Since you can attach multiple shapes to a body, this allows for more
possible setups. For example, you can make a car that is heavier in the back.

### Density
The shape density is used to compute the mass properties of the parent
body. The density can be zero or positive. You should generally use
similar densities for all your shapes. This will improve stacking
stability.

The mass of a body is not adjusted when you set the density. You must
call `b2Body_ApplyMassFromShapes()` for this to occur. Generally you should establish
the shape density in `b2ShapeDef` and avoid modifying it later because this
can be expensive, especially on a compound body.

```c
b2Shape_SetDensity(myShapeId, 5.0f);
b2Body_ApplyMassFromShapes(myBodyId);
```

### Friction
Friction is used to make objects slide along each other realistically.
Box2D supports static and dynamic friction, but uses the same parameter
for both. Box2D attempts to simulate friction accurately and the friction
strength is proportional to the normal force. This is called [Coulomb
friction](https://en.wikipedia.org/wiki/Friction). The friction parameter
is usually set between 0 and 1, but
can be any non-negative value. A friction value of 0 turns off friction
and a value of 1 makes the friction strong. When the friction force is
computed between two shapes, Box2D must combine the friction parameters
of the two parent shapes. This is done with the
[geometric mean](https://en.wikipedia.org/wiki/Geometric_mean):

```c
float mixedFriction = sqrtf(b2Shape_GetFriction(shapeIdA) * b2Shape_GetFriction(shapeIdB));
```

If one shape has zero friction then the mixed friction will be zero.

### Restitution
[Restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution) is used to make
objects bounce. The restitution value is
usually set to be between 0 and 1. Consider dropping a ball on a table.
A value of zero means the ball won't bounce. This is called an
*inelastic* collision. A value of one means the ball's velocity will be
exactly reflected. This is called a *perfectly elastic* collision.
Restitution is combined using the following formula.

```c
float mixedRestitution = b2MaxFloat(b2Shape_GetRestitution(shapeIdA), b2Shape_GetRestitution(shapeIdB));
```

Restitution is combined this way so that you can have a bouncy super
ball without having a bouncy floor.

When a shape develops multiple contacts, restitution is simulated
approximately. This is because Box2D uses a sequential solver. Box2D
also uses inelastic collisions when the collision velocity is small.
This is done to prevent jitter. See `b2WorldDef::restitutionThreshold`.

### Filtering {#filtering}
Collision filtering allows you to efficiently prevent collision between shapes.
For example, say you make a character that rides a bicycle. You want the
bicycle to collide with the terrain and the character to collide with
the terrain, but you don't want the character to collide with the
bicycle (because they must overlap). Box2D supports such collision
filtering using categories, masks, and groups.

Box2D supports 32 collision categories. For each shape you can specify
which category it belongs to. You can also specify what other categories
this shape can collide with. For example, you could specify in a
multiplayer game that players don't collide with each other. Rather
than identifying all the situations where things should not collide, I recommend
identifying all the situations where things should collide. This way you
don't get into situations where you are using 
[double negatives](https://en.wikipedia.org/wiki/Double_negative).
You can specify which things can collide using mask bits. For example:

```c
enum MyCategories
{
    PLAYER = 0x00000002,
    MONSTER = 0x00000004,
};

b2ShapeDef playerShapeDef = b2DefaultShapeDef();
b2ShapeDef monsterShapeDef = b2DefaultShapeDef();
playerShapeDef.filter.categoryBits = PLAYER;
monsterShapeDef.filter.categoryBits = MONSTER;

// Players collide with monsters, but not with other players
playerShapeDef.filter.maskBits = MONSTER;

// Monsters collide with players and other monsters
monsterShapeDef.filter.maskBits = PLAYER | MONSTER;
```

Here is the rule for a collision to occur:

```c
uint32_t catA = shapeA.filter.categoryBits;
uint32_t maskA = shapeA.filter.maskBits;
uint32_t catB = shapeB.filter.categoryBits;
uint32_t maskB = shapeB.filter.maskBits;

if ((catA & maskB) != 0 && (catB & maskA) != 0)
{
    // shapes can collide
}
```

Another filtering feature is *collision group*.
Collision groups let you specify a group index. You can have
all shapes with the same group index always collide (positive index)
or never collide (negative index). Group indices are usually used for
things that are somehow related, like the parts of a bicycle. In the
following example, shape1 and shape2 always collide, but shape3
and shape4 never collide.

```c
shape1Def.filter.groupIndex = 2;
shape2Def.filter.groupIndex = 2;
shape3Def.filter.groupIndex = -8;
shape4Def.filter.groupIndex = -8;
```

Collisions between shapes of different group indices are filtered
according the category and mask bits. If two shapes have the
same non-zero group index, then this overrides the category and mask.
Collision groups have a higher priority than categories and masks.

Note that additional collision filtering occurs automatically in Box2D. Here is a
list:
- A shape on a static body can only collide with a dynamic body.
- A shape on a kinematic body can only collide with a dynamic body.
- Shapes on the same body never collide with each other.
- You can optionally enable/disable collision between bodies connected by a joint.

Sometimes you might need to change collision filtering after a shape
has already been created. You can get and set the `b2Filter` structure on
an existing shape using `b2Shape_GetFilter()` and
`b2Shape_SetFilter()`. Changing the filter is expensive because
it causes contacts to be destroyed.

### Chain Shapes
The chain shape provides an efficient way to connect many line segments together
to construct your static game worlds. Chain shapes automatically
eliminate ghost collisions and provide one-sided collision.

If you don't care about ghost collisions, you can create a bunch of
two-sided segment shapes. The performance is similar.

The simplest way to use chain shapes is to create loops. Simply provide an
array of vertices.

```c
b2Vec2 points[4] = {
    {1.7f, 0.0f},
    {1.0f, 0.25f},
    {0.0f, 0.0f},
    {-1.7f, 0.4f}};

b2ChainDef chainDef = b2DefaultChainDef();
chainDef.points = points;
chainDef.count = 4;

b2ChainId myChainId = b2CreateChain(myBodyId, &chainDef);

// Later ...
b2DestroyChain(myChainId);

// Nullify id for safety
myChainId = b2_nullChainId;
```

The segment normal depends on the winding order. A counter-clockwise winding order orients the normal outwards and a clockwise winding order orients the normal inwards.

![Chain Shape Outwards Loop](images/chain_loop_outwards.svg)

![Chain Shape Inwards Loop](images/chain_loop_inwards.svg)

You may have a scrolling game world and would like to connect several chains together.
You can connect chains together using ghost vertices. To do this you must have the first three or last three points of each chain overlap. See the sample `ChainLink` for details.

![Chain Shape](images/chain_shape.svg)

Self-intersection of chain shapes is not supported. It might work, it
might not. The code that prevents ghost collisions assumes there are no
self-intersections of the chain. Also, very close vertices can cause
problems. Make sure all your points are more than than about a centimeter apart.

![Self Intersection is Bad](images/self_intersect.svg)

Each segment in the chain is created as a `b2ChainSegment` shape on the body. If you have the
shape id for a chain segment shape, you can get the owning chain id. This will return `b2_nullChainId`
if the shape is not a chain segment.

```c
b2ChainId chainId = b2SHape_GetParentChain(myShapeId);
```

You cannot create a chain segment shape directly.

### Sensors
Sometimes game logic needs to know when two shapes overlap yet there
should be no collision response. This is done by using sensors. A sensor
is a shape that detects overlap but does not produce a response.

You can flag any shape as being a sensor. Sensors may be static,
kinematic, or dynamic. Remember that you may have multiple shapes per
body and you can have any mix of sensors and solid shapes. Also,
sensors only form contacts when at least one body is dynamic, so you
will not get sensors overlap detection for kinematic versus kinematic,
kinematic versus static, or static versus static. Finally sensors do not
detect other sensors.

Sensors do not detect objects that pass through the sensor shape within 
one time step. If you have fast moving object and/or small sensors then you
should use a ray or shape cast to detect these events.

Sensor overlap detection is achieved using events, which are described
below.

## Contacts
Contacts are internal objects created by Box2D to manage collision between pairs of
shapes. They are fundamental to rigid body simulation in games.

### Terminology
Contacts have a fair bit of terminology that are important to review.

#### contact point
A contact point is a point where two shapes touch. Box2D approximates
contact with a small number of points. Specifically, contact between
two shapes has 0, 1, or 2 points. This is possible because Box2D uses
convex shapes.

#### contact normal
A contact normal is a unit vector that points from one shape to another.
By convention, the normal points from shapeA to shapeB.

#### contact separation
Separation is the opposite of penetration. Separation is negative when
shapes overlap.

#### contact manifold
Contact between two convex polygons may generate up to 2 contact points.
Both of these points use the same normal, so they are grouped into a
contact manifold, which is an approximation of a continuous region of
contact.

#### normal impulse
The normal force is the force applied at a contact point to prevent the
shapes from penetrating. For convenience, Box2D uses impulses. The
normal impulse is just the normal force multiplied by the time step. Since
Box2D uses sub-stepping, this is the sub-step time step.

#### tangent impulse
The tangent force is generated at a contact point to simulate friction.
For convenience, this is stored as an impulse.

#### contact point id
Box2D tries to re-use the contact impulse results from a time step as the
initial guess for the next time step. Box2D uses contact point ids to match
contact points across time steps. The ids contain geometric feature
indices that help to distinguish one contact point from another.

#### speculative contact
When two shapes are close together, Box2D will create up to two contact
points even if the shapes are not touching. This lets Box2D anticipate
collision to improve behavior. Speculative contact points have positive
separation.

### Contact Lifetime
Contacts are created when two shape's AABBs begin to overlap. Sometimes
collision filtering will prevent the creation of contacts. Contacts are
destroyed with the AABBs cease to overlap.

So you might gather that there may be contacts created for shapes that
are not touching (just their AABBs). Well, this is correct. It's a
\"chicken or egg\" problem. We don't know if we need a contact object
until one is created to analyze the collision. We could delete the
contact right away if the shapes are not touching, or we can just wait
until the AABBs stop overlapping. Box2D takes the latter approach
because it lets the system cache information to improve performance.

### Contact Data
As mentioned before, the contact is created and destroyed by
Box2D automatically. Contact data is not created by the user. However, you are
able to access the contact data.

You can get contact data from shapes or bodies. The contact data
on a shape is a sub-set of the contact data on a body. The contact
data is only returned for touching contacts. Contacts that are not
touching provide no meaningful information for an application.

Contact data is returned in arrays. So first you can ask a shape or
body how much space you'll need in your array. This number is conservative
and the actual number of contacts you'll receive may be less than
this number, but never more.

```c
int shapeContactCapacity = b2Shape_GetContactCapacity(myShapeId);
int bodyContactCapacity = b2Body_GetContactCapacity(myBodyId);
```

You could allocate array space to get all the contact data in all cases, or you could use a fixed size
array and get a limited number of results.

```c
b2ContactData contactData[10];
int shapeContactCount = b2Shape_GetContactData(myShapeId, contactData, 10);
int bodyContactCount = b2Body_GetContactData(myBodyId, contactData, 10);
```

`b2ContactData` contains the two shape ids and the manifold.

```c
for (int i = 0; i < bodyContactCount; ++i)
{
    b2ContactData* data = contactData + i;
    printf("point count = %d\n", data->manifold.pointCount);
}
```

Getting contact data off shapes and bodies is not the most efficient
way to handle contact data. Instead you should use contact events.

### Sensor Events
Sensor events are available after every call to `b2World_Step()`.
Sensor events are the best way to get information about sensors overlaps. There are
events for when a shape begins to overlap with a sensor.

```c
b2SensorEvents sensorEvents = b2World_GetSensorEvents(myWorldId);
for (int i = 0; i < sensorEvents.beginCount; ++i)
{
    b2SensorBeginTouchEvent* beginTouch = sensorEvents.beginEvents + i;
    void* myUserData = b2Shape_GetUserData(beginTouch->visitorShapeId);
    // process begin event
}
```

And there are events when a shape stops overlapping with a sensor. Be careful with end
touch events because they may be generated when shapes are destroyed. Test the shape
ids with `b2Shape_IsValid`.

```c
for (int i = 0; i < sensorEvents.endCount; ++i)
{
    b2SensorEndTouchEvent* endTouch = sensorEvents.endEvents + i;
    if (b2Shape_IsValid(endTouch->visitorShapeId))
    {
        void* myUserData = b2Shape_GetUserData(endTouch->visitorShapeId);
        // process end event
    }
}
```

Sensor events should be processed after the world step and before other game logic. This should
help you avoid processing stale data.

There are several user operations that can cause sensors to stop touching. Such operations
include:
- destroying a body or shape
- changing the filter on a shape
- disabling a body
- setting the body transform
These may generate end-touch events and these events are included with simulation events available
after the next call to `b2World_Step`. 

Sensor events are only enabled for a non-sensor shape if `b2ShapeDef::enableSensorEvents`
is true.

### Contact Events
Contact events are available after each world step. Like sensor events these should be
retrieved and processed before performing other game logic. Otherwise
you may be accessing orphaned/invalid data.

You can access all contact events in a single data structure. This is much more efficient
than using functions like `b2Body_GetContactData()`.

```c
b2ContactEvents contactEvents = b2World_GetContactEvents(myWorldId);
```

None of this data applies to sensors. All events involve at least one dynamic body.

There are three kinds of contact events:
1. Begin touch events
2. End touch events
3. Hit events

#### Contact Touch Event
`b2ContactBeginTouchEvent` is recorded when two shapes begin touching. These only
contain the two shape ids.

```c
for (int i = 0; i < contactEvents.beginCount; ++i)
{
    b2ContactBeginTouchEvent* beginEvent = contactEvents.beginEvents + i;
    ShapesStartTouching(beginEvent->shapeIdA, beginEvent->shapeIdB);
}
```

`b2ContactEndTouchEvent` is recorded when two shapes stop touching. These only
contain the two shape ids.

```c
for (int i = 0; i < contactEvents.endCount; ++i)
{
    b2ContactEndTouchEvent* endEvent = contactEvents.endEvents + i;

    // Use b2Shape_IsValid because a shape may have been destroyed
    if (b2Shape_IsValid(endEvent->shapeIdA) && b2Shape_IsValid(endEvent->shapeIdB))
    {
        ShapesStopTouching(endEvent->shapeIdA, endEvent->shapeIdB);
    }
}
```

Similar to `b2SensorEndTouchEvent`, `b2ContactEndTouchEvent` may be generated due to a user operation,
such as destroying a body or shape. These events are included with simulation events after the next `b2World_Step`.

Shapes only generate begin and end touch events if `b2ShapeDef::enableContactEvents` is true.

#### Hit Events
Typically in games you are mainly concerned about getting contact events for when
two shapes collide at a significant speed so you can play a sound and/or particle effect. Hit
events are the answer for this.

```c
for (int i = 0; i < contactEvents.hitCount; ++i)
{
    b2ContactHitEvent* hitEvent = contactEvents.hitEvents + i;
    if (hitEvent->approachSpeed > 10.0f)
    {
        // play sound
    }
}
```

Shapes only generate hit events if `b2ShapeDef::enableHitEvents` is true.
I recommend you only enable this for shapes that need hit events because
it creates some overhead. Box2D also only reports hit events that have an
approach speed larger than `b2WorldDef::hitEventThreshold`.

### Contact Filtering
Often in a game you don't want all objects to collide. For example, you
may want to create a door that only certain characters can pass through.
This is called contact filtering, because some interactions are filtered
out.

Contact filtering is setup on shapes and is covered [here](#filtering).

### Advanced Contact Handling

#### Custom Filtering Callback
For the best performance, use the contact filtering provided by `b2Filter`.
However, in some cases you may need custom filtering. You can do
this by registering a custom filter callback that implements `b2CustomFilterFcn()`.

```c
bool MyCustomFilter(b2ShapeId shapeIdA, b2ShapeId shapeIdB, void* context)
{
    MyGame* myGame = context;
    return myGame->WantsCollision(shapeIdA, shapeIdB);
}

// Elsewhere
b2World_SetCustomFilterCallback(myWorldId, MyCustomFilter, myGame);
```

This function must be [thread-safe](https://en.wikipedia.org/wiki/Thread_safety) and must not read from or write to the Box2D world. Otherwise you will get a [race condition](https://en.wikipedia.org/wiki/Race_condition). 

#### Pre-Solve Callback
This is called after collision detection, but before collision
resolution. This gives you a chance to disable the contact based on the contact geometry. For example, you can implement a one-sided platform using this callback.

The contact will be re-enabled each time through collision processing,
so you will need to disable the contact every time-step. This function must be thread-safe
and must not read from or write to the Box2D world.

```c
bool MyPreSolve(b2ShapeId shapeIdA, b2ShapeId shapeIdB, b2Manifold* manifold, void* context)
{
    MyGame* myGame = context;

    if (myGame->IsHittingBelowPlatform(shapeIdA, shapeIdB, manifold))
    {
        return false;
    }

    return true;
}

// Elsewhere
b2World_SetPreSolveCallback(myWorldId, MyPreSolve, myGame);
```

Note this currently does not work with high speed collisions, so you may see a
pause in those situations.

See the `Platformer` sample for more details.

## Joints
Joints are used to constrain bodies to the world or to each other.
Typical examples in games include ragdolls, teeters, and pulleys. Joints
can be combined in many different ways to create interesting motions.

Some joints provide limits so you can control the range of motion. Some
joints provide motors which can be used to drive the joint at a
prescribed speed until a prescribed force/torque is exceeded. And some
joints provide springs with damping.

Joint motors can be used in many ways. You can use motors to control
position by specifying a joint velocity that is proportional to the
difference between the actual and desired position. You can also use
motors to simulate joint friction: set the joint velocity to zero and
provide a small, but significant maximum motor force/torque. Then the
motor will attempt to keep the joint from moving until the load becomes
too strong.

### Joint Definition
Each joint type has an associated joint definition. All
joints are connected between two different bodies. One body may be static.
Joints between static and/or kinematic bodies are allowed, but have no
effect and use some processing time.

If a joint is connected to a disabled body, that joint is effectively disabled.
When the both bodies on a joint become enabled, the joint will automatically
be enabled as well. In other words, you do not need to explicitly enable
or disable a joint.

You can specify user data for any joint type and you can provide a flag
to prevent the attached bodies from colliding with each other. This is
the default behavior and you must set the `collideConnected`
Boolean to allow collision between two connected bodies.

Many joint definitions require that you provide some geometric data.
Often a joint will be defined by anchor points. These are points fixed
in the attached bodies. Box2D requires these points to be specified in
local coordinates. This way the joint can be specified even when the
current body transforms violate the joint constraint. Additionally, some joint
definitions need a reference angle between the bodies.
This may be necessary to constrain rotation correctly.

The rest of the joint definition data depends on the joint type. I
cover these below.

### Joint Lifetime
Joints are created using creation functions supplied for each joint type. They are destroyed
with a shared function. All joint types share a single id type `b2JointId`.

Here's an example of the lifetime of a revolute joint:

```c
b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
jointDef.bodyIdA = myBodyA;
jointDef.bodyIdB = myBodyB;
jointDef.localAnchorA = (b2Vec2){0.0f, 0.0f};
jointDef.localAnchorB = (b2Vec2){1.0f, 2.0f};

b2JointId myJointId = b2CreateRevoluteJoint(myWorldId, &jointDef);

// ... do stuff ...

b2DestroyJoint(myJointId);
myJointId = b2_nullJointId;
```

It is always good to nullify your ids after they are destroyed. 

Joint lifetime is related to body lifetime. Joints cannot exist detached from a body. 
So when a body is destroyed, all joints attached to that body are automatically destroyed.
This means you need to be careful to avoid using joint ids when the attached body was
destroyed. Box2D will assert if you use a dangling joint id.

> **Caution**:
> Joints are destroyed when an attached body is destroyed.

Fortunately you can check if your joint id is valid.

```c
if (b2Joint_IsValid(myJointId) == false)
{
    myJointId = b2_nullJointId;
}
```

This is certainly useful, but should not be overused because if you are creating
and destroying many joints, this may eventually alias to a different joint. All ids have
a limit of 64k generations.

### Using Joints
Many simulations create the joints and don't access them again until
they are destroyed. However, there is a lot of useful data contained in
joints that you can use to create a rich simulation.

First of all, you can get the type, bodies, anchor points, and user data from
a joint.

```c
b2JointType jointType = b2Joint_GetType(myJointId);
b2BodyId bodyIdA = b2Joint_GetBodyA(myJointId);
b2BodyId bodyIdB = b2Joint_GetBodyB(myJointId);
b2Vec2 localAnchorA = b2Joint_GetLocalAnchorA(myJointId);
b2Vec2 localAnchorB = b2Joint_GetLocalAnchorB(myJointId);
void* myUserData = b2Joint_GetUserData(myJointId);
```

All joints have a reaction force and torque. Reaction forces are
related to the [free body diagram](https://en.wikipedia.org/wiki/Free_body_diagram).
The Box2D convention is that the reaction force
is applied to body B at the anchor point. You can use reaction forces to
break joints or trigger other game events. These functions may do some
computations, so don't call them if you don't need the result.

```c
b2Vec2 force = b2Joint_GetConstraintForce(myJointId);
float torque = b2Joint_GetConstraintTorque(myJointId);
```

See the sample `BreakableJoint` for more details.

### Distance Joint
One of the simplest joints is a distance joint which says that the
distance between two points on two bodies must be constant. When you
specify a distance joint the two bodies should already be in place. Then
you specify the two anchor points in local coordinates. The first anchor
point is connected to body A, and the second anchor point is connected
to body B. These points imply the length of the distance constraint.

![Distance Joint](images/distance_joint.svg)

Here is an example of a distance joint definition. In this case I
decided to allow the bodies to collide.

```c
b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
jointDef.bodyIdA = myBodyIdA;
jointDef.bodyIdB = myBodyIdB;
jointDef.localAnchorA = (b2Vec2){1.0f, -3.0f};
jointDef.localAnchorB = (b2Vec2){0.0f, 0.5f};
b2Vec2 anchorA = b2Body_GetWorldPoint(myBodyIdA, jointDef.localAnchorA);
b2Vec2 anchorB = b2Body_GetWorldPoint(myBodyIdB, jointDef.localAnchorB);
jointDef.length = b2Distance(anchorA, anchorB);
jointDef.collideConnected = true;

b2JointId myJointId = b2CreateDistanceJoint(myWorldId, &jointDef);
```

The distance joint can also be made soft, like a spring-damper
connection. Softness is achieved by enabling the spring and tuning two values in the definition:
Hertz and damping ratio.

```c
jointDef.enableSpring = true;
jointDef.hertz = 2.0f;
jointDef.dampingRatio = 0.5f;
```

The hertz is the frequency of a [harmonic oscillator](https://en.wikipedia.org/wiki/Harmonic_oscillator) (like a
guitar string). Typically the frequency
should be less than a half the frequency of the time step. So if you are using
a 60Hz time step, the frequency of the distance joint should be less than 30Hz.
The reason is related to the [Nyquist frequency](https://en.wikipedia.org/wiki/Nyquist_frequency).

The damping ratio controls how fast the oscillations dissipate. A damping
ratio of one is [critical damping](https://en.wikipedia.org/wiki/Damping) and prevents
oscillation.

It is also possible to define a minimum and maximum length for the distance joint.
You can even motorize the distance joint to adjust its length dynamically.
See `b2DistanceJointDef` and the `DistanceJoint` sample for details.

### Revolute Joint
A revolute joint forces two bodies to share a common anchor point, often
called a hinge point or pivot. The revolute joint has a single degree of freedom:
the relative rotation of the two bodies. This is called the joint angle.

![Revolute Joint](images/revolute_joint.svg)

Like all joints, the anchor points are specified in local coordinates.
However, you can use the body utility functions to simplify this.

```c
b2Vec2 worldPivot = {10.0f, -4.0f};
b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
jointDef.bodyIdA = myBodyIdA;
jointDef.bodyIdB = myBodyIdB;
jointDef.localAnchorA = b2Body_GetLocalPoint(myBodyIdA, worldPivot);
jointDef.localAnchorB = b2Body_GetLocalPoint(myBodyIdB, worldPivot);

b2JointId myJointId = b2CreateRevoluteJoint(myWorldId, &jointDef);
```

The revolute joint angle is positive when bodyB rotates counter-clockwise
about the
anchor point. Like all angles in Box2D, the revolute angle is measured in
radians. By convention the revolute joint angle is zero when the two bodies
have equal angles. You can offset this using `b2RevoluteJointDef::referenceAngle`.

In some cases you might wish to control the joint angle. For this, the
revolute joint can simulate a joint limit and/or a motor.

A joint limit forces the joint angle to remain between a lower and upper
angle. The limit will apply as much torque as needed to make this
happen. The limit range should include zero, otherwise the joint will
lurch when the simulation begins. The lower and upper limit are relative to
the reference angle.

A joint motor allows you to specify the joint speed. The speed can be negative or
positive. A motor can have infinite force, but this is usually not desirable. Recall the eternal
question:

> *What happens when an irresistible force meets an immovable object?*

I can tell you it's not pretty. So you can provide a maximum torque for
the joint motor. The joint motor will maintain the specified speed
unless the required torque exceeds the specified maximum. When the
maximum torque is exceeded, the joint will slow down and can even
reverse.

You can use a joint motor to simulate joint friction. Just set the joint
speed to zero, and set the maximum torque to some small, but significant
value. The motor will try to prevent the joint from rotating, but will
yield to a significant load.

Here's a revision of the revolute joint definition above; this time the
joint has a limit and a motor enabled. The motor is setup to simulate
joint friction.

```c
b2Vec2 worldPivot = {10.0f, -4.0f};
b2RevoluteJointDef jointDef = b2DefaultRevoluteJointDef();
jointDef.bodyIdA = myBodyIdA;
jointDef.bodyIdB = myBodyIdB;
jointDef.localAnchorA = b2Body_GetLocalPoint(myBodyIdA, worldPivot);
jointDef.localAnchorB = b2Body_GetLocalPoint(myBodyIdB, worldPivot);
jointDef.lowerAngle = -0.5f * b2_pi; // -90 degrees
jointDef.upperAngle = 0.25f * b2_pi; // 45 degrees
jointDef.enableLimit = true;
jointDef.maxMotorTorque = 10.0f;
jointDef.motorSpeed = 0.0f;
jointDef.enableMotor = true;
```
You can access a revolute joint's angle, speed, and motor torque.

```c
float angleInRadians = b2RevoluteJoint_GetAngle(myJointId);
float speed = b2RevoluteJoint_GetMotorSpeed(myJointId);
float currentTorque = b2RevoluteJoint_GetMotorTorque(myJointId);
```

You also update the motor parameters each step.

```c
b2RevoluteJoint_SetMotorSpeed(myJointId, 20.0f);
b2RevoluteJoint_SetMaxMotorTorque(myJointId, 100.0f);
```

Joint motors have some interesting abilities. You can update the joint
speed every time step so you can make the joint move back-and-forth like
a sine-wave or according to whatever function you want.

```c
// ... Game Loop Begin ...

b2RevoluteJoint_SetMotorSpeed(myJointId, cosf(0.5f * time));

// ... Game Loop End ...
```

You can also use joint motors to track a desired joint angle. For example:

```c
// ... Game Loop Begin ...

float angleError = b2RevoluteJoint_GetAngle(myJointId) - angleTarget;
float gain = 0.1f;
b2RevoluteJoint_SetMotorSpeed(myJointId, -gain * angleError);

// ... Game Loop End ...
```

Generally your gain parameter should not be too large. Otherwise your
joint may become unstable.

### Prismatic Joint
A prismatic joint allows for relative translation of two bodies along a
local axis. A prismatic joint prevents relative rotation. Therefore,
a prismatic joint has a single degree of freedom.

![Prismatic Joint](images/prismatic_joint.svg)

The prismatic joint definition is similar to the revolute joint
description; just substitute translation for angle and force for torque.
Using this analogy provides an example prismatic joint definition with a
joint limit and a friction motor:

```c
b2Vec2 worldPivot = {10.0f, -4.0f};
b2Vec2 worldAxis = {1.0f, 0.0f};
b2PrismaticJointDef jointDef = b2DefaultPrismaticJointDef();
jointDef.bodyIdA = myBodyIdA;
jointDef.bodyIdB = myBodyIdB;
jointDef.localAnchorA = b2Body_GetLocalPoint(myBodyIdA, worldPivot);
jointDef.localAnchorB = b2Body_GetLocalPoint(myBodyIdB, worldPivot);
jointDef.localAxisA = b2Body_GetLocalVector(myBodyIdA, worldAxis);
jointDef.lowerTranslation = -5.0f;
jointDef.upperTranslation = 2.5f;
jointDef.enableLimit = true;
jointDef.maxMotorForce = 1.0f;
jointDef.motorSpeed = 0.0f;
jointDef.enableMotor = true;
```

The revolute joint has an implicit axis coming out of the screen. The
prismatic joint needs an explicit axis parallel to the screen. This axis
is fixed in body A.

The prismatic joint translation is zero when the anchor points overlap. I recommend
to have the prismatic anchor points close to the center of mass of the two bodies.
This will improve joint stiffness.

Using a prismatic joint is similar to using a revolute joint. Here are
the relevant member functions:

```c
float PrismaticJoint::GetJointTranslation() const;
float PrismaticJoint::GetJointSpeed() const;
float PrismaticJoint::GetMotorForce() const;
void PrismaticJoint::SetMotorSpeed(float speed);
void PrismaticJoint::SetMotorForce(float force);
```

### Mouse Joint
The mouse joint is used in the samples to manipulate bodies with the
mouse. It attempts to drive a point on a body towards the current
position of the cursor. There is no restriction on rotation.

The mouse joint definition has a target point, maximum force, Hertz,
and damping ratio. The target point initially coincides with the body's
anchor point. The maximum force is used to prevent violent reactions
when multiple dynamic bodies interact. You can make this as large as you
like. The frequency and damping ratio are used to create a spring/damper
effect similar to the distance joint.

### Weld Joint
The weld joint attempts to constrain all relative motion between two
bodies. See the `Cantilever` sample to see how the weld joint
behaves.

It is tempting to use the weld joint to define breakable structures.
However, the Box2D solver is approximate so the joints can be soft in some
cases regardless of the joint settings. So chains of bodies connected by weld
joints may flex.

See the `ContactEvent` sample for an example of how to merge and split bodies
without using the weld joint.

### Motor Joint
A motor joint lets you control the motion of a body by specifying target
position and rotation offsets. You can set the maximum motor force and
torque that will be applied to reach the target position and rotation.
If the body is blocked, it will stop and the contact forces will be
proportional the maximum motor force and torque. See `b2MotorJointDef` and
the `MotorJoint` sample for details.

### Wheel Joint
The wheel joint restricts a point on bodyB to a line on bodyA. The wheel
joint also provides a suspension spring and a motor. See the `Driving` sample
for details.

![Wheel Joint](images/wheel_joint.svg)

The wheel joint is designed specifically for vehicles. It provides a translation
and rotation. The translation has a spring and damper to simulate the vehicle
suspension. The rotation allows the wheel to rotate. You can specify an rotational
motor to drive the wheel and to apply braking. See `b2WheelJointDef` and the `Drive`
sample for details.

You may also use the wheel joint where you want free rotation and translation along
an axis. See the `ScissorLift` sample for details.

## Spatial Queries {#spatial}
Spatial queries allow you to inspect the world geometrically. There are overlap queries,
ray-casts, and shape-casts. These allow you to do things like:
- find a treasure chest near the player
- shoot a laser beam and destroy all asteroids in the path
- throw a grenade that is represented as a circle moving along a parabolic path

### Overlap Queries
Sometimes you want to determine all the shapes in a region. The world has a fast
log(N) method for this using the broad-phase data structure. Box2D provides these
overlap tests:
- axis-aligned bound box (AABB) overlap
- circle overlap
- capsule overlap
- polygon overlap

#### Query Filtering
A basic understanding of query filtering is needed before considering the specific queries.
Shape versus shape filtering was discussed [here](#filtering). A similar setup is used
for queries. This lets your queries only consider certain categories of shapes, it also
lets your shapes ignore certain queries.

Just like shapes, queries themselves can have a category. For example, you can have a `CAMERA`
or `PROJECTILE` category.

```c
enum MyCategories
{
    STATIC = 0x00000001,
    PLAYER = 0x00000002,
    MONSTER = 0x00000004,
    WINDOW = 0x00000008,
    CAMERA = 0x00000010,
    PROJECTILE = 0x00000020,
};

// Grenades collide with the static world, monsters, and windows but
// not players or other projectiles.
b2QueryFilter grenadeFilter;
grenadeFilter.categoryBits = PROJECTILE;
grenadeFilter.maskBits = STATIC | MONSTER | WINDOW;

// The view collides with the static world, monsters, and players.
b2QueryFilter viewFilter;
viewFilter.categoryBits = CAMERA;
viewFilter.maskBits = STATIC | PLAYER | MONSTER;
```

If you want to query everything you can use `b2DefaultQueryFilter()`;

#### AABB Overlap
You provide an AABB in world coordinates and an
implementation of `b2OverlapResultFcn()`. The world calls your function with each
shape whose AABB overlaps the query AABB. Return true to continue the
query, otherwise return false. For example, the following code finds all
the shapes that potentially intersect a specified AABB and wakes up
all of the associated bodies.

```c
bool MyOverlapCallback(b2ShapeId shapeId, void* context)
{
    b2BodyId bodyId = b2Shape_GetBody(shapeId);
    b2Body_SetAwake(bodyId, true);

    // Return true to continue the query.
    return true;
}

// Elsewhere ...
MyOverlapCallback callback;
b2AABB aabb;
aabb.lowerBound = (b2Vec2){-1.0f, -1.0f};
aabb.upperBound = (b2Vec2){1.0f, 1.0f};
b2QueryFilter filter = b2DefaultQueryFilter();
b2World_OverlapAABB(myWorldId, aabb, filter, MyOverlapCallback, &myGame);
```

Do not make any assumptions about the order of the callback. The order shapes
are returned to your callback may seem arbitrary.

#### Shape Overlap
The AABB overlap is very fast but not very accurate because it only considers
the shape bounding box. If you want an accurate overlap test, you can use a shape
overlap query. For example, here is how you can get all shapes that overlap
with a query circle.

```c
b2Circle circle = {b2Vec2_zero, 0.2f};
b2World_OverlapCircle(myWorldId, &circle, b2Transform_identity, grenadeFilter, MyOverlapCallback, &myGame);
```

### Ray-casts
You can use ray-casts to do line-of-sight checks, fire guns, etc. You
perform a ray-cast by implementing the `b2CastResultFcn()` callback
function and providing the
origin point and translation. The world calls your function with each shape
hit by the ray. Your callback is provided with the shape, the point of
intersection, the unit normal vector, and the fractional distance along
the ray. You cannot make any assumptions about the order of the points
sent to the callback. The callback may receive points that are further away
before receiving points that are closer.

You control the continuation of the ray-cast by returning a fraction.
Returning a fraction of zero indicates the ray-cast should be
terminated. A fraction of one indicates the ray-cast should continue as
if no hit occurred. If you return the fraction from the argument list,
the ray will be clipped to the current intersection point. So you can
ray-cast any shape, ray-cast all shapes, or ray-cast the closest shape
by returning the appropriate fraction.

You may also return of fraction of -1 to filter the shape. Then the
ray-cast will proceed as if the shape does not exist.

Here is an example:

```c
// This struct captures the closest hit shape
struct MyRayCastContext
{
    b2ShapeId shapeId;
    b2Vec2 point;
    b2Vec2 normal;
    float fraction;
};

float MyCastCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
    MyRayCastContext* myContext = context;
    myContext->shape = shape;
    myContext->point = point;
    myContext->normal = normal;
    myContext->fraction = fraction;
    return fraction;
}

// Elsewhere ...
MyRayCastContext context = {0};
b2Vec2 origin = {-1.0f, 0.0f};
b2Vec2 end(3.0f, 1.0f);
b2Vec2 translation = b2Sub(end, origin);
b2World_CastRay(myWorldId, origin, translation, viewFilter, MyCastCallback, &context);
```

Ray-cast results may be delivered in an arbitrary order. This doesn't affect the result for closest point ray-casts (except in ties). When you are collecting multiple hits along the ray, you may want to sort them according to the hit fraction. See the `RayCastWorld` sample for details.

### Shape-casts
Shape-casts are similar to ray-casts. You can view a ray-cast as tracing a point along a line. A shape-cast
allows you to trace a shape along a line. Since shapes can have rotation, you provide an origin transform instead of an origin point.

```c
// This struct captures the closest hit shape
struct MyRayCastContext
{
    b2ShapeId shapeId;
    b2Vec2 point;
    b2Vec2 normal;
    float fraction;
};

float MyCastCallback(b2ShapeId shapeId, b2Vec2 point, b2Vec2 normal, float fraction, void* context)
{
    MyRayCastContext* myContext = context;
    myContext->shape = shape;
    myContext->point = point;
    myContext->normal = normal;
    myContext->fraction = fraction;
    return fraction;
}

// Elsewhere ...
MyRayCastContext context = {0};
b2Circle circle = {b2Vec2_zero, {0.05f}};
b2Transform originTransform;
originTransform.p = (b2Vec2){-1.0f, 0.0f};
originTransform.q = b2Rot_identity;
b2Vec2 translation = {10.0f, -5.0f};
b2World_CastCircle(myWorldId, &circle, originTransform, translation, grenadeFilter, MyCastCallback, &context);
```

Otherwise, shape-casts are setup identically to ray-casts. You can expect shape-casts to generally be slower
than ray-casts. So only use a shape-cast if a ray-cast won't do.

Just like ray-casts, shape-casts results may be sent to the callback in any order. If you need multiple sorted results, you will need to write some code to collect and sort the results.
