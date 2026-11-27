# Best Practices {#best-practices}

This page collects the advice that comes up most often in issues and discussions. Each item gives a
rule, a short reason, and a link to the full reference. Read it before you build your game on Box2D.
If something already looks wrong, start with [Known Limitations](@ref known-limitations).

## Units and scale {#units-and-scale}

Box2D is tuned for meters, kilograms, and seconds. Moving shapes should be roughly between 0.1 and
20 meters. Static shapes can be up to 500 meters long. Split large static geometry across several
bodies. Keep your world within 16 kilometers of the origin or use [large world
support](@ref large-worlds).

Don't use pixels as meters. A 200 pixel sprite would be the size of a building and everything will
look slow and floaty. Convert between meters and pixels when you draw, or call
`b2SetLengthUnitsPerMeter()` before creating a world. See [Units](@ref units).

The box helpers take half extents. This makes a box that is 1 meter wide and 2 meters tall:

```c
b2Polygon box = b2MakeBox(0.5f, 1.0f);
```

If your sprites and debug drawing disagree, check for full extents passed as half extents.

## Time step {#time-step}

Use a fixed time step, such as 60Hz, with 4 sub-steps. Driving the step with a variable frame time
makes the simulation depend on the frame rate. Accumulate the frame time and take as many fixed steps
as fit. Read events after every step, not once per frame, because each step resets the event arrays.

```c
accumulator += frameTime;
while (accumulator >= timeStep)
{
    b2World_Step(myWorldId, timeStep, 4);
    ProcessEvents(myWorldId);
    accumulator -= timeStep;
}
```

More sub-steps make joints and stacks stiffer. See [Simulation](@ref world-step).

Also, I highly recommend putting your entire game simulation inside the accumulator. This makes
all game simulation time step independent, potentially deterministic, and reduces bugs. For example:

```c
accumulator += frameTime;
while (accumulator >= timeStep)
{
    GameLogic();
    Animation();
    b2World_Step(myWorldId, timeStep, 4);
    ProcessEvents(myWorldId);
    accumulator -= timeStep;
}
```

## Creating bodies {#creating-bodies}

Set the body type, position, and rotation in the body definition. Creating bodies at the origin and
then moving them makes every new shape overlap every other new shape, and the broad-phase becomes
\f$O(n^2)\f$. Changing the body type later is also expensive. See [Position and
angle](@ref body-position).

Avoid `b2Body_SetTransform()` during play. It is a teleport. The body arrives with no velocity
history and may land inside other shapes. To move a body along a path, drive a kinematic body with
`b2Body_SetTargetTransform()`.

Keep shapes near their body origin. Mass and rotational inertia are computed in the body frame. A
small polygon placed far from its body origin loses precision and can produce invalid inertia. For
static geometry in a large world, create several bodies near the geometry they hold instead of one
body at the world origin. This reduces floating point errors.

Give every shape on a dynamic body a positive density, and keep densities similar across touching
bodies. A dynamic body needs a finite, non-zero mass.

If you add many shapes to one body, set `b2ShapeDef::updateBodyMass` to false and call
`b2Body_UpdateMassFromShapes()` once afterwards. You must do this, or call `b2Body_SetMassData()`,
before the next step. See [Mass data](@ref mass-data).

## Compound bodies {#compound-bodies}

Pieces that never move relative to each other belong on one body as several shapes. This is a
compound body. Shapes on the same body are perfectly rigid. Weld joints between bodies are soft and
will flex, especially in long chains. For breakable objects, build a compound body and split it when
it breaks. The `ContactEvent` sample shows how. See [Weld joint](@ref weld-joint).

A concave dynamic object must be a compound body made of convex shapes. Each pair of shapes collides
independently, so another body can get wedged in a narrow gap between two shapes, even shapes on the
same body. Overlap or extend the shapes so no thin gap is left. For example, extend the legs of a
table up into the table top. Approximate collision is usually good enough and cheaper than an exact
decomposition.

Prefer fat shapes to long skinny ones. Rotational inertia grows faster with size than with mass, so
a thin body spins easily and is harder for collision. If a part spins too much or won't sleep, make
it larger rather than lighter.

## Joints {#joint-practice}

Keep the mass ratio between connected bodies below about 10 to 1. A heavy body hanging from a light
chain stretches the chain, and a light body squeezed between heavy bodies jitters. More sub-steps
help. Reducing the ratio helps more.

The bad case is a heavy body supported by a light one. A light body resting on a heavy one is fine.
Contacts work the same way, so put the heavy boxes at the bottom of a stack. Avoid joining two large
bodies through a small one.

Tune joint springs with a frequency in Hertz and a damping ratio instead of a stiffness. These don't
depend on the masses, so you can tune them without knowing the masses, and the spring stays stable at
any setting. A damping ratio of 1 removes oscillation. Don't build a stiff spring by applying forces
yourself. A spring made from forces needs at least 4 time steps per oscillation or it blows up.

A kinematic body that drags jointed dynamic bodies, such as a ragdoll attached to an animated
character, tears the joints if it moves too fast. Limit its speed. For a large jump, teleport the
kinematic body and the attached dynamic bodies together.

Rigs where several joints pull against a heavy body can become unstable. Lowering
`b2JointDef::constraintHertz` from the default 60 to around 30 softens the joints and usually
settles them.

Don't set `collideConnected` on welded bodies that overlap. The weld and the contacts fight each
other.

To stop a body from rotating, use [fixed rotation](@ref fixed-rotation). To pin a body to a point
or a line, use a [motion lock joint](@ref motion-lock-joint). Don't give a body zero mass to stop it
from translating.

## Ragdolls {#ragdolls}

Overlap the shapes at each joint, such as the upper and lower arm at the elbow. A gap lets other
objects get stuck in it and puts large joint forces on small bodies.

Jointed bodies don't collide with each other by default, but parts that aren't directly connected
still do. Overlapping parts, such as the two thighs, then fight the joints and the ragdoll jitters.
Give every shape in a ragdoll the same negative group index to turn off self collision, or filter the
pairs that overlap. The `Ragdoll` sample does this.

Small parts such as hands and feet add little and tend to get stuck in the ground. Leave them out
unless you need them. When you switch a character from animation to ragdoll, set the body velocities
from the animation or the transition will pop.

## Terrain and level geometry {#terrain}

Build ground from chain shapes. A row of boxes or separate segments makes sliding shapes snag on the
internal corners. Chains remove these [ghost collisions](@ref ghost-collisions). Use capsules or
rounded polygons for characters, since sharp boxes can still catch on small bumps.

Chains are one-sided. A counter-clockwise loop collides on the outside. Shapes approaching from the
back side pass through. If you need two-sided collision, use segment shapes. See [Chain
shapes](@ref chain-shapes).

Keep chain points more than about a centimeter apart, and don't let a chain cross itself. For a
loop, don't repeat the first point at the end because the closing segment is added for you.

The chain copies its points during `b2CreateChain()`, so you may free your point array afterwards.
To change chain geometry, destroy and recreate the chain. For streaming or destructible terrain, use
orphaned chain segments with `b2CreateChainSegmentShape()`.

Avoid stacking many overlapping static shapes in one spot. Every static shape a dynamic body
overlaps adds a contact.

Use `b2ComputeChains()` to merge many polygons into efficient chain shapes. This is especially
useful for tile-based geometry.

## Fast objects {#fast-objects}

Continuous collision keeps dynamic bodies from tunneling through static bodies by default. Dynamic
bodies can still tunnel through other dynamic bodies. Mark the few bodies that need it as bullets
with `b2BodyDef::isBullet`. See [Bullets](@ref bullets).

A speed limit is a cheap alternative to bullets. A body that moves much less than the thickness of
the shapes it hits in one step can't pass through them. `b2WorldDef::maximumLinearSpeed` caps the
speed of every body and defaults to 400 meters per second.

For projectiles that only need to report a hit, such as gunfire, use a ray cast or shape cast each
step instead of a body. Don't use a sensor because sensors are not swept and fast objects skip
past them. See [Ray-casts](@ref ray-casts).

## Sleep and waking {#sleep-and-waking}

Sleeping bodies cost almost nothing. Several functions do not wake bodies:

- `b2Body_SetTransform()`
- joint setters, such as motor speed, target angle, and spring settings
- force and impulse functions when `wake` is false

A sleeping body ignores forces and impulses. After changing a joint, call `b2Joint_WakeBodies()`.
After a teleport, call `b2Body_SetAwake()` if the body should start moving. If a body must never
sleep, set `b2BodyDef::enableSleep` to false. See [Sleep parameters](@ref sleep).

Bodies sleep in islands. An island is a group of bodies connected by touching contacts and joints,
and it sleeps only when every body in it is at rest. One body that keeps moving, such as a ball
rolling on top of a pile, keeps the whole pile awake. Static bodies don't join islands, so the ground
doesn't link everything together. Rolling resistance in `b2SurfaceMaterial` helps circles and
capsules come to rest.

## Events {#events-practice}

Events are off by default. Enable only what you need to improve performance:

- `b2ShapeDef::enableContactEvents` for begin and end touch, on at least one of the two shapes
- `b2ShapeDef::enableHitEvents` for impacts above `b2WorldDef::hitEventThreshold`
- `b2ShapeDef::enableSensorEvents` on **both** the sensor and the visiting shape

Process events right after each step and before other game logic. End events can refer to shapes
that were destroyed, so check ids with `b2Shape_IsValid()`. Compare ids with `B2_ID_EQUALS`.

Begin touch events are recorded before the solver runs, so they carry no impulses. To get contact
forces, use hit events, or read `b2Shape_GetContactData()` after the step for the shapes named in
the events. Ask the smaller shape, not the ground, because the ground may have hundreds of
contacts. See [Contact events](@ref contact-events).

## Callbacks and user data {#callbacks-practice}

Don't create, destroy, or modify bodies, shapes, or joints inside a callback. This applies to the
pre-solve, custom filter, and friction and restitution callbacks, which run during the step on
worker threads. It also applies to query and cast callbacks. Collect what you need and act after the
callback returns.

Disabling a contact in pre-solve lasts for one step. Keep the state of a one-way platform in your
game and decide again each step. See [Pre-solve callback](@ref pre-solve).

Treat ids as opaque handles. Don't read or construct their fields.

Store an entity id or a pointer to a long-lived game object in user data. Then there is nothing for
Box2D to free and no lifetime to manage.

## Changing properties {#changing-properties}

Set properties in the definitions when you can. Use setters for values that change during the game.
Shape friction and restitution setters update existing contacts. For custom mixing rules, see
[Friction and restitution callbacks](@ref material-callbacks).

A shape can't change into or out of a sensor. Create a second shape instead. This trade-off was made
to keep sensor events consistent.

## Performance {#performance-practice}

Measure performance in a release build. Debug builds run heavy validation and can be a hundred times
slower. See `BOX2D_VALIDATE`.

Read body positions for rendering from `b2World_GetBodyEvents()`. Move events list only the bodies
that moved, which is cheaper than calling getters on every body.

Cost follows the number of awake dynamic bodies. Let bodies sleep and destroy bodies you no longer
need, such as debris.

Multithreading pays off for larger worlds. A world with around a hundred bodies may run faster with
a single worker. See [Multithreading](@ref multi).

## Debugging {#debugging-practice}

Draw the world with `b2World_Draw()` and compare it with your sprites. Most scale and offset
mistakes show up right away.

To report a bug, a [recording](@ref recording) is the best way to reproduce it. Zip the file and
attach it to the GitHub issue. Recording doesn't support the pre-solve and custom filter callbacks.
