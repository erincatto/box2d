# Known Limitations {#known-limitations}

Box2D makes trade-offs for performance and stability. The behaviors on this page are known and
intentional. They are not bugs, so please don't report them as issues. Each section describes the
symptom, why it happens, and what to do about it.

Many problems come from setup rather than from the engine. Those are covered in
[Best Practices](@ref best-practices).

## Symptom index

| Symptom | See |
|---|---|
| Fast bodies pass through each other | [Tunneling between dynamic bodies](@ref tunneling) |
| Shapes overlap slightly while resting | [Resting overlap](@ref resting-overlap) |
| Joints stretch, sag, or flex | [Soft joints and mass ratios](@ref soft-joints) |
| A heavy body makes a joint rig jitter or explode | [Soft joints and mass ratios](@ref soft-joints) |
| Bounces are smaller or less consistent than expected | [Approximate restitution](@ref approximate-restitution) |
| A projectile doesn't follow an exact parabola | [Integration accuracy](@ref integration-accuracy) |
| A begin touch event arrives a step late or has zero impulse | [Contact event timing](@ref contact-event-timing) |
| A sensor misses a fast object | [Sensors are not swept](@ref sensors-not-swept) |
| Shapes pass through a chain from one side | [Chains are one-sided](@ref one-sided-chains) |
| A thin polygon or short capsule is not created | [Minimum shape size](@ref minimum-shape-size) |
| A kinematic body passes through static bodies | [Kinematic bodies don't collide with static bodies](@ref kinematic-static) |
| Character movers pass through each other | [Geometric movers don't collide with each other](@ref mover-collision) |
| Cargo drifts on a rotating platform | [Contact recycling drifts on slowly rotating platforms](@ref recycling-drift) |
| Shapes snag on seams in flat ground | [Terrain and level geometry](@ref terrain) |
| Everything looks slow and floaty | [Units and scale](@ref units-and-scale) |
| A body stays asleep after you move it or change a joint | [Sleep and waking](@ref sleep-and-waking) |
| The simulation is slow | [Performance](@ref performance-practice) |

## Tunneling between dynamic bodies {#tunneling}

Continuous collision prevents dynamic bodies from tunneling through static bodies. It is not used
between two dynamic bodies because it is too expensive to do for every pair. A small, fast dynamic
body can pass through another dynamic body in a single step.

Mark the fast bodies that matter as bullets. Bullets use continuous collision against all body
types, but not against other bullets. Use bullets sparingly. For gunfire and other projectiles that
only need a hit result, a ray cast or shape cast is cheaper and more reliable. You can simulate a
traveling bullet with multiple ray casts following a parabola with a straight segment for each time
step in your game code.

Continuous collision also does not account for joints. A fast body attached to a joint may stretch
the joint for a few steps before it recovers. This is commonly seen in fast vehicles that use rigid
bodies for wheels. For fast vehicles it is better to use a ray cast and suspension to simulate
wheels.

See also: [Bullets](@ref bullets).

## Resting overlap {#resting-overlap}

Resting shapes may overlap slightly due to the soft constraint system. This is a stability
trade-off. You can make contacts stiffer by using a smaller time step and more sub-steps. If you
try that then you can increase `b2WorldDef::contactHertz` to reduce the softness.

Deeper overlap is pushed apart at a limited speed, set by `b2WorldDef::contactSpeed`, so bodies
that start out overlapping separate over several steps instead of exploding. In large stacks some
overlap can persist. You should normally not need to tune the contact softness settings in
`b2WorldDef`. If overlap recovers too slowly, a lower `contactDampingRatio` resolves it faster at
the cost of a more energetic push.

## Soft joints and mass ratios {#soft-joints}

Box2D solves constraints with an approximate iterative method and uses soft constraints for
robustness. Joints and contacts are not perfectly rigid. Long chains sag, weld joints flex, and
contacts can compress under heavy loads.

Extreme mass ratios make this worse. A heavy body hanging from light bodies stretches the joints
between them, and several joints holding a heavy body can fight each other until the rig becomes
unstable. Keep the mass ratio between connected bodies below about 10 to 1.

What helps:

- More sub-steps make joints stiffer.
- If you use more sub-steps and a small time step you can safely increase
  `b2JointDef::constraintHertz` to make a joint stiffer.
- Shapes on the same body are perfectly rigid, so use a compound body instead of weld joints where
  you can.
- For an unstable rig, lower `b2JointDef::constraintHertz` to around 30.

See also: [Compound bodies](@ref compound-bodies), [Joints](@ref joint-practice).

## Approximate restitution {#approximate-restitution}

Restitution works well for a single contact. When a shape has several contacts, such as a box
landing on a resting box, Box2D solves them in sequence and the bounce is approximate. A box may
bounce less off a resting box than off the ground.

Collisions slower than `b2WorldDef::restitutionThreshold` (1 meter per second by default) don't
bounce at all. This prevents jitter in resting contact.

More restitution iterations, set with `b2WorldDef::restitutionIterations`, can improve accuracy.
Note that the default restitution is zero, so set it on the shape material if you want a bounce.

See also: [Restitution](@ref restitution).

## Integration accuracy {#integration-accuracy}

Box2D integrates motion with the [semi-implicit Euler
method](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method). It is fast and stable, but only
first-order accurate. A thrown body doesn't follow the exact parabola you would compute by hand, so
don't use an analytic formula to predict where the simulation will land. Simulate the trajectory with
the same time step instead.

## Contact event timing {#contact-event-timing}

Contacts are updated at the beginning of each step, before bodies move. This keeps objects from
sinking into each other. A contact begins touching when its shapes come within a small speculative
distance, so slow shapes report begin touch about when they meet. A fast shape can go from out of
range to impact within a single step, and then its begin touch event arrives after the next step.

Begin touch events are also recorded before the solver runs, so they carry no impulses. Use hit
events for impacts, or read `b2Shape_GetContactData()` after the step.

Sensor events work differently. Sensors are processed at the end of the step, so their events have
no delay.

See also: [Contact events](@ref contact-events), [Events](@ref events-practice).

## Sensors are not swept {#sensors-not-swept}

Sensors test overlap at the end of each step. They don't sweep. A static sensor usually catches a
fast shape passing through it, but a fast moving sensor, or a shape faster than the sensor is wide,
can skip over without an event.

For fast checks, use a ray cast or shape cast along the path of motion.

See also: [Sensors](@ref sensors).

## Chains are one-sided {#one-sided-chains}

Chain shapes collide on one side only. This is what lets them remove ghost collisions. Shapes coming
from the back side pass through, and ray casts and shape casts from the back side miss. Placing two
chains back to back does not make a reliable two-sided wall.

For two-sided collision, use segment shapes and accept the ghost collisions, or use polygons.

See also: [Chain shapes](@ref chain-shapes).

## Minimum shape size {#minimum-shape-size}

Box2D needs shapes larger than its tolerances.

`b2ComputeHull()` welds points that are very close and removes points that are nearly collinear.
A polygon thinner than about a centimeter collapses and the hull comes back empty. Use
`b2MakeBox()` for thin boxes, since it doesn't build a hull. Better still, make thin objects
thicker. A capsule whose centers are closer than the linear slop is rejected and
`b2CreateCapsuleShape()` returns a null shape id. Use a circle instead.

In meters, keep moving shapes larger than a centimeter. If you changed the length units, these
limits scale with them.

See also: [Polygons](@ref polygons).

## Kinematic bodies don't collide with static bodies {#kinematic-static}

Kinematic bodies don't collide with static bodies or other kinematic bodies, and generate no contact
events with them. A kinematic body has infinite mass, so nothing could push it back. If a moving
platform or enemy must be stopped by walls, use a dynamic body, perhaps driven by a motor joint, or
use a shape cast to find the walls yourself.

See also: [Body types](@ref body-types).

## Geometric movers don't collide with each other {#mover-collision}

The geometric character mover is not part of the simulation world. It doesn't collide with other
movers and generates no contact or sensor events. Handle character versus character yourself, for
example by keeping the movers in your own dynamic tree and querying it. If you need full physical
interaction, use the dynamic mover, which is a rigid body.

See also: [Character Mover](character.md).

## Contact recycling drifts on slowly rotating platforms {#recycling-drift}

Dynamic bodies stacked on a large rotating platform can drift. One example is crates on the deck
of a ship. You can prevent this by disabling recycling on the platform.

```c
platformBodyDef.enableContactRecycling = false;
```

See also: [Contact recycling](@ref contact-recycling).
