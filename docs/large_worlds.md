# Large Worlds (Double Precision)

Box2D can be built with double precision world positions for large worlds: simulations that range
far from the origin, where a single-precision float can no longer resolve a position. At a
coordinate of 1e7 meters a float has a step of about one meter, so bodies snap to a coarse grid
and contacts jitter. Double precision keeps full sub-millimeter resolution out to planetary
distances.

Only the world position boundary is double. Velocities, forces, shapes, contact manifolds, the
contact solver, and the broad-phase tree all stay float. This is the same boundary design Jolt
Physics uses for its big-world mode: doubles carry the absolute position, everything inside one
body's frame stays float, and the per-step motion the solver integrates is small and meters-scale
where float precision is ample. The cost is a few percent, not the 2x of an all-double build.

Double precision is **off by default**. With it off the build is byte-for-byte identical to a
normal Box2D build: every double-precision type collapses to its float counterpart through a
typedef, so there is no code path to regress and no performance change.

## Enabling it

Set the CMake option:

```cmake
set(BOX2D_DOUBLE_PRECISION ON)
```

The define is propagated to consumers as a `PUBLIC` compile definition, so anything that links
Box2D through CMake sees the same precision mode in the headers and cannot mismatch. For non-CMake
consumers there is a link-time guard: a float application linked against a double-precision library
(or the reverse) fails to link on the first Box2D call rather than miscompiling silently. At
runtime, `b2IsDoublePrecision()` reports which mode the library was built in, for bindings and
diagnostics.

## The two world-position types

Double precision adds two types and leaves the existing math types alone:

- `b2Position` — a world position. Two doubles in large world mode, an alias for `b2Vec2`
  otherwise.
- `b2WorldTransform` — a world transform: a `b2Position` translation and a float `b2Rot` rotation.
  An alias for `b2Transform` otherwise.

`b2Vec2`, `b2Rot`, `b2Transform`, and `b2AABB` stay float in both modes. `b2Transform` remains the
type for local and relative frames. `b2WorldTransform` is used for world-space. Rotations are
float in both modes, so the trig and the cross-platform determinism properties are unchanged.

The public API uses these types wherever it accepts or returns a world position: `b2BodyDef.position`,
`b2Body_GetPosition` / `b2Body_GetTransform`, `b2Body_SetTransform`, the explosion and ray-cast
origins, contact and ray-cast result points, and the body move event. With double precision off
these are all the float types they have always been, so existing code compiles unchanged.

With double precision **on**, `b2Position` and `b2Vec2` are distinct structs by design. Code that
passed a `b2Vec2` where a world position is now required no longer compiles, which is the intended
cost: enabling large world mode is a deliberate source migration, and the compiler points at every
site that needs a `b2MakePosition` / `b2ToVec2` conversion. Helpers cover the boundary:

```c
b2Position p = b2MakePosition( v );    // float vector -> world position
b2Vec2     v = b2ToVec2( p );          // world position -> float vector (lossy far from origin)
b2Vec2     d = b2PositionDelta( a, b ); // a - b, demoted to float (the precision boundary)
```

## Operating range

Full simulation correctness holds everywhere `b2Position` can represent. The practical limit comes
from the broad phase, which stays float and stores conservative (outward-rounded) float bounds. Far
from the origin the float bound quantization grows: about one meter at 1e7, sixteen meters at 1e8.
Overlapping shapes always still produce a pair, so correctness is preserved, but beyond roughly 1e7
to 1e8 meters the broad phase reports extra false pairs and loses some margin hysteresis, which
costs performance. Stay within about ±1e7 to ±1e8 meters.

## Precision carve-outs

A few query and collision entry points stay float-only in this release. They take or return float
world coordinates, so far from the origin they lose precision the same way a float build would.
They remain correct near the origin and are conservative far from it:

- `b2World_OverlapAABB`, `b2World_OverlapShape`, `b2World_CastShape`
- `b2World_CastMover`, `b2World_CollideMover`
- `b2Shape_RayCast`

The mover functions drive character controllers. A character run far from the origin through
`b2World_CastMover` / `b2World_CollideMover` gets float world coordinates and silently loses
sub-meter motion, so a kinematic character at 1e7 does not behave with double precision yet. This
is a functional limitation of the first large-world release, not just a thinner API. Keep movers
near the origin, or shift the world so the character stays near it.

`b2World_CastRay` and `b2World_CastRayClosest` are **not** carve-outs: their origin is a
`b2Position` and each candidate shape is re-centered in double, so a ray cast resolves the hit point
accurately anywhere in range.

## Recordings and snapshots

Recordings and snapshots store world positions at the build precision. A double-precision recording
or snapshot will not load into a float build, and the reverse, because the position layout differs
irreconcilably; the loader rejects a precision mismatch with a clear message rather than replaying
wrong. Within one precision mode they behave exactly as documented in
[Recording and Replay](#recording). A recording made far from the origin reproduces the run
exactly, because the recorded positions ride the same double-precision wire format the live
simulation uses.
