# Large Worlds (Double Precision) {#large-worlds}

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

Double precision is **off by default**. With it off every double-precision type collapses to its
float counterpart through a typedef and the boundary helpers reduce to plain float operations, so
a float build behaves as Box2D always has with no measurable cost.

This implementation is inspired by [Jolt](https://jrouwe.github.io/JoltPhysics/index.html#big-worlds).

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

- `b2Pos` — a world position. Two doubles in large world mode, an alias for `b2Vec2`
  otherwise.
- `b2WorldTransform` — a world transform: a `b2Pos` translation and a float `b2Rot` rotation.
  An alias for `b2Transform` otherwise.

`b2Vec2`, `b2Rot`, `b2Transform`, and `b2AABB` stay float in both modes. `b2Transform` remains the
type for local and relative frames. `b2WorldTransform` is used for world-space. Rotations are
float in both modes, so the trig and the cross-platform determinism properties are unchanged.

The public API uses these types wherever it accepts or returns a world position: `b2BodyDef.position`,
`b2Body_GetPosition` / `b2Body_GetTransform`, `b2Body_SetTransform`, the explosion and ray-cast
origins, contact and ray-cast result points, and the body move event. With double precision off
these are all the float types they have always been, so existing code compiles unchanged.

With double precision **on**, `b2Pos` and `b2Vec2` are distinct structs by design. Code that
passed a `b2Vec2` where a world position is now required no longer compiles, which is the intended
cost: enabling large world mode is a deliberate source migration, and the compiler points at every
site that needs a `b2ToPos` / `b2ToVec2` conversion. Helpers cover the boundary:

```c
b2Pos p = b2ToPos( v );     // float vector -> world position
b2Vec2     v = b2ToVec2( p );           // world position -> float vector (lossy far from origin)
b2Vec2     d = b2SubPos( a, b ); // a - b, demoted to float (the precision boundary)
float      x = b2RoundDownFloat( p.x ); // conservative narrowing, pair with b2RoundUpFloat to
                                        // build a float box that always contains double bounds
```

## Operating range

Full simulation correctness holds everywhere `b2Pos` can represent. The practical limit comes
from the broad phase, which stays float and stores conservative (outward-rounded) float bounds. Far
from the origin the float bound quantization grows: about one meter at 1e7, sixteen meters at 1e8.
Overlapping shapes always still produce a pair, so correctness is preserved, but beyond roughly 1e7
to 1e8 meters the broad phase reports extra false pairs and loses some margin hysteresis, which
costs performance. Stay within about ±1e7 to ±1e8 meters.

## Query origins

Every spatial query takes a `b2Pos` origin, and the query geometry is relative to that
origin: the overlap AABB and proxy points, the cast proxy, the ray translation of
`b2World_CastRay` and `b2Shape_RayCast` (whose origin is the ray start), the mover capsule, and
the planes `b2World_CollideMover` returns. Near the origin pass
`b2Pos_zero` and the query reads as a plain world query. Far from the origin pass a nearby
base, typically a body or camera position: the query then runs in float relative to that base
with full precision, and cast results come back as world `b2Pos` points.

```c
// A precise pick box around a distant point
b2AABB box = { { -0.001f, -0.001f }, { 0.001f, 0.001f } };
b2World_OverlapAABB( worldId, clickPoint, box, filter, MyCallback, &ctx );
```

Character controllers work at any distance: pass the mover's vicinity as the origin to
`b2World_CastMover` / `b2World_CollideMover` and keep the capsule coordinates relative to it. The
returned planes are relative to the same origin, and the plane solver is frame agnostic, so they
feed `b2SolvePlanes` directly.

Internally, overlap query boxes are lifted to world float with outward rounding, so they are
conservative: an overlapping shape is always found. Ray and shape casts traverse the tree with
the origin truncated to float, which displaces the traversal by up to one coordinate ULP, about
a meter at 1e7. A cast that merely grazes a shape by less than that can miss it. Shapes the tree
does report are re-centered on the origin in double, so reported hits are exact even where the
float bounds are not. `b2World_CastRay` and `b2World_CastRayClosest` take a `b2Pos` origin
directly and resolve hit points the same way.

## Recordings and snapshots

Recordings and snapshots store world positions at the build precision. A double-precision recording
or snapshot will not load into a float build, and the reverse, because the position layout differs
irreconcilably; the loader rejects a precision mismatch with a clear message rather than replaying
wrong. The format version covers the query origin arguments, so recordings made before origins
existed are refused at load. Within one precision mode they behave exactly as documented in
[Recording and Replay](#recording). A recording made far from the origin reproduces the run
exactly, because the recorded positions ride the same double-precision wire format the live
simulation uses.
