# Large Worlds {#large-worlds}

Box2D can handle worlds that extend up to 20 kilometers from the origin, depending on the shape
sizes used. Optionally, you can build Box2D with double-precision world positions. With
double-precision, worlds that extend to 1000 kilometers from the origin are feasible. The main
limitation is the broad-phase bounding boxes remain single-precision, so they start to become
excessively sized past 1000 kilometers.

This trade-off works well and using double-precision body positions only costs a few percent in
performance. All the heavy math is done in relative coordinates in single-precision. So the
optimized SIMD code works as is.

The Box2D implementation of double-precision is inspired by
[Jolt](https://jrouwe.github.io/JoltPhysics/index.html#big-worlds).

## Enabling large world support

Set the CMake option:

```cmake
set(BOX2D_DOUBLE_PRECISION ON)
```

Alternatively you can set this up in `include/box2d/config.h`. `b2IsDoublePrecision()` lets you
determine what type the library was built with.

## Double-precision math

Double-precision adds two types and leaves the existing math types alone:

- `b2Pos` — a world position. Two doubles in large world mode, an alias for `b2Vec2` otherwise.
- `b2WorldTransform` — a world transform: a `b2Pos` translation and a float `b2Rot` rotation. An
  alias for `b2Transform` otherwise.

`b2Vec2`, `b2Rot`, `b2Transform`, and `b2AABB` stay float in both modes. `b2Transform` remains the
type for local and relative frames. `b2WorldTransform` is used for world-space. Rotations are float
in both modes, so the trig and the cross-platform determinism properties are unchanged.

The public API uses these types wherever it accepts or returns a world position such as
`b2BodyDef.position`, `b2Body_GetPosition` / `b2Body_GetTransform`, and `b2Body_SetTransform`. The
ray-cast, overlap, and shape casts also specify an origin in double-precision.

With double-precision **on**, `b2Pos` and `b2Vec2` are distinct structs for type safety. Code that
passed a `b2Vec2` where a world position is now required no longer compiles, which is the intended
cost: enabling large world mode is a deliberate source migration, and the compiler points at every
site that needs a `b2ToPos` / `b2ToVec2` conversion. Here are some helpers:

```c
b2Vec2 v = {1.0f, 2.0f};
b2Pos p = b2ToPos(v);
b2Vec2 b = b2ToVec2( p ); // lossy
b2Vec2 c = b2SubPos( p, b2Pos_zero );
float x = b2RoundDownFloat( p.x );
```

## Operating range

Full simulation correctness holds everywhere `b2Pos` can represent. The practical limit comes from
the broad-phase, which stays float and stores conservative (outward-rounded) float bounds. Far from
the origin the float bound quantization grows: about one meter at 1e7 and eight meters at 1e8.
Overlapping shapes always still produce a pair, so correctness is preserved, but beyond roughly 1e7
to 1e8 meters the broad-phase reports extra false pairs. This makes performance dip.

## Query origins

Every spatial query takes a `b2Pos` origin, and the query geometry is relative to that origin: the
overlap AABB and proxy points, the cast proxy, the ray translation of `b2World_CastRay` and
`b2Shape_CastRay` (whose origin is the ray start), the mover capsule, and the planes
`b2World_CollideMover` returns.

I recommend to use an origin that is close to the query shape and make the shape relative to that
origin. Do this even when not using double-precision to make it easier to switch to double-precision
in the future. For example, this overlap query uses a distant point as the origin and the box
has small coordinates relative to that origin.

```c
// A precise pick box around a distant point
b2Pos origin = { 1000.0, -2000.0 };
b2AABB box = { { -0.001f, -0.001f }, { 0.001f, 0.001f } };
b2World_OverlapAABB( worldId, origin, box, filter, MyCallback, &ctx );
```

Internally, overlap query boxes are converted to world float with outward rounding, so they are
conservative and an overlapping shape is always found. Ray and shape casts traverse the tree with
the origin truncated to float, but the broad-phase bounding boxes are conservative so nothing is
missed. The detailed collision is done in local coordinates to preserve accuracy.

`b2World_CastRay` and `b2World_CastRayClosest` take a `b2Pos` origin and the returned hit point is
in double-precision.

## Recordings and snapshots

Recordings and snapshots store world positions at the build precision. So precision type differences
make recordings and snapshots incompatible. This is likely not a significant limitation in practice.
