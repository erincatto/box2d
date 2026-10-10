# v3.2 Performance

Box2D v3.2 steps the same scenes about twice as fast as v3.1.1. Across the 14 benchmark scenes the geometric mean speedup is **2.0x on one thread** and **2.2x on eight threads**. Large stacks, many pyramids and sleep/wake gain about **3x**.

These are relative numbers from one machine. Read the ratios, not the milliseconds.

## Results

Step time for the whole scene in milliseconds, lower is better. v3.2 is the default build: AVX2 kernels selected at runtime, built-in task scheduler, world capacity set from `b2World_GetMaxCapacity`.

| Benchmark | v3.1.1, 1 thread | v3.2, 1 thread | Speedup | v3.1.1, 8 threads | v3.2, 8 threads | Speedup |
|---|---:|---:|---:|---:|---:|---:|
| compounds | 2684 | 1112 | 2.41x | 902 | 265 | 3.40x |
| joint_grid | 2613 | 1969 | 1.33x | 559 | 325 | 1.72x |
| junkyard | 5918 | 2038 | 2.90x | 1350 | 452 | 2.98x |
| large_pyramid | 2070 | 727 | 2.85x | 370 | 137 | 2.70x |
| many_pyramids | 3095 | 1047 | 2.96x | 458 | 151 | 3.03x |
| rain | 8557 | 5788 | 1.48x | 2017 | 1045 | 1.93x |
| smash | 1841 | 836 | 2.20x | 585 | 222 | 2.63x |
| spinner | 5076 | 2713 | 1.87x | 1228 | 611 | 2.01x |
| tumbler | 2153 | 916 | 2.35x | 597 | 247 | 2.41x |
| washer | 7218 | 3048 | 2.37x | 1493 | 547 | 2.73x |
| queries | 1655 | 1286 | 1.29x | 1372 | 1173 | 1.17x |
| tree_cast | 1203 | 917 | 1.31x | 1195 | 921 | 1.30x |
| tile_world | 727 | 484 | 1.50x | 569 | 437 | 1.30x |
| sleep | 4592 | 1454 | 3.16x | 744 | 247 | 3.01x |
| **geometric mean** | | | **2.04x** | | | **2.19x** |

`queries` and `tree_cast` are dominated by ray casts, shape casts and overlap queries that run on the calling thread, so extra threads don't help them in either version.

## Where the gains come from

The speedup has three parts: the engine itself, AVX2, and the new capacity API.

| Configuration | 1 thread | 8 threads |
|---|---:|---:|
| v3.2 with SSE2 forced (`b2World_EnableSSE2Fallback`) | 1.83x | 2.03x |
| v3.2 default (AVX2 at runtime) | 2.03x | 2.18x |
| v3.2 default with world capacity | 2.04x | 2.19x |

- **Engine work, visible on SSE2: most of the gain.** This includes:
  - faster SAT
  - cache and broad-phase work
  - faster island splitting
  - contact recycling
  - the new contact margin
  - lower scheduler overhead
- **AVX2 runtime dispatch: about another 10%.** It is on by default and needs no build flag. The gain is largest in the solver-bound scenes: `large_pyramid`, `many_pyramids` and `sleep` go from about 2.2x to 2.9x on one thread. `rain` (ragdolls and joints) is the exception; it was slightly faster with SSE2 on one thread.
- **World capacity (`b2WorldDef::capacity`): no measurable change** in these step times. It pre-sizes the world's arrays and avoids reallocation while a scene grows. These timings exclude world creation and the first step, which is where that cost lands.

## Behavior differences that affect the comparison

The scenes are identical: body, shape and joint counts match in every benchmark. Some of the work per step is not identical.

- **Fewer contacts in v3.2.** v3.2 sizes the broad-phase AABB margin as a fraction of the shape size, instead of a fixed 5 cm. Small shapes get tighter boxes and create fewer contacts. For example, `washer` ends with 29k contacts instead of 42k, and `tumbler` with 8.6k instead of 11k. This is an engine improvement, so it counts toward the speedup.
- **Different multithreading setup.** v3.1.1 has no built-in scheduler, so it runs multithreaded through enkiTS, as its own benchmark app did. v3.2 uses its built-in scheduler, which is what you get by setting `b2WorldDef::workerCount`.

## Method

- **Machine and build:**
  - AMD Ryzen 9 9950X3D on Windows 11
  - MSVC 19.51, Release, default CMake options for each version
  - Turbo disabled through a power plan with the processor state pinned at 99%
- **Thread placement:** pinned to one hardware thread on each of eight cores (`start /affinity 0x5555`).
- **Scenes:** both versions ran the v3.2 benchmark scenes (`shared/benchmarks.c`). For v3.1.1 the scenes were ported to the v3.1.1 API without changing them.
- **Timing:** 60 Hz, 4 substeps. Times cover all steps after the first; world creation is excluded.
- **Repeats:** each value is the minimum of four runs, taken as two interleaved rounds of two runs, ordered so that thermal state is matched across configurations. Run-to-run spread was under 4% in nearly every cell.

## Performance Details

### Many pyramids

The `many_pyramids` benchmark is 400 small pyramids stacked in a 20 by 20 grid. It has 22,000 boxes and 58,000 contacts, with sleeping disabled, so every step solves the whole stack at rest. It measures the cost of contacts that persist from step to step, which is the common case in most games.

v3.2 runs this scene about 3x faster than v3.1.1:

| Threads | v3.1.1 | v3.2 | Speedup |
|---|---:|---:|---:|
| 1 | 3095 ms | 1047 ms | 2.96x |
| 8 | 458 ms | 151 ms | 3.03x |

These are times for 200 steps at 60 Hz with 4 sub-steps, measured on an AMD Ryzen 9 9950X3D with turbo disabled. Read the ratios, not the milliseconds.

#### Where the time went

Single-threaded cost per step, broken out by stage:

| Stage | v3.1.1 | v3.2 (SSE2) | v3.2 (AVX2) |
|---|---:|---:|---:|
| Collide | 5.94 ms | 0.64 ms | 0.64 ms |
| Biased solve | 3.53 ms | 1.56 ms | 0.99 ms |
| Relax | 3.53 ms | 2.71 ms | 1.52 ms |
| Prepare, warm start, integrate, store | 2.12 ms | 1.48 ms | 1.53 ms |
| Transforms, events and other | 0.78 ms | 0.69 ms | 0.70 ms |
| Total | 15.90 ms | 7.08 ms | 5.38 ms |

Collision is 9x faster, and the contact solver is about 2.3x faster. Here is how the gain divides among the changes, timed at each commit between the two releases:

| Change | Share of the gain |
|---|---:|
| Contact recycling (#1038) | 40% |
| Biased solve without friction (#1050) | 23% |
| AVX2 selected at runtime (#1118) | 16% |
| Solver inlining and faster constraint preparation (#1094, #1113) | 12% |
| Broad-phase and narrow-phase cache work (#1104, #1111) | 8% |

The remaining 1% is spread across other commits, each within measurement noise.

#### Contact recycling

In v3.1.1 every contact recomputed its manifold every step: a full separating axis test and clipping, about 100 ns per box pair. In a resting stack that work repeats the previous answer.

v3.2 keeps the manifold of a contact whose bodies have barely moved relative to each other since the manifold was last computed. The limits are:

- 5 cm of combined movement for touching shapes, or 2 cm for shapes that are not touching;
- about 11 degrees of rotation for either body.

A recycled contact keeps its anchors and normal and updates its separation from the body motion, the same way sub-stepping does. Its stored impulses warm start the solver directly. Every contact in `many_pyramids` is recycled every step, and collision drops to about 11 ns per contact. Turning recycling off brings collision back to 4.6 ms per step. The rest of the collision gain comes from the narrow-phase cache work.

A recycled contact skips more than the manifold. It does not:

- call the pre-solve callback;
- remix friction, restitution or tangent speed;
- produce begin or end touch events.

These updates resume the next time the contact is fully updated. If you rely on any of them every step, or see ghost collisions on a character, you can turn recycling off:

- per body, with `b2BodyDef::enableContactRecycling` or `b2Body_EnableContactRecycling`, which affects contacts created afterwards;
- for the whole world, with `b2World_SetContactRecycleDistance( worldId, 0.0f )`.

This scene is the best case for recycling. Scenes with more motion recycle fewer contacts and gain less.

#### The contact solver

Each sub-step solves contacts twice: a biased pass that pushes overlapping shapes apart, then a relax pass that removes the velocity the push added. In v3.1.1 both passes ran the same kernel with normal, friction and rolling resistance, so they cost the same.

In v3.2 the biased pass solves only the two normal constraints of each manifold. Friction is solved once per sub-step, in the relax pass. The biased pass reads only the first two thirds of each contact constraint. Before, it read nearly all of it. It is now 2.3x faster at the same SIMD width. This is a behavior change.

The relax pass also got cheaper:

- The soft constraint blending it used to compute and then discard is gone.
- Rolling resistance is skipped when it is zero.

The contact constraint layout also changed:

- It is smaller.
- Fields are ordered so the biased pass reads a contiguous prefix.
- It is 64-byte aligned.

Body gather and scatter are now force inlined. MSVC was not inlining them, so each body state went through memory four times per contact constraint. Constraint preparation now loads manifold points with SIMD and reads body velocities only when restitution or hit events need them.

#### AVX2

v3.2 detects AVX2 at runtime and uses an 8-wide contact solver when the CPU supports it, with no build flag needed. In this scene that halves the solver loop count and brings the solver from 5.7 ms down to 4.0 ms per step. With SSE2 forced, v3.2 is still 2.2x faster than v3.1.1.
