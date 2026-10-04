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
