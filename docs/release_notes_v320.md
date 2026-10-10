# v3.2 Release Notes

## API changes
- Joint definitions share a common `b2JointDef base` and use local frames instead of anchors, axes and reference angles.
- Mouse joint removed, the motor joint now has spring and velocity targets.
- Motion locks replace fixed rotation.
- Chain shapes are segment based.
- Contact ids (`b2ContactId`) in contact events, with `b2Contact_GetData`.
- Pre-solve callback redesigned, plus a separate continuous pre-solve callback.
- Native task system. Box2D runs its own threads when no task callbacks are hooked up.

## New features
- Large worlds with optional double precision (`BOX2D_DOUBLE_PRECISION`).
- Recording, replay and world snapshots.
- Contact recycling for better stability and performance.
- Dynamic character mover with new mover and pogo joints (experimental).
- Joint events for breakable joints (force and torque thresholds).
- Tunable continuous collision per body (safety factor).
- Restitution iterations and propagation.
- Wind and drag on shapes.
- Distance joint spring force limits.
- Loose chain segments.
- Runtime AVX2 dispatch with SSE2 fallback.

## Improvements
- Many optimizations.
- Faster broad-phase and ray casts. Much less single threaded work.
- Faster island merging and splitting.
- Invalid input is now rejected and logged in release builds.
- Various SIMD optimizations.

## Infrastructure
- pkg-config support.
- Swift and Zig builds in CI.
- MinGW and clang-cl builds.
- CMake presets.
- Revised samples UI and many new samples.

## Breaking changes
- `b2RelativeAngle` changed order.
- Wheel joint axis convention changed.
- Motor joint defaults are now all zero.
- `b2JointType` values changed, so stored joint type integers are invalid.
- Chain shapes creation changed a lot.
- Contact recycling is on by default. This will skip updates on friction and pre-solve until the shapes move more than 5cm from each other. You can disable contactd recycling on the world or per body.
- Adjusted tolerance in `b2Normalize`.
- Contact point normal velocity is only computed if needed for hit events or restitution.
