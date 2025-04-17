# v3.1 Release Notes

## API Changes
- 64-bit filter categories and masks
- 64-bit dynamic tree user data
- Renamed `b2SmoothSegment` to `b2ChainSegment`
- Cast and overlap functions modified for argument consistency
- Contact begin events now provide the manifold
- More consistent functions to make polygons
- Contact events are now disabled by default
- Replaced `b2Timer` with `uint64_t`
- Shape material properties now use `b2SurfaceMaterial`

## New Features
- New character mover features and sample
- Revised sensor system is now independent of body type and sleep
- Rolling resistance and tangent speed
- Friction and restitution mixing callbacks
- World explosions
- World access to the maximum linear speed
- More control over body mass updates
- Filter joint to disable collision between specific bodies
- Bodies can now have names for debugging
- Added `b2Body_SetTargetTransform` for kinematic bodies

## Improvements
- Cross-platform determinism
- Custom SSE2 and Neon for significantly improved performance
- SSE2 is the default instead of AVX2
- Removed SIMDE library dependency
- Faster ray and shape casts
- Faster continuous collision
- Each segment of a chain shape may have a different surface material
- Reduced overhead of restitution when not used
- Implemented atomic platform wrappers eliminating the `experimental:c11atomics` flag

## Bugs Fixes
- Many bug fixes based on user testing
- Fixed missing hit events
- Capsule and polygon manifold fixes
- Fixed missing contact end events
- PreSolve is now called in continuous collision
- Reduced clipping into chain shapes by fast bodies
- Friction and restitution are now remixed in the contact solver every time step
- Body move events are now correctly adjusted for time of impact

## Infrastructure
- Unit test for API coverage
- macOS and Windows samples built in GitHub actions
- CMake install
- imgui and glfw versions are now pinned in FetchContent
- Initial Emscripten support
