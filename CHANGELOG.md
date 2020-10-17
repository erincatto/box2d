# Changes for version 2.4.1

## API Changes
- Extended distance joint to have a minimum and maximum limit.
- Removed rope joint. Use the distance joint instead.
- B2_USER_SETTINGS and b2_user_settings.h can control user data, length units, and maximum polygon vertices.
- Default user data is now uintptr_t instead of void*
- b2FixtureDef::restitutionThreshold lets you set the restitution velocity threshold per fixture.

## BREAKING Changes
- BREAKING: distance joint 0 stiffness now means the spring is turned off rather than making the joint rigid.
- BREAKING: distance joint minimum and maximum must be set correctly to get old behavior.

## Infrastructure
- Library installation function available in CMake.
- Shared library (DLL) option available.
- Bug fixes

# Changes for version 2.4.0

## Infrastructure
- Documentation in Doxygen format
- CMake build system
- Unit test support
- Continuous integration testing using Travis CI
- Limited use of C++11 (nullptr and override)
- Restructured folders and renamed files to better match open-source standards
- MIT License
- Removed float32 and float64
- Linked the Box2D project to GitHub Sponsors

## Collision
- Chain and edge shape must now be one-sided to eliminate ghost collisions
- Broad-phase optimizations
- Added b2ShapeCast for linear shape casting

## Dynamics
- Joint limits are now predictive and not stateful
- Experimental 2D cloth (rope)
- b2Body::SetActive -> b2Body::SetEnabled
- Better support for running multiple worlds
- Handle zero density better
  - The body behaves like a static body
  - The body is drawn with a red color
- Added translation limit to wheel joint
- World dump now writes to box2d_dump.inl
- Static bodies are never awake
- All joints with spring-dampers now use stiffness and damping
- Added utility functions to convert frequency and damping ratio to stiffness and damping

## Testbed
- Testbed uses dear imgui
- glad OpenGL loader
- OpenGL 3.3 required

# Changes for version 2.3.0
- Polygon creation now computes the convex hull. Vertices no longer need to be ordered.
- The convex hull code will merge vertices closer than dm_linearSlop. This may lead to failure on very small polygons.
- Added b2MotorJoint.
- Bug fixes.
