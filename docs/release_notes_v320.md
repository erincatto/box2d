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

These compile but behave differently:

- `b2RelativeAngle(a, b)` swapped its arguments and still returns b minus a, so existing calls silently change sign.
- The wheel joint axis is the x-axis of `localFrameA`. The old default axis was vertical, so vertical suspension now needs `base.localFrameA.q = b2MakeRot(0.5f * B2_PI)`.
- Motor joint defaults are now all zero, so a default motor joint does nothing.
- `b2JointType` values changed, so stored joint type integers are invalid.
- Custom filtering is opt-in per shape. The custom filter callback only runs for pairs where one of the shapes sets `b2ShapeDef::enableCustomFiltering`.
- Contact recycling is on by default. A touching contact reuses its manifold and skips material updates and the pre-solve callback until the shapes move about 5cm or rotate about 11 degrees relative to each other. Disable it on the world with `b2World_SetContactRecycleDistance(worldId, 0.0f)` or per body with `enableContactRecycling`.
- `b2Normalize` and `b2GetLengthAndNormalize` used to return zero for a length below `FLT_EPSILON`. Now only a squared length at or below `1000 * FLT_MIN` returns zero, so tiny vectors normalize.
- Debug draw always applies `drawingBounds`, so a zeroed `b2DebugDraw` draws nothing. Start from `b2DefaultDebugDraw()`.
- Body names are truncated to `B2_NAME_LENGTH` characters, which is 10. It was 31.
- Contact point normal velocity is only computed if needed for hit events or restitution.
- `b2CreateSegmentShape` and `b2CreateCapsuleShape` return `b2_nullShapeId` for a degenerate segment or capsule.

Renamed and changed functions:

- World queries take a `b2Pos` origin as the second argument. Pass `b2Pos_zero` for the old behavior.
- Collision functions are verb first: `b2RayCast*` to `b2CastRay*`, `b2ShapeCast*` to `b2CastShape*`, `b2ShapeCast` to `b2CastShape`, `b2ShapeDistance` to `b2ComputeShapeDistance`, `b2SegmentDistance` to `b2ComputeSegmentDistance`, `b2TimeOfImpact` to `b2ComputeTimeOfImpact`.
- `b2Shape_RayCast` is now `b2Shape_CastRay(shapeId, origin, translation)` and returns `b2WorldCastOutput`.
- Task callbacks: `b2TaskCallbackFcn(void* taskContext)` and `b2EnqueueTaskCallbackFcn(task, taskContext, userContext)`. There is no item range and no worker index. `B2_MAX_WORKERS` is 32, it was 64.
- Allocator hooks: `b2AllocCallbackFcn(size_t size, int alignment)` and `b2FreeCallbackFcn(void* mem, size_t size)`. `b2GetByteCount` and `b2DynamicTree_GetByteCount` return `size_t`.
- Callback types end in `CallbackFcn`, such as `b2PreSolveCallbackFcn`, `b2CastResultCallbackFcn` and `b2FrictionCallbackFcn`. `b2SetAssertFcn` is now `b2SetAssertCallback`.
- Body: `b2Body_ApplyMassFromShapes` to `b2Body_UpdateMassFromShapes`, `b2Body_GetLocalCenterOfMass` to `b2Body_GetLocalCenter`, `b2Body_GetWorldCenterOfMass` to `b2Body_GetWorldCenter`. `b2Body_SetTargetTransform` takes a wake flag. `b2Body_EnableSleep` and `b2Body_IsSleepEnabled` are now `b2Body_EnableSleeping` and `b2Body_IsSleepingEnabled`. `b2Body_ComputeAABB` is now `b2Body_ComputeBounds`.
- Shape: `b2Shape_GetMassData` to `b2Shape_ComputeMassData`, `b2Shape_SetMaterial`/`GetMaterial` to `b2Shape_SetUserMaterial`/`GetUserMaterial` with a `uint64_t` id, `b2Shape_GetSensorOverlaps` to `b2Shape_GetSensorData`. `b2Shape_SetSurfaceMaterial` takes a pointer. `b2Shape_GetAABB` is now `b2Shape_GetBounds`.
- Chain: open chains require `ghost1` and `ghost2`, and every segment of an open chain collides. `b2ChainDef::count` is now `pointCount`, and `materialCount` must be 1 or the segment count. The chain friction, restitution and material functions are replaced by `b2Chain_SetSurfaceMaterial` and `b2Chain_SetAllSurfaceMaterials`.
- World: `b2WorldDef::maxContactPushSpeed` is now `contactSpeed`. `b2World_EnableSpeculative` is removed. `b2World_SetMaximumLinearSpeed` and `b2World_GetMaximumLinearSpeed` are now `b2World_SetMaxLinearSpeed` and `b2World_GetMaxLinearSpeed`. `b2World_GetContactTuning` is new.
- Contacts: `b2ManifoldPoint::point` and `b2ContactBeginTouchEvent::manifold` are removed. Use the contact id with `b2Contact_GetData`.
- Collision: the shape comes first in `b2PointIn*`, `b2CastRay*` and `b2CastShape*`. `b2Collide*` take one relative transform and return a `b2LocalManifold`. `b2DistanceInput` and `b2ShapeCastPairInput` take one relative `transform`.
- Dynamic tree: `b2CreateDynamicTree`, `b2DestroyDynamicTree`, `b2CreateTreeProxy` and `b2DestroyTreeProxy` replace the `b2DynamicTree_` versions. `b2DynamicTree` is opaque, `b2CreateDynamicTree` returns a pointer, and `b2TreeNode` and `b2TreeProxy` are no longer public. `b2DynamicTree_RayCast` is now `b2DynamicTree_CastRay`, and `b2DynamicTree_ShapeCast` is now `b2DynamicTree_CastBox`, which sweeps an AABB. `EnlargeProxy` and `Rebuild` are removed. `b2DynamicTree_Query` is now `b2DynamicTree_OverlapAABB` and `b2DynamicTree_GetAABB` is now `b2DynamicTree_GetProxyBounds`.
- Distance joint: `b2DistanceJoint_SetLength` and `GetLength` are now `SetRestLength` and `GetRestLength`. `SetLengthRange`, `GetMinLength` and `GetMaxLength` are now `SetLimits`, `GetLowerLimit` and `GetUpperLimit`, matching the other joints.
- Current joint values say so: `b2RevoluteJoint_GetAngle` is now `b2RevoluteJoint_GetCurrentAngle`, and `b2PrismaticJoint_GetTranslation` and `b2PrismaticJoint_GetSpeed` are now `b2PrismaticJoint_GetCurrentTranslation` and `b2PrismaticJoint_GetCurrentSpeed`. `b2RevoluteJoint_GetMotorTorque` and `b2PrismaticJoint_GetMotorForce` are now `b2RevoluteJoint_GetCurrentMotorTorque` and `b2PrismaticJoint_GetCurrentMotorForce`.
- Distance, revolute, prismatic and wheel joints: `SetMotorSpeed`/`GetMotorSpeed` to `SetTargetMotorSpeed`/`GetTargetMotorSpeed`. `b2WheelJoint_GetMotorTorque` is now `b2WheelJoint_GetCurrentMotorTorque` and `b2DistanceJoint_GetMotorForce` is now `b2DistanceJoint_GetCurrentMotorForce`.
- Joint: `b2Joint_SetLocalAnchorA`/`B`, `b2Joint_SetLocalAxisA`, `b2Joint_SetReferenceAngle` and their getters are replaced by `b2Joint_SetLocalFrameA`/`B` and `b2Joint_GetLocalFrameA`/`B`.
- Motor joint: `b2MotorJoint_SetMaxForce`/`GetMaxForce` to `SetMaxVelocityForce`/`GetMaxVelocityForce` and `SetMaxTorque`/`GetMaxTorque` to `SetMaxVelocityTorque`/`GetMaxVelocityTorque`. The linear and angular offset and correction factor functions are removed.
- Character mover: `b2PlaneSolverResult::translation` is now `delta`.
- Math: `b2IsValidRotation` is now `b2IsValidRot`.
- Debug draw: callbacks take `b2Pos` and `b2WorldTransform`, `DrawPolygonFcn` takes a transform, and `DrawSolidCircleFcn` takes a local center. `DrawSegmentFcn` is now `DrawLineFcn`, `drawContactImpulses` is now `drawContactForces` and `drawFrictionImpulses` is now `drawFrictionForces`.
- CMake: `BOX2D_AVX2` is removed, AVX2 is selected at runtime.
