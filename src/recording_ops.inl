// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// X-macro manifest for the recording system.
// Include with B2_REC_OP and ARG defined. No commas between ARG tokens.
//
// B2_REC_OP( opcode, Name, RET, ARGS )
//   RET in { RET_NONE, RET_BODYID, RET_SHAPEID }
//   ARGS = zero or more ARG( TAG, fieldName ) tokens, NO commas between them
//
// Opcode ranges:
//   0x00-0x0F  world lifecycle and config
//   0x10-0x1F  body create/destroy
//   0x20-0x3F  body mutators
//   0x40-0x4F  shape create/destroy
//   0x50-0x6F  shape mutators
//   0x70-0x7F  chain
//   0x80       step
//   0x90-0xDF  joints (create, generic, per-type)
//   0xE0-0xEF  spatial queries
//   0xF0-0xFF  markers

// Recordings are seeded with a world snapshot, not a CreateWorld op. DestroyWorld stays as the
// end-of-session marker the viewer reads.
B2_REC_OP( 0x01, DestroyWorld, RET_NONE, ARG( WORLDID, world ) )
B2_REC_OP( 0x80, Step, RET_NONE, ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )

// World config. The world arg is informational; replay always targets its own world.
B2_REC_OP( 0x02, WorldEnableSleeping, RET_NONE, ARG( WORLDID, world ) ARG( BOOL, flag ) )
B2_REC_OP( 0x03, WorldEnableContinuous, RET_NONE, ARG( WORLDID, world ) ARG( BOOL, flag ) )
B2_REC_OP( 0x04, WorldSetRestitutionThreshold, RET_NONE, ARG( WORLDID, world ) ARG( F32, value ) )
B2_REC_OP( 0x05, WorldSetHitEventThreshold, RET_NONE, ARG( WORLDID, world ) ARG( F32, value ) )
B2_REC_OP( 0x06, WorldSetGravity, RET_NONE, ARG( WORLDID, world ) ARG( VEC2, gravity ) )
B2_REC_OP( 0x07, WorldExplode, RET_NONE, ARG( WORLDID, world ) ARG( EXPLOSIONDEF, def ) )
B2_REC_OP( 0x08, WorldSetContactTuning, RET_NONE,
		   ARG( WORLDID, world ) ARG( F32, hertz ) ARG( F32, dampingRatio ) ARG( F32, pushSpeed ) )
B2_REC_OP( 0x09, WorldSetContactRecycleDistance, RET_NONE, ARG( WORLDID, world ) ARG( F32, recycleDistance ) )
B2_REC_OP( 0x0A, WorldSetMaximumLinearSpeed, RET_NONE, ARG( WORLDID, world ) ARG( F32, maximumLinearSpeed ) )
B2_REC_OP( 0x0B, WorldEnableWarmStarting, RET_NONE, ARG( WORLDID, world ) ARG( BOOL, flag ) )
B2_REC_OP( 0x0C, WorldRebuildStaticTree, RET_NONE, ARG( WORLDID, world ) )
B2_REC_OP( 0x0D, WorldEnableSpeculative, RET_NONE, ARG( WORLDID, world ) ARG( BOOL, flag ) )

// Body
B2_REC_OP( 0x10, CreateBody, RET_BODYID, ARG( WORLDID, world ) ARG( BODYDEF, def ) )
B2_REC_OP( 0x11, DestroyBody, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x20, BodySetTransform, RET_NONE, ARG( BODYID, body ) ARG( POSITION, position ) ARG( ROT, rotation ) )
B2_REC_OP( 0x21, BodySetLinearVelocity, RET_NONE, ARG( BODYID, body ) ARG( VEC2, v ) )
B2_REC_OP( 0x22, BodySetType, RET_NONE, ARG( BODYID, body ) ARG( I32, type ) )
B2_REC_OP( 0x23, BodySetName, RET_NONE, ARG( BODYID, body ) ARG( STR, name ) )
B2_REC_OP( 0x24, BodySetAngularVelocity, RET_NONE, ARG( BODYID, body ) ARG( F32, w ) )
B2_REC_OP( 0x25, BodySetTargetTransform, RET_NONE, ARG( BODYID, body ) ARG( WORLDXF, target ) ARG( F32, timeStep ) ARG( BOOL, wake ) )
B2_REC_OP( 0x26, BodyApplyForce, RET_NONE, ARG( BODYID, body ) ARG( VEC2, force ) ARG( POSITION, point ) ARG( BOOL, wake ) )
B2_REC_OP( 0x27, BodyApplyForceToCenter, RET_NONE, ARG( BODYID, body ) ARG( VEC2, force ) ARG( BOOL, wake ) )
B2_REC_OP( 0x28, BodyApplyTorque, RET_NONE, ARG( BODYID, body ) ARG( F32, torque ) ARG( BOOL, wake ) )
B2_REC_OP( 0x29, BodyClearForces, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x2A, BodyApplyLinearImpulse, RET_NONE, ARG( BODYID, body ) ARG( VEC2, impulse ) ARG( POSITION, point ) ARG( BOOL, wake ) )
B2_REC_OP( 0x2B, BodyApplyLinearImpulseToCenter, RET_NONE, ARG( BODYID, body ) ARG( VEC2, impulse ) ARG( BOOL, wake ) )
B2_REC_OP( 0x2C, BodyApplyAngularImpulse, RET_NONE, ARG( BODYID, body ) ARG( F32, impulse ) ARG( BOOL, wake ) )
B2_REC_OP( 0x2D, BodySetMassData, RET_NONE, ARG( BODYID, body ) ARG( MASSDATA, massData ) )
B2_REC_OP( 0x2E, BodyApplyMassFromShapes, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x2F, BodySetLinearDamping, RET_NONE, ARG( BODYID, body ) ARG( F32, damping ) )
B2_REC_OP( 0x30, BodySetAngularDamping, RET_NONE, ARG( BODYID, body ) ARG( F32, damping ) )
B2_REC_OP( 0x31, BodySetGravityScale, RET_NONE, ARG( BODYID, body ) ARG( F32, scale ) )
B2_REC_OP( 0x32, BodySetAwake, RET_NONE, ARG( BODYID, body ) ARG( BOOL, awake ) )
B2_REC_OP( 0x33, BodyWakeTouching, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x34, BodyEnableSleep, RET_NONE, ARG( BODYID, body ) ARG( BOOL, flag ) )
B2_REC_OP( 0x35, BodySetSleepThreshold, RET_NONE, ARG( BODYID, body ) ARG( F32, threshold ) )
B2_REC_OP( 0x36, BodyDisable, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x37, BodyEnable, RET_NONE, ARG( BODYID, body ) )
B2_REC_OP( 0x38, BodySetMotionLocks, RET_NONE, ARG( BODYID, body ) ARG( LOCKS, locks ) )
B2_REC_OP( 0x39, BodySetBullet, RET_NONE, ARG( BODYID, body ) ARG( BOOL, flag ) )
B2_REC_OP( 0x3A, BodyEnableContactRecycling, RET_NONE, ARG( BODYID, body ) ARG( BOOL, flag ) )
B2_REC_OP( 0x3B, BodyEnableContactEvents, RET_NONE, ARG( BODYID, body ) ARG( BOOL, flag ) )
B2_REC_OP( 0x3C, BodyEnableHitEvents, RET_NONE, ARG( BODYID, body ) ARG( BOOL, flag ) )

// Shape create/destroy
B2_REC_OP( 0x40, CreateCircleShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( CIRCLE, circle ) )
B2_REC_OP( 0x41, CreateCapsuleShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( CAPSULE, capsule ) )
B2_REC_OP( 0x42, CreateSegmentShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( SEGMENT, segment ) )
B2_REC_OP( 0x43, CreatePolygonShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( POLYGON, polygon ) )
B2_REC_OP( 0x44, CreateChainSegmentShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( CHAINSEG, chainSegment ) )
B2_REC_OP( 0x45, DestroyShape, RET_NONE, ARG( SHAPEID, shape ) ARG( BOOL, updateBodyMass ) )

// Shape mutators
B2_REC_OP( 0x50, ShapeSetDensity, RET_NONE, ARG( SHAPEID, shape ) ARG( F32, density ) ARG( BOOL, updateBodyMass ) )
B2_REC_OP( 0x51, ShapeSetFriction, RET_NONE, ARG( SHAPEID, shape ) ARG( F32, friction ) )
B2_REC_OP( 0x52, ShapeSetRestitution, RET_NONE, ARG( SHAPEID, shape ) ARG( F32, restitution ) )
B2_REC_OP( 0x53, ShapeSetUserMaterial, RET_NONE, ARG( SHAPEID, shape ) ARG( U64, material ) )
B2_REC_OP( 0x54, ShapeSetSurfaceMaterial, RET_NONE, ARG( SHAPEID, shape ) ARG( MATERIAL, material ) )
B2_REC_OP( 0x55, ShapeSetFilter, RET_NONE, ARG( SHAPEID, shape ) ARG( FILTER, filter ) )
B2_REC_OP( 0x56, ShapeEnableSensorEvents, RET_NONE, ARG( SHAPEID, shape ) ARG( BOOL, flag ) )
B2_REC_OP( 0x57, ShapeEnableContactEvents, RET_NONE, ARG( SHAPEID, shape ) ARG( BOOL, flag ) )
B2_REC_OP( 0x58, ShapeEnablePreSolveEvents, RET_NONE, ARG( SHAPEID, shape ) ARG( BOOL, flag ) )
B2_REC_OP( 0x59, ShapeEnableHitEvents, RET_NONE, ARG( SHAPEID, shape ) ARG( BOOL, flag ) )
B2_REC_OP( 0x5A, ShapeSetCircle, RET_NONE, ARG( SHAPEID, shape ) ARG( CIRCLE, circle ) )
B2_REC_OP( 0x5B, ShapeSetCapsule, RET_NONE, ARG( SHAPEID, shape ) ARG( CAPSULE, capsule ) )
B2_REC_OP( 0x5C, ShapeSetSegment, RET_NONE, ARG( SHAPEID, shape ) ARG( SEGMENT, segment ) )
B2_REC_OP( 0x5D, ShapeSetPolygon, RET_NONE, ARG( SHAPEID, shape ) ARG( POLYGON, polygon ) )
B2_REC_OP( 0x5E, ShapeSetChainSegment, RET_NONE, ARG( SHAPEID, shape ) ARG( CHAINSEG, chainSegment ) )
B2_REC_OP( 0x5F, ShapeApplyWind, RET_NONE,
		   ARG( SHAPEID, shape ) ARG( VEC2, wind ) ARG( F32, drag ) ARG( F32, lift ) ARG( BOOL, wake ) )

// Chain
B2_REC_OP( 0x70, CreateChain, RET_CHAINID, ARG( BODYID, body ) ARG( CHAINDEF, def ) )
B2_REC_OP( 0x71, DestroyChain, RET_NONE, ARG( CHAINID, chain ) )
B2_REC_OP( 0x72, ChainSetSurfaceMaterial, RET_NONE, ARG( CHAINID, chain ) ARG( MATERIAL, material ) ARG( I32, materialIndex ) )

// Joint destroy
B2_REC_OP( 0x90, DestroyJoint, RET_NONE, ARG( JOINTID, joint ) )

// Generic joint mutators
B2_REC_OP( 0x91, JointSetLocalFrameA, RET_NONE, ARG( JOINTID, joint ) ARG( XF, localFrame ) )
B2_REC_OP( 0x92, JointSetLocalFrameB, RET_NONE, ARG( JOINTID, joint ) ARG( XF, localFrame ) )
B2_REC_OP( 0x93, JointSetCollideConnected, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, shouldCollide ) )
B2_REC_OP( 0x94, JointWakeBodies, RET_NONE, ARG( JOINTID, joint ) )
B2_REC_OP( 0x95, JointSetConstraintTuning, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0x96, JointSetForceThreshold, RET_NONE, ARG( JOINTID, joint ) ARG( F32, threshold ) )
B2_REC_OP( 0x97, JointSetTorqueThreshold, RET_NONE, ARG( JOINTID, joint ) ARG( F32, threshold ) )

// Distance joint
B2_REC_OP( 0x98, CreateDistanceJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( DISTANCEJOINTDEF, def ) )
B2_REC_OP( 0x99, DistanceJointSetLength, RET_NONE, ARG( JOINTID, joint ) ARG( F32, length ) )
B2_REC_OP( 0x9A, DistanceJointEnableSpring, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableSpring ) )
B2_REC_OP( 0x9B, DistanceJointSetSpringForceRange, RET_NONE, ARG( JOINTID, joint ) ARG( F32, lowerForce ) ARG( F32, upperForce ) )
B2_REC_OP( 0x9C, DistanceJointSetSpringHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0x9D, DistanceJointSetSpringDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0x9E, DistanceJointEnableLimit, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableLimit ) )
B2_REC_OP( 0x9F, DistanceJointSetLengthRange, RET_NONE, ARG( JOINTID, joint ) ARG( F32, minLength ) ARG( F32, maxLength ) )
B2_REC_OP( 0xA0, DistanceJointEnableMotor, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableMotor ) )
B2_REC_OP( 0xA1, DistanceJointSetMotorSpeed, RET_NONE, ARG( JOINTID, joint ) ARG( F32, motorSpeed ) )
B2_REC_OP( 0xA2, DistanceJointSetMaxMotorForce, RET_NONE, ARG( JOINTID, joint ) ARG( F32, force ) )

// Filter joint
B2_REC_OP( 0xA3, CreateFilterJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( FILTERJOINTDEF, def ) )

// Motor joint
B2_REC_OP( 0xA4, CreateMotorJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( MOTORJOINTDEF, def ) )
B2_REC_OP( 0xA5, MotorJointSetLinearVelocity, RET_NONE, ARG( JOINTID, joint ) ARG( VEC2, velocity ) )
B2_REC_OP( 0xA6, MotorJointSetAngularVelocity, RET_NONE, ARG( JOINTID, joint ) ARG( F32, velocity ) )
B2_REC_OP( 0xA7, MotorJointSetMaxVelocityForce, RET_NONE, ARG( JOINTID, joint ) ARG( F32, maxForce ) )
B2_REC_OP( 0xA8, MotorJointSetMaxVelocityTorque, RET_NONE, ARG( JOINTID, joint ) ARG( F32, maxTorque ) )
B2_REC_OP( 0xA9, MotorJointSetLinearHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xAA, MotorJointSetLinearDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, damping ) )
B2_REC_OP( 0xAB, MotorJointSetAngularHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xAC, MotorJointSetAngularDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, damping ) )
B2_REC_OP( 0xAD, MotorJointSetMaxSpringForce, RET_NONE, ARG( JOINTID, joint ) ARG( F32, maxForce ) )
B2_REC_OP( 0xAE, MotorJointSetMaxSpringTorque, RET_NONE, ARG( JOINTID, joint ) ARG( F32, maxTorque ) )

// Mover joint
B2_REC_OP( 0xAF, CreateMoverJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( MOVERJOINTDEF, def ) )
B2_REC_OP( 0xB0, MoverJointSetLinearVelocity, RET_NONE, ARG( JOINTID, joint ) ARG( VEC2, velocity ) )
B2_REC_OP( 0xB1, MoverJointSetMaxVelocityForce, RET_NONE, ARG( JOINTID, joint ) ARG( VEC2, maxForce ) )

// Pogo joint
B2_REC_OP( 0xB2, CreatePogoJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( POGOJOINTDEF, def ) )
B2_REC_OP( 0xB3, PogoJointSetRestLength, RET_NONE, ARG( JOINTID, joint ) ARG( F32, length ) )
B2_REC_OP( 0xB4, PogoJointSetSpringHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xB5, PogoJointSetSpringDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )

// Prismatic joint
B2_REC_OP( 0xB6, CreatePrismaticJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( PRISMATICJOINTDEF, def ) )
B2_REC_OP( 0xB7, PrismaticJointEnableSpring, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableSpring ) )
B2_REC_OP( 0xB8, PrismaticJointSetSpringHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xB9, PrismaticJointSetSpringDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0xBA, PrismaticJointSetTargetTranslation, RET_NONE, ARG( JOINTID, joint ) ARG( F32, translation ) )
B2_REC_OP( 0xBB, PrismaticJointEnableLimit, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableLimit ) )
B2_REC_OP( 0xBC, PrismaticJointSetLimits, RET_NONE, ARG( JOINTID, joint ) ARG( F32, lower ) ARG( F32, upper ) )
B2_REC_OP( 0xBD, PrismaticJointEnableMotor, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableMotor ) )
B2_REC_OP( 0xBE, PrismaticJointSetMotorSpeed, RET_NONE, ARG( JOINTID, joint ) ARG( F32, motorSpeed ) )
B2_REC_OP( 0xBF, PrismaticJointSetMaxMotorForce, RET_NONE, ARG( JOINTID, joint ) ARG( F32, force ) )

// Revolute joint
B2_REC_OP( 0xC0, CreateRevoluteJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( REVOLUTEJOINTDEF, def ) )
B2_REC_OP( 0xC1, RevoluteJointEnableSpring, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableSpring ) )
B2_REC_OP( 0xC2, RevoluteJointSetSpringHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xC3, RevoluteJointSetSpringDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0xC4, RevoluteJointSetTargetAngle, RET_NONE, ARG( JOINTID, joint ) ARG( F32, angle ) )
B2_REC_OP( 0xC5, RevoluteJointEnableLimit, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableLimit ) )
B2_REC_OP( 0xC6, RevoluteJointSetLimits, RET_NONE, ARG( JOINTID, joint ) ARG( F32, lower ) ARG( F32, upper ) )
B2_REC_OP( 0xC7, RevoluteJointEnableMotor, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableMotor ) )
B2_REC_OP( 0xC8, RevoluteJointSetMotorSpeed, RET_NONE, ARG( JOINTID, joint ) ARG( F32, motorSpeed ) )
B2_REC_OP( 0xC9, RevoluteJointSetMaxMotorTorque, RET_NONE, ARG( JOINTID, joint ) ARG( F32, torque ) )

// Weld joint
B2_REC_OP( 0xCA, CreateWeldJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( WELDJOINTDEF, def ) )
B2_REC_OP( 0xCB, WeldJointSetLinearHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xCC, WeldJointSetLinearDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0xCD, WeldJointSetAngularHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xCE, WeldJointSetAngularDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )

// Wheel joint
B2_REC_OP( 0xCF, CreateWheelJoint, RET_JOINTID, ARG( WORLDID, world ) ARG( WHEELJOINTDEF, def ) )
B2_REC_OP( 0xD0, WheelJointEnableSpring, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableSpring ) )
B2_REC_OP( 0xD1, WheelJointSetSpringHertz, RET_NONE, ARG( JOINTID, joint ) ARG( F32, hertz ) )
B2_REC_OP( 0xD2, WheelJointSetSpringDampingRatio, RET_NONE, ARG( JOINTID, joint ) ARG( F32, dampingRatio ) )
B2_REC_OP( 0xD3, WheelJointEnableLimit, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableLimit ) )
B2_REC_OP( 0xD4, WheelJointSetLimits, RET_NONE, ARG( JOINTID, joint ) ARG( F32, lower ) ARG( F32, upper ) )
B2_REC_OP( 0xD5, WheelJointEnableMotor, RET_NONE, ARG( JOINTID, joint ) ARG( BOOL, enableMotor ) )
B2_REC_OP( 0xD6, WheelJointSetMotorSpeed, RET_NONE, ARG( JOINTID, joint ) ARG( F32, motorSpeed ) )
B2_REC_OP( 0xD7, WheelJointSetMaxMotorTorque, RET_NONE, ARG( JOINTID, joint ) ARG( F32, torque ) )

// Spatial queries. Inputs through the manifest (reader side). Hit tail / results hand-written.
// Query geometry is relative to the origin so recordings reproduce queries far from the world origin.
B2_REC_OP( 0xE0, QueryOverlapAABB, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( AABB, aabb ) ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE1, QueryOverlapShape, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( SHAPEPROXY, proxy ) ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE2, QueryCastRay, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( VEC2, translation ) ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE3, QueryCastShape, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( SHAPEPROXY, proxy ) ARG( VEC2, translation )
			   ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE4, QueryCollideMover, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( CAPSULE, mover ) ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE5, QueryCastRayClosest, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( VEC2, translation ) ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE6, QueryCastMover, RET_NONE,
		   ARG( WORLDID, world ) ARG( POSITION, origin ) ARG( CAPSULE, mover ) ARG( VEC2, translation )
			   ARG( QUERYFILTER, filter ) )
B2_REC_OP( 0xE7, ShapeTestPoint, RET_NONE, ARG( SHAPEID, shape ) ARG( POSITION, point ) )
B2_REC_OP( 0xE8, ShapeRayCast, RET_NONE, ARG( SHAPEID, shape ) ARG( POSITION, origin ) ARG( VEC2, translation ) )

B2_REC_OP( 0xF1, StateHash, RET_NONE, ARG( WORLDID, world ) ARG( U64, hash ) )

// Accumulated world bounds over the whole recording, written once at stop. Informational.
B2_REC_OP( 0xF2, RecordingBounds, RET_NONE, ARG( AABB, bounds ) )
