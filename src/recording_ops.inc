// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// X-macro manifest for the recording system.
// Include with B2_REC_OP and ARG defined; no commas between ARG tokens.
//
// B2_REC_OP( opcode, Name, RET, ARGS )
//   RET in { RET_NONE, RET_BODYID, RET_SHAPEID }
//   ARGS = zero or more ARG( TAG, fieldName ) tokens, NO commas between them
//
// Opcode ranges:
//   0x00-0x0F  world lifecycle
//   0x10-0x1F  body create/destroy
//   0x20-0x3F  body mutators
//   0x40-0x4F  shape create
//   0x80       step
//   0xF0-0xFF  markers

B2_REC_OP( 0x00, CreateWorld,           RET_NONE,    ARG( WORLDDEF, def ) )
B2_REC_OP( 0x01, DestroyWorld,          RET_NONE,    ARG( WORLDID, world ) )
B2_REC_OP( 0x80, Step,                  RET_NONE,    ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )
B2_REC_OP( 0x10, CreateBody,            RET_BODYID,  ARG( WORLDID, world ) ARG( BODYDEF, def ) )
B2_REC_OP( 0x11, DestroyBody,           RET_NONE,    ARG( BODYID, body ) )
B2_REC_OP( 0x20, BodySetTransform,      RET_NONE,    ARG( BODYID, body ) ARG( VEC2, position ) ARG( ROT, rotation ) )
B2_REC_OP( 0x21, BodySetLinearVelocity, RET_NONE,    ARG( BODYID, body ) ARG( VEC2, v ) )
B2_REC_OP( 0x40, CreateCircleShape,     RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( CIRCLE, circle ) )
B2_REC_OP( 0xF1, StateHash,             RET_NONE,    ARG( WORLDID, world ) ARG( U64, hash ) )
