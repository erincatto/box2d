// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdbool.h>
#include <stdint.h>

// Reader state threaded through the replay loop and all dispatch functions
typedef struct b2RecReader
{
	const uint8_t* data;
	int size;
	int cursor;
	b2WorldId replayWorldId; // world created during replay; valid after CreateWorld record
	int workerCount;         // 0 = use recorded count
	bool ok;                 // false on read overrun, id mismatch, or StateHash divergence
} b2RecReader;

// Read primitives

uint8_t b2RecR_U8( b2RecReader* rdr );
uint16_t b2RecR_U16( b2RecReader* rdr );
uint32_t b2RecR_U24( b2RecReader* rdr );
uint32_t b2RecR_U32( b2RecReader* rdr );
uint64_t b2RecR_U64( b2RecReader* rdr );
int32_t b2RecR_I32( b2RecReader* rdr );
float b2RecR_F32( b2RecReader* rdr );
bool b2RecR_BOOL( b2RecReader* rdr );
b2Vec2 b2RecR_VEC2( b2RecReader* rdr );
b2Rot b2RecR_ROT( b2RecReader* rdr );
b2Transform b2RecR_XF( b2RecReader* rdr );
b2WorldId b2RecR_WORLDID( b2RecReader* rdr );
b2BodyId b2RecR_BODYID( b2RecReader* rdr );
b2ShapeId b2RecR_SHAPEID( b2RecReader* rdr );
b2Circle b2RecR_CIRCLE( b2RecReader* rdr );
b2WorldDef b2RecR_WORLDDEF( b2RecReader* rdr );
b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr );
b2ShapeDef b2RecR_SHAPEDEF( b2RecReader* rdr );
