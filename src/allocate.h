// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

void* b2Alloc( int size );
void b2Free( void* mem, int size );
void* b2GrowAlloc( void* oldMem, int oldSize, int newSize );
