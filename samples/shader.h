// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

uint32_t CreateProgramFromFiles( const char* vertexPath, const char* fragmentPath );
uint32_t CreateProgramFromStrings( const char* vertexString, const char* fragmentString );

void CheckOpenGL();
void DumpInfoGL();
void PrintLogGL( int object );
