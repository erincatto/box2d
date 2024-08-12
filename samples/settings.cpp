// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS
#include "settings.h"

// todo consider using https://github.com/skeeto/pdjson
#include <assert.h>
#include <jsmn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* fileName = "settings.ini";

// Load a file. You must free the character array.
static bool ReadFile( char*& data, int& size, const char* filename )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		return false;
	}

	fseek( file, 0, SEEK_END );
	size = (int)ftell( file );
	fseek( file, 0, SEEK_SET );

	if ( size == 0 )
	{
		return false;
	}

	data = (char*)malloc( size + 1 );
	fread( data, size, 1, file );
	fclose( file );
	data[size] = 0;

	return true;
}

void Settings::Save()
{
	FILE* file = fopen( fileName, "w" );
	fprintf( file, "{\n" );
	fprintf( file, "  \"sampleIndex\": %d,\n", sampleIndex );
	fprintf( file, "  \"drawShapes\": %s,\n", drawShapes ? "true" : "false" );
	fprintf( file, "  \"drawJoints\": %s,\n", drawJoints ? "true" : "false" );
	fprintf( file, "  \"drawAABBs\": %s,\n", drawAABBs ? "true" : "false" );
	fprintf( file, "  \"drawContactPoints\": %s,\n", drawContactPoints ? "true" : "false" );
	fprintf( file, "  \"drawContactNormals\": %s,\n", drawContactNormals ? "true" : "false" );
	fprintf( file, "  \"drawContactImpulses\": %s,\n", drawContactImpulses ? "true" : "false" );
	fprintf( file, "  \"drawFrictionImpulse\": %s,\n", drawFrictionImpulses ? "true" : "false" );
	fprintf( file, "  \"drawMass\": %s,\n", drawMass ? "true" : "false" );
	fprintf( file, "  \"drawCounters\": %s,\n", drawCounters ? "true" : "false" );
	fprintf( file, "  \"drawProfile\": %s,\n", drawProfile ? "true" : "false" );
	fprintf( file, "  \"enableWarmStarting\": %s,\n", enableWarmStarting ? "true" : "false" );
	fprintf( file, "  \"enableContinuous\": %s,\n", enableContinuous ? "true" : "false" );
	fprintf( file, "  \"enableSleep\": %s\n", enableSleep ? "true" : "false" );
	fprintf( file, "}\n" );
	fclose( file );
}

static int jsoneq( const char* json, jsmntok_t* tok, const char* s )
{
	if ( tok->type == JSMN_STRING && (int)strlen( s ) == tok->end - tok->start &&
		 strncmp( json + tok->start, s, tok->end - tok->start ) == 0 )
	{
		return 0;
	}
	return -1;
}

#define MAX_TOKENS 32

void Settings::Load()
{
	char* data = nullptr;
	int size = 0;
	bool found = ReadFile( data, size, fileName );
	if ( found == false )
	{
		return;
	}

	jsmn_parser parser;
	jsmntok_t tokens[MAX_TOKENS];

	jsmn_init( &parser );

	// js - pointer to JSON string
	// tokens - an array of tokens available
	// 10 - number of tokens available
	int tokenCount = jsmn_parse( &parser, data, size, tokens, MAX_TOKENS );
	char buffer[32];

	for ( int i = 0; i < tokenCount; ++i )
	{
		if ( jsoneq( data, &tokens[i], "sampleIndex" ) == 0 )
		{
			int count = tokens[i + 1].end - tokens[i + 1].start;
			assert( count < 32 );
			const char* s = data + tokens[i + 1].start;
			strncpy( buffer, s, count );
			buffer[count] = 0;
			char* dummy;
			sampleIndex = (int)strtol( buffer, &dummy, 10 );
		}
		else if ( jsoneq( data, &tokens[i], "drawShapes" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				drawShapes = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				drawShapes = false;
			}
		}
		else if ( jsoneq( data, &tokens[i], "drawJoints" ) == 0 )
		{
			const char* s = data + tokens[i + 1].start;
			if ( strncmp( s, "true", 4 ) == 0 )
			{
				drawJoints = true;
			}
			else if ( strncmp( s, "false", 5 ) == 0 )
			{
				drawJoints = false;
			}
		}
	}

	free( data );
}
