// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#define _CRT_SECURE_NO_WARNINGS

#include "shader.h"

#include <assert.h>
#include <glad/glad.h>
#include <stdio.h>

#if defined( _WIN32 )
	#define _CRTDBG_MAP_ALLOC
	#include <crtdbg.h>
	#include <stdlib.h>
#else
	#include <stdlib.h>
#endif

void DumpInfoGL()
{
	const char* renderer = (const char*)glGetString( GL_RENDERER );
	const char* vendor = (const char*)glGetString( GL_VENDOR );
	const char* version = (const char*)glGetString( GL_VERSION );
	const char* glslVersion = (const char*)glGetString( GL_SHADING_LANGUAGE_VERSION );

	int major, minor;
	glGetIntegerv( GL_MAJOR_VERSION, &major );
	glGetIntegerv( GL_MINOR_VERSION, &minor );

	printf( "-------------------------------------------------------------\n" );
	printf( "GL Vendor    : %s\n", vendor );
	printf( "GL Renderer  : %s\n", renderer );
	printf( "GL Version   : %s\n", version );
	printf( "GL Version   : %d.%d\n", major, minor );
	printf( "GLSL Version : %s\n", glslVersion );
	printf( "-------------------------------------------------------------\n" );
}

void CheckErrorGL()
{
	GLenum errCode = glGetError();
	if ( errCode != GL_NO_ERROR )
	{
		printf( "OpenGL error = %d\n", errCode );
		assert( false );
	}
}

void PrintLogGL( uint32_t object )
{
	GLint log_length = 0;
	if ( glIsShader( object ) )
	{
		glGetShaderiv( object, GL_INFO_LOG_LENGTH, &log_length );
	}
	else if ( glIsProgram( object ) )
	{
		glGetProgramiv( object, GL_INFO_LOG_LENGTH, &log_length );
	}
	else
	{
		printf( "PrintLogGL: Not a shader or a program\n" );
		return;
	}

	char* log = (char*)malloc( log_length );

	if ( glIsShader( object ) )
	{
		glGetShaderInfoLog( object, log_length, nullptr, log );
	}
	else if ( glIsProgram( object ) )
	{
		glGetProgramInfoLog( object, log_length, nullptr, log );
	}

	printf( "PrintLogGL: %s", log );
	free( log );
}

static GLuint sCreateShaderFromString( const char* source, GLenum type )
{
	GLuint shader = glCreateShader( type );
	const char* sources[] = { source };

	glShaderSource( shader, 1, sources, nullptr );
	glCompileShader( shader );

	int success = GL_FALSE;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &success );

	if ( success == GL_FALSE )
	{
		printf( "Error compiling shader of type %d!\n", type );
		PrintLogGL( shader );
		glDeleteShader( shader );
		return 0;
	}

	return shader;
}

uint32_t CreateProgramFromStrings( const char* vertexString, const char* fragmentString )
{
	GLuint vertex = sCreateShaderFromString( vertexString, GL_VERTEX_SHADER );
	if ( vertex == 0 )
	{
		return 0;
	}

	GLuint fragment = sCreateShaderFromString( fragmentString, GL_FRAGMENT_SHADER );
	if ( fragment == 0 )
	{
		return 0;
	}

	GLuint program = glCreateProgram();
	glAttachShader( program, vertex );
	glAttachShader( program, fragment );

	glLinkProgram( program );

	int success = GL_FALSE;
	glGetProgramiv( program, GL_LINK_STATUS, &success );
	if ( success == GL_FALSE )
	{
		printf( "glLinkProgram:" );
		PrintLogGL( program );
		return 0;
	}

	glDeleteShader( vertex );
	glDeleteShader( fragment );

	return program;
}

static GLuint sCreateShaderFromFile( const char* filename, GLenum type )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		fprintf( stderr, "Error opening %s\n", filename );
		return 0;
	}

	fseek( file, 0, SEEK_END );
	long size = ftell( file );
	fseek( file, 0, SEEK_SET );

	char* source = static_cast<char*>( malloc( size + 1 ) );
	size_t count = fread( source, size, 1, file );
	fclose( file );

	source[size] = 0;

	GLuint shader = glCreateShader( type );
	const char* sources[] = { source };

	glShaderSource( shader, 1, sources, nullptr );
	glCompileShader( shader );

	int success = GL_FALSE;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &success );

	if ( success == GL_FALSE )
	{
		fprintf( stderr, "Error compiling shader of type %d!\n", type );
		PrintLogGL( shader );
	}

	free( source );
	return shader;
}

uint32_t CreateProgramFromFiles( const char* vertexPath, const char* fragmentPath )
{
	GLuint vertex = sCreateShaderFromFile( vertexPath, GL_VERTEX_SHADER );
	if ( vertex == 0 )
	{
		return 0;
	}

	GLuint fragment = sCreateShaderFromFile( fragmentPath, GL_FRAGMENT_SHADER );
	if ( fragment == 0 )
	{
		return 0;
	}

	GLuint program = glCreateProgram();
	glAttachShader( program, vertex );
	glAttachShader( program, fragment );

	glLinkProgram( program );

	int success = GL_FALSE;
	glGetProgramiv( program, GL_LINK_STATUS, &success );
	if ( success == GL_FALSE )
	{
		printf( "glLinkProgram:" );
		PrintLogGL( program );
		return 0;
	}

	glDeleteShader( vertex );
	glDeleteShader( fragment );

	return program;
}
