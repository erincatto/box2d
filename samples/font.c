// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "font.h"

#include "camera.h"
#include "shader.h"
#include "utility.h"

#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// clang-format off
#include "glad/glad.h"
#include "GLFW/glfw3.h"
// clang-format on

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static const char* vertexShaderSrc = R"( 
#version 330 core

layout (location = 0) in vec2 aPosition;
layout (location = 1) in vec2 aUV;
layout (location = 2) in vec4 aColor;

out vec4 color;
out vec2 uv;

uniform mat4 ProjectionMatrix;

void main()
{
	gl_Position = ProjectionMatrix * vec4(aPosition, 0.0, 1.0);

	color = aColor;
	uv = aUV;
}
)";

static const char* fragmentShaderSrc = R"(
#version 330 core

in vec4 color;
in vec2 uv;

uniform sampler2D FontAtlas;

out vec4 fragColor;

void main()
{
	fragColor = vec4(color.rgb, color.a * texture(FontAtlas, uv).r);
}
)";

Font::Font( const Camera* camera, float fontSize )
{
	m_camera = camera;
	m_fontSize = fontSize;
	m_characters = (stbtt_bakedchar*)malloc( m_characterCount * sizeof( stbtt_bakedchar ) );
	m_textureId = 0;
	m_vaoId = 0;
	m_vboId = 0;
	m_programId = 0;
}

Font::~Font()
{
	if ( m_programId != 0 )
	{
		DestroyShader( m_programId );
	}

	glDeleteBuffers( 1, &m_vboId );
	glDeleteVertexArrays( 1, &m_vaoId );

	if ( m_textureId != 0 )
	{
		glDeleteTextures( 1, &m_textureId );
	}

	free( m_characters );
}

void Font::AddText( float x, float y, b3HexColor color, const char* text )
{
	if (text == nullptr)
	{
		return;
	}

	b3Vec3 position = { x, y, 0.0f };
	RGBA8 c = MakeRGBA8( color );
	int pw = m_atlasWidth;
	int ph = m_atlasHeight;

	int i = 0;
	while ( text[i] != 0 )
	{
		int index = (int)text[i] - m_firstCharacter;

		if ( 0 <= index && index < m_characterCount )
		{
			// 1=opengl
			stbtt_aligned_quad q;
			stbtt_GetBakedQuad( m_characters, pw, ph, index, &position.x, &position.y, &q, 1 );

			Vertex v1 = { { q.x0, q.y0 }, { q.s0, q.t0 }, c };
			Vertex v2 = { { q.x1, q.y0 }, { q.s1, q.t0 }, c };
			Vertex v3 = { { q.x1, q.y1 }, { q.s1, q.t1 }, c };
			Vertex v4 = { { q.x0, q.y1 }, { q.s0, q.t1 }, c };

			m_vertices.push_back( v1 );
			m_vertices.push_back( v3 );
			m_vertices.push_back( v2 );
			m_vertices.push_back( v1 );
			m_vertices.push_back( v4 );
			m_vertices.push_back( v3 );
		}

		i += 1;
	}
}

void Font::Flush()
{
	Matrix4 projectionMatrix = MakeOrthographicMatrix( 0.0f, m_camera->m_width, m_camera->m_height,  0.0f, -1.0f, 1.0f );

	glUseProgram( m_programId );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glDisable( GL_DEPTH_TEST );

	int slot = 0;
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, m_textureId );

	glBindVertexArray( m_vaoId );
	glBindBuffer( GL_ARRAY_BUFFER, m_vboId );

	int textureUniform = glGetUniformLocation( m_programId, "FontAtlas" );
	glUniform1i( textureUniform, slot );

	int matrixUniform = glGetUniformLocation( m_programId, "ProjectionMatrix" );
	glUniformMatrix4fv( matrixUniform, 1, GL_FALSE, &projectionMatrix.cx.x );

	int totalVertexCount = int( m_vertices.size() );
	int drawCallCount = ( totalVertexCount / m_vboCapacity ) + 1;

	for ( int i = 0; i < drawCallCount; i++ )
	{
		const Vertex* data = m_vertices.data() + i * m_vboCapacity;

		int vertexCount;
		if ( i == drawCallCount - 1 )
		{
			vertexCount = totalVertexCount % m_vboCapacity;
		}
		else
		{
			vertexCount = m_vboCapacity;
		}

		glBufferSubData( GL_ARRAY_BUFFER, 0, vertexCount * sizeof( Vertex ), data );
		glDrawArrays( GL_TRIANGLES, 0, vertexCount );
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glBindTexture( GL_TEXTURE_2D, 0 );

	glDisable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );

	CheckOpenGL();

	m_vertices.clear();
}

bool Font::Initialize( const char* path )
{
	// Create atlas
	FILE* file = fopen( path, "rb" );
	if ( file == nullptr )
	{
		assert( false );
		return false;
	}

	int fileBufferCapacity = 1 << 20;
	unsigned char* fileBuffer = (unsigned char*)malloc( fileBufferCapacity * sizeof( unsigned char ) );
	fread( fileBuffer, 1, fileBufferCapacity, file );

	int pw = m_atlasWidth;
	int ph = m_atlasHeight;
	unsigned char* tempBitmap = (unsigned char*)malloc( pw * ph * sizeof( unsigned char ) );
	stbtt_BakeFontBitmap( fileBuffer, 0, m_fontSize, tempBitmap, pw, ph, m_firstCharacter, m_characterCount, m_characters );

	glGenTextures( 1, &m_textureId );
	glBindTexture( GL_TEXTURE_2D, m_textureId );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_R8, pw, ph, 0, GL_RED, GL_UNSIGNED_BYTE, tempBitmap );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	stbi_write_png( "build/fontAtlas.png", pw, ph, 1, tempBitmap, pw );

	fclose( file );
	free( fileBuffer );
	free( tempBitmap );
	fileBuffer = nullptr;
	tempBitmap = nullptr;

	m_programId = CreateProgramFromStrings( vertexShaderSrc, fragmentShaderSrc );
	if ( m_programId == 0 )
	{
		return false;
	}

	// Setting up the VAO and VBO
	glGenBuffers( 1, &m_vboId );
	glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
	glBufferData( GL_ARRAY_BUFFER, m_vboCapacity * sizeof( Vertex ), nullptr, GL_DYNAMIC_DRAW );

	glGenVertexArrays( 1, &m_vaoId );
	glBindVertexArray( m_vaoId );

	// position attribute
	glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, sizeof( Vertex ), (void*)offsetof( Vertex, position ) );
	glEnableVertexAttribArray( 0 );

	// uv attribute
	glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( Vertex ), (void*)offsetof( Vertex, uv ) );
	glEnableVertexAttribArray( 1 );

	// color attribute will be expanded to floats using normalization
	glVertexAttribPointer( 2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( Vertex ), (void*)offsetof( Vertex, color ) );
	glEnableVertexAttribArray( 2 );

	glBindVertexArray( 0 );

	CheckOpenGL();

	return true;
}

// For testing
// Matrix4 MakeProjectionMatrix( float width, float height )
//{
//	Matrix4 m;
//	m.cx = { 2.0f / width, 0.0f, 0.0f, 0.0f };
//	m.cy = { 0.0f, 2.0f / height, 0.0f, 0.0f };
//	m.cz = { 0.0f, 0.0f, -1.0f, 0.0f };
//	m.cw = { -1.0f, -1.0f, 0.0f, 1.0f };
//	return m;
//}

Font* Font::Create( const Camera* camera, const char* path, int fontSize )
{
	Font* font = new Font( camera, fontSize );
	if ( font->Initialize( path ) == false )
	{
		delete font;
		font = nullptr;
	}

	return font;
}
