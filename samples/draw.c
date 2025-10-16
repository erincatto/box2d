// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "draw.h"

#include "container.h"
#include "shader.h"

#include "box2d/math_functions.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#if defined( _MSC_VER )
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
#else
#include <stdlib.h>
#endif

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

//#include <imgui.h>

#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define BUFFER_OFFSET( x ) ( (const void*)( x ) )

#define SHADER_TEXT( x ) "#version 330\n" #x

typedef struct
{
	uint8_t r, g, b, a;
} RGBA8;

static RGBA8 MakeRGBA8( b2HexColor c, float alpha )
{
	return {
		(uint8_t)( ( c >> 16 ) & 0xFF ),
		(uint8_t)( ( c >> 8 ) & 0xFF ),
		(uint8_t)( c & 0xFF ),
		(uint8_t)( 0xFF * alpha ),
	};
}

Camera GetDefaultCamera( void )
{
	return (Camera){
		.m_center = { 0.0f, 20.0f },
		.m_zoom = 1.0f,
		.m_width = 1920.0f,
		.m_height = 1080.0f,
	};
}

void ResetView( Camera* camera )
{
	camera->m_center = (b2Vec2){ 0.0f, 20.0f };
	camera->m_zoom = 1.0f;
}

b2Vec2 ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint )
{
	float w = camera->m_width;
	float h = camera->m_height;
	float u = screenPoint.x / w;
	float v = ( h - screenPoint.y ) / h;

	float ratio = w / h;
	b2Vec2 extents = { camera->m_zoom * ratio, camera->m_zoom };

	b2Vec2 lower = b2Sub( camera->m_center, extents );
	b2Vec2 upper = b2Add( camera->m_center, extents );

	b2Vec2 pw = { ( 1.0f - u ) * lower.x + u * upper.x, ( 1.0f - v ) * lower.y + v * upper.y };
	return pw;
}

b2Vec2 ConvertWorldToScreen( Camera* camera, b2Vec2 worldPoint )
{
	float w = camera->m_width;
	float h = camera->m_height;
	float ratio = w / h;

	b2Vec2 extents = { camera->m_zoom * ratio, camera->m_zoom };

	b2Vec2 lower = b2Sub( camera->m_center, extents );
	b2Vec2 upper = b2Add( camera->m_center, extents );

	float u = ( worldPoint.x - lower.x ) / ( upper.x - lower.x );
	float v = ( worldPoint.y - lower.y ) / ( upper.y - lower.y );

	b2Vec2 ps = { u * w, ( 1.0f - v ) * h };
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
// This also includes the view transform
void BuildProjectionMatrix( Camera* camera, float* m, float zBias )
{
	float ratio = camera->m_width / camera->m_height;
	b2Vec2 extents = { camera->m_zoom * ratio, camera->m_zoom };

	b2Vec2 lower = b2Sub( camera->m_center, extents );
	b2Vec2 upper = b2Add( camera->m_center, extents );
	float w = upper.x - lower.x;
	float h = upper.y - lower.y;

	m[0] = 2.0f / w;
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / h;
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = -1.0f;
	m[11] = 0.0f;

	m[12] = -2.0f * camera->m_center.x / w;
	m[13] = -2.0f * camera->m_center.y / h;
	m[14] = zBias;
	m[15] = 1.0f;
}

void BuildProjectionMatrixNoZoom( Camera* camera, float* m )
{
	m[0] = camera->m_height / camera->m_width;
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 1.0f;
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = -1.0f;
	m[11] = 0.0f;

	m[12] = 0.0f;
	m[13] = 0.0f;
	m[14] = 0.0f;
	m[15] = 1.0f;
}

b2AABB GetViewBounds( Camera* camera )
{
	if ( camera->m_height == 0.0f || camera->m_width == 0.0f )
	{
		b2AABB bounds = { .lowerBound = b2Vec2_zero, .upperBound = b2Vec2_zero };
		return bounds;
	}

	b2AABB bounds;
	bounds.lowerBound = ConvertScreenToWorld( camera, (b2Vec2){ 0.0f, camera->m_height } );
	bounds.upperBound = ConvertScreenToWorld( camera, (b2Vec2){ camera->m_width, 0.0f } );
	return bounds;
}

typedef struct
{
	b2Vec2 position;
	b2Vec2 uv;
	RGBA8 color;
} FontVertex;

static const int m_firstCharacter = 32;
static const int m_characterCount = 96;
static const int m_atlasWidth = 512;
static const int m_atlasHeight = 512;
// The number of vertices the vbo can hold
static const int m_vboCapacity = 6 * 10000;

#define FONT_VERTEX_CAPACITY ( 4 * m_vboCapacity )

typedef struct
{
	float m_fontSize;
	stbtt_bakedchar* m_characters;
	FontVertex* m_vertices;
	int vertexCount;
	unsigned int m_textureId;
	uint32_t m_vaoId;
	uint32_t m_vboId;
	uint32_t m_programId;
} Font;

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

Font CreateFont( int fontSize, const char* trueTypeFile )
{
	Font font = { 0 };

	FILE* file = fopen( trueTypeFile, "rb" );
	if ( file == NULL )
	{
		assert( false );
		return font;
	}

	font.m_fontSize = fontSize;
	font.m_characters = malloc( m_characterCount * sizeof( stbtt_bakedchar ) );
	font.m_vertices = malloc( FONT_VERTEX_CAPACITY * sizeof( FontVertex ) );

	int fileBufferCapacity = 1 << 20;
	unsigned char* fileBuffer = (unsigned char*)malloc( fileBufferCapacity * sizeof( unsigned char ) );
	fread( fileBuffer, 1, fileBufferCapacity, file );

	int pw = m_atlasWidth;
	int ph = m_atlasHeight;
	unsigned char* tempBitmap = (unsigned char*)malloc( pw * ph * sizeof( unsigned char ) );
	stbtt_BakeFontBitmap( fileBuffer, 0, font.m_fontSize, tempBitmap, pw, ph, m_firstCharacter, m_characterCount,
						  font.m_characters );

	glGenTextures( 1, &font.m_textureId );
	glBindTexture( GL_TEXTURE_2D, font.m_textureId );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_R8, pw, ph, 0, GL_RED, GL_UNSIGNED_BYTE, tempBitmap );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	// for debugging
	// stbi_write_png( "build/fontAtlas.png", pw, ph, 1, tempBitmap, pw );

	fclose( file );
	free( fileBuffer );
	free( tempBitmap );
	fileBuffer = NULL;
	tempBitmap = NULL;

	font.m_programId = CreateProgramFromStrings( vertexShaderSrc, fragmentShaderSrc );
	if ( font.m_programId == 0 )
	{
		return font;
	}

	// Setting up the VAO and VBO
	glGenBuffers( 1, &font.m_vboId );
	glBindBuffer( GL_ARRAY_BUFFER, font.m_vboId );
	glBufferData( GL_ARRAY_BUFFER, m_vboCapacity * sizeof( FontVertex ), NULL, GL_DYNAMIC_DRAW );

	glGenVertexArrays( 1, &font.m_vaoId );
	glBindVertexArray( font.m_vaoId );

	// position attribute
	glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, sizeof( FontVertex ), (void*)offsetof( FontVertex, position ) );
	glEnableVertexAttribArray( 0 );

	// uv attribute
	glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, sizeof( FontVertex ), (void*)offsetof( FontVertex, uv ) );
	glEnableVertexAttribArray( 1 );

	// color attribute will be expanded to floats using normalization
	glVertexAttribPointer( 2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( FontVertex ), (void*)offsetof( FontVertex, color ) );
	glEnableVertexAttribArray( 2 );

	glBindVertexArray( 0 );

	CheckOpenGL();

	return font;
}

void DestroyFont( Font* font )
{
	if ( font->m_programId != 0 )
	{
		glDeleteProgram( font->m_programId );
	}

	glDeleteBuffers( 1, &font->m_vboId );
	glDeleteVertexArrays( 1, &font->m_vaoId );

	if ( font->m_textureId != 0 )
	{
		glDeleteTextures( 1, &font->m_textureId );
	}

	free( font->m_characters );
}

void AddText( Font* font, float x, float y, b2HexColor color, const char* text )
{
	if ( text == NULL )
	{
		return;
	}

	b2Vec2 position = { x, y };
	RGBA8 c = MakeRGBA8( color, 1.0f );
	int pw = m_atlasWidth;
	int ph = m_atlasHeight;

	int i = 0;
	while ( text[i] != 0 && font->vertexCount < FONT_VERTEX_CAPACITY - 6 )
	{
		int index = (int)text[i] - m_firstCharacter;

		if ( 0 <= index && index < m_characterCount )
		{
			// 1=opengl
			stbtt_aligned_quad q;
			stbtt_GetBakedQuad( font->m_characters, pw, ph, index, &position.x, &position.y, &q, 1 );

			FontVertex v1 = { { q.x0, q.y0 }, { q.s0, q.t0 }, c };
			FontVertex v2 = { { q.x1, q.y0 }, { q.s1, q.t0 }, c };
			FontVertex v3 = { { q.x1, q.y1 }, { q.s1, q.t1 }, c };
			FontVertex v4 = { { q.x0, q.y1 }, { q.s0, q.t1 }, c };

			int vertexIndex = font->vertexCount;
			font->m_vertices[vertexIndex + 0] = v1;
			font->m_vertices[vertexIndex + 1] = v3;
			font->m_vertices[vertexIndex + 2] = v2;
			font->m_vertices[vertexIndex + 3] = v1;
			font->m_vertices[vertexIndex + 4] = v4;
			font->m_vertices[vertexIndex + 5] = v3;
			font->vertexCount += 6;
		}

		i += 1;
	}
}

void FlushText( Font* font, Camera* camera )
{
	float projectionMatrix[16];
	BuildProjectionMatrixNoZoom( camera, projectionMatrix );

	glUseProgram( font->m_programId );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glDisable( GL_DEPTH_TEST );

	int slot = 0;
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, font->m_textureId );

	glBindVertexArray( font->m_vaoId );
	glBindBuffer( GL_ARRAY_BUFFER, font->m_vboId );

	int textureUniform = glGetUniformLocation( font->m_programId, "FontAtlas" );
	glUniform1i( textureUniform, slot );

	int matrixUniform = glGetUniformLocation( font->m_programId, "ProjectionMatrix" );
	glUniformMatrix4fv( matrixUniform, 1, GL_FALSE, projectionMatrix );

	int totalVertexCount = font->vertexCount;
	int drawCallCount = ( totalVertexCount / m_vboCapacity ) + 1;

	for ( int i = 0; i < drawCallCount; i++ )
	{
		const FontVertex* data = font->m_vertices + i * m_vboCapacity;

		int vertexCount;
		if ( i == drawCallCount - 1 )
		{
			vertexCount = totalVertexCount % m_vboCapacity;
		}
		else
		{
			vertexCount = m_vboCapacity;
		}

		glBufferSubData( GL_ARRAY_BUFFER, 0, vertexCount * sizeof( FontVertex ), data );
		glDrawArrays( GL_TRIANGLES, 0, vertexCount );
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glBindTexture( GL_TEXTURE_2D, 0 );

	glDisable( GL_BLEND );
	glEnable( GL_DEPTH_TEST );

	CheckOpenGL();

	font->vertexCount = 0;
}

typedef struct
{
	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_timeUniform;
	GLint m_resolutionUniform;
	GLint m_baseColorUniform;
} GLBackground;

GLBackground CreateBackground()
{
	GLBackground background = { 0 };

	background.m_programId = CreateProgramFromFiles( "samples/data/background.vs", "samples/data/background.fs" );
	background.m_timeUniform = glGetUniformLocation( background.m_programId, "time" );
	background.m_resolutionUniform = glGetUniformLocation( background.m_programId, "resolution" );
	background.m_baseColorUniform = glGetUniformLocation( background.m_programId, "baseColor" );
	int vertexAttribute = 0;

	// Generate
	glGenVertexArrays( 1, &background.m_vaoId );
	glGenBuffers( 1, &background.m_vboId );

	glBindVertexArray( background.m_vaoId );
	glEnableVertexAttribArray( vertexAttribute );

	// Single quad
	b2Vec2 vertices[] = { { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, 1.0f }, { 1.0f, -1.0f } };
	glBindBuffer( GL_ARRAY_BUFFER, background.m_vboId );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return background;
}

void DestroyBackground( GLBackground* background )
{
	if ( background->m_vaoId )
	{
		glDeleteVertexArrays( 1, &background->m_vaoId );
		glDeleteBuffers( 1, &background->m_vboId );
		background->m_vaoId = 0;
		background->m_vboId = 0;
	}

	if ( background->m_programId )
	{
		glDeleteProgram( background->m_programId );
		background->m_programId = 0;
	}
}

void DrawBackground( GLBackground* background, Camera* camera )
{
	glUseProgram( background->m_programId );

	float time = (float)glfwGetTime();
	time = fmodf( time, 100.0f );

	glUniform1f( background->m_timeUniform, time );
	glUniform2f( background->m_resolutionUniform, (float)camera->m_width, (float)camera->m_height );

	// struct RGBA8 c8 = MakeRGBA8( b2_colorGray2, 1.0f );
	// glUniform3f(m_baseColorUniform, c8.r/255.0f, c8.g/255.0f, c8.b/255.0f);
	glUniform3f( background->m_baseColorUniform, 0.2f, 0.2f, 0.2f );

	glBindVertexArray( background->m_vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, background->m_vboId );
	glDrawArrays( GL_TRIANGLE_STRIP, 0, 4 );
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );
}

#define POINT_BATCH_SIZE 2048

typedef struct
{
	b2Vec2 position;
	float size;
	RGBA8 rgba;
} PointData;

struct GLPoints
{
	std::vector<PointData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

void Create()
{
	const char* vs = "#version 330\n"
					 "uniform mat4 projectionMatrix;\n"
					 "layout(location = 0) in vec2 v_position;\n"
					 "layout(location = 1) in float v_size;\n"
					 "layout(location = 2) in vec4 v_color;\n"
					 "out vec4 f_color;\n"
					 "void main(void)\n"
					 "{\n"
					 "	f_color = v_color;\n"
					 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
					 "	gl_PointSize = v_size;\n"
					 "}\n";

	const char* fs = "#version 330\n"
					 "in vec4 f_color;\n"
					 "out vec4 color;\n"
					 "void main(void)\n"
					 "{\n"
					 "	color = f_color;\n"
					 "}\n";

	m_programId = CreateProgramFromStrings( vs, fs );
	m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
	int vertexAttribute = 0;
	int sizeAttribute = 1;
	int colorAttribute = 2;

	// Generate
	glGenVertexArrays( 1, &m_vaoId );
	glGenBuffers( 1, &m_vboId );

	glBindVertexArray( m_vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( sizeAttribute );
	glEnableVertexAttribArray( colorAttribute );

	// Vertex buffer
	glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
	glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( PointData ), nullptr, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, position ) );
	glVertexAttribPointer( sizeAttribute, 1, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, size ) );
	// save bandwidth by expanding color to floats in the shader
	glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( PointData ),
						   (void*)offsetof( PointData, rgba ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
}

void Destroy()
{
	if ( m_vaoId )
	{
		glDeleteVertexArrays( 1, &m_vaoId );
		glDeleteBuffers( 1, &m_vboId );
		m_vaoId = 0;
		m_vboId = 0;
	}

	if ( m_programId )
	{
		glDeleteProgram( m_programId );
		m_programId = 0;
	}
}

// todo instead of flushing, keep a growable array of data
// this will prevent sorting problems.

void AddPoint( b2Vec2 v, float size, b2HexColor c )
{
	RGBA8 rgba = MakeRGBA8( c, 1.0f );
	m_points.push_back( { v, size, rgba } );
}

void Flush( Camera* camera )
{
	int count = (int)m_points.size();
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( m_programId );

	float proj[16] = { 0.0f };
	camera->BuildProjectionMatrix( proj, 0.0f );

	glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
	glBindVertexArray( m_vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
	glEnable( GL_PROGRAM_POINT_SIZE );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, e_batchSize );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( PointData ), &m_points[base] );
		glDrawArrays( GL_POINTS, 0, batchCount );

		CheckOpenGL();

		count -= e_batchSize;
		base += e_batchSize;
	}

	glDisable( GL_PROGRAM_POINT_SIZE );
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	m_points.clear();
}

struct VertexData
{
	b2Vec2 position;
	RGBA8 rgba;
};

struct GLLines
{
	void Create()
	{
		const char* vs =
			R"(
			#version 330
			uniform mat4 projectionMatrix;
			layout(location = 0) in vec2 v_position;
			layout(location = 1) in vec4 v_color;
			out vec4 f_color;
			void main(void)
			{
				f_color = v_color;
				gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);
			}
			)";

		const char* fs =
			R"(
			#version 330
			in vec4 f_color;
			out vec4 color;
			void main(void)
			{
				color = f_color;
			}
			)";

		m_programId = CreateProgramFromStrings( vs, fs );
		m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
		int vertexAttribute = 0;
		int colorAttribute = 1;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 1, &m_vboId );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( colorAttribute );

		// Vertex buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( VertexData ), nullptr, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, position ) );
		// save bandwidth by expanding color to floats in the shader
		glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, rgba ) );

		CheckOpenGL();

		// Cleanup
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	void Destroy()
	{
		if ( m_vaoId )
		{
			glDeleteVertexArrays( 1, &m_vaoId );
			glDeleteBuffers( 1, &m_vboId );
			m_vaoId = 0;
			m_vboId = 0;
		}

		if ( m_programId )
		{
			glDeleteProgram( m_programId );
			m_programId = 0;
		}
	}

	void AddLine( b2Vec2 p1, b2Vec2 p2, b2HexColor c )
	{
		RGBA8 rgba = MakeRGBA8( c, 1.0f );
		m_points.push_back( { p1, rgba } );
		m_points.push_back( { p2, rgba } );
	}

	void Flush( Camera* camera )
	{
		int count = (int)m_points.size();
		if ( count == 0 )
		{
			return;
		}

		assert( count % 2 == 0 );

		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		camera->BuildProjectionMatrix( proj, 0.1f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_batchSize );
			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( VertexData ), &m_points[base] );

			glDrawArrays( GL_LINES, 0, batchCount );

			CheckOpenGL();

			count -= e_batchSize;
			base += e_batchSize;
		}

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		glDisable( GL_BLEND );

		m_points.clear();
	}

	// need lots of space for lines so they draw last
	// could also consider disabling depth buffer
	enum
	{
		// must be multiple of 2
		e_batchSize = 2 * 2048
	};

	std::vector<VertexData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

struct CircleData
{
	b2Vec2 position;
	float radius;
	RGBA8 rgba;
};

struct GLCircles
{
	void Create()
	{
		m_programId = CreateProgramFromFiles( "samples/data/circle.vs", "samples/data/circle.fs" );
		m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
		m_pixelScaleUniform = glGetUniformLocation( m_programId, "pixelScale" );
		int vertexAttribute = 0;
		int positionInstance = 1;
		int radiusInstance = 2;
		int colorInstance = 3;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 2, m_vboIds );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( positionInstance );
		glEnableVertexAttribArray( radiusInstance );
		glEnableVertexAttribArray( colorInstance );

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[0] );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

		// Circle buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( CircleData ), nullptr, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( positionInstance, 2, GL_FLOAT, GL_FALSE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, position ) );
		glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, radius ) );
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, rgba ) );

		glVertexAttribDivisor( positionInstance, 1 );
		glVertexAttribDivisor( radiusInstance, 1 );
		glVertexAttribDivisor( colorInstance, 1 );

		CheckOpenGL();

		// Cleanup
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	void Destroy()
	{
		if ( m_vaoId )
		{
			glDeleteVertexArrays( 1, &m_vaoId );
			glDeleteBuffers( 2, m_vboIds );
			m_vaoId = 0;
			m_vboIds[0] = 0;
			m_vboIds[1] = 0;
		}

		if ( m_programId )
		{
			glDeleteProgram( m_programId );
			m_programId = 0;
		}
	}

	void AddCircle( b2Vec2 center, float radius, b2HexColor color )
	{
		RGBA8 rgba = MakeRGBA8( color, 1.0f );
		m_circles.push_back( { center, radius, rgba } );
	}

	void Flush( Camera* camera )
	{
		int count = (int)m_circles.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		camera->BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, camera->m_height / camera->m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_batchSize );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( CircleData ), &m_circles[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckOpenGL();

			count -= e_batchSize;
			base += e_batchSize;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_circles.clear();
	}

	enum
	{
		e_batchSize = 2048
	};

	std::vector<CircleData> m_circles;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_pixelScaleUniform;
};

struct SolidCircleData
{
	b2Transform transform;
	float radius;
	RGBA8 rgba;
};

// Draws SDF circles using quad instancing. Apparently instancing of quads can be slow on older GPUs.
// https://www.reddit.com/r/opengl/comments/q7yikr/how_to_draw_several_quads_through_instancing/
// https://www.g-truc.net/post-0666.html
struct GLSolidCircles
{
	void Create()
	{
		m_programId = CreateProgramFromFiles( "samples/data/solid_circle.vs", "samples/data/solid_circle.fs" );
		m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
		m_pixelScaleUniform = glGetUniformLocation( m_programId, "pixelScale" );

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 2, m_vboIds );

		glBindVertexArray( m_vaoId );

		int vertexAttribute = 0;
		int transformInstance = 1;
		int radiusInstance = 2;
		int colorInstance = 3;
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( transformInstance );
		glEnableVertexAttribArray( radiusInstance );
		glEnableVertexAttribArray( colorInstance );

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[0] );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

		// Circle buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( SolidCircleData ), nullptr, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, transform ) );
		glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, radius ) );
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, rgba ) );

		glVertexAttribDivisor( transformInstance, 1 );
		glVertexAttribDivisor( radiusInstance, 1 );
		glVertexAttribDivisor( colorInstance, 1 );

		CheckOpenGL();

		// Cleanup
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	void Destroy()
	{
		if ( m_vaoId )
		{
			glDeleteVertexArrays( 1, &m_vaoId );
			glDeleteBuffers( 2, m_vboIds );
			m_vaoId = 0;
			m_vboIds[0] = 0;
			m_vboIds[1] = 0;
		}

		if ( m_programId )
		{
			glDeleteProgram( m_programId );
			m_programId = 0;
		}
	}

	void AddCircle( const b2Transform& transform, float radius, b2HexColor color )
	{
		RGBA8 rgba = MakeRGBA8( color, 1.0f );
		m_circles.push_back( { transform, radius, rgba } );
	}

	void Flush( Camera* camera )
	{
		int count = (int)m_circles.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		camera->BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, camera->m_height / camera->m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_batchSize );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( SolidCircleData ), &m_circles[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckOpenGL();

			count -= e_batchSize;
			base += e_batchSize;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_circles.clear();
	}

	enum
	{
		e_batchSize = 2048
	};

	std::vector<SolidCircleData> m_circles;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_pixelScaleUniform;
};

struct CapsuleData
{
	b2Transform transform;
	float radius;
	float length;
	RGBA8 rgba;
};

// Draw capsules using SDF-based shader
struct GLSolidCapsules
{
	void Create()
	{
		m_programId = CreateProgramFromFiles( "samples/data/solid_capsule.vs", "samples/data/solid_capsule.fs" );

		m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
		m_pixelScaleUniform = glGetUniformLocation( m_programId, "pixelScale" );

		int vertexAttribute = 0;
		int transformInstance = 1;
		int radiusInstance = 2;
		int lengthInstance = 3;
		int colorInstance = 4;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 2, m_vboIds );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( transformInstance );
		glEnableVertexAttribArray( radiusInstance );
		glEnableVertexAttribArray( lengthInstance );
		glEnableVertexAttribArray( colorInstance );

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[0] );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

		// Capsule buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( CapsuleData ), nullptr, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof( CapsuleData ),
							   (void*)offsetof( CapsuleData, transform ) );
		glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( CapsuleData ),
							   (void*)offsetof( CapsuleData, radius ) );
		glVertexAttribPointer( lengthInstance, 1, GL_FLOAT, GL_FALSE, sizeof( CapsuleData ),
							   (void*)offsetof( CapsuleData, length ) );
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( CapsuleData ),
							   (void*)offsetof( CapsuleData, rgba ) );

		glVertexAttribDivisor( transformInstance, 1 );
		glVertexAttribDivisor( radiusInstance, 1 );
		glVertexAttribDivisor( lengthInstance, 1 );
		glVertexAttribDivisor( colorInstance, 1 );

		CheckOpenGL();

		// Cleanup
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	void Destroy()
	{
		if ( m_vaoId )
		{
			glDeleteVertexArrays( 1, &m_vaoId );
			glDeleteBuffers( 2, m_vboIds );
			m_vaoId = 0;
			m_vboIds[0] = 0;
			m_vboIds[1] = 0;
		}

		if ( m_programId )
		{
			glDeleteProgram( m_programId );
			m_programId = 0;
		}
	}

	void AddCapsule( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor c )
	{
		b2Vec2 d = p2 - p1;
		float length = b2Length( d );
		if ( length < 0.001f )
		{
			printf( "WARNING: sample app: capsule too short!\n" );
			return;
		}

		b2Vec2 axis = { d.x / length, d.y / length };
		b2Transform transform;
		transform.p = 0.5f * ( p1 + p2 );
		transform.q.c = axis.x;
		transform.q.s = axis.y;

		RGBA8 rgba = MakeRGBA8( c, 1.0f );

		m_capsules.push_back( { transform, radius, length, rgba } );
	}

	void Flush( Camera* camera )
	{
		int count = (int)m_capsules.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		camera->BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, camera->m_height / camera->m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_batchSize );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( CapsuleData ), &m_capsules[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckOpenGL();

			count -= e_batchSize;
			base += e_batchSize;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_capsules.clear();
	}

	enum
	{
		e_batchSize = 2048
	};

	std::vector<CapsuleData> m_capsules;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_pixelScaleUniform;
};

struct PolygonData
{
	b2Transform transform;
	b2Vec2 p1, p2, p3, p4, p5, p6, p7, p8;
	int count;
	float radius;

	// keep color small
	RGBA8 color;
};

// Rounded and non-rounded convex polygons using an SDF-based shader.
struct GLSolidPolygons
{
	void Create()
	{
		m_programId = CreateProgramFromFiles( "samples/data/solid_polygon.vs", "samples/data/solid_polygon.fs" );

		m_projectionUniform = glGetUniformLocation( m_programId, "projectionMatrix" );
		m_pixelScaleUniform = glGetUniformLocation( m_programId, "pixelScale" );
		int vertexAttribute = 0;
		int instanceTransform = 1;
		int instancePoint12 = 2;
		int instancePoint34 = 3;
		int instancePoint56 = 4;
		int instancePoint78 = 5;
		int instancePointCount = 6;
		int instanceRadius = 7;
		int instanceColor = 8;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 2, m_vboIds );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( instanceTransform );
		glEnableVertexAttribArray( instancePoint12 );
		glEnableVertexAttribArray( instancePoint34 );
		glEnableVertexAttribArray( instancePoint56 );
		glEnableVertexAttribArray( instancePoint78 );
		glEnableVertexAttribArray( instancePointCount );
		glEnableVertexAttribArray( instanceRadius );
		glEnableVertexAttribArray( instanceColor );

		// Vertex buffer for single quad
		float a = 1.1f;
		b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[0] );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

		// Polygon buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glBufferData( GL_ARRAY_BUFFER, e_batchSize * sizeof( PolygonData ), nullptr, GL_DYNAMIC_DRAW );
		glVertexAttribPointer( instanceTransform, 4, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, transform ) );
		glVertexAttribPointer( instancePoint12, 4, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, p1 ) );
		glVertexAttribPointer( instancePoint34, 4, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, p3 ) );
		glVertexAttribPointer( instancePoint56, 4, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, p5 ) );
		glVertexAttribPointer( instancePoint78, 4, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, p7 ) );
		glVertexAttribIPointer( instancePointCount, 1, GL_INT, sizeof( PolygonData ), (void*)offsetof( PolygonData, count ) );
		glVertexAttribPointer( instanceRadius, 1, GL_FLOAT, GL_FALSE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, radius ) );
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer( instanceColor, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( PolygonData ),
							   (void*)offsetof( PolygonData, color ) );

		// These divisors tell glsl how to distribute per instance data
		glVertexAttribDivisor( instanceTransform, 1 );
		glVertexAttribDivisor( instancePoint12, 1 );
		glVertexAttribDivisor( instancePoint34, 1 );
		glVertexAttribDivisor( instancePoint56, 1 );
		glVertexAttribDivisor( instancePoint78, 1 );
		glVertexAttribDivisor( instancePointCount, 1 );
		glVertexAttribDivisor( instanceRadius, 1 );
		glVertexAttribDivisor( instanceColor, 1 );

		CheckOpenGL();

		// Cleanup
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
	}

	void Destroy()
	{
		if ( m_vaoId )
		{
			glDeleteVertexArrays( 1, &m_vaoId );
			glDeleteBuffers( 2, m_vboIds );
			m_vaoId = 0;
		}

		if ( m_programId )
		{
			glDeleteProgram( m_programId );
			m_programId = 0;
		}
	}

	void AddPolygon( const b2Transform& transform, const b2Vec2* points, int count, float radius, b2HexColor color )
	{
		PolygonData data = {};
		data.transform = transform;

		int n = count < 8 ? count : 8;
		b2Vec2* ps = &data.p1;
		for ( int i = 0; i < n; ++i )
		{
			ps[i] = points[i];
		}

		data.count = n;
		data.radius = radius;
		data.color = MakeRGBA8( color, 1.0f );

		m_polygons.push_back( data );
	}

	void Flush( Camera* camera )
	{
		int count = (int)m_polygons.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		camera->BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, camera->m_height / camera->m_zoom );

		glBindVertexArray( m_vaoId );
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );

		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_batchSize );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( PolygonData ), &m_polygons[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );
			CheckOpenGL();

			count -= e_batchSize;
			base += e_batchSize;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_polygons.clear();
	}

	enum
	{
		e_batchSize = 512
	};

	std::vector<PolygonData> m_polygons;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_pixelScaleUniform;
};

void DrawPolygonFcn( const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawPolygon( vertices, vertexCount, color );
}

void DrawSolidPolygonFcn( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color,
						  void* context )
{
	static_cast<Draw*>( context )->DrawSolidPolygon( transform, vertices, vertexCount, radius, color );
}

void DrawCircleFcn( b2Vec2 center, float radius, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawCircle( center, radius, color );
}

void DrawSolidCircleFcn( b2Transform transform, float radius, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawSolidCircle( transform, b2Vec2_zero, radius, color );
}

void DrawSolidCapsuleFcn( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawSolidCapsule( p1, p2, radius, color );
}

void DrawSegmentFcn( b2Vec2 p1, b2Vec2 p2, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawLine( p1, p2, color );
}

void DrawTransformFcn( b2Transform transform, void* context )
{
	static_cast<Draw*>( context )->DrawTransform( transform );
}

void DrawPointFcn( b2Vec2 p, float size, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawPoint( p, size, color );
}

void DrawStringFcn( b2Vec2 p, const char* s, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawString( p, s );
}

Draw::Draw()
{
	m_camera = nullptr;
	m_showUI = true;
	m_points = nullptr;
	m_lines = nullptr;
	m_circles = nullptr;
	m_solidCircles = nullptr;
	m_solidCapsules = nullptr;
	m_solidPolygons = nullptr;
	m_debugDraw = {};
	m_regularFont = nullptr;
	m_mediumFont = nullptr;
	m_largeFont = nullptr;
	m_background = nullptr;
}

Draw::~Draw()
{
	assert( m_points == nullptr );
	assert( m_lines == nullptr );
	assert( m_circles == nullptr );
	assert( m_solidCircles == nullptr );
	assert( m_solidCapsules == nullptr );
	assert( m_solidPolygons == nullptr );
	assert( m_background == nullptr );
}

void Draw::Create( Camera* camera )
{
	m_camera = camera;
	m_background = new GLBackground;
	m_background->Create();
	m_points = new GLPoints;
	m_points->Create();
	m_lines = new GLLines;
	m_lines->Create();
	m_circles = new GLCircles;
	m_circles->Create();
	m_solidCircles = new GLSolidCircles;
	m_solidCircles->Create();
	m_solidCapsules = new GLSolidCapsules;
	m_solidCapsules->Create();
	m_solidPolygons = new GLSolidPolygons;
	m_solidPolygons->Create();

	b2AABB bounds = { { -FLT_MAX, -FLT_MAX }, { FLT_MAX, FLT_MAX } };

	m_debugDraw = {};

	m_debugDraw.DrawPolygonFcn = DrawPolygonFcn;
	m_debugDraw.DrawSolidPolygonFcn = DrawSolidPolygonFcn;
	m_debugDraw.DrawCircleFcn = DrawCircleFcn;
	m_debugDraw.DrawSolidCircleFcn = DrawSolidCircleFcn;
	m_debugDraw.DrawSolidCapsuleFcn = DrawSolidCapsuleFcn;
	m_debugDraw.DrawSegmentFcn = DrawSegmentFcn;
	m_debugDraw.DrawTransformFcn = DrawTransformFcn;
	m_debugDraw.DrawPointFcn = DrawPointFcn;
	m_debugDraw.DrawStringFcn = DrawStringFcn;
	m_debugDraw.drawingBounds = bounds;

	m_debugDraw.drawShapes = true;
	m_debugDraw.drawJoints = true;
	m_debugDraw.drawJointExtras = false;
	m_debugDraw.drawBounds = false;
	m_debugDraw.drawMass = false;
	m_debugDraw.drawContacts = false;
	m_debugDraw.drawGraphColors = false;
	m_debugDraw.drawContactNormals = false;
	m_debugDraw.drawContactForces = false;
	m_debugDraw.drawContactFeatures = false;
	m_debugDraw.drawFrictionForces = false;
	m_debugDraw.drawIslands = false;

	m_debugDraw.context = this;
}

void Draw::Destroy()
{
	m_background->Destroy();
	delete m_background;
	m_background = nullptr;

	m_points->Destroy();
	delete m_points;
	m_points = nullptr;

	m_lines->Destroy();
	delete m_lines;
	m_lines = nullptr;

	m_circles->Destroy();
	delete m_circles;
	m_circles = nullptr;

	m_solidCircles->Destroy();
	delete m_solidCircles;
	m_solidCircles = nullptr;

	m_solidCapsules->Destroy();
	delete m_solidCapsules;
	m_solidCapsules = nullptr;

	m_solidPolygons->Destroy();
	delete m_solidPolygons;
	m_solidPolygons = nullptr;
}

void Draw::DrawPolygon( const b2Vec2* vertices, int vertexCount, b2HexColor color )
{
	b2Vec2 p1 = vertices[vertexCount - 1];
	for ( int i = 0; i < vertexCount; ++i )
	{
		b2Vec2 p2 = vertices[i];
		m_lines->AddLine( p1, p2, color );
		p1 = p2;
	}
}

void Draw::DrawSolidPolygon( b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius, b2HexColor color )
{
	m_solidPolygons->AddPolygon( transform, vertices, vertexCount, radius, color );
}

void Draw::DrawCircle( b2Vec2 center, float radius, b2HexColor color )
{
	m_circles->AddCircle( center, radius, color );
}

void Draw::DrawSolidCircle( b2Transform transform, b2Vec2 center, float radius, b2HexColor color )
{
	transform.p = b2TransformPoint( transform, center );
	m_solidCircles->AddCircle( transform, radius, color );
}

void Draw::DrawSolidCapsule( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color )
{
	m_solidCapsules->AddCapsule( p1, p2, radius, color );
}

void Draw::DrawLine( b2Vec2 p1, b2Vec2 p2, b2HexColor color )
{
	m_lines->AddLine( p1, p2, color );
}

void Draw::DrawTransform( b2Transform transform )
{
	const float k_axisScale = 0.2f;
	b2Vec2 p1 = transform.p;

	b2Vec2 p2 = b2MulAdd( p1, k_axisScale, b2Rot_GetXAxis( transform.q ) );
	m_lines->AddLine( p1, p2, b2_colorRed );

	p2 = b2MulAdd( p1, k_axisScale, b2Rot_GetYAxis( transform.q ) );
	m_lines->AddLine( p1, p2, b2_colorGreen );
}

void Draw::DrawPoint( b2Vec2 p, float size, b2HexColor color )
{
#ifdef __APPLE__
	size *= 2.0f;
#endif
	m_points->AddPoint( p, size, color );
}

void Draw::DrawString( int x, int y, const char* string, ... )
{
	// if (m_showUI == false)
	//{
	//	return;
	// }

	va_list arg;
	va_start( arg, string );
	ImGui::Begin( "Overlay", nullptr,
				  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
					  ImGuiWindowFlags_NoScrollbar );
	ImGui::SetCursorPos( ImVec2( float( x ), float( y ) ) );
	ImGui::TextColoredV( ImColor( 230, 153, 153, 255 ), string, arg );
	ImGui::End();
	va_end( arg );
}

void Draw::DrawString( b2Vec2 p, const char* string, ... )
{
	b2Vec2 ps = m_camera->ConvertWorldToScreen( p );

	va_list arg;
	va_start( arg, string );
	ImGui::Begin( "Overlay", nullptr,
				  ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize |
					  ImGuiWindowFlags_NoScrollbar );
	ImGui::SetCursorPos( ImVec2( ps.x, ps.y ) );
	ImGui::TextColoredV( ImColor( 230, 230, 230, 255 ), string, arg );
	ImGui::End();
	va_end( arg );
}

void Draw::DrawBounds( b2AABB aabb, b2HexColor c )
{
	b2Vec2 p1 = aabb.lowerBound;
	b2Vec2 p2 = { aabb.upperBound.x, aabb.lowerBound.y };
	b2Vec2 p3 = aabb.upperBound;
	b2Vec2 p4 = { aabb.lowerBound.x, aabb.upperBound.y };

	m_lines->AddLine( p1, p2, c );
	m_lines->AddLine( p2, p3, c );
	m_lines->AddLine( p3, p4, c );
	m_lines->AddLine( p4, p1, c );
}

void Draw::Flush()
{
	m_solidCircles->Flush( m_camera );
	m_solidCapsules->Flush( m_camera );
	m_solidPolygons->Flush( m_camera );
	m_circles->Flush( m_camera );
	m_lines->Flush( m_camera );
	m_points->Flush( m_camera );
	CheckOpenGL();
}

void Draw::DrawBackground()
{
	m_background->Draw( m_camera );
}
