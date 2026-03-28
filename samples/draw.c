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

#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "stb_image_write.h"

#define BUFFER_OFFSET( x ) ( (const void*)( x ) )

#define SHADER_TEXT( x ) "#version 330\n" #x

typedef struct
{
	uint8_t r, g, b, a;
} RGBA8;

static inline RGBA8 MakeRGBA8( b2HexColor c, float alpha )
{
	return (RGBA8){
		(uint8_t)( ( c >> 16 ) & 0xFF ),
		(uint8_t)( ( c >> 8 ) & 0xFF ),
		(uint8_t)( c & 0xFF ),
		(uint8_t)( 0xFF * alpha ),
	};
}

Camera GetDefaultCamera( void )
{
	return (Camera){
		.center = { 0.0f, 20.0f },
		.zoom = 1.0f,
		.width = 1920.0f,
		.height = 1080.0f,
	};
}

void ResetView( Camera* camera )
{
	camera->center = (b2Vec2){ 0.0f, 20.0f };
	camera->zoom = 1.0f;
}

b2Vec2 ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint )
{
	float w = camera->width;
	float h = camera->height;
	float u = screenPoint.x / w;
	float v = ( h - screenPoint.y ) / h;

	float ratio = w / h;
	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	b2Vec2 lower = b2Sub( camera->center, extents );
	b2Vec2 upper = b2Add( camera->center, extents );

	b2Vec2 pw = { ( 1.0f - u ) * lower.x + u * upper.x, ( 1.0f - v ) * lower.y + v * upper.y };
	return pw;
}

b2Vec2 ConvertWorldToScreen( Camera* camera, b2Vec2 worldPoint )
{
	float w = camera->width;
	float h = camera->height;
	float ratio = w / h;

	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	b2Vec2 lower = b2Sub( camera->center, extents );
	b2Vec2 upper = b2Add( camera->center, extents );

	float u = ( worldPoint.x - lower.x ) / ( upper.x - lower.x );
	float v = ( worldPoint.y - lower.y ) / ( upper.y - lower.y );

	b2Vec2 ps = { u * w, ( 1.0f - v ) * h };
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
// This also includes the view transform
static void BuildProjectionMatrix( Camera* camera, float* m, float zBias )
{
	float ratio = camera->width / camera->height;
	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	b2Vec2 lower = b2Sub( camera->center, extents );
	b2Vec2 upper = b2Add( camera->center, extents );
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

	m[12] = -2.0f * camera->center.x / w;
	m[13] = -2.0f * camera->center.y / h;
	m[14] = zBias;
	m[15] = 1.0f;
}

static void MakeOrthographicMatrix( float* m, float left, float right, float bottom, float top, float near, float far )
{
	m[0] = 2.0f / ( right - left );
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / ( top - bottom );
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = -2.0f / ( far - near );
	m[11] = 0.0f;

	m[12] = -( right + left ) / ( right - left );
	m[13] = -( top + bottom ) / ( top - bottom );
	m[14] = -( far + near ) / ( far - near );
	m[15] = 1.0f;
}

b2AABB GetViewBounds( Camera* camera )
{
	if ( camera->height == 0.0f || camera->width == 0.0f )
	{
		b2AABB bounds = { .lowerBound = b2Vec2_zero, .upperBound = b2Vec2_zero };
		return bounds;
	}

	b2AABB bounds;
	bounds.lowerBound = ConvertScreenToWorld( camera, (b2Vec2){ 0.0f, camera->height } );
	bounds.upperBound = ConvertScreenToWorld( camera, (b2Vec2){ camera->width, 0.0f } );
	return bounds;
}

typedef struct
{
	b2Vec2 position;
	b2Vec2 uv;
	RGBA8 color;
} FontVertex;

ARRAY_DECLARE( FontVertex );
ARRAY_INLINE( FontVertex );
ARRAY_SOURCE( FontVertex );

#define FONT_FIRST_CHARACTER 32
#define FONT_CHARACTER_COUNT 96
#define FONT_ATLAS_WIDTH 512
#define FONT_ATLAS_HEIGHT 512

// The number of vertices the vbo can hold. Must be a multiple of 6.
#define FONT_BATCH_SIZE ( 6 * 10000 )

typedef struct
{
	float fontSize;
	FontVertexArray vertices;
	stbtt_bakedchar* characters;
	unsigned int textureId;
	uint32_t vaoId;
	uint32_t vboId;
	uint32_t programId;
} Font;

Font CreateFont( const char* trueTypeFile, float fontSize )
{
	Font font = { 0 };

	FILE* file = fopen( trueTypeFile, "rb" );
	if ( file == NULL )
	{
		assert( false );
		return font;
	}

	font.vertices = FontVertexArray_Create( FONT_BATCH_SIZE );
	font.fontSize = fontSize;
	font.characters = malloc( FONT_CHARACTER_COUNT * sizeof( stbtt_bakedchar ) );

	int fileBufferCapacity = 1 << 20;
	unsigned char* fileBuffer = (unsigned char*)malloc( fileBufferCapacity * sizeof( unsigned char ) );
	fread( fileBuffer, 1, fileBufferCapacity, file );

	int pw = FONT_ATLAS_WIDTH;
	int ph = FONT_ATLAS_HEIGHT;
	unsigned char* tempBitmap = (unsigned char*)malloc( pw * ph * sizeof( unsigned char ) );
	stbtt_BakeFontBitmap( fileBuffer, 0, font.fontSize, tempBitmap, pw, ph, FONT_FIRST_CHARACTER, FONT_CHARACTER_COUNT,
						  font.characters );

	glGenTextures( 1, &font.textureId );
	glBindTexture( GL_TEXTURE_2D, font.textureId );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_R8, pw, ph, 0, GL_RED, GL_UNSIGNED_BYTE, tempBitmap );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	// for debugging
	// stbi_write_png( "build/fontAtlas.png", pw, ph, 1, tempBitmap, pw );

	fclose( file );
	free( fileBuffer );
	free( tempBitmap );
	fileBuffer = NULL;
	tempBitmap = NULL;

	font.programId = CreateProgramFromFiles( "samples/data/font.vs", "samples/data/font.fs" );
	if ( font.programId == 0 )
	{
		return font;
	}

	// Setting up the VAO and VBO
	glGenBuffers( 1, &font.vboId );
	glBindBuffer( GL_ARRAY_BUFFER, font.vboId );
	glBufferData( GL_ARRAY_BUFFER, FONT_BATCH_SIZE * sizeof( FontVertex ), NULL, GL_DYNAMIC_DRAW );

	glGenVertexArrays( 1, &font.vaoId );
	glBindVertexArray( font.vaoId );

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
	if ( font->programId != 0 )
	{
		glDeleteProgram( font->programId );
	}

	glDeleteBuffers( 1, &font->vboId );
	glDeleteVertexArrays( 1, &font->vaoId );

	if ( font->textureId != 0 )
	{
		glDeleteTextures( 1, &font->textureId );
	}

	free( font->characters );

	FontVertexArray_Destroy( &font->vertices );
}

void AddText( Font* font, float x, float y, b2HexColor color, const char* text )
{
	if ( text == NULL )
	{
		return;
	}

	b2Vec2 position = { x, y };
	RGBA8 c = MakeRGBA8( color, 1.0f );
	int pw = FONT_ATLAS_WIDTH;
	int ph = FONT_ATLAS_HEIGHT;

	int i = 0;
	while ( text[i] != 0 )
	{
		int index = (int)text[i] - FONT_FIRST_CHARACTER;

		if ( 0 <= index && index < FONT_CHARACTER_COUNT )
		{
			// 1=opengl
			stbtt_aligned_quad q;
			stbtt_GetBakedQuad( font->characters, pw, ph, index, &position.x, &position.y, &q, 1 );

			FontVertex v1 = { { q.x0, q.y0 }, { q.s0, q.t0 }, c };
			FontVertex v2 = { { q.x1, q.y0 }, { q.s1, q.t0 }, c };
			FontVertex v3 = { { q.x1, q.y1 }, { q.s1, q.t1 }, c };
			FontVertex v4 = { { q.x0, q.y1 }, { q.s0, q.t1 }, c };

			FontVertexArray_Push( &font->vertices, v1 );
			FontVertexArray_Push( &font->vertices, v3 );
			FontVertexArray_Push( &font->vertices, v2 );
			FontVertexArray_Push( &font->vertices, v1 );
			FontVertexArray_Push( &font->vertices, v4 );
			FontVertexArray_Push( &font->vertices, v3 );
		}

		i += 1;
	}
}

void FlushText( Font* font, Camera* camera )
{
	float projectionMatrix[16];
	MakeOrthographicMatrix( projectionMatrix, 0.0f, camera->width, camera->height, 0.0f, -1.0f, 1.0f );

	glUseProgram( font->programId );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	int slot = 0;
	glActiveTexture( GL_TEXTURE0 + slot );
	glBindTexture( GL_TEXTURE_2D, font->textureId );

	glBindVertexArray( font->vaoId );
	glBindBuffer( GL_ARRAY_BUFFER, font->vboId );

	int textureUniform = glGetUniformLocation( font->programId, "FontAtlas" );
	glUniform1i( textureUniform, slot );

	int matrixUniform = glGetUniformLocation( font->programId, "ProjectionMatrix" );
	glUniformMatrix4fv( matrixUniform, 1, GL_FALSE, projectionMatrix );

	int totalVertexCount = font->vertices.count;
	int drawCallCount = ( totalVertexCount / FONT_BATCH_SIZE ) + 1;

	for ( int i = 0; i < drawCallCount; i++ )
	{
		const FontVertex* data = font->vertices.data + i * FONT_BATCH_SIZE;

		int vertexCount;
		if ( i == drawCallCount - 1 )
		{
			vertexCount = totalVertexCount % FONT_BATCH_SIZE;
		}
		else
		{
			vertexCount = FONT_BATCH_SIZE;
		}

		glBufferSubData( GL_ARRAY_BUFFER, 0, vertexCount * sizeof( FontVertex ), data );
		glDrawArrays( GL_TRIANGLES, 0, vertexCount );
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glBindTexture( GL_TEXTURE_2D, 0 );

	glDisable( GL_BLEND );

	CheckOpenGL();

	font->vertices.count = 0;
}

typedef struct
{
	GLuint vaoId;
	GLuint vboId;
	GLuint programId;
	GLint timeUniform;
	GLint resolutionUniform;
	GLint baseColorUniform;
} Background;

Background CreateBackground()
{
	Background background = { 0 };

	background.programId = CreateProgramFromFiles( "samples/data/background.vs", "samples/data/background.fs" );
	background.timeUniform = glGetUniformLocation( background.programId, "time" );
	background.resolutionUniform = glGetUniformLocation( background.programId, "resolution" );
	background.baseColorUniform = glGetUniformLocation( background.programId, "baseColor" );
	int vertexAttribute = 0;

	// Generate
	glGenVertexArrays( 1, &background.vaoId );
	glGenBuffers( 1, &background.vboId );

	glBindVertexArray( background.vaoId );
	glEnableVertexAttribArray( vertexAttribute );

	// Single quad
	b2Vec2 vertices[] = { { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, 1.0f }, { 1.0f, -1.0f } };
	glBindBuffer( GL_ARRAY_BUFFER, background.vboId );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return background;
}

void DestroyBackground( Background* background )
{
	if ( background->vaoId )
	{
		glDeleteVertexArrays( 1, &background->vaoId );
		glDeleteBuffers( 1, &background->vboId );
		background->vaoId = 0;
		background->vboId = 0;
	}

	if ( background->programId )
	{
		glDeleteProgram( background->programId );
		background->programId = 0;
	}
}

void RenderBackground( Background* background, Camera* camera )
{
	glUseProgram( background->programId );

	float time = (float)glfwGetTime();
	time = fmodf( time, 100.0f );

	glUniform1f( background->timeUniform, time );
	glUniform2f( background->resolutionUniform, (float)camera->width, (float)camera->height );

	// struct RGBA8 c8 = MakeRGBA8( b2_colorGray2, 1.0f );
	// glUniform3f(baseColorUniform, c8.r/255.0f, c8.g/255.0f, c8.b/255.0f);
	glUniform3f( background->baseColorUniform, 0.2f, 0.2f, 0.2f );

	glBindVertexArray( background->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, background->vboId );
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

ARRAY_DECLARE( PointData );
ARRAY_INLINE( PointData );
ARRAY_SOURCE( PointData );

typedef struct
{
	PointDataArray points;
	GLuint vaoId;
	GLuint vboId;
	GLuint programId;
	GLint projectionUniform;
} PointRender;

PointRender CreatePointDrawData()
{
	PointRender render = { 0 };
	render.points = PointDataArray_Create( POINT_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/point.vs", "samples/data/point.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	int vertexAttribute = 0;
	int sizeAttribute = 1;
	int colorAttribute = 2;

	// Generate
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 1, &render.vboId );

	glBindVertexArray( render.vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( sizeAttribute );
	glEnableVertexAttribArray( colorAttribute );

	// Vertex buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboId );
	glBufferData( GL_ARRAY_BUFFER, POINT_BATCH_SIZE * sizeof( PointData ), NULL, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, position ) );
	glVertexAttribPointer( sizeAttribute, 1, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, size ) );
	// save bandwidth by expanding color to floats in the shader
	glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( PointData ),
						   (void*)offsetof( PointData, rgba ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return render;
}

void DestroyPointDrawData( PointRender* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 1, &render->vboId );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	PointDataArray_Destroy( &render->points );

	*render = (PointRender){ 0 };
}

void AddPoint( PointRender* render, b2Vec2 v, float size, b2HexColor c )
{
	RGBA8 rgba = MakeRGBA8( c, 1.0f );
	PointDataArray_Push( &render->points, (PointData){ v, size, rgba } );
}

void FlushPoints( PointRender* render, Camera* camera )
{
	int count = render->points.count;
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( render->programId );

	float proj[16] = { 0.0f };
	BuildProjectionMatrix( camera, proj, 0.0f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );
	glBindVertexArray( render->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, render->vboId );
	glEnable( GL_PROGRAM_POINT_SIZE );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, POINT_BATCH_SIZE );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( PointData ), render->points.data + base );
		glDrawArrays( GL_POINTS, 0, batchCount );

		CheckOpenGL();

		count -= POINT_BATCH_SIZE;
		base += POINT_BATCH_SIZE;
	}

	glDisable( GL_PROGRAM_POINT_SIZE );
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	render->points.count = 0;
}

#define LINE_BATCH_SIZE ( 2 * 2048 )

typedef struct
{
	b2Vec2 position;
	RGBA8 rgba;
} VertexData;

ARRAY_DECLARE( VertexData );
ARRAY_INLINE( VertexData );
ARRAY_SOURCE( VertexData );

typedef struct
{
	VertexDataArray points;
	GLuint vaoId;
	GLuint vboId;
	GLuint programId;
	GLint projectionUniform;
} LineRender;

LineRender CreateLineRender()
{
	LineRender render = { 0 };
	render.points = VertexDataArray_Create( LINE_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/line.vs", "samples/data/line.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	int vertexAttribute = 0;
	int colorAttribute = 1;

	// Generate
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 1, &render.vboId );

	glBindVertexArray( render.vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( colorAttribute );

	// Vertex buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboId );
	glBufferData( GL_ARRAY_BUFFER, LINE_BATCH_SIZE * sizeof( VertexData ), NULL, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
						   (void*)offsetof( VertexData, position ) );
	// save bandwidth by expanding color to floats in the shader
	glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( VertexData ),
						   (void*)offsetof( VertexData, rgba ) );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return render;
}

void DestroyLineRender( LineRender* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 1, &render->vboId );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	VertexDataArray_Destroy( &render->points );

	*render = (LineRender){ 0 };
}

void AddLine( LineRender* render, b2Vec2 p1, b2Vec2 p2, b2HexColor c )
{
	RGBA8 rgba = MakeRGBA8( c, 1.0f );
	VertexDataArray_Push( &render->points, (VertexData){ p1, rgba } );
	VertexDataArray_Push( &render->points, (VertexData){ p2, rgba } );
}

void FlushLines( LineRender* render, Camera* camera )
{
	int count = render->points.count;
	if ( count == 0 )
	{
		return;
	}

	assert( count % 2 == 0 );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	glUseProgram( render->programId );

	float proj[16] = { 0 };
	BuildProjectionMatrix( camera, proj, 0.1f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );

	glBindVertexArray( render->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, render->vboId );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, LINE_BATCH_SIZE );
		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( VertexData ), render->points.data + base );

		glDrawArrays( GL_LINES, 0, batchCount );

		CheckOpenGL();

		count -= LINE_BATCH_SIZE;
		base += LINE_BATCH_SIZE;
	}

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	glDisable( GL_BLEND );

	render->points.count = 0;
}

#define CIRCLE_BATCH_SIZE 2048

typedef struct
{
	b2Vec2 position;
	float radius;
	RGBA8 rgba;
} CircleData;

ARRAY_DECLARE( CircleData );
ARRAY_INLINE( CircleData );
ARRAY_SOURCE( CircleData );

typedef struct
{
	CircleDataArray circles;
	GLuint vaoId;
	GLuint vboIds[2];
	GLuint programId;
	GLint projectionUniform;
	GLint pixelScaleUniform;
} CircleRender;

CircleRender CreateCircles()
{
	CircleRender render = { 0 };
	render.circles = CircleDataArray_Create( CIRCLE_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/circle.vs", "samples/data/circle.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	render.pixelScaleUniform = glGetUniformLocation( render.programId, "pixelScale" );
	int vertexAttribute = 0;
	int positionInstance = 1;
	int radiusInstance = 2;
	int colorInstance = 3;

	// Generate
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 2, render.vboIds );

	glBindVertexArray( render.vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( positionInstance );
	glEnableVertexAttribArray( radiusInstance );
	glEnableVertexAttribArray( colorInstance );

	// Vertex buffer for single quad
	float a = 1.1f;
	b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[0] );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	// Circle buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[1] );
	glBufferData( GL_ARRAY_BUFFER, CIRCLE_BATCH_SIZE * sizeof( CircleData ), NULL, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( positionInstance, 2, GL_FLOAT, GL_FALSE, sizeof( CircleData ),
						   (void*)offsetof( CircleData, position ) );
	glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( CircleData ), (void*)offsetof( CircleData, radius ) );
	glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( CircleData ),
						   (void*)offsetof( CircleData, rgba ) );

	glVertexAttribDivisor( positionInstance, 1 );
	glVertexAttribDivisor( radiusInstance, 1 );
	glVertexAttribDivisor( colorInstance, 1 );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return render;
}

void DestroyCircles( CircleRender* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 2, render->vboIds );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	CircleDataArray_Destroy( &render->circles );

	*render = (CircleRender){ 0 };
}

void AddCircle( CircleRender* render, b2Vec2 center, float radius, b2HexColor color )
{
	RGBA8 rgba = MakeRGBA8( color, 1.0f );
	CircleDataArray_Push( &render->circles, (CircleData){ center, radius, rgba } );
}

void FlushCircles( CircleRender* render, Camera* camera )
{
	int count = render->circles.count;
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( render->programId );

	float proj[16] = { 0.0f };
	BuildProjectionMatrix( camera, proj, 0.2f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );
	glUniform1f( render->pixelScaleUniform, camera->height / camera->zoom );

	glBindVertexArray( render->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, render->vboIds[1] );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, CIRCLE_BATCH_SIZE );

		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( CircleData ), render->circles.data + base );
		glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

		CheckOpenGL();

		count -= CIRCLE_BATCH_SIZE;
		base += CIRCLE_BATCH_SIZE;
	}

	glDisable( GL_BLEND );

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	render->circles.count = 0;
}

typedef struct
{
	b2Transform transform;
	float radius;
	RGBA8 rgba;
} SolidCircle;

ARRAY_DECLARE( SolidCircle );
ARRAY_INLINE( SolidCircle );
ARRAY_SOURCE( SolidCircle );

#define SOLID_CIRCLE_BATCH_SIZE 2048

// Draws SDF circles using quad instancing. Apparently instancing of quads can be slow on older GPUs.
// https://www.reddit.com/r/opengl/comments/q7yikr/how_to_draw_several_quads_through_instancing/
// https://www.g-truc.net/post-0666.html
typedef struct
{
	SolidCircleArray circles;
	GLuint vaoId;
	GLuint vboIds[2];
	GLuint programId;
	GLint projectionUniform;
	GLint pixelScaleUniform;
} SolidCircles;

SolidCircles CreateSolidCircles()
{
	SolidCircles render = { 0 };
	render.circles = SolidCircleArray_Create( SOLID_CIRCLE_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/solid_circle.vs", "samples/data/solid_circle.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	render.pixelScaleUniform = glGetUniformLocation( render.programId, "pixelScale" );

	// Generate
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 2, render.vboIds );

	glBindVertexArray( render.vaoId );

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
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[0] );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	// Circle buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[1] );
	glBufferData( GL_ARRAY_BUFFER, SOLID_CIRCLE_BATCH_SIZE * sizeof( SolidCircle ), NULL, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof( SolidCircle ),
						   (void*)offsetof( SolidCircle, transform ) );
	glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( SolidCircle ), (void*)offsetof( SolidCircle, radius ) );
	glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( SolidCircle ),
						   (void*)offsetof( SolidCircle, rgba ) );

	glVertexAttribDivisor( transformInstance, 1 );
	glVertexAttribDivisor( radiusInstance, 1 );
	glVertexAttribDivisor( colorInstance, 1 );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return render;
}

void DestroySolidCircles( SolidCircles* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 2, render->vboIds );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	SolidCircleArray_Destroy( &render->circles );

	*render = (SolidCircles){ 0 };
}

void AddSolidCircle( SolidCircles* render, b2Transform transform, float radius, b2HexColor color )
{
	RGBA8 rgba = MakeRGBA8( color, 1.0f );
	SolidCircleArray_Push( &render->circles, (SolidCircle){ transform, radius, rgba } );
}

void FlushSolidCircles( SolidCircles* render, Camera* camera )
{
	int count = render->circles.count;
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( render->programId );

	float proj[16] = { 0.0f };
	BuildProjectionMatrix( camera, proj, 0.2f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );
	glUniform1f( render->pixelScaleUniform, camera->height / camera->zoom );

	glBindVertexArray( render->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, render->vboIds[1] );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, SOLID_CIRCLE_BATCH_SIZE );

		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( SolidCircle ), render->circles.data + base );
		glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

		CheckOpenGL();

		count -= SOLID_CIRCLE_BATCH_SIZE;
		base += SOLID_CIRCLE_BATCH_SIZE;
	}

	glDisable( GL_BLEND );

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	render->circles.count = 0;
}

typedef struct
{
	b2Transform transform;
	float radius;
	float length;
	RGBA8 rgba;
} Capsule;

ARRAY_DECLARE( Capsule );
ARRAY_INLINE( Capsule );
ARRAY_SOURCE( Capsule );

#define CAPSULE_BATCH_SIZE 2048

// Draw capsules using SDF-based shader
typedef struct
{
	CapsuleArray capsules;
	GLuint vaoId;
	GLuint vboIds[2];
	GLuint programId;
	GLint projectionUniform;
	GLint pixelScaleUniform;
} Capsules;

Capsules CreateCapsules()
{
	Capsules render = { 0 };
	render.capsules = CapsuleArray_Create( CAPSULE_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/solid_capsule.vs", "samples/data/solid_capsule.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	render.pixelScaleUniform = glGetUniformLocation( render.programId, "pixelScale" );

	int vertexAttribute = 0;
	int transformInstance = 1;
	int radiusInstance = 2;
	int lengthInstance = 3;
	int colorInstance = 4;

	// Generate
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 2, render.vboIds );

	glBindVertexArray( render.vaoId );
	glEnableVertexAttribArray( vertexAttribute );
	glEnableVertexAttribArray( transformInstance );
	glEnableVertexAttribArray( radiusInstance );
	glEnableVertexAttribArray( lengthInstance );
	glEnableVertexAttribArray( colorInstance );

	// Vertex buffer for single quad
	float a = 1.1f;
	b2Vec2 vertices[] = { { -a, -a }, { a, -a }, { -a, a }, { a, -a }, { a, a }, { -a, a } };
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[0] );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	// Capsule buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[1] );
	glBufferData( GL_ARRAY_BUFFER, CAPSULE_BATCH_SIZE * sizeof( Capsule ), NULL, GL_DYNAMIC_DRAW );

	glVertexAttribPointer( transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof( Capsule ), (void*)offsetof( Capsule, transform ) );
	glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( Capsule ), (void*)offsetof( Capsule, radius ) );
	glVertexAttribPointer( lengthInstance, 1, GL_FLOAT, GL_FALSE, sizeof( Capsule ), (void*)offsetof( Capsule, length ) );
	glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( Capsule ), (void*)offsetof( Capsule, rgba ) );

	glVertexAttribDivisor( transformInstance, 1 );
	glVertexAttribDivisor( radiusInstance, 1 );
	glVertexAttribDivisor( lengthInstance, 1 );
	glVertexAttribDivisor( colorInstance, 1 );

	CheckOpenGL();

	// Cleanup
	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );

	return render;
}

void DestroyCapsules( Capsules* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 2, render->vboIds );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	CapsuleArray_Destroy( &render->capsules );

	*render = (Capsules){ 0 };
}

void AddCapsule( Capsules* render, b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor c )
{
	b2Vec2 d = b2Sub( p2, p1 );
	float length = b2Length( d );
	if ( length < 0.001f )
	{
		printf( "WARNING: sample app: capsule too short!\n" );
		return;
	}

	b2Vec2 axis = { d.x / length, d.y / length };
	b2Transform transform;
	transform.p = b2Lerp( p1, p2, 0.5f );
	transform.q.c = axis.x;
	transform.q.s = axis.y;

	RGBA8 rgba = MakeRGBA8( c, 1.0f );

	CapsuleArray_Push( &render->capsules, (Capsule){ transform, radius, length, rgba } );
}

void FlushCapsules( Capsules* render, Camera* camera )
{
	int count = render->capsules.count;
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( render->programId );

	float proj[16] = { 0.0f };
	BuildProjectionMatrix( camera, proj, 0.2f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );
	glUniform1f( render->pixelScaleUniform, camera->height / camera->zoom );

	glBindVertexArray( render->vaoId );

	glBindBuffer( GL_ARRAY_BUFFER, render->vboIds[1] );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, CAPSULE_BATCH_SIZE );

		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( Capsule ), render->capsules.data + base );
		glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

		CheckOpenGL();

		count -= CAPSULE_BATCH_SIZE;
		base += CAPSULE_BATCH_SIZE;
	}

	glDisable( GL_BLEND );

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	render->capsules.count = 0;
}

typedef struct
{
	b2Transform transform;
	b2Vec2 p1, p2, p3, p4, p5, p6, p7, p8;
	int count;
	float radius;

	// keep color small
	RGBA8 color;
} Polygon;

ARRAY_DECLARE( Polygon );
ARRAY_INLINE( Polygon );
ARRAY_SOURCE( Polygon );

#define POLYGON_BATCH_SIZE 2048

// Rounded and non-rounded convex polygons using an SDF-based shader.
typedef struct
{
	PolygonArray polygons;
	GLuint vaoId;
	GLuint vboIds[2];
	GLuint programId;
	GLint projectionUniform;
	GLint pixelScaleUniform;
} Polygons;

Polygons CreatePolygons()
{
	Polygons render = { 0 };
	render.polygons = PolygonArray_Create( POLYGON_BATCH_SIZE );
	render.programId = CreateProgramFromFiles( "samples/data/solid_polygon.vs", "samples/data/solid_polygon.fs" );
	render.projectionUniform = glGetUniformLocation( render.programId, "projectionMatrix" );
	render.pixelScaleUniform = glGetUniformLocation( render.programId, "pixelScale" );

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
	glGenVertexArrays( 1, &render.vaoId );
	glGenBuffers( 2, render.vboIds );

	glBindVertexArray( render.vaoId );
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
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[0] );
	glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
	glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

	// Polygon buffer
	glBindBuffer( GL_ARRAY_BUFFER, render.vboIds[1] );
	glBufferData( GL_ARRAY_BUFFER, POLYGON_BATCH_SIZE * sizeof( Polygon ), NULL, GL_DYNAMIC_DRAW );
	glVertexAttribPointer( instanceTransform, 4, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, transform ) );
	glVertexAttribPointer( instancePoint12, 4, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, p1 ) );
	glVertexAttribPointer( instancePoint34, 4, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, p3 ) );
	glVertexAttribPointer( instancePoint56, 4, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, p5 ) );
	glVertexAttribPointer( instancePoint78, 4, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, p7 ) );
	glVertexAttribIPointer( instancePointCount, 1, GL_INT, sizeof( Polygon ), (void*)offsetof( Polygon, count ) );
	glVertexAttribPointer( instanceRadius, 1, GL_FLOAT, GL_FALSE, sizeof( Polygon ), (void*)offsetof( Polygon, radius ) );
	// color will get automatically expanded to floats in the shader
	glVertexAttribPointer( instanceColor, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( Polygon ), (void*)offsetof( Polygon, color ) );

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

	return render;
}

void DestroyPolygons( Polygons* render )
{
	if ( render->vaoId )
	{
		glDeleteVertexArrays( 1, &render->vaoId );
		glDeleteBuffers( 2, render->vboIds );
	}

	if ( render->programId )
	{
		glDeleteProgram( render->programId );
	}

	PolygonArray_Destroy( &render->polygons );

	*render = (Polygons){ 0 };
}

void AddPolygon( Polygons* render, b2Transform transform, const b2Vec2* points, int count, float radius, b2HexColor color )
{
	Polygon data = {};
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

	PolygonArray_Push( &render->polygons, data );
}

void FlushPolygons( Polygons* render, Camera* camera )
{
	int count = render->polygons.count;
	if ( count == 0 )
	{
		return;
	}

	glUseProgram( render->programId );

	float proj[16] = { 0.0f };
	BuildProjectionMatrix( camera, proj, 0.2f );

	glUniformMatrix4fv( render->projectionUniform, 1, GL_FALSE, proj );
	glUniform1f( render->pixelScaleUniform, camera->height / camera->zoom );

	glBindVertexArray( render->vaoId );
	glBindBuffer( GL_ARRAY_BUFFER, render->vboIds[1] );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	int base = 0;
	while ( count > 0 )
	{
		int batchCount = b2MinInt( count, POLYGON_BATCH_SIZE );

		glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( Polygon ), render->polygons.data + base );
		glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );
		CheckOpenGL();

		count -= POLYGON_BATCH_SIZE;
		base += POLYGON_BATCH_SIZE;
	}

	glDisable( GL_BLEND );

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );

	render->polygons.count = 0;
}

typedef struct Draw
{
	Background background;
	PointRender points;
	LineRender lines;
	CircleRender hollowCircles;
	SolidCircles circles;
	Capsules capsules;
	Polygons polygons;
	Font font;
} Draw;

Draw* CreateDraw( void )
{
	Draw* draw = malloc( sizeof( Draw ) );
	*draw = (Draw){ 0 };
	draw->background = CreateBackground();
	draw->points = CreatePointDrawData();
	draw->lines = CreateLineRender();
	draw->hollowCircles = CreateCircles();
	draw->circles = CreateSolidCircles();
	draw->capsules = CreateCapsules();
	draw->polygons = CreatePolygons();
	draw->font = CreateFont( "samples/data/droid_sans.ttf", 18.0f );
	return draw;
}

void DestroyDraw( Draw* draw )
{
	DestroyBackground( &draw->background );
	DestroyPointDrawData( &draw->points );
	DestroyLineRender( &draw->lines );
	DestroyCircles( &draw->hollowCircles );
	DestroySolidCircles( &draw->circles );
	DestroyCapsules( &draw->capsules );
	DestroyPolygons( &draw->polygons );
	DestroyFont( &draw->font );
	free( draw );
}

void DrawPoint( Draw* draw, b2Vec2 p, float size, b2HexColor color )
{
	AddPoint( &draw->points, p, size, color );
}

void DrawLine( Draw* draw, b2Vec2 p1, b2Vec2 p2, b2HexColor color )
{
	AddLine( &draw->lines, p1, p2, color );
}

void DrawCircle( Draw* draw, b2Vec2 center, float radius, b2HexColor color )
{
	AddCircle( &draw->hollowCircles, center, radius, color );
}

void DrawSolidCircle( Draw* draw, b2Transform transform, float radius, b2HexColor color )
{
	AddSolidCircle( &draw->circles, transform, radius, color );
}

void DrawSolidCapsule( Draw* draw, b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color )
{
	AddCapsule( &draw->capsules, p1, p2, radius, color );
}

void DrawPolygon( Draw* draw, const b2Vec2* vertices, int vertexCount, b2HexColor color )
{
	b2Vec2 p1 = vertices[vertexCount - 1];
	for ( int i = 0; i < vertexCount; ++i )
	{
		b2Vec2 p2 = vertices[i];
		AddLine( &draw->lines, p1, p2, color );
		p1 = p2;
	}
}

void DrawSolidPolygon( Draw* draw, b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius,
					   b2HexColor color )
{
	AddPolygon( &draw->polygons, transform, vertices, vertexCount, radius, color );
}

void DrawTransform( Draw* draw, b2Transform transform, float scale )
{
	b2Vec2 p1 = transform.p;

	b2Vec2 p2 = b2MulAdd( p1, scale, b2Rot_GetXAxis( transform.q ) );
	AddLine( &draw->lines, p1, p2, b2_colorRed );

	p2 = b2MulAdd( p1, scale, b2Rot_GetYAxis( transform.q ) );
	AddLine( &draw->lines, p1, p2, b2_colorGreen );
}

void DrawBounds( Draw* draw, b2AABB aabb, b2HexColor color )
{
	b2Vec2 p1 = aabb.lowerBound;
	b2Vec2 p2 = { aabb.upperBound.x, aabb.lowerBound.y };
	b2Vec2 p3 = aabb.upperBound;
	b2Vec2 p4 = { aabb.lowerBound.x, aabb.upperBound.y };

	AddLine( &draw->lines, p1, p2, color );
	AddLine( &draw->lines, p2, p3, color );
	AddLine( &draw->lines, p3, p4, color );
	AddLine( &draw->lines, p4, p1, color );
}

void DrawScreenString( Draw* draw, float x, float y, b2HexColor color, const char* string, ... )
{
	char buffer[256];
	va_list arg;
	va_start( arg, string );
	vsnprintf( buffer, 256, string, arg );
	va_end( arg );

	buffer[255] = 0;
	AddText( &draw->font, x, y, color, buffer );
}

void DrawWorldString( Draw* draw, Camera* camera, b2Vec2 p, b2HexColor color, const char* string, ... )
{
	b2Vec2 ps = ConvertWorldToScreen( camera, p );

	char buffer[256];
	va_list arg;
	va_start( arg, string );
	vsnprintf( buffer, 256, string, arg );
	va_end( arg );

	buffer[255] = 0;
	AddText( &draw->font, ps.x, ps.y, color, buffer );
}

void FlushDraw( Draw* draw, Camera* camera )
{
	// order matters
	FlushSolidCircles( &draw->circles, camera );
	FlushCapsules( &draw->capsules, camera );
	FlushPolygons( &draw->polygons, camera );
	FlushCircles( &draw->hollowCircles, camera );
	FlushLines( &draw->lines, camera );
	FlushPoints( &draw->points, camera );
	FlushText( &draw->font, camera );
	CheckOpenGL();
}

void DrawBackground( Draw* draw, Camera* camera )
{
	RenderBackground( &draw->background, camera );
}
