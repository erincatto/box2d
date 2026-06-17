// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "draw.h"

#include "container.h"
#include "shader.h"
#include "shaders_embedded.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <assert.h>
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

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include "stb_image_write.h"

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
	camera->center = (b2Pos){ 0.0f, 20.0f };
	camera->zoom = 1.0f;
}

b2Pos ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint )
{
	float w = camera->width;
	float h = camera->height;
	float u = screenPoint.x / w;
	float v = ( h - screenPoint.y ) / h;

	float ratio = w / h;
	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	// Form the offset from the view center in float, then add to the center. Building
	// center +/- extents in float would lose the view-sized extents far from the origin.
	b2Vec2 offset = { extents.x * ( 2.0f * u - 1.0f ), extents.y * ( 2.0f * v - 1.0f ) };
	return b2OffsetPos( camera->center, offset );
}

b2Vec2 ConvertViewToScreen( Camera* camera, b2Vec2 viewPoint )
{
	float w = camera->width;
	float h = camera->height;
	float ratio = w / h;

	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	float u = ( viewPoint.x + extents.x ) / ( 2.0f * extents.x );
	float v = ( viewPoint.y + extents.y ) / ( 2.0f * extents.y );

	b2Vec2 ps = { u * w, ( 1.0f - v ) * h };
	return ps;
}

b2Vec2 ConvertWorldToScreen( Camera* camera, b2Pos worldPoint )
{
	// Distance from the view center, demoted to float, then the float mapping
	return ConvertViewToScreen( camera, b2SubPos( worldPoint, camera->center ) );
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
// This also includes the view transform
static void BuildProjectionMatrix( Camera* camera, float* m, float zBias )
{
	float ratio = camera->width / camera->height;
	b2Vec2 extents = { camera->zoom * ratio, camera->zoom };

	float w = 2.0f * extents.x;
	float h = 2.0f * extents.y;

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

	// Vertices reach the GPU already shifted into camera relative space, the engine draw path and the
	// Draw helpers subtract the view center, so the view center is the origin here. In large world
	// mode this also keeps double coordinates out of the shader.
	m[12] = 0.0f;
	m[13] = 0.0f;
	m[14] = zBias;
	m[15] = 1.0f;
}

b2AABB GetViewBounds( Camera* camera )
{
	if ( camera->height == 0.0f || camera->width == 0.0f )
	{
		b2AABB bounds = {
			.lowerBound = b2Vec2_zero,
			.upperBound = b2Vec2_zero,
		};
		return bounds;
	}

	b2Pos lower = ConvertScreenToWorld( camera, (b2Vec2){ 0.0f, camera->height } );
	b2Pos upper = ConvertScreenToWorld( camera, (b2Vec2){ camera->width, 0.0f } );

	// Engine cull box stays float. Round outward so nothing visible is clipped far from the origin.
	b2AABB bounds;
	bounds.lowerBound = (b2Vec2){ b2RoundDownFloat( lower.x ), b2RoundDownFloat( lower.y ) };
	bounds.upperBound = (b2Vec2){ b2RoundUpFloat( upper.x ), b2RoundUpFloat( upper.y ) };
	return bounds;
}

void FocusOnBounds( Camera* camera, b2AABB bounds )
{
	if ( camera->width == 0 )
	{
		return;
	}

	b2Vec2 extents = b2AABB_Extents( bounds );

	if ( extents.x < B2_LINEAR_SLOP || extents.y < B2_LINEAR_SLOP )
	{
		return;
	}

	float invRatio = camera->height / camera->width;
	camera->zoom = b2MaxFloat( extents.x * invRatio, extents.y );

	// Need to guard against zero because zoom can get stuck there
	camera->zoom = b2MaxFloat( camera->zoom, 0.01f );

	camera->center = b2ToPos( b2AABB_Center( bounds ) );
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

	background.programId = CreateProgramFromStrings( k_background_vs, k_background_fs );
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
	render.programId = CreateProgramFromStrings( k_point_vs, k_point_fs );
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
	render.programId = CreateProgramFromStrings( k_line_vs, k_line_fs );
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
	render.programId = CreateProgramFromStrings( k_circle_vs, k_circle_fs );
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
	render.programId = CreateProgramFromStrings( k_solid_circle_vs, k_solid_circle_fs );
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
	render.programId = CreateProgramFromStrings( k_solid_capsule_vs, k_solid_capsule_fs );
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
	render.polygons = PolygonArray_Create( 10 * POLYGON_BATCH_SIZE );
	render.programId = CreateProgramFromStrings( k_solid_polygon_vs, k_solid_polygon_fs );
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

	// Camera center in large world mode, subtracted by the DrawWorld helpers. Zero in float mode.
	b2Pos origin;
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
	free( draw );
}

void SetDrawOrigin( Draw* draw, b2Pos origin )
{
	draw->origin = origin;
}

void DrawPoint( Draw* draw, b2Pos p, float size, b2HexColor color )
{
	AddPoint( &draw->points, b2SubPos( p, draw->origin ), size, color );
}

void DrawLine( Draw* draw, b2Pos p1, b2Pos p2, b2HexColor color )
{
	AddLine( &draw->lines, b2SubPos( p1, draw->origin ), b2SubPos( p2, draw->origin ), color );
}

void DrawCircle( Draw* draw, b2Pos center, float radius, b2HexColor color )
{
	AddCircle( &draw->hollowCircles, b2SubPos( center, draw->origin ), radius, color );
}

void DrawCapsule( Draw* draw, b2Pos p1, b2Pos p2, float radius, b2HexColor color )
{
	AddCapsule( &draw->capsules, b2SubPos( p1, draw->origin ), b2SubPos( p2, draw->origin ), radius, color );
}

void DrawPolygon( Draw* draw, b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, b2HexColor color )
{
	b2Transform xf = b2ToRelativeTransform( transform, draw->origin );
	b2Vec2 p1 = b2TransformPoint( xf, vertices[vertexCount - 1] );
	for ( int i = 0; i < vertexCount; ++i )
	{
		b2Vec2 p2 = b2TransformPoint( xf, vertices[i] );
		AddLine( &draw->lines, p1, p2, color );
		p1 = p2;
	}
}

void DrawSolidCircle( Draw* draw, b2WorldTransform transform, b2Vec2 center, float radius, b2HexColor color )
{
	// Fold the local center offset into the world transform, then shift into the camera frame
	b2WorldTransform xf = { b2TransformWorldPoint( transform, center ), transform.q };
	b2Transform localTransform = b2ToRelativeTransform( xf, draw->origin );
	AddSolidCircle( &draw->circles, localTransform, radius, color );
}

void DrawSolidPolygon( Draw* draw, b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, float radius,
							b2HexColor color )
{
	AddPolygon( &draw->polygons, b2ToRelativeTransform( transform, draw->origin ), vertices, vertexCount, radius, color );
}

void DrawTransform( Draw* draw, b2WorldTransform transform, float scale )
{
	b2Transform xf = b2ToRelativeTransform( transform, draw->origin );

	b2Vec2 p1 = xf.p;

	b2Vec2 p2 = b2MulAdd( p1, scale, b2Rot_GetXAxis( xf.q ) );
	AddLine( &draw->lines, p1, p2, b2_colorRed );

	p2 = b2MulAdd( p1, scale, b2Rot_GetYAxis( xf.q ) );
	AddLine( &draw->lines, p1, p2, b2_colorGreen );
}

void DrawBounds( Draw* draw, b2AABB aabb, b2HexColor color )
{
	b2Vec2 lower = b2SubPos( b2ToPos( aabb.lowerBound ), draw->origin );
	b2Vec2 upper = b2SubPos( b2ToPos( aabb.upperBound ), draw->origin );

	b2Vec2 p1 = lower;
	b2Vec2 p2 = { upper.x, lower.y };
	b2Vec2 p3 = upper;
	b2Vec2 p4 = { lower.x, upper.y };

	AddLine( &draw->lines, p1, p2, color );
	AddLine( &draw->lines, p2, p3, color );
	AddLine( &draw->lines, p3, p4, color );
	AddLine( &draw->lines, p4, p1, color );
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
	CheckOpenGL();
}

void DrawBackground( Draw* draw, Camera* camera )
{
	RenderBackground( &draw->background, camera );
}
