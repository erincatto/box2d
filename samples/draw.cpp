// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"

#include "shader.h"

#include "box2d/math_functions.h"

#include <stdarg.h>
#include <stdio.h>
#include <vector>

#if defined( _WIN32 )
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

#include <imgui.h>

#define BUFFER_OFFSET( x ) ( (const void*)( x ) )

#define SHADER_TEXT( x ) "#version 330\n" #x

struct RGBA8
{
	uint8_t r, g, b, a;
};

static inline RGBA8 MakeRGBA8( b2HexColor c, float alpha )
{
	return { uint8_t( ( c >> 16 ) & 0xFF ), uint8_t( ( c >> 8 ) & 0xFF ), uint8_t( c & 0xFF ), uint8_t( 0xFF * alpha ) };
}

Draw g_draw;
Camera g_camera;

Camera::Camera()
{
	m_width = 1280;
	m_height = 800;
	ResetView();
}

void Camera::ResetView()
{
	m_center = { 0.0f, 20.0f };
	m_zoom = 1.0f;
}

b2Vec2 Camera::ConvertScreenToWorld( b2Vec2 ps )
{
	float w = float( m_width );
	float h = float( m_height );
	float u = ps.x / w;
	float v = ( h - ps.y ) / h;

	float ratio = w / h;
	b2Vec2 extents = { m_zoom * ratio, m_zoom };

	b2Vec2 lower = b2Sub( m_center, extents );
	b2Vec2 upper = b2Add( m_center, extents );

	b2Vec2 pw = { ( 1.0f - u ) * lower.x + u * upper.x, ( 1.0f - v ) * lower.y + v * upper.y };
	return pw;
}

b2Vec2 Camera::ConvertWorldToScreen( b2Vec2 pw )
{
	float w = float( m_width );
	float h = float( m_height );
	float ratio = w / h;

	b2Vec2 extents = { m_zoom * ratio, m_zoom };

	b2Vec2 lower = b2Sub( m_center, extents );
	b2Vec2 upper = b2Add( m_center, extents );

	float u = ( pw.x - lower.x ) / ( upper.x - lower.x );
	float v = ( pw.y - lower.y ) / ( upper.y - lower.y );

	b2Vec2 ps = { u * w, ( 1.0f - v ) * h };
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
// This also includes the view transform
void Camera::BuildProjectionMatrix( float* m, float zBias )
{
	float ratio = float( m_width ) / float( m_height );
	b2Vec2 extents = { m_zoom * ratio, m_zoom };

	b2Vec2 lower = b2Sub( m_center, extents );
	b2Vec2 upper = b2Add( m_center, extents );
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

	m[12] = -2.0f * m_center.x / w;
	m[13] = -2.0f * m_center.y / h;
	m[14] = zBias;
	m[15] = 1.0f;
}

b2AABB Camera::GetViewBounds()
{
	b2AABB bounds;
	bounds.lowerBound = ConvertScreenToWorld( { 0.0f, (float)m_height } );
	bounds.upperBound = ConvertScreenToWorld( { (float)m_width, 0.0f } );
	return bounds;
}

struct GLBackground
{
	void Create()
	{
		m_programId = CreateProgramFromFiles( "samples/data/background.vs", "samples/data/background.fs" );
		m_timeUniform = glGetUniformLocation( m_programId, "time" );
		m_resolutionUniform = glGetUniformLocation( m_programId, "resolution" );
		m_baseColorUniform = glGetUniformLocation( m_programId, "baseColor" );
		int vertexAttribute = 0;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 1, &m_vboId );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );

		// Single quad
		b2Vec2 vertices[] = { { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, 1.0f }, { 1.0f, -1.0f } };
		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glBufferData( GL_ARRAY_BUFFER, sizeof( vertices ), vertices, GL_STATIC_DRAW );
		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET( 0 ) );

		CheckErrorGL();

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

	void Draw()
	{
		glUseProgram( m_programId );

		float time = (float)glfwGetTime();
		time = fmodf(time, 100.0f);
		
		glUniform1f( m_timeUniform, time );
		glUniform2f( m_resolutionUniform, (float)g_camera.m_width, (float)g_camera.m_height );

		// struct RGBA8 c8 = MakeRGBA8( b2_colorGray2, 1.0f );
		// glUniform3f(m_baseColorUniform, c8.r/255.0f, c8.g/255.0f, c8.b/255.0f);
		glUniform3f( m_baseColorUniform, 0.2f, 0.2f, 0.2f );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glDrawArrays( GL_TRIANGLE_STRIP, 0, 4 );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );
	}

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_timeUniform;
	GLint m_resolutionUniform;
	GLint m_baseColorUniform;
};

struct PointData
{
	b2Vec2 position;
	float size;
	RGBA8 rgba;
};

struct GLPoints
{
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
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( PointData ), NULL, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( PointData ),
							   (void*)offsetof( PointData, position ) );
		glVertexAttribPointer( sizeAttribute, 1, GL_FLOAT, GL_FALSE, sizeof( PointData ), (void*)offsetof( PointData, size ) );
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( PointData ),
							   (void*)offsetof( PointData, rgba ) );

		CheckErrorGL();

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

	void Flush()
	{
		int count = (int)m_points.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.0f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glEnable( GL_PROGRAM_POINT_SIZE );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );
			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( PointData ), &m_points[base] );
			glDrawArrays( GL_POINTS, 0, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_PROGRAM_POINT_SIZE );
		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_points.clear();
	}

	enum
	{
		e_maxCount = 2048
	};

	std::vector<PointData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

struct VertexData
{
	b2Vec2 position;
	RGBA8 rgba;
};

struct GLLines
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in vec4 v_color;\n"
						 "out vec4 f_color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	f_color = v_color;\n"
						 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
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
		int colorAttribute = 1;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 1, &m_vboId );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( colorAttribute );

		// Vertex buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( VertexData ), NULL, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, position ) );
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, rgba ) );

		CheckErrorGL();

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

	void Flush()
	{
		int count = (int)m_points.size();
		if ( count == 0 )
		{
			return;
		}

		assert( count % 2 == 0 );

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.1f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );
			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( VertexData ), &m_points[base] );

			glDrawArrays( GL_LINES, 0, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_points.clear();
	}

	// need lots of space for lines so they draw last
	// could also consider disabling depth buffer
	enum
	{
		// must be multiple of 2
		e_maxCount = 2 * 2048
	};

	std::vector<VertexData> m_points;

	GLuint m_vaoId;
	GLuint m_vboId;
	GLuint m_programId;
	GLint m_projectionUniform;
};

// todo this is not used anymore and has untested changes
struct GLTriangles
{
	void Create()
	{
		const char* vs = "#version 330\n"
						 "uniform mat4 projectionMatrix;\n"
						 "layout(location = 0) in vec2 v_position;\n"
						 "layout(location = 1) in vec4 v_color;\n"
						 "out vec4 f_color;\n"
						 "void main(void)\n"
						 "{\n"
						 "	f_color = v_color;\n"
						 "	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
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
		int colorAttribute = 1;

		// Generate
		glGenVertexArrays( 1, &m_vaoId );
		glGenBuffers( 1, &m_vboId );

		glBindVertexArray( m_vaoId );
		glEnableVertexAttribArray( vertexAttribute );
		glEnableVertexAttribArray( colorAttribute );

		// Vertex buffer
		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( VertexData ), NULL, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( vertexAttribute, 2, GL_FLOAT, GL_FALSE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, position ) );
		// color will get automatically expanded to floats in the shader
		glVertexAttribPointer( colorAttribute, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( VertexData ),
							   (void*)offsetof( VertexData, rgba ) );

		CheckErrorGL();

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

	void AddTriangle( b2Vec2 p1, b2Vec2 p2, b2Vec2 p3, b2HexColor c )
	{
		RGBA8 rgba = MakeRGBA8( c, 1.0f );
		m_points.push_back( { p1, rgba } );
		m_points.push_back( { p2, rgba } );
		m_points.push_back( { p3, rgba } );
	}

	void Flush()
	{
		int count = (int)m_points.size();
		if ( count == 0 )
		{
			return;
		}

		assert( count % 3 == 0 );

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboId );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( VertexData ), &m_points[base] );
			glDrawArrays( GL_TRIANGLES, 0, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_points.clear();
	}

	enum
	{
		// must be multiple of 3
		e_maxCount = 3 * 512
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
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( CircleData ), NULL, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( positionInstance, 2, GL_FLOAT, GL_FALSE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, position ) );
		glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, radius ) );
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( CircleData ),
							   (void*)offsetof( CircleData, rgba ) );

		glVertexAttribDivisor( positionInstance, 1 );
		glVertexAttribDivisor( radiusInstance, 1 );
		glVertexAttribDivisor( colorInstance, 1 );

		CheckErrorGL();

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

	void Flush()
	{
		int count = (int)m_circles.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, g_camera.m_height / g_camera.m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( CircleData ), &m_circles[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_circles.clear();
	}

	enum
	{
		e_maxCount = 2048
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
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( SolidCircleData ), NULL, GL_DYNAMIC_DRAW );

		glVertexAttribPointer( transformInstance, 4, GL_FLOAT, GL_FALSE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, transform ) );
		glVertexAttribPointer( radiusInstance, 1, GL_FLOAT, GL_FALSE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, radius ) );
		glVertexAttribPointer( colorInstance, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof( SolidCircleData ),
							   (void*)offsetof( SolidCircleData, rgba ) );

		glVertexAttribDivisor( transformInstance, 1 );
		glVertexAttribDivisor( radiusInstance, 1 );
		glVertexAttribDivisor( colorInstance, 1 );

		CheckErrorGL();

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
		m_circles.push_back( { { transform.p.x, transform.p.y, transform.q.c, transform.q.s }, radius, rgba } );
	}

	void Flush()
	{
		int count = (int)m_circles.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, g_camera.m_height / g_camera.m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( SolidCircleData ), &m_circles[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_circles.clear();
	}

	enum
	{
		e_maxCount = 2048
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
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( CapsuleData ), NULL, GL_DYNAMIC_DRAW );

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

		CheckErrorGL();

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

		m_capsules.push_back( { { transform.p.x, transform.p.y, transform.q.c, transform.q.s }, radius, length, rgba } );
	}

	void Flush()
	{
		int count = (int)m_capsules.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, g_camera.m_height / g_camera.m_zoom );

		glBindVertexArray( m_vaoId );

		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( CapsuleData ), &m_capsules[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );

			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_capsules.clear();
	}

	enum
	{
		e_maxCount = 2048
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
		glBufferData( GL_ARRAY_BUFFER, e_maxCount * sizeof( PolygonData ), NULL, GL_DYNAMIC_DRAW );
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

		CheckErrorGL();

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

	void Flush()
	{
		int count = (int)m_polygons.size();
		if ( count == 0 )
		{
			return;
		}

		glUseProgram( m_programId );

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix( proj, 0.2f );

		glUniformMatrix4fv( m_projectionUniform, 1, GL_FALSE, proj );
		glUniform1f( m_pixelScaleUniform, g_camera.m_height / g_camera.m_zoom );

		glBindVertexArray( m_vaoId );
		glBindBuffer( GL_ARRAY_BUFFER, m_vboIds[1] );

		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

		int base = 0;
		while ( count > 0 )
		{
			int batchCount = b2MinInt( count, e_maxCount );

			glBufferSubData( GL_ARRAY_BUFFER, 0, batchCount * sizeof( PolygonData ), &m_polygons[base] );
			glDrawArraysInstanced( GL_TRIANGLES, 0, 6, batchCount );
			CheckErrorGL();

			count -= e_maxCount;
			base += e_maxCount;
		}

		glDisable( GL_BLEND );

		glBindBuffer( GL_ARRAY_BUFFER, 0 );
		glBindVertexArray( 0 );
		glUseProgram( 0 );

		m_polygons.clear();
	}

	enum
	{
		e_maxCount = 512
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
	static_cast<Draw*>( context )->DrawSegment( p1, p2, color );
}

void DrawTransformFcn( b2Transform transform, void* context )
{
	static_cast<Draw*>( context )->DrawTransform( transform );
}

void DrawPointFcn( b2Vec2 p, float size, b2HexColor color, void* context )
{
	static_cast<Draw*>( context )->DrawPoint( p, size, color );
}

void DrawStringFcn( b2Vec2 p, const char* s, void* context )
{
	static_cast<Draw*>( context )->DrawString( p, s );
}

Draw::Draw()
{
	m_showUI = true;
	m_points = nullptr;
	m_lines = nullptr;
	m_triangles = nullptr;
	m_circles = nullptr;
	m_solidCircles = nullptr;
	m_solidCapsules = nullptr;
	m_solidPolygons = nullptr;
	m_debugDraw = {};
	m_smallFont = nullptr;
	m_mediumFont = nullptr;
	m_largeFont = nullptr;
	m_regularFont = nullptr;
	m_background = nullptr;
}

Draw::~Draw()
{
	assert( m_points == nullptr );
	assert( m_lines == nullptr );
	assert( m_triangles == nullptr );
	assert( m_circles == nullptr );
	assert( m_solidCircles == nullptr );
	assert( m_solidCapsules == nullptr );
	assert( m_solidPolygons == nullptr );
	assert( m_background == nullptr );
}

void Draw::Create()
{
	m_background = new GLBackground;
	m_background->Create();
	m_points = new GLPoints;
	m_points->Create();
	m_lines = new GLLines;
	m_lines->Create();
	m_triangles = new GLTriangles;
	m_triangles->Create();
	m_circles = new GLCircles;
	m_circles->Create();
	m_solidCircles = new GLSolidCircles;
	m_solidCircles->Create();
	m_solidCapsules = new GLSolidCapsules;
	m_solidCapsules->Create();
	m_solidPolygons = new GLSolidPolygons;
	m_solidPolygons->Create();

	b2AABB bounds = { { -FLT_MAX, -FLT_MAX }, { FLT_MAX, FLT_MAX } };

	m_debugDraw = { DrawPolygonFcn,
					DrawSolidPolygonFcn,
					DrawCircleFcn,
					DrawSolidCircleFcn,
					DrawSolidCapsuleFcn,
					DrawSegmentFcn,
					DrawTransformFcn,
					DrawPointFcn,
					DrawStringFcn,
					bounds,
					false, // drawUsingBounds
					true,  // shapes
					true,  // joints
					false, // joint extras
					false, // aabbs
					false, // mass
					false, // contacts
					false, // colors
					false, // normals
					false, // impulse
					false, // friction
					this };
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

	m_triangles->Destroy();
	delete m_triangles;
	m_triangles = nullptr;

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

void Draw::DrawSegment( b2Vec2 p1, b2Vec2 p2, b2HexColor color )
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
	ImGui::PushFont( g_draw.m_regularFont );
	ImGui::SetCursorPos( ImVec2( float( x ), float( y ) ) );
	ImGui::TextColoredV( ImColor( 230, 153, 153, 255 ), string, arg );
	ImGui::PopFont();
	ImGui::End();
	va_end( arg );
}

void Draw::DrawString( b2Vec2 p, const char* string, ... )
{
	b2Vec2 ps = g_camera.ConvertWorldToScreen( p );

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

void Draw::DrawAABB( b2AABB aabb, b2HexColor c )
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
	m_solidCircles->Flush();
	m_solidCapsules->Flush();
	m_solidPolygons->Flush();
	m_triangles->Flush();
	m_circles->Flush();
	m_lines->Flush();
	m_points->Flush();
	CheckErrorGL();
}

void Draw::DrawBackground()
{
	m_background->Draw();
}
