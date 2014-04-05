//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

// Source altered and distributed from https://github.com/AdrienHerubel/imgui

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#include <GL/gl.h>
#endif

#include "RenderGL3.h"

// Some math headers don't have PI defined.
static const float PI = 3.14159265f;

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// Pull render interface.
enum GfxCmdType
{
	GFXCMD_RECT,
	GFXCMD_TRIANGLE,
	GFXCMD_LINE,
	GFXCMD_TEXT,
	GFXCMD_SCISSOR,
};

struct GfxRect
{
	short x, y, w, h, r;
};

struct GfxText
{
	float x, y;
	TextAlign align;
	const char* text;
};

struct GfxLine
{
	short x0, y0, x1, y1, r;
};

struct GfxCmd
{
	char type;
	char flags;
	char pad[2];
	unsigned int col;
	union
	{
		GfxLine line;
		GfxRect rect;
		GfxText text;
	};
};

static const unsigned TEMP_COORD_COUNT = 100;
static float g_tempCoords[TEMP_COORD_COUNT * 2];
static float g_tempNormals[TEMP_COORD_COUNT * 2];
static float g_tempVertices[TEMP_COORD_COUNT * 12 + (TEMP_COORD_COUNT - 2) * 6];
static float g_tempTextureCoords[TEMP_COORD_COUNT * 12 + (TEMP_COORD_COUNT - 2) * 6];
static float g_tempColors[TEMP_COORD_COUNT * 24 + (TEMP_COORD_COUNT - 2) * 12];

static const int CIRCLE_VERTS = 8 * 4;
static float g_circleVerts[CIRCLE_VERTS * 2];

static stbtt_bakedchar g_cdata[96]; // ASCII 32..126 is 95 glyphs
static GLuint g_ftex = 0;
static GLuint g_whitetex = 0;
static GLuint g_vao = 0;
static GLuint g_vbos[3] = { 0, 0, 0 };
static GLuint g_program = 0;
static GLuint g_programViewportLocation = 0;
static GLuint g_programTextureLocation = 0;

static const unsigned TEXT_POOL_SIZE = 8000;
static char g_textPool[TEXT_POOL_SIZE];
static unsigned g_textPoolSize = 0;
static const char* allocText(const char* text)
{
	unsigned len = (unsigned)strlen(text) + 1;
	if (g_textPoolSize + len >= TEXT_POOL_SIZE)
		return 0;
	char* dst = &g_textPool[g_textPoolSize];
	memcpy(dst, text, len);
	g_textPoolSize += len;
	return dst;
}

static const unsigned GFXCMD_QUEUE_SIZE = 5000;
static GfxCmd g_gfxCmdQueue[GFXCMD_QUEUE_SIZE];
static unsigned g_gfxCmdQueueSize = 0;

static void ResetGfxCmdQueue()
{
	g_gfxCmdQueueSize = 0;
	g_textPoolSize = 0;
}

void AddGfxCmdScissor(int x, int y, int w, int h)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_SCISSOR;
	cmd.flags = x < 0 ? 0 : 1;      // on/off flag.
	cmd.col = 0;
	cmd.rect.x = (short)x;
	cmd.rect.y = (short)y;
	cmd.rect.w = (short)w;
	cmd.rect.h = (short)h;
}

void AddGfxCmdRect(float x, float y, float w, float h, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_RECT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.rect.x = (short)(x*8.0f);
	cmd.rect.y = (short)(y*8.0f);
	cmd.rect.w = (short)(w*8.0f);
	cmd.rect.h = (short)(h*8.0f);
	cmd.rect.r = 0;
}

void AddGfxCmdLine(float x0, float y0, float x1, float y1, float r, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_LINE;
	cmd.flags = 0;
	cmd.col = color;
	cmd.line.x0 = (short)(x0*8.0f);
	cmd.line.y0 = (short)(y0*8.0f);
	cmd.line.x1 = (short)(x1*8.0f);
	cmd.line.y1 = (short)(y1*8.0f);
	cmd.line.r = (short)(r*8.0f);
}

void AddGfxCmdRoundedRect(float x, float y, float w, float h, float r, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_RECT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.rect.x = (short)(x*8.0f);
	cmd.rect.y = (short)(y*8.0f);
	cmd.rect.w = (short)(w*8.0f);
	cmd.rect.h = (short)(h*8.0f);
	cmd.rect.r = (short)(r*8.0f);
}

void AddGfxCmdTriangle(int x, int y, int w, int h, int flags, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_TRIANGLE;
	cmd.flags = (char)flags;
	cmd.col = color;
	cmd.rect.x = (short)(x*8.0f);
	cmd.rect.y = (short)(y*8.0f);
	cmd.rect.w = (short)(w*8.0f);
	cmd.rect.h = (short)(h*8.0f);
}

//
void AddGfxCmdText(float x, float y, TextAlign align, const char* text, unsigned int color)
{
	if (g_gfxCmdQueueSize >= GFXCMD_QUEUE_SIZE)
		return;
	GfxCmd& cmd = g_gfxCmdQueue[g_gfxCmdQueueSize++];
	cmd.type = GFXCMD_TEXT;
	cmd.flags = 0;
	cmd.col = color;
	cmd.text.x = x;
	cmd.text.y = y;
	cmd.text.align = align;
	cmd.text.text = allocText(text);
}

//
void AddGfxCmdText(int x, int y, TextAlign align, const char* text, unsigned int color)
{
	AddGfxCmdText(float(x), float(y), align, text, color);
}

//
static void sDrawPolygon(const float* coords, unsigned numCoords, float r, unsigned int col)
{
	if (numCoords > TEMP_COORD_COUNT) numCoords = TEMP_COORD_COUNT;

	for (unsigned i = 0, j = numCoords - 1; i < numCoords; j = i++)
	{
		const float* v0 = &coords[j * 2];
		const float* v1 = &coords[i * 2];
		float dx = v1[0] - v0[0];
		float dy = v1[1] - v0[1];
		float d = sqrtf(dx*dx + dy*dy);
		if (d > 0)
		{
			d = 1.0f / d;
			dx *= d;
			dy *= d;
		}
		g_tempNormals[j * 2 + 0] = dy;
		g_tempNormals[j * 2 + 1] = -dx;
	}

	float colf[4] = { (float)(col & 0xff) / 255.f, (float)((col >> 8) & 0xff) / 255.f, (float)((col >> 16) & 0xff) / 255.f, (float)((col >> 24) & 0xff) / 255.f };
	float colTransf[4] = { (float)(col & 0xff) / 255.f, (float)((col >> 8) & 0xff) / 255.f, (float)((col >> 16) & 0xff) / 255.f, 0 };

	for (unsigned i = 0, j = numCoords - 1; i < numCoords; j = i++)
	{
		float dlx0 = g_tempNormals[j * 2 + 0];
		float dly0 = g_tempNormals[j * 2 + 1];
		float dlx1 = g_tempNormals[i * 2 + 0];
		float dly1 = g_tempNormals[i * 2 + 1];
		float dmx = (dlx0 + dlx1) * 0.5f;
		float dmy = (dly0 + dly1) * 0.5f;
		float   dmr2 = dmx*dmx + dmy*dmy;
		if (dmr2 > 0.000001f)
		{
			float   scale = 1.0f / dmr2;
			if (scale > 10.0f) scale = 10.0f;
			dmx *= scale;
			dmy *= scale;
		}
		g_tempCoords[i * 2 + 0] = coords[i * 2 + 0] + dmx*r;
		g_tempCoords[i * 2 + 1] = coords[i * 2 + 1] + dmy*r;
	}

	int vSize = numCoords * 12 + (numCoords - 2) * 6;
	int uvSize = numCoords * 2 * 6 + (numCoords - 2) * 2 * 3;
	int cSize = numCoords * 4 * 6 + (numCoords - 2) * 4 * 3;
	float * v = g_tempVertices;
	float * uv = g_tempTextureCoords;
	memset(uv, 0, uvSize * sizeof(float));
	float * c = g_tempColors;
	memset(c, 1, cSize * sizeof(float));

	float * ptrV = v;
	float * ptrC = c;
	for (unsigned i = 0, j = numCoords - 1; i < numCoords; j = i++)
	{
		*ptrV = coords[i * 2];
		*(ptrV + 1) = coords[i * 2 + 1];
		ptrV += 2;
		*ptrV = coords[j * 2];
		*(ptrV + 1) = coords[j * 2 + 1];
		ptrV += 2;
		*ptrV = g_tempCoords[j * 2];
		*(ptrV + 1) = g_tempCoords[j * 2 + 1];
		ptrV += 2;
		*ptrV = g_tempCoords[j * 2];
		*(ptrV + 1) = g_tempCoords[j * 2 + 1];
		ptrV += 2;
		*ptrV = g_tempCoords[i * 2];
		*(ptrV + 1) = g_tempCoords[i * 2 + 1];
		ptrV += 2;
		*ptrV = coords[i * 2];
		*(ptrV + 1) = coords[i * 2 + 1];
		ptrV += 2;

		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
		*ptrC = colTransf[0];
		*(ptrC + 1) = colTransf[1];
		*(ptrC + 2) = colTransf[2];
		*(ptrC + 3) = colTransf[3];
		ptrC += 4;
		*ptrC = colTransf[0];
		*(ptrC + 1) = colTransf[1];
		*(ptrC + 2) = colTransf[2];
		*(ptrC + 3) = colTransf[3];
		ptrC += 4;
		*ptrC = colTransf[0];
		*(ptrC + 1) = colTransf[1];
		*(ptrC + 2) = colTransf[2];
		*(ptrC + 3) = colTransf[3];
		ptrC += 4;
		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
	}

	for (unsigned i = 2; i < numCoords; ++i)
	{
		*ptrV = coords[0];
		*(ptrV + 1) = coords[1];
		ptrV += 2;
		*ptrV = coords[(i - 1) * 2];
		*(ptrV + 1) = coords[(i - 1) * 2 + 1];
		ptrV += 2;
		*ptrV = coords[i * 2];
		*(ptrV + 1) = coords[i * 2 + 1];
		ptrV += 2;

		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
		*ptrC = colf[0];
		*(ptrC + 1) = colf[1];
		*(ptrC + 2) = colf[2];
		*(ptrC + 3) = colf[3];
		ptrC += 4;
	}
	glBindTexture(GL_TEXTURE_2D, g_whitetex);

	glBindVertexArray(g_vao);
	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[0]);
	glBufferData(GL_ARRAY_BUFFER, vSize*sizeof(float), v, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[1]);
	glBufferData(GL_ARRAY_BUFFER, uvSize*sizeof(float), uv, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[2]);
	glBufferData(GL_ARRAY_BUFFER, cSize*sizeof(float), c, GL_STATIC_DRAW);
	glDrawArrays(GL_TRIANGLES, 0, (numCoords * 2 + numCoords - 2) * 3);
}

static void sDrawRect(float x, float y, float w, float h, float fth, unsigned int col)
{
	float verts[4 * 2] =
	{
		x + 0.5f, y + 0.5f,
		x + w - 0.5f, y + 0.5f,
		x + w - 0.5f, y + h - 0.5f,
		x + 0.5f, y + h - 0.5f,
	};
	sDrawPolygon(verts, 4, fth, col);
}

/*
static void drawEllipse(float x, float y, float w, float h, float fth, unsigned int col)
{
float verts[CIRCLE_VERTS*2];
const float* cverts = g_circleVerts;
float* v = verts;

for (int i = 0; i < CIRCLE_VERTS; ++i)
{
*v++ = x + cverts[i*2]*w;
*v++ = y + cverts[i*2+1]*h;
}

drawPolygon(verts, CIRCLE_VERTS, fth, col);
}
*/

static void sDrawRoundedRect(float x, float y, float w, float h, float r, float fth, unsigned int col)
{
	const unsigned n = CIRCLE_VERTS / 4;
	float verts[(n + 1) * 4 * 2];
	const float* cverts = g_circleVerts;
	float* v = verts;

	for (unsigned i = 0; i <= n; ++i)
	{
		*v++ = x + w - r + cverts[i * 2] * r;
		*v++ = y + h - r + cverts[i * 2 + 1] * r;
	}

	for (unsigned i = n; i <= n * 2; ++i)
	{
		*v++ = x + r + cverts[i * 2] * r;
		*v++ = y + h - r + cverts[i * 2 + 1] * r;
	}

	for (unsigned i = n * 2; i <= n * 3; ++i)
	{
		*v++ = x + r + cverts[i * 2] * r;
		*v++ = y + r + cverts[i * 2 + 1] * r;
	}

	for (unsigned i = n * 3; i < n * 4; ++i)
	{
		*v++ = x + w - r + cverts[i * 2] * r;
		*v++ = y + r + cverts[i * 2 + 1] * r;
	}
	*v++ = x + w - r + cverts[0] * r;
	*v++ = y + r + cverts[1] * r;

	sDrawPolygon(verts, (n + 1) * 4, fth, col);
}

//
void sRenderLine(float x0, float y0, float x1, float y1, float r, float fth, unsigned int col)
{
	float dx = x1 - x0;
	float dy = y1 - y0;
	float d = sqrtf(dx*dx + dy*dy);
	if (d > 0.0001f)
	{
		d = 1.0f / d;
		dx *= d;
		dy *= d;
	}
	float nx = dy;
	float ny = -dx;
	float verts[4 * 2];
	r -= fth;
	r *= 0.5f;
	if (r < 0.01f) r = 0.01f;
	dx *= r;
	dy *= r;
	nx *= r;
	ny *= r;

	verts[0] = x0 - dx - nx;
	verts[1] = y0 - dy - ny;

	verts[2] = x0 - dx + nx;
	verts[3] = y0 - dy + ny;

	verts[4] = x1 + dx + nx;
	verts[5] = y1 + dy + ny;

	verts[6] = x1 + dx - nx;
	verts[7] = y1 + dy - ny;

	sDrawPolygon(verts, 4, fth, col);
}

//
bool RenderGLInit(const char* fontpath)
{
	for (int i = 0; i < CIRCLE_VERTS; ++i)
	{
		float a = (float)i / (float)CIRCLE_VERTS * PI * 2;
		g_circleVerts[i * 2 + 0] = cosf(a);
		g_circleVerts[i * 2 + 1] = sinf(a);
	}

	// Load font.
	FILE* fp = fopen(fontpath, "rb");
	if (!fp) return false;
	fseek(fp, 0, SEEK_END);
	int size = (int)ftell(fp);
	fseek(fp, 0, SEEK_SET);

	unsigned char* ttfBuffer = (unsigned char*)malloc(size);
	if (!ttfBuffer)
	{
		fclose(fp);
		return false;
	}

	fread(ttfBuffer, 1, size, fp);
	fclose(fp);
	fp = 0;

	unsigned char* bmap = (unsigned char*)malloc(512 * 512);
	if (!bmap)
	{
		free(ttfBuffer);
		return false;
	}

	stbtt_BakeFontBitmap(ttfBuffer, 0, 15.0f, bmap, 512, 512, 32, 96, g_cdata);

	// can free ttf_buffer at this point
	glGenTextures(1, &g_ftex);
	glBindTexture(GL_TEXTURE_2D, g_ftex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, 512, 512, 0, GL_RED, GL_UNSIGNED_BYTE, bmap);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// can free ttf_buffer at this point
	unsigned char white_alpha = 255;
	glGenTextures(1, &g_whitetex);
	glBindTexture(GL_TEXTURE_2D, g_whitetex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, 1, 1, 0, GL_RED, GL_UNSIGNED_BYTE, &white_alpha);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glGenVertexArrays(1, &g_vao);
	glGenBuffers(3, g_vbos);

	glBindVertexArray(g_vao);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);

	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[0]);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT)* 2, (void*)0);
	glBufferData(GL_ARRAY_BUFFER, 0, 0, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[1]);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT)* 2, (void*)0);
	glBufferData(GL_ARRAY_BUFFER, 0, 0, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, g_vbos[2]);
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT)* 4, (void*)0);
	glBufferData(GL_ARRAY_BUFFER, 0, 0, GL_STATIC_DRAW);
	g_program = glCreateProgram();

	const char * vs =
		"#version 150\n"
		"uniform vec2 Viewport;\n"
		"in vec2 VertexPosition;\n"
		"in vec2 VertexTexCoord;\n"
		"in vec4 VertexColor;\n"
		"out vec2 texCoord;\n"
		"out vec4 vertexColor;\n"
		"void main(void)\n"
		"{\n"
		"    vertexColor = VertexColor;\n"
		"    texCoord = VertexTexCoord;\n"
		"    gl_Position = vec4(VertexPosition * 2.0 / Viewport - 1.0, 0.0, 1.0);\n"
		"}\n";
	GLuint vso = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vso, 1, (const char **)&vs, NULL);
	glCompileShader(vso);
	glAttachShader(g_program, vso);

	const char * fs =
		"#version 150\n"
		"in vec2 texCoord;\n"
		"in vec4 vertexColor;\n"
		"uniform sampler2D Texture;\n"
		"out vec4  Color;\n"
		"void main(void)\n"
		"{\n"
		"    float alpha = texture(Texture, texCoord).r;\n"
		"    Color = vec4(vertexColor.rgb, vertexColor.a * alpha);\n"
		"}\n";
	GLuint fso = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(fso, 1, (const char **)&fs, NULL);
	glCompileShader(fso);
	glAttachShader(g_program, fso);

	glBindAttribLocation(g_program, 0, "VertexPosition");
	glBindAttribLocation(g_program, 1, "VertexTexCoord");
	glBindAttribLocation(g_program, 2, "VertexColor");
	glBindFragDataLocation(g_program, 0, "Color");
	glLinkProgram(g_program);
	glDeleteShader(vso);
	glDeleteShader(fso);

	glUseProgram(g_program);
	g_programViewportLocation = glGetUniformLocation(g_program, "Viewport");
	g_programTextureLocation = glGetUniformLocation(g_program, "Texture");

	glUseProgram(0);

	free(ttfBuffer);
	free(bmap);

	return true;
}

//
void RenderGLDestroy()
{
	if (g_ftex)
	{
		glDeleteTextures(1, &g_ftex);
		g_ftex = 0;
	}

	if (g_vao)
	{
		glDeleteVertexArrays(1, &g_vao);
		glDeleteBuffers(3, g_vbos);
		g_vao = 0;
	}

	if (g_program)
	{
		glDeleteProgram(g_program);
		g_program = 0;
	}

}

static void sGetBakedQuad(stbtt_bakedchar *chardata, int pw, int ph, int char_index,
						 float *xpos, float *ypos, stbtt_aligned_quad *q)
{
	stbtt_bakedchar *b = chardata + char_index;
	int round_x = STBTT_ifloor(*xpos + b->xoff);
	int round_y = STBTT_ifloor(*ypos - b->yoff);

	q->x0 = (float)round_x;
	q->y0 = (float)round_y;
	q->x1 = (float)round_x + b->x1 - b->x0;
	q->y1 = (float)round_y - b->y1 + b->y0;

	q->s0 = b->x0 / (float)pw;
	q->t0 = b->y0 / (float)pw;
	q->s1 = b->x1 / (float)ph;
	q->t1 = b->y1 / (float)ph;

	*xpos += b->xadvance;
}

static const float g_tabStops[4] = { 150, 210, 270, 330 };

static float sGetTextLength(stbtt_bakedchar *chardata, const char* text)
{
	float xpos = 0;
	float len = 0;
	while (*text)
	{
		int c = (unsigned char)*text;
		if (c == '\t')
		{
			for (int i = 0; i < 4; ++i)
			{
				if (xpos < g_tabStops[i])
				{
					xpos = g_tabStops[i];
					break;
				}
			}
		}
		else if (c >= 32 && c < 128)
		{
			stbtt_bakedchar *b = chardata + c - 32;
			int round_x = STBTT_ifloor((xpos + b->xoff) + 0.5);
			len = round_x + b->x1 - b->x0 + 0.5f;
			xpos += b->xadvance;
		}
		++text;
	}
	return len;
}

//
void sRenderString(float x, float y, const char *text, TextAlign align, unsigned int col)
{
	if (!g_ftex) return;
	if (!text) return;

	if (align == TEXT_ALIGN_CENTER)
		x -= sGetTextLength(g_cdata, text) / 2;
	else if (align == TEXT_ALIGN_RIGHT)
		x -= sGetTextLength(g_cdata, text);

	float r = (float)(col & 0xff) / 255.f;
	float g = (float)((col >> 8) & 0xff) / 255.f;
	float b = (float)((col >> 16) & 0xff) / 255.f;
	float a = (float)((col >> 24) & 0xff) / 255.f;

	// assume orthographic projection with units = screen pixels, origin at top left
	glBindTexture(GL_TEXTURE_2D, g_ftex);

	const float ox = x;

	while (*text)
	{
		int c = (unsigned char)*text;
		if (c == '\t')
		{
			for (int i = 0; i < 4; ++i)
			{
				if (x < g_tabStops[i] + ox)
				{
					x = g_tabStops[i] + ox;
					break;
				}
			}
		}
		else if (c >= 32 && c < 128)
		{
			stbtt_aligned_quad q;
			sGetBakedQuad(g_cdata, 512, 512, c - 32, &x, &y, &q);

			float v[12] = {
				q.x0, q.y0,
				q.x1, q.y1,
				q.x1, q.y0,
				q.x0, q.y0,
				q.x0, q.y1,
				q.x1, q.y1,
			};
			float uv[12] = {
				q.s0, q.t0,
				q.s1, q.t1,
				q.s1, q.t0,
				q.s0, q.t0,
				q.s0, q.t1,
				q.s1, q.t1,
			};
			float color[24] = {
				r, g, b, a,
				r, g, b, a,
				r, g, b, a,
				r, g, b, a,
				r, g, b, a,
				r, g, b, a,
			};
			glBindVertexArray(g_vao);
			glBindBuffer(GL_ARRAY_BUFFER, g_vbos[0]);
			glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), v, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, g_vbos[1]);
			glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), uv, GL_STATIC_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, g_vbos[2]);
			glBufferData(GL_ARRAY_BUFFER, 24 * sizeof(float), color, GL_STATIC_DRAW);
			glDrawArrays(GL_TRIANGLES, 0, 6);

		}
		++text;
	}
}

//
void RenderGLFlush(int width, int height)
{
	const GfxCmd* q = g_gfxCmdQueue;
	int nq = g_gfxCmdQueueSize;

	const float s = 1.0f / 8.0f;

	glViewport(0, 0, width, height);
	glUseProgram(g_program);
	glUniform2f(g_programViewportLocation, (float)width, (float)height);
	glUniform1i(g_programTextureLocation, 0);

	glDisable(GL_SCISSOR_TEST);
	for (int i = 0; i < nq; ++i)
	{
		const GfxCmd& cmd = q[i];
		if (cmd.type == GFXCMD_RECT)
		{
			if (cmd.rect.r == 0)
			{
				sDrawRect((float)cmd.rect.x*s + 0.5f, (float)cmd.rect.y*s + 0.5f,
						 (float)cmd.rect.w*s - 1, (float)cmd.rect.h*s - 1,
						 1.0f, cmd.col);
			}
			else
			{
				sDrawRoundedRect((float)cmd.rect.x*s + 0.5f, (float)cmd.rect.y*s + 0.5f,
								(float)cmd.rect.w*s - 1, (float)cmd.rect.h*s - 1,
								(float)cmd.rect.r*s, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == GFXCMD_LINE)
		{
			sRenderLine(cmd.line.x0*s, cmd.line.y0*s, cmd.line.x1*s, cmd.line.y1*s, cmd.line.r*s, 1.0f, cmd.col);
		}
		else if (cmd.type == GFXCMD_TRIANGLE)
		{
			if (cmd.flags == 1)
			{
				const float verts[3 * 2] =
				{
					(float)cmd.rect.x*s + 0.5f, (float)cmd.rect.y*s + 0.5f,
					(float)cmd.rect.x*s + 0.5f + (float)cmd.rect.w*s - 1, (float)cmd.rect.y*s + 0.5f + (float)cmd.rect.h*s / 2 - 0.5f,
					(float)cmd.rect.x*s + 0.5f, (float)cmd.rect.y*s + 0.5f + (float)cmd.rect.h*s - 1,
				};
				sDrawPolygon(verts, 3, 1.0f, cmd.col);
			}
			if (cmd.flags == 2)
			{
				const float verts[3 * 2] =
				{
					(float)cmd.rect.x*s + 0.5f, (float)cmd.rect.y*s + 0.5f + (float)cmd.rect.h*s - 1,
					(float)cmd.rect.x*s + 0.5f + (float)cmd.rect.w*s / 2 - 0.5f, (float)cmd.rect.y*s + 0.5f,
					(float)cmd.rect.x*s + 0.5f + (float)cmd.rect.w*s - 1, (float)cmd.rect.y*s + 0.5f + (float)cmd.rect.h*s - 1,
				};
				sDrawPolygon(verts, 3, 1.0f, cmd.col);
			}
		}
		else if (cmd.type == GFXCMD_TEXT)
		{
			sRenderString(cmd.text.x, cmd.text.y, cmd.text.text, cmd.text.align, cmd.col);
		}
		else if (cmd.type == GFXCMD_SCISSOR)
		{
			if (cmd.flags)
			{
				glEnable(GL_SCISSOR_TEST);
				glScissor(cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h);
			}
			else
			{
				glDisable(GL_SCISSOR_TEST);
			}
		}
	}
	glDisable(GL_SCISSOR_TEST);
	glUseProgram(0);

	ResetGfxCmdQueue();
}

