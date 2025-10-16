// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"
#include "utility.h"
#include "stb_truetype.h"

#include <vector>

struct Camera;

struct Vertex
{
	b3Vec2 position;
	b3Vec2 uv;
	RGBA8 color;
};

class Font
{
public:
	static Font* Create( const Camera* camera, const char* path, int fontSize );
	~Font();

	void AddText( float x, float y, b3HexColor color, const char* text);

	void Flush( );

private:
	Font( const Camera* camera, float fontSize );
	bool Initialize( const char* path );

	static constexpr int m_firstCharacter = 32;
	static constexpr int m_characterCount = 96;
	static constexpr int m_atlasWidth = 512;
	static constexpr int m_atlasHeight = 512;

	// The number of vertices the vbo can hold
	static constexpr int m_vboCapacity = 6 * 10000;

	const Camera* m_camera;
	float m_fontSize;
	stbtt_bakedchar* m_characters;
	std::vector<Vertex> m_vertices;
	unsigned int m_textureId;
	uint32_t m_vaoId;
	uint32_t m_vboId;
	uint32_t m_programId;
};
