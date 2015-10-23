/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#include <Box2D/Particle/b2VoronoiDiagram.h>
#include <Box2D/Particle/b2StackQueue.h>
#include <Box2D/Collision/b2Collision.h>

b2VoronoiDiagram::b2VoronoiDiagram(
	b2StackAllocator* allocator, int32 generatorCapacity)
{
	m_allocator = allocator;
	m_generatorBuffer =
		(Generator*) allocator->Allocate(
			sizeof(Generator) * generatorCapacity);
	m_generatorCapacity = generatorCapacity;
	m_generatorCount = 0;
	m_countX = 0;
	m_countY = 0;
	m_diagram = NULL;
}

b2VoronoiDiagram::~b2VoronoiDiagram()
{
	if (m_diagram)
	{
		m_allocator->Free(m_diagram);
	}
	m_allocator->Free(m_generatorBuffer);
}

void b2VoronoiDiagram::AddGenerator(
	const b2Vec2& center, int32 tag, bool necessary)
{
	b2Assert(m_generatorCount < m_generatorCapacity);
	Generator& g = m_generatorBuffer[m_generatorCount++];
	g.center = center;
	g.tag = tag;
	g.necessary = necessary;
}

void b2VoronoiDiagram::Generate(float32 radius, float32 margin)
{
	b2Assert(m_diagram == NULL);
	float32 inverseRadius = 1 / radius;
	b2Vec2 lower(+b2_maxFloat, +b2_maxFloat);
	b2Vec2 upper(-b2_maxFloat, -b2_maxFloat);
	for (int32 k = 0; k < m_generatorCount; k++)
	{
		Generator& g = m_generatorBuffer[k];
		if (g.necessary)
		{
			lower = b2Min(lower, g.center);
			upper = b2Max(upper, g.center);
		}
	}
	lower.x -= margin;
	lower.y -= margin;
	upper.x += margin;
	upper.y += margin;
	m_countX = 1 + (int32) (inverseRadius * (upper.x - lower.x));
	m_countY = 1 + (int32) (inverseRadius * (upper.y - lower.y));
	m_diagram = (Generator**)
		m_allocator->Allocate(sizeof(Generator*) * m_countX * m_countY);
	for (int32 i = 0; i < m_countX * m_countY; i++)
	{
		m_diagram[i] = NULL;
	}
	// (4 * m_countX * m_countY) is the queue capacity that is experimentally
	// known to be necessary and sufficient for general particle distributions.
	b2StackQueue<b2VoronoiDiagramTask> queue(
		m_allocator, 4 * m_countX * m_countY);
	for (int32 k = 0; k < m_generatorCount; k++)
	{
		Generator& g = m_generatorBuffer[k];
		g.center = inverseRadius * (g.center - lower);
		int32 x = (int32) g.center.x;
		int32 y = (int32) g.center.y;
		if (x >=0 && y >= 0 && x < m_countX && y < m_countY)
		{
			queue.Push(b2VoronoiDiagramTask(x, y, x + y * m_countX, &g));
		}
	}
	while (!queue.Empty())
	{
		int32 x = queue.Front().m_x;
		int32 y = queue.Front().m_y;
		int32 i = queue.Front().m_i;
		Generator* g = queue.Front().m_generator;
		queue.Pop();
		if (!m_diagram[i])
		{
			m_diagram[i] = g;
			if (x > 0)
			{
				queue.Push(b2VoronoiDiagramTask(x - 1, y, i - 1, g));
			}
			if (y > 0)
			{
				queue.Push(b2VoronoiDiagramTask(x, y - 1, i - m_countX, g));
			}
			if (x < m_countX - 1)
			{
				queue.Push(b2VoronoiDiagramTask(x + 1, y, i + 1, g));
			}
			if (y < m_countY - 1)
			{
				queue.Push(b2VoronoiDiagramTask(x, y + 1, i + m_countX, g));
			}
		}
	}
	for (int32 y = 0; y < m_countY; y++)
	{
		for (int32 x = 0; x < m_countX - 1; x++)
		{
			int32 i = x + y * m_countX;
			Generator* a = m_diagram[i];
			Generator* b = m_diagram[i + 1];
			if (a != b)
			{
				queue.Push(b2VoronoiDiagramTask(x, y, i, b));
				queue.Push(b2VoronoiDiagramTask(x + 1, y, i + 1, a));
			}
		}
	}
	for (int32 y = 0; y < m_countY - 1; y++)
	{
		for (int32 x = 0; x < m_countX; x++)
		{
			int32 i = x + y * m_countX;
			Generator* a = m_diagram[i];
			Generator* b = m_diagram[i + m_countX];
			if (a != b)
			{
				queue.Push(b2VoronoiDiagramTask(x, y, i, b));
				queue.Push(b2VoronoiDiagramTask(x, y + 1, i + m_countX, a));
			}
		}
	}
	while (!queue.Empty())
	{
		const b2VoronoiDiagramTask& task = queue.Front();
		int32 x = task.m_x;
		int32 y = task.m_y;
		int32 i = task.m_i;
		Generator* k = task.m_generator;
		queue.Pop();
		Generator* a = m_diagram[i];
		Generator* b = k;
		if (a != b)
		{
			float32 ax = a->center.x - x;
			float32 ay = a->center.y - y;
			float32 bx = b->center.x - x;
			float32 by = b->center.y - y;
			float32 a2 = ax * ax + ay * ay;
			float32 b2 = bx * bx + by * by;
			if (a2 > b2)
			{
				m_diagram[i] = b;
				if (x > 0)
				{
					queue.Push(b2VoronoiDiagramTask(x - 1, y, i - 1, b));
				}
				if (y > 0)
				{
					queue.Push(b2VoronoiDiagramTask(x, y - 1, i - m_countX, b));
				}
				if (x < m_countX - 1)
				{
					queue.Push(b2VoronoiDiagramTask(x + 1, y, i + 1, b));
				}
				if (y < m_countY - 1)
				{
					queue.Push(b2VoronoiDiagramTask(x, y + 1, i + m_countX, b));
				}
			}
		}
	}
}

void b2VoronoiDiagram::GetNodes(NodeCallback& callback) const
{
	for (int32 y = 0; y < m_countY - 1; y++)
	{
		for (int32 x = 0; x < m_countX - 1; x++)
		{
			int32 i = x + y * m_countX;
			const Generator* a = m_diagram[i];
			const Generator* b = m_diagram[i + 1];
			const Generator* c = m_diagram[i + m_countX];
			const Generator* d = m_diagram[i + 1 + m_countX];
			if (b != c)
			{
				if (a != b && a != c &&
					(a->necessary || b->necessary || c->necessary))
				{
					callback(a->tag, b->tag, c->tag);
				}
				if (d != b && d != c &&
					(b->necessary || d->necessary || c->necessary))
				{
					callback(b->tag, d->tag, c->tag);
				}
			}
		}
	}
}
