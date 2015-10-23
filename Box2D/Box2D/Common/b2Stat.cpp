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
#include "b2Stat.h"

#include <algorithm>
#include <cfloat>

b2Stat::b2Stat()
{
	Clear();
}

void b2Stat::Record( float32 t )
{
	m_total += t;
	m_min = std::min(m_min,t);
	m_max = std::max(m_max,t);
	m_count++;
}

int b2Stat::GetCount() const
{
	return m_count;
}

float32 b2Stat::GetMean() const
{
	if (m_count == 0)
	{
		return 0.0f;
	}
	return (float32)(m_total / m_count);
}

float32 b2Stat::GetMin() const
{
	return m_min;
}

float32 b2Stat::GetMax() const
{
	return m_max;
}

void b2Stat::Clear()
{
	m_count = 0;
	m_total = 0;
	m_min = FLT_MAX;
	m_max = -FLT_MAX;
}

