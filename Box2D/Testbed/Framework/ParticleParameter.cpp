/*
* Copyright (c) 2014 Google, Inc.
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

#include "ParticleParameter.h"

const uint32 ParticleParameter::k_DefaultOptions =
	OptionDrawShapes | OptionDrawParticles;

const ParticleParameter::Value ParticleParameter::k_particleTypes[] =
{
	{ b2_waterParticle, ParticleParameter::k_DefaultOptions, "water" },
	{ b2_waterParticle, ParticleParameter::k_DefaultOptions |
				ParticleParameter::OptionStrictContacts, "water (strict)" },
	{ b2_springParticle, ParticleParameter::k_DefaultOptions, "spring" },
	{ b2_elasticParticle, ParticleParameter::k_DefaultOptions, "elastic" },
	{ b2_viscousParticle, ParticleParameter::k_DefaultOptions, "viscous" },
	{ b2_powderParticle, ParticleParameter::k_DefaultOptions, "powder" },
	{ b2_tensileParticle, ParticleParameter::k_DefaultOptions, "tensile" },
	{ b2_colorMixingParticle, ParticleParameter::k_DefaultOptions,
		"color mixing" },
	{ b2_wallParticle, ParticleParameter::k_DefaultOptions, "wall" },
	{ b2_barrierParticle | b2_wallParticle,
	    ParticleParameter::k_DefaultOptions, "barrier" },
	{ b2_staticPressureParticle, ParticleParameter::k_DefaultOptions,
		"static pressure" },
	{ b2_waterParticle, ParticleParameter::k_DefaultOptions |
				ParticleParameter::OptionDrawAABBs, "water (bounding boxes)" },
};
const ParticleParameter::Value *ParticleParameter::k_particleTypesPtr =
	ParticleParameter::k_particleTypes;
const uint32 ParticleParameter::k_particleTypesCount =
	B2_ARRAY_SIZE(ParticleParameter::k_particleTypes);

const ParticleParameter::Definition
	ParticleParameter::k_defaultDefinition[] =
{
	{
		ParticleParameter::k_particleTypes,
		ParticleParameter::k_particleTypesCount
	},
};

// Reset to the default state.
void ParticleParameter::Reset()
{
	m_restartOnChange = true;
	m_index = 0;
	SetDefinition(k_defaultDefinition, B2_ARRAY_SIZE(k_defaultDefinition));
	Set(0);
}

// Set the parameter definition.
void ParticleParameter::SetDefinition(const Definition *definition,
									  uint32 definitionCount)
{
	m_definition = definition;
	m_definitionCount = definitionCount;
	m_valueCount = 0;
	for (uint32 i = 0; i < m_definitionCount; ++i)
	{
		m_valueCount += m_definition[i].numValues;
	}
	// Refresh the selected value.
	Set(Get());
}

// Set selected parameter index.
void ParticleParameter::Set(uint32 index)
{
	m_changed = m_index != index;
	m_index = m_valueCount ? index % m_valueCount : index;
	m_value = FindParticleParameterValue();
	b2Assert(m_value);
}

// Increment the parameter index.
void ParticleParameter::Increment()
{
	uint32 index = Get();
	Set(index >= m_valueCount ? 0 : index + 1);
}

// Decrement the parameter index.
void ParticleParameter::Decrement()
{
	uint32 index = Get();
	Set(index == 0 ? m_valueCount - 1 : index - 1);
}

// Determine whether the parameter changed and reset the changed flag.
bool ParticleParameter::Changed(bool * const restart)
{
	bool changed = m_changed;
	m_changed = false;
	if (restart)
	{
		*restart = changed && GetRestartOnChange();
	}
	return changed;
}

// Find index by value falling back to -1 if the value isn't found.
int32 ParticleParameter::FindIndexByValue(uint32 value) const
{
	uint32 index = 0;
	for (uint32 i = 0; i < m_definitionCount; ++i)
	{
		const Definition &definition = m_definition[i];
		const uint32 numValues = definition.numValues;
		for (uint32 j = 0; j < numValues; ++j, ++index)
		{
			if (definition.values[j].value == value) return index;
		}
	}
	return -1;
}

// Find the value of the current parameter.
const ParticleParameter::Value*
	ParticleParameter::FindParticleParameterValue() const
{
	uint32 start = 0;
	const uint32 index = Get();
	for (uint32 i = 0; i < m_definitionCount; ++i)
	{
		const Definition &definition = m_definition[i];
		const uint32 end = start + definition.numValues;
		if (index >= start && index < end)
		{
			return &definition.values[index - start];
		}
		start = end;
	}
	return NULL;
}
