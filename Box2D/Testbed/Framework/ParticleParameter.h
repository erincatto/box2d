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
#ifndef PARTICLE_PARAMETER_H
#define PARTICLE_PARAMETER_H
#include <Box2D/Box2D.h>

// Manages a list of parameter values that can be iterated through using
// simple UI elements.
class ParticleParameter
{
public:

	enum Options
	{
		OptionStrictContacts = 1 << 0,
		OptionDrawShapes = 1 << 1,
		OptionDrawParticles = 1 << 2,
		OptionDrawJoints = 1 << 3,
		OptionDrawAABBs = 1 << 4,
		OptionDrawContactPoints = 1 << 5,
		OptionDrawContactNormals = 1 << 6,
		OptionDrawContactImpulse = 1 << 7,
		OptionDrawFrictionImpulse = 1 << 8,
		OptionDrawCOMs = 1 << 9,
		OptionDrawStats = 1 << 10,
		OptionDrawProfile = 1 << 11
	};

	static const uint32 k_DefaultOptions;

	// Value of a particle parameter.
	struct Value
	{
		// Value associated with the parameter.
		uint32 value;
		// Any global (non particle-specific) options associated with this
		// parameter
		uint32 options;
		// Name to display when this parameter is selected.
		const char *name;
	};

	// Particle parameter definition.
	struct Definition
	{
		// Values associated with this particle parameter definition.
		const Value *values;
		// Number of entries in the "values" array.
		uint32 numValues;

		// Calculate the mask (bitwise OR) of all values in the definition.
		uint32 CalculateValueMask() const
		{
			uint32 mask = 0;
			for (uint32 i = 0; i < numValues; i++)
			{
				mask |= values[i].value;
			}
			return mask;
		}
	};

public:
	ParticleParameter() { Reset(); }

	// Reset to the default state.
	void Reset();

	// Set the parameter definition.  "definition" is directly referenced by
	// this class so must be present for the lifetime of this object.
	void SetDefinition(const Definition *definition,
					   uint32 definitionCount);

	// Get selected parameter index.
	uint32 Get() const { return m_index; }

	// Set selected parameter index.
	void Set(uint32 index);

	// Increment the parameter index.
	void Increment();
	// Decrement the parameter index.
	void Decrement();

	// Determine whether the parameter changed, reset the changed flag and
	// optionally determine whether the current test should be restarted if
	// restart != NULL.
	bool Changed(bool * const restart);

	// Get the selected parameter value.
	uint32 GetValue() const
	{
		b2Assert(m_value);
		return m_value->value;
	}

	// Get the selected parameter name.
	const char* GetName() const
	{
		b2Assert(m_value);
		return m_value->name;
	}

	// Get the selected parameter value.
	uint32 GetOptions() const
	{
		b2Assert(m_value);
		return m_value->options;
	}

	// Set whether to restart the test when changing this parameter,
	// in if this is set it disables Reset().
	void SetRestartOnChange(bool enable) { m_restartOnChange = enable; }

	// Get whether to restart the test when changing this parameter.
	bool GetRestartOnChange() const { return m_restartOnChange; }

	// Find index by value falling back to -1 if the value isn't found.
	int32 FindIndexByValue(uint32 value) const;

protected:
	// Find the value of the current parameter.
	const Value* FindParticleParameterValue() const;

private:
	// Index of the currently selected parameter value out of the list of
	// parameter definitions.
	uint32 m_index;
	// Whether the parameter has changed in the last time Changed() was called.
	bool m_changed;
	// Whether tests should be restarted when this parameter changes.
	bool m_restartOnChange;
	// The currently selected parameter.
	const Value *m_value;
	// Array of available parameter values that can be selected.
	const Definition *m_definition;
	// Number of items in the m_definition array.
	uint32 m_definitionCount;
	// Number of values referenced by the m_definition array.
	uint32 m_valueCount;

public:
	// Pointer to the k_particleTypes array.
	static const Value *k_particleTypesPtr;
	// Number of items in the k_particleTypes array.
	static const uint32 k_particleTypesCount;
	// All basic particle types supported by the particle module.
	static const Value k_particleTypes[];
	// Default parameter definition for this class.
	static const Definition k_defaultDefinition[];
};

#endif  // PARTICLE_PARAMETER_H
