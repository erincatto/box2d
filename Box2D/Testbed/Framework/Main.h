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
#ifndef MAIN_H
#define MAIN_H
#include <Box2D/Box2D.h>
#include "ParticleParameter.h"

namespace TestMain
{

// Set whether to restart the test on particle parameter changes.
// This parameter is re-enabled when the test changes.
void SetRestartOnParticleParameterChange(bool enable);

// Set the currently selected particle parameter value.  This value must
// match one of the values in TestMain::k_particleTypes or one of the values
// referenced by particleParameterDef passed to SetParticleParameters().
uint32 SetParticleParameterValue(uint32 value);

// Get the currently selected particle parameter value and enable particle
// parameter selection arrows on Android.
uint32 GetParticleParameterValue();

// Override the default particle parameters for the test.
void SetParticleParameters(
	const ParticleParameter::Definition * const particleParameterDef,
	const uint32 particleParameterDefCount);

}  // namespace TestMain

#endif  // MAIN_H
