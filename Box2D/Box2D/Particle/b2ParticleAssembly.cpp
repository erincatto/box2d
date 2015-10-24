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
#include <Box2D/Particle/b2ParticleAssembly.h>
#include <Box2D/Particle/b2ParticleSystem.h>

extern "C" {

// Helper function, called from assembly routine.
void GrowParticleContactBuffer(
	b2GrowableBuffer<b2ParticleContact>& contacts)
{
	// Set contacts.count = capacity instead of count because there are
	// items past the end of the array waiting to be post-processed.
	// We must maintain the entire contacts array.
	// TODO: It would be better to have the items awaiting post-processing
	// in their own array on the stack.
	contacts.SetCount(contacts.GetCapacity());
	contacts.Grow();
}

} // extern "C"

