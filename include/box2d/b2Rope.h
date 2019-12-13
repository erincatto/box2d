/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include "Box2D/Common/b2Math.h"

class b2Draw;

enum b2BendingModel
{
	b2_pbdDistanceBendingModel,
	b2_pbdAngleBendingModel,
	b2_forceAngleBendingModel,
	b2_xpbdAngleBendingModel
};

///
struct b2RopeTuning
{
	b2RopeTuning()
	{
		bendingModel = b2_pbdDistanceBendingModel;
		damping = 0.0f;
		stretchStiffness = 1.0f;
		bendStiffness = 0.5f;
		bendHertz = 1.0f;
		bendDamping = 0.0f;
	}

	b2BendingModel bendingModel;
	float32 damping;
	float32 stretchStiffness;
	float32 bendStiffness;
	float32 bendHertz;
	float32 bendDamping;
};

/// 
struct b2RopeDef
{
	b2RopeDef()
	{
		vertices = nullptr;
		count = 0;
		masses = nullptr;
		gravity.SetZero();
	}

	///
	b2Vec2* vertices;

	///
	int32 count;

	///
	float32* masses;

	///
	b2Vec2 gravity;

	b2RopeTuning tuning;
};

/// 
class b2Rope
{
public:
	b2Rope();
	~b2Rope();

	///
	void Initialize(const b2RopeDef* def);

	///
	void SetTuning(const b2RopeTuning& tuning);

	///
	void Step(float32 timeStep, int32 iterations, const b2Vec2& position);

	///
	int32 GetVertexCount() const
	{
		return m_count;
	}

	///
	const b2Vec2* GetVertices() const
	{
		return m_ps;
	}

	///
	void Draw(b2Draw* draw) const;

	///
	void SetAngle(float32 angle);

private:

	void SolveStretch();
	//void SolveBend_PBD_Distance();
	void SolveBend_PBD_Angle();
	void SolveBend_XPBD_Angle(float32 dt);
	void ApplyBendForces(float32 dt);

	int32 m_count;
	int32 m_stretchCount;
	int32 m_bendCount;

	b2Vec2* m_bindPositions;
	b2Vec2* m_ps;
	b2Vec2* m_p0s;
	b2Vec2* m_vs;

	float32* m_ims;

	float32* m_Ls;
	float32* m_as;
	float32* m_bendingLambdas;

	b2Vec2 m_gravity;

	b2RopeTuning m_tuning;
};

#endif
