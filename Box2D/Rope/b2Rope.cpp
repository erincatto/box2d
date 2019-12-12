/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#include "Box2D/Rope/b2Rope.h"
#include "Box2D/Common/b2Draw.h"

b2Rope::b2Rope()
{
	m_count = 0;
	m_bindPositions = nullptr;
	m_ps = nullptr;
	m_p0s = nullptr;
	m_vs = nullptr;
	m_ims = nullptr;
	m_Ls = nullptr;
	m_as = nullptr;
	m_gravity.SetZero();
}

b2Rope::~b2Rope()
{
	b2Free(m_bindPositions);
	b2Free(m_ps);
	b2Free(m_p0s);
	b2Free(m_vs);
	b2Free(m_ims);
	b2Free(m_Ls);
	b2Free(m_as);
	b2Free(m_bendingLambdas);
}

void b2Rope::Initialize(const b2RopeDef* def)
{
	b2Assert(def->count >= 3);
	m_count = def->count;
	m_bindPositions = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_ps = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_p0s = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_vs = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
	m_ims = (float32*)b2Alloc(m_count * sizeof(float32));

	for (int32 i = 0; i < m_count; ++i)
	{
		m_bindPositions[i] = def->vertices[i];
		m_ps[i] = def->vertices[i];
		m_p0s[i] = def->vertices[i];
		m_vs[i].SetZero();

		float32 m = def->masses[i];
		if (m > 0.0f)
		{
			m_ims[i] = 1.0f / m;
		}
		else
		{
			m_ims[i] = 0.0f;
		}
	}

	m_stretchCount = m_count - 1;
	m_bendCount = m_count - 2;
	m_Ls = (float32*)b2Alloc(m_stretchCount * sizeof(float32));
	m_as = (float32*)b2Alloc(m_bendCount * sizeof(float32));
	m_bendingLambdas = (float32*)b2Alloc(m_bendCount * sizeof(float32));

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i+1];
		m_Ls[i] = b2Distance(p1, p2);
	}

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];
		b2Vec2 p3 = m_ps[i + 2];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		float32 a = b2Cross(d1, d2);
		float32 b = b2Dot(d1, d2);

		m_as[i] = b2Atan2(a, b);
		m_bendingLambdas[i] = 0.0f;
	}

	m_gravity = def->gravity;
	m_tuning = def->tuning;
}

void b2Rope::SetTuning(const b2RopeTuning& tuning)
{
	m_tuning = tuning;
}

void b2Rope::Step(float32 dt, int32 iterations, const b2Vec2& position)
{
	if (dt == 0.0)
	{
		return;
	}

	const float32 inv_dt = 1.0f / dt;
	float32 d = expf(- dt * m_tuning.damping);

	for (int32 i = 0; i < m_count; ++i)
	{
		if (m_ims[i] > 0.0f)
		{
			m_vs[i] += dt * m_gravity;
			m_vs[i] *= d;
		}
		else
		{
			m_vs[i] = inv_dt * (m_bindPositions[i] + position - m_p0s[i]);
		}
	}

	if (m_tuning.bendingModel == b2_forceAngleBendingModel)
	{
		ApplyBendForces(dt);
	}

	for (int32 i = 0; i < m_count; ++i)
	{
		m_ps[i] += dt * m_vs[i];
	}

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		m_bendingLambdas[i] = 0.0f;
	}

	for (int32 i = 0; i < iterations; ++i)
	{
		if (m_tuning.bendingModel == b2_pbdAngleBendingModel)
		{
			SolveBend_PBD_Angle();
		}
		else if (m_tuning.bendingModel == b2_xpbdAngleBendingModel)
		{
			SolveBend_XPBD_Angle(dt);
		}

		SolveStretch();
	}

	for (int32 i = 0; i < m_count; ++i)
	{
		m_vs[i] = inv_dt * (m_ps[i] - m_p0s[i]);
		m_p0s[i] = m_ps[i];
	}
}

void b2Rope::SolveStretch()
{
	const float32 k2 = m_tuning.stretchStiffness;

	for (int32 i = 0; i < m_stretchCount; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];

		b2Vec2 d = p2 - p1;
		float32 L = d.Normalize();

		float32 im1 = m_ims[i];
		float32 im2 = m_ims[i + 1];

		if (im1 + im2 == 0.0f)
		{
			continue;
		}

		float32 s1 = im1 / (im1 + im2);
		float32 s2 = im2 / (im1 + im2);

		p1 -= k2 * s1 * (m_Ls[i] - L) * d;
		p2 += k2 * s2 * (m_Ls[i] - L) * d;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
	}
}

void b2Rope::SetAngle(float32 angle)
{
	for (int32 i = 0; i < m_bendCount; ++i)
	{
		m_as[i] = angle;
	}
}

void b2Rope::SolveBend_PBD_Angle()
{
	const float32 k3 = m_tuning.bendStiffness;

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];
		b2Vec2 p3 = m_ps[i + 2];

		float32 m1 = m_ims[i];
		float32 m2 = m_ims[i + 1];
		float32 m3 = m_ims[i + 2];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		float32 L1sqr = d1.LengthSquared();
		float32 L2sqr = d2.LengthSquared();

		if (L1sqr * L2sqr == 0.0f)
		{
			continue;
		}

		float32 a = b2Cross(d1, d2);
		float32 b = b2Dot(d1, d2);

		float32 angle = b2Atan2(a, b);

		b2Vec2 Jd1 = (-1.0f / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (1.0f / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		float32 mass = m1 * b2Dot(J1, J1) + m2 * b2Dot(J2, J2) + m3 * b2Dot(J3, J3);
		if (mass == 0.0f)
		{
			continue;
		}

		mass = 1.0f / mass;

		float32 C = angle - m_as[i];

		while (C > b2_pi)
		{
			angle -= 2 * b2_pi;
			C = angle - m_as[i];
		}

		while (C < -b2_pi)
		{
			angle += 2.0f * b2_pi;
			C = angle - m_as[i];
		}

		float32 impulse = - k3 * mass * C;

		p1 += (m1 * impulse) * J1;
		p2 += (m2 * impulse) * J2;
		p3 += (m3 * impulse) * J3;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
		m_ps[i + 2] = p3;
	}
}

void b2Rope::SolveBend_XPBD_Angle(float32 dt)
{
	b2Assert(dt > 0.0f);

	for (int32 i = 0; i < m_bendCount; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];
		b2Vec2 p3 = m_ps[i + 2];

		b2Vec2 v1 = m_vs[i];
		b2Vec2 v2 = m_vs[i + 1];
		b2Vec2 v3 = m_vs[i + 2];

		float32 m1 = m_ims[i];
		float32 m2 = m_ims[i + 1];
		float32 m3 = m_ims[i + 2];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		float32 L1sqr = d1.LengthSquared();
		float32 L2sqr = d2.LengthSquared();

		if (L1sqr * L2sqr == 0.0f)
		{
			continue;
		}

		float32 a = b2Cross(d1, d2);
		float32 b = b2Dot(d1, d2);

		float32 angle = b2Atan2(a, b);

		b2Vec2 Jd1 = (-1.0f / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (1.0f / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		float32 lambda = m_bendingLambdas[i];

		float32 W = m1 * b2Dot(J1, J1) + m2 * b2Dot(J2, J2) + m3 * b2Dot(J3, J3);
		if (W == 0.0f)
		{
			continue;
		}

		float32 meff = 1.0f / W;

		// omega = 2 * pi * hz
		float32 omega = 2.0f * b2_pi * m_tuning.bendHertz;
		const float32 spring = meff * omega * omega;
		const float32 damper = 2.0f * meff * m_tuning.bendDamping * omega;

		const float32 alpha = 1.0f / (spring * dt * dt);
		const float32 beta = dt * dt * damper;
		float32 C = angle - m_as[i];
		while (C > b2_pi)
		{
			angle -= 2 * b2_pi;
			C = angle - m_as[i];
		}

		while (C < -b2_pi)
		{
			angle += 2.0f * b2_pi;
			C = angle - m_as[i];
		}

		float32 Cdot = b2Dot(J1, v1) + b2Dot(J2, v2) + b2Dot(J3, v3);
		float32 B = C + alpha * lambda + alpha * beta * Cdot;
		float32 Ws = (1.0f + alpha * beta / dt) * W + alpha;

		float32 impulse = -B / Ws;

		p1 += (m1 * impulse) * J1;
		p2 += (m2 * impulse) * J2;
		p3 += (m3 * impulse) * J3;
		lambda += impulse;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
		m_ps[i + 2] = p3;
		m_bendingLambdas[i] = lambda;
	}
}

void b2Rope::ApplyBendForces(float32 dt)
{
	int32 count3 = m_count - 2;
	const float32 stiffness = m_tuning.bendStiffness;
	const float32 damping = m_tuning.bendDamping;

	for (int32 i = 0; i < count3; ++i)
	{
		b2Vec2 p1 = m_ps[i];
		b2Vec2 p2 = m_ps[i + 1];
		b2Vec2 p3 = m_ps[i + 2];

		b2Vec2 v1 = m_vs[i];
		b2Vec2 v2 = m_vs[i + 1];
		b2Vec2 v3 = m_vs[i + 2];

		float32 m1 = m_ims[i];
		float32 m2 = m_ims[i + 1];
		float32 m3 = m_ims[i + 2];

		b2Vec2 d1 = p2 - p1;
		b2Vec2 d2 = p3 - p2;

		float32 L1sqr = d1.LengthSquared();
		float32 L2sqr = d2.LengthSquared();

		if (L1sqr * L2sqr == 0.0f)
		{
			continue;
		}

		float32 a = b2Cross(d1, d2);
		float32 b = b2Dot(d1, d2);

		float32 angle = b2Atan2(a, b);

		b2Vec2 Jd1 = (-1.0f / L1sqr) * d1.Skew();
		b2Vec2 Jd2 = (1.0f / L2sqr) * d2.Skew();

		b2Vec2 J1 = -Jd1;
		b2Vec2 J2 = Jd1 - Jd2;
		b2Vec2 J3 = Jd2;

		float32 W = m1 * b2Dot(J1, J1) + m2 * b2Dot(J2, J2) + m3 * b2Dot(J3, J3);
		if (W == 0.0f)
		{
			continue;
		}

		float32 meff = 1.0f / W;

		// omega = 2 * pi * hz
		float32 omega = 2.0f * b2_pi * m_tuning.bendHertz;
		const float32 spring = meff * omega * omega;
		const float32 damper = 2.0f * meff * m_tuning.bendDamping * omega;

		float32 C = angle - m_as[i];
		float32 Cdot = b2Dot(J1, v1) + b2Dot(J2, v2) + b2Dot(J3, v3);

		while (C > b2_pi)
		{
			angle -= 2 * b2_pi;
			C = angle - m_as[i];
		}

		while (C < -b2_pi)
		{
			angle += 2.0f * b2_pi;
			C = angle - m_as[i];
		}

		float32 impulse = -dt * (spring * C + damper * Cdot);

		m_vs[i + 0] += (m1 * impulse) * J1;
		m_vs[i + 1] += (m2 * impulse) * J2;
		m_vs[i + 2] += (m3 * impulse) * J3;
	}
}

void b2Rope::Draw(b2Draw* draw) const
{
	b2Color c(0.4f, 0.5f, 0.7f);
	b2Color pg(0.1f, 0.8f, 0.1f);
	b2Color pd(0.7f, 0.2f, 0.4f);

	for (int32 i = 0; i < m_count - 1; ++i)
	{
		draw->DrawSegment(m_ps[i], m_ps[i+1], c);

		const b2Color& pc = m_ims[i] > 0.0f ? pd : pg;
		draw->DrawPoint(m_ps[i], 5.0f, pc);
	}

	const b2Color& pc = m_ims[m_count - 1] > 0.0f ? pd : pg;
	draw->DrawPoint(m_ps[m_count - 1], 5.0f, pc);
}
