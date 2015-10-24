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
#ifndef B2_PARTICLE
#define B2_PARTICLE

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2IntrusiveList.h>

struct b2Color;
class b2ParticleGroup;

/// @file

/// The particle type. Can be combined with the | operator.
enum b2ParticleFlag
{
	/// Water particle.
	b2_waterParticle = 0,
	/// Removed after next simulation step.
	b2_zombieParticle = 1 << 1,
	/// Zero velocity.
	b2_wallParticle = 1 << 2,
	/// With restitution from stretching.
	b2_springParticle = 1 << 3,
	/// With restitution from deformation.
	b2_elasticParticle = 1 << 4,
	/// With viscosity.
	b2_viscousParticle = 1 << 5,
	/// Without isotropic pressure.
	b2_powderParticle = 1 << 6,
	/// With surface tension.
	b2_tensileParticle = 1 << 7,
	/// Mix color between contacting particles.
	b2_colorMixingParticle = 1 << 8,
	/// Call b2DestructionListener on destruction.
	b2_destructionListenerParticle = 1 << 9,
	/// Prevents other particles from leaking.
	b2_barrierParticle = 1 << 10,
	/// Less compressibility.
	b2_staticPressureParticle = 1 << 11,
	/// Makes pairs or triads with other particles.
	b2_reactiveParticle = 1 << 12,
	/// With high repulsive force.
	b2_repulsiveParticle = 1 << 13,
	/// Call b2ContactListener when this particle is about to interact with
	/// a rigid body or stops interacting with a rigid body.
	/// This results in an expensive operation compared to using
	/// b2_fixtureContactFilterParticle to detect collisions between
	/// particles.
	b2_fixtureContactListenerParticle = 1 << 14,
	/// Call b2ContactListener when this particle is about to interact with
	/// another particle or stops interacting with another particle.
	/// This results in an expensive operation compared to using
	/// b2_particleContactFilterParticle to detect collisions between
	/// particles.
	b2_particleContactListenerParticle = 1 << 15,
	/// Call b2ContactFilter when this particle interacts with rigid bodies.
	b2_fixtureContactFilterParticle = 1 << 16,
	/// Call b2ContactFilter when this particle interacts with other
	/// particles.
	b2_particleContactFilterParticle = 1 << 17,
};

/// Small color object for each particle
class b2ParticleColor
{
public:
	b2ParticleColor() {}
	/// Constructor with four elements: r (red), g (green), b (blue), and a
	/// (opacity).
	/// Each element can be specified 0 to 255.
	b2Inline b2ParticleColor(uint8 r, uint8 g, uint8 b, uint8 a)
	{
		Set(r, g, b, a);
	}

	/// Constructor that initializes the above four elements with the value of
	/// the b2Color object.
	b2ParticleColor(const b2Color& color);

	/// True when all four color elements equal 0. When true, a particle color
	/// buffer isn't allocated by CreateParticle().
	///
	bool IsZero() const
	{
		return !r && !g && !b && !a;
	}

	/// Used internally to convert the value of b2Color.
	///
	b2Color GetColor() const;

	/// Sets color for current object using the four elements described above.
	///
	b2Inline void Set(uint8 r_, uint8 g_, uint8 b_, uint8 a_)
	{
		r = r_;
		g = g_;
		b = b_;
		a = a_;
	}

	/// Initializes the object with the value of the b2Color.
	///
	void Set(const b2Color& color);

	/// Assign a b2ParticleColor to this instance.
	b2ParticleColor& operator = (const b2ParticleColor &color)
	{
		Set(color.r, color.g, color.b, color.a);
		return *this;
	}

	/// Multiplies r, g, b, a members by s where s is a value between 0.0
	/// and 1.0.
	b2ParticleColor& operator *= (float32 s)
	{
		Set((uint8)(r * s), (uint8)(g * s), (uint8)(b * s), (uint8)(a * s));
		return *this;
	}

	/// Scales r, g, b, a members by s where s is a value between 0 and 255.
	b2ParticleColor& operator *= (uint8 s)
	{
		// 1..256 to maintain the complete dynamic range.
		const int32 scale = (int32)s + 1;
		Set((uint8)(((int32)r * scale) >> k_bitsPerComponent),
			(uint8)(((int32)g * scale) >> k_bitsPerComponent),
			(uint8)(((int32)b * scale) >> k_bitsPerComponent),
			(uint8)(((int32)a * scale) >> k_bitsPerComponent));
		return *this;
	}

	/// Scales r, g, b, a members by s returning the modified b2ParticleColor.
	b2ParticleColor operator * (float32 s) const
	{
		return MultiplyByScalar(s);
	}

	/// Scales r, g, b, a members by s returning the modified b2ParticleColor.
	b2ParticleColor operator * (uint8 s) const
	{
		return MultiplyByScalar(s);
	}

	/// Add two colors.  This is a non-saturating addition so values
	/// overflows will wrap.
	b2Inline b2ParticleColor& operator += (const b2ParticleColor &color)
	{
		r += color.r;
		g += color.g;
		b += color.b;
		a += color.a;
		return *this;
	}

	/// Add two colors.  This is a non-saturating addition so values
	/// overflows will wrap.
	b2ParticleColor operator + (const b2ParticleColor &color) const
	{
		b2ParticleColor newColor(*this);
		newColor += color;
		return newColor;
	}

	/// Subtract a color from this color.  This is a subtraction without
	/// saturation so underflows will wrap.
	b2Inline b2ParticleColor& operator -= (const b2ParticleColor &color)
	{
		r -= color.r;
		g -= color.g;
		b -= color.b;
		a -= color.a;
		return *this;
	}

	/// Subtract a color from this color returning the result.  This is a
	/// subtraction without saturation so underflows will wrap.
	b2ParticleColor operator - (const b2ParticleColor &color) const
	{
		b2ParticleColor newColor(*this);
		newColor -= color;
		return newColor;
	}

	/// Compare this color with the specified color.
	bool operator == (const b2ParticleColor &color) const
	{
		return r == color.r && g == color.g && b == color.b && a == color.a;
	}

	/// Mix mixColor with this color using strength to control how much of
	/// mixColor is mixed with this color and vice versa.  The range of
	/// strength is 0..128 where 0 results in no color mixing and 128 results
	/// in an equal mix of both colors.  strength 0..128 is analogous to an
	/// alpha channel value between 0.0f..0.5f.
	b2Inline void Mix(b2ParticleColor * const mixColor, const int32 strength)
	{
		MixColors(this, mixColor, strength);
	}

	/// Mix colorA with colorB using strength to control how much of
	/// colorA is mixed with colorB and vice versa.  The range of
	/// strength is 0..128 where 0 results in no color mixing and 128 results
	/// in an equal mix of both colors.  strength 0..128 is analogous to an
	/// alpha channel value between 0.0f..0.5f.
	static b2Inline void MixColors(b2ParticleColor * const colorA,
							 b2ParticleColor * const colorB,
							 const int32 strength)
	{
		const uint8 dr = (uint8)((strength * (colorB->r - colorA->r)) >>
								 k_bitsPerComponent);
		const uint8 dg = (uint8)((strength * (colorB->g - colorA->g)) >>
								 k_bitsPerComponent);
		const uint8 db = (uint8)((strength * (colorB->b - colorA->b)) >>
								 k_bitsPerComponent);
		const uint8 da = (uint8)((strength * (colorB->a - colorA->a)) >>
								 k_bitsPerComponent);
		colorA->r += dr;
		colorA->g += dg;
		colorA->b += db;
		colorA->a += da;
		colorB->r -= dr;
		colorB->g -= dg;
		colorB->b -= db;
		colorB->a -= da;
	}

private:
	/// Generalization of the multiply operator using a scalar in-place
	/// multiplication.
	template <typename T>
	b2ParticleColor MultiplyByScalar(T s) const
	{
		b2ParticleColor color(*this);
		color *= s;
		return color;
	}

public:
	uint8 r, g, b, a;

protected:
	/// Maximum value of a b2ParticleColor component.
	static const float32 k_maxValue;
	/// 1.0 / k_maxValue.
	static const float32 k_inverseMaxValue;
	/// Number of bits used to store each b2ParticleColor component.
	static const uint8 k_bitsPerComponent;
};

extern b2ParticleColor b2ParticleColor_zero;

/// A particle definition holds all the data needed to construct a particle.
/// You can safely re-use these definitions.
struct b2ParticleDef
{
	b2ParticleDef()
	{
		flags = 0;
		position = b2Vec2_zero;
		velocity = b2Vec2_zero;
		color = b2ParticleColor_zero;
		lifetime = 0.0f;
		userData = NULL;
		group = NULL;
	}

#if BOX2D_EXTERNAL_LANGUAGE_API
	/// Set position with direct floats
	void SetPosition(float32 x, float32 y);

	/// Set color with direct ints.
	void SetColor(int32 r, int32 g, int32 b, int32 a);
#endif // BOX2D_EXTERNAL_LANGUAGE_API

	/// \brief Specifies the type of particle (see #b2ParticleFlag).
	///
	/// A particle may be more than one type.
	/// Multiple types are chained by logical sums, for example:
	/// pd.flags = b2_elasticParticle | b2_viscousParticle
	uint32 flags;

	/// The world position of the particle.
	b2Vec2 position;

	/// The linear velocity of the particle in world co-ordinates.
	b2Vec2 velocity;

	/// The color of the particle.
	b2ParticleColor color;

	/// Lifetime of the particle in seconds.  A value <= 0.0f indicates a
	/// particle with infinite lifetime.
	float32 lifetime;

	/// Use this to store application-specific body data.
	void* userData;

	/// An existing particle group to which the particle will be added.
	b2ParticleGroup* group;

};

/// A helper function to calculate the optimal number of iterations.
int32 b2CalculateParticleIterations(
	float32 gravity, float32 radius, float32 timeStep);

/// Handle to a particle. Particle indices are ephemeral: the same index might
/// refer to a different particle, from frame-to-frame. If you need to keep a
/// reference to a particular particle across frames, you should acquire a
/// b2ParticleHandle. Use #b2ParticleSystem::GetParticleHandleFromIndex() to
/// retrieve the b2ParticleHandle of a particle from the particle system.
class b2ParticleHandle : public b2TypedIntrusiveListNode<b2ParticleHandle>
{
	// Allow b2ParticleSystem to use SetIndex() to associate particle handles
	// with particle indices.
	friend class b2ParticleSystem;

public:
	/// Initialize the index associated with the handle to an invalid index.
	b2ParticleHandle() : m_index(b2_invalidParticleIndex) { }
	/// Empty destructor.
	~b2ParticleHandle() { }

	/// Get the index of the particle associated with this handle.
	int32 GetIndex() const { return m_index; }

private:
	/// Set the index of the particle associated with this handle.
	void SetIndex(int32 index) { m_index = index; }

private:
	// Index of the particle within the particle system.
	int32 m_index;
};

#if BOX2D_EXTERNAL_LANGUAGE_API
inline void b2ParticleDef::SetPosition(float32 x, float32 y)
{
	position.Set(x, y);
}

inline void b2ParticleDef::SetColor(int32 r, int32 g, int32 b, int32 a)
{
	color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
}
#endif // BOX2D_EXTERNAL_LANGUAGE_API

#endif
