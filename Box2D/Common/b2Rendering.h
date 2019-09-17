
#ifndef B2_RENDERING_H
#define B2_RENDERING_H

#include "Box2D/Common/b2Math.h"
#include "Box2D/Common/b2Settings.h"

#include <limits>

class b2Body;
class b2World;

/// RGB color representation
/// (component values are expected in the [0, 1] range)
struct b2Color
{
    float32 r = 0;
    float32 g = 0;
    float32 b = 0;

    b2Color() = default;

    b2Color(float32 r, float32 g, float32 b) : r(r), g(g), b(b) {}

    b2Color operator+(const b2Color& c) const
    {
        return b2Color(r + c.r, g + c.g, b + c.b);
    }

    b2Color operator*(const b2Color& c) const
    {
        return b2Color(r * c.r, g * c.g, b * c.b);
    }

    b2Color operator+(float32 s) const { return b2Color(r + s, g + s, b + s); }

    b2Color operator*(float32 s) const { return b2Color(r * s, g * s, b * s); }
};

/// A basic fixture material
struct b2Material
{
    b2Color color;
    
    /// Controls specular lighting (>= 0)
    /// (0 = flat color, ie. no specular lighting)
    float32 shininess = 0;
    
    /// Emissive lighting intensity [0, 1]
    float32 emit_intensity = 0;
};

/// Light definition
struct b2LightDef
{
    b2Color color{ 1, 1, 1 };
    
    /// Light intensity (>= 0)
    float32 intensity = 1.0f;

    /// The illumination "range": the light intensity decreases linearly from full
    /// intensity down to zero at `attenuation_distance`. While not physically accurate,
    /// it's an easy way to control the lighting results and define area lights.
    float32 attenuation_distance = std::numeric_limits<float>::infinity();

    /// Parent body
    b2Body* body = nullptr;

    /// Position relative to the parent body
    b2Vec2 position{ 0, 0 };
};

class b2Light
{
    friend class b2World;

public:
    const b2LightDef& GetDef() const { return m_def; }

    b2World* GetWorld() { return m_world; }
    const b2World* GetWorld() const { return m_world; }

    b2Light* GetNext() { return m_next; }
    const b2Light* GetNext() const { return m_next; }

private:
    b2Light(const b2LightDef* def, b2World* world) : m_def(*def), m_world(world) {}

private:
    b2LightDef m_def;
    b2World* m_world = nullptr;
    b2Light* m_prev = nullptr;
    b2Light* m_next = nullptr;
};

#endif  // B2_RENDERING_H
