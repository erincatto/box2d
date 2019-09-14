
#ifndef B2_RENDERING_H
#define B2_RENDERING_H

#include "Box2D/Common/b2Math.h"
#include "Box2D/Common/b2Settings.h"

class b2Body;
class b2World;

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

struct b2Material
{
    b2Color color;
    float32 shininess = 0;
    float32 emit_intensity = 0;
    float32 reflect = 0;
};

struct b2LightDef
{
    b2Color color;
    float32 intensity = 0;
    b2Vec2 position{ 0, 0 };
    b2Body* body = nullptr;
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

#endif // B2_RENDERING_H
