#include <cassert>
#include <glm/gtc/constants.hpp>
#include "GeoRand.h"

namespace geo
{
    Random::Random()
        : m_Rng(std::random_device{}())
    {
    }

    Random::Random(u32 seed)
        : m_Rng(seed)
    {
    }

    void Random::SetSeed(u32 seed)
    {
        m_Rng.seed(seed);
    }

    i32 Random::Int(i32 min, i32 max)
    {
        assert(min <= max);

        std::uniform_int_distribution<i32> dist(min, max);
        return dist(m_Rng);
    }

    u32 Random::UInt(u32 min, u32 max)
    {
        assert(min <= max);

        std::uniform_int_distribution<u32> dist(min, max);
        return dist(m_Rng);
    }

    f32 Random::Float(f32 min, f32 max)
    {
        assert(min <= max);

        std::uniform_real_distribution<f32> dist(min, max);
        return dist(m_Rng);
    }

    glm::vec2 Random::Float2(f32 min, f32 max)
    {
        return { Float(min, max), Float(min, max) };
    }

    glm::vec3 Random::Float3(f32 min, f32 max)
    {
        return { Float(min, max), Float(min, max), Float(min, max) };
    }

    glm::vec2 Random::Dir2D(f32 length)
    {
        f32 angle = Float(0.0f, glm::two_pi<f32>());
        return {
            glm::cos(angle) * length,
            glm::sin(angle) * length
        };
    }

    glm::vec3 Random::Dir3D(f32 length)
    {
        // Uniform spherical distribution
        f32 z = Float(-1.0f, 1.0f);
        f32 theta = Float(0.0f, glm::two_pi<f32>());

        f32 r = glm::sqrt(1.0f - z * z);

        return {
            r * glm::cos(theta) * length,
            r * glm::sin(theta) * length,
            z * length
        };
    }
}