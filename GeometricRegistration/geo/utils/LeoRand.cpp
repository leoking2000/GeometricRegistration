#include <cassert>
#include <glm/gtc/constants.hpp>
#include "LeoRand.h"

namespace geo
{
    Random::Random()
        : m_Rng(std::random_device{}())
    {
    }

    Random::Random(unsigned int seed)
        : m_Rng(seed)
    {
    }

    void Random::SetSeed(unsigned int seed)
    {
        m_Rng.seed(seed);
    }

    int Random::Int(int min, int max)
    {
        assert(min <= max);

        std::uniform_int_distribution<int> dist(min, max);
        return dist(m_Rng);
    }

    unsigned int Random::UInt(unsigned int min, unsigned int max)
    {
        assert(min <= max);

        std::uniform_int_distribution<unsigned int> dist(min, max);
        return dist(m_Rng);
    }

    float Random::Float(float min, float max)
    {
        assert(min <= max);

        std::uniform_real_distribution<float> dist(min, max);
        return dist(m_Rng);
    }

    glm::vec2 Random::Float2(float min, float max)
    {
        return { Float(min, max), Float(min, max) };
    }

    glm::vec3 Random::Float3(float min, float max)
    {
        return { Float(min, max), Float(min, max), Float(min, max) };
    }

    glm::vec2 Random::Dir2D(float length)
    {
        float angle = Float(0.0f, glm::two_pi<float>());
        return {
            glm::cos(angle) * length,
            glm::sin(angle) * length
        };
    }

    glm::vec3 Random::Dir3D(float length)
    {
        // Uniform spherical distribution
        float z = Float(-1.0f, 1.0f);
        float theta = Float(0.0f, glm::two_pi<float>());

        float r = glm::sqrt(1.0f - z * z);

        return {
            r * glm::cos(theta) * length,
            r * glm::sin(theta) * length,
            z * length
        };
    }
}