#pragma once
#include <glm/glm.hpp>

namespace geo
{
    class INearestNeighbor
    {
    public:
        virtual ~INearestNeighbor() = default;
        virtual glm::vec3 FindClosestPoint(const glm::vec3& p) const = 0;
    };
}


