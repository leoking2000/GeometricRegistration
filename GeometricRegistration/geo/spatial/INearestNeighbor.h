#pragma once
#include <glm/glm.hpp>

namespace geo
{
    class INearestNeighbor
    {
    public:
        virtual glm::vec3 FindClosestPoint(const glm::vec3& p) const = 0;
        virtual float DistanceFromClosest(const glm::vec3& p) const = 0;
    public:
        virtual ~INearestNeighbor() = default;
    };
}


