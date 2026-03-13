#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <core/GeoTypes.h>

namespace geo
{
    class INearestNeighbor
    {
    public:
        virtual void      Build() = 0;
    public:
        virtual index_t   Query(const glm::vec3& point, f32* distSq = nullptr) const = 0;
        virtual void      QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const = 0;
    public:
        virtual bool      Empty() const = 0;
        virtual size_t    Size()  const = 0;
    public:
        virtual glm::vec3 FindClosestPoint(const glm::vec3& point) const = 0;
    public:
        virtual ~INearestNeighbor() = default;
    };
}
