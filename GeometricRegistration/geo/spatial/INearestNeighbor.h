#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <utils/GeoTypes.h>

namespace geo
{
    // Indexes an external point array by reference.
    // The referenced points must outlive this object.
    class INearestNeighbor
    {
    public:
        // Returns the the nearest-neighbor index of input point.
        virtual index_t   Query(const glm::vec3& point) const = 0;
        // Writes the nearest-neighbor index for each input point.
        // Resizes results to points.size() if results has the wrong size.
        virtual void      QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const = 0;
        // The number of points in the external point array.
        virtual index_t   Size()  const = 0;
    public:
        virtual ~INearestNeighbor() = default;
    };
}
