#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <geo/utils/GeoTypes.h>

namespace geo
{
    // Abstract interface for nearest-neighbor search structures.
    //
    // This class operates over an *externally owned* point set:
    // - It does NOT store or own the point data.
    // - It assumes the referenced point array remains valid for the lifetime
    //   of the implementation.
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
