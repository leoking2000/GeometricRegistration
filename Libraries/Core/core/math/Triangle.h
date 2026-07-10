#include <glm/glm.hpp>
#include "Types.h"

namespace core
{
    // Returns unit normal of triangle (a,b,c)
    // Right-hand rule: (b - a) x (c - a)
    glm::vec3 TriangleNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    // Triangle area = 0.5 * || (b-a) x (c-a) ||
    f32       TriangleArea(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    // Centroid (uniform area triangle)
    glm::vec3 TriangleCentroid(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);

    // Computes the closest point on a triangle (A, B, C) to point p.
    //
    // Outputs:
    // - out_closestPoint : closest point on triangle surface
    // - out_distance     : Euclidean distance to p
    //
    // Returns:
    // - true if a valid closest point was computed (within maxDist)
    bool closestPointToTriangle(
        glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p,
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& faceNormal,
        f32 maxDist = F32_MAX);
}
