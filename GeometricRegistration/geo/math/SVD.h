#pragma once
#include <glm/glm.hpp>

namespace geo
{
    struct SVDResult
    {
        glm::mat3 U;
        glm::vec3 S;
        glm::mat3 V;
    };

    // Compute SVD of a 3x3 matrix: A = U * diag(S) * V^T
    SVDResult SVD(const glm::mat3& A);
}
