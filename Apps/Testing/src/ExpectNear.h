#pragma once
#include <glm/glm.hpp>

static inline void ExpectMat3Near(
    const glm::mat3& A,
    const glm::mat3& B,
    float tolerance = 1e-5f)
{
    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            EXPECT_NEAR(A[c][r], B[c][r], tolerance) << "Mismatch at (" << r << ", " << c << ")";
        }
    }
}

static inline void ExpectVec3Near(
    const glm::vec3& a,
    const glm::vec3& b,
    float tolerance = 1e-5f)
{
    EXPECT_NEAR(a.x, b.x, tolerance);
    EXPECT_NEAR(a.y, b.y, tolerance);
    EXPECT_NEAR(a.z, b.z, tolerance);
}
