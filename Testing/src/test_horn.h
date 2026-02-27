#pragma once
#include <gtest/gtest.h>
#include <geo/GeometricRegistration.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <cmath>

//RigidTransform ComputeRigidTransformHorn(const PointCloud3D& target, const PointCloud3D& source, unsigned int power_iterations)
//{
//    assert(source.Count() == target.Count());
//    assert(source.Count() >= 3);
//
//    const size_t n = source.Count();
//
//    // 1️. Compute centroids
//    glm::vec3 cS = source.Centroid();
//    glm::vec3 cT = target.Centroid();
//
//    // 2️. Build cross‑covariance H = Σ ( (s_i - cS) * (t_i - cT)^T )
//    glm::mat3 H(0.0f);
//    for (size_t i = 0; i < n; ++i)
//    {
//        glm::vec3 p = source[i] - cS;
//        glm::vec3 q = target[i] - cT;
//        H += glm::outerProduct(p, q); // p * q^T
//    }
//
//    // 3️. Construct the symmetric 4×4 Horn matrix N
//    float Nmat[4][4] = {};
//
//    float trace = H[0][0] + H[1][1] + H[2][2];
//
//    Nmat[0][0] = trace;
//    Nmat[0][1] = H[1][2] - H[2][1];
//    Nmat[0][2] = H[2][0] - H[0][2];
//    Nmat[0][3] = H[0][1] - H[1][0];
//
//    Nmat[1][0] = H[1][2] - H[2][1];
//    Nmat[1][1] = H[0][0] - H[1][1] - H[2][2];
//    Nmat[1][2] = H[0][1] + H[1][0];
//    Nmat[1][3] = H[0][2] + H[2][0];
//
//    Nmat[2][0] = H[2][0] - H[0][2];
//    Nmat[2][1] = H[0][1] + H[1][0];
//    Nmat[2][2] = -H[0][0] + H[1][1] - H[2][2];
//    Nmat[2][3] = H[1][2] + H[2][1];
//
//    Nmat[3][0] = H[0][1] - H[1][0];
//    Nmat[3][1] = H[0][2] + H[2][0];
//    Nmat[3][2] = H[1][2] + H[2][1];
//    Nmat[3][3] = -H[0][0] - H[1][1] + H[2][2];
//
//    // 4️. Power iteration to find largest eigenvector (unit quaternion)
//    glm::vec4 q(1, 0, 0, 0);
//
//    for (unsigned int i = 0; i < power_iterations; ++i)
//    {
//        glm::vec4 q_new;
//        for (int r = 0; r < 4; ++r)
//        {
//            float sum = 0.0f;
//            for (int c = 0; c < 4; ++c)
//                sum += Nmat[r][c] * q[c];
//            q_new[r] = sum;
//        }
//        q = glm::normalize(q_new);
//    }
//
//    // 5️. Convert quaternion to rotation matrix
//    float w = q[0];
//    float x = q[1];
//    float y = q[2];
//    float z = q[3];
//
//    glm::mat3 R;
//    R[0][0] = 1 - 2 * y * y - 2 * z * z;
//    R[0][1] = 2 * x * y - 2 * z * w;
//    R[0][2] = 2 * x * z + 2 * y * w;
//
//    R[1][0] = 2 * x * y + 2 * z * w;
//    R[1][1] = 1 - 2 * x * x - 2 * z * z;
//    R[1][2] = 2 * y * z - 2 * x * w;
//
//    R[2][0] = 2 * x * z - 2 * y * w;
//    R[2][1] = 2 * y * z + 2 * x * w;
//    R[2][2] = 1 - 2 * x * x - 2 * y * y;
//
//    // 6️. Compute translation
//    glm::vec3 t = cT - R * cS;
//
//    return { R, t };
//}

static constexpr float EPS = 1e-4f;

bool Mat3Near(const glm::mat3& A, const glm::mat3& B, float eps = EPS)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (std::abs(A[i][j] - B[i][j]) > eps)
                return false;
    return true;
}

bool Vec3Near(const glm::vec3& a, const glm::vec3& b, float eps = EPS)
{
    return glm::length(a - b) < eps;
}






