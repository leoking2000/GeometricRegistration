#pragma once
#include <glm/glm.hpp>
#include <array>
#include <vector>
#include "utils/GeoTypes.h"
#include "RigidTransform.h"

namespace geo
{
    using Vec6 = std::array<f32, 6>;
    using Mat6 = std::array<std::array<f32, 6>, 6>;

    // solves A*x=b system that is 6x6
    Vec6 Solve6x6(Mat6 A_in, Vec6 b_in);

    struct SVDResult
    {
        glm::mat3 U;
        glm::vec3 S;
        glm::mat3 V;
    };

    // Compute SVD of a 3x3 matrix: A = U * diag(S) * V^T
    SVDResult SVD(const glm::mat3& A);

    // Solves min_{R,t} sum_i ||R x_i + t - y_i||^2
    RigidTransform SolveRigidPointToPoint(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target);

    // Solves min_{R,t} sum_i w_i ||R x_i + t - y_i||^2
    RigidTransform SolveRigidPointToPointWeighted(const std::vector<glm::vec3>& source,const std::vector<glm::vec3>& target,
        const std::vector<f32>& weights);

    // Solves linearized point-to-plane least squares using target normals
    // residual_i = dot(n_i, (R x_i + t - y_i)
    RigidTransform SolveRigidPointToPlane(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target,
        const std::vector<glm::vec3>& normals);

    // Solves linearized point-to-plane least squares using target normals
    // residual_i = (dot(n_i, (R x_i + t - y_i) - c_i) where c_i is the offset
    RigidTransform SolveRigidPointToPlaneShifted(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target,
        const std::vector<glm::vec3>& normals, const std::vector<f32>& offsets);
}

