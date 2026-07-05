#pragma once
#include <glm/glm.hpp>
#include <array>
#include <vector>
#include <geo/GeoTypes.h>
#include "RigidTransform.h"

namespace geo
{
    using Vec6 = std::array<f32, 6>;
    using Mat6 = std::array<std::array<f32, 6>, 6>;

    // Solves a symmetric positive-definite 6x6 linear system using Eigen LDLT decomposition.
    // Returns zero vector if decomposition fails or matrix is not positive definite.
    Vec6 SolveSymmetric6x6(const Mat6& A_in, const Vec6& b_in);

    struct SVDResult
    {
        glm::mat3 U;
        glm::vec3 S;
        glm::mat3 V;
    };

    // Computes Singular Value Decomposition (SVD) of a 3x3 matrix.
    //
    // Uses Eigen::JacobiSVD with full U and V decomposition.
    // Returned decomposition satisfies: A = U * S * V^T
    SVDResult SVD(const glm::mat3& A);

    // Solves rigid transform (rotation + translation) using point-to-point alignment.
    // Solves min_{R,t} sum_i ||R x_i + t - y_i||^2
    //
    // Method:
    // - Computes centroids
    // - Builds covariance matrix H
    // - Uses SVD to extract optimal rotation (Kabsch algorithm)
    // - Computes translation from centroid alignment
    RigidTransform SolveRigidPointToPoint(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target);

    // Solves weighted point-to-point rigid alignment.
    // Solves min_{R,t} sum_i w_i ||R x_i + t - y_i||^2
    //
    // Extends Kabsch algorithm by incorporating per-point weights:
    // - weighted centroids
    // - weighted covariance matrix
    RigidTransform SolveRigidPointToPointWeighted(const std::vector<glm::vec3>& source,const std::vector<glm::vec3>& target,
        const std::vector<f32>& weights);

    // Solves rigid alignment using point-to-plane error minimization.
    // residual_i = dot(n_i, (R x_i + t - y_i)
    //
    // Linearized ICP formulation:
    // - Builds 6x6 normal equations system
    // - Solves for small rotation (axis-angle) + translation
    RigidTransform SolveRigidPointToPlane(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target,
        const std::vector<glm::vec3>& normals);

    // Solves point-to-plane alignment with an additional per-point offset term.
    // residual_i = (dot(n_i, (R x_i + t - y_i) - c_i) where c_i is the offset => linearized RHS: dot(n, q - p) + c
    // Extension of linear ICP where each correspondence has bias term c.
    RigidTransform SolveRigidPointToPlaneShifted(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target,
        const std::vector<glm::vec3>& normals, const std::vector<f32>& offsets);

    // Computes Root Mean Squared Error (RMSE) for point-to-point correspondences.
    //
    // Assumptions:
    // - source[i] corresponds to target[i] (1-to-1 index mapping)
    // - both arrays must have identical size
    //
    // Returns:
    // - RMSE value in world units
    // - F32_MAX if inputs are empty or sizes mismatch
    f32 PointToPointRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target);

    // Computes point-to-plane RMSE for pre-established correspondences.
    //
    // Error metric:
    // - projects source-target residual onto target normal
    // - measures squared distance along surface normal direction
    //
    // Assumptions:
    // - source[i] corresponds to target[i]
    // - normals[i] is the surface normal at target[i]
    // - all vectors must have identical size
    //
    // Returns:
    // - RMSE value in world units
    // - F32_MAX if inputs are empty or sizes mismatch
    f32 PointToPlaneRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target, const std::vector<glm::vec3>& normals);
}

