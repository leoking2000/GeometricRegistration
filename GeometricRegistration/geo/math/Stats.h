#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "utils/GeoTypes.h"

namespace geo
{
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
