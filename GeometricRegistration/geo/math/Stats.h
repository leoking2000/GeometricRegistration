#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "utils/GeoTypes.h"

namespace geo
{
    // Computes RMSE over pre-established point correspondences,
    // we assume correspondences are already established by index.
    // Returns F32_MAX when the input is empty.
    f32 PointToPointRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target);

    // Computes point-to-plane RMSE over pre-established correspondences,
    // we assume correspondences are already established by index,
    // using the normal associated with each target point.
    // Returns F32_MAX when the input is empty.
    f32 PointToPlaneRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target, const std::vector<glm::vec3>& normals);
}
