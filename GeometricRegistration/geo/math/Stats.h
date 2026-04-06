#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <functional>
#include "utils/GeoTypes.h"

namespace geo
{
    // Returns the arithmetic mean of the input values.
    // Empty-input behavior: Returns F32_MAX if values is empty.
    f32 Mean(const std::vector<f32>& values);

    // Returns the median of the input values.
    // Empty-input behavior: Returns F32_MAX if values is empty.
    // Notes:
    // - Sorts a local copy of the input.
    // - For an even number of samples, returns the midpoint of the two central values.
    f32 Median(std::vector<f32> values);

    // Returns the quantile at p using linear interpolation between adjacent sorted samples.
    // Empty-input behavior: Returns F32_MAX if values is empty.
    // Parameter handling: p is clamped to [0, 1].
    // Notes:
    // - Sorts a local copy of the input.
    // - p = 0 returns the minimum value.
    // - p = 0.5 returns the median.
    // - p = 1 returns the maximum value.
    f32 Percentile(std::vector<f32> values, f32 p);

    // Returns the root mean square error of scalar residual magnitudes.
    // Expected use: errorSq gets the residual of sample i squared.
    f32 RMSE(index_t N, std::function<f32(index_t)> errorSq);

    // Returns the RMSE after keeping the smallest keepRatio fraction of the scalar residual magnitudes.
    // Expected use: residuals are squared scalar magnitudes.
    // Empty-input behavior: Returns F32_MAX if residuals is empty.
    // Parameter handling: keepRatio is clamped to [0, 1].
    // Notes:
    // - The function keeps the smallest residuals after sorting.
    // - At least one residual is kept when the input is non-empty.
    // - keepRatio = 1 uses all residuals.
    f32 TrimmedRMSE(std::vector<f32> residuals, f32 keepRatio);
}
