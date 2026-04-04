#pragma once
#include <vector>
#include <functional>
#include "utils/GeoTypes.h"

namespace geo
{
    f32 Mean(const std::vector<f32>& values);
    f32 Median(std::vector<f32> values);
    f32 Percentile(std::vector<f32> values, f32 alpha); // What value is larger than (alpha)% of the data?
    
    f32 RMSE(const std::vector<f32>& residuals); // Root Mean Square Error, Note: it takes the mean of residuals[i]^2
    f32 TrimmedRMSE(std::vector<f32> residuals, f32 keepRatio); // Root Mean Square Error, Note: it takes the mean of residuals[i]^2

	// Root Mean Square Error
	template<typename T>
	f32 RMSE(const std::vector<T>& values, const std::vector<T>& targets, std::function<f32(T, T)> errorSq)
	{
		assert(values.size() == targets.size());

		const size_t N = std::min(values.size(), targets.size());
		if (N == 0) return 0.0f;

		f32 sumSq = 0.0f;
		for (size_t i = 0; i < N; i++)
		{
			sumSq += errorSq(values[i], targets[i]);
		}

		return std::sqrtf(sumSq / (f32)N);
	}

}