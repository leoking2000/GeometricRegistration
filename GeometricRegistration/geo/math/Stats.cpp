#include "Stats.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

namespace geo
{
	f32 Mean(const std::vector<f32>& values)
	{
		const size_t N = values.size();

		if (N == 0) return 0.0f;

		f64 sum = 0.0f;
		for (size_t i = 0; i < N; i++)
		{
			sum += (f64)values[i];
		}

		return f32(sum / (f64)N);
	}

	f32 Median(std::vector<f32> values)
	{
		const size_t N = values.size();

		if (N == 0) return 0.0f;

		std::sort(values.begin(), values.end());

		const size_t mid = N / 2;

		return ((N % 2) == 1) ? values[mid] : 0.5f * (values[mid - 1] + values[mid]);
	}

	f32 Percentile(std::vector<f32> values, f32 alpha)
	{
		const size_t N = values.size();
		if (N == 0) return 0.0f;

		alpha = std::clamp(alpha, 0.0f, 1.0f);
		std::sort(values.begin(), values.end());

		const f32    pos = alpha * (f32)N;
		const size_t lo  = (size_t)(std::floor(pos));
		const size_t hi  = (size_t)(std::ceil(pos));

		if (lo == hi){
			return values[lo];
		}

		const f32 t = pos - (f32)lo;
		return (1.0f - t) * values[lo] + t * values[hi];
	}

	f32 RMSE(const std::vector<f32>& residuals)
	{
		const size_t N = residuals.size();
		if (N == 0) return 0.0f;

		f64 sumSq = 0.0f;
		for (size_t i = 0; i < N; i++)
		{
			sumSq += (f64)residuals[i] * (f64)residuals[i];
		}

		return (f32)std::sqrt(sumSq / (f64)N);
	}

	f32 TrimmedRMSE(std::vector<f32> residuals, f32 keepRatio)
	{
		const size_t N = residuals.size();
		if (N == 0) return 0.0f;

		keepRatio = std::clamp(keepRatio, 0.0f, 1.0f);
		std::sort(residuals.begin(), residuals.end());

		size_t keepCount = (size_t)std::floor(keepRatio * (f32)N);
		keepCount = std::max<size_t>(1, keepCount);

		f64 sumSq = 0.0f;
		for (size_t i = 0; i < keepCount; i++)
		{
			sumSq += (f64)residuals[i] * (f64)residuals[i];
		}

		return (f32)std::sqrt(sumSq / (f64)N);
	}

}

