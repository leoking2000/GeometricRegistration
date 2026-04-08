#include "Stats.h"
#include <cassert>
#include <cmath>

namespace geo
{
	f32 PointToPointRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target)
	{
		assert(source.size() == target.size());

		const size_t N = source.size();
		if (N == 0) return F32_MAX;

		f64 sumSq = 0.0f;
		for (size_t i = 0; i < N; i++)
		{
			glm::vec3 diff = source[i] - target[i];
			sumSq += glm::dot(diff, diff);
		}

		return (f32)std::sqrt(sumSq / (f64)N);
	}

	f32 PointToPlaneRMSE(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target, const std::vector<glm::vec3>& normals)
	{
		assert(source.size() == target.size());
		assert(target.size() == normals.size());

		const size_t N = source.size();
		if (N == 0) return F32_MAX;

		f64 sumSq = 0.0f;
		for (size_t i = 0; i < N; i++)
		{
			glm::vec3 diff = source[i] - target[i];
			const f64 r = (f64)glm::dot(diff, normals[i]);
			sumSq += r * r;
		}

		return (f32)std::sqrt(sumSq / (f64)N);
	}
}

