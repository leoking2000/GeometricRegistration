#include <limits>
#include "LinearNN.h"


namespace geo
{

	LinearNN::LinearNN(const std::vector<glm::vec3>& points)
		:
		m_points(points)
	{
	}

	glm::vec3 LinearNN::FindClosestPoint(const glm::vec3& query) const
	{
		glm::vec3 best;
		float dist;

		Search(query, best, dist);

		return best;
	}

	float LinearNN::DistanceFromClosest(const glm::vec3& query) const
	{
		glm::vec3 best;
		float dist;

		Search(query, best, dist);

		return dist;
	}

	void LinearNN::Search(const glm::vec3& query, glm::vec3& best, float& dist) const
	{
		assert(!m_points.empty());

		float bestDistSq = std::numeric_limits<float>::max();

		for (const auto& point : m_points)
		{
			float d = glm::dot(point - query, point - query);

			if (d < bestDistSq)
			{
				bestDistSq = d;
				best = point;
			}
		}

		dist = glm::sqrt(bestDistSq);
	}

}