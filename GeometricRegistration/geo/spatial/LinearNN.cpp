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
		dist = std::numeric_limits<float>::max();

		for (const auto point : m_points)
		{
			float distance = glm::distance(point, query);
			if (dist > distance)
			{
				dist = distance;
				best = point;
			}
		}
	}

}