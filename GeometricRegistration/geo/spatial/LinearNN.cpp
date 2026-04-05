#include <cassert>
#include "LinearNN.h"

namespace geo
{
	LinearNN::LinearNN(const std::vector<glm::vec3>& points)
		:
		m_points(points)
	{
		assert(!m_points.empty());
	}

	index_t LinearNN::Query(const glm::vec3& point) const
	{
		assert(!m_points.empty());

		f32 bestDistSq = F32_MAX;
		index_t best = 0;

		// do linear search
		for (index_t i = 0; i < (index_t)m_points.size(); i++)
		{
			glm::vec3 diff = m_points[i] - point;
			f32 d2 = glm::dot(diff, diff);

			if (d2 < bestDistSq)
			{
				bestDistSq = d2;
				best = i;
			}
		}

		return best;
	}

	void LinearNN::QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const
	{
		if (results.size() != points.size())
		{
			results.resize(points.size(), 0);
		}

		// This is slow but is only for testing, no point to have multithreading here
		for (size_t i = 0; i < points.size(); i++)
		{
			results[i] = Query(points[i]);
		}
	}

	index_t LinearNN::Size() const
	{
		return (index_t)m_points.size();
	}
}
