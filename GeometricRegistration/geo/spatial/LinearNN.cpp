#include <cassert>
#include "LinearNN.h"


namespace geo
{

	LinearNN::LinearNN(const std::vector<glm::vec3>& points)
		:
		m_points(points)
	{
	}

	void LinearNN::Build()
	{
	}

	index_t LinearNN::Query(const glm::vec3& point, f32* distSq) const
	{
		f32 distSqBest;
		index_t best = Search(point, distSqBest);

		if (distSq != nullptr) {
			*distSq = distSqBest;
		}

		return best;
	}

	void LinearNN::QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const
	{
		if (results.size() != points.size())
		{
			results.resize(points.size());
		}

		// This is slow but is only for testing, no point to have multithreding here
		for (size_t i = 0; i < points.size(); i++)
		{
			results[i] = Query(points[i]);
		}
	}

	bool LinearNN::Empty() const
	{
		return m_points.empty();
	}

	size_t LinearNN::Size() const
	{
		return m_points.size();
	}

	glm::vec3 LinearNN::FindClosestPoint(const glm::vec3& point) const
	{
		f32 distSqBest;
		index_t best = Search(point, distSqBest);

		return m_points[best];
	}

	index_t LinearNN::Search(const glm::vec3& query, f32& distSq) const
	{
		assert(!m_points.empty());

		f32 bestDistSq = F32_MAX;
		index_t best = 0;

		// do linear search
		for (index_t i = 0; i < (index_t)m_points.size(); i++)
		{
			f32 d2 = glm::dot(m_points[i] - query, m_points[i] - query);

			if (d2 < bestDistSq)
			{
				bestDistSq = d2;
				best = i;
			}
		}

		distSq = bestDistSq;
		return best;
	}

}