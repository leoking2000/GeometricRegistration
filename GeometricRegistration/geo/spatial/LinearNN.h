#pragma once
#include <vector>
#include "INearestNeighbor.h"

namespace geo
{
	// used for testing
	class LinearNN : public INearestNeighbor
	{
	public:
		LinearNN(const std::vector<glm::vec3>& points);
	public:
		virtual glm::vec3 FindClosestPoint(const glm::vec3& query) const override;
		virtual float DistanceFromClosest(const glm::vec3& query) const override;
	private:
		void Search(const glm::vec3& query, glm::vec3& best, float& dist) const;
	private:
		const std::vector<glm::vec3>& m_points;
	};
}