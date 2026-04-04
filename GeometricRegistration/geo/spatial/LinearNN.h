#pragma once
#include <vector>
#include <utils/GeoTypes.h>
#include "INearestNeighbor.h"

namespace geo
{
	// used for testing
	class LinearNN : public INearestNeighbor
	{
	public:
		LinearNN(const std::vector<glm::vec3>& points);
	public:
		virtual void      Build() override;
		virtual index_t   Query(const glm::vec3& point, f32* distSq = nullptr) const override;
		virtual void      QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const override;
		virtual bool      Empty() const override;
		virtual size_t    Size()  const override;
		virtual glm::vec3 FindClosestPoint(const glm::vec3& point) const override;
	private:
		index_t Search(const glm::vec3& query, f32& distSq) const;
	private:
		const std::vector<glm::vec3>& m_points;
	};
}