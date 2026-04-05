#pragma once
#include "INearestNeighbor.h"

namespace geo
{
	// Reference implementation for testing and validation.
	// Simplicity and correctness are prioritized over performance.
	class LinearNN : public INearestNeighbor
	{
	public:
		LinearNN(const std::vector<glm::vec3>& points);
	public:
		index_t    Query(const glm::vec3& point) const override;
		void       QueryBatch(const std::vector<glm::vec3>& points, std::vector<index_t>& results) const override;
		index_t    Size()  const override;
	private:
		const std::vector<glm::vec3>& m_points;
	};
}