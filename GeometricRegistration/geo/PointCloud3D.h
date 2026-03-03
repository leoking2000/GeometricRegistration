#pragma once
#include "INearestNeighbor.h"
#include <vector>

namespace geo
{
	// A Point cloud of 3D points, uses glm::vec3 
	class PointCloud3D : public INearestNeighbor
	{
	public:
		PointCloud3D() = default;
		explicit PointCloud3D(std::vector<glm::vec3>&& points);
		// NOTE: arr must have size of 3*points_count
		explicit PointCloud3D(const float* arr, size_t points_count);
	public:
		size_t Count() const;
		glm::vec3 Centroid() const;
	public:
		glm::vec3& operator[](size_t i);
		const glm::vec3& operator[](size_t i) const;
	public:
		void Transform(const glm::mat3& rot, const glm::vec3& t);
		virtual glm::vec3 FindClosestPoint(const glm::vec3& sourcePoint) const override;
	public:
		auto begin() { return m_points.begin(); }
		auto end() { return m_points.end(); }
		auto begin() const { return m_points.cbegin(); }
		auto end() const { return m_points.cend(); }
		std::vector<glm::vec3>& GetStorage();
		const std::vector<glm::vec3>& GetStorage() const;
	private:
		void recalculateCentroid();
	private:
		std::vector<glm::vec3> m_points;
		glm::vec3 m_centroid{ 0.0f };
	};
}