#pragma once
#include "geo/math/RigidTransform.h"
#include <vector>

// TODO: 
// Build PointCloud from mesh
// PointCloud3DBuilder

namespace geo
{
	// A Point cloud of 3D points, uses glm::vec3 
	class PointCloud3D
	{
	public:
		explicit PointCloud3D(std::vector<glm::vec3>&& points);	
		explicit PointCloud3D(const float* arr, size_t float_count);
	public:
		size_t Size() const;
		bool Empty() const;
		glm::vec3 Centroid() const;
	public:
		const glm::vec3& operator[](size_t i) const;
	public:
		void Transform(const RigidTransform& transform);
		void Transform(const glm::mat3& rot, const glm::vec3& t);
	public:
		auto begin() const { return m_points.cbegin(); }
		auto end() const { return m_points.cend(); }
	public:
		const std::vector<glm::vec3>& GetStorage() const;
	private:
		void recalculateCentroid();
	private:
		std::vector<glm::vec3> m_points;
		glm::vec3 m_centroid{ 0.0f };
	};
}