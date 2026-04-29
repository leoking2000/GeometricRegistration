#pragma once
#include <vector>
#include <utils/GeoTypes.h>
#include <math/RigidTransform.h>
#include <math/BBox.h>

namespace geo
{
	// A Point cloud of 3D points, uses glm::vec3 
	class PointCloud3D
	{
	public:
		explicit PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals = {});
	public:
		index_t Size() const;
		bool HasNormals() const;
	public:
		const glm::vec3& Point(index_t i) const;
		const glm::vec3& Normal(index_t i) const;
		glm::vec3 Centroid() const;
	public:
		BBox ComputeBoundingBox() const;
	public:
		// Note: Transform mutates point positions, and invalidates any spatial index built on this data.
		void Transform(const RigidTransform& transform);
	public:
		auto begin() const { return m_points.cbegin(); }
		auto end() const { return m_points.cend(); }
	public:
		const std::vector<glm::vec3>& GetPoints() const { return m_points; };
		const std::vector<glm::vec3>& GetNormals() const { return m_normals; };
	private:
		void recalculateCentroid();
	private:
		std::vector<glm::vec3> m_points;
		std::vector<glm::vec3> m_normals;
		glm::vec3 m_centroid{ 0.0f };
	};
}
