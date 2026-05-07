#pragma once
#include <vector>
#include <utils/GeoTypes.h>
#include <math/RigidTransform.h>
#include <math/BBox.h>

namespace geo
{
	// Represents a 3D point cloud composed of positions (and optionally normals).
	// Internally uses glm::vec3 for both points and normals.
	class PointCloud3D final
	{
	public:
		// Constructor
		// @param points   Vector of 3D positions (required)
		// @param normals  Optional vector of normals (must match points size if provided)
		explicit PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals = {});
	public:
		// @return Number of points in the cloud
		index_t Size() const;
		// @return True if normals are present and valid
		bool HasNormals() const;
	public:
		// Access a point by index (no bounds checking)
		// @param i Index of the point
		// @return Const reference to the point
		const glm::vec3& Point(index_t i) const;

		// Access a normal by index (assumes normals exist and no bounds checking)
		// @param i Index of the normal
		// @return Const reference to the normal
		const glm::vec3& Normal(index_t i) const;

		// Computes or returns the cached centroid of the point cloud
		// @return Geometric center of all points
		const glm::vec3& Centroid() const;
	public:
		// Computes the axis-aligned bounding box (AABB) of the point cloud
		// @return Bounding box enclosing all points
		BBox ComputeBoundingBox() const;
	public:
		// Applies a rigid transformation (rotation + translation) to all points.
		// Note:
		// - This operation mutates the internal point data.
		// - Any previously built spatial acceleration structures (e.g., KD-trees) become invalid.
		void Transform(const RigidTransform& transform);
	public:
		// Iterator support for range-based for loops (read-only)
		auto begin() const { return m_points.cbegin(); }
		auto end() const { return m_points.cend(); }
	public:
		// Direct access to underlying point container (read-only)
		const std::vector<glm::vec3>& GetPoints() const { return m_points; };
		// Direct access to underlying normal container (read-only)
		const std::vector<glm::vec3>& GetNormals() const { return m_normals; };
	private:
		// Recomputes the centroid based on current point data
		// Should be called after any modification to m_points
		void recalculateCentroid();
	private:
		std::vector<glm::vec3> m_points;  // Storage for 3D points
		std::vector<glm::vec3> m_normals; // Storage for normals (optional, may be empty)
		glm::vec3 m_centroid{ 0.0f };     // Cached centroid of the point cloud 
	};
}
