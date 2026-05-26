#include <cassert>
#include "PointCloud3D.h"

namespace geo
{
	PointCloud3D::PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals)
		:
		m_points(std::move(points)),
		m_normals(std::move(normals))
	{
		RecalculateCentroid();
	}

	index_t PointCloud3D::Size() const
	{
		return (index_t)m_points.size();
	}

	bool PointCloud3D::HasNormals() const
	{
		return !m_points.empty() && m_points.size() == m_normals.size();
	}

	const glm::vec3& PointCloud3D::Point(index_t i) const
	{
		assert(i < m_points.size());
		return m_points[i];
	}

	const glm::vec3& PointCloud3D::Normal(index_t i) const
	{
		assert(HasNormals());
		assert(i < m_normals.size());

		return m_normals[i];
	}

	const glm::vec3& PointCloud3D::Centroid() const
	{
		return m_centroid;
	}

	BBox PointCloud3D::ComputeBoundingBox() const
	{
		// Return default (empty) bounding box if no points exist
		if (m_points.empty()) {
			return BBox();
		}

		// Initialize min/max with first point
		glm::vec3 minP = m_points[0];
		glm::vec3 maxP = m_points[0];

		// Expand bounds to include all points
		for (size_t i = 1; i < m_points.size(); ++i)
		{
			const glm::vec3& p = m_points[i];

			minP = glm::min(minP, p);
			maxP = glm::max(maxP, p);
		}

		return BBox(minP, maxP);
	}

	void PointCloud3D::Transform(const RigidTransform& transform)
	{
		if (HasNormals())
		{
			// Transform both points and normals
			for (size_t i = 0; i < m_points.size(); i++)
			{
				m_points[i] = transform.TransformPoint(m_points[i]);
				m_normals[i] = glm::normalize(transform.TransformNormal(m_normals[i]));
			}
		}
		else
		{
			// Transform only points
			for (size_t i = 0; i < m_points.size(); i++)
			{
				m_points[i] = transform.TransformPoint(m_points[i]);
			}
		}

		// Update cached centroid using same transform
		m_centroid = transform.TransformPoint(m_centroid);
	}

	void PointCloud3D::RecalculateCentroid()
	{
		glm::dvec3 c(0.0);

		// Handle empty point cloud
		if (m_points.empty()) {
			m_centroid = c;
			return;
		}

		// Accumulate all point positions
		for (const auto& p : m_points)
		{
			c += p;
		}

		// Compute average (centroid)
		m_centroid = c / f64(m_points.size());
	}

	PointCloud3D PointCloud3D::UniformSubsample(f32 voxelSize) const
	{
		assert(voxelSize > 0.0f);

		struct Accumulator
		{
			glm::vec3 position = glm::vec3(0.0f);
			glm::vec3 normal = glm::vec3(0.0f);
			u32 count = 0;
		};

		struct IVec3Hash
		{
			size_t operator()(const glm::ivec3& v) const noexcept
			{
				size_t h1 = std::hash<i32>()(v.x);
				size_t h2 = std::hash<i32>()(v.y);
				size_t h3 = std::hash<i32>()(v.z);

				return h1 ^ (h2 << 1) ^ (h3 << 2);
			}
		};

		std::unordered_map<glm::ivec3, Accumulator, IVec3Hash> voxels;
		voxels.reserve(Size());

		const bool hasNormals = HasNormals();

		// Accumulate points into voxel cells
		for (u32 i = 0; i < Size(); i++)
		{
			const glm::vec3& p = m_points[i];

			glm::ivec3 coord(
				(i32)std::floor(p.x / voxelSize),
				(i32)std::floor(p.y / voxelSize),
				(i32)std::floor(p.z / voxelSize)
			);

			auto& voxel = voxels[coord];

			voxel.position += p;

			if (hasNormals)
			{
				voxel.normal += m_normals[i];
			}

			voxel.count++;
		}

		// Build subsampled cloud
		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;

		points.reserve(voxels.size());

		if (hasNormals)
		{
			normals.reserve(voxels.size());
		}

		for (const auto& [coord, voxel] : voxels)
		{
			glm::vec3 avgPoint = voxel.position / (f32)voxel.count;
			points.push_back(avgPoint);

			if (hasNormals)
			{
				glm::vec3 avgNormal = glm::normalize(voxel.normal);
				normals.push_back(avgNormal);
			}
		}

		return PointCloud3D(std::move(points), std::move(normals));
	}
}

