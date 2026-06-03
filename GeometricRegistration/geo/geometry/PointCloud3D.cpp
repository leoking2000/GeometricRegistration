#include <cassert>
#include <numeric>
#include <geo/io/IOUtils.h>
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

	PointCloud3D PointCloud3D::UniformSubsample(index_t targetCount, u32 seed) const
	{
		assert(targetCount > 0);
		assert(targetCount <= Size());

		std::mt19937 rng(seed);

		std::vector<index_t> idx(Size());
		std::iota(idx.begin(), idx.end(), 0);

		// only shuffles the first targetCount elements
		for (index_t i = 0; i < targetCount; i++)
		{
			std::uniform_int_distribution<index_t> dist(i, Size() - 1);
			std::swap(idx[i], idx[dist(rng)]);
		}

		std::vector<glm::vec3> pts;
		pts.reserve(targetCount);
		for (index_t i = 0; i < targetCount; i++)
			pts.emplace_back(m_points[idx[i]]);

		return PointCloud3D(std::move(pts), {});
	}

	// Converts this point cloud into a generic dump structure for serialization.
	static io::GeometryDumpData ToDump(const PointCloud3D& pc)
	{
		io::GeometryDumpData data;

		data.geometryType = io::GeometryType::POINT_CLOUD;
		data.fileType = io::FileType::PLY;

		data.points = pc.GetPoints();
		data.normals = pc.HasNormals() ? pc.GetNormals() : std::vector<glm::vec3>{};

		// Point clouds do not use indices
		data.indexBuffer.clear();

		return data;
	}

	void PointCloud3D::Save(const std::filesystem::path& path) const
	{
		io::GeometryDumpData data = ToDump(*this);

		data.filePath = path;

		// Delegate actual writing to IO layer (format chosen by extension)
		io::SavePLY(path, data);
	}

	PointCloud3D PointCloud3D::Load(const std::filesystem::path & path)
	{
		io::GeometryDumpData data = io::LoadGeometry(path);

		std::vector<glm::vec3> points = std::move(data.points);
		std::vector<glm::vec3> normals;

		if (data.HasNormals())
		{
			normals = std::move(data.normals);
		}

		return PointCloud3D(std::move(points), std::move(normals));
	}
}

