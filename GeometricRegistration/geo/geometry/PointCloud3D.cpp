#include <cassert>
#include "PointCloud3D.h"

namespace geo
{
	PointCloud3D::PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals)
		:
		m_points(std::move(points)),
		m_normals(std::move(normals))
	{
		assert(!m_points.empty());
		assert(m_points.size() == m_normals.size() || m_normals.empty());

		recalculateCentroid();
	}

	index_t PointCloud3D::Size() const
	{
		return (index_t)m_points.size();
	}

	bool PointCloud3D::HasNormals() const
	{
		return m_points.size() == m_normals.size();
	}

	const glm::vec3& PointCloud3D::Point(index_t i) const
	{
		assert(i < m_points.size());
		return m_points[i];
	}

	const glm::vec3& PointCloud3D::Normal(index_t i) const
	{
		assert(m_points.size() == m_normals.size());
		assert(i < m_normals.size());

		return m_normals[i];
	}

	glm::vec3 PointCloud3D::Centroid() const
	{
		return m_centroid;
	}

	void PointCloud3D::Transform(const RigidTransform& transform)
	{
		if (HasNormals())
		{
			for (size_t i = 0; i < m_points.size(); i++)
			{
				m_points[i] = transform.TransformPoint(m_points[i]);
				m_normals[i] = glm::normalize(transform.TransformNormal(m_normals[i]));
			}
		}
		else
		{
			for (size_t i = 0; i < m_points.size(); i++)
			{
				m_points[i] = transform.TransformPoint(m_points[i]);
			}
		}

		m_centroid = transform.TransformPoint(m_centroid);
	}

	void PointCloud3D::recalculateCentroid()
	{
		glm::vec3 c(0.0f);
		for (const auto& p : m_points)
		{
			c += p;
		}
		m_centroid = c / f32(m_points.size());
	}
}

