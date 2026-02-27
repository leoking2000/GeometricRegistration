#include "PointCloud3D.h"

namespace geo
{
	PointCloud3D::PointCloud3D(std::vector<glm::vec3>&& points)
		:
		m_points(std::move(points))
	{
		recalculateCentroid();
	}

	PointCloud3D::PointCloud3D(const float* arr, size_t points_count)
	{
		m_points.reserve(points_count);

		for (size_t i = 0; i < 3 * points_count; i += 3)
		{
			float x = arr[i];
			float y = arr[i + 1];
			float z = arr[i + 2];

			m_points.emplace_back(x, y, z);
		}

		recalculateCentroid();
	}


	glm::vec3 PointCloud3D::Centroid() const
	{
		return m_centroid;
	}

	glm::vec3& PointCloud3D::operator[](size_t i)
	{
		return m_points[i];
	}

	const glm::vec3& PointCloud3D::operator[](size_t i) const
	{
		return m_points[i];
	}

	size_t PointCloud3D::Count() const
	{
		return (int)m_points.size();
	}

	void PointCloud3D::Transform(const glm::mat3& rot, const glm::vec3& t)
	{
		for (auto& point : m_points)
		{
			point = rot * point + t;
		}
		m_centroid = rot * m_centroid + t;
	}

	glm::vec3 PointCloud3D::FindClosestPoint(const glm::vec3& sourcePoint) const
	{
		float closestDist = std::numeric_limits<float>::max();
		glm::vec3 closestPoint(0.0f);

		for (auto& p : m_points)
		{
			float dist = glm::distance(sourcePoint, p);
			if (dist < closestDist)
			{
				closestDist = dist;
				closestPoint = p;
			}
		}

		return closestPoint;
	}

	void PointCloud3D::recalculateCentroid()
	{
		glm::vec3 c(0.0f);
		for (const auto& p : m_points)
		{
			c += p;
		}

		if (!m_points.empty()) {
			m_centroid = c / float(m_points.size());
		}
		else {
			m_centroid = glm::vec3(0.0f);
		}
	}
}


