#include <cassert>
#include "PointCloud3D.h"

namespace geo
{
	PointCloud3D::PointCloud3D(std::vector<glm::vec3>&& points)
		:
		m_points(std::move(points))
	{
		recalculateCentroid();
	}

	PointCloud3D::PointCloud3D(const float* arr, size_t float_count)
	{
		assert(arr != nullptr);
		assert(float_count % 3 == 0);

		m_points.reserve(float_count / 3);
		for (size_t i = 0; i < float_count; i += 3)
		{
			m_points.emplace_back(arr[i], arr[i + 1], arr[i + 2]);
		}

		recalculateCentroid();
	}


	glm::vec3 PointCloud3D::Centroid() const
	{
		return m_centroid;
	}

	const glm::vec3& PointCloud3D::operator[](size_t i) const
	{
		assert(i < m_points.size());
		return m_points[i];
	}

	size_t PointCloud3D::Size() const
	{
		return m_points.size();
	}

	bool PointCloud3D::Empty() const
	{
		return m_points.size() == 0;
	}

	void PointCloud3D::Transform(const RigidTransform& transform)
	{
		Transform(transform.rotation, transform.translation);
	}

	void PointCloud3D::Transform(const glm::mat3& rot, const glm::vec3& t)
	{
		for (auto& point : m_points)
		{
			point = rot * point + t;
		}
		m_centroid = rot * m_centroid + t;
	}

	const std::vector<glm::vec3>& PointCloud3D::GetStorage() const
	{
		return m_points;
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


