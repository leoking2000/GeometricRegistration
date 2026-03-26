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

	PointCloud3D::PointCloud3D(const f32* arr, index_t float_count)
	{
		assert(arr != nullptr);
		assert(float_count % 3 == 0);

		m_points.reserve(float_count / 3);
		for (index_t i = 0; i < float_count; i += 3)
		{
			m_points.emplace_back(arr[i], arr[i + 1], arr[i + 2]);
		}

		recalculateCentroid();
	}


	glm::vec3 PointCloud3D::Centroid() const
	{
		return m_centroid;
	}

	const glm::vec3& PointCloud3D::operator[](index_t i) const
	{
		assert(i < m_points.size());
		return m_points[i];
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

	index_t PointCloud3D::Size() const
	{
		return (index_t)m_points.size();
	}

	bool PointCloud3D::Empty() const
	{
		return m_points.empty();
	}

	void PointCloud3D::Transform(const RigidTransform& transform)
	{
		Transform(transform.rotation, transform.translation);
	}

	void PointCloud3D::Transform(const glm::mat3& rot, const glm::vec3& t)
	{
		if (HasNormals())
		{
			for (index_t i = 0; i < m_points.size(); i++)
			{
				m_points[i] = rot * m_points[i] + t;
				m_normals[i] = glm::normalize(rot * m_normals[i]);
			}
		}
		else
		{
			for (auto& point : m_points)
			{
				point = rot * point + t;
			}
		}

		m_centroid = rot * m_centroid + t;
	}

	const std::vector<glm::vec3>& PointCloud3D::GetStorage() const
	{
		return m_points;
	}

	const std::vector<glm::vec3>& PointCloud3D::GetNormals() const
	{
		return m_normals;
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

	PointCloud3D GenerateRandomPointCloudRect(const glm::vec3& center, f32 width, f32 height, f32 depth, u32 pointCount, 
		Random& rng, bool haveNormals)
	{
		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;

		points.reserve(pointCount);
		normals.reserve(pointCount);

		f32 hx = width * 0.5f;
		f32 hy = height * 0.5f;
		f32 hz = depth * 0.5f;

		for (u32 i = 0; i < pointCount; ++i)
		{
			u32 face = rng.UInt(0, 5);

			glm::vec3 p;
			glm::vec3 n;

			switch (face)
			{
			case 0: // +X
				p.x = center.x + hx;
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(1, 0, 0);
				break;

			case 1: // -X
				p.x = center.x - hx;
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(-1, 0, 0);
				break;

			case 2: // +Y
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + hy;
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(0, 1, 0);
				break;

			case 3: // -Y
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y - hy;
				p.z = center.z + rng.Float(-hz, hz);
				n = glm::vec3(0, -1, 0);
				break;

			case 4: // +Z
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z + hz;
				n = glm::vec3(0, 0, 1);
				break;

			case 5: // -Z
				p.x = center.x + rng.Float(-hx, hx);
				p.y = center.y + rng.Float(-hy, hy);
				p.z = center.z - hz;
				n = glm::vec3(0, 0, -1);
				break;
			}

			points.push_back(p);
			normals.push_back(n);
		}

		PointCloud3D cloud(std::move(points), haveNormals ? std::move(normals) : std::vector<glm::vec3>());

		return cloud;
	}
}


