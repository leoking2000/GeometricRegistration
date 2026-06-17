#include <cassert>
#include <numeric>
#include <random>
#include <geo/io/IOUtils.h>
#include <geo/utils/logging/LogMacros.h>
#include "PointCloud3D.h"

namespace geo
{
	PointCloud3D::PointCloud3D(std::vector<glm::vec3> points, std::vector<glm::vec3> normals)
		:
		m_points(std::move(points)),
		m_normals(std::move(normals))
	{
		RecalculateCentroid();

		GEOLOGINFO("PointCloud3D created | points=" << m_points.size()
			     << " normals=" << m_normals.size()
			     << " hasNormals=" << (HasNormals() ? "yes" : "no"));
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

		GEOLOGDEBUG("UniformSubsample | input=" << Size() << " output=" << targetCount << " seed=" << seed);

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

	void PointCloud3D::Save(const std::filesystem::path& path) const
	{
		GEOLOGINFO("Saving point cloud: " << path 
			<< " | points=" << m_points.size() << " | normals=" << m_normals.size());

		io::GeometryDumpData dump;
		dump.filePath = path;
		dump.fileType = io::GetFileType(path);
		dump.geometryType = io::GeometryType::POINT_CLOUD;

		dump.positions = m_points;
		dump.normals   = m_normals;

		io::SaveGeometry(path, dump);

		GEOLOGINFO("Point cloud saved successfully");
	}

	struct Key
	{
		u32 vi, ni;
		bool operator==(const Key& o) const { return vi == o.vi && ni == o.ni; }
	};

	struct KeyHash
	{
		size_t operator()(const Key& k) const
		{
			size_t h = 2166136261u;
			h ^= k.vi; h *= 16777619u;
			h ^= k.ni; h *= 16777619u;
			return h;
		}
	};

	PointCloud3D PointCloud3D::Load(const std::filesystem::path& path)
	{
		GEOLOGINFO("Loading point cloud: " << path);

		io::GeometryDumpData dump = io::LoadGeometry(path);

		if (dump.positions.empty())
		{
			GEOLOGWARN("PointCloud3D::Load — no points in file: " << path);
			return PointCloud3D{};
		}

		// ------------------------------------------------------------------
		// True point cloud (no index buffer) — arrays are already aligned.
		// Every slot i is consistent across points and normals by construction.
		// ------------------------------------------------------------------
		if (!dump.HasIndices())
		{
			return PointCloud3D(std::move(dump.positions), std::move(dump.normals));
		}

		const glm::vec3 defaultNormal(0.0f, 1.0f, 0.0f);

		std::unordered_set<Key, KeyHash> seen;
		seen.reserve(dump.indexBuffer.size() * 3);

		std::vector<glm::vec3> points;
		std::vector<glm::vec3> normals;
		points.reserve(dump.indexBuffer.size() * 3);
		normals.reserve(dump.indexBuffer.size() * 3);

		for (const io::TriangleIndex& tri : dump.indexBuffer)
		{
			for (int v = 0; v < 3; v++)
			{
				const u32 vi = tri.vertexIndex[v];
				const u32 ni = tri.normalIndex[v];

				// insert() returns false in .second if key was already present
				if (!seen.insert(Key{ vi, ni }).second) continue;

				points.emplace_back(vi < dump.positions.size() ? dump.positions[vi] : glm::vec3(0.0f));
				normals.emplace_back(ni < dump.normals.size() ? dump.normals[ni] : defaultNormal);
			}
		}

		return PointCloud3D(std::move(points), std::move(normals));
	}
}

