#include <cassert>
#include <limits>
#include <geo/utils/logging/LogMacros.h>
#include "Mesh.h"

namespace geo
{
	Mesh::Mesh(std::string filename, 
		std::vector<glm::vec3> points, std::vector<glm::uvec3> triangles, std::vector<glm::vec3> normals)
		:
		Mesh(filename, PointCloud3D(points, normals), triangles)
	{}

	Mesh::Mesh(std::string filename, PointCloud3D pointCloud, std::vector<glm::uvec3> triangles)
		:
		m_filename(filename),
		m_pointCloud(std::move(pointCloud))
	{
		// reserve triangle storage
		m_triangles.resize(triangles.size());

        const auto& points = m_pointCloud.GetPoints();

        #pragma omp parallel for schedule(static)
		for (int i = 0; i < (int)triangles.size(); i++)
		{
            glm::uvec3 tri = triangles[i];

			TriangleData data{};
			data.vertexIndices = tri;

			const glm::vec3& p0 = points[tri.x];
			const glm::vec3& p1 = points[tri.y];
			const glm::vec3& p2 = points[tri.z];

			const glm::vec3 e0 = p1 - p0;
			const glm::vec3 e1 = p2 - p0;

			glm::vec3 cross = glm::cross(e0, e1);

			const float crossLength = glm::length(cross);

			data.area = 0.5f * crossLength;

            f32 eps = std::numeric_limits<float>::epsilon();
			data.faceNormal = 
                glm::normalize((crossLength > eps) ? cross / crossLength : glm::vec3(1.0f, 0.0f, 0.0f));

            m_triangles[i] = data;
		}

		// generate normals if missing
		if (!m_pointCloud.HasNormals())
		{
            GEOLOGWARN("Mesh " << m_filename << " does not have vertex normals, we compute them");
			ComputeVertexNormals();
		}

		// computes bounding box and area
        m_bounding_box = m_pointCloud.ComputeBoundingBox();
        ComputeSurfaceArea();
	}
	
	Mesh::~Mesh()
	{
	}

	const glm::vec3& Mesh::Point(index_t i) const
	{
		return m_pointCloud.Point(i);
	}

	const glm::vec3& Mesh::Normal(index_t i) const
	{
		return m_pointCloud.Normal(i);
	}

	const TriangleData& Mesh::Triangle(index_t i) const
	{
		assert(i < m_triangles.size());
		return m_triangles[i];
	}

	const glm::vec3& Mesh::TriangleVertex(index_t tri, index_t corner) const
	{
		assert(tri < m_triangles.size());
		assert(corner < 3);
		return m_pointCloud.Point(m_triangles[tri][corner]);
	}

    PointCloud3D Mesh::SamplePointsUniform(u32 n, Random& rng, bool includeNormals) const
    {
        assert(!m_triangles.empty());
        assert(n > 0);

        const auto& positions = m_pointCloud.GetPoints();
        const auto& normals = m_pointCloud.GetNormals();

        // Prepare Output Buffers
        std::vector<glm::vec3> sampledPoints;
        std::vector<glm::vec3> sampledNormals;

        sampledPoints.reserve(n);

        if (includeNormals)
        {
            sampledNormals.reserve(n);
        }

        // Build cumulative distribution function (CDF) from triangle areas
        std::vector<f32> cdf;
        cdf.reserve(m_triangles.size());

        f32 totalArea = 0.0f;
        for (const TriangleData& tri : m_triangles)
        {
            totalArea += tri.area;
            cdf.emplace_back(totalArea);
        }

        if (totalArea <= std::numeric_limits<float>::epsilon())
        {
            return PointCloud3D({});
        }

        // Uniformly sample points on surface
        for (u32 i = 0; i < n; ++i)
        {
            // pick triangle proportional to area
            float r = rng.Float(0.0f, totalArea);

            auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
            index_t triIndex = static_cast<index_t>(std::distance(cdf.begin(), it));

            const TriangleData& tri = m_triangles[triIndex];

            const glm::vec3& p0 = positions[tri.vertexIndices.x];
            const glm::vec3& p1 = positions[tri.vertexIndices.y];
            const glm::vec3& p2 = positions[tri.vertexIndices.z];

            // Uniform barycentric sampling
            float u = rng.Float();
            float v = rng.Float();

            // reflection trick
            if (u + v > 1.0f)
            {
                u = 1.0f - u;
                v = 1.0f - v;
            }

            float w = 1.0f - u - v;

            glm::vec3 sample =
                w * p0 +
                u * p1 +
                v * p2;

            sampledPoints.emplace_back(sample);

            // Interpolate normals
            if (includeNormals)
            {
                glm::vec3 normal;

                if (!normals.empty())
                {
                    const glm::vec3& n0 = normals[tri.vertexIndices.x];
                    const glm::vec3& n1 = normals[tri.vertexIndices.y];
                    const glm::vec3& n2 = normals[tri.vertexIndices.z];

                    normal =
                        w * n0 +
                        u * n1 +
                        v * n2;

                    float len = glm::length(normal);

                    if (len > std::numeric_limits<float>::epsilon())
                    {
                        normal /= len;
                    }
                    else
                    {
                        normal = tri.faceNormal;
                    }
                }
                else
                {
                    normal = tri.faceNormal;
                }

                sampledNormals.emplace_back(normal);
            }
        }

        return PointCloud3D(std::move(sampledPoints), std::move(sampledNormals));
    }

    void Mesh::ComputeVertexNormals()
    {
        std::vector<glm::vec3>& normals = const_cast<std::vector<glm::vec3>&>(m_pointCloud.GetNormals());

        normals.clear();
        normals.resize(VertexCount(), glm::vec3(0.0f));

        // Accumulate face normals to vertices
        for (const TriangleData& tri : m_triangles)
        {
            const glm::vec3 weightedNormal = tri.faceNormal * tri.area;

            normals[tri[0]] += weightedNormal;
            normals[tri[1]] += weightedNormal;
            normals[tri[2]] += weightedNormal;
        }

        // Normalize all vertex normals
        for (glm::vec3& n : normals)
        {
            n = glm::normalize(n);
        }
    }

    void Mesh::ComputeSurfaceArea()
    {
        m_area = 0.0f;

        //#pragma omp parallel for reduction(+:m_area)
        for (size_t i = 0; i < m_triangles.size(); i++)
        {
            m_area += m_triangles[i].area;
        }
    }

}
