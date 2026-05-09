#include <omp.h>
#include <cassert>
#include <limits>
#include <geo/utils/logging/LogMacros.h>
#include "Mesh.h"

namespace geo
{
    Mesh::Mesh(const std::string& filename, std::vector<glm::vec3> points, std::vector<glm::uvec3> triangles, std::vector<glm::vec3> normals)
        :
        m_filename(filename),
        m_vertices(std::move(points)),
        m_normals(std::move(normals))
    {
        ComputeTriangleData(triangles);

        // generate normals if missing
        if (m_normals.size() != m_vertices.size())
        {
            GEOLOGWARN("Mesh " << m_filename << " does not contain vertex normals. Computing normals.");
            ComputeVertexNormals();
        }

        ComputeSurfaceArea();
        ComputeBoundingBox();
    }

    Mesh::~Mesh()
    {
    }
    
    void Mesh::ComputeTriangleData(const std::vector<glm::uvec3>& indexBuffer)
    {   
        constexpr f32 eps = std::numeric_limits<float>::epsilon();

        // clear and reserve triangle storage
        m_triangles.clear();
        m_triangles.resize(indexBuffer.size());

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)m_triangles.size(); i++)
        {
            const glm::uvec3 tri = indexBuffer[i];

            TriangleData data{};
            data.vertexIndices = tri;

            const glm::vec3& p1 = m_vertices[tri.x];
            const glm::vec3& p2 = m_vertices[tri.y];
            const glm::vec3& p3 = m_vertices[tri.z];

            glm::vec3 cross = glm::cross(p2 - p1, p3 - p1);
            const float crossLength = glm::length(cross);
            // in case the points do not form a triangle
            data.faceNormal = (crossLength > eps) ? cross / crossLength : glm::vec3(1.0f, 0.0f, 0.0f);

            data.area = 0.5f * crossLength;

            m_triangles[i] = data;
        }
    }

    // runs if no normals were given
    void Mesh::ComputeVertexNormals()
    {   
        constexpr f32 eps = std::numeric_limits<float>::epsilon();

        m_normals.clear();
        m_normals.resize(m_vertices.size(), glm::vec3(0.0f));
    
        for (size_t i = 0; i < m_triangles.size(); i++)
        {
            const TriangleData& tri = m_triangles[i];

            m_normals[tri[0]] += tri.faceNormal;
            m_normals[tri[1]] += tri.faceNormal;
            m_normals[tri[2]] += tri.faceNormal;
        }

        // Normalize
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)m_normals.size(); ++i)
        {
            const f32 len = glm::length(m_normals[i]);

            if (len > eps)
            {
                m_normals[i] /= len;
            }
            else
            {
                // fallback normal
                m_normals[i] = glm::vec3(1.0f, 0.0f, 0.0f);
            }
        }
    }

    void Mesh::ComputeBoundingBox()
    {
        m_bounding_box = BBox();

        for (const glm::vec3& v : m_vertices)
        {
            m_bounding_box.ExpandBy(v);
        }
    }

    void Mesh::ComputeSurfaceArea()
    {
        m_area = 0.0f;
        f64 totalArea = 0.0;

        #pragma omp parallel for reduction(+:totalArea)
        for (int i = 0; i < (int)m_triangles.size(); i++)
        {
            totalArea += m_triangles[i].area;
        }

        m_area = totalArea;
    }

    void Mesh::Flatten()
    {
        // TODO: parallelize this?

        std::vector<glm::vec3> flatVertices;
        std::vector<glm::vec3> flatNormals;
        std::vector<TriangleData> flatTriangles;

        flatVertices.reserve(m_triangles.size() * 3);
        flatNormals.reserve(m_triangles.size() * 3);
        flatTriangles.reserve(m_triangles.size());

        for (index_t i = 0; i < (index_t)m_triangles.size(); i++)
        {
            const TriangleData& tri = m_triangles[i];
            TriangleData newTri{};

            for (index_t c = 0; c < 3; c++)
            {
                const index_t oldIndex = tri.vertexIndices[c];
                const index_t newIndex = (index_t)flatVertices.size();

                flatVertices.emplace_back(m_vertices[oldIndex]);
                flatNormals.emplace_back(m_normals[oldIndex]);

                newTri.vertexIndices[c] = newIndex;
            }

            newTri.faceNormal = tri.faceNormal;
            newTri.area = tri.area;

            flatTriangles.emplace_back(newTri);
        }

        m_vertices = std::move(flatVertices);
        m_normals = std::move(flatNormals);
        m_triangles = std::move(flatTriangles);
    }

    PointCloud3D Mesh::SamplePointsUniform(u32 n, Random& rng, bool includeNormals) const
    {
        assert(!m_triangles.empty());
        assert(n > 0);
    
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
    
        // Uniformly sample points on surface
        for (u32 i = 0; i < n; ++i)
        {
            // pick triangle proportional to area
            float r = rng.Float(0.0f, totalArea);
    
            auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
            index_t triIndex = static_cast<index_t>(std::distance(cdf.begin(), it));
    
            const TriangleData& tri = m_triangles[triIndex];
    
            const glm::vec3& p0 = m_vertices[tri[0]];
            const glm::vec3& p1 = m_vertices[tri[1]];
            const glm::vec3& p2 = m_vertices[tri[2]];
    
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
    
                const glm::vec3& n0 = m_normals[tri[0]];
                const glm::vec3& n1 = m_normals[tri[1]];
                const glm::vec3& n2 = m_normals[tri[2]];
    
                normal = w * n0 + u * n1 + v * n2;
    
                float len = glm::length(normal);
    
                if (len > std::numeric_limits<float>::epsilon())
                {
                    normal /= len;
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
}
