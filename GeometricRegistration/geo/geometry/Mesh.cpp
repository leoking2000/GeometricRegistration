#include <omp.h>
#include <cassert>
#include <limits>
#include <geo/utils/logging/LogMacros.h>
#include <geo/io/IOUtils.h>
#include "Mesh.h"

#pragma warning( disable : 6993)

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

            data.faceNormal = glm::normalize(cross);
            data.area = 0.5f * glm::length(cross);

            m_triangles[i] = data;
        }
    }

    // runs if no normals were given
    void Mesh::ComputeVertexNormals()
    {   
        m_normals.clear();
        m_normals.resize(m_vertices.size(), glm::vec3(0.0f));
    
        for (size_t i = 0; i < m_triangles.size(); i++)
        {
            const TriangleData& tri = m_triangles[i];

            m_normals[tri[0]] += tri.faceNormal * tri.area;
            m_normals[tri[1]] += tri.faceNormal * tri.area;
            m_normals[tri[2]] += tri.faceNormal * tri.area;
        }

        // Normalize
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)m_normals.size(); ++i)
        {
            m_normals[i] = glm::normalize(m_normals[i]);
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
        m_area = 0.0;
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

    static io::GeometryDumpData ToDump(const Mesh& mesh)
    {
        io::GeometryDumpData dump;
        dump.fileType = io::FileType::UNKNOWN; // set by caller via path extension
        dump.geometryType = io::GeometryType::TRIANGLE_MESH;

        dump.positions = mesh.GetVertices();
        dump.normals = mesh.GetNormals();

        const auto& tris = mesh.GetTriangles();
        dump.indexBuffer.reserve(tris.size());

        for (const TriangleData& tri : tris)
        {
            io::TriangleIndex idx;
            idx.vertexIndex = tri.vertexIndices;
            idx.normalIndex = tri.vertexIndices; // normals are per-vertex and aligned
            idx.coordsIndex = glm::uvec3(0u);    // no texcoords on Mesh
            idx.colorIndex = glm::uvec3(0u);
            dump.indexBuffer.push_back(idx);
        }

        // bbox — copy the already-computed one rather than recomputing
        dump.bbox = mesh.BoundingBox();

        return dump;
    }

    void Mesh::Save(const std::filesystem::path& path) const
    {
        if (m_vertices.empty())
        {
            GEOLOGWARN("Mesh::Save — mesh is empty, nothing written to " << path);
            return;
        }

        io::GeometryDumpData dump = ToDump(*this);
        dump.filePath = path;
        dump.fileType = io::GetFileType(path);

        io::SaveGeometry(path, dump);

        GEOLOGINFO("Mesh::Save — wrote " << VertexCount() << " vertices, " << TriangleCount() << " triangles to " << path);
    }

    struct Key
    {
        u32 vi, ni;
        bool operator<(const Key& o) const
        {
            if (vi != o.vi) return vi < o.vi;
            return ni < o.ni;
        }
    };

    Mesh Mesh::Load(const std::filesystem::path& path)
    {
        io::GeometryDumpData dump = io::LoadGeometry(path);

        if (dump.positions.empty())
        {
            GEOLOGERROR("Mesh::Load - no vertices in file: " << path);
            return Mesh{};
        }

        if (!dump.HasIndices())
        {
            GEOLOGERROR("Mesh::Load - file contains no face data (point cloud?): " << path);
            return Mesh{};
        }

        std::map<Key, u32>      cache;
        std::vector<glm::vec3>  points;
        std::vector<glm::vec3>  normals;
        std::vector<glm::uvec3> indexBuffer;

        points.reserve(dump.positions.size());
        normals.reserve(dump.positions.size());
        indexBuffer.reserve(dump.indexBuffer.size());

        for (const io::TriangleIndex& tri : dump.indexBuffer)
        {
            glm::uvec3 face;

            for (int v = 0; v < 3; v++)
            {
                const u32 vi = tri.vertexIndex[v];
                const u32 ni = tri.normalIndex[v];
                const Key key{ vi, ni };

                auto [it, inserted] = cache.emplace(key, static_cast<u32>(points.size()));

                if (inserted)
                {
                    points.push_back(vi < dump.positions.size() ? dump.positions[vi] : glm::vec3(0.0f));

                    normals.push_back(ni < dump.normals.size() ? dump.normals[ni] : glm::vec3(0.0f, 1.0f, 0.0f));
                }

                face[v] = it->second;
            }

            indexBuffer.push_back(face);
        }

        return Mesh(
            io::GetFileName(path),
            std::move(points),
            std::move(indexBuffer),
            std::move(normals)
        );
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
            if (it == cdf.end()) --it;  // clamp to last triangle
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
