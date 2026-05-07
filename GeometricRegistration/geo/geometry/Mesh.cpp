#include "Mesh.h"
#include <algorithm>
#include <glm/geometric.hpp>

namespace geo
{
    Mesh::Mesh(const std::string& filename,
        std::vector<glm::vec3> vertex_buffer,
        std::vector<glm::ivec3> index_buffer,
        std::vector<glm::vec3> normal_buffer,
        std::vector<glm::vec2> coords_buffer
    )
        : 
        m_filename(filename),
        m_vertices_buffer(std::move(vertex_buffer)),
        m_index_buffer(std::move(index_buffer)),
        m_normals_buffer(std::move(normal_buffer)),
        m_coords_buffer(std::move(coords_buffer))
    {
        // --- Compute Bounding Box ---
        ComputeBoundingBox();

        // --- Optional: compute normals if missing ---
        if (m_normals_buffer.empty() && !m_index_buffer.empty())
        {
            m_normals_buffer.resize(m_vertices_buffer.size(), glm::vec3(0.0f));

            for (const auto& tri : m_index_buffer)
            {
                const glm::vec3& v0 = m_vertices_buffer[tri.x];
                const glm::vec3& v1 = m_vertices_buffer[tri.y];
                const glm::vec3& v2 = m_vertices_buffer[tri.z];

                glm::vec3 n = glm::normalize(glm::cross(v1 - v0, v2 - v0));

                m_normals_buffer[tri.x] = n;
                m_normals_buffer[tri.y] = n;
                m_normals_buffer[tri.z] = n;
            }

            for (auto& n : m_normals_buffer)
            {
                n = glm::normalize(n);
            }
        }
    }

    Mesh::~Mesh() = default;

    void Mesh::flatten()
    {
        // Convert indexed mesh to triangle soup (no shared vertices)
        if (m_index_buffer.empty())
            return;

        std::vector<glm::vec3> new_vertices;
        std::vector<glm::vec3> new_normals;
        std::vector<glm::vec2> new_coords;

        new_vertices.reserve(m_index_buffer.size() * 3);
        if (!m_normals_buffer.empty())
            new_normals.reserve(m_index_buffer.size() * 3);
        if (!m_coords_buffer.empty())
            new_coords.reserve(m_index_buffer.size() * 3);

        for (const auto& tri : m_index_buffer)
        {
            const int ids[3] = { tri.x, tri.y, tri.z };

            for (int i = 0; i < 3; ++i)
            {
                int idx = ids[i];

                new_vertices.push_back(m_vertices_buffer[idx]);

                if (!m_normals_buffer.empty())
                    new_normals.push_back(m_normals_buffer[idx]);

                if (!m_coords_buffer.empty())
                    new_coords.push_back(m_coords_buffer[idx]);
            }
        }

        // Replace buffers
        m_vertices_buffer = std::move(new_vertices);
        m_normals_buffer = std::move(new_normals);
        m_coords_buffer = std::move(new_coords);

        // Clear indices (now implicit triangles)
        m_index_buffer.clear();

        // Recompute bounding box (important!)
        ComputeBoundingBox();
    }

    void Mesh::ComputeBoundingBox()
    {
        if (!m_vertices_buffer.empty())
        {
            glm::vec3 min = m_vertices_buffer[0];
            glm::vec3 max = m_vertices_buffer[0];

            for (const auto& v : m_vertices_buffer)
            {
                min = glm::min(min, v);
                max = glm::max(max, v);
            }

            m_boundingBox = BBox(min, max);
        }
        else
        {
            m_boundingBox.MakeEmpty();
        }
    }

}
