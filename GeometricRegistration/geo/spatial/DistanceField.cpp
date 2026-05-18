#include <cassert>
#include <geo/utils/logging/LogMacros.h>
#include "DistanceField.h"

namespace geo
{
    bool closestPointToTriangle(glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p, 
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        f32 maxDist)
    {
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ap = p - a;

        // 1. Calculate distance from the triangle plane and Early Exit
        glm::vec3 faceNormal = glm::normalize(glm::cross(ab, ac));
        f32 distToPlane = glm::dot(p - a, faceNormal);
        if (glm::abs(distToPlane) >= maxDist)
        {
            return false;
        }

        // 2. Vertex Region A
        f32 d1 = glm::dot(ab, ap);
        f32 d2 = glm::dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) 
        {
            f32 d = glm::distance(p, a);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = a;
            out_distance = d;
            return true;
        }

        // 3. Vertex Region B
        glm::vec3 bp = p - b;
        f32 d3 = glm::dot(ab, bp);
        f32 d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {

            float d = glm::distance(p, b);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = b;
            out_distance = d;
            return true;
        }

        // 4. Edge Region AB
        f32 vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            f32 v = d1 / (d1 - d3);
            glm::vec3 closest = a + v * ab;
            float d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 5. Vertex Region C
        glm::vec3 cp = p - c;
        f32 d5 = glm::dot(ab, cp);
        f32 d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            f32 d = glm::distance(p, c);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = c;
            out_distance = d;
            return true;
        }

        // 6. Edge Region AC
        f32 vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            f32 w = d2 / (d2 - d6);
            glm::vec3 closest = a + w * ac;
            f32 d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 7. Edge Region BC
        f32 va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            f32 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            glm::vec3 closest = b + w * (c - b);
            float d = glm::distance(p, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 8. Face Region
        f32 denom = 1.0f / (va + vb + vc);
        f32 v = vb * denom;
        f32 w = vc * denom;
        glm::vec3 closest = a + ab * v + ac * w;

        out_closestPoint = closest;
        out_distance = glm::abs(distToPlane);
        return true;
    }

    bool closestPointToTriangleByIndex(
        glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p,
        index_t tri, const Mesh& mesh,
        f32 maxDist)
    {
        const glm::vec3& a  = mesh.TriangleVertex(tri, 0);
        const glm::vec3& b  = mesh.TriangleVertex(tri, 1);
        const glm::vec3& c  = mesh.TriangleVertex(tri, 2);

        return closestPointToTriangle(out_closestPoint, out_distance, p, a, b, c, maxDist);
    }

    void DFCell::addTriangle(index_t tri)
    {
        glm::vec3 cp;
        f32 dist;

        if (closestPointToTriangleByIndex(cp, dist, m_center, tri, *m_mesh, IsOccupied() ? m_distance : FLT_MAX))
        {
            if (m_distance > glm::abs(dist))
            {
                m_tri = tri;
                m_distance = glm::abs(dist);

                const glm::vec3& a = m_mesh->TriangleVertex(tri, 0);
                const glm::vec3& b = m_mesh->TriangleVertex(tri, 1);
                const glm::vec3& c = m_mesh->TriangleVertex(tri, 2);

                glm::vec3 faceNormal = glm::normalize(glm::cross(a - b, a - c));
                m_sign = glm::dot(m_center - cp, faceNormal) >= 0.0f ? 1.0f : -1.0f;
            }      
        }
    }

    DistanceField::DistanceField(const DistanceFieldParameters& params)
    {
        // Store the max distance of the truncation
        m_max_dist = params.max_distance;
        // create the bounding box
        m_box = BBox(params.bounding_box.Min() - glm::vec3(m_max_dist), params.bounding_box.Max() + glm::vec3(m_max_dist));

        m_resolution = params.resolution;
        m_cellSize = m_box.MaxSize() / m_resolution;

        // expand bounding box
        glm::vec3 offset_low(0.0f);
        glm::vec3 offset_high(0.0f);
        for (u32 i = 0; i < 3; i++)
        {
            i32 dim = (i32)glm::ceil(m_box.Size()[i] / m_cellSize);
            f32 expansion = m_cellSize * dim - m_box.Size()[i];
            offset_low[i] -= expansion / 2;
            offset_high[i] += expansion / 2;
            m_dims[i] = dim;
        }
        m_box.Set(m_box.Min() + offset_low, m_box.Max() + offset_high);

        // rehash map
        m_cells.rehash(2048);
    }

    void DistanceField::Build(const Mesh& mesh)
    {
        // loop through every triagnle
        for (index_t t = 0; t < mesh.TriangleCount(); t++)
        {
            // get triagle veritices
            glm::vec3 tri[3];
            tri[0] = mesh.TriangleVertex(t, 0);
            tri[1] = mesh.TriangleVertex(t, 1);
            tri[2] = mesh.TriangleVertex(t, 2);

            BBox trb;
            trb.ExpandBy(tri[0]);
            trb.ExpandBy(tri[1]);
            trb.ExpandBy(tri[2]);

            const f32 expand = m_cellSize * 0.51f;

            i32 i_start = (i32)std::floor((trb.Min().x - expand - m_box.Min().x) / m_cellSize);
            i32 j_start = (i32)std::floor((trb.Min().y - expand - m_box.Min().y) / m_cellSize);
            i32 k_start = (i32)std::floor((trb.Min().z - expand - m_box.Min().z) / m_cellSize);

            i32 i_stop = (i32)std::floor((trb.Max().x + expand - m_box.Min().x) / m_cellSize);
            i32 j_stop = (i32)std::floor((trb.Max().y + expand - m_box.Min().y) / m_cellSize);
            i32 k_stop = (i32)std::floor((trb.Max().z + expand - m_box.Min().z) / m_cellSize);

            i_start = glm::clamp(i_start, 0, m_dims.x - 1); i_stop = glm::clamp(i_stop, 0, m_dims.x - 1);
            j_start = glm::clamp(j_start, 0, m_dims.y - 1); j_stop = glm::clamp(j_stop, 0, m_dims.y - 1);
            k_start = glm::clamp(k_start, 0, m_dims.z - 1); k_stop = glm::clamp(k_stop, 0, m_dims.z - 1);

            for (i32 k = k_start; k <= k_stop; k++)
            {
                for (i32 j = j_start; j <= j_stop; j++)
                {
                    for (i32 i = i_start; i <= i_stop; i++)
                    {
                        glm::uvec3 coords((u32)i, (u32)j, (u32)k);

                        u64 key = Hash(coords);
                        if (m_cells.find(key) == m_cells.end())
                        {
                            m_cells[key] = 
                                DFCell(m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f), coords, &mesh);
                        }
                        m_cells[key].addTriangle(t);
                    }
                }
            }
        }

        glm::vec3 q;
        std::unordered_map<u64, DFCell, U64Hash> clean;
        for (auto& p : m_cells)
        {
            if (p.second.IsOccupied()) {
                clean.insert(p);
            }
        }
        m_cells = std::move(clean);

        Expand(mesh);
        Compact();
    }

    f32 DistanceField::operator()(const glm::vec3& q) const
    {
        glm::vec3 a = (q - m_box.Min()) / m_cellSize;
        if (a.x < 0 || a.y < 0 || a.z < 0) return m_max_dist;

        glm::uvec3 coord(u32(a.x), u32(a.y), u32(a.z));
        if (coord.x >= u32(m_dims.x) || coord.y >= u32(m_dims.y) || coord.z >= u32(m_dims.z))
            return m_max_dist;

        u64 key = Hash(coord);
        auto cellp = m_compact_cells.find(key);
        if (cellp == m_compact_cells.end())
            return m_max_dist;

        return cellp->second;
    }

    void DistanceField::Expand(const Mesh& mesh)
    {
        std::unordered_map<u64, DFCell, U64Hash> boundary = m_cells; // TODO: just have a std::vector<u64> for the boundary???
        std::unordered_map<u64, DFCell, U64Hash> frontier;

        bool changed = false;

        do
        {
            for (auto& c : boundary)
            {
                i32 i_low  = std::max(0           , c.second.Coord().x - 1);
                i32 i_high = std::min(m_dims.x - 1, c.second.Coord().x + 1);

                i32 j_low  = std::max(0          , c.second.Coord().y - 1);
                i32 j_high = std::min(m_dims.y - 1, c.second.Coord().y + 1);

                i32 k_low  = std::max(0           , c.second.Coord().z - 1);
                i32 k_high = std::min(m_dims.z - 1, c.second.Coord().z + 1);

                for (i32 k = k_low; k <= k_high; k++)
                {
                    for (i32 j = j_low; j <= j_high; j++)
                    {
                        for (i32 i = i_low; i <= i_high; i++)
                        {
                            glm::uvec3 coords((u32)i, (u32)j, (u32)k);
                            u64 key = Hash(coords);

                            if (key == c.first)
                                continue;

                            if (m_cells.find(key) != m_cells.end())
                                continue;

                            glm::vec3 pos = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                            DFCell newCell = DFCell(pos, coords, &mesh);

                            auto iter = frontier.find(key);
                            if (iter == frontier.end()) // the cell does not exits 
                            {
                                newCell.addTriangle(c.second.ClosestTriangleIndex());

                                if (glm::abs(newCell.Distance()) > m_max_dist)
                                {
                                    continue;
                                }

                                frontier[key] = newCell;
                            }
                            else // the cell already exits
                            {
                                glm::vec3 cp;
                                f32 dist = FLT_MAX;

                                closestPointToTriangleByIndex(cp, dist, pos, c.second.ClosestTriangleIndex(), mesh, F32_MAX);

                                if (glm::abs(dist) < glm::abs(iter->second.Distance()) && glm::abs(dist) < m_max_dist)
                                {
                                    newCell.addTriangle(c.second.ClosestTriangleIndex());
                                    frontier[key] = newCell;
                                }
                            }
                        }
                    }
                }
            }

            for (auto& p : frontier)
            {
                m_cells[p.first] = p.second;
            }

            changed = frontier.size() > 0;
            boundary = std::move(frontier);

            frontier.clear();
        } while (changed);
    }

    void DistanceField::Compact()
    {
        for (auto& cell : m_cells)
        {
            f32 dist;
            dist = cell.second.Distance();
            m_compact_cells[cell.first] = dist;
        }

        m_cells.clear();
    }
}
