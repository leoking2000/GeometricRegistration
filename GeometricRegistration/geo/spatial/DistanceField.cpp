#include <cassert>
#include <geo/utils/logging/LogMacros.h>
#include "DistanceField.h"

namespace geo
{
    bool closestPointToTriangle(glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& query, 
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, 
        const glm::vec3& an, const glm::vec3& bn, const glm::vec3& cn,
        const glm::vec3& faceNormal,
        f32 maxDist)
    {
        // 1. Calculate distance from the triangle plane and Early Exit
        float distToPlane = glm::dot(query - a, faceNormal);
        if (std::fabs(distToPlane) >= maxDist)
        {
            return false;
        }

        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ap = query - a;

        // 2. Vertex Region A
        float d1 = glm::dot(ab, ap);
        float d2 = glm::dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) 
        {
            float d = glm::distance(query, a);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = a;
            out_distance = d;
            return true;
        }

        // 3. Vertex Region B
        glm::vec3 bp = query - b;
        float d3 = glm::dot(ab, bp);
        float d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {

            float d = glm::distance(query, b);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = b;
            out_distance = d;
            return true;
        }

        // 4. Edge Region AB
        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            float v = d1 / (d1 - d3);
            glm::vec3 closest = a + v * ab;
            float d = glm::distance(query, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 5. Vertex Region C
        glm::vec3 cp = query - c;
        float d5 = glm::dot(ab, cp);
        float d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            float d = glm::distance(query, c);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = c;
            out_distance = d;
            return true;
        }

        // 6. Edge Region AC
        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            float w = d2 / (d2 - d6);
            glm::vec3 closest = a + w * ac;
            float d = glm::distance(query, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 7. Edge Region BC
        float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            glm::vec3 closest = b + w * (c - b);
            float d = glm::distance(query, closest);

            if (d >= maxDist) {
                return false;
            }

            out_closestPoint = closest;
            out_distance = d;
            return true;
        }

        // 8. Face Region
        float denom = 1.0f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        glm::vec3 closest = a + ab * v + ac * w;

        out_closestPoint = closest;
        out_distance = std::fabs(distToPlane);
        return true;
    }

    bool closestPointToTriangleByIndex(
        glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& query,
        index_t tri, const Mesh& mesh,
        f32 maxDist)
    {
        if (tri >= mesh.TriangleCount()) {
            GEOLOGERROR("triagnle index is invaild!!!");
            assert(false && "invalid triangle index");
            return false;
        }

        const glm::vec3& a  = mesh.TriangleVertex(tri, 0);
        const glm::vec3& b  = mesh.TriangleVertex(tri, 1);
        const glm::vec3& c  = mesh.TriangleVertex(tri, 2);
        const glm::vec3& an = mesh.TriangleVertexNormal(tri, 0);
        const glm::vec3& bn = mesh.TriangleVertexNormal(tri, 1);
        const glm::vec3& cn = mesh.TriangleVertexNormal(tri, 2);

        const glm::vec3& fn = mesh.Triangle(tri).faceNormal;

        return closestPointToTriangle(out_closestPoint, out_distance, query, a, b, c, an, bn, cn, fn, maxDist);
    }

    f32 DFCell::operator()(const glm::vec3& q) const
    {
        glm::vec3 cp;
        f32 dist;

        if (closestPointToTriangleByIndex(cp, dist, q, m_tri, *m_mesh, F32_MAX))
        {
            return dist * m_sing;
        }

        return m_distance * m_sing; // this should not hit becouse max_dist=F32_MAX
    }

    void DFCell::addTriangle(index_t tri)
    {
        glm::vec3 cp;
        f32 dist;

        const glm::vec3& fn = m_mesh->Triangle(tri).faceNormal;

        if (closestPointToTriangleByIndex(cp, dist, m_center, tri, *m_mesh, IsOccupied() ? m_distance : FLT_MAX))
        {
            if (m_distance > dist)
            {
                m_tri = tri;
                m_distance = dist;
                m_sing = glm::dot(fn, m_center - cp) >= 0.0f ? 1.0f : -1.0f;
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
            u32 dim = std::ceilf(m_box.Size()[i] / m_cellSize);
            f32 expansion = m_cellSize * dim - m_box.Size()[i];
            offset_low[i] -= expansion / 2;
            offset_high[i] += expansion / 2;
            m_dims[i] = (i32)dim;
        }
        m_box.Set(m_box.Min() + offset_low, m_box.Max() + offset_high);

        // rehash map
        m_cells.rehash(2048);
    }

    void DistanceField::Build(const Mesh& mesh, bool compact)
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


            const f32 expand = m_cellSize * 0.866f; // half diagonal of a cube face, conservative

            i32 i_start = (i32)std::floorf((trb.Min().x - expand - m_box.Min().x) / m_cellSize);
            i32 j_start = (i32)std::floorf((trb.Min().y - expand - m_box.Min().y) / m_cellSize);
            i32 k_start = (i32)std::floorf((trb.Min().z - expand - m_box.Min().z) / m_cellSize);

            i32 i_stop = (i32)std::floorf((trb.Max().x + expand - m_box.Min().x) / m_cellSize);
            i32 j_stop = (i32)std::floorf((trb.Max().y + expand - m_box.Min().y) / m_cellSize);
            i32 k_stop = (i32)std::floorf((trb.Max().z + expand - m_box.Min().z) / m_cellSize);

            i_start = glm::clamp(i_start, 0, m_dims.x - 1); i_stop = glm::clamp(i_stop, 0, m_dims.x - 1);
            j_start = glm::clamp(j_start, 0, m_dims.y - 1); j_stop = glm::clamp(j_stop, 0, m_dims.y - 1);
            k_start = glm::clamp(k_start, 0, m_dims.z - 1); k_stop = glm::clamp(k_stop, 0, m_dims.z - 1);

            for (i32 k = k_start; k <= k_stop; k++)
            {
                for (i32 j = j_start; j <= j_stop; j++)
                {
                    for (i32 i = i_start; i <= i_stop; i++)
                    {
                        glm::uvec3 coords(i, j, k);

                        u64 key = Hash(coords);
                        if (m_cells.find(key) == m_cells.end())
                        {
                            glm::vec3 center = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                            m_cells[key] = DFCell(center, coords, &mesh);
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

        if (IS_LEVEL_ACTIVE(LogLevel::LOG_DEBUG)) {
            Check();
        }

        if (compact) {
            Compact();
        }
    }

    f32 DistanceField::operator()(const glm::vec3& q) const
    {
        glm::vec3 a = (q - m_box.Min()) / m_cellSize;
        if (a.x < 0 || a.y < 0 || a.z < 0) return m_max_dist;

        glm::uvec3 coord(u32(a.x), u32(a.y), u32(a.z));
        if (coord.x >= u32(m_dims.x) || coord.y >= u32(m_dims.y) || coord.z >= u32(m_dims.z))
            return m_max_dist;

        u64 key = Hash(coord);

        if (isCompact) 
        {
            auto cellp = m_compact_cells.find(key);
            if (cellp == m_compact_cells.end())
                return m_max_dist;

            return cellp->second;
        }
        else 
        {
            auto cellp = m_cells.find(key);
            if (cellp == m_cells.end())
                return m_max_dist;

            return cellp->second(q);
        }
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
                            u64 key = Hash({ i, j, k });

                            if (key == c.first)
                                continue;
                            if (m_cells.find(key) != m_cells.end())
                                continue;

                            auto iter = frontier.find(key);
                            if (iter == frontier.end()) // the cell does not exits 
                            {
                                glm::vec3 pos = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                                DFCell newCell = DFCell(pos, { i, j, k }, &mesh);

                                newCell.addTriangle(c.second.ClosestTriangleIndex());

                                if (std::fabs(newCell.Distance()) > m_max_dist)
                                    continue;

                                frontier[key] = newCell;
                            }
                            else // the cell already exits
                            {
                                glm::vec3 pos = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);

                                glm::vec3 cp;
                                f32 dist = F32_MAX;

                                closestPointToTriangleByIndex(cp, dist, pos, c.second.ClosestTriangleIndex(), mesh, F32_MAX);

                                if (dist < std::fabs(iter->second.Distance()) && dist < m_max_dist)
                                {
                                    DFCell newCell = DFCell(pos, { i, j, k }, &mesh);
                                    newCell.addTriangle(c.second.ClosestTriangleIndex());
                                    frontier[key] = newCell;
                                }
                            }
                        }
                    }
                }
            }

            m_cells.insert(frontier.begin(), frontier.end());
            changed = frontier.size() > 0;
            boundary = std::move(frontier);

            frontier.clear();
        } while (changed);
    }

    void DistanceField::Check()
    {
        u32 error_counter = 0;

        for (auto& cell : m_cells)
        {
            f32 cell_dist = std::fabs(cell.second.Distance());

            i32 i_low = std::max(0, cell.second.Coord().x - 1);
            i32 i_high = std::min(m_dims.x - 1, cell.second.Coord().x + 1);
            i32 j_low = std::max(0, cell.second.Coord().y - 1);
            i32 j_high = std::min(m_dims.y - 1, cell.second.Coord().y + 1);
            i32 k_low = std::max(0, cell.second.Coord().z - 1);
            i32 k_high = std::min(m_dims.z - 1, cell.second.Coord().z + 1);

            for (i32 k = k_low; k <= k_high; k++) 
            {
                for (i32 j = j_low; j <= j_high; j++) 
                {
                    for (i32 i = i_low; i <= i_high; i++)
                    {
                        u64 key = Hash({ u32(i), u32(j), u32(k) });

                        if (key == cell.first)
                            continue;

                        auto neighbor = m_cells.find(key);
                        if (neighbor != m_cells.end())
                        {
                            f32 neighbor_dist = std::fabs(neighbor->second.Distance());
                            f32 diff = std::fabs(cell_dist - neighbor_dist);

                            // Lipschitz condition: |d(a) - d(b)| <= dist(a, b)
                            glm::vec3 neighborCenter = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                            f32 centerDist = glm::distance(cell.second.Center(), neighborCenter);

                            if (diff > centerDist + 1e-4f)
                            {
                                error_counter++;
                                GEOLOGDEBUG("Lipschitz violation between ("
                                    << cell.second.Coord().x << "," << cell.second.Coord().y << "," << cell.second.Coord().z
                                    << ") and (" << i << "," << j << "," << k << ")"
                                    << " diff=" << diff << " maxAllowed=" << centerDist);
                            }
                        }
                        else if (cell_dist < m_max_dist - m_cellSize)
                        {
                            // This cell is inside the narrow band but its neighbor is missing
                            error_counter++;
                            GEOLOGDEBUG("Missing neighbor at (" << i << "," << j << "," << k << ")");
                        }
                    }
                }
            }
        }

        GEOLOGDEBUG("Found " << error_counter << " problems out of " << m_cells.size() << " cells");
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
        isCompact = true;
    }
}
