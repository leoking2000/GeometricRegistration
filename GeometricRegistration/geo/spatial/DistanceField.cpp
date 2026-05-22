#include <cassert>
#include <fstream>
#include <geo/utils/logging/LogMacros.h>
#include <geo/io/IOUtils.h>
#include "DistanceField.h"

namespace geo
{
    bool closestPointToTriangle(glm::vec3& out_closestPoint, f32& out_distance,
        const glm::vec3& p, 
        const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
        const glm::vec3& faceNormal,
        f32 maxDist)
    {
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ap = p - a;

        // 1. Calculate distance from the triangle plane and Early Exit
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
        f32 sum = va + vb + vc;
        f32 denom = 1.0f / sum;
        f32 v = vb * denom;
        f32 w = vc * denom;
        glm::vec3 closest = a + ab * v + ac * w;

        out_closestPoint = closest;
        out_distance = glm::distance(p, closest);
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

        const glm::vec3& fn = mesh.Triangle(tri).faceNormal;

        return closestPointToTriangle(out_closestPoint, out_distance, p, a, b, c, fn, maxDist);
    }

    void DFCell::addTriangle(index_t tri)
    {
        glm::vec3 cp;
        f32 dist = F32_MAX;

        if (closestPointToTriangleByIndex(cp, dist, m_center, tri, *m_mesh, IsOccupied() ? m_distance : FLT_MAX))
        {
            if (m_distance > dist)
            {
                m_tri = tri;
                m_distance = dist;
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
        m_compact_cells.rehash(2048);
    }

    void DistanceField::Build(const Mesh& mesh)
    {
        m_cells.clear();
        m_compact_cells.clear();

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

            // 1.866 ~ worst-case distance from a voxel center to any point influencing a triangle
            // Derived from the maximal voxel-to-triangle feature coverage bound in a unit cube
            // half cell diagonal — conservative
            const f32 expand = m_cellSize * 1.866f;

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

        std::unordered_map<u64, DFCell, U64Hash> clean;
        for (auto& p : m_cells)
        {
            if (p.second.IsOccupied()) {
                clean.insert(p);
            }
        }
        m_cells = std::move(clean);

        Expand(mesh);
        computeSignAndCompact(mesh);
    }

    f32 DistanceField::operator()(const glm::vec3& q) const
    {
        glm::vec3 grid = (q - m_box.Min()) / m_cellSize;
        glm::ivec3 coord = glm::floor(grid);

        if (coord.x < 0 || coord.y < 0 || coord.z < 0) return m_max_dist;

        if (coord.x >= m_dims.x || coord.y >= m_dims.y || coord.z >= m_dims.z) return m_max_dist;

        u64 key = Hash(glm::uvec3(coord));
        auto it = m_compact_cells.find(key);

        if (it != m_compact_cells.end()) return it->second;

        return m_max_dist;
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
                i32 i_low = std::max(0, c.second.Coord().x - 1);
                i32 i_high = std::min(m_dims.x - 1, c.second.Coord().x + 1);

                i32 j_low = std::max(0, c.second.Coord().y - 1);
                i32 j_high = std::min(m_dims.y - 1, c.second.Coord().y + 1);

                i32 k_low = std::max(0, c.second.Coord().z - 1);
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

                                if (newCell.Distance() > m_max_dist)
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
                                
                                if (dist < iter->second.Distance() && dist < m_max_dist)
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

    void DistanceField::computeSignAndCompact(const Mesh& mesh)
    {
        for (auto& cell : m_cells)
        {
            glm::vec3 cp;
            f32 dist;
            closestPointToTriangleByIndex(cp, dist, cell.second.Center(), cell.second.ClosestTriangleIndex(), mesh, F32_MAX);

            const glm::vec3& faceNormal = mesh.Triangle(cell.second.ClosestTriangleIndex()).faceNormal;
            f32 sign = (glm::dot(faceNormal, cell.second.Center() - cp) >= 0.0f) ? 1.0f : -1.0f;

            m_compact_cells[cell.first] = dist * sign;
        }

        m_cells.clear();
    }

    // ============================================================
    // Binary serialization
    //
    // Format:
    //   [4]  magic "GSDF"
    //   [4]  version = 1
    //   [12] box min (3 × f32)
    //   [12] box max (3 × f32)
    //   [12] dims    (3 × i32)
    //   [4]  resolution (u32)
    //   [4]  cellSize   (f32)
    //   [4]  max_dist   (f32)
    //   [8]  cell count (u64)
    //   [N × 12] cells: u64 key + f32 distance
    // ============================================================

    static constexpr u32 SDF_MAGIC = 0x46445347; // "GSDF"
    static constexpr u32 SDF_VERSION = 1;

    bool DistanceField::Save(const std::filesystem::path& path) const
    {
        if (m_compact_cells.empty())
        {
            GEOLOGERROR("Save: SDF not built yet (no compact cells)");
            return false;
        }

        std::ofstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("Save: failed to open: " << path);
            return false;
        }


        // Header
        file.write(reinterpret_cast<const char*>(&SDF_MAGIC), 4);
        file.write(reinterpret_cast<const char*>(&SDF_VERSION), 4);

        glm::vec3 bmin = m_box.Min();
        glm::vec3 bmax = m_box.Max();
        file.write(reinterpret_cast<const char*>(&bmin), 12);
        file.write(reinterpret_cast<const char*>(&bmax), 12);
        file.write(reinterpret_cast<const char*>(&m_dims), 12);
        file.write(reinterpret_cast<const char*>(&m_resolution), 4);
        file.write(reinterpret_cast<const char*>(&m_cellSize), 4);
        file.write(reinterpret_cast<const char*>(&m_max_dist), 4);

        // Cells
        u64 count = m_compact_cells.size();
        file.write(reinterpret_cast<const char*>(&count), 8);

        // Pack all cells into one buffer
        const size_t bytesPerCell = sizeof(u64) + sizeof(f32);
        std::vector<u8> buf(count * bytesPerCell);
        u8* ptr = buf.data();

        for (const auto& [key, dist] : m_compact_cells)
        {
            std::memcpy(ptr, &key, 8); ptr += 8;
            std::memcpy(ptr, &dist, 4); ptr += 4;
        }

        file.write(reinterpret_cast<const char*>(buf.data()), (std::streamsize)buf.size());

        if (!file.good())
        {
            GEOLOGERROR("Save: write error for: " << path);
            return false;
        }

        return true;
    }

    bool DistanceField::Load(const std::filesystem::path& path, DistanceField& out)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            GEOLOGERROR("Load: failed to open: " << path);
            return false;
        }

        // Validate magic and version
        u32 magic = 0, version = 0;
        file.read(reinterpret_cast<char*>(&magic), 4);
        file.read(reinterpret_cast<char*>(&version), 4);

        if (magic != SDF_MAGIC)
        {
            GEOLOGERROR("Load: not a valid SDF file (bad magic)");
            return false;
        }
        if (version != SDF_VERSION)
        {
            GEOLOGERROR("Load: unsupported SDF version " << version);
            return false;
        }

        // Read grid parameters
        out = DistanceField();  // default construct

        glm::vec3 bmin, bmax;
        file.read(reinterpret_cast<char*>(&bmin), 12);
        file.read(reinterpret_cast<char*>(&bmax), 12);
        file.read(reinterpret_cast<char*>(&out.m_dims), 12);
        file.read(reinterpret_cast<char*>(&out.m_resolution), 4);
        file.read(reinterpret_cast<char*>(&out.m_cellSize), 4);
        file.read(reinterpret_cast<char*>(&out.m_max_dist), 4);

        out.m_box = BBox(bmin, bmax);

        // Read cells
        u64 count = 0;
        file.read(reinterpret_cast<char*>(&count), 8);

        if (!file.good())
        {
            GEOLOGERROR("Load: read error in header");
            return false;
        }

        const size_t bytesPerCell = sizeof(u64) + sizeof(f32);
        std::vector<u8> buf(count * bytesPerCell);
        file.read(reinterpret_cast<char*>(buf.data()), (std::streamsize)buf.size());

        if (!file.good())
        {
            GEOLOGERROR("Load: read error in cell data");
            return false;
        }

        out.m_compact_cells.rehash(2048);
        out.m_compact_cells.reserve(count);

        const u8* ptr = buf.data();
        for (u64 i = 0; i < count; i++)
        {
            u64 key;  f32 dist;
            std::memcpy(&key, ptr, 8); ptr += 8;
            std::memcpy(&dist, ptr, 4); ptr += 4;
            out.m_compact_cells.emplace(key, dist);
        }

        return true;
    }
}
