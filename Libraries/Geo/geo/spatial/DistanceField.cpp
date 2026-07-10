#include <cassert>
#include <fstream>
#include <core/logging/Log.h>
#include <core/utils/Time.h>
#include <core/io/IOUtils.h>
#include "DistanceField.h"


#ifdef DF_PARALLEL_BUILD
#pragma warning( disable : 6993 )
#include <omp.h>
#endif

namespace geo
{
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

        return core::closestPointToTriangle(out_closestPoint, out_distance, p, a, b, c, fn, maxDist);
    }

    void DFCell::SetResult(index_t tri, f32 dist)
    {
        if (dist < m_distance)
        {
            m_tri = tri;
            m_distance = dist;
        }
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

    void DistanceField::PreBuild(const DistanceFieldParameters& params)
    {
        // Store the max distance of the truncation
        m_max_dist = params.max_distance;

        // create the bounding box
        m_box = core::BBox(params.bounding_box.Min() - glm::vec3(m_max_dist),
                     params.bounding_box.Max() + glm::vec3(m_max_dist));

        if (params.cell_size > 0.0f)
        {
            m_cellSize = params.cell_size;
            m_resolution = (u32)glm::ceil(m_box.MaxSize() / m_cellSize);
        }
        else
        {
            m_resolution = params.resolution;
            m_cellSize = m_box.MaxSize() / m_resolution;
        }

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

        LOGDEBUG("DistanceField Build Params:"
                 << "\ndimensions   ="   << m_dims.x << " x " << m_dims.y << " x " << m_dims.z
                 << "\nresolution   ="   << m_resolution
                 << "\ncell_size    ="   << m_cellSize
                 << "\nmax_distance ="   << m_max_dist
                 << "\nBBox size    ="   << m_box.Size().x << "," << m_box.Size().y << "," << m_box.Size().z
        );
    }

    void DistanceField::Build(const DistanceFieldParameters& params, const Mesh& mesh)
    {
        core::TimePoint start = core::Clock::now();
        LOGINFO("DistanceField: Build started");

        LOGDEBUG("DistanceField: Mesh triangles: " << mesh.TriangleCount());

        PreBuild(params);

        m_cells.clear();
        m_compact_cells.clear();

#ifdef DF_PARALLEL_BUILD
        LOGINFO("DistanceField: using PARALLEL build");

        core::TimePoint TriangleStart = core::Clock::now();

        // Each thread owns its own cell map — no sharing, no locks
        const int numThreads = omp_get_max_threads();
        std::vector<std::unordered_map<u64, DFCell, U64Hash>> threadMaps(numThreads);
        for (auto& m : threadMaps) {
            m.rehash(2048);
        }

        const f32 expand = m_cellSize * 1.866f; // half cell diagonal — conservative

        // paralize over every triagnle
        #pragma omp parallel for schedule(static)
        for (int t = 0; t < (int)mesh.TriangleCount(); t++)
        {
            const int tid = omp_get_thread_num();
            auto& localMap = threadMaps[tid];

            const glm::vec3& v0 = mesh.TriangleVertex(t, 0);
            const glm::vec3& v1 = mesh.TriangleVertex(t, 1);
            const glm::vec3& v2 = mesh.TriangleVertex(t, 2);

            core::BBox trb;
            trb.ExpandBy(v0);
            trb.ExpandBy(v1);
            trb.ExpandBy(v2);

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

                        auto it = localMap.find(key);
                        if (it == localMap.end())
                        {
                            glm::vec3 center = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);
                            it = localMap.emplace(key, DFCell(center, coords, &mesh)).first;
                        }
                        it->second.addTriangle(t);
                    }
                }
            }
        }

        // Serial merge — take minimum distance across all thread maps
        for (auto& localMap : threadMaps)
        {
            for (auto& [key, cell] : localMap)
            {
                auto it = m_cells.find(key);
                if (it == m_cells.end())
                {
                    m_cells.emplace(key, std::move(cell));
                }
                else if (std::fabs(cell.Distance()) < std::fabs(it->second.Distance()))
                {
                    it->second = std::move(cell);
                }
            }
        }

        core::TimePoint TriangleEnd = core::Clock::now();
        LOGVERBOSE("DistanceField: Time of building norrow band: " 
            << core::TimeDifferenceMs(TriangleEnd, TriangleStart) << "ms");

        ExpandSeiral(mesh);
        //ExpandParaller(mesh);
#else
        LOGINFO("DistanceField: using SERIAL build");
        core::TimePoint TriangleStart = core::Clock::now();

        // loop through every triagnle
        for (index_t t = 0; t < mesh.TriangleCount(); t++)
        {
            // get triagle veritices
            glm::vec3 tri[3];
            tri[0] = mesh.TriangleVertex(t, 0);
            tri[1] = mesh.TriangleVertex(t, 1);
            tri[2] = mesh.TriangleVertex(t, 2);

            core::BBox trb;
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

        core::TimePoint TriangleEnd = core::Clock::now();
        LOGVERBOSE("DistanceField: Time of building norrow band: " << 
            core::TimeDifferenceMs(TriangleEnd, TriangleStart) << "ms");

        ExpandSeiral(mesh);
#endif

        computeSignAndCompact(mesh);

        LOGINFO("DistanceField: Total Build Time: " << core::TimeDifferenceMs(core::Clock::now(), start) << "ms");
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

    void DistanceField::ExpandSeiral(const Mesh& mesh)
    {
        LOGVERBOSE("DistanceField: expansion started");
        core::TimingStat expandTime;

        core::TimePoint start = core::Clock::now();

        std::vector<u64> boundary;
        std::unordered_map<u64, DFCell, U64Hash> frontier;

        boundary.reserve(m_cells.size());
        for (auto& p : m_cells) {
            boundary.push_back(p.first);  
        }

        bool changed = false;

        do
        {
            core::TimePoint start_loop = core::Clock::now();

            for (u64 bkey : boundary)
            {
                const DFCell& bc        = m_cells.at(bkey);
                const glm::ivec3& coord = bc.Coord();
                const index_t     btri  = bc.ClosestTriangleIndex();

                i32 i_low  = std::max(0           , coord.x - 1);
                i32 i_high = std::min(m_dims.x - 1, coord.x + 1);

                i32 j_low  = std::max(0           , coord.y - 1);
                i32 j_high = std::min(m_dims.y - 1, coord.y + 1);

                i32 k_low  = std::max(0           , coord.z - 1);
                i32 k_high = std::min(m_dims.z - 1, coord.z + 1);

                for (i32 k = k_low; k <= k_high; k++)
                {
                    for (i32 j = j_low; j <= j_high; j++)
                    {
                        for (i32 i = i_low; i <= i_high; i++)
                        {
                            glm::uvec3 coords((u32)i, (u32)j, (u32)k);
                            u64 key = Hash(coords);

                            // we skip ourselfs
                            if (key == bkey) {
                                continue;
                            }

                            // we skip already processed cells
                            if (m_cells.find(key) != m_cells.end()) {
                                continue;
                            }

                            glm::vec3 pos = m_box.Min() + m_cellSize * glm::vec3(i + 0.5f, j + 0.5f, k + 0.5f);

                            glm::vec3 cp;
                            f32 dist = FLT_MAX;
                            if (!closestPointToTriangleByIndex(cp, dist, pos, btri, mesh, m_max_dist)) {
                                continue;
                            }

                            auto iter = frontier.find(key);
                            if (iter == frontier.end()) // the cell does not exits 
                            {
                                DFCell newCell(pos, coords, &mesh);
                                newCell.SetResult(btri, dist);
                                frontier.emplace(key, std::move(newCell));
                            }
                            else // the cell already exits
                            {
                                iter->second.SetResult(btri, dist);
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

            boundary.clear();
            for (auto& p : frontier) {
                boundary.push_back(p.first);
            }

            frontier.clear();

            expandTime.AddSample(core::TimeDifferenceMs(core::Clock::now(), start_loop));

        } while (changed);

        core::TimePoint end = core::Clock::now();
        LOGVERBOSE("DistanceField: Total Time of Expand(): " 
            << core::TimeDifferenceMs(end, start) << "ms\n" << expandTime.ToString());
    }

    //void DistanceField::ExpandParaller(const Mesh& mesh)
    //{
    //
    //}

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

        LOGDEBUG("DistanceField: compact SDF size = " << m_compact_cells.size());
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
        LOGINFO("Saving DistanceField: " << path);

        LOGDEBUG("Cell count = " << m_compact_cells.size());

        if (m_compact_cells.empty())
        {
            LOGERROR("Save: SDF not built yet (no compact cells)");
            return false;
        }

        std::ofstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            LOGERROR("Save: failed to open: " << path);
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
            LOGERROR("Save: write error for: " << path);
            return false;
        }

        return true;
    }

    bool DistanceField::Load(const std::filesystem::path& path, DistanceField& out)
    {
        LOGINFO("Loading DistanceField: " << path);

        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            LOGERROR("Load: failed to open: " << path);
            return false;
        }

        // Validate magic and version
        u32 magic = 0, version = 0;
        file.read(reinterpret_cast<char*>(&magic), 4);
        file.read(reinterpret_cast<char*>(&version), 4);

        if (magic != SDF_MAGIC)
        {
            LOGERROR("Load: not a valid SDF file (bad magic)");
            return false;
        }
        if (version != SDF_VERSION)
        {
            LOGERROR("Load: unsupported SDF version " << version);
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

        out.m_box = core::BBox(bmin, bmax);

        LOGDEBUG("SDF dims=" << out.m_dims.x << "," << out.m_dims.y << "," << out.m_dims.z);
        LOGDEBUG("CellSize=" << out.m_cellSize << " maxDist=" << out.m_max_dist);

        // Read cells
        u64 count = 0;
        file.read(reinterpret_cast<char*>(&count), 8);

        if (!file.good())
        {
            LOGERROR("Load: read error in header");
            return false;
        }

        const size_t bytesPerCell = sizeof(u64) + sizeof(f32);
        std::vector<u8> buf(count * bytesPerCell);
        file.read(reinterpret_cast<char*>(buf.data()), (std::streamsize)buf.size());

        if (!file.good())
        {
            LOGERROR("Load: read error in cell data");
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

        LOGINFO("SDF loaded: cells=" << count);

        return true;
    }
}
