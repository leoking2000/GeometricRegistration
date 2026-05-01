#include <omp.h>
#include <cassert>
#include "KDTree.h"
#include "DistanceField.h"


namespace geo
{
    GridDescriptor ComputeGridDescriptor(const BBox& bbox, u32 resolution, f32 padding)
    {
        GridDescriptor data;

        data.bbox = bbox;
        data.bbox.ExpandByFactor(padding);

        glm::vec3 size = data.bbox.Size();
        f32 maxDim = std::max(size.x, std::max(size.y, size.z));

        data.voxelSize = maxDim / resolution;
        data.gridSize = glm::uvec3(
            (u32)std::ceil(size.x / data.voxelSize),
            (u32)std::ceil(size.y / data.voxelSize),
            (u32)std::ceil(size.z / data.voxelSize)
        );

        return data;
    }

    SparseVoxelGrid::SparseVoxelGrid(const GridDescriptor& descriptor, f32 defaultValue)
        :
        m_descriptor(descriptor),
        m_defaultValue(defaultValue)
    {
        m_data.rehash(2048);
    }

    f32 SparseVoxelGrid::Get(const glm::uvec3& coord) const
    {
        if (!IsInside(coord)) {
            return m_defaultValue;
        }

        auto res = m_data.find(ToKey(coord));
        if (res == m_data.end()) {
            return m_defaultValue;
        }

        return res->second;
    }

    void SparseVoxelGrid::Set(const glm::uvec3& coord, f32 value)
    {
        if (IsInside(coord)) {
            m_data.insert_or_assign(ToKey(coord), value);
        }
    }

    bool SparseVoxelGrid::IsInside(const glm::uvec3& coord) const
    {
        return coord.x < m_descriptor.gridSize.x && coord.y < m_descriptor.gridSize.y && coord.z < m_descriptor.gridSize.z;
    }

    void DistanceField::Build(const PointCloud3D& cloud, u32 resolution, f32 dTrunc, f32 padding)
    {
        // 1. Compute grid
        const BBox bbox = cloud.ComputeBoundingBox();
        GridDescriptor descriptor = ComputeGridDescriptor(bbox, resolution, padding);

        m_grid = SparseVoxelGrid(descriptor, dTrunc);

        // 2. get points and create the KdTree
        const auto& points  = cloud.GetPoints();
        //const auto& normals = cloud.GetNormals();
        const KDTree nn = KDTree(points);

        const glm::uvec3 size = descriptor.gridSize;
        const uint32_t total = size.x * size.y * size.z;

        glm::vec3 origin = descriptor.bbox.Min();
        f32 voxelSize = descriptor.voxelSize;

        // 3. For each voxel compute distance

        // 3.1 Thread-local buffers
        const int numThreads = omp_get_max_threads();
        std::vector<std::vector<std::pair<glm::uvec3, float>>> buffers(numThreads);

        // 3.2 reserve to reduce reallocations (rough heuristic)
        for (auto& b : buffers)
            b.reserve(1024);

        // 3.3 Parallel compute (no locks)
        #pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& local = buffers[tid];

            #pragma omp for schedule(static)
            for (i32 i = 0; i < (i32)total; ++i)
            {
                // --- index to 3D coords (fast integer math) ---
                u32 x = i % size.x;
                u32 y = (i / size.x) % size.y;
                u32 z = i / (size.x * size.y);

                glm::uvec3 coord(x, y, z);

                // --- world position ---
                glm::vec3 world = origin + (glm::vec3(x, y, z) + 0.5f) * voxelSize;

                // --- nearest neighbor ---
                index_t idx = nn.Query(world);
                f32 dist = glm::distance(world, points[idx]);

                // --- truncate ---
                if (dist < dTrunc)
                {
                    local.emplace_back(coord, dist);
                }
            }
        }

        // 4. Merge (single-threaded, cache-friendly)
        for (const auto& local : buffers)
        {
            for (const auto& [coord, dist] : local)
            {
                m_grid.Set(coord, dist);
            }
        }
    }

    f32 DistanceField::operator()(const glm::vec3& q) const
    {
        const glm::vec3 local = (q - m_grid.GetBoundingBox().Min()) / m_grid.GetDescriptor().voxelSize;
        glm::ivec3 icoord = glm::floor(local);

        if (icoord.x < 0 || icoord.y < 0 || icoord.z < 0)
        {
            return m_grid.GetDefaultValue(); // dTrunc
        }

        return m_grid.Get(glm::uvec3(icoord));
    }


}
