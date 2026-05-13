#include <omp.h>
#include <cassert>
#include <geo/utils/logging/LogMacros.h>
#include "KDTree.h"
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
        if (std::abs(distToPlane) >= maxDist)
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
        out_distance = std::abs(distToPlane);
        return true;
    }

    f32 DFCell::operator()(const glm::vec3& q) const
    {
        if (m_tri >= m_mesh->TriangleCount()) {
            GEOLOGERROR("m_tri triagnle index is invaild!!!");
            return F32_MAX;
        }

        glm::vec3 cp;
        f32 dist;

        const glm::vec3& a = m_mesh->TriangleVertex(m_tri, 0);
        const glm::vec3& b = m_mesh->TriangleVertex(m_tri, 1);
        const glm::vec3& c = m_mesh->TriangleVertex(m_tri, 2);
        const glm::vec3& an = m_mesh->TriangleVertexNormal(m_tri, 0);
        const glm::vec3& bn = m_mesh->TriangleVertexNormal(m_tri, 1);
        const glm::vec3& cn = m_mesh->TriangleVertexNormal(m_tri, 2);

        const glm::vec3& fn = m_mesh->Triangle(m_tri).faceNormal;

        if (closestPointToTriangle(cp, dist, q, a, b, c, an, bn, cn, fn, F32_MAX))
        {
            return dist * m_sign;
        }

        return F32_MAX;
    }

    void DFCell::addTriangle(index_t tri)
    {
        if (m_tri >= m_mesh->TriangleCount()) {
            GEOLOGERROR("triagnle index is invaild!!!");
            return;
        }

        glm::vec3 cp;
        f32 dist;

        const glm::vec3& a = m_mesh->TriangleVertex(tri, 0);
        const glm::vec3& b = m_mesh->TriangleVertex(tri, 1);
        const glm::vec3& c = m_mesh->TriangleVertex(tri, 2);
        const glm::vec3& an = m_mesh->TriangleVertexNormal(tri, 0);
        const glm::vec3& bn = m_mesh->TriangleVertexNormal(tri, 1);
        const glm::vec3& cn = m_mesh->TriangleVertexNormal(tri, 2);

        const glm::vec3& fn = m_mesh->Triangle(tri).faceNormal;

        if (closestPointToTriangle(cp, dist, m_center, a, b, c, an, bn, cn, fn, m_occupied ? m_distance : FLT_MAX))
        {
            f32 abs_dist = std::fabs(dist);
            if (m_distance > abs_dist)
            {
                m_tri = tri;
                m_distance = abs_dist;
                m_occupied = true;
                m_sign = glm::dot(fn, m_center - cp) >= 0.0f ? 1.0f : -1.0f;
            }      
        }
    }
    



    //==========================================================================================================================

    GridDescriptor ComputeGridDescriptor(const BBox& bbox, u32 resolution, f32 padding)
    {
        GridDescriptor data;

        // 1. Expand bbox
        data.bbox = bbox;
        data.bbox.ExpandByFactor(padding);

        // 2. Compute size
        glm::vec3 size = data.bbox.Size();
        f32 maxDim = std::max(size.x, std::max(size.y, size.z));

        // 3. Voxel size (uniform)
        data.voxelSize = maxDim / resolution;

        // 4. Grid resolution
        data.gridSize = glm::uvec3(
            (u32)std::ceil(size.x / data.voxelSize),
            (u32)std::ceil(size.y / data.voxelSize),
            (u32)std::ceil(size.z / data.voxelSize)
        );

        // 5. Snap bbox to grid
        glm::vec3 snappedSize = glm::vec3(data.gridSize) * data.voxelSize;
        data.bbox = BBox(data.bbox.Min(), data.bbox.Min() + snappedSize);

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

    void NaiveDistanceField::Build(const PointCloud3D& cloud, u32 resolution, f32 dTrunc, f32 padding)
    {
        // 1. Compute grid
        const BBox bbox = cloud.ComputeBoundingBox();
        GridDescriptor descriptor = ComputeGridDescriptor(bbox, resolution, padding);

        m_grid = SparseVoxelGrid(descriptor, dTrunc);

        // 2. get points and create the KdTree
        const auto& points = cloud.GetPoints();
        const auto& normals = cloud.GetNormals();
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
//#pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& local = buffers[tid];

//#pragma omp for schedule(static)
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
                glm::vec3 p = points[idx];
                glm::vec3 n = normals[idx];

                //f32 dist = std::abs(glm::dot(world - p, n)); // point to plane dist
                f32 dist = glm::distance(world, p);

                // --- truncate ---
                if (dist < dTrunc)
                {
                    local.emplace_back(coord, dist);
                }
            }
        }

        // 4. Merge
        for (const auto& local : buffers)
        {
            for (const auto& [coord, dist] : local)
            {
                m_grid.Set(coord, dist);
            }
        }
    }

    f32 NaiveDistanceField::operator()(const glm::vec3& q) const
    {
        const glm::vec3 local = (q - m_grid.GetBoundingBox().Min()) / m_grid.GetDescriptor().voxelSize - 0.5f;
        glm::ivec3 icoord = glm::floor(local);

        if (icoord.x < 0 || icoord.y < 0 || icoord.z < 0)
        {
            return m_grid.GetDefaultValue(); // dTrunc
        }

        return m_grid.Get(glm::uvec3(icoord));
    }

}
