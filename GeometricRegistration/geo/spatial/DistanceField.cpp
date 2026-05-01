#include <cassert>
#include "DistanceField.h"


namespace geo
{
    GridDescriptor ComputeGridDescriptor(const BBox& bbox, f32 resolution, f32 padding)
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
}
